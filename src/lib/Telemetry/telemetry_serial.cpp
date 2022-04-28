#include "telemetry_serial.h"

#ifdef SERIAL_TELEMETRY

#include "telemetry_protocol.h"
#include "crsf_protocol.h"
#include "CRSF.h"
#include "msp.h"
#include "logging.h"
#include <stdio.h>

extern MSP msp;
extern CRSF crsf;

#define MSP_WP              118
#define MSP_SET_WP          209
#define MSP_WP_GETINFO      20
#define NAV_MAX_WAYPOINTS   120

extern Stream *TxBackpack;
#define DefaultTelemetrySerial TxBackpack
#define TELEMETRY_SERIAL_BAUD 115200

Stream *TelemetrySerial::serialPort;
FIFO TelemetrySerial::OutFIFO;
#ifdef PLATFORM_ESP32
    portMUX_TYPE TelemetrySerial::FIFOmux = portMUX_INITIALIZER_UNLOCKED;
#endif
uint8_t TelemetrySerial::outBuffer[CRSF_MAX_PACKET_LEN] = {0};
uint8_t TelemetrySerial::telemetryType = TELEMETRY_SERIAL_TYPE_NONE;
mavlink_message_t TelemetrySerial::mavSendMsg;
mavlink_message_t TelemetrySerial::mavRecvMsg;
mavlink_status_t TelemetrySerial::mavRecvStatus;

static uint8_t mavSystemId = 1;
static uint8_t mavComponentId = MAV_COMP_ID_SYSTEM_CONTROL;
static uint8_t mspVersion = 1;

void TelemetrySerial::start(Stream *serialPort)
{
    stop();

    TelemetrySerial::serialPort = serialPort;

    if (serialPort == DefaultTelemetrySerial && BACKPACK_LOGGING_BAUD != TELEMETRY_SERIAL_BAUD) {
        ((HardwareSerial *)serialPort)->updateBaudRate(TELEMETRY_SERIAL_BAUD);
    }
}

void TelemetrySerial::stop()
{
    if (serialPort) {
#ifdef PLATFORM_ESP32
        portENTER_CRITICAL(&FIFOmux);
#endif

        OutFIFO.flush();

        serialPort->flush();
        while (serialPort->available()) {
            serialPort->read();
        }
                
        if (serialPort == DefaultTelemetrySerial && BACKPACK_LOGGING_BAUD != TELEMETRY_SERIAL_BAUD) {
            ((HardwareSerial *)serialPort)->updateBaudRate(BACKPACK_LOGGING_BAUD);
        }

#ifdef PLATFORM_ESP32
        portEXIT_CRITICAL(&FIFOmux);
#endif   

        serialPort = NULL;
    }
}

bool ICACHE_RAM_ATTR TelemetrySerial::addToSend(uint8_t size, uint8_t *data)
{
    if (!size)
        return false;

    bool res = false;
#ifdef PLATFORM_ESP32
    portENTER_CRITICAL(&FIFOmux);
#endif
    if (OutFIFO.ensure(size + 1))
    {
        OutFIFO.push(size); // length
        OutFIFO.pushBytes(data, size);
        res = true;
    }
#ifdef PLATFORM_ESP32
    portEXIT_CRITICAL(&FIFOmux);
#endif    
    return res;
}

ICACHE_RAM_ATTR bool TelemetrySerial::mavlinkSendMessage(void)
{
    uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
    int msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavSendMsg);
    return addToSend(msgLength, mavBuffer);
}

#define TELEMETRY_MSP_VERSION    1
#define TELEMETRY_MSP_VERSION_2  2
#define TELEMETRY_MSP_VER_SHIFT  5
#define TELEMETRY_MSP_VER_MASK   (0x7 << TELEMETRY_MSP_VER_SHIFT)
#define TELEMETRY_MSP_SEQ_MASK   0x0F
#define TELEMETRY_MSP_START_FLAG (1 << 4)

ICACHE_RAM_ATTR bool TelemetrySerial::handleCrsfRxTelemetry(uint8_t *data)
{
    if (!serialPort || telemetryType == TELEMETRY_SERIAL_TYPE_NONE)
        return false;

    crsf_ext_header_t *header = (crsf_ext_header_t*)data;
    int8_t crsfSize = CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX]);

    if (telemetryType == TELEMETRY_SERIAL_TYPE_CRSF) {
        return addToSend(crsfSize, data);
    }
    
    if (header->type < CRSF_FRAMETYPE_DEVICE_PING) {
        // crsf telemetry packet
        
        if (telemetryType == TELEMETRY_SERIAL_TYPE_MAVLINK) {
            // convert to mavlink telemetry packet

            if (header->type == CRSF_FRAMETYPE_GPS) {
                static unsigned long crsfFrameTypeGPSMicros = 0;
                if (micros() - crsfFrameTypeGPSMicros < 1000000u) {
                    return true;
                } else {
                    crsfFrameTypeGPSMicros = micros();
                    //return true;
                }

                uint8_t *byte = (&header->type + 1);
                uint32_t gpsSol_llh_lat = *byte << 24 | *(byte + 1) << 16 | *(byte + 2) << 8 | *(byte + 3);
                byte += 4;
                uint32_t gpsSol_llh_lon = *byte << 24 | *(byte + 1) << 16 | *(byte + 2) << 8 | *(byte + 3);
                byte += 4;
                uint32_t gpsSol_groundSpeed = *(byte) << 8 | *(byte + 1);
                byte += 2;
                uint32_t gpsSol_groundCourse = *(byte) << 8 | *(byte + 1);
                byte += 2;
                uint32_t gpsSol_llh_alt = *(byte) << 8 | *(byte + 1);
                byte += 2;
                uint32_t gpsSol_numSat = *(byte);
                byte += 1;

                mavlink_msg_gps_raw_int_pack(mavSystemId, mavComponentId, &mavSendMsg,
                    // time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
                    micros(),
                    // fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
                    3,
                    // lat Latitude in 1E7 degrees
                    gpsSol_llh_lat,
                    // lon Longitude in 1E7 degrees
                    gpsSol_llh_lon,
                    // alt Altitude in 1E3 meters (millimeters) above MSL
                    gpsSol_llh_alt * 10,
                    // eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                    0,
                    // epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
                    0,
                    // vel GPS ground speed (m/s * 100). If unknown, set to: 65535
                    gpsSol_groundSpeed,
                    // cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
                    gpsSol_groundCourse * 10,
                    // satellites_visible Number of satellites visible. If unknown, set to 255
                    gpsSol_numSat,
                    // alt_ellipsoid Altitude (above WGS84, EGM96 ellipsoid). Positive for up
                    0,
                    // h_acc Position uncertainty in mm,
                    0,
                    // v_acc Altitude uncertainty in mm,
                    0,
                    // vel_acc Speed uncertainty in mm (??)
                    0,
                    // hdg_acc Heading uncertainty in degE5
                    0,
                    // yaw Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
                    0
                );

                mavlinkSendMessage();
            }
            else if (header->type == CRSF_FRAMETYPE_ATTITUDE) {
                static unsigned long crsfFrameTypeAttitudeMicros = 0;
                if (micros() - crsfFrameTypeAttitudeMicros < 1000000u) {
                    return true;
                } else {
                    crsfFrameTypeAttitudeMicros = micros();
                    //return true;
                }

                uint8_t *byte = (&header->type + 1);

                uint16_t pitch = *(byte) << 8 | *(byte + 1);
                byte += 2;
                uint16_t roll = *(byte) << 8 | *(byte + 1);
                byte += 2;
                uint16_t yaw = *(byte) << 8 | *(byte + 1);
                byte += 2;

                mavlink_msg_attitude_pack(mavSystemId, mavComponentId, &mavSendMsg,
                    // time_boot_ms Timestamp (milliseconds since system boot)
                    millis(),
                    // roll Roll angle (rad)
                    RADIANS_TO_MAVLINK_RANGE(roll / 10000),
                    // pitch Pitch angle (rad)
                    RADIANS_TO_MAVLINK_RANGE(-pitch / 10000),
                    // yaw Yaw angle (rad)
                    RADIANS_TO_MAVLINK_RANGE(yaw / 10000),
                    // rollspeed Roll angular speed (rad/s)
                    0,
                    // pitchspeed Pitch angular speed (rad/s)
                    0,
                    // yawspeed Yaw angular speed (rad/s)
                    0
                );

                mavlinkSendMessage();
            }

            return true;
        }

        return false;
    }

    if (header->type == CRSF_FRAMETYPE_MSP_RESP) {
        uint8_t *mspStart = data + sizeof (crsf_ext_header_t);
        const uint8_t mspHeader = *mspStart;
        //const uint8_t seqNumber = mspHeader & TELEMETRY_MSP_SEQ_MASK;
        const uint8_t version = (mspHeader & TELEMETRY_MSP_VER_MASK) >> TELEMETRY_MSP_VER_SHIFT;

        mspStart++;

        static uint16_t mspPayloadSize;
        uint16_t cmd = 0;
        uint8_t mspFramePayloadSize = header->frame_size - 5; // crsf frame_size - type byte - crc byte - dest_addr byte - orig_addr byte - msp header byte

        if (mspHeader & TELEMETRY_MSP_START_FLAG) {
            if (version == TELEMETRY_MSP_VERSION_2) {
                mspPayloadSize = (mspStart[0] | (mspStart[1] << 8));
                cmd = (mspStart[2] | (mspStart[3] << 8));
            } else {
                mspPayloadSize = mspStart[0];
                uint8_t mspSize = mspPayloadSize + 2; // payload size + size byte + crc byte

                // restore cmd byte from CRC
                for (uint8_t i = 0;i < mspSize; i++) {
                    cmd ^= mspStart[i];
                }
            }
        }

        if (telemetryType == TELEMETRY_SERIAL_TYPE_MSP) {
            if (mspVersion == 1) {
                uint8_t buf[mspPayloadSize + 6]; // payload + header 3 byte + cmd byte + size byte + crc byte
                memcpy(buf, "$M>", 3);
                buf[3] = (uint8_t)mspPayloadSize;
                buf[4] = (uint8_t)cmd;
                memcpy(&buf[5], mspStart + 1, mspPayloadSize + 1); // msp payload + crc byte
                addToSend(sizeof buf, buf);
            }
            else if (mspVersion == 2) {
                uint8_t buf[3 + sizeof (mspHeaderV2_t) + mspPayloadSize + 1]; // prefix + header + payload + crc

                if (mspHeader & TELEMETRY_MSP_START_FLAG) {
                    mspHeaderV2_t header = {0};
                    header.payloadSize = mspPayloadSize;
                    header.function = cmd;
                    header.flags = 0;

                    uint16_t p = 0;
                    memcpy(&buf[p], "$X>", 3);
                    p += 3;
                    memcpy(&buf[p], &header, sizeof header);
                    p += sizeof header;
                }
                
                memcpy(&buf[p], mspStart + 1, mspFramePayloadSize);
                p += header.payloadSize;

                uint8_t m_crc = 0;
                for (uint8_t i = 0; i < sizeof (header) + header.payloadSize; i++) {
                    m_crc = crc8_dvb_s2(m_crc, buf[3 + i]);
                }

                memcpy(&buf[p], &m_crc, sizeof m_crc);
                p += sizeof m_crc;
                
                addToSend(sizeof buf, buf);
            }
            
            return true;
        }
        else if (telemetryType == TELEMETRY_SERIAL_TYPE_MAVLINK) {
            // convert to mavlink response

            if (cmd == MSP_WP_GETINFO) {
                uint8_t p = 0; // U8 MSP Size
                p++; // U8 Reserved for waypoint capabilities
                p++; // U8 Maximum number of waypoints supported
                p++; // U8 Is current mission valid
                uint8_t waypointsCount = mspStart[p++]; // U8 Number of waypoints in current mission

                mavlink_msg_mission_count_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, waypointsCount, MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            } else
            if (cmd == MSP_WP) {
                uint8_t p = 0; // U8 MSP Size
                uint8_t wp_no = mspStart[p++]; // U8 wp_no
                uint8_t action = mspStart[p++]; // U8 action (WAYPOINT)
                uint32_t lat = 0; // U32 lat
                lat |= mspStart[p++] << 0;
                lat |= mspStart[p++] << 8;
                lat |= mspStart[p++] << 16;
                lat |= mspStart[p++] << 24;
                uint32_t lon = 0; // U32 lon
                lon |= mspStart[p++] << 0;
                lon |= mspStart[p++] << 8;
                lon |= mspStart[p++] << 16;
                lon |= mspStart[p++] << 24;
                uint32_t alt = 0; // U32 altitude (cm)
                alt |= mspStart[p++] << 0;
                alt |= mspStart[p++] << 8;
                alt |= mspStart[p++] << 16;
                alt |= mspStart[p++] << 24;
                // U16 P1
                // U16 P2
                // U16 P3
                // U8 flags

                mavlink_msg_mission_item_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid,
                    wp_no - 1,
                    action == NAV_WP_ACTION_RTH ? MAV_FRAME_MISSION : MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    action == NAV_WP_ACTION_RTH ? MAV_CMD_NAV_RETURN_TO_LAUNCH : MAV_CMD_NAV_WAYPOINT,
                    0,
                    1,
                    0, 0, 0, 0,
                    lat / 1e7f,
                    lon / 1e7f,
                    alt / 100.0f,
                    MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            }
            
            return true;
        }
        else if (telemetryType == TELEMETRY_SERIAL_TYPE_DBG) {
            return true;
        }
    }

    return false;
}

static int incomingMissionWpCount = 0;
static int incomingMissionWpSequence = 0;

bool TelemetrySerial::setWaypoint(uint8_t wpNumber, const navWaypoint_t *wpData)
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_SET_WP;

    mspPacket.payloadSize = 1 + sizeof (navWaypoint_t);
    mspPacket.payload[0] = wpNumber; // waypoint number
    if (mspPacket.payloadSize > MSP_PORT_INBUF_SIZE) 
        return false;
    memcpy(&mspPacket.payload[1], wpData, sizeof (navWaypoint_t));

    crsf.AddMspMessage(&mspPacket);

    return true;
}

bool TelemetrySerial::getWaypoint(uint8_t wpNumber)
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_WP;

    mspPacket.payloadSize = 1;
    mspPacket.payload[0] = wpNumber; // waypoint number

    crsf.AddMspMessage(&mspPacket);

    return true;
}

bool TelemetrySerial::getWaypointCount()
{
    mspPacket_t mspPacket;
    mspPacket.reset();
    mspPacket.makeCommand();

    mspPacket.function = MSP_WP_GETINFO;
    mspPacket.payloadSize = 0;

    crsf.AddMspMessage(&mspPacket);

    return true;
}

bool TelemetrySerial::handleIncoming_MISSION_CLEAR_ALL(void)
{
    mavlink_mission_clear_all_t msg;
    mavlink_msg_mission_clear_all_decode(&mavRecvMsg, &msg);

    if (msg.target_system == mavSystemId) {
        // set zero waypoint for reset
        navWaypoint_t navWaypoint = {0};
        setWaypoint(1, &navWaypoint);

        mavlink_msg_mission_ack_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
        mavlinkSendMessage();

        return true;
    }

    return false;
}

bool TelemetrySerial::handleIncoming_MISSION_COUNT(void)
{
    mavlink_mission_count_t msg;
    mavlink_msg_mission_count_decode(&mavRecvMsg, &msg);

    if (msg.target_system == mavSystemId) {
        incomingMissionWpCount = msg.count; // We need to know how many items to request
        incomingMissionWpSequence = 0;
        mavlink_msg_mission_request_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, incomingMissionWpSequence, MAV_MISSION_TYPE_MISSION);
        mavlinkSendMessage();
        return true;
    }

    return false;
}

bool TelemetrySerial::handleIncoming_MISSION_ITEM(void)
{
    mavlink_mission_item_t msg;
    mavlink_msg_mission_item_decode(&mavRecvMsg, &msg);

    if (msg.target_system == mavSystemId) {
        if (msg.seq == incomingMissionWpSequence) {
            incomingMissionWpSequence++;

            navWaypoint_t wp;
            wp.action = (msg.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) ? NAV_WP_ACTION_RTH : NAV_WP_ACTION_WAYPOINT;
            wp.lat = (int32_t)(msg.x * 1e7f);
            wp.lon = (int32_t)(msg.y * 1e7f);
            wp.alt = msg.z * 100.0f;
            wp.p1 = 0;
            wp.p2 = 0;
            wp.p3 = 0;
            wp.flag = (incomingMissionWpSequence >= incomingMissionWpCount) ? NAV_WP_FLAG_LAST : 0;

            setWaypoint(incomingMissionWpSequence, &wp);

            if (incomingMissionWpSequence >= incomingMissionWpCount) {
                mavlink_msg_mission_ack_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            }
            else {
                mavlink_msg_mission_request_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, incomingMissionWpSequence, MAV_MISSION_TYPE_MISSION);
                mavlinkSendMessage();
            }
        }
        else {
            // Wrong sequence number received
            mavlink_msg_mission_ack_pack(mavSystemId, mavComponentId, &mavSendMsg, mavRecvMsg.sysid, mavRecvMsg.compid, MAV_MISSION_INVALID_SEQUENCE, MAV_MISSION_TYPE_MISSION);
            mavlinkSendMessage();
        }

        return true;
    }

    return false;
}

bool TelemetrySerial::handleIncoming_MISSION_REQUEST_LIST(void)
{
    mavlink_mission_request_list_t msg;
    mavlink_msg_mission_request_list_decode(&mavRecvMsg, &msg);

    if (msg.target_system == mavSystemId) {
        getWaypointCount();
        return true;
    }

    return true;
}

bool TelemetrySerial::handleIncoming_MISSION_REQUEST(void)
{
    mavlink_mission_request_t msg;
    mavlink_msg_mission_request_decode(&mavRecvMsg, &msg);

    if (msg.target_system == mavSystemId) {
        getWaypoint(msg.seq + 1);
        return true;
    }

    return false;
}

ICACHE_RAM_ATTR bool TelemetrySerial::handleSerial()
{
    if (telemetryType == TELEMETRY_SERIAL_TYPE_CRSF) {
        static uint8_t crsfInBuffer[CRSF_FRAME_SIZE_MAX];
        static uint8_t crsfInBufferOffset = 0;

        while (serialPort->available()) {
            uint8_t byte = serialPort->read();

            if (crsfInBufferOffset == 0) {
                if (byte != CRSF_ADDRESS_FLIGHT_CONTROLLER && byte != CRSF_ADDRESS_CRSF_TRANSMITTER) {
                    continue;
                }
            }

            crsfInBuffer[crsfInBufferOffset++] = byte;

            if (crsfInBufferOffset < 2) {
                continue;
            }
            else if (crsfInBufferOffset == 2) {
                if (CRSF_FRAME_SIZE(crsfInBuffer[CRSF_TELEMETRY_LENGTH_INDEX]) > CRSF_FRAME_SIZE_MAX) {
                    crsfInBufferOffset = 0;
                    continue;
                }
            }

            if (crsfInBufferOffset == CRSF_FRAME_SIZE(crsfInBuffer[CRSF_TELEMETRY_LENGTH_INDEX])) {
                crsf.AddMspMessage(crsfInBufferOffset, crsfInBuffer);
                crsfInBufferOffset = 0;
                return true;
            }
        }
    }
    else if (telemetryType == TELEMETRY_SERIAL_TYPE_MSP) {
        static mspPacket_t lastMspPacket;
        static unsigned long lastMspMicros = 0;

        while (serialPort->available()) {
            if (!lastMspMicros) {
                lastMspMicros = micros();
            }

            uint8_t byte = serialPort->read();

            if (msp.processReceivedByte(byte)) {
                mspPacket_t *mspPacket = msp.getReceivedPacket();
                if (micros() - lastMspMicros < 1000000u && mspPacket->function == lastMspPacket.function && mspPacket->version == lastMspPacket.version) {
                    // repeated packet
                    msp.markPacketReceived();
                    return true;
                } else {
                    lastMspMicros = 0;
                    memcpy(&lastMspPacket, mspPacket, sizeof lastMspPacket);
                }

                if (mspPacket->version && mspPacket->version != mspVersion) {
                    mspVersion = mspPacket->version;
                }
                
                crsf.AddMspMessage(mspPacket);
                msp.markPacketReceived();

                return true;
            }
        }
    }
    else if (telemetryType == TELEMETRY_SERIAL_TYPE_MAVLINK) {
        while (serialPort->available()) {
            uint8_t result = mavlink_parse_char(0, serialPort->read(), &mavRecvMsg, &mavRecvStatus);

            if (result == MAVLINK_FRAMING_OK) {
                if (mavRecvMsg.msgid == 0x42) {
                    // REQUEST_DATA_STREAM
                    mavlink_data_stream_t msg;
                    mavlink_msg_data_stream_decode(&mavRecvMsg, &msg);

                    mavlink_msg_data_stream_pack(mavSystemId, mavComponentId, &mavSendMsg,
                        msg.stream_id,
                        1,
                        msg.on_off
                    );

                    mavlinkSendMessage();

                    return true;
                }

                if (mavRecvMsg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    uint8_t mavSystemType = MAV_TYPE_GENERIC;
                    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                    uint8_t mavCustomMode = COPTER_MODE_ENUM_END;
                    uint8_t mavSystemState = MAV_STATE_STANDBY;

                    mavlink_msg_heartbeat_pack(mavSystemId, mavComponentId, &mavSendMsg,
                        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
                        mavSystemType,
                        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
                        MAV_AUTOPILOT_GENERIC,
                        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
                        mavModes,
                        // custom_mode A bitfield for use for autopilot-specific flags.
                        mavCustomMode,
                        // system_status System status flag, see MAV_STATE ENUM
                        mavSystemState
                    );

                    mavlinkSendMessage();

                    return true;
                }

                // convert to MSP request

                switch (mavRecvMsg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        break;
                    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                        return handleIncoming_MISSION_CLEAR_ALL();
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                        return handleIncoming_MISSION_COUNT();
                    case MAVLINK_MSG_ID_MISSION_ITEM:
                        return handleIncoming_MISSION_ITEM();
                    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                        return handleIncoming_MISSION_REQUEST_LIST();
                    case MAVLINK_MSG_ID_MISSION_REQUEST:
                        return handleIncoming_MISSION_REQUEST();
                    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                        return true;
                    default:
                        return true;
                }

                return true;
            }
        }
    }
    else if (telemetryType == TELEMETRY_SERIAL_TYPE_DBG) {
        static uint8_t buf[100];
        static uint8_t *b = buf;

        while (serialPort->available()) {
            uint8_t byte = serialPort->read();
            *b++ = byte;
            if (b - buf > sizeof(buf))
                return false;
        }

        uint8_t size = b - buf;
        if (size) {
            addToSend(size, buf);
            b = buf;
            return true;
        }

        return false;
    }

    return false;
}

void ICACHE_RAM_ATTR TelemetrySerial::handleOut()
{
    // both static to split up larger packages
    static uint8_t packageLengthRemaining = 0;
    static uint8_t sendingOffset = 0;
    uint8_t periodBytesSize = CRSF_MAX_PACKET_LEN;

    // if partial package remaining, or data in the output FIFO that needs to be written
    if (packageLengthRemaining > 0 || OutFIFO.size() > 0) {
        uint8_t periodBytesRemaining = periodBytesSize;
        while (periodBytesRemaining)
        {
#ifdef PLATFORM_ESP32
            portENTER_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
#endif
            // no package is in transit so get new data from the fifo
            if (packageLengthRemaining == 0) {
                packageLengthRemaining = OutFIFO.pop();
                OutFIFO.popBytes(outBuffer, packageLengthRemaining);
                sendingOffset = 0;
            }
#ifdef PLATFORM_ESP32
            portEXIT_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
#endif

            // if the package is long we need to split it up so it fits in the sending interval
            uint8_t writeLength;
            if (packageLengthRemaining > periodBytesRemaining) {
                if (periodBytesRemaining < periodBytesSize) {  // only start to send a split packet as the first packet
                    break;
                }
                writeLength = periodBytesRemaining;
            } else {
                writeLength = packageLengthRemaining;
            }

            serialPort->write(outBuffer + sendingOffset, writeLength);

            sendingOffset += writeLength;
            packageLengthRemaining -= writeLength;
            periodBytesRemaining -= writeLength;

            // No bytes left to send, exit
            if (OutFIFO.size() == 0)
                break;
        }
        serialPort->flush();

        // flush input
        while (serialPort->available()) {
            serialPort->read();
        }
    }
}

int TelemetrySerial::start()
{
    return DURATION_IMMEDIATELY;
}

ICACHE_RAM_ATTR int TelemetrySerial::timeout()
{
    if (telemetryType == TELEMETRY_SERIAL_TYPE_NONE) {
        if (serialPort) {
            stop();
        }
        return 1000;
    }
        
    if (!serialPort) {
        start(DefaultTelemetrySerial);
    }

    static bool out = false;

    if (out) {
        TelemetrySerial::handleOut();
    } else {
        TelemetrySerial::handleSerial();
    }
    
    out = !out;

    return DURATION_IMMEDIATELY;
}

device_t TelemetrySerial::device = {
  .initialize = NULL,
  .start = TelemetrySerial::start,
  .event = NULL,
  .timeout = TelemetrySerial::timeout
};

#endif