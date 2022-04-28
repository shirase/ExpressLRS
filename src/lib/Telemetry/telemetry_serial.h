#pragma once

#include "targets.h"
#include "FIFO.h"
#include "device.h"

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "../MAVLink/common/mavlink.h"

#define CRSF_MSP_LENGTH_OFFSET 1

#define COPTER_MODE_ENUM_END 22

#define M_PIf       3.14159265358979323846f

/**
 * MAVLink requires angles to be in the range -Pi..Pi.
 * This converts angles from a range of 0..Pi to -Pi..Pi
 */
#define RADIANS_TO_MAVLINK_RANGE(angle) (angle > M_PIf) ? angle - (2 * M_PIf) : angle

enum {
    TELEMETRY_SERIAL_TYPE_NONE = 0,
    TELEMETRY_SERIAL_TYPE_MAVLINK,
    TELEMETRY_SERIAL_TYPE_MSP,
    TELEMETRY_SERIAL_TYPE_CRSF,
    TELEMETRY_SERIAL_TYPE_DBG,
};

typedef struct __attribute__ ((packed)) {
    uint8_t action;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t p1, p2, p3;
    uint8_t flag;
} navWaypoint_t;

typedef enum {
    NAV_WP_ACTION_WAYPOINT  = 0x01,
    NAV_WP_ACTION_HOLD_TIME = 0x03,
    NAV_WP_ACTION_RTH       = 0x04,
    NAV_WP_ACTION_SET_POI   = 0x05,
    NAV_WP_ACTION_JUMP      = 0x06,
    NAV_WP_ACTION_SET_HEAD  = 0x07,
    NAV_WP_ACTION_LAND      = 0x08
} navWaypointActions_e;

typedef enum {
    NAV_WP_FLAG_HOME = 0x48,
    NAV_WP_FLAG_LAST = 0xA5
} navWaypointFlags_e;

class TelemetrySerial
{
    public:
        static void start(Stream *serialPort);
        static void stop();
        ICACHE_RAM_ATTR static bool handleCrsfRxTelemetry(uint8_t *data);

        static device_t device;
        static uint8_t telemetryType;

    private:
        static Stream *serialPort;
        static FIFO OutFIFO;
#ifdef PLATFORM_ESP32
        static portMUX_TYPE FIFOmux;
#endif
        static uint8_t outBuffer[];
        static mavlink_message_t mavSendMsg;
        static mavlink_message_t mavRecvMsg;
        static mavlink_status_t mavRecvStatus;

        ICACHE_RAM_ATTR static void handleOut();
        static int start();
        ICACHE_RAM_ATTR static int timeout();
        ICACHE_RAM_ATTR static bool addToSend(uint8_t size, uint8_t *data);
        ICACHE_RAM_ATTR static bool handleSerial();
        ICACHE_RAM_ATTR static bool mavlinkSendMessage(void);
        static bool setWaypoint(uint8_t wpNumber, const navWaypoint_t *wpData);
        static bool getWaypoint(uint8_t wpNumber);
        static bool getWaypointCount();
        static bool handleIncoming_MISSION_CLEAR_ALL(void);
        static bool handleIncoming_MISSION_COUNT(void);
        static bool handleIncoming_MISSION_ITEM(void);
        static bool handleIncoming_MISSION_REQUEST_LIST(void);
        static bool handleIncoming_MISSION_REQUEST(void);
};