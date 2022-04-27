#include "tracker.h"
#include "telemetry.h"
#include "crsf_protocol.h"
#include "logging.h"

/*
    CRSF frame has the structure:
    <Device address> <Frame length> <Type> <Payload> <CRC>
    Device address: (uint8_t)
    Frame length:   length in  bytes including Type (uint8_t)
    Type:           (uint8_t)
    CRC:            (uint8_t), crc of <Type> and <Payload>
*/
typedef struct crsf_telemetry_frame_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload;
} PACKED crsf_telemetry_frame_t;

/*
    type 0x02 GPS
    Payload:
    int32_t     Latitude ( degree / 10`000`000 )
    int32_t     Longitude (degree / 10`000`000 )
    uint16_t    Groundspeed ( km/h / 10 )
    uint16_t    GPS heading ( degree / 100 )
    uint16      Altitude ( meter ­1000m offset )
    uint8_t     Satellites in use ( counter )
    */
typedef struct crsf_telemetry_gps_s {
    int32_t latitude; // degree / 10`000`000
    int32_t longitude; // degree / 10`000`000
    uint16_t groundSpeed; // km/h / 10
    uint16_t heading; // degree / 100
    uint16_t altitude; // meter ­1000m offset
    uint8_t satellites; // counter
} PACKED crsf_telemetry_gps_t;

crsf_telemetry_gps_t gpsHome = {0,};
crsf_telemetry_gps_t gpsRemote = {0,};

bool parceCRSFTelemetry(uint8_t *CRSFinBuffer)
{
    const crsf_telemetry_frame_t *telemetryPackage = (crsf_telemetry_frame_t *) CRSFinBuffer;

    if (telemetryPackage->type == CRSF_FRAMETYPE_COMMAND || telemetryPackage->type == CRSF_FRAMETYPE_DEVICE_PING) {
        return false;
    }

    if (telemetryPackage->type == CRSF_FRAMETYPE_GPS) {
        const crsf_telemetry_gps_t *gps_data = (crsf_telemetry_gps_t *) &telemetryPackage->payload;
        memcpy(&gpsRemote, gps_data, sizeof(gpsRemote));

        DBGLN("%u %u %u %u", gps_data->latitude, gps_data->longitude, gps_data->altitude, gps_data->heading);

        return true;
    }

    return false;
}