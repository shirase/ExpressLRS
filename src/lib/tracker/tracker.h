#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "targets.h"

typedef struct crsf_telemetry_gps_s {
    int32_t Latitude;
    int32_t Longitude;
    uint16_t Groundspeed; // km/h / 10
    uint16_t GPS_heading; // degree / 100
    uint16_t Altitude; // meter ­1000m offset
    uint8_t Satellites; // counter
} PACKED crsf_telemetry_gps_t;

bool parceCRSFTelemetry(uint8_t *CRSFinBuffer);