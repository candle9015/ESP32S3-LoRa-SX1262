#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>

struct GpsTelemetry {
    double latitude;
    double longitude;
    double altitude;    // metri
    float speedKnots;
    float courseTrue;
    uint8_t satellites;
    bool valid;
    uint32_t lastFixTime; // millis quando l'ultimo fix valido fu acquisito
};

void setupGPS();
void updateGPS();
bool isGPSReady();
bool isGPSFixed();
GpsTelemetry getGPSState();
String buildGpsTelemetryPayload(const String& prefix = "GPS");
String getGPSStatusString();

#endif
