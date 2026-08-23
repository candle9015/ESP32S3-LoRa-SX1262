#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>

struct ImuTelemetry {
    float rollDeg;
    float pitchDeg;
    float yawRateDegPerSec;
    float accelX;
    float accelY;
    float accelZ;
    float tempC;
    bool valid;
};

void setupIMU();
void updateIMU();
void calibrateIMU(uint16_t samples = 200);
bool isIMUReady();
float getRollDeg();
float getPitchDeg();
float getYawRateDegPerSec();
float getTemperatureC();
ImuTelemetry getIMUState();

#endif
