#include "imu_manager.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Usiamo direttamente l'istanza hardware Wire1 fornita dall'ESP32
#define imuWire Wire1

//static TwoWire imuWire(1);
static Adafruit_MPU6050 mpu;
static bool imuReady = false;
static bool imuCalibrated = false;
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float yawRateDegPerSec = 0.0f;
static float accelX = 0.0f;
static float accelY = 0.0f;
static float accelZ = 0.0f;
static float tempC = 0.0f;
static float gyroXOffset = 0.0f;
static float gyroYOffset = 0.0f;
static float gyroZOffset = 0.0f;
static uint32_t lastImuMicros = 0;
static uint32_t lastImuDebugMs = 0;

void setupIMU() {
    Serial.println("[IMU] trying to initialize on SDA=42 / SCL=46");

    // Ripuliamo il bus prima di inizializzarlo
    imuWire.end();
    delay(10);

    // Inizializziamo il bus Wire1 nativo
    bool ok = imuWire.begin(42, 46, 100000);
    if (!ok) {
        Serial.println("[IMU] Errore critico: Impossibile allocare il bus I2C hardware!");
        imuReady = false;
        return;
    }
    
    // Piccolo ritardo per stabilizzare le linee elettriche
    delay(50); 

    imuWire.beginTransmission(0x68);
    byte error = imuWire.endTransmission();

    if (error != 0) {
        Serial.printf("[IMU] MPU missing on I2C address 0x68 (error=%d). Hardware not detected.\n", error);
        imuReady = false;
        imuCalibrated = false;
        return;
    }

    Serial.println("[IMU] MPU detected!");

    if (!mpu.begin(0x68, &imuWire)) {
        Serial.println("[IMU] initialization failed after detection.");
        imuReady = false;
        imuCalibrated = false;
        return;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    imuReady = true;
    lastImuMicros = micros();
    calibrateIMU(200);
    Serial.println("[IMU] initialized successfully");
   
}

void calibrateIMU(uint16_t samples) {
    if (!imuReady) {
        return;
    }

    float gxSum = 0.0f;
    float gySum = 0.0f;
    float gzSum = 0.0f;

    for (uint16_t i = 0; i < samples; ++i) {
        sensors_event_t a, g, tempEvent;
        mpu.getEvent(&a, &g, &tempEvent);
        gxSum += g.gyro.x;
        gySum += g.gyro.y;
        gzSum += g.gyro.z;
        delay(2);
    }

    gyroXOffset = gxSum / samples;
    gyroYOffset = gySum / samples;
    gyroZOffset = gzSum / samples;
    imuCalibrated = true;
    Serial.println("[IMU] calibration complete");
}

void updateIMU() {
    if (!imuReady) {
        return;
    }

    sensors_event_t accelEvent, gyroEvent, tempEvent;
    if (!mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent)) {
        Serial.println("[IMU] read failed, sensor lost during runtime. Disabling IMU.");
        imuReady = false;
        imuCalibrated = false;
        return;
    }

    accelX = accelEvent.acceleration.x;
    accelY = accelEvent.acceleration.y;
    accelZ = accelEvent.acceleration.z;
    tempC = tempEvent.temperature;

    float gx = gyroEvent.gyro.x - gyroXOffset;
    float gy = gyroEvent.gyro.y - gyroYOffset;
    float gz = gyroEvent.gyro.z - gyroZOffset;

    float dt = (micros() - lastImuMicros) / 1000000.0f;
    lastImuMicros = micros();
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.016f;
    }

    float accelRoll = atan2f(accelY, sqrtf(accelX * accelX + accelZ * accelZ)) * 180.0f / PI;
    float accelPitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;

    if (!imuCalibrated) {
        rollDeg = accelRoll;
        pitchDeg = accelPitch;
    } else {
        rollDeg = 0.98f * (rollDeg + gx * dt) + 0.02f * accelRoll;
        pitchDeg = 0.98f * (pitchDeg + gy * dt) + 0.02f * accelPitch;
    }

    yawRateDegPerSec = gz;

    /*
    if (millis() - lastImuDebugMs >= 5000) {
        lastImuDebugMs = millis();
        Serial.printf("[IMU] roll=%.1f pitch=%.1f yawRate=%.1f temp=%.1f\n",
                      rollDeg, pitchDeg, yawRateDegPerSec, tempC);
    }
    */
}

bool isIMUReady() {
    return imuReady;
}

float getRollDeg() {
    return rollDeg;
}

float getPitchDeg() {
    return pitchDeg;
}

float getYawRateDegPerSec() {
    return yawRateDegPerSec;
}

float getTemperatureC() {
    return tempC;
}

ImuTelemetry getIMUState() {
    ImuTelemetry state;
    state.rollDeg = rollDeg;
    state.pitchDeg = pitchDeg;
    state.yawRateDegPerSec = yawRateDegPerSec;
    state.accelX = accelX;
    state.accelY = accelY;
    state.accelZ = accelZ;
    state.tempC = tempC;
    state.valid = imuReady;
    return state;
}

String buildImuTelemetryPayload(const String& prefix) {
    ImuTelemetry state = getIMUState();
    String payload = prefix;

    if (!state.valid) {
        payload += " | IMU=INVALID";
        return payload;
    }

    payload += " | ROLL=" + String(state.rollDeg, 1);
    payload += " | PITCH=" + String(state.pitchDeg, 1);
    payload += " | YAW_RATE=" + String(state.yawRateDegPerSec, 1);
    payload += " | AX=" + String(state.accelX, 2);
    payload += " | AY=" + String(state.accelY, 2);
    payload += " | AZ=" + String(state.accelZ, 2);
    payload += " | TEMP=" + String(state.tempC, 1);

    Serial.printf("[IMU] Telemetry Payload: %s\n", payload.c_str());

    return payload;
}
