#include <Arduino.h>
#include "display_manager.h"
#include "lora_manager.h"
#include "ble_manager.h"
#include "pwm_manager.h"
#include "imu_manager.h"

#define VEXT_PIN 45

void setup() {
    Serial.begin(115200);

    setupBLE();
    setup4pwm();
    setupDisplay();
    setup4LoRa();
    initIMU();

    yawValue = 128;
    rollValue = 128;
    pitchValue = 128;
    throttleValue = 0;
    setDefaultDevice(yawValue, rollValue, pitchValue, throttleValue);

    Serial.println();
    Serial.println();
}

void updateDataMonitor(uint32_t &lastDisplayUpdate) {
    if (millis() - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = millis();
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
    }
}

void loop() {
    static uint32_t lastDisplayUpdate = 0;
    updateIMU();
    RX_Manager(lastDisplayUpdate);
    updateDataMonitor(lastDisplayUpdate);
}
