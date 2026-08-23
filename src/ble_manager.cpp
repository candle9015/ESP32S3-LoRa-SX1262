#include "ble_manager.h"
#include "lora_manager.h"
#include "display_manager.h"
#include "control_manager.h"
#include "imu_manager.h"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicTX = NULL;
BLECharacteristic* pCharacteristicRX = NULL;
bool deviceConnected = false;
String bleIncomingMsg = "";
String bleOutgoingMsg = "";

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleIncomingMsg = String(value.c_str());
            Serial.print("[BLE] Ricevuto da iOS: ");
            Serial.println(bleIncomingMsg);

            String ack = handleBleCommand(bleIncomingMsg);
            if (ack.length() > 0 && pCharacteristicTX != NULL) {
                bleOutgoingMsg = ack;
                pCharacteristicTX->setValue(ack.c_str());
                pCharacteristicTX->notify();

                radioStatus = "BLE OK";
                lastRxMsg = bleIncomingMsg + " -> " + ack;
                updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
            }
        }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        radioStatus = "BLE CONN";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        Serial.println("[BLE] iPhone connesso!");
    };

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        radioStatus = "BLE DISC";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        Serial.println("[BLE] iPhone disconnesso!");
        BLEDevice::startAdvertising();
    }
};

void setupBLE() {
    BLEDevice::init("LoRa_Web_Bridge");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
    );

    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristicRX->setCallbacks(new MyCallbacks());
    pCharacteristicTX->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] In attesa di connessione iOS...");
}

String handleBleCommand(const String& command) {
    String trimmed = normalizeBleCommand(command);
    trimmed.trim();

    if (trimmed.indexOf('&') >= 0 || trimmed.indexOf(';') >= 0) {
        String combinedAck = "";
        int start = 0;
        while (start < trimmed.length()) {
            int end = trimmed.indexOf('&', start);
            if (end < 0) {
                end = trimmed.indexOf(';', start);
            }
            if (end < 0) {
                end = trimmed.length();
            }

            String token = trimmed.substring(start, end);
            token.trim();
            if (token.length() > 0 && token.indexOf('=') > 0) {
                int separator = token.indexOf('=');
                String key = token.substring(0, separator);
                String value = token.substring(separator + 1);
                key.trim();
                value.trim();
                String reply = processKeyValueCommand(key, value, "BLE");
                if (combinedAck.length() > 0) {
                    combinedAck += " | ";
                }
                combinedAck += reply;
            }

            if (end >= trimmed.length()) {
                break;
            }
            start = end + 1;
        }

        if (combinedAck.length() > 0) {
            lastRxMsg = trimmed + " -> " + combinedAck;
            radioStatus = "BLE CMD";
            updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
            return "[BLE][ACK] " + combinedAck;
        }
    }

    if (trimmed == "PING") {
        return "[BLE][ACK] PONG";
    }

    if (trimmed == "STATUS") {
        return "[BLE][ACK] " + buildImuTelemetryPayload("STATUS | RSSI=" + String(radio.getRSSI()) + " dBm");
    }

    if (trimmed == "ARM" || trimmed == "DISARM" || trimmed == "HOVER" || trimmed == "EMERGENCY_STOP" ||
        trimmed == "THROTTLE_UP" || trimmed == "THROTTLE_DOWN" ||
        trimmed == "ROLL_LEFT" || trimmed == "ROLL_RIGHT" ||
        trimmed == "PITCH_UP" || trimmed == "PITCH_DOWN" ||
        trimmed == "YAW_LEFT" || trimmed == "YAW_RIGHT") {
        String ack = processRemoteCommand(trimmed);
        lastRxMsg = trimmed + " -> " + ack;
        radioStatus = "BLE CMD";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        return "[BLE][ACK] " + trimmed;
    }

    int separator = trimmed.indexOf('=');
    if (separator <= 0) {
        return "[BLE][ERR] Comando non valido: " + trimmed;
    }

    String key = trimmed.substring(0, separator);
    String value = trimmed.substring(separator + 1);
    key.trim();
    value.trim();
    return processKeyValueCommand(key, value, "BLE");
}

String normalizeBleCommand(const String& command) {
    String normalized = command;
    normalized.trim();

    if (normalized.startsWith("TX=")) {
        normalized = normalized.substring(3);
        normalized.trim();
    }

    if (normalized.startsWith("BLE=")) {
        normalized = normalized.substring(4);
        normalized.trim();
    }

    return normalized;
}
