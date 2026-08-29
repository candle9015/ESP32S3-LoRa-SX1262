#include "ble_manager.h"
#include "lora_manager.h"
#include "display_manager.h"
#include "control_manager.h"
#include "imu_manager.h"
#include "gps_manager.h"
#include "pwm_manager.h"
#include "protocol.h"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicTX = NULL;
BLECharacteristic* pCharacteristicRX = NULL;
bool deviceConnected = false;
String bleIncomingMsg = "";
String bleOutgoingMsg = "";
static uint32_t lastTelemetryBroadcastMs = 0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            bleIncomingMsg = String(value.c_str());
            //Serial.print("[BLE] Ricevuto da iOS: ");
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
        broadcastBleTelemetry();
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

    // FIX: il descrittore CCCD (0x2902) deve avere permesso di scrittura
    // esplicito. Senza questa riga, Windows (che scrive il CCCD in modo
    // esplicito per abilitare le notify, a differenza di macOS/iOS) riceve
    // un rifiuto GATT_WRITE_NOT_PERMIT e le notify non si attivano mai,
    // pur restando la connessione GATT valida.
    BLE2902* pTxCccd = new BLE2902();
    pTxCccd->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
    pCharacteristicTX->addDescriptor(pTxCccd);

    pService->start();

    // Debug: stampa gli handle assegnati per poter correlare eventuali
    // errori GATT (es. "GATT_WRITE_NOT_PERMIT, handle:0x00xx") con
    // l'attributo esatto a cui si riferiscono.
    Serial.printf("[BLE][DEBUG] Service handle:        0x%04x\n", pService->getHandle());
    Serial.printf("[BLE][DEBUG] RX characteristic handle: 0x%04x\n", pCharacteristicRX->getHandle());
    Serial.printf("[BLE][DEBUG] TX characteristic handle: 0x%04x\n", pCharacteristicTX->getHandle());
    Serial.printf("[BLE][DEBUG] TX CCCD descriptor handle: 0x%04x\n", pTxCccd->getHandle());

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] In attesa di connessione iOS...");
}

String handleBleCommand(const String& command) {
    String trimmed = normalizeBleCommand(command);
    trimmed.trim();

    if (trimmed.startsWith("C|")) {
        uint8_t seq = 0;
        uint8_t flags = 0;
        uint8_t throttle = 0;
        uint8_t roll = 0;
        uint8_t pitch = 0;
        uint8_t yaw = 0;
        uint8_t crc = 0;

        if (compact_protocol::parseControlFrame(trimmed, seq, flags, throttle, roll, pitch, yaw, crc)) {
            throttleValue = throttle;
            rollValue = roll;
            pitchValue = pitch;
            yawValue = yaw;
            setChannelsFastPair(CH_ROLL, rollValue, CH_PITCH, pitchValue);
            setChannelFromInput(CH_THROTTLE, throttleValue);
            setChannelFast(CH_YAW, yawValue);
            lastRxMsg = trimmed;
            radioStatus = "BLE CTRL";
            updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
            return "[BLE][ACK] compact CTRL";
        }
    }

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
        String response = "[BLE][ACK] " + buildImuTelemetryPayload("STATUS | RSSI=" + String(radio.getRSSI()) + " dBm");
        response += " | GPS: " + getGPSStatusString();
        return response;
    }

    if (trimmed == "GPS_STATUS") {
        return buildGpsTelemetryPayload("GPS_STATUS");
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

void broadcastBleTelemetry() {
    if (!deviceConnected || pCharacteristicTX == NULL) {
        return;
    }

    ImuTelemetry telemetry = getIMUState();
    String payload = compact_protocol::encodeTelemetryFrame(
        0,
        0,
        static_cast<uint8_t>(throttleValue),
        static_cast<int16_t>(telemetry.rollDeg),
        static_cast<int16_t>(telemetry.pitchDeg),
        static_cast<int16_t>(telemetry.yawRateDegPerSec),
        static_cast<int16_t>(telemetry.tempC * 10.0f),
        static_cast<int8_t>(radio.getRSSI()),
        100);

    pCharacteristicTX->setValue(payload.c_str());
    pCharacteristicTX->notify();
    lastTelemetryBroadcastMs = millis();
    // Serial.printf("[BLE] Telemetry broadcast: %s\n", payload.c_str());
}

void updateBleTelemetryLoop() {
    if (!deviceConnected || pCharacteristicTX == NULL) {
        return;
    }

    if (millis() - lastTelemetryBroadcastMs >= 2000) {
        broadcastBleTelemetry();
    }
}