#include "lora_manager.h"
#include "display_manager.h"
#include "control_manager.h"
#include "imu_manager.h"

const char testPayload[] = "HELTEC TEST";
Module* mod = new Module(RADIO_CS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
SX1262 radio = SX1262(mod);

uint32_t txCount = 0;
uint32_t rxCount = 0;
String radioStatus = "Inizializzazione...";
String lastRxMsg = "Nessun dato";
String currentRadioFreq = String(FREQ_RTX, 3);

void setup4LoRa() {
    Serial.print(F("[Radio] Configurazione... "));
    int state = radio.begin(
        FREQ_RTX,
        BW,
        SF,
        CR,
        SYNC_WORD,
        POWER,
        PREAMBLE
    );

    if (state == RADIOLIB_ERR_NONE) {
        state = radio.setSyncWord(SYNC_WORD, CTRL_BITS);
        if (state == RADIOLIB_ERR_NONE) {
            radioStatus = "Radio OK";
            Serial.println(F("Configurazione Radio: OK!"));

            state = radio.startReceive();
            if (state != RADIOLIB_ERR_NONE) {
                Serial.printf("Errore startReceive: %d\n", state);
            }
        }
    }

    if (state != RADIOLIB_ERR_NONE) {
        radioStatus = "Radio ERR: " + String(state);
        Serial.printf("ERRORE Radio: %d\n", state);
    }

    pinMode(RADIO_DIO1, INPUT);
}

void rxMsgParserAndResponse(String rxData) {
    Serial.println("[Radio] from RX: " + rxData );

    if (rxData == "PING") {
        Serial.println("[Radio] Ricevuto PING, invio PONG...");
        txCount++;
        radio.transmit("[ACK] PONG - Sistema Attivo");
        updateDisplay(txCount, String(FREQ_RTX), "PONG SENT", lastRxMsg.c_str());
        return;
    }

    if (rxData == "STATUS") {
        txCount++;
        String statusMsg = "[ACK] " + buildImuTelemetryPayload("STATUS: OK | RSSI=" + String(radio.getRSSI()) + "dBm");
        radio.transmit(statusMsg);
        return;
    }

    if (isLiveRemoteTextCommand(rxData)) {
        processRemoteCommand(rxData);
        Serial.println("[Radio] Live control applied without ack: " + rxData);
        return;
    }

    if (rxData == "ARM" || rxData == "DISARM" || rxData == "HOVER" || rxData == "EMERGENCY_STOP") {
        String ack = processRemoteCommand(rxData);
        if (ack.length() > 0) {
            txCount++;
            radio.transmit(ack);
            Serial.println("[Radio] Remote cmd processed: " + rxData + " -> " + ack);
        }
        return;
    }

    if (rxData.indexOf('&') >= 0 || rxData.indexOf(';') >= 0) {
        String combined = rxData;
        int start = 0;
        String combinedResp = "";
        while (start < combined.length()) {
            int end = combined.indexOf('&', start);
            if (end < 0) {
                end = combined.indexOf(';', start);
            }
            if (end < 0) {
                end = combined.length();
            }

            String token = combined.substring(start, end);
            token.trim();
            if (token.length() > 0 && token.indexOf('=') > 0) {
                int sep = token.indexOf('=');
                String key = token.substring(0, sep);
                String value = token.substring(sep + 1);
                key.trim(); value.trim();
                if (isLiveControlKey(key)) {
                    processKeyValueCommand(key, value, "LORA");
                    continue;
                }
                String resp = processKeyValueCommand(key, value, "LORA");
                if (combinedResp.length() > 0) {
                    combinedResp += " | ";
                }
                combinedResp += resp;
            }

            if (end >= combined.length()) {
                break;
            }
            start = end + 1;
        }

        if (combinedResp.length() > 0) {
            txCount++;
            radio.transmit(combinedResp);
            Serial.println("[Radio] Multi-key processed -> " + combinedResp);
            return;
        }
        return;
    }

    int sep = rxData.indexOf('=');
    if (sep > 0) {
        String key = rxData.substring(0, sep);
        String value = rxData.substring(sep + 1);
        key.trim(); value.trim();
        if (isLiveControlKey(key)) {
            processKeyValueCommand(key, value, "LORA");
            Serial.println("[Radio] Live key applied without ack: " + key + "=" + value);
            return;
        }
        String resp = processKeyValueCommand(key, value, "LORA");
        if (resp.length() > 0) {
            txCount++;
            radio.transmit(resp);
            Serial.println("[Radio] Key=Value processed -> " + resp);
        }
        return;
    }

    if (isWireStarted && isPwmStarted && isPwmResponding) {
        long servoValue = rxData.toInt();
        if ((servoValue != 0 || rxData == "0")) {
            setChannelFast(CH_THROTTLE, servoValue);
        }
    }
}

void RX_Manager(uint32_t &lastDisplayUpdate) {
    if (digitalRead(RADIO_DIO1) == HIGH) {
        String rxData;
        int rxState = radio.readData(rxData);

        if (rxState == RADIOLIB_ERR_NONE) {
            rxCount++;
            updateDisplay(txCount, currentRadioFreq, "PONG SENT", lastRxMsg.c_str());
            radioStatus = "RX " + String((int)radio.getRSSI()) + "dBm";

            Serial.printf("[Radio] Ricevuto: %s | RSSI: %.2f | SNR: %.2f\n",
                          rxData.c_str(), radio.getRSSI(), radio.getSNR());

            rxMsgParserAndResponse(rxData);

            updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
            lastDisplayUpdate = millis();
        }
        else if (rxState == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println(F("[Radio] Errore CRC! Parametri corretti ma segnale disturbato."));
            radioStatus = "CRC ERROR";
        }
        else {
            Serial.printf("[Radio] Errore RX: %d\n", rxState);
        }

        radio.startReceive();
    }
}

void TX_Manager(uint32_t &lastTx) {
    if (millis() - lastTx >= 5000) {
        lastTx = millis();

        radioStatus = "TX...";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());

        delay(50);

        int txState = radio.transmit(testPayload);

        if (txState == RADIOLIB_ERR_NONE) {
            txCount++;
            radioStatus = "INVIATO OK";
            Serial.printf("[Radio] Pacchetto #%d inviato\n", txCount);
        } else {
            radioStatus = "ERR " + String(txState);
            Serial.printf("[Radio] Errore trasmissione: %d\n", txState);
        }

        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        radio.startReceive();
    }
}
