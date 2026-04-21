#include <Arduino.h>
#include <Wire.h>
#include "display_manager.h"
#include <RadioLib.h>

// Definizione Pin Heltec V3
#define VEXT_PIN   45
#define RADIO_CS    8
#define RADIO_DIO1  14
#define RADIO_RST   12
#define RADIO_BUSY  13

const char testPayload[] = "HELTEC TEST";

// Correzione istanza Radio (RadioLib richiede puntatore a Module)
Module* mod = new Module(RADIO_CS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
SX1262 radio = SX1262(mod);

// Variabili di stato
uint32_t txCount = 0;
uint32_t rxCount = 0;
String radioStatus = "Inizializzazione...";
String lastRxMsg = "Nessun dato";
char displayMsg[32];


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  
  // Inizializzazione Display tramite il nuovo manager
  setupDisplay();

  Serial.println(F("[Display] Inizializzato con fix hardware"));

  // 4. Inizializzazione Radio (Parametri Meshtastic LongFast)
  Serial.print(F("[Radio] Configurazione... "));
  int state = radio.begin(
        869.525,  // Frequenza MHz
        125.0,    // Bandwidth kHz
        11,       // Spreading Factor
        8,        // Coding Rate (4/8)
        0x24,     // Sync Word (byte alto, verrà completato sotto)
        5,        // Potenza ridotta a 5dBm per testare la stabilità grafica
        32        // Preamble Length
    );

  if (state == RADIOLIB_ERR_NONE) {
    state = radio.setSyncWord(0x24, 0xB4); 
    if (state == RADIOLIB_ERR_NONE) {
      radioStatus = "Radio OK";
      Serial.println(F("Configurazione Radio: OK!"));
      
      // Avvia la modalità ricezione
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

  updateDisplay(txCount, radioStatus, lastRxMsg.c_str());
  pinMode(RADIO_DIO1, INPUT);
}

void loop() {
    static uint32_t lastDisplayUpdate = 0;
    static uint32_t lastTx = 0;

    // 1. Gestione Ricezione (RX) - Interroga la radio solo se il pin DIO1 è alto
    if (digitalRead(RADIO_DIO1) == HIGH) {
        String rxData;
        int rxState = radio.readData(rxData);

        if (rxState == RADIOLIB_ERR_NONE) {
            rxCount++;
            lastRxMsg = rxData;
            radioStatus = "RX " + String((int)radio.getRSSI()) + "dBm";

            Serial.printf("[Radio] Ricevuto: %s | RSSI: %.2f | SNR: %.2f\n", 
                          rxData.c_str(), radio.getRSSI(), radio.getSNR());

            // AGGIORNAMENTO IMMEDIATO del display
            updateDisplay(txCount, radioStatus, lastRxMsg.c_str());
            lastDisplayUpdate = millis(); 
        } 
        else if (rxState == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println(F("[Radio] Errore CRC! Parametri corretti ma segnale disturbato."));
            radioStatus = "CRC ERROR";
        }
        else {
            Serial.printf("[Radio] Errore RX: %d\n", rxState);
        }
        
        // Fondamentale: riattiva la ricezione dopo ogni evento su DIO1
        radio.startReceive();
    }

    // 2. Gestione Radio TX (ogni 5 secondi)
    if (millis() - lastTx >= 5000) {
        lastTx = millis();
        
        radioStatus = "TX...";
        updateDisplay(txCount, radioStatus, lastRxMsg.c_str()); // Mostra l'ultimo messaggio RX anche durante TX
        
        // Pausa di sicurezza: lascia che l'I2C finisca prima che la radio assorba corrente
        delay(50); 

        // Trasmissione LoRa Reale
        int txState = radio.transmit(testPayload);

        if (txState == RADIOLIB_ERR_NONE) {
            txCount++;
            radioStatus = "INVIATO OK";
            Serial.printf("[Radio] Pacchetto #%d inviato\n", txCount);
        } else {
            radioStatus = "ERR " + String(txState);
            Serial.printf("[Radio] Errore trasmissione: %d\n", txState);
        }

        updateDisplay(txCount, radioStatus, lastRxMsg.c_str()); // Mostra l'ultimo messaggio RX anche dopo TX
        
        // Torna in modalità ricezione dopo la trasmissione
        radio.startReceive();
    }

    // 3. Aggiornamento periodico display (ogni secondo)
    if (millis() - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = millis();
        updateDisplay(txCount, radioStatus, lastRxMsg.c_str());
    }
}
