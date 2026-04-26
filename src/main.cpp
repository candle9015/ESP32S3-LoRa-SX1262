#include <Arduino.h>
#include <Wire.h>
#include "display_manager.h"
#include <RadioLib.h>
// Test 4 PWM
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Definizione Pin Heltec V3
//Display
#define VEXT_PIN   45
//Radio LoRa SX1262
#define RADIO_CS    8
#define RADIO_DIO1  14
#define RADIO_RST   12
#define RADIO_BUSY  13
// PWM
#define PWM_SDA  42
#define PWM_SCL  46

const char testPayload[] = "HELTEC TEST";

// Correzione istanza Radio (RadioLib richiede puntatore a Module)
Module* mod = new Module(RADIO_CS, RADIO_DIO1, RADIO_RST, RADIO_BUSY);
SX1262 radio = SX1262(mod);
// Valori Radio RF - HF
#define FREQ_RTX  869.525 
#define BW  125.0
#define SF  11
#define CR  8
#define SYNC_WORD  0x24
#define POWER  5
#define PREAMBLE 32
#define CTRL_BITS 0xB4

// Variabili di stato
uint32_t txCount = 0;
uint32_t rxCount = 0;
String radioStatus = "Inizializzazione...";
String lastRxMsg = "Nessun dato";
char displayMsg[32];

// Specifichiamo di usare Wire1 (il secondo bus hardware)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);
uint32_t isWireStarted = 1;
uint32_t isPwmStarted = 1;
uint32_t isPwmResponding = 1;
// Valori calibrati per un servo standard 9g
#define SERVOMIN  150 
#define SERVOMAX  600

// Variabile per tracciare la posizione corrente del servo
uint16_t currentServoPos = (SERVOMIN + SERVOMAX) / 2;

void setup4pwm() {
    Serial.println("--- RESET BUS I2C ---"); 
    // 1. Chiudiamo eventuali istanze appese
    Wire1.end();

    // 2. Inizializziamo i pin PWM_SDA e PWM_SCL con i Pull-Up interni attivi
    // Usiamo 100kHz per massima stabilità
    bool status = Wire1.begin(PWM_SDA, PWM_SCL, 100000);

    if (!status) {
        Serial.println("ERRORE: Impossibile inizializzare Wire sui pin 42/46");
        isWireStarted = 0;
    }

    // 3. Test di presenza del chip PCA9685
    pwm.begin();
    pwm.setPWMFreq(50);
    // Prova l'inizializzazione della PWM
    if (!pwm.begin()) {
        Serial.println("Scheda PWM NON trovata. Controlla i pin 42 e 46.");
        // Non bloccare il codice, prova a scansionare
        isPwmStarted = 0;
    } else {
        Serial.println("Scheda PWM trovata!");
        pwm.setPWMFreq(50);
    }
    // Verifichiamo se risponde all'indirizzo 0x40
    Wire1.beginTransmission(0x40);
    if (Wire1.endTransmission() == 0) {
        Serial.println("SUCCESSO: Scheda PWM trovata a 0x40!");
    } else {
        Serial.println("CRITICO: Il chip a 0x40 non risponde ancora.");
        Serial.println("Controlla se il filo 3V3 e GND sono invertiti sulla PWM.");
        isPwmResponding = 0;    
    }
}

void setup4LoRa() {  

    // Inizializzazione Radio (Parametri Meshtastic LongFast)
    Serial.print(F("[Radio] Configurazione... "));
    int state = radio.begin(
        FREQ_RTX,  // Frequenza MHz
        BW,        // Bandwidth kHz
        SF,        // Spreading Factor
        CR,        // Coding Rate (4/8)
        SYNC_WORD, // Sync Word (byte alto, verrà completato sotto)
        POWER,     // Potenza ridotta a 5dBm per testare la stabilità grafica
        PREAMBLE   // Preamble Length
    );

    if (state == RADIOLIB_ERR_NONE) {
        state = radio.setSyncWord(SYNC_WORD, CTRL_BITS); 
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

    //updateDisplay(txCount, radioStatus, lastRxMsg.c_str());
    pinMode(RADIO_DIO1, INPUT);
}

void setup() {
    Serial.begin(115200); 
    // Init setup for PWM Servo - N°1 CH-0  
    setup4pwm();

    // Init LoRa
    setup4LoRa();

    // Inizializzazione Display tramite il nuovo manager+
    Serial.println();
    Serial.println();
    setupDisplay();

}

// === GESTIONE SERVO BASATA SU MESSAGGIO RX ===
void rxMsgParserAndResponse(String rxData) {
    Serial.println("[Radio] from RX: " + rxData );

    // === GESTIONE SERVO BASATA SU MESSAGGIO RX ===
    // Gestione comando PING
    if (rxData == "PING") {
        Serial.println("[Radio] Ricevuto PING, invio PONG...");
        txCount++;
        radio.transmit("[ACK] PONG - Sistema Attivo");
        updateDisplay(txCount, String(FREQ_RTX), "PONG SENT", lastRxMsg.c_str());
        return;
    }

    // Gestione comando STATUS remoto
    if (rxData == "STATUS") {
        txCount++;
        String statusMsg = "[ACK] STATUS: OK | RSSI: " + String(radio.getRSSI()) + "dBm";
        radio.transmit(statusMsg);
        return;
    }

    // Gestione comandi numerici per il Servo
    // Prova a parsare il messaggio come numero (0-255 o 0-1023)
    if(isWireStarted && isPwmStarted && isPwmResponding){
        long servoValue = rxData.toInt();
        if ((servoValue != 0 || rxData == "0")) {
            // Mappa il valore ricevuto all'intervallo del servo
            // Assumiamo input 0-255 (tipico da sensori)
            uint16_t targetPulse = map(servoValue, 0, 255, SERVOMIN, SERVOMAX);
            // Limita ai valori min/max del servo
            targetPulse = constrain(targetPulse, SERVOMIN, SERVOMAX);
            
            // Spostamento graduale dalla posizione corrente alla target
            int step = (targetPulse > currentServoPos) ? 5 : -5;
            while (currentServoPos != targetPulse) {
                currentServoPos += step;
                // Verifica che non si superi il target
                if ((step > 0 && currentServoPos > targetPulse) ||
                    (step < 0 && currentServoPos < targetPulse)) {
                    currentServoPos = targetPulse;
                }
                pwm.setPWM(0, 0, currentServoPos);
                delay(20); // Velocità dello spostamento (20ms per step)
            }
            Serial.printf("[Servo] Posizione: %d (pulselen: %d)\n", servoValue, currentServoPos);

            // === INVIO FEEDBACK AL GATEWAY ===
            txCount++; // Incrementiamo il contatore trasmissioni dell'ESP
            String feedback = "[ACK] Servo impostato a " + String(servoValue);
            
            // Trasmettiamo la conferma
            radio.transmit(feedback);
            
            Serial.println("[Radio] Feedback inviato al Gateway");
        }
    }
}

void RX_Manager(uint32_t &lastDisplayUpdate){
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

            // Rsponde in base al messaggio ricevuto
            rxMsgParserAndResponse(rxData);

            // AGGIORNAMENTO IMMEDIATO del display
            updateDisplay(txCount, String(FREQ_RTX), radioStatus, lastRxMsg.c_str());
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
}

void TX_Manager(uint32_t lastTx){
    // 2. Gestione Radio TX (ogni 5 secondi)
    if (millis() - lastTx >= 5000) {
        lastTx = millis();
        
        radioStatus = "TX...";
        updateDisplay(txCount, String(FREQ_RTX), radioStatus, lastRxMsg.c_str()); // Mostra l'ultimo messaggio RX anche durante TX
        
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

        updateDisplay(txCount, String(FREQ_RTX), radioStatus, lastRxMsg.c_str()); // Mostra l'ultimo messaggio RX anche dopo TX
        
        // Torna in modalità ricezione dopo la trasmissione
        radio.startReceive();
    }
}

void updateDataMonitor(uint32_t lastDisplayUpdate){
    // 3. Aggiornamento periodico display (ogni secondo)
    if (millis() - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = millis();
        updateDisplay(txCount, String(FREQ_RTX), radioStatus, lastRxMsg.c_str());
    }
}

void loop() {
    static uint32_t lastDisplayUpdate = 0;
    static uint32_t lastTx = 0;
    
    RX_Manager(lastDisplayUpdate);
    //TX_Manager(lastTx);
    updateDataMonitor(lastDisplayUpdate);
    
}
