#include <Arduino.h>
#include <Wire.h>
#include "display_manager.h"
#include <RadioLib.h>
// Test 4 PWM
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// Bluetooth LE per iOS
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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

// UUID per BLE (Generati casualmente)
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristicTX = NULL;
bool deviceConnected = false;
String bleIncomingMsg = "";
String bleOutgoingMsg = "";

String handleBleCommand(const String& command);
String normalizeBleCommand(const String& command);
String processRemoteCommand(const String& command);
// Forward declaration for shared key=value handler used by BLE and LoRa code
String processKeyValueCommand(const String& keyIn, const String& valueIn, const String& via);
// Forward declaration for gradual channel movement
void moveChannelGradual(uint8_t channel, long value, int stepDelayMs = 20, int stepSize = 5);
void setServoFromInput(long servoValue);
void setChannelFromInput(uint8_t channel, long value);

// Variabili di stato
uint32_t txCount = 0;
uint32_t rxCount = 0;
String radioStatus = "Inizializzazione...";
String lastRxMsg = "Nessun dato";
String currentRadioFreq = String(FREQ_RTX, 3);
char displayMsg[32];

// Specifichiamo di usare Wire1 (il secondo bus hardware)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);
uint32_t isWireStarted = 1;
uint32_t isPwmStarted = 1;
uint32_t isPwmResponding = 1;
// Valori calibrati per un servo standard 9g
#define SERVOMIN  150 
#define SERVOMAX  600

// Channel mapping for remote control outputs
constexpr uint8_t CH_THROTTLE = 0;
constexpr uint8_t CH_ROLL = 1;
constexpr uint8_t CH_PITCH = 2;
constexpr uint8_t CH_YAW = 3;

// Variabile per tracciare la posizione corrente dei servo per canale (in pulse)
uint16_t currentChannelPulse[4] = { (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2 };

// Stato per il controllo remoto
bool armed = false;
int throttleValue = 0; // 0-255 range, default start 0
int rollValue = 128;
int pitchValue = 128;
int yawValue = 128;


// Callback per ricezione dati da iOS via BLE
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
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

// Callback per gestione connessione/disconnessione iOS
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        radioStatus = "BLE CONN";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        Serial.println("[BLE] iPhone connesso!");
    };
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        radioStatus = "BLE DISC";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        Serial.println("[BLE] iPhone disconnesso!");
        BLEDevice::startAdvertising(); // Riavvia advertising
    }
};

// Funzione per inizializzare il bus I2C e la scheda PWM
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
    // Impostiamo la frequenza a 50Hz per i servo standard
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

// Funzione per inizializzare la Radio LoRa SX1262
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

    // Impostiamo il Sync Word completo con i bit di controllo
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

    // Gestione errori 
    if (state != RADIOLIB_ERR_NONE) {
        radioStatus = "Radio ERR: " + String(state);
        Serial.printf("ERRORE Radio: %d\n", state);
    }

    //updateDisplay(txCount, radioStatus, lastRxMsg.c_str());
    pinMode(RADIO_DIO1, INPUT);
}

// Funzione per inizializzare il BLE e avviare l'advertising
void setupBLE() {
    // Inizializzazione BLE
    BLEDevice::init("LoRa_Web_Bridge");
    // Creiamo il server BLE e impostiamo i callback per connessione/disconnessione
    pServer = BLEDevice::createServer();
    // Impostiamo i callback per connessione/disconnessione
    pServer->setCallbacks(new MyServerCallbacks());
    // Creiamo il servizio BLE con il nostro UUID
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ |
                        BLECharacteristic::PROPERTY_WRITE |
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
    // Impostiamo i callback per la caratteristica TX
    pCharacteristicTX->setCallbacks(new MyCallbacks());
    // Aggiungiamo il descriptor BLE2902 per abilitare le notifiche
    pCharacteristicTX->addDescriptor(new BLE2902());
    // Avviamo il servizio BLE
    pService->start();
    // Avviamo l'advertising BLE
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    // Aggiungiamo il nostro UUID di servizio all'advertising
    pAdvertising->addServiceUUID(SERVICE_UUID);
    // Impostiamo l'advertising per includere il nome del dispositivo e il servizio
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] In attesa di connessione iOS...");
}

// Funzione per gestire i comandi ricevuti via BLE
String handleBleCommand(const String& command) {
    String trimmed = normalizeBleCommand(command);
    trimmed.trim();

    if (trimmed == "PING") {
        return "[BLE][ACK] PONG";
    }

    if (trimmed == "STATUS") {
        return "[BLE][ACK] STATUS | RSSI=" + String(radio.getRSSI()) + " dBm";
    }

    // Gestione comandi testuali di controllo remoto (singole parole)
    if (trimmed == "ARM" || trimmed == "DISARM" || trimmed == "HOVER" || trimmed == "EMERGENCY_STOP" ||
        trimmed == "THROTTLE_UP" || trimmed == "THROTTLE_DOWN" ||
        trimmed == "ROLL_LEFT" || trimmed == "ROLL_RIGHT" ||
        trimmed == "PITCH_UP" || trimmed == "PITCH_DOWN" ||
        trimmed == "YAW_LEFT" || trimmed == "YAW_RIGHT") {
        String ack = processRemoteCommand(trimmed);
        // Aggiorna display e stato locale
        lastRxMsg = trimmed + " -> " + ack;
        radioStatus = "BLE CMD";
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        return "[BLE][ACK] " + trimmed;
    }

    int separator = trimmed.indexOf('=');
    if (separator <= 0) {
        return "[BLE][ERR] Comando non valido: " + trimmed;
    }
    // Split key=value
    String key = trimmed.substring(0, separator);
    // Split value after '='
    String value = trimmed.substring(separator + 1);
    key.trim();
    value.trim();
    // Delegate key=value handling to a shared function so LoRa and BLE use the same logic
    return processKeyValueCommand(key, value, /*via=*/"BLE");
}

// Shared handler for key=value control commands. 'via' used for logging/context ("BLE" or "LORA").
String processKeyValueCommand(const String& keyIn, const String& valueIn, const String& via) {
    String key = keyIn;
    String value = valueIn;
    key.trim();
    value.trim();

    if (key == "FREQ") {
        float freq = value.toFloat();
        if (freq > 0.0f) {
            radio.standby();
            delay(5);
            int state = radio.setFrequency(freq);
            if (state == RADIOLIB_ERR_NONE) {
                currentRadioFreq = String(freq, 3);
                updateDisplay(txCount, currentRadioFreq, via + " FREQ OK", bleIncomingMsg.c_str());
                radio.startReceive();
                return "[" + via + "][ACK] FREQ=" + String(freq, 3);
            }
            return "[" + via + "][ERR] FREQ " + String(state);
        }
    }

    if (key == "POWER") {
        int power = value.toInt();
        int state = radio.setOutputPower(power);
        if (state == RADIOLIB_ERR_NONE) {
            return "[" + via + "][ACK] POWER=" + String(power);
        }
        return "[" + via + "][ERR] POWER " + String(state);
    }

    if (key == "THROTTLE") {
        long t = value.toInt();
        t = constrain(t, 0L, 255L);
        throttleValue = (int)t;
        setChannelFromInput(CH_THROTTLE, throttleValue);
        updateDisplay(txCount, currentRadioFreq, via + " THROTTLE", lastRxMsg.c_str());
        return "[" + via + "][ACK] THROTTLE=" + String(throttleValue);
    }

    if (key == "ROLL") {
        long r = value.toInt();
        r = constrain(r, 0L, 255L);
        rollValue = (int)r;
        moveChannelGradual(CH_ROLL, rollValue);
        updateDisplay(txCount, currentRadioFreq, via + " ROLL", lastRxMsg.c_str());
        return "[" + via + "][ACK] ROLL=" + String(rollValue);
    }

    if (key == "SF") {
        int sf = value.toInt();
        int state = radio.setSpreadingFactor(sf);
        if (state == RADIOLIB_ERR_NONE) {
            return "[" + via + "][ACK] SF=" + String(sf);
        }
        return "[" + via + "][ERR] SF " + String(state);
    }

    if (key == "BW") {
        float bw = value.toFloat();
        int state = radio.setBandwidth(bw);
        if (state == RADIOLIB_ERR_NONE) {
            return "[" + via + "][ACK] BW=" + String(bw, 0);
        }
        return "[" + via + "][ERR] BW " + String(state);
    }

    return "[" + via + "][ERR] Chiave sconosciuta: " + key;
}

// Normalize BLE command by removing optional prefixes like "TX=" or "BLE=" and trimming whitespace
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

// Funzione di setup principale
void setup() {
    // Inizializzazione Serial per debug
    Serial.begin(115200); 
    // Attendere che la seriale sia pronta
    setupBLE();
    // Init setup for PWM Servo - N°1 CH-0  
    setup4pwm();
    // Init LoRa
    setup4LoRa();
    // Aggiorna display iniziale
    Serial.println();
    Serial.println();
    setupDisplay();

}

// === GESTIONE SERVO BASATA SU MESSAGGIO RX ===

// Applica un valore 0-255 al servo (mappato a SERVOMIN..SERVOMAX)
void setServoFromInput(long servoValue) {
    // default maps to throttle channel
    setChannelFromInput(CH_THROTTLE, servoValue);
}

// Set channel immediately to mapped pulse value and update currentChannelPulse
void setChannelFromInput(uint8_t channel, long value) {
    if (!isWireStarted || !isPwmStarted || !isPwmResponding) return;
    uint16_t pulse = map(value, 0, 255, SERVOMIN, SERVOMAX);
    pulse = constrain(pulse, SERVOMIN, SERVOMAX);
    pwm.setPWM(channel, 0, pulse);
    currentChannelPulse[channel] = pulse;
    Serial.printf("[Servo] CH%d set: %ld -> pulse %d\n", channel, value, pulse);
}

// Gradually move a PWM channel from its current pulse to the target mapped from a 0-255 value
void moveChannelGradual(uint8_t channel, long value, int stepDelayMs, int stepSize) {
    if (!isWireStarted || !isPwmStarted || !isPwmResponding) return;
    uint16_t targetPulse = map(value, 0, 255, SERVOMIN, SERVOMAX);
    targetPulse = constrain(targetPulse, SERVOMIN, SERVOMAX);

    uint16_t &currentPulse = currentChannelPulse[channel];
    if (currentPulse == targetPulse) return;

    int step = (targetPulse > currentPulse) ? stepSize : -stepSize;
    while (currentPulse != targetPulse) {
        int next = (int)currentPulse + step;
        // avoid overshoot
        if ((step > 0 && next > targetPulse) || (step < 0 && next < (int)targetPulse)) next = targetPulse;
        currentPulse = (uint16_t)next;
        pwm.setPWM(channel, 0, currentPulse);
        delay(stepDelayMs);
    }
    Serial.printf("[Servo] CH%d moved to pulse %d (value %ld)\n", channel, currentPulse, value);
}

// Esegui comandi di controllo remoto: aggiorna stati e attua effetti locali (servo/PWM)
String processRemoteCommand(const String& command) {
    String cmd = command;
    cmd.trim();

    if (cmd == "ARM") {
        armed = true;
        updateDisplay(txCount, currentRadioFreq, "ARMED", bleIncomingMsg.c_str());
        return "[ACK] ARM";
    }
    if (cmd == "DISARM") {
        armed = false;
        updateDisplay(txCount, currentRadioFreq, "DISARMED", bleIncomingMsg.c_str());
        return "[ACK] DISARM";
    }
    if (cmd == "HOVER") {
        // Placeholder: implement hover behavior as needed
        return "[ACK] HOVER";
    }
    if (cmd == "EMERGENCY_STOP") {
        armed = false;
        // Azzeriamo l'output PWM come misura di sicurezza
        if (isWireStarted && isPwmStarted && isPwmResponding) {
            pwm.setPWM(0, 0, SERVOMIN);
        }
        updateDisplay(txCount, currentRadioFreq, "EMERGENCY STOP", bleIncomingMsg.c_str());
        return "[ACK] EMERGENCY_STOP";
    }

    if (cmd == "THROTTLE_UP") {
        throttleValue = min(255, throttleValue + 10);
        setChannelFromInput(CH_THROTTLE, throttleValue);
        return "[ACK] THROTTLE_UP";
    }
    if (cmd == "THROTTLE_DOWN") {
        throttleValue = max(0, throttleValue - 10);
        setChannelFromInput(CH_THROTTLE, throttleValue);
        return "[ACK] THROTTLE_DOWN";
    }

    // Roll control: left decreases, right increases
    if (cmd == "ROLL_LEFT") {
        rollValue = max(0, rollValue - 10);
        moveChannelGradual(CH_ROLL, rollValue);
        return "[ACK] ROLL_LEFT";
    }
    if (cmd == "ROLL_RIGHT") {
        rollValue = min(255, rollValue + 10);
        moveChannelGradual(CH_ROLL, rollValue);
        return "[ACK] ROLL_RIGHT";
    }

    // Pitch control
    if (cmd == "PITCH_UP") {
        pitchValue = min(255, pitchValue + 10);
        setChannelFromInput(CH_PITCH, pitchValue);
        return "[ACK] PITCH_UP";
    }
    if (cmd == "PITCH_DOWN") {
        pitchValue = max(0, pitchValue - 10);
        setChannelFromInput(CH_PITCH, pitchValue);
        return "[ACK] PITCH_DOWN";
    }

    // Yaw control
    if (cmd == "YAW_LEFT") {
        yawValue = max(0, yawValue - 10);
        setChannelFromInput(CH_YAW, yawValue);
        return "[ACK] YAW_LEFT";
    }
    if (cmd == "YAW_RIGHT") {
        yawValue = min(255, yawValue + 10);
        setChannelFromInput(CH_YAW, yawValue);
        return "[ACK] YAW_RIGHT";
    }

    return "[ACK] UNKNOWN";
}

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

    // Remote control textual commands (ARM/DISARM/HOVER/EMERGENCY/THROTTLE etc)
    if (rxData == "ARM" || rxData == "DISARM" || rxData == "HOVER" || rxData == "EMERGENCY_STOP" ||
        rxData == "THROTTLE_UP" || rxData == "THROTTLE_DOWN" ||
        rxData == "ROLL_LEFT" || rxData == "ROLL_RIGHT" ||
        rxData == "PITCH_UP" || rxData == "PITCH_DOWN" ||
        rxData == "YAW_LEFT" || rxData == "YAW_RIGHT") {
        String ack = processRemoteCommand(rxData);
        txCount++;
        radio.transmit(ack);
        Serial.println("[Radio] Remote cmd processed: " + rxData + " -> " + ack);
        return;
    }

    // Gestione comandi numerici per il Servo
    // Prova a parsare il messaggio come numero (0-255 o 0-1023)
    // If message contains '=' treat it as key=value and delegate to shared handler
    int sep = rxData.indexOf('=');
    if (sep > 0) {
        String key = rxData.substring(0, sep);
        String value = rxData.substring(sep + 1);
        key.trim(); value.trim();
        String resp = processKeyValueCommand(key, value, /*via=*/"LORA");
        txCount++;
        radio.transmit(resp);
        Serial.println("[Radio] Key=Value processed -> " + resp);
        return;
    }

    if(isWireStarted && isPwmStarted && isPwmResponding){
        long servoValue = rxData.toInt();
        if ((servoValue != 0 || rxData == "0")) {
            // Move throttle channel smoothly to requested value
            moveChannelGradual(CH_THROTTLE, servoValue);

            Serial.printf("[Servo] Posizione: %d (pulselen: %d)\n", servoValue, currentChannelPulse[CH_THROTTLE]);

            // === INVIO FEEDBACK AL GATEWAY ===
            txCount++; // Incrementiamo il contatore trasmissioni dell'ESP
            String feedback = "[ACK] Servo impostato a " + String(servoValue);
            
            // Trasmettiamo la conferma
            radio.transmit(feedback);
            
            Serial.println("[Radio] Feedback inviato al Gateway");
        }
    }
}

// === GESTIONE LOOP PRINCIPALE ===
void RX_Manager(uint32_t &lastDisplayUpdate){
    // 1. Gestione Ricezione (RX) - Interroga la radio solo se il pin DIO1 è alto
    if (digitalRead(RADIO_DIO1) == HIGH) {
        String rxData;
        int rxState = radio.readData(rxData);

        if (rxState == RADIOLIB_ERR_NONE) {
            rxCount++;
            updateDisplay(txCount, currentRadioFreq, "PONG SENT", lastRxMsg.c_str());
            radioStatus = "RX " + String((int)radio.getRSSI()) + "dBm";

            Serial.printf("[Radio] Ricevuto: %s | RSSI: %.2f | SNR: %.2f\n", 
                          rxData.c_str(), radio.getRSSI(), radio.getSNR());

            // Rsponde in base al messaggio ricevuto
            rxMsgParserAndResponse(rxData);

            // AGGIORNAMENTO IMMEDIATO del display
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
        
        // Fondamentale: riattiva la ricezione dopo ogni evento su DIO1
        radio.startReceive();
    }
}

// Gestione trasmissione periodica (ogni 5 secondi)
void TX_Manager(uint32_t lastTx){
    // 2. Gestione Radio TX (ogni 5 secondi)
    if (millis() - lastTx >= 5000) {
        lastTx = millis();
        
        radioStatus = "TX...";
        // Mostra l'ultimo messaggio RX anche durante TX
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        
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

        // Mostra l'ultimo messaggio RX anche dopo TX
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
        
        // Torna in modalità ricezione dopo la trasmissione
        radio.startReceive();
    }
}

// Aggiornamento periodico del display (ogni secondo)
void updateDataMonitor(uint32_t &lastDisplayUpdate){
    // 3. Aggiornamento periodico display (ogni secondo)
    if (millis() - lastDisplayUpdate >= 1000) {
        lastDisplayUpdate = millis();
        updateDisplay(txCount, currentRadioFreq, radioStatus, lastRxMsg.c_str());
    }
}

// === LOOP PRINCIPALE ===
void loop() {
    static uint32_t lastDisplayUpdate = 0;
    static uint32_t lastTx = 0;
    // Gestione Ricezione (RX) - Interroga la radio solo se il pin DIO1 è alto
    RX_Manager(lastDisplayUpdate);
    //TX_Manager(lastTx);

    // Aggiornamento periodico del display (ogni secondo)
    updateDataMonitor(lastDisplayUpdate);
    
}
