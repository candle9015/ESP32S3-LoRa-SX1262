#include "gps_manager.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// UART1 sul pin RX=47, TX=48
#define GPS_RX_PIN 47
#define GPS_TX_PIN 48
#define GPS_BAUD 9600

static HardwareSerial gpsSerial(1);
static TinyGPSPlus gps;

static bool gpsReady = false;
static double latitude = 0.0;
static double longitude = 0.0;
static double altitude = 0.0;
static float speedKnots = 0.0f;
static float courseTrue = 0.0f;
static uint8_t satellites = 0;
static bool gpsFixed = false;
static uint32_t lastFixTime = 0;
static uint32_t lastGpsDebugMs = 0;

void setupGPS() {
    Serial.println("[GPS] Initializing GY-GPS6MV2 on UART1 (RX=47, TX=48, 9600 baud)");
    
    // Inizializza UART1
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    if (!gpsSerial) {
        Serial.println("[GPS] Error: UART1 initialization failed!");
        gpsReady = false;
        return;
    }

    delay(100);
    gpsReady = true;
    lastGpsDebugMs = millis();
    Serial.println("[GPS] Initialized successfully, waiting for satellite fix...");
}

void updateGPS() {
    if (!gpsReady) {
        return;
    }

    // Leggi tutti i dati disponibili dal GPS
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        if (gps.encode(c)) {
            // Una frase NMEA completa è stata parsata
            if (gps.location.isUpdated() && gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                lastFixTime = millis();
            }
            
            if (gps.altitude.isUpdated()) {
                altitude = gps.altitude.meters();
            }
            
            if (gps.speed.isUpdated()) {
                speedKnots = gps.speed.knots();
            }
            
            if (gps.course.isUpdated()) {
                courseTrue = gps.course.deg();
            }
            
            if (gps.satellites.isUpdated()) {
                satellites = gps.satellites.value();
            }
        }
    }

    // TinyGPS mantiene il fix valido finche il dato non diventa obsoleto.
    gpsFixed = gps.location.isValid() && gps.location.age() < 10000;

    // Debug log periodico
    uint32_t now = millis();
    if (now - lastGpsDebugMs >= 5000) {
        lastGpsDebugMs = now;
        const uint32_t charsProcessed = gps.charsProcessed();
        const uint32_t passedChecksums = gps.passedChecksum();
        const uint32_t failedChecksums = gps.failedChecksum();
        const int bytesAvailable = gpsSerial.available();
        if (gpsFixed) {
            Serial.printf("[GPS] FIX - Lat: %.6f, Lon: %.6f, Alt: %.1f m, Sats: %u, Speed: %.1f kts\n",
                         latitude, longitude, altitude, satellites, speedKnots);
        } else if (charsProcessed == 0 && bytesAvailable == 0) {
            Serial.printf("[GPS] NO DATA - UART1 RX GPIO %d, TX GPIO %d, baud %d. Verifica alimentazione e collega GPS TX -> ESP32 GPIO %d. (available=%d)\n",
                          GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD, GPS_RX_PIN, bytesAvailable);
        } else if (passedChecksums == 0 && failedChecksums > 0) {
            Serial.printf("[GPS] INVALID NMEA - No valid checksum. Possible baud mismatch (currently %d). Chars: %lu, Passed: %lu, Failed: %lu\n",
                         GPS_BAUD, charsProcessed, passedChecksums, failedChecksums);
        } else if (passedChecksums > 0) {
            Serial.printf("[GPS] NO FIX - Valid NMEA received, waiting for satellites. Chars: %lu, Valid sentences: %lu, With fix: %lu, Failed: %lu, Sats: %u\n",
                         charsProcessed, passedChecksums, gps.sentencesWithFix(), failedChecksums, satellites);
        } else {
            Serial.printf("[GPS] NO FIX - Waiting for NMEA data. Chars: %lu, Valid sentences: %lu, Failed: %lu, Sats: %u, Available: %d\n",
                         charsProcessed, passedChecksums, failedChecksums, satellites, bytesAvailable);
        }
    }
}

bool isGPSReady() {
    return gpsReady;
}

bool isGPSFixed() {
    return gpsFixed && (millis() - lastFixTime < 10000); // fix valido se ricevuto negli ultimi 10s
}

GpsTelemetry getGPSState() {
    return {
        .latitude = latitude,
        .longitude = longitude,
        .altitude = altitude,
        .speedKnots = speedKnots,
        .courseTrue = courseTrue,
        .satellites = satellites,
        .valid = isGPSFixed(),
        .lastFixTime = lastFixTime
    };
}

String buildGpsTelemetryPayload(const String& prefix) {
    // Formato: GPS|lat|lon|alt|speed|course|sats|valid
    String payload = prefix;
    payload += "|";
    
    if (isGPSFixed()) {
        payload += String(latitude, 6);
        payload += "|";
        payload += String(longitude, 6);
        payload += "|";
        payload += String(altitude, 1);
        payload += "|";
        payload += String(speedKnots, 1);
        payload += "|";
        payload += String(courseTrue, 1);
        payload += "|";
        payload += String(satellites);
        payload += "|1";
    } else {
        payload += "0|0|0|0|0|";
        payload += String(satellites);
        payload += "|0";
    }
    
    return payload;
}

String getGPSStatusString() {
    if (!gpsReady) {
        return "GPS not initialized";
    }
    
    if (isGPSFixed()) {
        return String("[FIXED] Lat: ") + String(latitude, 6) + 
               ", Lon: " + String(longitude, 6) + 
               ", Alt: " + String(altitude, 1) + "m" +
               ", Sats: " + String(satellites);
    } else {
        return String("[NO FIX] Searching... Sats acquired: ") + String(satellites);
    }
}
