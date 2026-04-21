#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include "HT_SSD1306Wire.h"

// Pin specifici per il display OLED Heltec V3
#define OLED_RST   21
#define OLED_SDA   17
#define OLED_SCL   18

void setupDisplay();
void updateDisplay(uint32_t txCount, const String& radioStatus, const char* testPayload);

#endif