#include "display_manager.h"
#include "images.h"

// Istanza del display (statica per questo file)
static SSD1306Wire display(0x3c, 500000, OLED_SDA, OLED_SCL, GEOMETRY_128_64, OLED_RST);

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void setupDisplay() {
    VextON();
    delay(100);
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.display();
}

void updateDisplay(uint32_t txCount, const String& radioStatus, const char* lastRxMsg) {
    display.clear();

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "HELTEC LORA32 V3");
    display.drawHorizontalLine(0, 12, 128);
    
    display.drawString(0, 16, "Freq: 869.525 MHz");
    display.drawString(0, 28, "Status: " + radioStatus);
    
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 40, "Sent: " + String(txCount));

    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 52, String(lastRxMsg));

    display.display();
}