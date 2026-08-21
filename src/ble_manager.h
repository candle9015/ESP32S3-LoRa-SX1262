#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a9"

extern BLEServer* pServer;
extern BLECharacteristic* pCharacteristicTX;
extern BLECharacteristic* pCharacteristicRX;
extern bool deviceConnected;
extern String bleIncomingMsg;
extern String bleOutgoingMsg;

void setupBLE();
String handleBleCommand(const String& command);
String normalizeBleCommand(const String& command);

#endif
