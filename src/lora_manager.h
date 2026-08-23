#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>
#include <RadioLib.h>
#include "pwm_manager.h"

#define RADIO_CS    8
#define RADIO_DIO1  14
#define RADIO_RST   12
#define RADIO_BUSY  13

#define FREQ_RTX  869.525
#define BW  125.0
#define SF  7
#define CR  8
#define POWER  5
#define PREAMBLE 32
#define SYNC_WORD  0x24
#define CTRL_BITS 0xB4

extern const char testPayload[];
extern Module* mod;
extern SX1262 radio;

extern uint32_t txCount;
extern uint32_t rxCount;
extern String radioStatus;
extern String lastRxMsg;
extern String currentRadioFreq;

extern uint32_t isWireStarted;
extern uint32_t isPwmStarted;
extern uint32_t isPwmResponding;
extern int throttleValue;
extern int rollValue;
extern int pitchValue;
extern int yawValue;

constexpr uint8_t CH_THROTTLE = 0;
constexpr uint8_t CH_ROLL = 1;
constexpr uint8_t CH_PITCH = 2;
constexpr uint8_t CH_YAW = 3;

void setup4LoRa();
void RX_Manager(uint32_t &lastDisplayUpdate);
void TX_Manager(uint32_t &lastTx);
void rxMsgParserAndResponse(String rxData);

String processRemoteCommand(const String& command);
String processKeyValueCommand(const String& keyIn, const String& valueIn, const String& via);
bool isLiveControlKey(const String& key);
bool isLiveRemoteTextCommand(const String& cmd);
void setChannelFast(uint8_t channel, long value);
void setChannelsFastPair(uint8_t channelA, long valueA, uint8_t channelB, long valueB);
void setChannelFromInput(uint8_t channel, long value);

#endif
