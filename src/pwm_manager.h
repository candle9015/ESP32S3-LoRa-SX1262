#ifndef PWM_MANAGER_H
#define PWM_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "lora_manager.h"

#define PWM_SDA  42
#define PWM_SCL  46

#define SERVOMIN  150
#define SERVOMAX  600

extern Adafruit_PWMServoDriver pwm;
extern uint32_t isWireStarted;
extern uint32_t isPwmStarted;
extern uint32_t isPwmResponding;
extern uint16_t currentChannelPulse[4];

extern bool armed;
extern int throttleValue;
extern int rollValue;
extern int pitchValue;
extern int yawValue;

void setup4pwm();
void setServoFromInput(long servoValue);
void setDefaultDevice(long yaw, long roll, long pitch, long throttle);
void setChannelFast(uint8_t channel, long value);
void setChannelsFastPair(uint8_t channelA, long valueA, uint8_t channelB, long valueB);
void setChannelFromInput(uint8_t channel, long value);
void moveChannelsGradualBatch(uint8_t channelA, long valueA, uint8_t channelB, long valueB, int stepDelayMs = 20, int stepSize = 5);
void moveChannelGradual(uint8_t channel, long value, int stepDelayMs = 20, int stepSize = 5);

#endif
