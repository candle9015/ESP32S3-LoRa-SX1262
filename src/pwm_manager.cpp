#include "pwm_manager.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);
uint32_t isWireStarted = 1;
uint32_t isPwmStarted = 1;
uint32_t isPwmResponding = 1;
uint16_t currentChannelPulse[4] = { (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2, (SERVOMIN + SERVOMAX) / 2 };

bool armed = false;
int throttleValue = 0;
int rollValue = 128;
int pitchValue = 128;
int yawValue = 128;

void setup4pwm() {
    Serial.println("--- RESET BUS I2C ---");
    Wire1.end();

    bool status = Wire1.begin(PWM_SDA, PWM_SCL, 100000);
    if (!status) {
        Serial.println("ERRORE: Impossibile inizializzare Wire sui pin 42/46");
        isWireStarted = 0;
    }

    pwm.begin();
    pwm.setPWMFreq(50);

    if (!pwm.begin()) {
        Serial.println("Scheda PWM NON trovata. Controlla i pin 42 e 46.");
        isPwmStarted = 0;
    } else {
        Serial.println("Scheda PWM trovata!");
        pwm.setPWMFreq(50);
    }

    Wire1.beginTransmission(0x40);
    if (Wire1.endTransmission() == 0) {
        Serial.println("SUCCESSO: Scheda PWM trovata a 0x40!");
    } else {
        Serial.println("CRITICO: Il chip a 0x40 non risponde ancora.");
        isPwmResponding = 0;
    }
}

void setServoFromInput(long servoValue) {
    setChannelFromInput(CH_THROTTLE, servoValue);
}

void setDefaultDevice(long yaw, long roll, long pitch, long throttle) {
    yawValue = constrain(yaw, 0L, 255L);
    rollValue = constrain(roll, 0L, 255L);
    pitchValue = constrain(pitch, 0L, 255L);
    throttleValue = constrain(throttle, 0L, 255L);

    setChannelFast(CH_YAW, yawValue);
    setChannelFast(CH_ROLL, rollValue);
    setChannelFast(CH_PITCH, pitchValue);
    setChannelFast(CH_THROTTLE, throttleValue);
}

void setChannelFast(uint8_t channel, long value) {
    if (!isWireStarted || !isPwmStarted || !isPwmResponding) return;
    uint16_t pulse = map(value, 0, 255, SERVOMIN, SERVOMAX);
    pulse = constrain(pulse, SERVOMIN, SERVOMAX);
    pwm.setPWM(channel, 0, pulse);
    currentChannelPulse[channel] = pulse;
}

void setChannelsFastPair(uint8_t channelA, long valueA, uint8_t channelB, long valueB) {
    setChannelFast(channelA, valueA);
    setChannelFast(channelB, valueB);
}

void setChannelFromInput(uint8_t channel, long value) {
    setChannelFast(channel, value);
}

void moveChannelsGradualBatch(uint8_t channelA, long valueA, uint8_t channelB, long valueB, int stepDelayMs, int stepSize) {
    if (!isWireStarted || !isPwmStarted || !isPwmResponding) return;

    uint16_t targetPulseA = map(valueA, 0, 255, SERVOMIN, SERVOMAX);
    uint16_t targetPulseB = map(valueB, 0, 255, SERVOMIN, SERVOMAX);
    targetPulseA = constrain(targetPulseA, SERVOMIN, SERVOMAX);
    targetPulseB = constrain(targetPulseB, SERVOMIN, SERVOMAX);

    uint16_t &currentPulseA = currentChannelPulse[channelA];
    uint16_t &currentPulseB = currentChannelPulse[channelB];

    int stepA = (targetPulseA > currentPulseA) ? stepSize : ((targetPulseA < currentPulseA) ? -stepSize : 0);
    int stepB = (targetPulseB > currentPulseB) ? stepSize : ((targetPulseB < currentPulseB) ? -stepSize : 0);

    while (currentPulseA != targetPulseA || currentPulseB != targetPulseB) {
        if (currentPulseA != targetPulseA) {
            int nextA = (int)currentPulseA + stepA;
            if ((stepA > 0 && nextA > targetPulseA) || (stepA < 0 && nextA < (int)targetPulseA)) {
                nextA = targetPulseA;
            }
            currentPulseA = (uint16_t)nextA;
        }

        if (currentPulseB != targetPulseB) {
            int nextB = (int)currentPulseB + stepB;
            if ((stepB > 0 && nextB > targetPulseB) || (stepB < 0 && nextB < (int)targetPulseB)) {
                nextB = targetPulseB;
            }
            currentPulseB = (uint16_t)nextB;
        }

        pwm.setPWM(channelA, 0, currentPulseA);
        pwm.setPWM(channelB, 0, currentPulseB);
        delay(stepDelayMs);
    }

    Serial.printf("[Servo] batch CH%d/CH%d moved to pulses %d/%d (values %ld/%ld)\n",
                  channelA, channelB, currentPulseA, currentPulseB, valueA, valueB);
}

void moveChannelGradual(uint8_t channel, long value, int stepDelayMs, int stepSize) {
    if (!isWireStarted || !isPwmStarted || !isPwmResponding) return;
    uint16_t targetPulse = map(value, 0, 255, SERVOMIN, SERVOMAX);
    targetPulse = constrain(targetPulse, SERVOMIN, SERVOMAX);

    uint16_t &currentPulse = currentChannelPulse[channel];
    if (currentPulse == targetPulse) return;

    int step = (targetPulse > currentPulse) ? stepSize : -stepSize;
    while (currentPulse != targetPulse) {
        int next = (int)currentPulse + step;
        if ((step > 0 && next > targetPulse) || (step < 0 && next < (int)targetPulse)) next = targetPulse;
        currentPulse = (uint16_t)next;
        pwm.setPWM(channel, 0, currentPulse);
        delay(stepDelayMs);
    }
    Serial.printf("[Servo] CH%d moved to pulse %d (value %ld)\n", channel, currentPulse, value);
}
