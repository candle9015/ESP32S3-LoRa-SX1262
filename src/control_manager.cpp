#include "control_manager.h"
#include "lora_manager.h"
#include "ble_manager.h"
#include "pwm_manager.h"
#include "display_manager.h"

bool isLiveControlKey(const String& key) {
    return key == "THROTTLE" || key == "AXIS" || key == "ROLL" || key == "PITCH" || key == "YAW";
}

bool isLiveRemoteTextCommand(const String& cmd) {
    return cmd == "THROTTLE_UP" || cmd == "THROTTLE_DOWN" ||
           cmd == "ROLL_LEFT" || cmd == "ROLL_RIGHT" ||
           cmd == "PITCH_UP" || cmd == "PITCH_DOWN" ||
           cmd == "YAW_LEFT" || cmd == "YAW_RIGHT";
}

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
        return "[ACK] HOVER";
    }
    if (cmd == "EMERGENCY_STOP") {
        armed = false;
        yawValue = 128;
        setDefaultDevice(yawValue, rollValue, pitchValue, throttleValue);
        if (isWireStarted && isPwmStarted && isPwmResponding) {
            pwm.setPWM(0, 0, SERVOMIN);
            pwm.setPWM(1, 0, SERVOMIN);
            pwm.setPWM(2, 0, SERVOMIN);
            pwm.setPWM(3, 0, SERVOMIN);
        }
        updateDisplay(txCount, currentRadioFreq, "EMERGENCY STOP", bleIncomingMsg.c_str());
        return "[ACK] EMERGENCY_STOP";
    }

    if (cmd == "THROTTLE_UP") {
        throttleValue = min(255, throttleValue + 10);
        setChannelFromInput(CH_THROTTLE, throttleValue);
        return "";
    }
    if (cmd == "THROTTLE_DOWN") {
        throttleValue = max(0, throttleValue - 10);
        setChannelFromInput(CH_THROTTLE, throttleValue);
        return "";
    }

    if (cmd == "ROLL_LEFT") {
        rollValue = max(0, rollValue - 10);
        setChannelFast(CH_ROLL, rollValue);
        return "";
    }
    if (cmd == "ROLL_RIGHT") {
        rollValue = min(255, rollValue + 10);
        setChannelFast(CH_ROLL, rollValue);
        return "";
    }

    if (cmd == "PITCH_UP") {
        pitchValue = min(255, pitchValue + 10);
        setChannelFast(CH_PITCH, pitchValue);
        return "";
    }
    if (cmd == "PITCH_DOWN") {
        pitchValue = max(0, pitchValue - 10);
        setChannelFast(CH_PITCH, pitchValue);
        return "";
    }

    if (cmd == "YAW_LEFT") {
        yawValue = max(0, yawValue - 10);
        setChannelFast(CH_YAW, yawValue);
        return "";
    }
    if (cmd == "YAW_RIGHT") {
        yawValue = min(255, yawValue + 10);
        setChannelFast(CH_YAW, yawValue);
        return "";
    }

    return "[ACK] UNKNOWN";
}

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
        return "";
    }

    if (key == "AXIS") {
        int comma = value.indexOf(',');
        if (comma > 0) {
            long r = value.substring(0, comma).toInt();
            long p = value.substring(comma + 1).toInt();
            r = constrain(r, 0L, 255L);
            p = constrain(p, 0L, 255L);
            rollValue = (int)r;
            pitchValue = (int)p;
            setChannelsFastPair(CH_ROLL, rollValue, CH_PITCH, pitchValue);
            updateDisplay(txCount, currentRadioFreq, via + " AXIS", lastRxMsg.c_str());
            return "";
        }
        return "[" + via + "][ERR] AXIS formato non valido";
    }

    if (key == "ROLL") {
        long r = value.toInt();
        r = constrain(r, 0L, 255L);
        rollValue = (int)r;
        setChannelFast(CH_ROLL, rollValue);
        updateDisplay(txCount, currentRadioFreq, via + " ROLL", lastRxMsg.c_str());
        return "";
    }

    if (key == "PITCH") {
        long p = value.toInt();
        p = constrain(p, 0L, 255L);
        pitchValue = (int)p;
        setChannelFast(CH_PITCH, pitchValue);
        updateDisplay(txCount, currentRadioFreq, via + " PITCH", lastRxMsg.c_str());
        return "";
    }

    if (key == "YAW") {
        long y = value.toInt();
        y = constrain(y, 0L, 255L);
        yawValue = (int)y;
        setChannelFast(CH_YAW, yawValue);
        updateDisplay(txCount, currentRadioFreq, via + " YAW", lastRxMsg.c_str());
        return "";
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
