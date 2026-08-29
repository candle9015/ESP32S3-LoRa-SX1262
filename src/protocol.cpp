#include "protocol.h"

namespace compact_protocol {

uint8_t crc8FromValues(const uint8_t values[], uint8_t count) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < count; ++i) {
        crc ^= values[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

String encodeControlFrame(uint8_t seq,
                          uint8_t flags,
                          uint8_t throttle,
                          uint8_t roll,
                          uint8_t pitch,
                          uint8_t yaw) {
    uint8_t values[6] = { seq, flags, throttle, roll, pitch, yaw };
    uint8_t crc = crc8FromValues(values, 6);
    String out = "C|";
    out += String(seq);
    out += "|";
    out += String(flags);
    out += "|";
    out += String(throttle);
    out += "|";
    out += String(roll);
    out += "|";
    out += String(pitch);
    out += "|";
    out += String(yaw);
    out += "|";
    out += String(crc);
    return out;
}

String encodeTelemetryFrame(uint8_t seq,
                            uint8_t flags,
                            uint8_t throttle,
                            int16_t roll,
                            int16_t pitch,
                            int16_t yaw,
                            int16_t tempC10,
                            int8_t rssi,
                            uint8_t battery) {
    uint8_t values[8] = {
        seq,
        flags,
        throttle,
        static_cast<uint8_t>(roll & 0xFF),
        static_cast<uint8_t>(pitch & 0xFF),
        static_cast<uint8_t>(yaw & 0xFF),
        static_cast<uint8_t>(tempC10 & 0xFF),
        static_cast<uint8_t>(battery)
    };
    uint8_t crc = crc8FromValues(values, 8);

    String out = "T|";
    out += String(seq);
    out += "|";
    out += String(flags);
    out += "|";
    out += String(throttle);
    out += "|";
    out += String((int)roll);
    out += "|";
    out += String((int)pitch);
    out += "|";
    out += String((int)yaw);
    out += "|";
    out += String((int)tempC10);
    out += "|";
    out += String((int)rssi);
    out += "|";
    out += String((int)battery);
    out += "|";
    out += String(crc);
    return out;
}

String encodeGpsFrame(double latitude,
                      double longitude,
                      double altitude,
                      float speedKnots,
                      float course,
                      uint8_t satellites,
                      bool valid) {
    String out = "G|";
    out += String(latitude, 6);
    out += "|";
    out += String(longitude, 6);
    out += "|";
    out += String(altitude, 1);
    out += "|";
    out += String(speedKnots, 1);
    out += "|";
    out += String(course, 1);
    out += "|";
    out += String(satellites);
    out += "|";
    out += valid ? "1" : "0";
    return out;
}

bool parseControlFrame(const String& frame,
                       uint8_t& seq,
                       uint8_t& flags,
                       uint8_t& throttle,
                       uint8_t& roll,
                       uint8_t& pitch,
                       uint8_t& yaw,
                       uint8_t& crc) {
    if (frame.length() == 0) {
        return false;
    }

    String payload = frame;
    int start = payload.indexOf('|');
    if (start < 0) {
        return false;
    }

    String prefix = payload.substring(0, start);
    if (prefix != "C") {
        return false;
    }

    payload = payload.substring(start + 1);
    String parts[7];
    for (int i = 0; i < 7; ++i) {
        int end = payload.indexOf('|');
        if (end < 0) {
            if (i == 6) {
                parts[i] = payload;
            } else {
                return false;
            }
        } else {
            parts[i] = payload.substring(0, end);
            payload = payload.substring(end + 1);
        }
    }

    if (parts[0].length() == 0 || parts[1].length() == 0 || parts[2].length() == 0 ||
        parts[3].length() == 0 || parts[4].length() == 0 || parts[5].length() == 0 ||
        parts[6].length() == 0) {
        return false;
    }

    seq = static_cast<uint8_t>(parts[0].toInt());
    flags = static_cast<uint8_t>(parts[1].toInt());
    throttle = static_cast<uint8_t>(parts[2].toInt());
    roll = static_cast<uint8_t>(parts[3].toInt());
    pitch = static_cast<uint8_t>(parts[4].toInt());
    yaw = static_cast<uint8_t>(parts[5].toInt());
    crc = static_cast<uint8_t>(parts[6].toInt());

    uint8_t expectedCrc = crc8FromValues(new uint8_t[6]{ seq, flags, throttle, roll, pitch, yaw }, 6);
    return crc == expectedCrc;
}

bool parseTelemetryFrame(const String& frame,
                         uint8_t& seq,
                         uint8_t& flags,
                         uint8_t& throttle,
                         int16_t& roll,
                         int16_t& pitch,
                         int16_t& yaw,
                         int16_t& tempC10,
                         int8_t& rssi,
                         uint8_t& battery,
                         uint8_t& crc) {
    if (frame.length() == 0) {
        return false;
    }

    String payload = frame;
    int start = payload.indexOf('|');
    if (start < 0) {
        return false;
    }

    String prefix = payload.substring(0, start);
    if (prefix != "T") {
        return false;
    }

    payload = payload.substring(start + 1);
    String parts[10];
    for (int i = 0; i < 10; ++i) {
        int end = payload.indexOf('|');
        if (end < 0) {
            if (i == 9) {
                parts[i] = payload;
            } else {
                return false;
            }
        } else {
            parts[i] = payload.substring(0, end);
            payload = payload.substring(end + 1);
        }
    }

    if (parts[0].length() == 0 || parts[1].length() == 0 || parts[2].length() == 0 ||
        parts[3].length() == 0 || parts[4].length() == 0 || parts[5].length() == 0 ||
        parts[6].length() == 0 || parts[7].length() == 0 || parts[8].length() == 0 ||
        parts[9].length() == 0) {
        return false;
    }

    seq = static_cast<uint8_t>(parts[0].toInt());
    flags = static_cast<uint8_t>(parts[1].toInt());
    throttle = static_cast<uint8_t>(parts[2].toInt());
    roll = static_cast<int16_t>(parts[3].toInt());
    pitch = static_cast<int16_t>(parts[4].toInt());
    yaw = static_cast<int16_t>(parts[5].toInt());
    tempC10 = static_cast<int16_t>(parts[6].toInt());
    rssi = static_cast<int8_t>(parts[7].toInt());
    battery = static_cast<uint8_t>(parts[8].toInt());
    crc = static_cast<uint8_t>(parts[9].toInt());
    return true;
}

}  // namespace compact_protocol
