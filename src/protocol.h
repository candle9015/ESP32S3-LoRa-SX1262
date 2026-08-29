#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

namespace compact_protocol {

constexpr char kControlPrefix = 'C';
constexpr char kTelemetryPrefix = 'T';
constexpr char kGpsPrefix = 'G';

String encodeControlFrame(uint8_t seq,
                          uint8_t flags,
                          uint8_t throttle,
                          uint8_t roll,
                          uint8_t pitch,
                          uint8_t yaw);

String encodeTelemetryFrame(uint8_t seq,
                            uint8_t flags,
                            uint8_t throttle,
                            int16_t roll,
                            int16_t pitch,
                            int16_t yaw,
                            int16_t tempC10,
                            int8_t rssi,
                            uint8_t battery);

            String encodeGpsFrame(double latitude,
                            double longitude,
                            double altitude,
                            float speedKnots,
                            float course,
                            uint8_t satellites,
                            bool valid);

bool parseControlFrame(const String& frame,
                       uint8_t& seq,
                       uint8_t& flags,
                       uint8_t& throttle,
                       uint8_t& roll,
                       uint8_t& pitch,
                       uint8_t& yaw,
                       uint8_t& crc);

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
                         uint8_t& crc);

uint8_t crc8FromValues(const uint8_t values[], uint8_t count);

}  // namespace compact_protocol

#endif
