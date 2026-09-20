// sdr_protocol.h - RM official information wave protocol definitions
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

namespace rm_radar {
namespace sdr {

// Frame synchronization word
constexpr uint16_t SYNC_WORD = 0xAA55;
constexpr uint16_t SYNC_WORD_INV = 0x55AA; // inverted variant for noise immunity

// Protocol frame layout (byte-aligned, little-endian by default)
// | Sync(2) | Length(2) | RobotID(1) | X(4) | Y(4) | HP(2) | Ammo(2) | Status(2) | CRC16(2) |
#pragma pack(push, 1)
struct SdrFrame {
    uint16_t sync;        // 0xAA55
    uint16_t length;      // payload length in bytes (excluding sync+length+crc)
    uint8_t  robot_id;    // robot identifier
    float    x;           // world x coordinate (meters)
    float    y;           // world y coordinate (meters)
    uint16_t hp;          // hit points
    uint16_t ammo;        // ammunition count
    uint16_t status;      // status bitfield
    uint16_t crc16;       // CRC-16 checksum (polynomial 0x8005, init 0xFFFF)
};
#pragma pack(pop)

constexpr size_t SDR_FRAME_SIZE = sizeof(SdrFrame);
constexpr size_t SDR_PAYLOAD_SIZE = SDR_FRAME_SIZE - 6; // minus sync, length, crc

// Decoded target information
struct SdrTarget {
    uint8_t  robot_id = 0;
    float    x = 0.0f;
    float    y = 0.0f;
    uint16_t hp = 0;
    uint16_t ammo = 0;
    uint16_t status = 0;
    uint64_t timestamp_us = 0;
    float    confidence = 1.0f;
};

// CRC-16 (CCITT-like, poly 0x8005, init 0xFFFF)
inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// CRC of the frame payload (everything between length field and crc field)
inline uint16_t frameCrc(const SdrFrame& f) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&f) + 4; // skip sync+length
    return crc16(p, SDR_PAYLOAD_SIZE);
}

// Byte order conversion helpers
inline bool isLittleEndian() {
    uint16_t x = 1;
    return *reinterpret_cast<uint8_t*>(&x) == 1;
}

inline uint16_t bswap16(uint16_t v) {
    return (v >> 8) | (v << 8);
}

inline uint32_t bswap32(uint32_t v) {
    return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) |
           ((v >> 8) & 0xFF00) | ((v >> 24) & 0xFF);
}

inline float bswapFloat(float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    u = bswap32(u);
    std::memcpy(&v, &u, 4);
    return v;
}

} // namespace sdr
} // namespace rm_radar
