// common_types.h - Shared data types across all radar subsystems
// Defines the standardized target data format used for inter-module communication.
#pragma once

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

namespace rm_radar {

// 3D vector in world/sensor coordinate system
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float norm() const { return std::sqrt(x * x + y * y + z * z); }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};

// 2D vector for planar operations (ground plane mapping)
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    float norm() const { return std::sqrt(x * x + y * y); }
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
};

// Target category
enum class TargetType : uint8_t {
    UNKNOWN = 0,
    INFANTRY,      // 步兵
    HERO,          // 英雄
    ENGINEER,      // 工程
    SENTRY,        // 哨兵
    DRONE,         // 无人机
    STATIC_OBSTACLE
};

// Standardized target output format (unified across all subsystems)
struct Target {
    uint32_t id = 0;                 // Unique target ID
    TargetType type = TargetType::UNKNOWN;
    Vec3 position;                   // World coordinate (x, y, z) in meters
    Vec3 velocity;                   // Velocity (m/s)
    float confidence = 0.0f;         // Detection confidence [0, 1]
    uint64_t timestamp_us = 0;       // Hardware timestamp in microseconds
    uint32_t point_count = 0;        // Point cloud size (lidar) / bounding box pixels (vision)
    float bbox[6] = {0};             // 3D bounding box: [x_min,y_min,z_min,x_max,y_max,z_max]

    bool valid() const { return confidence > 0.0f; }
};

// Sensor source identifier for fusion
enum class SensorSource : uint8_t {
    LIDAR = 0,
    VISION,
    SDR,
    MONOCULAR,
    FUSED
};

// Timing helper
inline uint64_t now_us() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

} // namespace rm_radar
