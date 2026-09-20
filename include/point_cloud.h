// point_cloud.h - LiDAR point cloud data structures for the perception subsystem
#pragma once

#include "common_types.h"
#include <vector>
#include <cstdint>
#include <string>

namespace rm_radar {
namespace lidar {

// Single LiDAR point with XYZ coordinates, intensity and laser ring id
struct PointXYZIR {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    uint16_t ring = 0;

    PointXYZIR() = default;
    PointXYZIR(float x_, float y_, float z_, float i_ = 0.0f, uint16_t r_ = 0)
        : x(x_), y(y_), z(z_), intensity(i_), ring(r_) {}
};

// Point cloud container with timestamp and coordinate frame id
struct PointCloud {
    std::vector<PointXYZIR> points;
    uint64_t timestamp_us = 0;
    std::string frame_id = "lidar";

    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    void clear() { points.clear(); }
    void push_back(const PointXYZIR& p) { points.push_back(p); }
    void reserve(size_t n) { points.reserve(n); }
};

// 3D axis-aligned bounding box
struct BoundingBox3D {
    Vec3 center;       // box center in world coordinates
    Vec3 size;         // length(x), width(y), height(z) in meters
    float yaw = 0.0f;  // orientation around z-axis (radians)

    // 8 corner points of the box
    std::array<Vec3, 8> corners() const {
        float lx = size.x * 0.5f, ly = size.y * 0.5f, lz = size.z * 0.5f;
        float c = std::cos(yaw), s = std::sin(yaw);
        std::array<Vec3, 8> out;
        int idx = 0;
        for (int sx : {-1, 1}) for (int sy : {-1, 1}) for (int sz : {-1, 1}) {
            float dx = sx * lx, dy = sy * ly;
            // rotate around z
            float rx = c * dx - s * dy;
            float ry = s * dx + c * dy;
            out[idx++] = Vec3(center.x + rx, center.y + ry, center.z + sz * lz);
        }
        return out;
    }
};

// Cluster descriptor output from the segmentation stage
struct Cluster {
    std::vector<PointXYZIR> points;
    Vec3 centroid;           // mean position of the cluster
    BoundingBox3D bbox;      // 3D bounding box
    uint32_t point_count = 0;
    float avg_intensity = 0.0f;
    float distance = 0.0f;   // distance from sensor origin
};

// Static obstacle template for filtering known fixed objects
struct StaticObstacle {
    Vec3 position;   // world position
    float radius;    // matching radius (meters)
    std::string label;
};

} // namespace lidar
} // namespace rm_radar
