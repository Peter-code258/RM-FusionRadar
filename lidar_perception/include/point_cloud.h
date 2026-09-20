// point_cloud.h - LiDAR point cloud data structure
#pragma once

#include "common_types.h"
#include <vector>
#include <cstdint>

namespace rm_radar {
namespace lidar {

// Single LiDAR point
struct PointXYZIR {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    uint16_t ring = 0;  // laser ring id

    PointXYZIR() = default;
    PointXYZIR(float x_, float y_, float z_, float i_ = 0, uint16_t r_ = 0)
        : x(x_), y(y_), z(z_), intensity(i_), ring(r_) {}
};

// Point cloud container
struct PointCloud {
    std::vector<PointXYZIR> points;
    uint64_t timestamp_us = 0;
    std::string frame_id = "lidar";

    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    void clear() { points.clear(); }
    void push_back(const PointXYZIR& p) { points.push_back(p); }
};

// 3D bounding box
struct BoundingBox {
    Vec3 center;
    Vec3 size;  // length, width, height
    float yaw = 0.0f;
};

} // namespace lidar
} // namespace rm_radar
