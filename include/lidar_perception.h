// lidar_perception.h - Full LiDAR perception pipeline
// Pipeline: raw cloud -> passthrough -> voxel downsample -> RANSAC ground removal
//           -> octree euclidean clustering -> Kalman tracking -> structured targets
#pragma once

#include "common_types.h"
#include "config_loader.h"
#include "logger.h"
#include "math_utils.h"
#include "point_cloud.h"

#include <string>
#include <vector>
#include <memory>
#include <cstdint>

namespace rm_radar {
namespace lidar {

// Configuration parameters loaded from YAML
struct LidarConfig {
    // passthrough
    float x_min = -8.0f, x_max = 8.0f;
    float y_min = -15.0f, y_max = 15.0f;
    float z_min = 0.1f, z_max = 2.0f;
    // voxel
    float voxel_leaf = 0.05f;
    float voxel_leaf_far = 0.08f;
    float far_dist_threshold = 15.0f;
    // ground
    float ground_dist_thresh = 0.08f;
    int ground_max_iter = 100;
    float ground_optimize_z_below = 0.5f;
    // clustering
    float cluster_base_radius = 0.35f;
    float cluster_radius_gain = 0.012f;
    int cluster_min_size = 15;
    int cluster_max_size = 800;
    // tracking
    float max_match_distance = 0.5f;
    int max_lost_frames = 3;
    // static obstacles
    bool static_filter_enabled = true;
    std::vector<StaticObstacle> static_obstacles;
    // multi-radar
    bool multi_radar_enabled = false;
    std::vector<std::array<float, 16>> extrinsics;
    // output
    bool publish_udp = false;
    std::string udp_host = "127.0.0.1";
    int udp_port = 8800;
    // visualization
    bool visualization_enabled = true;
    std::string window_name = "LiDAR Perception";
    int point_size = 2;
};

// Tracked target maintained by the tracker
struct TrackedTarget {
    uint32_t id = 0;
    Vec3 position;
    Vec3 velocity;
    BoundingBox3D bbox;
    uint32_t point_count = 0;
    float avg_intensity = 0.0f;
    float confidence = 0.0f;
    int lost_frames = 0;      // consecutive frames without association
    bool active = true;
    math::KalmanFilter3D kf;  // per-track constant-velocity Kalman filter
};

// Result of one frame of processing
struct PerceptionResult {
    std::vector<Target> targets;   // standardized output targets
    std::vector<Cluster> clusters; // raw clusters (for debug)
    PointCloud filtered_cloud;     // post-filter cloud (for visualization)
    uint64_t timestamp_us = 0;
    float processing_time_ms = 0.0f;
};

class LidarPerception {
public:
    LidarPerception();
    ~LidarPerception();

    // Load configuration from YAML file
    bool loadConfig(const std::string& yaml_path);

    // Initialize the pipeline (must be called after loadConfig)
    bool init();

    // Process a single point cloud frame, return detected targets
    PerceptionResult process(const PointCloud& input);

    // Set static obstacle templates externally
    void setStaticObstacles(const std::vector<StaticObstacle>& obs);

    // Set multi-radar extrinsic matrices (row-major 4x4)
    void setExtrinsics(const std::vector<std::array<float, 16>>& ext);

    // Get current configuration
    const LidarConfig& config() const { return cfg_; }

    // Get active tracked targets
    const std::vector<TrackedTarget>& tracks() const { return tracks_; }

private:
    // ---- Pipeline stages ----
    PointCloud passthrough(const PointCloud& in);
    PointCloud voxelDownsample(const PointCloud& in);
    PointCloud removeGround(const PointCloud& in);
    std::vector<Cluster> cluster(const PointCloud& in);
    void track(const std::vector<Cluster>& clusters, std::vector<Target>& out_targets);

    // ---- Helpers ----
    bool isStaticObstacle(const Vec3& pos) const;
    void updateConfigFromFile();
    BoundingBox3D computeBBox(const std::vector<PointXYZIR>& pts) const;

    LidarConfig cfg_;
    Config config_loader_;
    std::vector<TrackedTarget> tracks_;
    uint32_t next_track_id_ = 1;
    bool initialized_ = false;
};

} // namespace lidar
} // namespace rm_radar
