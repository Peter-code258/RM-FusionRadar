// anti_drone_tracker.h - UAV tracking and countermeasure subsystem
// Two-level architecture: wide-angle coarse tracking + telephoto/galvo fine tracking
#pragma once

#include "common_types.h"
#include "eskf.h"
#include "servo_controller.h"
#include "laser_controller.h"
#include "config_loader.h"
#include "logger.h"

#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <cstdint>

namespace rm_radar {
namespace anti_drone {

// Detection from wide-angle or telephoto camera
struct DroneDetection {
    float cx, cy;        // center pixel coordinates
    float w, h;          // bounding box size
    float confidence;
    bool is_telephoto;   // true if from telephoto/ROI camera
};

// Tracking stage
enum class TrackStage {
    COARSE = 0,   // wide-angle global search
    FINE = 1      // telephoto/galvo precision tracking
};

// Full tracking result
struct TrackingResult {
    uint32_t track_id = 0;
    Vec3 world_position;
    Vec3 world_velocity;
    Vec3 predicted_next;       // predicted position for next frame
    TrackStage stage = TrackStage::COARSE;
    float stability = 0.0f;    // tracking stability (0-1)
    bool laser_emitting = false;
    ServoCommand servo_cmd;
};

class AntiDroneTracker {
public:
    AntiDroneTracker();
    ~AntiDroneTracker();

    bool loadConfig(const std::string& yaml_path);
    bool init();

    // Process one frame of detections
    TrackingResult process(const std::vector<DroneDetection>& detections,
                            const cv::Mat& frame = cv::Mat());

    // Feed LiDAR range measurement for 3D fusion
    void setLidarRange(float range_m) { lidar_range_ = range_m; has_lidar_ = true; }

    // Feed laser spot detection for closed-loop correction
    void setSpotOffset(float offset_x, float offset_y) {
        servo_.applySpotCorrection(offset_x, offset_y);
    }

    // Optical axis calibration: set laser-camera boresight offset
    void setOpticalAxisOffset(float yaw_offset, float pitch_offset) {
        axis_offset_yaw_ = yaw_offset;
        axis_offset_pitch_ = pitch_offset;
    }

    // Get current tracking stage
    TrackStage stage() const { return stage_; }
    const ESKF& filter() const { return eskf_; }
    const LaserController& laser() const { return laser_; }

    // Convert pixel detection to world coordinate (using camera intrinsics + lidar range)
    Vec3 pixelToWorld(const DroneDetection& det, int img_w, int img_h) const;

    // Compute tracking stability from detection consistency
    float computeStability(const std::vector<DroneDetection>& dets) const;

private:
    // Coarse tracking: wide-angle global detection -> servo to center
    TrackingResult coarseTrack(const std::vector<DroneDetection>& dets,
                               int img_w, int img_h, float dt);

    // Fine tracking: ROI sub-pixel localization + ESKF prediction
    TrackingResult fineTrack(const DroneDetection& det,
                             int img_w, int img_h, float dt);

    // Decide whether to switch from coarse to fine
    bool shouldSwitchToFine(float stability) const;

    // Config
    float coarse_to_fine_threshold_ = 0.7f;
    float fine_to_coarse_threshold_ = 0.3f;
    float camera_fov_x_ = 1.047f;   // 60 deg
    float camera_fov_y_ = 0.785f;   // 45 deg
    float gimbal_yaw_ = 0.0f;
    float gimbal_pitch_ = 0.0f;
    Vec3 gimbal_pos_{0, 0, 1.5f};

    ESKF eskf_;
    ServoController servo_;
    LaserController laser_;

    TrackStage stage_ = TrackStage::COARSE;
    uint32_t track_id_ = 1;
    float stability_ = 0.0f;
    DroneDetection last_det_;

    // LiDAR fusion
    bool has_lidar_ = false;
    float lidar_range_ = 0.0f;

    // Optical axis calibration
    float axis_offset_yaw_ = 0.0f;
    float axis_offset_pitch_ = 0.0f;

    Config config_loader_;
};

} // namespace anti_drone
} // namespace rm_radar
