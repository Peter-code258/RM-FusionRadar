// anti_drone_tracker.cpp - UAV tracking and countermeasure implementation
#include "anti_drone_tracker.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace rm_radar {
namespace anti_drone {

AntiDroneTracker::AntiDroneTracker() = default;
AntiDroneTracker::~AntiDroneTracker() = default;

bool AntiDroneTracker::loadConfig(const std::string& yaml_path) {
    if (!config_loader_.load(yaml_path)) {
        RM_LOG_WARN("Anti-drone config not found, using defaults");
        return false;
    }
    coarse_to_fine_threshold_ = config_loader_.getFloat("coarse_to_fine_threshold", coarse_to_fine_threshold_);
    fine_to_coarse_threshold_ = config_loader_.getFloat("fine_to_coarse_threshold", fine_to_coarse_threshold_);
    camera_fov_x_ = config_loader_.getFloat("fov_x", camera_fov_x_);
    camera_fov_y_ = config_loader_.getFloat("fov_y", camera_fov_y_);
    RM_LOG_INFO("Anti-drone tracker config loaded");
    return true;
}

bool AntiDroneTracker::init() {
    eskf_.reset();
    laser_.reset();
    servo_.reset();
    RM_LOG_INFO("Anti-drone tracker initialized");
    return true;
}

Vec3 AntiDroneTracker::pixelToWorld(const DroneDetection& det, int img_w, int img_h) const {
    // Convert pixel to angle relative to image center
    float angle_x = (det.cx - img_w * 0.5f) / (img_w * 0.5f) * (camera_fov_x_ * 0.5f);
    float angle_y = (det.cy - img_h * 0.5f) / (img_h * 0.5f) * (camera_fov_y_ * 0.5f);

    // Add gimbal angles
    float total_yaw = gimbal_yaw_ + angle_x + axis_offset_yaw_;
    float total_pitch = gimbal_pitch_ + angle_y + axis_offset_pitch_;

    // Estimate range: use lidar if available, else assume 10m
    float range = has_lidar_ ? lidar_range_ : 10.0f;

    Vec3 world;
    world.x = gimbal_pos_.x + range * std::cos(total_pitch) * std::cos(total_yaw);
    world.y = gimbal_pos_.y + range * std::cos(total_pitch) * std::sin(total_yaw);
    world.z = gimbal_pos_.z + range * std::sin(total_pitch);
    return world;
}

float AntiDroneTracker::computeStability(const std::vector<DroneDetection>& dets) const {
    if (dets.empty()) return 0.0f;
    // Stability based on detection confidence and box size consistency
    float conf = 0;
    for (const auto& d : dets) conf = std::max(conf, d.confidence);
    // Penalize large box jumps
    float jump = std::abs(dets[0].cx - last_det_.cx) + std::abs(dets[0].cy - last_det_.cy);
    float stability = conf * (1.0f - std::min(jump / 200.0f, 1.0f));
    return std::max(0.0f, std::min(1.0f, stability));
}

bool AntiDroneTracker::shouldSwitchToFine(float stability) const {
    return stability > coarse_to_fine_threshold_;
}

TrackingResult AntiDroneTracker::coarseTrack(const std::vector<DroneDetection>& dets,
                                              int img_w, int img_h, float dt) {
    TrackingResult result;
    result.stage = TrackStage::COARSE;

    if (dets.empty()) {
        stability_ *= 0.9f;
        result.stability = stability_;
        return result;
    }

    // Pick highest confidence detection
    auto best = *std::max_element(dets.begin(), dets.end(),
        [](const DroneDetection& a, const DroneDetection& b) { return a.confidence < b.confidence; });
    last_det_ = best;

    stability_ = computeStability(dets);
    result.stability = stability_;

    Vec3 world = pixelToWorld(best, img_w, img_h);

    if (!eskf_.initialized()) {
        eskf_.init(world);
    } else {
        eskf_.predict(dt);
        eskf_.update(world);
    }

    result.world_position = eskf_.position();
    result.world_velocity = eskf_.velocity();
    result.predicted_next = eskf_.predictPosition(dt);
    result.track_id = track_id_;

    // Servo: move gimbal to center the target
    result.servo_cmd = servo_.compute(result.world_position, result.world_velocity, gimbal_pos_, dt);
    gimbal_yaw_ = result.servo_cmd.yaw;
    gimbal_pitch_ = result.servo_cmd.pitch;

    // Switch to fine if stable enough
    if (shouldSwitchToFine(stability_)) {
        stage_ = TrackStage::FINE;
        result.stage = TrackStage::FINE;
    }

    // Laser control
    result.laser_emitting = laser_.update(stability_, dt * 1000.0f);
    return result;
}

TrackingResult AntiDroneTracker::fineTrack(const DroneDetection& det,
                                            int img_w, int img_h, float dt) {
    TrackingResult result;
    result.stage = TrackStage::FINE;

    // Sub-pixel refinement (simulated: use detection center directly)
    Vec3 world = pixelToWorld(det, img_w, img_h);

    eskf_.predict(dt);
    eskf_.update(world);

    result.world_position = eskf_.position();
    result.world_velocity = eskf_.velocity();
    result.predicted_next = eskf_.predictPosition(dt);
    result.track_id = track_id_;

    // Compute stability from ESKF innovation (smaller innovation = more stable)
    float innov = std::sqrt(
        std::pow(world.x - result.world_position.x, 2) +
        std::pow(world.y - result.world_position.y, 2) +
        std::pow(world.z - result.world_position.z, 2));
    stability_ = std::max(0.0f, 1.0f - innov * 2.0f);
    result.stability = stability_;

    // Precision servo with feedforward
    result.servo_cmd = servo_.compute(result.world_position, result.world_velocity, gimbal_pos_, dt);
    gimbal_yaw_ = result.servo_cmd.yaw;
    gimbal_pitch_ = result.servo_cmd.pitch;

    // Switch back to coarse if lost
    if (stability_ < fine_to_coarse_threshold_) {
        stage_ = TrackStage::COARSE;
        result.stage = TrackStage::COARSE;
    }

    result.laser_emitting = laser_.update(stability_, dt * 1000.0f);
    return result;
}

TrackingResult AntiDroneTracker::process(const std::vector<DroneDetection>& detections,
                                          const cv::Mat& frame) {
    float dt = 0.016f; // ~60fps
    int img_w = frame.empty() ? 640 : frame.cols;
    int img_h = frame.empty() ? 480 : frame.rows;

    if (stage_ == TrackStage::COARSE) {
        return coarseTrack(detections, img_w, img_h, dt);
    } else {
        // Fine mode: use the best detection, or fall back to prediction
        if (detections.empty()) {
            eskf_.predict(dt);
            TrackingResult r;
            r.stage = TrackStage::FINE;
            r.world_position = eskf_.position();
            r.world_velocity = eskf_.velocity();
            r.predicted_next = eskf_.predictPosition(dt);
            r.track_id = track_id_;
            r.stability = stability_ * 0.9f;
            r.servo_cmd = servo_.compute(r.world_position, r.world_velocity, gimbal_pos_, dt);
            r.laser_emitting = laser_.update(r.stability, dt * 1000.0f);
            return r;
        }
        auto best = *std::max_element(detections.begin(), detections.end(),
            [](const DroneDetection& a, const DroneDetection& b) { return a.confidence < b.confidence; });
        last_det_ = best;
        return fineTrack(best, img_w, img_h, dt);
    }
}

} // namespace anti_drone
} // namespace rm_radar
