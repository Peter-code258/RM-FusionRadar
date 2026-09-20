// mono_radar.h - Low-cost monocular vision radar subsystem
// Pipeline: distortion correction -> detection input -> ground contact point ->
//           homography mapping -> Kalman tracking -> global coordinate output
#pragma once

#include "common_types.h"
#include "math_utils.h"
#include "config_loader.h"
#include "logger.h"

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include <cstdint>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace rm_radar {
namespace mono {

// Pixel coordinate
struct PixelPoint {
    float u = 0;
    float v = 0;
};

// Detection box from YOLO
struct DetectionBox {
    float x1, y1, x2, y2;  // pixel coordinates
    float confidence;
    int class_id;
};

// Obstacle geometry model for blind spot handling
struct ObstacleModel {
    std::vector<PixelPoint> polygon_pixels; // obstacle footprint in image
    std::vector<cv::Point2f> world_polygon; // in world coordinates
};

// Camera intrinsic model
struct CameraModel {
    cv::Mat K;            // 3x3 intrinsic matrix
    cv::Mat dist;         // distortion coefficients [k1,k2,p1,p2,k3]
    cv::Mat map1, map2;   // undistortion maps
    int width = 0, height = 0;
    bool initialized = false;

    void buildUndistortMaps() {
        if (K.empty() || dist.empty()) return;
        cv::initUndistortRectifyMap(K, dist, cv::Mat(), K,
                                    cv::Size(width, height), CV_32FC1, map1, map2);
        initialized = true;
    }

    cv::Mat undistort(const cv::Mat& img) const {
        cv::Mat out;
        cv::remap(img, out, map1, map2, cv::INTER_LINEAR);
        return out;
    }

    // Undistort a single pixel point
    PixelPoint undistortPoint(const PixelPoint& p) const {
        std::vector<cv::Point2f> pts = {{p.u, p.v}};
        std::vector<cv::Point2f> undist;
        cv::undistortPoints(pts, undist, K, dist, cv::Mat(), K);
        if (undist.empty()) return p;
        return {undist[0].x, undist[0].y};
    }
};

// Tracked target with re-identification support
struct MonoTrack {
    uint32_t id = 0;
    Vec3 world_position;
    Vec3 velocity;
    math::KalmanFilter3D kf;
    PixelPoint last_pixel;
    int lost_frames = 0;
    bool in_blind_spot = false;
    float confidence = 0.0f;
};

class MonoRadar {
public:
    MonoRadar();
    ~MonoRadar();

    bool loadConfig(const std::string& yaml_path);
    bool init();

    // Set camera intrinsics
    void setCamera(const cv::Mat& K, const cv::Mat& dist, int width, int height);

    // Calibrate homography from pixel-world correspondences (RANSAC)
    bool calibrateHomography(const std::vector<PixelPoint>& pixels,
                             const std::vector<cv::Point2f>& world_pts);

    // Set zone-based homographies (for better long-range accuracy)
    void setZoneHomographies(const std::vector<cv::Mat>& homographies,
                             const std::vector<cv::Rect>& zones);

    // Set obstacle models for blind spot tracking
    void setObstacles(const std::vector<ObstacleModel>& obstacles);

    // Process detections for one frame, return world-coordinate targets
    std::vector<Target> process(const std::vector<DetectionBox>& detections,
                                const cv::Mat& frame = cv::Mat());

    // Check camera attitude stability; returns true if stable
    bool checkAttitude(const cv::Mat& frame);
    float attitudeDeviation() const { return attitude_dev_; }

    // Get tracks
    const std::vector<MonoTrack>& tracks() const { return tracks_; }

    // Check if a pixel point is inside any obstacle blind spot
    bool isInBlindSpot(const PixelPoint& p) const;

    // Map pixel to world coordinates via homography
    cv::Point2f pixelToWorld(const PixelPoint& p) const;

    // Extract ground contact point (bottom center of detection box)
    PixelPoint groundContactPoint(const DetectionBox& det) const;

    // Distance compensation factor for a given world distance
    float distanceCompensation(float world_dist) const;

    const CameraModel& camera() const { return camera_; }
    const cv::Mat& homography() const { return H_; }

private:
    // Data association (Hungarian) between tracks and detections
    std::vector<int> associate(const std::vector<MonoTrack>& tracks,
                                const std::vector<cv::Point2f>& world_pts);

    // Re-identify a target that reappeared from blind spot
    int reidentify(const cv::Point2f& world_pt);

    // Config
    float match_threshold_ = 1.0f;
    int max_lost_frames_ = 10;
    float attitude_threshold_ = 5.0f; // degrees
    bool distance_compensation_ = true;
    float comp_gain_ = 0.02f;

    CameraModel camera_;
    cv::Mat H_;                       // global homography (3x3)
    std::vector<cv::Mat> zone_H_;     // zone homographies
    std::vector<cv::Rect> zone_rects_;
    std::vector<ObstacleModel> obstacles_;

    std::vector<MonoTrack> tracks_;
    uint32_t next_id_ = 1;

    float attitude_dev_ = 0.0f;
    cv::Mat prev_frame_gray_;

    Config config_loader_;
};

} // namespace mono
} // namespace rm_radar
