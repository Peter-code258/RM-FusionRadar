// mono_radar.cpp - Monocular vision radar implementation
#include "mono_radar.h"

#include <opencv2/video.hpp>
#include <algorithm>
#include <cmath>

namespace rm_radar {
namespace mono {

MonoRadar::MonoRadar() = default;
MonoRadar::~MonoRadar() = default;

bool MonoRadar::loadConfig(const std::string& yaml_path) {
    if (!config_loader_.load(yaml_path)) {
        RM_LOG_WARN("Mono config not found: " + yaml_path + ", using defaults");
        return false;
    }
    match_threshold_ = config_loader_.getFloat("match_threshold", match_threshold_);
    max_lost_frames_ = config_loader_.getInt("max_lost_frames", max_lost_frames_);
    attitude_threshold_ = config_loader_.getFloat("attitude_threshold", attitude_threshold_);
    distance_compensation_ = config_loader_.getBool("distance_compensation", distance_compensation_);
    comp_gain_ = config_loader_.getFloat("comp_gain", comp_gain_);
    RM_LOG_INFO("Mono radar config loaded");
    return true;
}

bool MonoRadar::init() {
    RM_LOG_INFO("Mono radar initialized");
    return true;
}

void MonoRadar::setCamera(const cv::Mat& K, const cv::Mat& dist, int w, int h) {
    camera_.K = K.clone();
    camera_.dist = dist.clone();
    camera_.width = w;
    camera_.height = h;
    camera_.buildUndistortMaps();
}

bool MonoRadar::calibrateHomography(const std::vector<PixelPoint>& pixels,
                                     const std::vector<cv::Point2f>& world_pts) {
    if (pixels.size() != world_pts.size() || pixels.size() < 4) {
        RM_LOG_ERROR("Need at least 4 point correspondences");
        return false;
    }
    std::vector<cv::Point2f> src, dst;
    for (const auto& p : pixels) src.emplace_back(p.u, p.v);
    for (const auto& w : world_pts) dst.push_back(w);

    // RANSAC homography estimation
    H_ = cv::findHomography(src, dst, cv::RANSAC, 3.0);
    if (H_.empty()) {
        RM_LOG_ERROR("Homography estimation failed");
        return false;
    }
    RM_LOG_INFO("Homography calibrated with " + std::to_string(pixels.size()) + " points");
    return true;
}

void MonoRadar::setZoneHomographies(const std::vector<cv::Mat>& homographies,
                                     const std::vector<cv::Rect>& zones) {
    zone_H_ = homographies;
    zone_rects_ = zones;
}

void MonoRadar::setObstacles(const std::vector<ObstacleModel>& obstacles) {
    obstacles_ = obstacles;
}

PixelPoint MonoRadar::groundContactPoint(const DetectionBox& det) const {
    // Bottom center of the detection box
    return {(det.x1 + det.x2) * 0.5f, det.y2};
}

cv::Point2f MonoRadar::pixelToWorld(const PixelPoint& p) const {
    if (H_.empty()) return {0, 0};

    // First check zone homographies
    for (size_t i = 0; i < zone_rects_.size(); ++i) {
        if (zone_rects_[i].contains(cv::Point((int)p.u, (int)p.v)) && i < zone_H_.size()) {
            std::vector<cv::Point2f> src = {{p.u, p.v}}, dst;
            cv::perspectiveTransform(src, dst, zone_H_[i]);
            if (!dst.empty()) return dst[0];
        }
    }

    // Fall back to global homography
    std::vector<cv::Point2f> src = {{p.u, p.v}}, dst;
    cv::perspectiveTransform(src, dst, H_);
    return dst.empty() ? cv::Point2f(0, 0) : dst[0];
}

bool MonoRadar::isInBlindSpot(const PixelPoint& p) const {
    cv::Point pt((int)p.u, (int)p.v);
    for (const auto& obs : obstacles_) {
        std::vector<cv::Point> poly;
        for (const auto& pp : obs.polygon_pixels) {
            poly.emplace_back((int)pp.u, (int)pp.v);
        }
        if (!poly.empty() && cv::pointPolygonTest(poly, pt, false) >= 0) {
            return true;
        }
    }
    return false;
}

float MonoRadar::distanceCompensation(float world_dist) const {
    if (!distance_compensation_) return 1.0f;
    // Compensate projection deviation for distant targets
    return 1.0f + comp_gain_ * world_dist;
}

std::vector<int> MonoRadar::associate(const std::vector<MonoTrack>& tracks,
                                       const std::vector<cv::Point2f>& world_pts) {
    int n = static_cast<int>(tracks.size());
    int m = static_cast<int>(world_pts.size());
    if (n == 0 || m == 0) return std::vector<int>();

    std::vector<std::vector<float>> cost(n, std::vector<float>(m, 1e9f));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            float dx = tracks[i].world_position.x - world_pts[j].x;
            float dy = tracks[i].world_position.y - world_pts[j].y;
            cost[i][j] = std::sqrt(dx*dx + dy*dy);
        }
    }
    math::HungarianMatcher matcher;
    return matcher.solve(cost, match_threshold_);
}

int MonoRadar::reidentify(const cv::Point2f& world_pt) {
    // Find a lost track whose predicted position is close
    for (auto& t : tracks_) {
        if (t.lost_frames > 0 && t.lost_frames <= max_lost_frames_) {
            float dx = t.world_position.x - world_pt.x;
            float dy = t.world_position.y - world_pt.y;
            if (std::sqrt(dx*dx + dy*dy) < match_threshold_ * 2.0f) {
                return t.id;
            }
        }
    }
    return -1;
}

std::vector<Target> MonoRadar::process(const std::vector<DetectionBox>& detections,
                                         const cv::Mat& frame) {
    std::vector<Target> result;
    std::vector<cv::Point2f> world_pts;
    std::vector<PixelPoint> contact_pts;

    // 1. Ground contact point extraction + homography mapping
    for (const auto& det : detections) {
        PixelPoint cp = groundContactPoint(det);
        if (camera_.initialized) {
            cp = camera_.undistortPoint(cp);
        }
        cv::Point2f wp = pixelToWorld(cp);

        // Distance compensation
        float dist = std::sqrt(wp.x*wp.x + wp.y*wp.y);
        float comp = distanceCompensation(dist);
        wp.x *= comp;
        wp.y *= comp;

        world_pts.push_back(wp);
        contact_pts.push_back(cp);
    }

    // 2. Associate detections to tracks
    std::vector<int> assignment = associate(tracks_, world_pts);
    std::vector<bool> det_used(world_pts.size(), false);

    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0) {
            int j = assignment[i];
            det_used[j] = true;
            tracks_[i].kf.predict(0.016f); // ~60fps
            tracks_[i].kf.update(Vec3(world_pts[j].x, world_pts[j].y, 0));
            tracks_[i].world_position = tracks_[i].kf.position();
            tracks_[i].velocity = tracks_[i].kf.velocity();
            tracks_[i].last_pixel = contact_pts[j];
            tracks_[i].lost_frames = 0;
            tracks_[i].in_blind_spot = isInBlindSpot(contact_pts[j]);
            tracks_[i].confidence = std::min(1.0f, tracks_[i].confidence + 0.1f);
        } else {
            // Track lost: continue Kalman prediction
            tracks_[i].kf.predict(0.016f);
            tracks_[i].world_position = tracks_[i].kf.position();
            tracks_[i].lost_frames++;
            tracks_[i].confidence *= 0.9f;
        }
    }

    // 3. Create new tracks or re-identify for unmatched detections
    for (size_t j = 0; j < world_pts.size(); ++j) {
        if (det_used[j]) continue;
        int rid = reidentify(world_pts[j]);
        if (rid > 0) {
            // Re-identify existing track
            for (auto& t : tracks_) {
                if (t.id == rid) {
                    t.kf.update(Vec3(world_pts[j].x, world_pts[j].y, 0));
                    t.world_position = t.kf.position();
                    t.lost_frames = 0;
                    t.confidence = std::max(t.confidence, 0.5f);
                    break;
                }
            }
        } else {
            MonoTrack t;
            t.id = next_id_++;
            t.world_position = Vec3(world_pts[j].x, world_pts[j].y, 0);
            t.kf.init(t.world_position);
            t.last_pixel = contact_pts[j];
            t.confidence = 0.5f;
            t.in_blind_spot = isInBlindSpot(contact_pts[j]);
            tracks_.push_back(t);
        }
    }

    // 4. Remove stale tracks
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const MonoTrack& t) { return t.lost_frames > max_lost_frames_; }),
        tracks_.end());

    // 5. Build output
    for (const auto& t : tracks_) {
        if (t.confidence < 0.2f) continue;
        Target target;
        target.id = t.id;
        target.type = TargetType::UNKNOWN;
        target.position = t.world_position;
        target.velocity = t.velocity;
        target.confidence = t.confidence;
        target.timestamp_us = now_us();
        result.push_back(target);
    }
    return result;
}

bool MonoRadar::checkAttitude(const cv::Mat& frame) {
    if (frame.empty()) return true;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    if (prev_frame_gray_.empty()) {
        prev_frame_gray_ = gray;
        return true;
    }

    // Estimate affine motion between frames
    std::vector<cv::Point2f> prev_pts, curr_pts;
    cv::goodFeaturesToTrack(prev_frame_gray_, prev_pts, 100, 0.01, 10);
    if (prev_pts.size() < 10) {
        prev_frame_gray_ = gray;
        return true;
    }

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame_gray_, gray, prev_pts, curr_pts, status, err);

    // Compute rotation angle from motion
    std::vector<cv::Point2f> good_prev, good_curr;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            good_prev.push_back(prev_pts[i]);
            good_curr.push_back(curr_pts[i]);
        }
    }

    if (good_prev.size() >= 3) {
        cv::Mat affine = cv::estimateAffinePartial2D(good_prev, good_curr);
        if (!affine.empty()) {
            float angle = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0)) * 180.0f / CV_PI;
            attitude_dev_ = std::abs(angle);
            if (attitude_dev_ > attitude_threshold_) {
                RM_LOG_WARN("Camera attitude deviation: " + std::to_string(attitude_dev_) + " deg");
            }
        }
    }

    prev_frame_gray_ = gray;
    return attitude_dev_ <= attitude_threshold_;
}

} // namespace mono
} // namespace rm_radar
