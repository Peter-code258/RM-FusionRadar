// fusion_center.cpp - Multi-sensor fusion center implementation
#include "fusion_center.h"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace rm_radar {
namespace fusion {

static std::mutex g_mutex;

FusionCenter::FusionCenter() = default;
FusionCenter::~FusionCenter() = default;

bool FusionCenter::loadConfig(const std::string& yaml_path) {
    if (!config_loader_.load(yaml_path)) {
        RM_LOG_WARN("Fusion config not found: " + yaml_path + ", using defaults");
        return false;
    }
    match_threshold_ = config_loader_.getFloat("match_threshold", match_threshold_);
    confidence_threshold_ = config_loader_.getFloat("confidence_threshold", confidence_threshold_);
    sensor_timeout_us_ = static_cast<uint64_t>(config_loader_.getFloat("sensor_timeout_ms", 500.0f) * 1000.0f);
    near_dist_ = config_loader_.getFloat("near_distance", near_dist_);
    far_dist_ = config_loader_.getFloat("far_distance", far_dist_);
    near_w_.vision = config_loader_.getFloat("near_vision_weight", near_w_.vision);
    near_w_.radar = config_loader_.getFloat("near_radar_weight", near_w_.radar);
    mid_w_.vision = config_loader_.getFloat("mid_vision_weight", mid_w_.vision);
    mid_w_.radar = config_loader_.getFloat("mid_radar_weight", mid_w_.radar);
    far_w_.vision = config_loader_.getFloat("far_vision_weight", far_w_.vision);
    far_w_.radar = config_loader_.getFloat("far_radar_weight", far_w_.radar);
    RM_LOG_INFO("Fusion center config loaded");
    return true;
}

bool FusionCenter::init() {
    RM_LOG_INFO("Fusion center initialized");
    return true;
}

void FusionCenter::setTransform(SensorSource src, const std::array<float, 16>& s2b,
                                const std::array<float, 16>& b2w) {
    TransformChain tc;
    tc.sensor_to_body = s2b;
    tc.body_to_world = b2w;
    tc.valid = true;
    transforms_[static_cast<int>(src)] = tc;
}

Vec3 FusionCenter::transformPoint(const std::array<float, 16>& m, const Vec3& p) const {
    return Vec3(
        m[0]*p.x + m[1]*p.y + m[2]*p.z + m[3],
        m[4]*p.x + m[5]*p.y + m[6]*p.z + m[7],
        m[8]*p.x + m[9]*p.y + m[10]*p.z + m[11]
    );
}

Vec3 FusionCenter::toWorld(const SensorMeasurement& m) const {
    auto it = transforms_.find(static_cast<int>(m.source));
    Vec3 world = m.target.position;
    if (it != transforms_.end() && it->second.valid) {
        Vec3 body = transformPoint(it->second.sensor_to_body, m.target.position);
        world = transformPoint(it->second.body_to_world, body);
    }
    return world;
}

void FusionCenter::feedMeasurement(const SensorMeasurement& m) {
    std::lock_guard<std::mutex> lock(g_mutex);
    SensorMeasurement wm = m;
    wm.target.position = toWorld(m);
    pending_.push_back(wm);
    last_sensor_ts_[static_cast<int>(m.source)] = now_us();
}

void FusionCenter::synchronizeTime(std::vector<SensorMeasurement>& meas) {
    // Linear interpolation / alignment to the latest timestamp in the batch
    if (meas.empty()) return;
    uint64_t latest = 0;
    for (const auto& m : meas) latest = std::max(latest, m.timestamp_us);

    for (auto& m : meas) {
        uint64_t dt_us = latest - m.timestamp_us;
        if (dt_us > 0 && dt_us < 200000) { // only interpolate small gaps
            float dt = dt_us / 1e6f;
            // Predict position forward using velocity
            m.target.position.x += m.target.velocity.x * dt;
            m.target.position.y += m.target.velocity.y * dt;
            m.target.position.z += m.target.velocity.z * dt;
            m.timestamp_us = latest;
        }
    }
}

std::vector<int> FusionCenter::associate(const std::vector<FusedTarget>& tracks,
                                          const std::vector<SensorMeasurement>& meas) {
    int n = static_cast<int>(tracks.size());
    int m_count = static_cast<int>(meas.size());
    if (n == 0 || m_count == 0) return std::vector<int>();

    std::vector<std::vector<float>> cost(n, std::vector<float>(m_count, 1e9f));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m_count; ++j) {
            float dx = tracks[i].target.position.x - meas[j].target.position.x;
            float dy = tracks[i].target.position.y - meas[j].target.position.y;
            float dz = tracks[i].target.position.z - meas[j].target.position.z;
            cost[i][j] = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
    }
    math::HungarianMatcher matcher;
    return matcher.solve(cost, match_threshold_);
}

FusionWeights FusionCenter::weightsForDistance(float dist) const {
    if (dist < near_dist_) return {near_w_.vision, near_w_.radar, near_w_.sdr};
    if (dist < far_dist_) return {mid_w_.vision, mid_w_.radar, mid_w_.sdr};
    return {far_w_.vision, far_w_.radar, far_w_.sdr};
}

FusedTarget FusionCenter::fuse(const std::vector<SensorMeasurement>& meas_for_track,
                                const FusedTarget& prev) {
    FusedTarget result = prev;
    result.contributors.clear();

    if (meas_for_track.empty()) {
        // No new measurements; decay confidence
        result.target.confidence *= 0.9f;
        return result;
    }

    float dist = prev.target.position.norm();
    FusionWeights w = weightsForDistance(dist);

    float weighted_x = 0, weighted_y = 0, weighted_z = 0;
    float vx = 0, vy = 0, vz = 0;
    float total_w = 0.0f;
    float conf_sum = 0.0f;

    for (const auto& m : meas_for_track) {
        float weight = 0.0f;
        switch (m.source) {
            case SensorSource::VISION:
            case SensorSource::MONOCULAR: weight = w.vision; break;
            case SensorSource::LIDAR: weight = w.radar; break;
            case SensorSource::SDR: weight = w.sdr; break;
            default: weight = 0.5f; break;
        }
        weight *= m.sensor_confidence;
        weighted_x += m.target.position.x * weight;
        weighted_y += m.target.position.y * weight;
        weighted_z += m.target.position.z * weight;
        vx += m.target.velocity.x * weight;
        vy += m.target.velocity.y * weight;
        vz += m.target.velocity.z * weight;
        conf_sum += m.target.confidence * weight;
        total_w += weight;
        result.contributors.push_back(m.source);
    }

    if (total_w > 1e-6f) {
        result.target.position.x = weighted_x / total_w;
        result.target.position.y = weighted_y / total_w;
        result.target.position.z = weighted_z / total_w;
        result.target.velocity.x = vx / total_w;
        result.target.velocity.y = vy / total_w;
        result.target.velocity.z = vz / total_w;
        result.target.confidence = std::min(1.0f, conf_sum / total_w);
    }
    result.last_update_us = now_us();
    return result;
}

bool FusionCenter::isSensorHealthy(SensorSource src) const {
    auto it = last_sensor_ts_.find(static_cast<int>(src));
    if (it == last_sensor_ts_.end()) return false;
    return (now_us() - it->second) < sensor_timeout_us_;
}

void FusionCenter::checkSensorHealth() {
    // Determine which sensors are alive
    bool vision_ok = isSensorHealthy(SensorSource::VISION) || isSensorHealthy(SensorSource::MONOCULAR);
    bool radar_ok = isSensorHealthy(SensorSource::LIDAR);
    bool sdr_ok = isSensorHealthy(SensorSource::SDR);

    if (force_degradation_) {
        degradation_ = forced_degradation_;
        return;
    }

    // 3-level degradation logic
    if (vision_ok && radar_ok && sdr_ok) {
        degradation_ = DegradationLevel::FULL;
    } else if (!vision_ok && radar_ok) {
        degradation_ = DegradationLevel::NO_VISION;
    } else if (!radar_ok && vision_ok) {
        degradation_ = DegradationLevel::NO_RADAR;
    } else if (!vision_ok && !radar_ok && sdr_ok) {
        degradation_ = DegradationLevel::SDR_ONLY;
    } else if (radar_ok) {
        degradation_ = DegradationLevel::NO_VISION;
    } else if (sdr_ok) {
        degradation_ = DegradationLevel::SDR_ONLY;
    } else {
        degradation_ = DegradationLevel::SDR_ONLY; // last resort
    }
}

void FusionCenter::updateDegradation() {
    // Filter out measurements from failed sensors based on degradation level
    checkSensorHealth();
    std::vector<SensorMeasurement> filtered;
    for (const auto& m : pending_) {
        bool keep = false;
        switch (degradation_) {
            case DegradationLevel::FULL:
                keep = true;
                break;
            case DegradationLevel::NO_VISION:
                keep = (m.source == SensorSource::LIDAR || m.source == SensorSource::SDR);
                break;
            case DegradationLevel::NO_RADAR:
                keep = (m.source == SensorSource::VISION || m.source == SensorSource::MONOCULAR
                        || m.source == SensorSource::SDR);
                break;
            case DegradationLevel::SDR_ONLY:
                keep = (m.source == SensorSource::SDR);
                break;
        }
        if (keep) filtered.push_back(m);
    }
    pending_ = filtered;
}

void FusionCenter::updateConfidence(FusedTarget& ft, bool updated) {
    if (updated) {
        ft.target.confidence = std::min(1.0f, ft.target.confidence + 0.1f);
    } else {
        ft.target.confidence *= 0.85f;
    }
    ft.confidence = ft.target.confidence;
}

std::vector<Target> FusionCenter::process() {
    std::lock_guard<std::mutex> lock(g_mutex);

    // Step 1: Degradation filtering
    updateDegradation();

    // Step 2: Time synchronization
    synchronizeTime(pending_);

    // Step 3: Associate measurements to existing tracks
    std::vector<int> assignment = associate(fused_, pending_);
    std::vector<bool> meas_used(pending_.size(), false);

    // Group measurements per track
    std::vector<std::vector<SensorMeasurement>> per_track(fused_.size());
    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0) {
            per_track[i].push_back(pending_[assignment[i]]);
            meas_used[assignment[i]] = true;
        }
    }

    // Step 4: Fuse each track
    std::vector<FusedTarget> new_fused;
    for (size_t i = 0; i < fused_.size(); ++i) {
        FusedTarget ft = fuse(per_track[i], fused_[i]);
        updateConfidence(ft, !per_track[i].empty());
        if (ft.target.confidence > confidence_threshold_) {
            new_fused.push_back(ft);
        }
    }

    // Step 5: Group nearby unmatched measurements (cross-sensor) then create new tracks
    std::vector<bool> processed(pending_.size(), false);
    for (size_t j = 0; j < pending_.size(); ++j) {
        if (meas_used[j] || processed[j]) continue;
        // Start a new group with this measurement
        std::vector<SensorMeasurement> group;
        group.push_back(pending_[j]);
        processed[j] = true;
        // Find all other unmatched measurements close to this one
        for (size_t k = j + 1; k < pending_.size(); ++k) {
            if (meas_used[k] || processed[k]) continue;
            float dx = pending_[j].target.position.x - pending_[k].target.position.x;
            float dy = pending_[j].target.position.y - pending_[k].target.position.y;
            float dz = pending_[j].target.position.z - pending_[k].target.position.z;
            if (std::sqrt(dx*dx + dy*dy + dz*dz) < match_threshold_) {
                group.push_back(pending_[k]);
                processed[k] = true;
            }
        }
        // Fuse the group into a single new track
        FusedTarget ft;
        ft.target.id = next_id_++;
        ft.target.confidence = 0.5f;
        ft.confidence = 0.5f;
        ft.last_update_us = now_us();
        ft = fuse(group, ft);
        new_fused.push_back(ft);
    }

    fused_ = new_fused;
    pending_.clear();

    // Build output
    std::vector<Target> out;
    for (const auto& ft : fused_) {
        out.push_back(ft.target);
    }
    return out;
}

void FusionCenter::forceDegradation(DegradationLevel level) {
    force_degradation_ = true;
    forced_degradation_ = level;
}

std::string FusionCenter::statusString() const {
    std::string s = "Degradation: ";
    switch (degradation_) {
        case DegradationLevel::FULL: s += "FULL"; break;
        case DegradationLevel::NO_VISION: s += "NO_VISION (radar+SDR)"; break;
        case DegradationLevel::NO_RADAR: s += "NO_RADAR (vision+SDR)"; break;
        case DegradationLevel::SDR_ONLY: s += "SDR_ONLY"; break;
    }
    s += " | Targets: " + std::to_string(fused_.size());
    return s;
}

} // namespace fusion
} // namespace rm_radar
