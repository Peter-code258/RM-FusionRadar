// fusion_center.h - Multi-sensor fusion processing center
// Post-fusion architecture: sensor-level independent processing, data-layer fusion
#pragma once

#include "common_types.h"
#include "math_utils.h"
#include "config_loader.h"
#include "logger.h"

#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <chrono>
#include <cstdint>

namespace rm_radar {
namespace fusion {

// Sensor source identifiers (must match common_types.h SensorSource)
using SensorSource = rm_radar::SensorSource;

// A measurement coming from one sensor, already in world coordinates
struct SensorMeasurement {
    SensorSource source;
    Target target;              // target with world position
    uint64_t timestamp_us;
    float sensor_confidence;    // confidence reported by the sensor
};

// Coordinate transform chain: sensor -> body -> world
struct TransformChain {
    std::array<float, 16> sensor_to_body;  // 4x4 row-major
    std::array<float, 16> body_to_world;   // 4x4 row-major
    bool valid = false;
};

// Fusion weight configuration by distance
struct FusionWeights {
    float vision;
    float radar;
    float sdr;
};

// Degradation level
enum class DegradationLevel {
    FULL = 0,          // all sensors active
    NO_VISION,         // vision failed -> radar + SDR
    NO_RADAR,          // radar failed -> monocular vision
    SDR_ONLY           // all perception failed -> SDR pure data
};

// A fused target with confidence and source contributions
struct FusedTarget {
    Target target;
    float confidence = 0.0f;
    std::vector<SensorSource> contributors;
    uint64_t last_update_us = 0;
};

class FusionCenter {
public:
    FusionCenter();
    ~FusionCenter();

    bool loadConfig(const std::string& yaml_path);
    bool init();

    // Feed a measurement from a sensor (thread-safe push)
    void feedMeasurement(const SensorMeasurement& m);

    // Run one fusion cycle: associate, fuse, degrade, output
    std::vector<Target> process();

    // Set sensor-to-body and body-to-world transforms
    void setTransform(SensorSource src, const std::array<float, 16>& sensor_to_body,
                      const std::array<float, 16>& body_to_world);

    // Manual degradation control (for testing)
    void forceDegradation(DegradationLevel level);
    DegradationLevel degradationLevel() const { return degradation_; }

    // Get fused targets
    const std::vector<FusedTarget>& fusedTargets() const { return fused_; }

    // Sensor health
    bool isSensorHealthy(SensorSource src) const;
    std::string statusString() const;

private:
    // Coordinate transform
    Vec3 transformPoint(const std::array<float, 16>& m, const Vec3& p) const;
    Vec3 toWorld(const SensorMeasurement& m) const;

    // Time synchronization
    void synchronizeTime(std::vector<SensorMeasurement>& meas);

    // Data association (Hungarian)
    std::vector<int> associate(const std::vector<FusedTarget>& tracks,
                               const std::vector<SensorMeasurement>& meas);

    // Dynamic weighted fusion
    FusionWeights weightsForDistance(float dist) const;
    FusedTarget fuse(const std::vector<SensorMeasurement>& meas_for_track,
                     const FusedTarget& prev);

    // Fault detection & degradation
    void checkSensorHealth();
    void updateDegradation();

    // Confidence management
    void updateConfidence(FusedTarget& ft, bool updated);

    // Config
    float match_threshold_ = 1.0f;       // association threshold (m)
    float confidence_threshold_ = 0.15f; // below this, target is removed
    uint64_t sensor_timeout_us_ = 500000; // sensor data timeout for fault
    int max_lost_frames_ = 5;

    // Sensor transforms
    std::unordered_map<int, TransformChain> transforms_;

    // Pending measurements (per cycle)
    std::vector<SensorMeasurement> pending_;

    // Fused tracks
    std::vector<FusedTarget> fused_;
    uint32_t next_id_ = 1;

    // Sensor health tracking
    std::unordered_map<int, uint64_t> last_sensor_ts_;
    DegradationLevel degradation_ = DegradationLevel::FULL;
    DegradationLevel forced_degradation_ = DegradationLevel::FULL;
    bool force_degradation_ = false;

    // Weights by range
    struct { float vision, radar, sdr; } near_w_{0.7f, 0.3f, 0.9f};
    struct { float vision, radar, sdr; } mid_w_{0.5f, 0.5f, 0.9f};
    struct { float vision, radar, sdr; } far_w_{0.2f, 0.8f, 0.9f};
    float near_dist_ = 8.0f;
    float far_dist_ = 15.0f;

    Config config_loader_;
};

} // namespace fusion
} // namespace rm_radar
