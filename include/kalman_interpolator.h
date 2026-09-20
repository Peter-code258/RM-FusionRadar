// kalman_interpolator.h - Kalman filter based trajectory interpolation (10Hz -> 30Hz)
#pragma once

#include "sdr_protocol.h"
#include "common_types.h"
#include "math_utils.h"

#include <unordered_map>
#include <vector>
#include <cstdint>

namespace rm_radar {
namespace sdr {

// Interpolates official 10Hz position data to a higher output rate (e.g. 30Hz)
// using a constant-velocity Kalman filter per robot ID.
class KalmanInterpolator {
public:
    KalmanInterpolator(int input_rate = 10, int output_rate = 30)
        : input_dt_(1.0f / input_rate), output_dt_(1.0f / output_rate) {}

    // Update with a new measurement at input rate
    void update(const SdrTarget& meas) {
        auto& kf = filters_[meas.robot_id];
        if (!kf.initialized()) {
            kf.init(Vec3(meas.x, meas.y, 0.0f));
        } else {
            kf.predict(input_dt_);
            kf.update(Vec3(meas.x, meas.y, 0.0f));
        }
        last_meas_[meas.robot_id] = meas;
        last_update_us_[meas.robot_id] = now_us();
    }

    // Generate interpolated samples at output rate between measurements.
    // Call this periodically; it returns predicted positions for each tracked robot.
    std::vector<SdrTarget> interpolate() {
        std::vector<SdrTarget> out;
        for (auto& [id, kf] : filters_) {
            kf.predict(output_dt_);
            Vec3 pos = kf.position();
            Vec3 vel = kf.velocity();
            SdrTarget t = last_meas_[id];
            t.x = pos.x;
            t.y = pos.y;
            t.timestamp_us = now_us();
            // confidence decays if no recent measurement
            uint64_t age = now_us() - last_update_us_[id];
            t.confidence = std::max(0.1f, 1.0f - static_cast<float>(age) / 1e6f);
            out.push_back(t);
        }
        return out;
    }

    // Check if a robot has recent data (within timeout_us)
    bool isFresh(uint8_t robot_id, uint64_t timeout_us = 500000) const {
        auto it = last_update_us_.find(robot_id);
        if (it == last_update_us_.end()) return false;
        return (now_us() - it->second) < timeout_us;
    }

    void reset() {
        filters_.clear();
        last_meas_.clear();
        last_update_us_.clear();
    }

private:
    float input_dt_;
    float output_dt_;
    std::unordered_map<uint8_t, math::KalmanFilter3D> filters_;
    std::unordered_map<uint8_t, SdrTarget> last_meas_;
    std::unordered_map<uint8_t, uint64_t> last_update_us_;
};

} // namespace sdr
} // namespace rm_radar
