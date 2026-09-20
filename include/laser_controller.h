// laser_controller.h - Laser countermeasure controller
#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <cstring>

namespace rm_radar {
namespace anti_drone {

// Laser emission state
enum class LaserState {
    STANDBY = 0,
    ARMING,
    EMITTING,
    COOLDOWN
};

// Laser controller with tracking stability gating and effective countermeasure判定
class LaserController {
public:
    void reset() {
        state_ = LaserState::STANDBY;
        emit_time_ms_ = 0;
        stability_frames_ = 0;
        total_emit_time_ms_ = 0;
    }

    // Update based on tracking stability (0.0 - 1.0)
    // Returns true if laser should be emitting
    bool update(float tracking_stability, float dt_ms) {
        switch (state_) {
            case LaserState::STANDBY:
                if (tracking_stability > arm_threshold_) {
                    stability_frames_++;
                    if (stability_frames_ >= arm_frames_) {
                        state_ = LaserState::EMITTING;
                        emit_time_ms_ = 0;
                    }
                } else {
                    stability_frames_ = 0;
                }
                break;
            case LaserState::EMITTING:
                emit_time_ms_ += dt_ms;
                total_emit_time_ms_ += dt_ms;
                if (tracking_stability < stop_threshold_) {
                    state_ = LaserState::COOLDOWN;
                    cooldown_ms_ = 0;
                } else if (emit_time_ms_ >= max_emit_ms_) {
                    state_ = LaserState::COOLDOWN;
                    cooldown_ms_ = 0;
                }
                break;
            case LaserState::COOLDOWN:
                cooldown_ms_ += dt_ms;
                if (cooldown_ms_ >= cooldown_duration_ms_) {
                    state_ = LaserState::STANDBY;
                    stability_frames_ = 0;
                }
                break;
            default:
                break;
        }
        return state_ == LaserState::EMITTING;
    }

    bool isEmitting() const { return state_ == LaserState::EMITTING; }
    LaserState state() const { return state_; }
    float emitTimeMs() const { return emit_time_ms_; }
    float totalEmitTimeMs() const { return total_emit_time_ms_; }

    // Check if countermeasure was effective (total emit time exceeds threshold)
    bool isCountermeasureEffective() const {
        return total_emit_time_ms_ >= effective_threshold_ms_;
    }

    // Serial frame for laser control: [HEAD 0xCD] [state(1)] [duration_ms(4)] [CRC8]
    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> frame;
        frame.push_back(0xCD);
        frame.push_back(static_cast<uint8_t>(state_));
        uint32_t dur = static_cast<uint32_t>(emit_time_ms_);
        for (int i = 0; i < 4; ++i) frame.push_back((dur >> (i*8)) & 0xFF);
        uint8_t crc = 0;
        for (size_t i = 1; i < frame.size(); ++i) crc ^= frame[i];
        frame.push_back(crc);
        return frame;
    }

    void setArmThreshold(float t) { arm_threshold_ = t; }
    void setArmFrames(int n) { arm_frames_ = n; }
    void setMaxEmitMs(float ms) { max_emit_ms_ = ms; }

private:
    LaserState state_ = LaserState::STANDBY;
    float emit_time_ms_ = 0;
    float total_emit_time_ms_ = 0;
    float cooldown_ms_ = 0;
    int stability_frames_ = 0;

    float arm_threshold_ = 0.8f;
    int arm_frames_ = 10;
    float stop_threshold_ = 0.5f;
    float max_emit_ms_ = 5000.0f;
    float cooldown_duration_ms_ = 1000.0f;
    float effective_threshold_ms_ = 3000.0f;
};

} // namespace anti_drone
} // namespace rm_radar
