// servo_controller.h - Gimbal/galvo servo controller with feedforward compensation
#pragma once

#include "common_types.h"
#include <array>
#include <cstdint>
#include <cstring>
#include <string>

namespace rm_radar {
namespace anti_drone {

// Servo control output for 2-axis gimbal/galvo
struct ServoCommand {
    float yaw = 0.0f;     // radians
    float pitch = 0.0f;   // radians
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
};

// PID controller
struct PID {
    float kp = 1.0f, ki = 0.0f, kd = 0.1f;
    float integral = 0.0f, prev_error = 0.0f;
    float out_min = -1.0f, out_max = 1.0f;

    float update(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prev_error) / std::max(dt, 1e-6f);
        prev_error = error;
        float out = kp * error + ki * integral + kd * derivative;
        return std::max(out_min, std::min(out_max, out));
    }
    void reset() { integral = 0.0f; prev_error = 0.0f; }
};

// Two-axis servo controller with target motion feedforward
class ServoController {
public:
    ServoController() {
        yaw_pid_.kp = 2.0f; yaw_pid_.kd = 0.05f;
        pitch_pid_.kp = 2.0f; pitch_pid_.kd = 0.05f;
    }

    // Compute servo command to point at target_world from gimbal position
    // target_vel is used for feedforward compensation
    ServoCommand compute(const Vec3& target_world, const Vec3& target_vel,
                         const Vec3& gimbal_pos, float dt) {
        Vec3 rel = target_world - gimbal_pos;
        float desired_yaw = std::atan2(rel.y, rel.x);
        float dist_xy = std::sqrt(rel.x*rel.x + rel.y*rel.y);
        float desired_pitch = std::atan2(rel.z, dist_xy);

        // Feedforward: compensate for target angular velocity
        float yaw_ff = 0.0f, pitch_ff = 0.0f;
        if (dist_xy > 0.5f) {
            // Angular rate from target velocity
            yaw_ff = (target_vel.x * (-rel.y) + target_vel.y * rel.x) / (dist_xy * dist_xy);
            float r3 = dist_xy * dist_xy + rel.z * rel.z;
            pitch_ff = (target_vel.z * dist_xy - rel.z * (rel.x*target_vel.x + rel.y*target_vel.y) / dist_xy) / r3;
        }
        yaw_ff *= feedforward_gain_;
        pitch_ff *= feedforward_gain_;

        ServoCommand cmd;
        cmd.yaw = desired_yaw + yaw_pid_.update(desired_yaw - current_yaw_, dt) + yaw_ff * dt;
        cmd.pitch = desired_pitch + pitch_pid_.update(desired_pitch - current_pitch_, dt) + pitch_ff * dt;
        cmd.yaw_rate = yaw_ff;
        cmd.pitch_rate = pitch_ff;

        current_yaw_ = desired_yaw;
        current_pitch_ = desired_pitch;
        return cmd;
    }

    void reset() { yaw_pid_.reset(); pitch_pid_.reset(); current_yaw_ = 0; current_pitch_ = 0; }
    void setFeedforwardGain(float g) { feedforward_gain_ = g; }

    // Spot closed-loop correction: apply measured laser spot offset
    void applySpotCorrection(float spot_offset_x, float spot_offset_y) {
        current_yaw_ += spot_offset_x * spot_correction_gain_;
        current_pitch_ += spot_offset_y * spot_correction_gain_;
    }

private:
    PID yaw_pid_, pitch_pid_;
    float current_yaw_ = 0.0f;
    float current_pitch_ = 0.0f;
    float feedforward_gain_ = 0.8f;
    float spot_correction_gain_ = 0.5f;
};

// Serial protocol encoder for gimbal/galvo control (1kHz current loop)
class SerialProtocol {
public:
    // Encode a servo command into a serial frame
    // Frame: [HEAD 0xAB] [yaw(4)] [pitch(4)] [yaw_rate(4)] [pitch_rate(4)] [CRC8]
    static std::vector<uint8_t> encode(const ServoCommand& cmd) {
        std::vector<uint8_t> frame;
        frame.push_back(0xAB);
        auto appendFloat = [&](float v) {
            uint32_t u;
            std::memcpy(&u, &v, 4);
            for (int i = 0; i < 4; ++i) frame.push_back((u >> (i*8)) & 0xFF);
        };
        appendFloat(cmd.yaw);
        appendFloat(cmd.pitch);
        appendFloat(cmd.yaw_rate);
        appendFloat(cmd.pitch_rate);
        // CRC8
        uint8_t crc = 0;
        for (size_t i = 1; i < frame.size(); ++i) crc ^= frame[i];
        frame.push_back(crc);
        return frame;
    }
};

} // namespace anti_drone
} // namespace rm_radar
