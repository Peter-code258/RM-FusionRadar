// channel_scanner.h - Pre-match automatic channel scanner for RM official info wave
#pragma once

#include <vector>
#include <cstdint>
#include <functional>
#include <string>

namespace rm_radar {
namespace sdr {

// Signal strength measurement for a single channel
struct ChannelMeasurement {
    int channel = 0;        // channel index (0-15)
    float signal_strength = 0.0f;  // RSSI or correlation score
    int valid_frames = 0;   // number of valid frames decoded on this channel
};

// Scans 16 channels and locks on the one with the strongest official signal.
// Uses a pluggable measurement callback to adapt to different SDR frontends.
class ChannelScanner {
public:
    // Callback: tune to channel, return signal strength and valid frame count
    using MeasureFunc = std::function<ChannelMeasurement(int channel)>;

    ChannelScanner(int num_channels = 16, int samples_per_channel = 50)
        : num_channels_(num_channels), samples_per_channel_(samples_per_channel) {}

    // Run the full scan; returns the best channel index
    int scan(MeasureFunc measure) {
        measurements_.clear();
        int best_channel = -1;
        float best_score = -1.0f;

        for (int ch = 0; ch < num_channels_; ++ch) {
            ChannelMeasurement m{ch, 0.0f, 0};
            // Average over multiple samples
            for (int s = 0; s < samples_per_channel_; ++s) {
                ChannelMeasurement sm = measure(ch);
                m.signal_strength += sm.signal_strength;
                m.valid_frames += sm.valid_frames;
            }
            m.signal_strength /= samples_per_channel_;
            measurements_.push_back(m);

            // Score combines signal strength and valid frame count
            float score = m.signal_strength + static_cast<float>(m.valid_frames) * 10.0f;
            if (score > best_score) {
                best_score = score;
                best_channel = ch;
            }
        }
        locked_channel_ = best_channel;
        return best_channel;
    }

    int lockedChannel() const { return locked_channel_; }
    const std::vector<ChannelMeasurement>& measurements() const { return measurements_; }

private:
    int num_channels_;
    int samples_per_channel_;
    int locked_channel_ = -1;
    std::vector<ChannelMeasurement> measurements_;
};

} // namespace sdr
} // namespace rm_radar
