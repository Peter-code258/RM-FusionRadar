// sdr_decoder.h - SDR baseband bitstream decoder with frame sync, CRC, protocol unpack
#pragma once

#include "sdr_protocol.h"
#include "common_types.h"
#include "logger.h"

#include <vector>
#include <deque>
#include <cstdint>
#include <string>

namespace rm_radar {
namespace sdr {

// Decoder configuration
struct SdrConfig {
    bool big_endian = false;        // byte order of incoming frames
    bool verify_crc = true;         // enable CRC-16 verification
    int  max_buffer_size = 4096;    // byte buffer size for sync search
    bool use_inverted_sync = true;  // also accept 0x55AA as sync (anti-interference)
    // UDP publish
    std::string udp_host = "127.0.0.1";
    int udp_port = 8801;
    bool publish_udp = true;
    // Interpolation
    bool enable_interpolation = true;
    int output_rate_hz = 30;        // interpolated output rate
    int input_rate_hz = 10;         // official info wave rate
};

// Main SDR decoder
class SdrDecoder {
public:
    SdrDecoder();
    ~SdrDecoder();

    bool loadConfig(const std::string& yaml_path);
    bool init();

    // Feed raw bytes from GNU Radio baseband output
    // Returns decoded targets found in this chunk
    std::vector<SdrTarget> feed(const uint8_t* data, size_t len);

    // Feed a complete frame (for testing / known frame input)
    bool decodeFrame(const SdrFrame& frame, SdrTarget& out);

    // Get all targets decoded since last clear
    const std::vector<SdrTarget>& targets() const { return targets_; }
    void clearTargets() { targets_.clear(); }

    // Statistics
    size_t framesReceived() const { return frames_received_; }
    size_t crcErrors() const { return crc_errors_; }
    size_t syncFound() const { return sync_found_; }

    const SdrConfig& config() const { return cfg_; }

private:
    // Search for sync word in the byte buffer
    bool findSync(size_t& offset) const;
    // Try to extract a frame starting at buffer offset
    bool tryExtractFrame(size_t offset, SdrFrame& out_frame) const;
    // Convert frame bytes to target (handle endianness)
    SdrTarget frameToTarget(const SdrFrame& f) const;

    SdrConfig cfg_;
    std::vector<uint8_t> buffer_;
    std::vector<SdrTarget> targets_;

    size_t frames_received_ = 0;
    size_t crc_errors_ = 0;
    size_t sync_found_ = 0;
};

} // namespace sdr
} // namespace rm_radar
