// sdr_decoder.cpp - SDR baseband decoder implementation
#include "sdr_decoder.h"
#include "config_loader.h"

#include <cstring>
#include <algorithm>

namespace rm_radar {
namespace sdr {

SdrDecoder::SdrDecoder() = default;
SdrDecoder::~SdrDecoder() = default;

bool SdrDecoder::loadConfig(const std::string& yaml_path) {
    Config c;
    if (!c.load(yaml_path)) {
        RM_LOG_WARN("SDR config not found: " + yaml_path + ", using defaults");
        return false;
    }
    cfg_.big_endian = c.getBool("big_endian", cfg_.big_endian);
    cfg_.verify_crc = c.getBool("verify_crc", cfg_.verify_crc);
    cfg_.use_inverted_sync = c.getBool("use_inverted_sync", cfg_.use_inverted_sync);
    cfg_.udp_host = c.get("udp_host", cfg_.udp_host);
    cfg_.udp_port = c.getInt("udp_port", cfg_.udp_port);
    cfg_.publish_udp = c.getBool("publish_udp", cfg_.publish_udp);
    cfg_.enable_interpolation = c.getBool("enable_interpolation", cfg_.enable_interpolation);
    cfg_.output_rate_hz = c.getInt("output_rate_hz", cfg_.output_rate_hz);
    cfg_.input_rate_hz = c.getInt("input_rate_hz", cfg_.input_rate_hz);
    RM_LOG_INFO("SDR decoder config loaded");
    return true;
}

bool SdrDecoder::init() {
    buffer_.reserve(cfg_.max_buffer_size);
    RM_LOG_INFO("SDR decoder initialized");
    return true;
}

bool SdrDecoder::findSync(size_t& offset) const {
    if (buffer_.size() < 2) return false;
    for (size_t i = 0; i <= buffer_.size() - 2; ++i) {
        uint16_t sync = static_cast<uint16_t>(buffer_[i]) |
                        (static_cast<uint16_t>(buffer_[i + 1]) << 8);
        if (sync == SYNC_WORD || (cfg_.use_inverted_sync && sync == SYNC_WORD_INV)) {
            offset = i;
            return true;
        }
    }
    return false;
}

bool SdrDecoder::tryExtractFrame(size_t offset, SdrFrame& out_frame) const {
    if (buffer_.size() < offset + SDR_FRAME_SIZE) return false;
    std::memcpy(&out_frame, buffer_.data() + offset, SDR_FRAME_SIZE);
    return true;
}

SdrTarget SdrDecoder::frameToTarget(const SdrFrame& f) const {
    SdrTarget t;
    bool swap = cfg_.big_endian == isLittleEndian(); // if mismatch, swap

    t.robot_id = f.robot_id;
    t.x = swap ? bswapFloat(f.x) : f.x;
    t.y = swap ? bswapFloat(f.y) : f.y;
    t.hp = swap ? bswap16(f.hp) : f.hp;
    t.ammo = swap ? bswap16(f.ammo) : f.ammo;
    t.status = swap ? bswap16(f.status) : f.status;
    t.timestamp_us = now_us();
    t.confidence = 1.0f;
    return t;
}

bool SdrDecoder::decodeFrame(const SdrFrame& frame, SdrTarget& out) {
    if (cfg_.verify_crc) {
        uint16_t calc = frameCrc(frame);
        uint16_t recv = cfg_.big_endian ? bswap16(frame.crc16) : frame.crc16;
        if (calc != recv) {
            crc_errors_++;
            return false;
        }
    }
    frames_received_++;
    out = frameToTarget(frame);
    targets_.push_back(out);
    return true;
}

std::vector<SdrTarget> SdrDecoder::feed(const uint8_t* data, size_t len) {
    std::vector<SdrTarget> results;

    // Append to buffer
    buffer_.insert(buffer_.end(), data, data + len);
    if (buffer_.size() > static_cast<size_t>(cfg_.max_buffer_size)) {
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + (buffer_.size() - cfg_.max_buffer_size));
    }

    // Repeatedly find sync and extract frames
    size_t offset = 0;
    while (findSync(offset)) {
        SdrFrame frame;
        if (!tryExtractFrame(offset, frame)) {
            break; // not enough bytes yet
        }

        SdrTarget target;
        if (decodeFrame(frame, target)) {
            sync_found_++;
            results.push_back(target);
        }

        // Remove consumed bytes up to end of this frame
        size_t consume = offset + SDR_FRAME_SIZE;
        buffer_.erase(buffer_.begin(), buffer_.begin() + consume);
    }

    return results;
}

} // namespace sdr
} // namespace rm_radar
