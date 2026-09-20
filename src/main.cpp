// main.cpp - SDR decoder demo with synthetic frame generation and UDP publishing
#include "sdr_decoder.h"
#include "kalman_interpolator.h"
#include "udp_publisher.h"
#include "channel_scanner.h"
#include "config_loader.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <vector>

using namespace rm_radar;
using namespace rm_radar::sdr;

// Build a synthetic valid frame for testing
static SdrFrame buildFrame(uint8_t robot_id, float x, float y,
                           uint16_t hp, uint16_t ammo, uint16_t status) {
    SdrFrame f;
    f.sync = SYNC_WORD;
    f.length = static_cast<uint16_t>(SDR_PAYLOAD_SIZE);
    f.robot_id = robot_id;
    f.x = x;
    f.y = y;
    f.hp = hp;
    f.ammo = ammo;
    f.status = status;
    f.crc16 = frameCrc(f);
    return f;
}

int main(int argc, char** argv) {
    std::string config_path = "config/sdr_config.yaml";
    if (argc > 1) config_path = argv[1];

    RM_LOG_INFO("Starting SDR Radio Decoder subsystem");

    SdrDecoder decoder;
    decoder.loadConfig(config_path);
    decoder.init();

    KalmanInterpolator interpolator(decoder.config().input_rate_hz,
                                    decoder.config().output_rate_hz);

    UdpPublisher publisher;
    if (decoder.config().publish_udp) {
        publisher.init(decoder.config().udp_host, decoder.config().udp_port);
    }

    // Demo channel scan (synthetic measurements)
    std::cout << "=== Pre-match Channel Scan ===" << std::endl;
    ChannelScanner scanner(16, 10);
    int best = scanner.scan([](int ch) {
        // Simulate: channel 7 is the official channel
        float strength = (ch == 7) ? 0.9f : 0.1f + static_cast<float>(rand()) / RAND_MAX * 0.05f;
        int frames = (ch == 7) ? 5 : 0;
        return ChannelMeasurement{ch, strength, frames};
    });
    std::cout << "Locked channel: " << best << std::endl;
    for (const auto& m : scanner.measurements()) {
        std::cout << "  Ch " << m.channel << ": strength=" << m.signal_strength
                  << " frames=" << m.valid_frames << std::endl;
    }

    // Simulate 10Hz official data stream for 5 seconds
    std::cout << "\n=== Decoding synthetic 10Hz stream ===" << std::endl;
    const float dt = 0.1f; // 100ms
    float t = 0.0f;
    int total_frames = 50;

    for (int frame = 0; frame < total_frames; ++frame) {
        // Two robots moving
        SdrFrame f1 = buildFrame(1, 3.0f + std::sin(t) * 2.0f, 5.0f + std::cos(t) * 1.0f, 2000, 50, 0x01);
        SdrFrame f2 = buildFrame(2, -2.0f + std::cos(t * 0.7f) * 1.5f, 8.0f, 1500, 30, 0x02);

        // Feed as raw bytes
        std::vector<uint8_t> bytes;
        auto appendFrame = [&](const SdrFrame& f) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(&f);
            bytes.insert(bytes.end(), p, p + sizeof(SdrFrame));
        };
        appendFrame(f1);
        appendFrame(f2);

        auto targets = decoder.feed(bytes.data(), bytes.size());

        std::cout << "[Frame " << frame << "] decoded " << targets.size() << " targets";
        for (const auto& tgt : targets) {
            std::cout << " | Robot" << (int)tgt.robot_id
                      << " at (" << tgt.x << ", " << tgt.y << ")"
                      << " HP=" << tgt.hp << " Ammo=" << tgt.ammo;
            interpolator.update(tgt);
        }
        std::cout << std::endl;

        // Interpolate to 30Hz (3 output samples per input)
        for (int i = 0; i < 3; ++i) {
            auto interp = interpolator.interpolate();
            if (decoder.config().publish_udp) {
                publisher.publish(interp);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }

        t += dt;
    }

    std::cout << "\n=== Statistics ===" << std::endl;
    std::cout << "Frames received: " << decoder.framesReceived() << std::endl;
    std::cout << "CRC errors: " << decoder.crcErrors() << std::endl;
    std::cout << "Sync found: " << decoder.syncFound() << std::endl;

    RM_LOG_INFO("SDR decoder demo finished");
    return 0;
}
