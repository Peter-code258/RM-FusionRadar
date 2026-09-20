// main.cpp - Multi-sensor fusion center demo
#include "fusion_center.h"
#include "udp_io.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::fusion;

static Target makeTarget(uint32_t id, float x, float y, float vx, float vy, float conf) {
    Target t;
    t.id = id;
    t.type = TargetType::INFANTRY;
    t.position = Vec3(x, y, 0.5f);
    t.velocity = Vec3(vx, vy, 0);
    t.confidence = conf;
    t.timestamp_us = now_us();
    return t;
}

int main(int argc, char** argv) {
    std::string config_path = "config/fusion_config.yaml";
    if (argc > 1) config_path = argv[1];

    RM_LOG_INFO("Starting Multi-Sensor Fusion Center");

    FusionCenter center;
    center.loadConfig(config_path);
    center.init();

    // Set up identity transforms for demo
    std::array<float, 16> identity = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    center.setTransform(SensorSource::LIDAR, identity, identity);
    center.setTransform(SensorSource::VISION, identity, identity);
    center.setTransform(SensorSource::SDR, identity, identity);

    // UDP output to combat units
    UdpPublisher publisher;
    publisher.init("127.0.0.1", 8900);

    std::cout << "=== Fusion Center running ===" << std::endl;
    int cycle = 0;

    while (cycle < 100) {
        // Simulate sensor inputs
        float t = cycle * 0.1f;
        float x = 3.0f + std::sin(t) * 2.0f;
        float y = 5.0f;

        // LiDAR measurement (slightly noisy)
        center.feedMeasurement({SensorSource::LIDAR, makeTarget(0, x + 0.05f, y + 0.03f, 0.2f, 0, 0.8f), now_us(), 0.8f});
        // Vision measurement
        center.feedMeasurement({SensorSource::VISION, makeTarget(0, x - 0.04f, y + 0.02f, 0.2f, 0, 0.7f), now_us(), 0.7f});
        // SDR measurement (high confidence)
        if (cycle % 3 == 0) {
            center.feedMeasurement({SensorSource::SDR, makeTarget(0, x, y, 0.2f, 0, 0.95f), now_us(), 0.95f});
        }

        // Run fusion
        auto targets = center.process();

        // Debug terminal output
        if (cycle % 10 == 0) {
            std::cout << "[" << cycle << "] " << center.statusString() << std::endl;
            for (const auto& t : targets) {
                std::cout << "  Fused Target #" << t.id
                          << " pos=(" << t.position.x << ", " << t.position.y << ")"
                          << " conf=" << t.confidence << std::endl;
            }
        }

        // Publish fused targets
        publisher.publish(targets);

        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Test degradation
    std::cout << "\n=== Testing degradation ===" << std::endl;
    center.forceDegradation(DegradationLevel::SDR_ONLY);
    center.feedMeasurement({SensorSource::LIDAR, makeTarget(0, 1, 1, 0, 0, 0.8f), now_us(), 0.8f});
    center.feedMeasurement({SensorSource::SDR, makeTarget(0, 1.01f, 1.01f, 0, 0, 0.95f), now_us(), 0.95f});
    auto out = center.process();
    std::cout << center.statusString() << " -> " << out.size() << " targets" << std::endl;

    RM_LOG_INFO("Fusion center demo finished");
    return 0;
}
