// main.cpp - Anti-drone tracking and countermeasure demo
#include "anti_drone_tracker.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::anti_drone;

int main(int argc, char** argv) {
    std::string config_path = "config/anti_drone_config.yaml";
    if (argc > 1) config_path = argv[1];

    RM_LOG_INFO("Starting Anti-Drone Tracking subsystem");

    AntiDroneTracker tracker;
    tracker.loadConfig(config_path);
    tracker.init();

    std::cout << "=== Anti-Drone Tracker running ===" << std::endl;

    for (int frame = 0; frame < 180; ++frame) {
        float t = frame * 0.016f;
        // Simulate drone moving in a circle
        float drone_x = 5.0f + std::cos(t * 2.0f) * 2.0f;
        float drone_y = std::sin(t * 2.0f) * 2.0f;

        // Convert world to pixel (simplified inverse projection)
        float cx = 320 + drone_x * 30;
        float cy = 240 - drone_y * 30;

        std::vector<DroneDetection> dets;
        DroneDetection d;
        d.cx = cx; d.cy = cy; d.w = 20; d.h = 20;
        d.confidence = 0.85f + 0.1f * std::sin(t * 5);
        d.is_telephoto = (tracker.stage() == TrackStage::FINE);
        dets.push_back(d);

        // Feed lidar range
        tracker.setLidarRange(8.0f);

        // Feed laser spot offset (simulated closed-loop)
        tracker.setSpotOffset(0.001f * std::sin(t * 3), 0.001f * std::cos(t * 3));

        TrackingResult r = tracker.process(dets);

        if (frame % 20 == 0) {
            std::cout << "[Frame " << frame << "] stage="
                      << (r.stage == TrackStage::COARSE ? "COARSE" : "FINE")
                      << " pos=(" << r.world_position.x << ", " << r.world_position.y << ", " << r.world_position.z << ")"
                      << " stability=" << r.stability
                      << " laser=" << (r.laser_emitting ? "ON" : "OFF")
                      << " yaw=" << r.servo_cmd.yaw << " pitch=" << r.servo_cmd.pitch
                      << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "\nLaser total emit time: " << tracker.laser().totalEmitTimeMs() << "ms" << std::endl;
    std::cout << "Countermeasure effective: " << (tracker.laser().isCountermeasureEffective() ? "YES" : "NO") << std::endl;

    RM_LOG_INFO("Anti-drone demo finished");
    return 0;
}
