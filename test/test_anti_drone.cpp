// test_anti_drone.cpp - Unit tests for anti-drone tracking
#include "anti_drone_tracker.h"
#include <iostream>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::anti_drone;

static int passed = 0, failed = 0;
#define TEST(name) std::cout << "[TEST] " << name << " ... "
#define PASS() do { std::cout << "PASS" << std::endl; passed++; } while(0)
#define FAIL(m) do { std::cout << "FAIL: " << m << std::endl; failed++; } while(0)

void test_eskf_prediction() {
    TEST("ESKF prediction maintains velocity");
    ESKF eskf;
    eskf.init(Vec3(0, 0, 0), Vec3(1, 0, 0));
    eskf.predict(1.0f);
    Vec3 pos = eskf.position();
    if (std::abs(pos.x - 1.0f) < 0.01f) PASS(); else FAIL("prediction incorrect");
}

void test_eskf_update() {
    TEST("ESKF update corrects position");
    ESKF eskf;
    eskf.init(Vec3(0, 0, 0), Vec3(0, 0, 0));
    eskf.predict(0.1f);
    eskf.update(Vec3(1, 0, 0));
    Vec3 pos = eskf.position();
    if (pos.x > 0.0f) PASS(); else FAIL("update did not move position");
}

void test_laser_starts_on_stability() {
    TEST("laser fires after sustained stability");
    LaserController laser;
    laser.reset();
    // 10 frames of high stability
    for (int i = 0; i < 12; ++i) {
        laser.update(0.9f, 100.0f);
    }
    if (laser.isEmitting()) PASS(); else FAIL("laser did not fire");
}

void test_laser_stops_on_instability() {
    TEST("laser stops when stability drops");
    LaserController laser;
    laser.reset();
    for (int i = 0; i < 12; ++i) laser.update(0.9f, 100.0f);
    bool was_on = laser.isEmitting();
    laser.update(0.1f, 100.0f);
    if (was_on && !laser.isEmitting()) PASS(); else FAIL("laser did not stop");
}

void test_servo_feedforward() {
    TEST("servo controller produces valid command");
    ServoController servo;
    ServoCommand cmd = servo.compute(Vec3(5, 0, 2), Vec3(1, 0, 0), Vec3(0,0,1.5f), 0.016f);
    if (std::isfinite(cmd.yaw) && std::isfinite(cmd.pitch)) PASS();
    else FAIL("servo command invalid");
}

void test_tracker_pipeline() {
    TEST("tracker runs full pipeline");
    AntiDroneTracker tracker;
    tracker.init();
    DroneDetection d = {320, 240, 20, 20, 0.9f, false};
    auto r = tracker.process({d});
    if (r.track_id > 0) PASS(); else FAIL("no track produced");
}

void test_pixel_to_world() {
    TEST("pixel to world conversion");
    AntiDroneTracker tracker;
    tracker.init();
    DroneDetection d = {320, 240, 20, 20, 0.9f, false};
    Vec3 w = tracker.pixelToWorld(d, 640, 480);
    if (std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z)) PASS();
    else FAIL("world coords invalid");
}

void test_optical_axis_calibration() {
    TEST("optical axis offset calibration");
    AntiDroneTracker tracker;
    tracker.init();
    tracker.setOpticalAxisOffset(0.01f, 0.01f);
    DroneDetection d = {320, 240, 20, 20, 0.9f, false};
    Vec3 w1 = tracker.pixelToWorld(d, 640, 480);
    tracker.setOpticalAxisOffset(0.5f, 0.5f);
    Vec3 w2 = tracker.pixelToWorld(d, 640, 480);
    if (std::abs(w1.x - w2.x) > 0.01f || std::abs(w1.y - w2.y) > 0.01f) PASS();
    else FAIL("axis offset had no effect");
}

int main() {
    std::cout << "=== Anti-Drone Tracking Unit Tests ===" << std::endl;
    test_eskf_prediction();
    test_eskf_update();
    test_laser_starts_on_stability();
    test_laser_stops_on_instability();
    test_servo_feedforward();
    test_tracker_pipeline();
    test_pixel_to_world();
    test_optical_axis_calibration();
    std::cout << "\n=== " << passed << " passed, " << failed << " failed ===" << std::endl;
    return failed == 0 ? 0 : 1;
}
