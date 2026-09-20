// test_fusion.cpp - Unit tests for the fusion center
#include "fusion_center.h"
#include <iostream>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::fusion;

static int passed = 0, failed = 0;
#define TEST(name) std::cout << "[TEST] " << name << " ... "
#define PASS() do { std::cout << "PASS" << std::endl; passed++; } while(0)
#define FAIL(m) do { std::cout << "FAIL: " << m << std::endl; failed++; } while(0)

static Target mk(float x, float y, float conf) {
    Target t;
    t.id = 0;
    t.position = Vec3(x, y, 0.5f);
    t.velocity = Vec3(0, 0, 0);
    t.confidence = conf;
    t.timestamp_us = now_us();
    return t;
}

void test_coordinate_transform() {
    TEST("coordinate transform chain");
    FusionCenter fc;
    fc.init();
    std::array<float, 16> s2b = {1,0,0,1.0f, 0,1,0,2.0f, 0,0,1,0, 0,0,0,1};
    std::array<float, 16> b2w = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, s2b, b2w);

    SensorMeasurement m;
    m.source = SensorSource::LIDAR;
    m.target = mk(0, 0, 1.0f);
    m.timestamp_us = now_us();
    m.sensor_confidence = 1.0f;
    fc.feedMeasurement(m);

    // After transform, position should be (1, 2, 0.5)
    auto out = fc.process();
    if (!out.empty() && std::abs(out[0].position.x - 1.0f) < 0.01f &&
        std::abs(out[0].position.y - 2.0f) < 0.01f) {
        PASS();
    } else {
        FAIL("transform incorrect");
    }
}

void test_fusion_combines_sensors() {
    TEST("fusion combines lidar + vision + SDR");
    FusionCenter fc;
    fc.init();
    std::array<float, 16> I = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, I, I);
    fc.setTransform(SensorSource::VISION, I, I);
    fc.setTransform(SensorSource::SDR, I, I);

    fc.feedMeasurement({SensorSource::LIDAR, mk(3.0f, 5.0f, 0.8f), now_us(), 0.8f});
    fc.feedMeasurement({SensorSource::VISION, mk(3.0f, 5.0f, 0.7f), now_us(), 0.7f});
    fc.feedMeasurement({SensorSource::SDR, mk(3.0f, 5.0f, 0.95f), now_us(), 0.95f});

    auto out = fc.process();
    if (out.size() == 1) {
        float err = std::abs(out[0].position.x - 3.0f) + std::abs(out[0].position.y - 5.0f);
        if (err < 0.1f) PASS(); else FAIL("fusion error too large: " + std::to_string(err));
    } else {
        FAIL("expected 1 fused target, got " + std::to_string(out.size()));
    }
}

void test_association_groups_same_target() {
    TEST("association groups same target from multiple sensors");
    FusionCenter fc;
    fc.init();
    std::array<float, 16> I = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, I, I);
    fc.setTransform(SensorSource::VISION, I, I);

    // Two sensors seeing the same target (within match threshold)
    fc.feedMeasurement({SensorSource::LIDAR, mk(1.0f, 1.0f, 0.8f), now_us(), 0.8f});
    fc.feedMeasurement({SensorSource::VISION, mk(1.05f, 1.02f, 0.7f), now_us(), 0.7f});

    auto out = fc.process();
    if (out.size() == 1) PASS(); else FAIL("expected 1 fused target, got " + std::to_string(out.size()));
}

void test_degradation_filters_sensors() {
    TEST("degradation filters failed sensors");
    FusionCenter fc;
    fc.init();
    fc.forceDegradation(DegradationLevel::SDR_ONLY);

    std::array<float, 16> I = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, I, I);
    fc.setTransform(SensorSource::SDR, I, I);

    // Feed lidar and SDR; in SDR_ONLY mode, lidar should be ignored
    fc.feedMeasurement({SensorSource::LIDAR, mk(10.0f, 10.0f, 0.8f), now_us(), 0.8f});
    fc.feedMeasurement({SensorSource::SDR, mk(1.0f, 1.0f, 0.95f), now_us(), 0.95f});

    auto out = fc.process();
    // Only SDR target should survive
    if (out.size() == 1 && std::abs(out[0].position.x - 1.0f) < 0.01f) {
        PASS();
    } else {
        FAIL("degradation did not filter correctly");
    }
}

void test_confidence_decay_removes_ghost() {
    TEST("low confidence targets are removed");
    FusionCenter fc;
    fc.init();
    std::array<float, 16> I = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, I, I);

    fc.feedMeasurement({SensorSource::LIDAR, mk(1.0f, 1.0f, 0.8f), now_us(), 0.8f});
    fc.process(); // creates target with conf ~0.5

    // Process several cycles with no new measurements -> confidence decays
    for (int i = 0; i < 20; ++i) fc.process();

    auto out = fc.process();
    if (out.empty()) PASS(); else FAIL("ghost target not removed");
}

void test_fault_detection() {
    TEST("sensor fault detection via timeout");
    FusionCenter fc;
    fc.init();
    // Feed lidar then check health
    std::array<float, 16> I = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    fc.setTransform(SensorSource::LIDAR, I, I);
    fc.feedMeasurement({SensorSource::LIDAR, mk(1,1,0.8f), now_us(), 0.8f});
    fc.process();
    if (fc.isSensorHealthy(SensorSource::LIDAR)) PASS(); else FAIL("lidar should be healthy");
}

int main() {
    std::cout << "=== Fusion Center Unit Tests ===" << std::endl;
    test_coordinate_transform();
    test_fusion_combines_sensors();
    test_association_groups_same_target();
    test_degradation_filters_sensors();
    test_confidence_decay_removes_ghost();
    test_fault_detection();
    std::cout << "\n=== " << passed << " passed, " << failed << " failed ===" << std::endl;
    return failed == 0 ? 0 : 1;
}
