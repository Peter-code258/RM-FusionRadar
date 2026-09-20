// test_mono.cpp - Unit tests for monocular vision radar
#include "mono_radar.h"
#include <iostream>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::mono;

static int passed = 0, failed = 0;
#define TEST(name) std::cout << "[TEST] " << name << " ... "
#define PASS() do { std::cout << "PASS" << std::endl; passed++; } while(0)
#define FAIL(m) do { std::cout << "FAIL: " << m << std::endl; failed++; } while(0)

void test_homography_calibration() {
    TEST("homography calibration with RANSAC");
    MonoRadar radar;
    radar.init();
    // 8 correspondences
    std::vector<PixelPoint> pixels = {{100,400}, {540,400}, {320,300}, {200,350},
                                       {440,350}, {320,200}, {150,250}, {490,250}};
    std::vector<cv::Point2f> world = {{-4,0}, {4,0}, {0,3}, {-2,1},
                                        {2,1}, {0,6}, {-3,4}, {3,4}};
    bool ok = radar.calibrateHomography(pixels, world);
    if (ok && !radar.homography().empty()) PASS(); else FAIL("calibration failed");
}

void test_pixel_to_world_mapping() {
    TEST("pixel to world coordinate mapping");
    MonoRadar radar;
    radar.init();
    std::vector<PixelPoint> pixels = {{0,0}, {100,0}, {0,100}, {100,100}};
    std::vector<cv::Point2f> world = {{0,0}, {1,0}, {0,1}, {1,1}};
    radar.calibrateHomography(pixels, world);

    cv::Point2f wp = radar.pixelToWorld({50, 50});
    // Point (50,50) should map near (0.5, 0.5)
    if (std::abs(wp.x - 0.5f) < 0.1f && std::abs(wp.y - 0.5f) < 0.1f) {
        PASS();
    } else {
        FAIL("mapping incorrect: (" + std::to_string(wp.x) + ", " + std::to_string(wp.y) + ")");
    }
}

void test_ground_contact_point() {
    TEST("ground contact point extraction");
    MonoRadar radar;
    radar.init();
    DetectionBox det = {100, 100, 200, 300, 0.9f, 0};
    // Bottom center should be ((100+200)/2, 300) = (150, 300)
    PixelPoint cp = radar.groundContactPoint(det);
    if (std::abs(cp.u - 150) < 0.01f && std::abs(cp.v - 300) < 0.01f) {
        PASS();
    } else {
        FAIL("contact point incorrect");
    }
}

void test_blind_spot_detection() {
    TEST("blind spot obstacle detection");
    MonoRadar radar;
    radar.init();
    ObstacleModel obs;
    obs.polygon_pixels = {{0,0}, {100,0}, {100,100}, {0,100}};
    radar.setObstacles({obs});

    if (radar.isInBlindSpot({50, 50}) && !radar.isInBlindSpot({200, 200})) {
        PASS();
    } else {
        FAIL("blind spot detection incorrect");
    }
}

void test_tracking_stable_id() {
    TEST("tracking maintains stable ID");
    MonoRadar radar;
    radar.init();
    std::vector<PixelPoint> pixels = {{0,0}, {100,0}, {0,100}, {100,100}};
    std::vector<cv::Point2f> world = {{0,0}, {1,0}, {0,1}, {1,1}};
    radar.calibrateHomography(pixels, world);

    auto r1 = radar.process({{0, 0, 10, 10, 0.9f, 0}});
    uint32_t id1 = r1.empty() ? 0 : r1[0].id;
    auto r2 = radar.process({{1, 1, 11, 11, 0.9f, 0}});
    uint32_t id2 = r2.empty() ? 0 : r2[0].id;
    if (id1 == id2 && id1 != 0) PASS(); else FAIL("ID changed");
}

void test_distance_compensation() {
    TEST("distance compensation factor increases with distance");
    MonoRadar radar;
    radar.init();
    float near = radar.distanceCompensation(1.0f);
    float far = radar.distanceCompensation(20.0f);
    if (far > near) PASS(); else FAIL("compensation should increase with distance");
}

int main() {
    std::cout << "=== Mono Vision Radar Unit Tests ===" << std::endl;
    test_homography_calibration();
    test_pixel_to_world_mapping();
    test_ground_contact_point();
    test_blind_spot_detection();
    test_tracking_stable_id();
    test_distance_compensation();
    std::cout << "\n=== " << passed << " passed, " << failed << " failed ===" << std::endl;
    return failed == 0 ? 0 : 1;
}
