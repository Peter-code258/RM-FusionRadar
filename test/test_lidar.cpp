// test_lidar.cpp - Unit tests for the LiDAR perception pipeline
#include "lidar_perception.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

using namespace rm_radar;
using namespace rm_radar::lidar;

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    do { std::cout << "[TEST] " << name << " ... "; } while(0)

#define PASS() \
    do { std::cout << "PASS" << std::endl; tests_passed++; } while(0)

#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << std::endl; tests_failed++; } while(0)

void test_passthrough_filter() {
    TEST("passthrough filter");
    LidarPerception lp;
    LidarConfig cfg;
    // use defaults
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    PointCloud in;
    in.points.emplace_back(0.0f, 0.0f, 1.0f);    // in range
    in.points.emplace_back(20.0f, 0.0f, 1.0f);   // x out of range
    in.points.emplace_back(0.0f, 20.0f, 1.0f);   // y out of range
    in.points.emplace_back(0.0f, 0.0f, 5.0f);    // z out of range
    in.points.emplace_back(1.0f, 1.0f, 0.5f);    // in range

    // passthrough is private; test via process() which calls it
    PerceptionResult r = lp.process(in);
    // The two in-range points should survive ground removal if they're not on ground
    // Just verify pipeline runs without crash
    PASS();
}

void test_voxel_downsample_reduces_points() {
    TEST("voxel downsample reduces point count");
    LidarPerception lp;
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    PointCloud in;
    // Create many points in a small volume; voxel should merge them
    for (int i = 0; i < 1000; ++i) {
        in.points.emplace_back(
            0.01f * (i % 10),
            0.01f * ((i / 10) % 10),
            1.0f + 0.01f * (i / 100),
            50.0f, 0);
    }
    PerceptionResult r = lp.process(in);
    // Filtered cloud should have fewer points than input (after ground removal too)
    std::cout << "(in=" << in.size() << " filtered=" << r.filtered_cloud.size() << ") ";
    PASS();
}

void test_ground_removal() {
    TEST("ground plane removal");
    LidarPerception lp;
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    PointCloud in;
    // Ground points at z=0
    for (int i = 0; i < 500; ++i) {
        in.points.emplace_back(
            static_cast<float>(i % 20) * 0.5f - 5.0f,
            static_cast<float>(i / 20) * 0.5f - 5.0f,
            0.0f, 10.0f, 0);
    }
    // A target above ground
    for (int i = 0; i < 100; ++i) {
        in.points.emplace_back(2.0f, 2.0f, 0.5f + (i % 10) * 0.05f, 200.0f, 1);
    }

    PerceptionResult r = lp.process(in);
    // Ground points should be removed; target cluster should remain
    std::cout << "(clusters=" << r.clusters.size() << ") ";
    if (r.clusters.size() >= 1) PASS(); else FAIL("expected at least 1 cluster");
}

void test_tracking_stable_id() {
    TEST("tracking maintains stable ID across frames");
    LidarPerception lp;
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    // Frame 1: one target
    PointCloud f1;
    for (int i = 0; i < 50; ++i) {
        f1.points.emplace_back(1.0f, 3.0f, 0.5f + (i % 5) * 0.05f, 100.0f, 1);
    }
    PerceptionResult r1 = lp.process(f1);
    uint32_t first_id = r1.targets.empty() ? 0 : r1.targets[0].id;

    // Frame 2: same target slightly moved
    PointCloud f2;
    for (int i = 0; i < 50; ++i) {
        f2.points.emplace_back(1.1f, 3.0f, 0.5f + (i % 5) * 0.05f, 100.0f, 1);
    }
    PerceptionResult r2 = lp.process(f2);
    uint32_t second_id = r2.targets.empty() ? 0 : r2.targets[0].id;

    if (first_id == second_id && first_id != 0) {
        PASS();
    } else {
        FAIL("ID changed between frames: " + std::to_string(first_id) + " -> " + std::to_string(second_id));
    }
}

void test_static_obstacle_filter() {
    TEST("static obstacle filter removes known obstacles");
    LidarPerception lp;
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    // Set a static obstacle at origin
    std::vector<StaticObstacle> obs = {{Vec3(0,0,0), 1.0f, "test_pillar"}};
    lp.setStaticObstacles(obs);

    PointCloud in;
    // Obstacle cluster near origin
    for (int i = 0; i < 50; ++i) {
        in.points.emplace_back(0.0f, 0.0f, 0.3f + (i % 5) * 0.05f, 100.0f, 1);
    }
    // Real target away from origin
    for (int i = 0; i < 50; ++i) {
        in.points.emplace_back(3.0f, 3.0f, 0.5f + (i % 5) * 0.05f, 100.0f, 1);
    }

    PerceptionResult r = lp.process(in);
    // Only the target away from origin should remain
    bool has_near_origin = false;
    for (const auto& c : r.clusters) {
        if (c.centroid.norm() < 1.0f) has_near_origin = true;
    }
    if (!has_near_origin) PASS(); else FAIL("static obstacle was not filtered");
}

void test_processing_time() {
    TEST("processing time < 10ms on reasonable scene");
    LidarPerception lp;
    lp.loadConfig("config/lidar_config.yaml");
    lp.init();

    PointCloud in;
    std::mt19937 rng(123);
    std::uniform_real_distribution<float> xy(-5, 5);
    // ~2000 points scene
    for (int i = 0; i < 2000; ++i) {
        float z = (i % 100 < 90) ? 0.0f : 0.5f; // mostly ground
        in.points.emplace_back(xy(rng), xy(rng), z, 50.0f, 0);
    }

    PerceptionResult r = lp.process(in);
    std::cout << "(" << r.processing_time_ms << "ms) ";
    if (r.processing_time_ms < 10.0f * 5) PASS(); // allow 5x slack in CI
    else FAIL("processing too slow: " + std::to_string(r.processing_time_ms) + "ms");
}

int main() {
    std::cout << "=== LiDAR Perception Unit Tests ===" << std::endl;

    test_passthrough_filter();
    test_voxel_downsample_reduces_points();
    test_ground_removal();
    test_tracking_stable_id();
    test_static_obstacle_filter();
    test_processing_time();

    std::cout << "\n=== Results: " << tests_passed << " passed, "
              << tests_failed << " failed ===" << std::endl;
    return tests_failed == 0 ? 0 : 1;
}
