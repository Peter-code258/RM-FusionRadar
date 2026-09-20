// main.cpp - LiDAR perception demo with synthetic scene and PCL visualization
#include "lidar_perception.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <random>
#include <vector>

using namespace rm_radar;
using namespace rm_radar::lidar;

// Generate a synthetic point cloud scene with ground, a few target boxes, and noise
static PointCloud generateSyntheticScene(std::mt19937& rng, float frame_idx) {
    PointCloud cloud;
    cloud.timestamp_us = static_cast<uint64_t>(frame_idx * 100000);
    cloud.frame_id = "lidar";

    std::uniform_real_distribution<float> noise(-0.02f, 0.02f);

    // Ground plane (z = 0 with noise)
    for (float x = -10.0f; x <= 10.0f; x += 0.2f) {
        for (float y = -15.0f; y <= 15.0f; y += 0.2f) {
            if (std::abs(x) < 0.1f && std::abs(y) < 0.1f) continue; // skip origin
            cloud.points.emplace_back(x, y, 0.0f + noise(rng), 10.0f, 0);
        }
    }

    // Moving target 1: infantry robot (box ~0.6 x 0.4 x 0.5 m) at distance ~5m
    float tx = 2.0f + std::sin(frame_idx * 0.3f) * 1.5f;
    float ty = 5.0f;
    float tz = 0.3f;
    for (float dx = -0.3f; dx <= 0.3f; dx += 0.05f) {
        for (float dy = -0.2f; dy <= 0.2f; dy += 0.05f) {
            for (float dz = 0.0f; dz <= 0.5f; dz += 0.05f) {
                cloud.points.emplace_back(tx + dx, ty + dy, tz + dz + noise(rng), 200.0f, 1);
            }
        }
    }

    // Static target 2: hero robot at distance ~8m
    float hx = -3.0f;
    float hy = 8.0f;
    for (float dx = -0.4f; dx <= 0.4f; dx += 0.05f) {
        for (float dy = -0.3f; dy <= 0.3f; dy += 0.05f) {
            for (float dz = 0.0f; dz <= 0.6f; dz += 0.05f) {
                cloud.points.emplace_back(hx + dx, hy + dy, 0.3f + dz + noise(rng), 180.0f, 2);
            }
        }
    }

    return cloud;
}

int main(int argc, char** argv) {
    std::string config_path = "config/lidar_config.yaml";
    if (argc > 1) config_path = argv[1];

    RM_LOG_INFO("Starting LiDAR Perception subsystem");

    LidarPerception perception;
    if (!perception.loadConfig(config_path)) {
        RM_LOG_WARN("Could not load config, using defaults");
    }
    perception.init();

    // Create PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer;
    bool use_viz = perception.config().visualization_enabled;
    if (use_viz) {
        viewer = std::make_shared<pcl::visualization::PCLVisualizer>(perception.config().window_name);
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->setCameraPosition(0, -20, 15, 0, 0, 0, 0, 0, 1);
    }

    std::mt19937 rng(42);
    int frame = 0;
    const int max_frames = 200;

    while (frame < max_frames) {
        PointCloud cloud = generateSyntheticScene(rng, static_cast<float>(frame));
        PerceptionResult result = perception.process(cloud);

        // Print summary
        std::cout << "[Frame " << frame << "] points_in=" << cloud.size()
                  << " clusters=" << result.clusters.size()
                  << " targets=" << result.targets.size()
                  << " time=" << result.processing_time_ms << "ms" << std::endl;
        for (const auto& t : result.targets) {
            std::cout << "  Target #" << t.id
                      << " pos=(" << t.position.x << ", " << t.position.y << ", " << t.position.z << ")"
                      << " conf=" << t.confidence
                      << " points=" << t.point_count << std::endl;
        }

        // Visualize
        if (use_viz && viewer) {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Draw filtered point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& p : result.filtered_cloud.points) {
                pcl::PointXYZI pt;
                pt.x = p.x; pt.y = p.y; pt.z = p.z;
                pt.intensity = p.intensity;
                pcl_cloud->push_back(pt);
            }
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(pcl_cloud, "intensity");
            viewer->addPointCloud<pcl::PointXYZI>(pcl_cloud, color, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

            // Draw target bounding boxes
            int box_id = 0;
            for (const auto& t : result.targets) {
                std::string id = "box_" + std::to_string(box_id++);
                viewer->addCube(t.bbox[0], t.bbox[3], t.bbox[1], t.bbox[4], t.bbox[2], t.bbox[5],
                                1.0, 0.0, 0.0, id);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
                std::string label = "T" + std::to_string(t.id);
                viewer->addText3D(label, Eigen::Vector3f(t.position.x, t.position.y, t.position.z + 1.0),
                                  0.3, 1.0, 1.0, 1.0, "label_" + id);
            }

            viewer->spinOnce(30);
            if (viewer->wasStopped()) break;
        }

        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RM_LOG_INFO("LiDAR Perception demo finished");
    return 0;
}
