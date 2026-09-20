// main.cpp - Monocular vision radar demo with synthetic detections
#include "mono_radar.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace rm_radar;
using namespace rm_radar::mono;

int main(int argc, char** argv) {
    std::string config_path = "config/mono_config.yaml";
    if (argc > 1) config_path = argv[1];

    RM_LOG_INFO("Starting Monocular Vision Radar");

    MonoRadar radar;
    radar.loadConfig(config_path);
    radar.init();

    // Set up synthetic camera intrinsics
    cv::Mat K = (cv::Mat_<double>(3,3) << 800, 0, 320,  0, 800, 240,  0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(1,5) << 0.1, -0.05, 0, 0, 0);
    radar.setCamera(K, dist, 640, 480);

    // Calibrate homography with synthetic correspondences
    std::vector<PixelPoint> pixels = {{100,400}, {540,400}, {320,300}, {200,350}, {440,350}, {320,200}, {150,250}, {490,250}};
    std::vector<cv::Point2f> world = {{-4,0}, {4,0}, {0,3}, {-2,1}, {2,1}, {0,6}, {-3,4}, {3,4}};
    radar.calibrateHomography(pixels, world);

    // Set up obstacle blind spots
    std::vector<ObstacleModel> obstacles;
    ObstacleModel obs;
    obs.polygon_pixels = {{300, 280}, {340, 280}, {340, 320}, {300, 320}};
    obstacles.push_back(obs);
    radar.setObstacles(obstacles);

    std::cout << "=== Mono Radar running ===" << std::endl;

    for (int frame = 0; frame < 120; ++frame) {
        // Synthetic moving target
        float u = 320 + std::sin(frame * 0.1f) * 100;
        float v = 350 + std::cos(frame * 0.08f) * 30;
        std::vector<DetectionBox> dets = {{u - 20, v - 40, u + 20, v, 0.9f, 0}};

        cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC3);
        // Draw blind spot
        cv::rectangle(canvas, cv::Rect(300, 280, 40, 40), cv::Scalar(0, 165, 255), 2);
        // Draw detection
        cv::rectangle(canvas, cv::Rect((int)dets[0].x1, (int)dets[0].y1, 40, 40), cv::Scalar(0, 255, 0), 2);

        auto targets = radar.process(dets, canvas);
        radar.checkAttitude(canvas);

        if (frame % 15 == 0) {
            std::cout << "[Frame " << frame << "] targets=" << targets.size()
                      << " attitude_dev=" << radar.attitudeDeviation() << "deg" << std::endl;
            for (const auto& t : targets) {
                std::cout << "  Target #" << t.id
                          << " world=(" << t.position.x << ", " << t.position.y << ")"
                          << " conf=" << t.confidence << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    RM_LOG_INFO("Mono radar demo finished");
    return 0;
}
