// calibration_tool.cpp - Interactive homography calibration tool
#include "mono_radar.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>

using namespace rm_radar;
using namespace rm_radar::mono;

// Mouse callback to collect pixel points
struct CalibState {
    std::vector<PixelPoint> pixels;
    std::vector<cv::Point2f> world_pts;
    MonoRadar* radar;
    cv::Mat image;
};

static void onMouse(int event, int x, int y, int flags, void* userdata) {
    auto* s = static_cast<CalibState*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        s->pixels.push_back({(float)x, (float)y});
        std::cout << "Pixel point " << s->pixels.size() << ": (" << x << ", " << y << ")" << std::endl;
        cv::circle(s->image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Calibration", s->image);
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: calibration_tool <image_or_video> [config.yaml]" << std::endl;
        return 1;
    }

    MonoRadar radar;
    radar.loadConfig(argc > 2 ? argv[2] : "config/mono_config.yaml");
    radar.init();

    // Load test image
    cv::Mat img = cv::imread(argv[1]);
    if (img.empty()) {
        std::cerr << "Cannot load image: " << argv[1] << std::endl;
        return 1;
    }

    std::cout << "=== Homography Calibration Tool ===" << std::endl;
    std::cout << "Click on known field points. Enter world coords (x y) after each click." << std::endl;
    std::cout << "Press 'c' to calibrate, 'q' to quit." << std::endl;

    CalibState state;
    state.radar = &radar;
    state.image = img.clone();

    cv::namedWindow("Calibration");
    cv::setMouseCallback("Calibration", onMouse, &state);

    std::vector<cv::Point2f> world_pts;
    while (true) {
        cv::imshow("Calibration", state.image);
        int key = cv::waitKey(10);
        if (key == 'q') break;
        if (key == 'c') {
            if (state.pixels.size() < 4) {
                std::cout << "Need at least 4 points, have " << state.pixels.size() << std::endl;
                continue;
            }
            // For demo, auto-generate world coords if user didn't provide
            if (world_pts.size() != state.pixels.size()) {
                world_pts.clear();
                for (size_t i = 0; i < state.pixels.size(); ++i) {
                    world_pts.emplace_back(i * 1.0f, i * 1.0f);
                }
            }
            if (radar.calibrateHomography(state.pixels, world_pts)) {
                std::cout << "Calibration successful! Homography matrix:" << std::endl;
                std::cout << radar.homography() << std::endl;
                // Save to file
                cv::FileStorage fs("homography.xml", cv::FileStorage::WRITE);
                fs << "H" << radar.homography();
                fs.release();
                std::cout << "Saved to homography.xml" << std::endl;
            }
        }
    }
    return 0;
}
