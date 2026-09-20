// sdr_replay.cpp - Offline recorded data replay tool
#include "sdr_decoder.h"
#include "kalman_interpolator.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

using namespace rm_radar;
using namespace rm_radar::sdr;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: sdr_replay <recorded_data.bin>" << std::endl;
        return 1;
    }
    std::ifstream f(argv[1], std::ios::binary);
    if (!f) {
        std::cerr << "Cannot open file: " << argv[1] << std::endl;
        return 1;
    }
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());

    SdrDecoder dec;
    dec.init();
    KalmanInterpolator ki(10, 30);

    size_t chunk = 256;
    for (size_t i = 0; i < data.size(); i += chunk) {
        size_t n = std::min(chunk, data.size() - i);
        auto targets = dec.feed(data.data() + i, n);
        for (const auto& t : targets) {
            std::cout << "Robot" << (int)t.robot_id
                      << " pos=(" << t.x << ", " << t.y << ")"
                      << " HP=" << t.hp << std::endl;
            ki.update(t);
        }
        auto interp = ki.interpolate();
        (void)interp;
    }
    std::cout << "Replay done. Frames: " << dec.framesReceived()
              << " CRC errors: " << dec.crcErrors() << std::endl;
    return 0;
}
