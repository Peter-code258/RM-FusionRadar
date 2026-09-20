// channel_scan.cpp - Standalone channel scanning tool
#include "channel_scanner.h"
#include "sdr_protocol.h"
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace rm_radar::sdr;

int main() {
    std::srand(std::time(nullptr));
    ChannelScanner scanner(16, 20);
    std::cout << "Scanning 16 channels for official information wave..." << std::endl;
    int best = scanner.scan([](int ch) {
        // In real use, this tunes the SDR frontend and measures RSSI
        float strength = 0.05f + static_cast<float>(rand()) / RAND_MAX * 0.1f;
        int frames = (rand() % 100 < 5) ? 1 : 0;
        return ChannelMeasurement{ch, strength, frames};
    });
    std::cout << "Best channel: " << best << std::endl;
    for (const auto& m : scanner.measurements()) {
        std::cout << "  Ch" << m.channel << ": " << m.signal_strength
                  << " frames=" << m.valid_frames << std::endl;
    }
    return 0;
}
