// udp_io.h - UDP input/output for the fusion center
#pragma once

#include "fusion_center.h"
#include "common_types.h"
#include <string>
#include <vector>
#include <cstdint>
#include <thread>
#include <atomic>
#include <functional>

namespace rm_radar {
namespace fusion {

// UDP receiver that listens for sensor measurements on a port
class UdpReceiver {
public:
    using Callback = std::function<void(const SensorMeasurement&)>;

    UdpReceiver();
    ~UdpReceiver();

    bool init(int port);
    void start(Callback cb);
    void stop();

private:
    void recvLoop();
    int sockfd_ = -1;
    int port_ = 0;
    std::atomic<bool> running_{false};
    std::thread thread_;
    Callback callback_;
};

// UDP publisher that sends fused target list to combat units
class UdpPublisher {
public:
    UdpPublisher();
    ~UdpPublisher();

    bool init(const std::string& host, int port);
    bool publish(const std::vector<Target>& targets);
    void close();

private:
    int sockfd_ = -1;
    std::string host_;
    int port_ = 0;
};

} // namespace fusion
} // namespace rm_radar
