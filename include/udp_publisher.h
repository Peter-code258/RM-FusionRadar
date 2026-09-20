// udp_publisher.h - Lightweight UDP publisher for decoded SDR targets
#pragma once

#include "sdr_protocol.h"
#include <string>
#include <vector>
#include <cstdint>

namespace rm_radar {
namespace sdr {

class UdpPublisher {
public:
    UdpPublisher();
    ~UdpPublisher();

    bool init(const std::string& host, int port);
    bool publish(const std::vector<SdrTarget>& targets);
    void close();

private:
    int sockfd_ = -1;
    std::string host_;
    int port_ = 0;
};

} // namespace sdr
} // namespace rm_radar
