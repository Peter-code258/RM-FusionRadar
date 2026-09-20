// udp_io.cpp - UDP I/O implementation
#include "udp_io.h"
#include "logger.h"

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace rm_radar {
namespace fusion {

UdpReceiver::UdpReceiver() = default;
UdpReceiver::~UdpReceiver() { stop(); }

bool UdpReceiver::init(int port) {
    port_ = port;
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) return false;
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        return false;
    }
    return true;
}

void UdpReceiver::start(Callback cb) {
    callback_ = cb;
    running_ = true;
    thread_ = std::thread(&UdpReceiver::recvLoop, this);
}

void UdpReceiver::recvLoop() {
    std::vector<uint8_t> buf(65536);
    while (running_) {
        ssize_t n = recvfrom(sockfd_, buf.data(), buf.size(), 0, nullptr, nullptr);
        if (n <= 0) continue;
        // Parse: [source(1)][count(4)][target0]...
        if (n < 5) continue;
        SensorSource src = static_cast<SensorSource>(buf[0]);
        uint32_t count;
        std::memcpy(&count, buf.data() + 1, 4);
        size_t target_size = sizeof(Target);
        for (uint32_t i = 0; i < count; ++i) {
            size_t off = 5 + i * target_size;
            if (off + target_size > (size_t)n) break;
            Target t;
            std::memcpy(&t, buf.data() + off, target_size);
            SensorMeasurement m;
            m.source = src;
            m.target = t;
            m.timestamp_us = t.timestamp_us;
            m.sensor_confidence = t.confidence;
            if (callback_) callback_(m);
        }
    }
}

void UdpReceiver::stop() {
    running_ = false;
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
}

UdpPublisher::UdpPublisher() = default;
UdpPublisher::~UdpPublisher() { close(); }

bool UdpPublisher::init(const std::string& host, int port) {
    host_ = host;
    port_ = port;
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    return sockfd_ >= 0;
}

bool UdpPublisher::publish(const std::vector<Target>& targets) {
    if (sockfd_ < 0) return false;
    std::vector<uint8_t> buf;
    uint32_t count = static_cast<uint32_t>(targets.size());
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&count),
               reinterpret_cast<uint8_t*>(&count) + 4);
    for (const auto& t : targets) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&t);
        buf.insert(buf.end(), p, p + sizeof(Target));
    }
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
    ssize_t sent = sendto(sockfd_, buf.data(), buf.size(), 0,
                          reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    return sent > 0;
}

void UdpPublisher::close() {
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

} // namespace fusion
} // namespace rm_radar
