// udp_publisher.cpp - UDP publisher implementation using native sockets
#include "udp_publisher.h"
#include "logger.h"

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace rm_radar {
namespace sdr {

UdpPublisher::UdpPublisher() = default;
UdpPublisher::~UdpPublisher() { close(); }

bool UdpPublisher::init(const std::string& host, int port) {
    host_ = host;
    port_ = port;
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        RM_LOG_ERROR("Failed to create UDP socket");
        return false;
    }
    RM_LOG_INFO("UDP publisher initialized -> " + host + ":" + std::to_string(port));
    return true;
}

bool UdpPublisher::publish(const std::vector<SdrTarget>& targets) {
    if (sockfd_ < 0 || targets.empty()) return false;

    // Serialize targets: [count(4)] [target0] [target1] ...
    // Use a simple binary format
    std::vector<uint8_t> buf;
    uint32_t count = static_cast<uint32_t>(targets.size());
    buf.insert(buf.end(), reinterpret_cast<uint8_t*>(&count),
               reinterpret_cast<uint8_t*>(&count) + 4);
    for (const auto& t : targets) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&t);
        buf.insert(buf.end(), p, p + sizeof(SdrTarget));
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

} // namespace sdr
} // namespace rm_radar
