#include "reciever_socket.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {
int reserve_udp_port() {
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0) {
        throw std::runtime_error("socket failed");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (bind(
            socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)
        ) < 0) {
        close(socket_fd);
        throw std::runtime_error("bind failed");
    }

    socklen_t length = sizeof(address);
    if (getsockname(
            socket_fd, reinterpret_cast<sockaddr*>(&address), &length
        ) < 0) {
        close(socket_fd);
        throw std::runtime_error("getsockname failed");
    }
    const int port = ntohs(address.sin_port);
    close(socket_fd);
    return port;
}

void send_datagram(int port, const std::vector<uint8_t>& payload) {
    const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0) {
        throw std::runtime_error("socket failed");
    }

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    destination.sin_port = htons(static_cast<uint16_t>(port));
    const ssize_t sent = sendto(
        socket_fd, payload.data(), payload.size(), 0,
        reinterpret_cast<sockaddr*>(&destination), sizeof(destination)
    );
    close(socket_fd);
    if (sent != static_cast<ssize_t>(payload.size())) {
        throw std::runtime_error("sendto failed");
    }
}
} // namespace

TEST(ReceiverSocket, AcceptsExactDatagramWithoutSourceFilter) {
    const int port = reserve_udp_port();
    dephan_ros::receiver_socket receiver("0.0.0.0", port);
    const std::vector<uint8_t> input{1, 2, 3, 4, 5, 6};
    send_datagram(port, input);

    std::vector<uint8_t> output(input.size());
    EXPECT_EQ(receiver.get_packet(output.data(), output.size()), 0);
    EXPECT_EQ(output, input);
}

TEST(ReceiverSocket, RejectsWrongDatagramSize) {
    const int port = reserve_udp_port();
    dephan_ros::receiver_socket receiver("", port);
    send_datagram(port, {1, 2, 3});

    std::vector<uint8_t> output(4);
    EXPECT_EQ(receiver.get_packet(output.data(), output.size()), -1);
}

TEST(ReceiverSocket, RejectsUnexpectedSourceAddress) {
    const int port = reserve_udp_port();
    dephan_ros::receiver_socket receiver("127.0.0.2", port);
    send_datagram(port, {1, 2, 3, 4});

    std::vector<uint8_t> output(4);
    EXPECT_EQ(receiver.get_packet(output.data(), output.size()), -1);
}

TEST(ReceiverSocket, ReportsTimeoutWithoutData) {
    const int port = reserve_udp_port();
    dephan_ros::receiver_socket receiver("0.0.0.0", port);
    std::vector<uint8_t> output(4);

    EXPECT_EQ(receiver.get_packet(output.data(), output.size()), 1);
}

TEST(ReceiverSocket, ReopensAndReceivesData) {
    const int port = reserve_udp_port();
    dephan_ros::receiver_socket receiver("0.0.0.0", port);
    receiver.reopen();
    const std::vector<uint8_t> input{4, 3, 2, 1};
    send_datagram(port, input);

    std::vector<uint8_t> output(input.size());
    EXPECT_EQ(receiver.get_packet(output.data(), output.size()), 0);
    EXPECT_EQ(output, input);
}

TEST(ReceiverSocket, RejectsInvalidSourceAddress) {
    EXPECT_THROW(
        dephan_ros::receiver_socket("not-an-ip", reserve_udp_port()),
        std::invalid_argument
    );
}
