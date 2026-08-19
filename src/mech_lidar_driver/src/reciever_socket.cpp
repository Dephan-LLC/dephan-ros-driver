/*
 * Copyright (c) 2024, DEPHAN LLC, Aleksandr Plukchi, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file reciever_socket.cpp
 * @brief UDP socket for receiving data packets from lidar
 */

#include "reciever_socket.hpp"

#include <fcntl.h>
#include <unistd.h> // close
#include <cstring> // memset
#include <stdexcept>
#include <system_error>

namespace dephan_ros {
receiver_socket::receiver_socket(std::string ip_addr, int port) :
    m_ip_addr(ip_addr),
    m_filter_by_source(!ip_addr.empty() && ip_addr != "0.0.0.0"),
    m_sock_port(port) {
    if (m_filter_by_source &&
        inet_pton(AF_INET, m_ip_addr.c_str(), &m_expected_addr) != 1) {
        throw std::invalid_argument("Invalid source IP address: " + m_ip_addr);
    }

    open_socket();
}

receiver_socket::~receiver_socket() {
    close_socket();
}

void receiver_socket::reopen() {
    close_socket();
    open_socket();
}

void receiver_socket::open_socket() {
    memset((char*) &si_me, 0, sizeof(si_me));
    memset((char*) &si_from, 0, sizeof(si_from));
    si_from_len = sizeof(si_from);

    si_me.sin_family      = AF_INET;
    si_me.sin_port        = htons(m_sock_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    // create a UDP socket
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket == -1) {
        throw std::system_error(
            errno, std::generic_category(), "socket() failed"
        );
    }

    try {
        int reuse = 1;
        if (setsockopt(
                udp_socket, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuse,
                sizeof(reuse)
            ) == -1) {
            throw std::system_error(
                errno, std::generic_category(),
                "setsockopt(SO_REUSEADDR) failed"
            );
        }

#ifdef SO_REUSEPORT
        if (setsockopt(
                udp_socket, SOL_SOCKET, SO_REUSEPORT, (const char*) &reuse,
                sizeof(reuse)
            ) == -1) {
            throw std::system_error(
                errno, std::generic_category(),
                "setsockopt(SO_REUSEPORT) failed"
            );
        }
#endif

        if (bind(udp_socket, (struct sockaddr*) &si_me, sizeof(si_me)) == -1) {
            throw std::system_error(
                errno, std::generic_category(), "bind() failed"
            );
        }
        if (fcntl(udp_socket, F_SETFL, O_NONBLOCK) < 0) {
            throw std::system_error(
                errno, std::generic_category(), "fcntl(O_NONBLOCK) failed"
            );
        }
    }
    catch (...) {
        close_socket();
        throw;
    }

    m_fds[0].fd     = udp_socket;
    m_fds[0].events = POLLIN;
    m_fds[0].revents = 0;
}

void receiver_socket::close_socket() noexcept {
    if (udp_socket >= 0) {
        close(udp_socket);
        udp_socket = -1;
    }
    m_fds[0].fd = -1;
    m_fds[0].revents = 0;
}

int receiver_socket::get_packet(uint8_t* buf, int len) {
    if (udp_socket < 0) {
        throw std::system_error(
            EBADF, std::generic_category(), "UDP socket is closed"
        );
    }

    // try to poll socket
    m_fds[0].revents = 0;
    int poll_return = poll(m_fds, 1, POLL_TIMEOUT);
    if (poll_return == 0) {
        // A timeout is normal while the motor is stopped. Stream health is
        // reported through the ROS diagnostics topic.
        return 1;
    }
    if (poll_return < 0) {
        if (errno == EINTR) {
            return 1;
        }
        throw std::system_error(
            errno, std::generic_category(), "poll() failed"
        );
    }
    if (m_fds[0].revents & POLLERR || m_fds[0].revents & POLLHUP ||
        m_fds[0].revents & POLLNVAL) {
        throw std::system_error(
            EIO, std::generic_category(), "poll() reported a socket error"
        );
    }
    if ((m_fds[0].revents & POLLIN) == 0) {
        return -1;
    }

    // try to receive some data, this is a non-blocking call (O_NONBLOCK)
    si_from_len = sizeof(si_from);
    int recv_len = recvfrom(
        udp_socket, buf, len, MSG_TRUNC, (struct sockaddr*) &si_from,
        &si_from_len
    );

    if (recv_len < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            throw std::system_error(
                errno, std::generic_category(), "recvfrom() failed"
            );
        }
        return 1;
    }

    if (m_filter_by_source &&
        si_from.sin_addr.s_addr != m_expected_addr.s_addr) {
        // ip mismatch
        return -1;
    }

    if (recv_len != len) {
        // incomplete package
        return -1;
    }

    return 0;
}
} // namespace dephan_ros
