/*
 * Copyright (c) 2024, DEPHAN LLC, Aleksandr Plukchi, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file receiver_socket.cpp
 * @brief UDP socket for receiving data packets from lidar
 */

#include "reciever_socket.hpp"

#include <fcntl.h>
#include <unistd.h> // close
#include <iostream>
#include <cstring> // memset
#include <system_error>

namespace dephan_ros {
receiver_socket::receiver_socket(std::string ip_addr, int port) :
    m_ip_addr(ip_addr), m_sock_port(port) {
    // zero out the structure
    memset((char*) &si_me, 0, sizeof(si_me));

    si_me.sin_family      = AF_INET;
    si_me.sin_port        = htons(m_sock_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    // create a UDP socket
    if ((udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        throw std::system_error(
            errno, std::generic_category(), "socket() failed"
        );
    }

    int reuse = 1;
    if (setsockopt(
            udp_socket, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuse,
            sizeof(reuse)
        ) == -1) {
        throw std::system_error(
            errno, std::generic_category(), "setsockopt(SO_REUSEADDR) failed"
        );
    }

#ifdef SO_REUSEPORT
    if (setsockopt(
            udp_socket, SOL_SOCKET, SO_REUSEPORT, (const char*) &reuse,
            sizeof(reuse)
        ) == -1) {
        throw std::system_error(
            errno, std::generic_category(), "setsockopt(SO_REUSEPORT) failed"
        );
    }
#endif

    // bind socket to port
    if (bind(udp_socket, (struct sockaddr*) &si_me, sizeof(si_me)) == -1) {
        throw std::system_error(
            errno, std::generic_category(), "bind() failed"
        );
    }
    // set O_NONBLOCK flag to avoid recvfrom hang
    if (fcntl(udp_socket, F_SETFL, O_NONBLOCK) < 0) {
        throw std::system_error(
            errno, std::generic_category(), "fcntl(O_NONBLOCK) failed"
        );
    }

    m_fds[0].fd     = udp_socket;
    m_fds[0].events = POLLIN;
}

receiver_socket::~receiver_socket() {
    if (close(udp_socket) == -1) {
        perror("close() failed");
    }
}

int receiver_socket::get_packet(uint8_t* buf, int len) {
    // try to poll socket
    int poll_return = poll(m_fds, 1, POLL_TIMEOUT);
    if (poll_return == 0) {
        // poll() timeout
        std::cerr << "poll() : timeout" << std::endl;
        return 1;
    }
    if (poll_return < 0) {
        if (errno != EINTR) {
            throw std::system_error(
                errno, std::generic_category(), "poll() failed"
            );
        }
    }
    if (m_fds[0].revents & POLLERR || m_fds[0].revents & POLLHUP ||
        m_fds[0].revents & POLLNVAL) {
        std::cerr << "poll() : device error" << std::endl;
        return -1;
    }
    if ((m_fds[0].revents & POLLIN) == 0) {
        std::cerr << "poll() : unexpected event occured" << std::endl;
        return -1;
    }

    // try to receive some data, this is a non-blocking call (O_NONBLOCK)
    int recv_len = recvfrom(
        udp_socket, buf, len, MSG_TRUNC, (struct sockaddr*) &si_from,
        &si_from_len
    );

    if (si_from.sin_addr.s_addr != inet_addr(m_ip_addr.c_str())) {
        // ip mismatch
        return -1;
    }

    if (recv_len < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
            throw std::system_error(
                errno, std::generic_category(), "recvfrom() failed"
            );
        }
        return -1;
    }

    if (recv_len != len) {
        // incomplete package
        return -1;
    }

    return 0;
}
} // namespace dephan_ros
