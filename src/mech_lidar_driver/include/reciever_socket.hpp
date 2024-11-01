/*
 * Copyright (c) 2024, DEPHAN LLC, Aleksandr Plukchi, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file receiver_socket.hpp
 * @brief UDP socket for receiving data packets from lidar
 */

#ifndef RECEIVER_SOCKET_HPP
#define RECEIVER_SOCKET_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <string>
#include <cstdint>

namespace dephan_ros {
class receiver_socket {
public:
    /**
     * Constructs and opens a socket with specific IP address and port.
     *
     * @param[in] config device configuration (include ip and port).
     */
    receiver_socket(std::string ip_addr, int port);

    /**
     * Destructs the UDP server - closes the socket.
     */
    ~receiver_socket();

    /**
     * Get single data packet.
     *
     * @param[in] buf buffer for packet.
     * @param[in] len packet lenght.
     */
    int get_packet(uint8_t* buf, int len);

private:
    // Disabled copy constructor
    receiver_socket(const receiver_socket&);

    // Disabled assign operator
    receiver_socket& operator= (const receiver_socket&);

    std::string m_ip_addr;
    int m_sock_port;
    int udp_socket;

    // contains info about source, destination address and port
    struct sockaddr_in si_me, si_from;
    socklen_t si_from_len = sizeof(si_from);

    const int POLL_TIMEOUT = 500; // in milliseconds
    struct pollfd m_fds[1];
};
} // namespace dephan_ros

#endif /* RECEIVER_SOCKET_HPP */
