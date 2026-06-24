/*
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file http_client.cpp
 * @brief Minimal HTTP client for lidar web API commands.
 */

#include "http_client.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace dephan_ros {
namespace {
class addrinfo_ptr {
public:
    explicit addrinfo_ptr(addrinfo* info) : m_info(info) {}
    ~addrinfo_ptr() {
        if (m_info) {
            freeaddrinfo(m_info);
        }
    }

    addrinfo* get() const {
        return m_info;
    }

private:
    addrinfo* m_info;
};

class socket_fd {
public:
    explicit socket_fd(int fd = -1) : m_fd(fd) {}
    ~socket_fd() {
        if (m_fd >= 0) {
            close(m_fd);
        }
    }

    int get() const {
        return m_fd;
    }

    int release() {
        int fd = m_fd;
        m_fd  = -1;
        return fd;
    }

    void reset(int fd) {
        if (m_fd >= 0) {
            close(m_fd);
        }
        m_fd = fd;
    }

private:
    int m_fd;
};

void set_timeout(int fd, int timeout_ms) {
    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0 ||
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        throw std::system_error(
            errno, std::generic_category(), "setsockopt(timeout) failed"
        );
    }
}

void send_all(int fd, const std::string& data) {
    size_t sent = 0;
    while (sent < data.size()) {
        ssize_t n = send(fd, data.data() + sent, data.size() - sent, 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::system_error(
                errno, std::generic_category(), "send() failed"
            );
        }
        if (n == 0) {
            throw std::runtime_error("send() returned zero bytes");
        }
        sent += static_cast<size_t>(n);
    }
}

HttpResponse parse_response(const std::string& raw) {
    const std::string sep = "\r\n\r\n";
    size_t body_pos       = raw.find(sep);
    if (body_pos == std::string::npos) {
        throw std::runtime_error("Malformed HTTP response: no header/body split");
    }

    HttpResponse response;
    response.headers = raw.substr(0, body_pos);
    response.body    = raw.substr(body_pos + sep.size());

    std::istringstream header_stream(response.headers);
    std::string http_version;
    header_stream >> http_version >> response.status_code;
    std::getline(header_stream, response.reason);
    if (!response.reason.empty() && response.reason[0] == ' ') {
        response.reason.erase(0, 1);
    }
    if (!response.reason.empty() && response.reason.back() == '\r') {
        response.reason.pop_back();
    }

    if (http_version.find("HTTP/") != 0 || response.status_code <= 0) {
        throw std::runtime_error("Malformed HTTP status line");
    }

    return response;
}

HttpResponse parse_header_block(const std::string& headers) {
    return parse_response(headers + "\r\n\r\n");
}
} // namespace

HttpClient::HttpClient(std::string host, int port, int timeout_ms) :
    m_host(std::move(host)), m_port(port), m_timeout_ms(timeout_ms) {
    if (m_host.empty()) {
        throw std::invalid_argument("HTTP host is empty");
    }
    if (m_port <= 0 || m_port > 65535) {
        throw std::invalid_argument("HTTP port is out of range");
    }
    if (m_timeout_ms <= 0) {
        throw std::invalid_argument("HTTP timeout must be positive");
    }
}

HttpResponse HttpClient::get(const std::string& path) const {
    return request("GET", path, "");
}

HttpResponse HttpClient::post(
    const std::string& path, const std::string& body
) const {
    return request("POST", path, body);
}

HttpResponse HttpClient::stream_get(
    const std::string& path, std::ostream& body_stream,
    std::ostream* status_stream
) const {
    if (path.empty() || path[0] != '/') {
        throw std::invalid_argument("HTTP path must start with '/'");
    }

    socket_fd sock(connect_socket());

    std::ostringstream request_stream;
    request_stream << "GET " << path << " HTTP/1.0\r\n"
                   << "Host: " << m_host << "\r\n"
                   << "Connection: close\r\n"
                   << "\r\n";

    send_all(sock.get(), request_stream.str());

    const std::string sep = "\r\n\r\n";
    std::string raw_headers;
    bool headers_complete = false;
    HttpResponse response;

    char buf[2048];
    while (true) {
        ssize_t n = recv(sock.get(), buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                throw std::runtime_error("HTTP response timeout");
            }
            throw std::system_error(
                errno, std::generic_category(), "recv() failed"
            );
        }
        if (n == 0) {
            break;
        }

        if (!headers_complete) {
            raw_headers.append(buf, static_cast<size_t>(n));
            size_t body_pos = raw_headers.find(sep);
            if (body_pos == std::string::npos) {
                continue;
            }

            headers_complete = true;
            response =
                parse_header_block(raw_headers.substr(0, body_pos));
            if (status_stream) {
                *status_stream << "HTTP " << response.status_code;
                if (!response.reason.empty()) {
                    *status_stream << " " << response.reason;
                }
                *status_stream << std::endl;
            }
            std::string first_body =
                raw_headers.substr(body_pos + sep.size());
            if (!first_body.empty()) {
                body_stream.write(first_body.data(), first_body.size());
                body_stream.flush();
            }
        }
        else {
            body_stream.write(buf, n);
            body_stream.flush();
        }
    }

    if (!headers_complete) {
        throw std::runtime_error("Malformed HTTP response: no header/body split");
    }

    return response;
}

HttpResponse HttpClient::request(
    const std::string& method, const std::string& path,
    const std::string& body
) const {
    if (path.empty() || path[0] != '/') {
        throw std::invalid_argument("HTTP path must start with '/'");
    }

    socket_fd sock(connect_socket());

    std::ostringstream request_stream;
    request_stream << method << " " << path << " HTTP/1.0\r\n"
                   << "Host: " << m_host << "\r\n"
                   << "Connection: close\r\n";
    if (method == "POST") {
        request_stream << "Content-Type: application/json\r\n"
                       << "Content-Length: " << body.size() << "\r\n";
    }
    request_stream << "\r\n" << body;

    send_all(sock.get(), request_stream.str());

    std::string raw_response;
    char buf[2048];
    while (true) {
        ssize_t n = recv(sock.get(), buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                throw std::runtime_error("HTTP response timeout");
            }
            throw std::system_error(
                errno, std::generic_category(), "recv() failed"
            );
        }
        if (n == 0) {
            break;
        }
        raw_response.append(buf, static_cast<size_t>(n));
    }

    return parse_response(raw_response);
}

int HttpClient::connect_socket() const {
    addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    std::string port = std::to_string(m_port);
    addrinfo* raw_info = nullptr;
    int rc = getaddrinfo(m_host.c_str(), port.c_str(), &hints, &raw_info);
    if (rc != 0) {
        throw std::runtime_error(
            "getaddrinfo() failed: " + std::string(gai_strerror(rc))
        );
    }

    addrinfo_ptr info(raw_info);
    std::system_error last_error(errno, std::generic_category());

    for (addrinfo* ai = info.get(); ai != nullptr; ai = ai->ai_next) {
        socket_fd sock(socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol));
        if (sock.get() < 0) {
            last_error = std::system_error(
                errno, std::generic_category(), "socket() failed"
            );
            continue;
        }

        int flags = fcntl(sock.get(), F_GETFL, 0);
        if (flags < 0 || fcntl(sock.get(), F_SETFL, flags | O_NONBLOCK) < 0) {
            last_error = std::system_error(
                errno, std::generic_category(), "fcntl(O_NONBLOCK) failed"
            );
            continue;
        }

        rc = connect(sock.get(), ai->ai_addr, ai->ai_addrlen);
        if (rc < 0 && errno != EINPROGRESS) {
            last_error = std::system_error(
                errno, std::generic_category(), "connect() failed"
            );
            continue;
        }

        pollfd pfd{};
        pfd.fd     = sock.get();
        pfd.events = POLLOUT;
        rc         = poll(&pfd, 1, m_timeout_ms);
        if (rc == 0) {
            last_error =
                std::system_error(ETIMEDOUT, std::generic_category());
            continue;
        }
        if (rc < 0) {
            last_error =
                std::system_error(errno, std::generic_category(), "poll() failed");
            continue;
        }

        int connect_error = 0;
        socklen_t len     = sizeof(connect_error);
        if (getsockopt(
                sock.get(), SOL_SOCKET, SO_ERROR, &connect_error, &len
            ) < 0) {
            last_error = std::system_error(
                errno, std::generic_category(), "getsockopt(SO_ERROR) failed"
            );
            continue;
        }
        if (connect_error != 0) {
            last_error = std::system_error(
                connect_error, std::generic_category(), "connect() failed"
            );
            continue;
        }

        if (fcntl(sock.get(), F_SETFL, flags) < 0) {
            last_error = std::system_error(
                errno, std::generic_category(), "fcntl(restore flags) failed"
            );
            continue;
        }
        set_timeout(sock.get(), m_timeout_ms);
        return sock.release();
    }

    throw last_error;
}

} // namespace dephan_ros
