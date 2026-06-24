/*
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file http_client.hpp
 * @brief Minimal HTTP client for lidar web API commands.
 */

#ifndef HTTP_CLIENT_HPP
#define HTTP_CLIENT_HPP

#include <ostream>
#include <string>

namespace dephan_ros {

struct HttpResponse {
    int status_code = 0;
    std::string reason;
    std::string headers;
    std::string body;
};

class HttpClient {
public:
    HttpClient(std::string host, int port = 80, int timeout_ms = 3000);

    HttpResponse get(const std::string& path) const;
    HttpResponse post(const std::string& path, const std::string& body) const;
    HttpResponse stream_get(
        const std::string& path, std::ostream& body_stream,
        std::ostream* status_stream = nullptr
    ) const;

private:
    HttpResponse request(
        const std::string& method, const std::string& path,
        const std::string& body
    ) const;

    int connect_socket() const;

    std::string m_host;
    int m_port;
    int m_timeout_ms;
};

} // namespace dephan_ros

#endif /* HTTP_CLIENT_HPP */
