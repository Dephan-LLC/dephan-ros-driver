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

/**
 * @brief Parsed HTTP response returned by HttpClient requests.
 *
 * The client stores the status code, reason phrase, raw response headers and
 * response body separately. The status line is normalized into status_code and
 * reason; headers do not include the trailing CRLF separator.
 */
struct HttpResponse {
    /**
     * @brief Numeric HTTP status code, for example 200 or 404.
     */
    int status_code = 0;

    /**
     * @brief HTTP reason phrase from the status line, for example "OK".
     */
    std::string reason;

    /**
     * @brief Raw HTTP response headers without the body.
     */
    std::string headers;

    /**
     * @brief Response body bytes represented as a string.
     */
    std::string body;
};

/**
 * @brief Small blocking HTTP/1.0 client used by LiDAR web API CLI commands.
 *
 * HttpClient intentionally implements only the subset needed by the driver
 * command-line interface: GET, POST with a JSON body, and streaming GET for
 * server-sent events or binary downloads. It does not handle authentication,
 * redirects, TLS, chunked transfer encoding or persistent connections.
 */
class HttpClient {
public:
    /**
     * @brief Create a client for a LiDAR HTTP endpoint.
     *
     * @param host LiDAR host name or IP address.
     * @param port TCP port of the LiDAR HTTP server.
     * @param timeout_ms Socket connect, send and receive timeout in
     * milliseconds.
     *
     * @throws std::invalid_argument if host is empty, port is outside the TCP
     * range or timeout_ms is not positive.
     */
    HttpClient(std::string host, int port = 80, int timeout_ms = 3000);

    /**
     * @brief Execute a blocking HTTP GET request.
     *
     * @param path Absolute request path beginning with '/', for example
     * "/config.json".
     * @return Parsed HTTP response.
     *
     * @throws std::invalid_argument if path is not absolute.
     * @throws std::runtime_error or std::system_error on transport or response
     * parsing errors.
     */
    HttpResponse get(const std::string& path) const;

    /**
     * @brief Execute a blocking HTTP POST request with a JSON body.
     *
     * @param path Absolute request path beginning with '/', for example
     * "/config/motor_speed".
     * @param body Raw request body. For configuration writes this must already
     * be a valid JSON token such as "10", "true" or "\"INFO\"".
     * @return Parsed HTTP response.
     *
     * @throws std::invalid_argument if path is not absolute.
     * @throws std::runtime_error or std::system_error on transport or response
     * parsing errors.
     */
    HttpResponse post(const std::string& path, const std::string& body) const;

    /**
     * @brief Execute a GET request and write the body as it is received.
     *
     * This method is used for endpoints where buffering the complete response is
     * undesirable, such as the Safety zones SSE stream and LUT binary download.
     *
     * @param path Absolute request path beginning with '/'.
     * @param body_stream Stream that receives response body bytes.
     * @param status_stream Optional stream for printing the HTTP status line.
     * @return Parsed HTTP status and headers. The body field is empty because
     * bytes are forwarded to body_stream.
     *
     * @throws std::invalid_argument if path is not absolute.
     * @throws std::runtime_error or std::system_error on transport or response
     * parsing errors.
     */
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
