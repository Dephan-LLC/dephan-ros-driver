#include "http_client.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
class OneShotHttpServer {
public:
    explicit OneShotHttpServer(std::string response) : response_(std::move(response)) {
        socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) {
            throw std::runtime_error("socket failed");
        }

        int reuse = 1;
        setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
            listen(socket_, 1) < 0) {
            close(socket_);
            throw std::runtime_error("bind/listen failed");
        }

        socklen_t length = sizeof(address);
        if (getsockname(
                socket_, reinterpret_cast<sockaddr*>(&address), &length
            ) < 0) {
            close(socket_);
            throw std::runtime_error("getsockname failed");
        }
        port_ = ntohs(address.sin_port);
        worker_ = std::thread([this]() { serve(); });
    }

    ~OneShotHttpServer() {
        if (worker_.joinable() && socket_ >= 0) {
            shutdown(socket_, SHUT_RDWR);
            close(socket_);
            socket_ = -1;
        }
        wait();
        if (socket_ >= 0) {
            close(socket_);
        }
    }

    int port() const {
        return port_;
    }

    void wait() {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    const std::string& request() const {
        return request_;
    }

private:
    void serve() {
        const int client = accept(socket_, nullptr, nullptr);
        if (client < 0) {
            return;
        }

        char buffer[1024];
        size_t expected_size = 0;
        while (true) {
            const ssize_t received = recv(client, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                break;
            }
            request_.append(buffer, static_cast<size_t>(received));

            const size_t header_end = request_.find("\r\n\r\n");
            if (header_end == std::string::npos) {
                continue;
            }
            if (expected_size == 0) {
                expected_size = header_end + 4;
                const std::string header = request_.substr(0, header_end);
                const std::string key = "Content-Length: ";
                const size_t content_length = header.find(key);
                if (content_length != std::string::npos) {
                    const size_t value_start = content_length + key.size();
                    expected_size += static_cast<size_t>(std::stoul(
                        header.substr(value_start)
                    ));
                }
            }
            if (request_.size() >= expected_size) {
                break;
            }
        }

        size_t sent = 0;
        while (sent < response_.size()) {
            const ssize_t count = send(
                client, response_.data() + sent, response_.size() - sent, 0
            );
            if (count <= 0) {
                break;
            }
            sent += static_cast<size_t>(count);
        }
        shutdown(client, SHUT_RDWR);
        close(client);
    }

    int socket_ = -1;
    int port_ = 0;
    std::string response_;
    std::string request_;
    std::thread worker_;
};

TEST(HttpClient, ValidatesEndpointAndPath) {
    EXPECT_THROW(dephan_ros::HttpClient("", 80, 100), std::invalid_argument);
    EXPECT_THROW(dephan_ros::HttpClient("127.0.0.1", 0, 100), std::invalid_argument);

    dephan_ros::HttpClient client("127.0.0.1", 80, 100);
    EXPECT_THROW(client.get("status.json"), std::invalid_argument);
}

TEST(HttpClient, ExecutesGetAndParsesResponse) {
    OneShotHttpServer server(
        "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n{\"ok\":true}"
    );
    dephan_ros::HttpClient client("127.0.0.1", server.port(), 1000);

    const auto response = client.get("/status.json");
    server.wait();

    EXPECT_EQ(response.status_code, 200);
    EXPECT_EQ(response.reason, "OK");
    EXPECT_EQ(response.body, "{\"ok\":true}");
    EXPECT_EQ(server.request().find("GET /status.json HTTP/1.0"), 0U);
}

TEST(HttpClient, SendsJsonPostBody) {
    OneShotHttpServer server("HTTP/1.0 200 OK\r\n\r\nOK");
    dephan_ros::HttpClient client("127.0.0.1", server.port(), 1000);

    const auto response = client.post("/config/motor_speed", "10");
    server.wait();

    EXPECT_EQ(response.body, "OK");
    EXPECT_NE(
        server.request().find("POST /config/motor_speed HTTP/1.0"),
        std::string::npos
    );
    EXPECT_NE(server.request().find("Content-Length: 2"), std::string::npos);
    EXPECT_NE(server.request().find("\r\n\r\n10"), std::string::npos);
}

TEST(HttpClient, StreamsResponseBodyAndStatus) {
    OneShotHttpServer server(
        "HTTP/1.0 200 OK\r\nContent-Type: text/event-stream\r\n\r\n"
        "event: status\ndata: {}\n\n"
    );
    dephan_ros::HttpClient client("127.0.0.1", server.port(), 1000);
    std::ostringstream body;
    std::ostringstream status;

    const auto response = client.stream_get("/events", body, &status);
    server.wait();

    EXPECT_EQ(response.status_code, 200);
    EXPECT_EQ(status.str(), "HTTP 200 OK\n");
    EXPECT_EQ(body.str(), "event: status\ndata: {}\n\n");
}
} // namespace
