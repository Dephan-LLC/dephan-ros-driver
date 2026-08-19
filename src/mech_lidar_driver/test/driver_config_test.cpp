#include "driver_config.hpp"

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <stdexcept>

namespace {
using dephan_ros::CaptureType;
using dephan_ros::DriverMode;
using nlohmann::json;

json valid_udp_config() {
    return {
        {"mode", "UDP"},
        {"capture_type", "FULL"},
        {"ip", "0.0.0.0"},
        {"port", 50007},
        {"topic", "scan"}
    };
}
} // namespace

TEST(DriverConfig, ParsesRequiredUdpFieldsAndDefaults) {
    const auto result = dephan_ros::parse_driver_config(valid_udp_config());

    EXPECT_EQ(result.mode, DriverMode::UDP);
    EXPECT_EQ(result.capture_type, CaptureType::FULL);
    EXPECT_EQ(result.ip, "0.0.0.0");
    EXPECT_EQ(result.port, 50007);
    EXPECT_EQ(result.topic, "scan");
    EXPECT_EQ(result.frame_id, "base_link");
    EXPECT_TRUE(result.diagnostics_enabled);
    EXPECT_EQ(result.diagnostics_topic, "diagnostics");
    EXPECT_EQ(result.udp_reconnect_initial_ms, 250);
    EXPECT_EQ(result.udp_reconnect_max_ms, 5000);
}

TEST(DriverConfig, ParsesPcapConfiguration) {
    const json value = {
        {"mode", "PCAP"},
        {"capture_type", "SINGLE"},
        {"pcap_path", "/tmp/lidar.pcap"},
        {"topic", "scan"}
    };

    const auto result = dephan_ros::parse_driver_config(value);

    EXPECT_EQ(result.mode, DriverMode::PCAP);
    EXPECT_EQ(result.capture_type, CaptureType::SINGLE);
    EXPECT_EQ(result.pcap_path, "/tmp/lidar.pcap");
}

TEST(DriverConfig, RejectsMissingModeSpecificField) {
    json value = valid_udp_config();
    value.erase("port");

    EXPECT_THROW(dephan_ros::parse_driver_config(value), std::invalid_argument);
}

TEST(DriverConfig, RejectsWrongTypesAndRanges) {
    json value = valid_udp_config();
    value["port"] = "50007";
    EXPECT_THROW(dephan_ros::parse_driver_config(value), std::invalid_argument);

    value = valid_udp_config();
    value["http_timeout_ms"] = 0;
    EXPECT_THROW(dephan_ros::parse_driver_config(value), std::invalid_argument);

    value = valid_udp_config();
    value["udp_reconnect_initial_ms"] = 1000;
    value["udp_reconnect_max_ms"] = 500;
    EXPECT_THROW(dephan_ros::parse_driver_config(value), std::invalid_argument);
}

TEST(DriverConfig, RejectsUnknownFields) {
    json value = valid_udp_config();
    value["udp_port_typo"] = 50007;

    EXPECT_THROW(dephan_ros::parse_driver_config(value), std::invalid_argument);
}

TEST(DriverConfig, RoundTripsValidatedConfiguration) {
    const auto parsed = dephan_ros::parse_driver_config(valid_udp_config());
    const auto serialized = dephan_ros::driver_config_to_json(parsed);
    const auto round_trip = dephan_ros::parse_driver_config(serialized);

    EXPECT_EQ(round_trip.mode, parsed.mode);
    EXPECT_EQ(round_trip.capture_type, parsed.capture_type);
    EXPECT_EQ(round_trip.ip, parsed.ip);
    EXPECT_EQ(round_trip.port, parsed.port);
    EXPECT_EQ(round_trip.diagnostics_period_s, 1.0);
    EXPECT_EQ(round_trip.no_data_timeout_s, 2.0);
    EXPECT_EQ(round_trip.udp_reconnect_initial_ms, 250);
    EXPECT_EQ(round_trip.udp_reconnect_max_ms, 5000);
}
