#include "driver_cli.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace {
using dephan_ros::HttpCommandType;

TEST(DriverCli, ParsesDriverConfigPath) {
    const auto options = dephan_ros::parse_cli(
        {"mech_driver", "--config", "/tmp/driver.json"}
    );

    EXPECT_TRUE(options.config_provided);
    EXPECT_EQ(options.config_path, "/tmp/driver.json");
    EXPECT_FALSE(dephan_ros::has_http_command(options));
}

TEST(DriverCli, ParsesHttpCommandAndConnectionOptions) {
    const auto options = dephan_ros::parse_cli({
        "mech_driver", "--lidar-ip", "192.168.0.42", "--http-port", "8080",
        "--get-contamination-status"
    });

    EXPECT_EQ(options.http_host, "192.168.0.42");
    EXPECT_EQ(options.http_port, 8080);
    EXPECT_EQ(options.http_command, HttpCommandType::Get);
    EXPECT_EQ(options.http_path, "/contamination/status.json");
}

TEST(DriverCli, StopsParsingAtRos2Arguments) {
    const auto options = dephan_ros::parse_cli({
        "mech_driver", "--config", "/tmp/driver.json", "--ros-args", "-r",
        "__node:=front_lidar", "--params-file", "/tmp/params.yaml"
    });

    EXPECT_EQ(options.config_path, "/tmp/driver.json");
}

TEST(DriverCli, BuildsConfigAndZoneRequestBodies) {
    const auto set_config = dephan_ros::parse_cli(
        {"mech_driver", "--set-config", "motor_speed", "10"}
    );
    EXPECT_EQ(set_config.http_command, HttpCommandType::SetConfig);
    EXPECT_EQ(set_config.config_name, "motor_speed");
    EXPECT_EQ(set_config.config_value, "10");

    const auto delete_zone = dephan_ros::parse_cli(
        {"mech_driver", "--delete-zone", "front"}
    );
    EXPECT_EQ(delete_zone.http_body, "{\"name\":\"front\"}");
}

TEST(DriverCli, RejectsConflictingCommandsAndInvalidPorts) {
    EXPECT_THROW(
        dephan_ros::parse_cli(
            {"mech_driver", "--get-config", "--get-status"}
        ),
        std::runtime_error
    );
    EXPECT_THROW(
        dephan_ros::parse_cli(
            {"mech_driver", "--http-port", "70000", "--get-config"}
        ),
        std::runtime_error
    );
}
} // namespace
