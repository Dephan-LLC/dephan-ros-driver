#include "driver_diagnostics.hpp"

#include <gtest/gtest.h>

TEST(DriverDiagnostics, UsesConcreteUdpSourceAddress) {
    EXPECT_EQ(
        dephan_ros::diagnostic_hardware_id(
            "192.168.0.120", "", "192.168.0.121"
        ),
        "192.168.0.120"
    );
}

TEST(DriverDiagnostics, UsesHttpHostForWildcardUdpSource) {
    EXPECT_EQ(
        dephan_ros::diagnostic_hardware_id(
            "0.0.0.0", "", "192.168.0.121"
        ),
        "192.168.0.121"
    );
    EXPECT_EQ(
        dephan_ros::diagnostic_hardware_id("", "", "lidar.local"),
        "lidar.local"
    );
}

TEST(DriverDiagnostics, PcapPathTakesPrecedence) {
    EXPECT_EQ(
        dephan_ros::diagnostic_hardware_id(
            "0.0.0.0", "/data/scan.pcap", "192.168.0.121"
        ),
        "/data/scan.pcap"
    );
}
