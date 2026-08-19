#include "safety_zone_markers.hpp"
#include <gtest/gtest.h>
#include <visualization_msgs/Marker.h>

namespace dephan_ros {
namespace {
using json = nlohmann::json;

constexpr double PI = 3.14159265358979323846;

TEST(SafetyZoneMarkersTest, BuildsRotatedTriggeredTwoPointMarker) {
    const json zones = {
        {"zones", {{{"name", "front_line"},
                    {"enabled", true},
                    {"segments", {{{"type", "two_points"},
                                    {"vertices", {{-0.5, -1.0}, {0.5, -1.0}}}}}}}}}
    };

    const auto markers = build_safety_zone_markers(
        zones, {{"front_line", true}}, "base_link", PI / 2.0,
        ros::Time(123, 456)
    );

    ASSERT_EQ(markers.markers.size(), 2u);
    EXPECT_EQ(markers.markers[0].action, visualization_msgs::Marker::DELETEALL);
    const auto& marker = markers.markers[1];
    EXPECT_EQ(marker.type, visualization_msgs::Marker::LINE_STRIP);
    EXPECT_EQ(marker.header.frame_id, "base_link");
    EXPECT_EQ(marker.header.stamp.sec, 123u);
    ASSERT_EQ(marker.points.size(), 2u);
    EXPECT_NEAR(marker.points[0].x, 1.0, 1e-9);
    EXPECT_NEAR(marker.points[0].y, -0.5, 1e-9);
    EXPECT_NEAR(marker.points[1].x, 1.0, 1e-9);
    EXPECT_NEAR(marker.points[1].y, 0.5, 1e-9);
    EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
    EXPECT_FLOAT_EQ(marker.color.g, 0.1f);
}

TEST(SafetyZoneMarkersTest, ClosesUntriggeredPolygon) {
    const json zones = {
        {"zones", {{{"name", "polygon"},
                    {"segments", {{{"type", "polygon"},
                                    {"vertices", {{0.0, 0.0}, {1.0, 0.0},
                                                  {0.0, 1.0}}}}}}}}}
    };

    const auto markers = build_safety_zone_markers(
        zones, {{"polygon", false}}, "base_link", 0.0, ros::Time(0)
    );

    ASSERT_EQ(markers.markers.size(), 2u);
    const auto& marker = markers.markers[1];
    ASSERT_EQ(marker.points.size(), 4u);
    EXPECT_DOUBLE_EQ(marker.points.front().x, marker.points.back().x);
    EXPECT_DOUBLE_EQ(marker.points.front().y, marker.points.back().y);
    EXPECT_FLOAT_EQ(marker.color.r, 0.1f);
    EXPECT_FLOAT_EQ(marker.color.g, 0.8f);
}

TEST(SafetyZoneMarkersTest, BuildsSectorWithUnknownStatusColor) {
    const json zones = {
        {"zones", {{{"name", "sector"},
                    {"segments", {{{"type", "sector"},
                                    {"angle_start_deg", -30.0},
                                    {"angle_end_deg", 30.0},
                                    {"dist_max_m", 2.5}}}}}}}
    };

    const auto markers = build_safety_zone_markers(
        zones, json::object(), "base_link", 0.0, ros::Time(0)
    );

    ASSERT_EQ(markers.markers.size(), 2u);
    const auto& marker = markers.markers[1];
    EXPECT_GE(marker.points.size(), 11u);
    EXPECT_DOUBLE_EQ(marker.points.front().x, 0.0);
    EXPECT_DOUBLE_EQ(marker.points.back().x, 0.0);
    EXPECT_FLOAT_EQ(marker.color.r, 1.0f);
    EXPECT_FLOAT_EQ(marker.color.g, 0.8f);
}

TEST(SafetyZoneMarkersTest, OmitsUnsupportedAndInvalidSegments) {
    const json zones = {
        {"zones", {{{"name", "invalid"},
                    {"segments", {{{"type", "unknown"}},
                                  {{"type", "two_points"},
                                   {"vertices", {{0.0, 0.0}}}}}}}}}
    };

    const auto markers = build_safety_zone_markers(
        zones, json::object(), "base_link", 0.0, ros::Time(0)
    );

    ASSERT_EQ(markers.markers.size(), 1u);
    EXPECT_EQ(markers.markers[0].action, visualization_msgs::Marker::DELETEALL);
}
} // namespace
} // namespace dephan_ros
