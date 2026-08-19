/**
 * @file safety_zone_markers.cpp
 * @brief Conversion of LiDAR safety-zone JSON into RViz markers.
 */

#include "safety_zone_markers.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/duration.hpp>

namespace dephan_ros {
namespace {
using json = nlohmann::json;

constexpr double PI = 3.14159265358979323846;

geometry_msgs::msg::Point make_point(
    double x, double y, double angle_offset_rad
) {
    geometry_msgs::msg::Point point;
    point.x = x * std::cos(angle_offset_rad) - y * std::sin(angle_offset_rad);
    point.y = x * std::sin(angle_offset_rad) + y * std::cos(angle_offset_rad);
    point.z = 0.0;
    return point;
}

bool zone_is_enabled(const json& zone) {
    return !zone.contains("enabled") || zone.value("enabled", true);
}

bool zone_status_known(const json& status, const std::string& name) {
    return status.is_object() && status.contains(name) && status[name].is_boolean();
}

bool zone_is_triggered(const json& status, const std::string& name) {
    return zone_status_known(status, name) && status[name].get<bool>();
}

void set_marker_color(
    visualization_msgs::msg::Marker& marker, bool enabled, bool status_known,
    bool triggered
) {
    marker.color.a = enabled ? 0.95f : 0.35f;
    if (!enabled) {
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
    }
    else if (!status_known) {
        marker.color.r = 1.0f;
        marker.color.g = 0.8f;
        marker.color.b = 0.1f;
    }
    else if (triggered) {
        marker.color.r = 1.0f;
        marker.color.g = 0.1f;
        marker.color.b = 0.1f;
    }
    else {
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.2f;
    }
}

void append_sector_points(
    visualization_msgs::msg::Marker& marker, const json& segment,
    double angle_offset_rad
) {
    double start = segment.value("angle_start_deg", 0.0) * PI / 180.0;
    double end = segment.value("angle_end_deg", 0.0) * PI / 180.0;
    double span = end - start;
    while (span < 0.0) {
        span += 2.0 * PI;
    }

    double radius = segment.value("dist_max_m", 0.0);
    int steps = std::max(8, static_cast<int>(std::ceil(span / (PI / 36.0))));
    marker.points.push_back(make_point(0.0, 0.0, angle_offset_rad));
    for (int i = 0; i <= steps; ++i) {
        double angle = start + span * static_cast<double>(i) /
                                   static_cast<double>(steps);
        marker.points.push_back(make_point(
            radius * std::cos(angle), radius * std::sin(angle),
            angle_offset_rad
        ));
    }
    marker.points.push_back(make_point(0.0, 0.0, angle_offset_rad));
}

void append_polygon_points(
    visualization_msgs::msg::Marker& marker, const json& segment,
    double angle_offset_rad
) {
    if (!segment.contains("vertices") || !segment["vertices"].is_array()) {
        return;
    }

    for (const auto& vertex : segment["vertices"]) {
        if (vertex.is_array() && vertex.size() >= 2) {
            marker.points.push_back(make_point(
                vertex[0].get<double>(), vertex[1].get<double>(),
                angle_offset_rad
            ));
        }
    }
    if (!marker.points.empty()) {
        marker.points.push_back(marker.points.front());
    }
}

void append_two_point_segment(
    visualization_msgs::msg::Marker& marker, const json& segment,
    double angle_offset_rad
) {
    if (!segment.contains("vertices") || !segment["vertices"].is_array() ||
        segment["vertices"].size() != 2) {
        return;
    }
    for (const auto& vertex : segment["vertices"]) {
        if (!vertex.is_array() || vertex.size() < 2) {
            marker.points.clear();
            return;
        }
        marker.points.push_back(make_point(
            vertex[0].get<double>(), vertex[1].get<double>(), angle_offset_rad
        ));
    }
}
} // namespace

visualization_msgs::msg::MarkerArray build_safety_zone_markers(
    const json& zones, const json& status, const std::string& frame_id,
    double angle_offset_rad, const rclcpp::Time& stamp
) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    if (!zones.is_object() || !zones.contains("zones") ||
        !zones["zones"].is_array()) {
        return markers;
    }

    int marker_id = 1;
    for (const auto& zone : zones["zones"]) {
        if (!zone.is_object() || !zone.contains("segments") ||
            !zone["segments"].is_array()) {
            continue;
        }

        std::string name = zone.value("name", "zone");
        bool enabled = zone_is_enabled(zone);
        bool status_known = zone_status_known(status, name);
        bool triggered = zone_is_triggered(status, name);

        for (const auto& segment : zone["segments"]) {
            if (!segment.is_object()) {
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = stamp;
            marker.ns = "safety_zones";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            set_marker_color(marker, enabled, status_known, triggered);

            std::string type = segment.value("type", "");
            if (type == "sector") {
                append_sector_points(marker, segment, angle_offset_rad);
            }
            else if (type == "polygon") {
                append_polygon_points(marker, segment, angle_offset_rad);
            }
            else if (type == "two_points") {
                append_two_point_segment(marker, segment, angle_offset_rad);
            }

            if (marker.points.size() >= 2) {
                markers.markers.push_back(marker);
            }
        }
    }

    return markers;
}
} // namespace dephan_ros
