/**
 * @file safety_zone_markers.hpp
 * @brief Conversion of LiDAR safety-zone JSON into RViz markers.
 */

#ifndef SAFETY_ZONE_MARKERS_HPP
#define SAFETY_ZONE_MARKERS_HPP

#include <nlohmann/json.hpp>
#include <ros/time.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace dephan_ros {
/**
 * Build an RViz marker array for all supported safety-zone segments.
 *
 * The returned array starts with a DELETEALL marker so removed zones disappear
 * from RViz. Sector, polygon, and two_points segments are rotated by the same
 * angle offset as LaserScan and PointCloud2 output.
 *
 * @param zones Response body from GET /zones.json.
 * @param status Response body from GET /zones/status.json.
 * @param frame_id TF frame assigned to generated markers.
 * @param angle_offset_rad Counter-clockwise coordinate rotation in radians.
 * @param stamp ROS timestamp assigned to generated markers.
 */
visualization_msgs::MarkerArray build_safety_zone_markers(
    const nlohmann::json& zones, const nlohmann::json& status,
    const std::string& frame_id, double angle_offset_rad,
    const ros::Time& stamp
);
} // namespace dephan_ros

#endif
