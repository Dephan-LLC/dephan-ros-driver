/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file ros_driver.cpp
 * @brief ROS driver for DEPHAN LLC LiDars
 */

#include "ros_driver.hpp"
#include "http_client.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <limits>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/UInt8MultiArray.h>
#include <utility>
#include <vector>

namespace dephan_ros {
namespace {
using json = nlohmann::json;

const float PI = 3.14159265358979323846f;
const int POINTS_PER_REV = pkt_hdl_Mech::POINTS_PER_REV;
const int PACKETS_PER_REV = POINTS_PER_REV / pkt_hdl_Mech::CHANELLS;

std::string default_pointcloud_topic(const std::string& laserscan_topic) {
    std::string result = laserscan_topic;
    size_t pos = result.find("laserscan");
    if (pos != std::string::npos) {
        result.replace(pos, std::string("laserscan").size(), "pointcloud");
        return result;
    }
    return result + "_pointcloud";
}

float range_min_or_default(const std::vector<float>& ranges) {
    float result = std::numeric_limits<float>::infinity();
    for (float range : ranges) {
        if (std::isfinite(range) && range < result) {
            result = range;
        }
    }
    return std::isfinite(result) ? result : 0.0f;
}

float range_max_or_default(const std::vector<float>& ranges) {
    float result = 0.0f;
    for (float range : ranges) {
        if (std::isfinite(range) && range > result) {
            result = range;
        }
    }
    return result;
}

sensor_msgs::PointCloud2 make_pointcloud_msg(
    const sensor_msgs::LaserScan& scan
) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header = scan.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(scan.ranges.size());
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(scan.ranges.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < scan.ranges.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        float range = scan.ranges[i];
        if (!std::isfinite(range)) {
            *iter_x = std::numeric_limits<float>::quiet_NaN();
            *iter_y = std::numeric_limits<float>::quiet_NaN();
            *iter_z = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        float angle = scan.angle_min +
                      static_cast<float>(i) * scan.angle_increment;
        *iter_x = range * std::cos(angle);
        *iter_y = range * std::sin(angle);
        *iter_z = 0.0f;
    }

    return cloud;
}

geometry_msgs::Point make_point(double x, double y, double angle_offset_rad) {
    geometry_msgs::Point point;
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
    visualization_msgs::Marker& marker, bool enabled, bool status_known,
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
    visualization_msgs::Marker& marker, const json& segment,
    double angle_offset_rad
) {
    double start = segment.value("angle_start_deg", 0.0) * PI / 180.0;
    double end   = segment.value("angle_end_deg", 0.0) * PI / 180.0;
    double span  = end - start;
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
    visualization_msgs::Marker& marker, const json& segment,
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

visualization_msgs::MarkerArray build_safety_zone_markers(
    const json& zones, const json& status, const std::string& frame_id,
    double angle_offset_rad
) {
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
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

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "safety_zones";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.lifetime = ros::Duration(2.0);
            set_marker_color(marker, enabled, status_known, triggered);

            std::string type = segment.value("type", "");
            if (type == "sector") {
                append_sector_points(marker, segment, angle_offset_rad);
            }
            else if (type == "polygon") {
                append_polygon_points(marker, segment, angle_offset_rad);
            }

            if (marker.points.size() >= 2) {
                markers.markers.push_back(marker);
            }
        }
    }

    return markers;
}

void publish_scan_and_cloud(
    ros::Publisher& scan_publisher, ros::Publisher& cloud_publisher,
    const sensor_msgs::LaserScan::Ptr& scan
) {
    sensor_msgs::PointCloud2 cloud = make_pointcloud_msg(*scan);
    scan_publisher.publish(scan);
    cloud_publisher.publish(cloud);
}
} // namespace

Driver::Driver(
    ros::NodeHandle nh, std::string ip_addr, unsigned port,
    std::string cloud_topic, DriverRuntimeOptions options
) : ip_addr(ip_addr), port(port), runtime_options(std::move(options)) {

    // setup socket for receiving data
    socket.reset(new receiver_socket(ip_addr, port));

    // ROS publising routine
    laserscan_publisher = nh.advertise<sensor_msgs::LaserScan>(cloud_topic, 10);
    pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(
        runtime_options.pointcloud_topic.empty()
            ? default_pointcloud_topic(cloud_topic)
            : runtime_options.pointcloud_topic,
        10
    );
    if (runtime_options.safety_debug) {
        safety_zones_publisher =
            nh.advertise<visualization_msgs::MarkerArray>(
                runtime_options.safety_zones_topic, 1, true
            );
        safety_status_publisher = nh.advertise<std_msgs::String>(
            runtime_options.safety_status_topic, 1, true
        );
        safety_debug_running = true;
        safety_debug_thread = std::thread([this]() {
            while (ros::ok() && safety_debug_running.load()) {
                publish_safety_debug();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }
}

Driver::Driver(
    ros::NodeHandle nh, std::string pcap_path, std::string cloud_topic,
    DriverRuntimeOptions options
) : pcap_path(pcap_path), runtime_options(std::move(options)) {

    // setup libtins sniffer for reading data
    pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});

    // get the first packet's timestamp for time-correct packets reading
    Tins::Packet _pkt(pcap_sniffer->next_packet());
    _prev_pkt_tmstmp = _pkt.timestamp().microseconds();

    // ROS publising routine
    laserscan_publisher = nh.advertise<sensor_msgs::LaserScan>(cloud_topic, 10);
    pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(
        runtime_options.pointcloud_topic.empty()
            ? default_pointcloud_topic(cloud_topic)
            : runtime_options.pointcloud_topic,
        10
    );
    if (runtime_options.safety_debug) {
        safety_zones_publisher =
            nh.advertise<visualization_msgs::MarkerArray>(
                runtime_options.safety_zones_topic, 1, true
            );
        safety_status_publisher = nh.advertise<std_msgs::String>(
            runtime_options.safety_status_topic, 1, true
        );
        safety_debug_running = true;
        safety_debug_thread = std::thread([this]() {
            while (ros::ok() && safety_debug_running.load()) {
                publish_safety_debug();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }
}

Driver::~Driver() {
    safety_debug_running = false;
    if (safety_debug_thread.joinable()) {
        safety_debug_thread.join();
    }
}

void Driver::poll() {

    // pcap_sniffer provided?
    if (pcap_sniffer)
        _poll_pcap();

    // UDP mode otherwise
    else
        _poll_udp();
}

void Driver::poll_full() {

    // pcap_sniffer provided?
    if (pcap_sniffer)
        _poll_full_pcap();

    // UDP mode otherwise
    else
        _poll_full_udp();
}

void Driver::publish_safety_debug() {
    if (!runtime_options.safety_debug) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    double period_s = std::max(0.1, runtime_options.safety_debug_period_s);
    if (last_safety_debug_publish.time_since_epoch().count() != 0) {
        auto elapsed = std::chrono::duration<double>(
            now - last_safety_debug_publish
        );
        if (elapsed.count() < period_s) {
            return;
        }
    }
    last_safety_debug_publish = now;

    try {
        HttpClient client(
            runtime_options.http_host, runtime_options.http_port,
            runtime_options.http_timeout_ms
        );

        json status_json = json::object();
        HttpResponse status_response = client.get("/zones/status.json");
        if (status_response.status_code >= 200 &&
            status_response.status_code < 300) {
            std_msgs::String status_msg;
            status_msg.data = status_response.body;
            safety_status_publisher.publish(status_msg);
            if (!status_response.body.empty()) {
                status_json = json::parse(status_response.body);
            }
        }

        HttpResponse zones_response = client.get("/zones.json");
        if (zones_response.status_code >= 200 &&
            zones_response.status_code < 300 && !zones_response.body.empty()) {
            visualization_msgs::MarkerArray markers = build_safety_zone_markers(
                json::parse(zones_response.body), status_json,
                runtime_options.frame_id, runtime_options.angle_offset_rad
            );
            safety_zones_publisher.publish(markers);
        }
    }
    catch (const std::exception& ex) {
        ROS_WARN_STREAM_THROTTLE(
            5.0, "Safety debug polling failed: " << ex.what()
        );
    }
}

void Driver::_poll_full_udp() {

    // initialzie ros pointcloud v2 message
    sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan);

    msg->ranges.assign(POINTS_PER_REV, std::numeric_limits<float>::infinity());
    msg->intensities.assign(POINTS_PER_REV, 0.0f);

    // wait until one full revolution is received
    for (size_t i = 0; i < PACKETS_PER_REV; i++) {

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

        // wait until we are receive data
        while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)) {
            if (!ros::ok()) {
                return;
            }
        }

        // transform raw packet to handled packet
        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        // fill ros message by encoder point index
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            uint16_t point_idx = hdl_pkt.point_index(chnl);
            msg->ranges[point_idx] = hdl_pkt.ranges[chnl] / 1000;
            msg->intensities[point_idx] = hdl_pkt.intensities[chnl];
        }
    }
    // fill ros message by constant data
    msg->angle_min       = runtime_options.angle_offset_rad;
    msg->angle_max       = runtime_options.angle_offset_rad + 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = ros::Time::now();

    // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    publish_scan_and_cloud(laserscan_publisher, pointcloud_publisher, msg);
}

void Driver::_poll_full_pcap() {

    // initialzie ros pointcloud v2 message
    sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan);

    msg->ranges.assign(POINTS_PER_REV, std::numeric_limits<float>::infinity());
    msg->intensities.assign(POINTS_PER_REV, 0.0f);

    // wait until one full revolution is read from the target PCAP file
    for (size_t i = 0; i < PACKETS_PER_REV; i++) {

        // get the next packet from the target PCAP file
        Tins::Packet pkt(pcap_sniffer->next_packet());

        // is packet extracted with problems?
        if (!pkt) {
            ROS_INFO("Starting over...");
            pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
            Tins::Packet _pkt(pcap_sniffer->next_packet());
            _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
        }

        // normal operation otherwise
        else {

            // sleep for time-correct packets reading
            auto _cur_pkt_tmstmp = pkt.timestamp().microseconds();
            std::this_thread::sleep_for(
                std::chrono::microseconds(_cur_pkt_tmstmp - _prev_pkt_tmstmp)
            );
            _prev_pkt_tmstmp = _cur_pkt_tmstmp;

            // create raw-pdu collection
            std::vector<uint8_t> raw_pdu =
                pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();

            // initialize raw packet collection
            packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]);

            // fill raw_pkt with raw-pdu
            std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

            // transform raw packet to handled packet
            pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

            // fill ros message by encoder point index
            for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
                uint16_t point_idx = hdl_pkt.point_index(chnl);
                msg->ranges[point_idx] = hdl_pkt.ranges[chnl] / 1000;
                msg->intensities[point_idx] = hdl_pkt.intensities[chnl];
            }
        }
    }
    // fill ros message by constant data
    msg->angle_min       = runtime_options.angle_offset_rad;
    msg->angle_max       = runtime_options.angle_offset_rad + 2 * PI;
    msg->angle_increment = 2 * PI / POINTS_PER_REV;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    msg->range_min       = range_min_or_default(msg->ranges);
    msg->range_max       = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = ros::Time::now();

    // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    publish_scan_and_cloud(laserscan_publisher, pointcloud_publisher, msg);
}

std::pair<std::string, unsigned> Driver::get_network_params() {
    return {ip_addr, port};
}

void Driver::_poll_udp() {

    // initialize ros pointcloud v2 message
    sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan);

    // initialize raw packet collection
    packet::raw_packet_t raw_pkt(new uint8_t[packet::PKT_LEN]);

    // wait until we are receive data
    while (socket->get_packet(raw_pkt.get(), packet::PKT_LEN)) {
        if (!ros::ok()) {
            return;
        }
    }

    // transform raw packet to handled packet
    pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

    // fill ros message by data from the handled packet
    msg->angle_min       = hdl_pkt.angles[0] + runtime_options.angle_offset_rad;
    msg->angle_max       = hdl_pkt.angles[hdl_pkt.CHANELLS - 1] +
                           runtime_options.angle_offset_rad;
    msg->angle_increment = hdl_pkt.RAD_RESOLUTION;
    msg->scan_time       = 0.1;
    msg->time_increment  = msg->scan_time / POINTS_PER_REV;
    for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
        msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
        msg->intensities.push_back(hdl_pkt.intensities[chnl]);
    }
    msg->range_min = range_min_or_default(msg->ranges);
    msg->range_max = range_max_or_default(msg->ranges);

    // add timestamp to ros message
    msg->header.stamp = ros::Time::now();

    // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    publish_scan_and_cloud(laserscan_publisher, pointcloud_publisher, msg);
}

void Driver::_poll_pcap() {

    // initialzie ros pointcloud v2 message
    sensor_msgs::LaserScan::Ptr msg(new sensor_msgs::LaserScan);

    // get the next packet from the target PCAP file
    Tins::Packet pkt(pcap_sniffer->next_packet());

    // is packet extracted with problems?
    if (!pkt) {
        ROS_INFO("Starting over...");
        pcap_sniffer.reset(new Tins::FileSniffer{pcap_path});
        Tins::Packet _pkt(pcap_sniffer->next_packet());
        _prev_pkt_tmstmp = _pkt.timestamp().microseconds();
    }

    // normal operation otherwise
    else {

        // sleep for time-correct packets reading
        auto _cur_pkt_tmstmp = pkt.timestamp().microseconds();
        std::this_thread::sleep_for(
            std::chrono::microseconds(_cur_pkt_tmstmp - _prev_pkt_tmstmp)
        );
        _prev_pkt_tmstmp = _cur_pkt_tmstmp;

        // create raw-pdu collection
        std::vector<uint8_t> raw_pdu =
            pkt.pdu()->rfind_pdu<Tins::RawPDU>().payload();

        // initialize raw packet collection
        packet::raw_packet_t raw_pkt(new uint8_t[raw_pdu.size()]);

        // fill raw_pkt with raw-pdu
        std::copy(raw_pdu.begin(), raw_pdu.end(), raw_pkt.get());

        // transform raw packet to handled packet
        pkt_hdl_Mech hdl_pkt(std::move(raw_pkt));

        // fill ros message by data from the handled packet
        msg->angle_min = hdl_pkt.angles[0] + runtime_options.angle_offset_rad;
        msg->angle_max = hdl_pkt.angles[hdl_pkt.CHANELLS - 1] +
                         runtime_options.angle_offset_rad;
        msg->angle_increment = hdl_pkt.RAD_RESOLUTION;
        msg->scan_time       = 0.1;
        msg->time_increment  = msg->scan_time / POINTS_PER_REV;
        for (size_t chnl = 0; chnl < hdl_pkt.CHANELLS; ++chnl) {
            msg->ranges.push_back(hdl_pkt.ranges[chnl] / 1000);
            msg->intensities.push_back(hdl_pkt.intensities[chnl]);
        }
        msg->range_min = range_min_or_default(msg->ranges);
        msg->range_max = range_max_or_default(msg->ranges);
    }

    // add timestamp to ros message
    msg->header.stamp = ros::Time::now();

    // add frame id to ros message
    msg->header.frame_id = runtime_options.frame_id;

    // publish ros messages to topics
    publish_scan_and_cloud(laserscan_publisher, pointcloud_publisher, msg);
}
} // namespace dephan_ros
