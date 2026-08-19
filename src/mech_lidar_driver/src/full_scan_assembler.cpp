/**
 * @file full_scan_assembler.cpp
 * @brief Assembly of M360 packets into one complete revolution.
 */

#include "full_scan_assembler.hpp"

#include <limits>

namespace dephan_ros {
void FullScanAssembler::begin_rotation(const pkt_hdl_Mech& packet) {
    if (has_rotation_ && !complete_ && packet_count_ > 0) {
        ++discarded_revolutions_;
    }

    has_rotation_ = true;
    complete_ = false;
    rotation_counter_ = packet.rotation_counter();
    rotation_frequency_hz_ = packet.rotation_frequency_hz();
    timestamp_seconds_ = packet.timestamp_seconds();
    timestamp_fraction_ = packet.timestamp_fraction();
    packet_count_ = 0;
    received_.fill(false);
    ranges_mm_.fill(std::numeric_limits<float>::infinity());
    intensities_.fill(0.0F);
}

bool FullScanAssembler::add_packet(const pkt_hdl_Mech& packet) {
    if (!has_rotation_) {
        begin_rotation(packet);
    }
    else if (packet.rotation_counter() != rotation_counter_) {
        const int16_t rotation_delta = static_cast<int16_t>(
            packet.rotation_counter() - rotation_counter_
        );
        if (rotation_delta < 0) {
            return complete_;
        }
        begin_rotation(packet);
    }

    const size_t packet_index =
        packet.first_point_index() / pkt_hdl_Mech::CHANELLS;
    if (received_[packet_index]) {
        return complete_;
    }

    if (packet.first_point_index() == 0) {
        rotation_frequency_hz_ = packet.rotation_frequency_hz();
        timestamp_seconds_ = packet.timestamp_seconds();
        timestamp_fraction_ = packet.timestamp_fraction();
    }

    for (size_t channel = 0; channel < pkt_hdl_Mech::CHANELLS; ++channel) {
        const size_t point_index = packet.point_index(channel);
        ranges_mm_[point_index] = packet.ranges[channel];
        intensities_[point_index] = packet.intensities[channel];
    }

    received_[packet_index] = true;
    ++packet_count_;
    complete_ = packet_count_ == PACKETS_PER_REV;
    return complete_;
}

bool FullScanAssembler::complete() const {
    return complete_;
}

size_t FullScanAssembler::packet_count() const {
    return packet_count_;
}

uint16_t FullScanAssembler::rotation_counter() const {
    return rotation_counter_;
}

uint16_t FullScanAssembler::rotation_frequency_hz() const {
    return rotation_frequency_hz_;
}

uint32_t FullScanAssembler::timestamp_seconds() const {
    return timestamp_seconds_;
}

uint32_t FullScanAssembler::timestamp_fraction() const {
    return timestamp_fraction_;
}

const std::array<float, pkt_hdl_Mech::POINTS_PER_REV>&
FullScanAssembler::ranges_mm() const {
    return ranges_mm_;
}

const std::array<float, pkt_hdl_Mech::POINTS_PER_REV>&
FullScanAssembler::intensities() const {
    return intensities_;
}

size_t FullScanAssembler::discarded_revolutions() const {
    return discarded_revolutions_;
}
} // namespace dephan_ros
