/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov, Aleksandr Plukchi
 * All rights reserved.
 */

/**
 * @file packet_handler_mech.cpp
 * @brief Raw packets handler class for mechanical LiDar
 */

#include "packet_handler_mech.hpp"

#include <limits>
#include <stdexcept>

namespace {
uint16_t read_u16_le(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32_le(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}
} // namespace

namespace dephan_ros {
pkt_hdl_Mech::pkt_hdl_Mech(raw_packet_t pkt) : raw_pkt(std::move(pkt)) {
    if (!raw_pkt) {
        throw std::runtime_error("empty packet has been passed to ctor");
    }

    if (raw_pkt[0] != magic || raw_pkt[1] != protocol_version) {
        throw std::runtime_error("unsupported M360 packet version");
    }

    rotation_counter_ = read_u16_le(raw_pkt.get() + 2);
    rotation_frequency_hz_ = read_u16_le(raw_pkt.get() + 4);
    enc_signal = read_u16_le(raw_pkt.get() + 6);
    timestamp_fraction_ = read_u32_le(raw_pkt.get() + 8);
    timestamp_seconds_ = read_u32_le(raw_pkt.get() + 12);

    if (enc_signal >= POINTS_PER_REV || enc_signal % CHANELLS != 0) {
        throw std::runtime_error("invalid M360 packet encoder index");
    }

    for (size_t chnl = 0; chnl < CHANELLS; chnl++) {
        const size_t point_offset = 16 + chnl * 8;
        const uint32_t distance_mm = read_u32_le(raw_pkt.get() + point_offset);
        ranges[chnl] = distance_mm == std::numeric_limits<uint32_t>::max()
                           ? std::numeric_limits<float>::infinity()
                           : static_cast<float>(distance_mm);
        intensities[chnl] =
            static_cast<float>(read_u32_le(raw_pkt.get() + point_offset + 4));

        angles[chnl] = (enc_signal + chnl) * RAD_RESOLUTION;
    }
}

uint16_t pkt_hdl_Mech::rotation_counter() const {
    return rotation_counter_;
}

uint16_t pkt_hdl_Mech::rotation_frequency_hz() const {
    return rotation_frequency_hz_;
}

uint16_t pkt_hdl_Mech::first_point_index() const {
    return enc_signal;
}

uint16_t pkt_hdl_Mech::point_index(size_t chnl) const {
    if (chnl >= CHANELLS) {
        throw std::out_of_range("M360 packet channel is out of range");
    }
    return static_cast<uint16_t>((enc_signal + chnl) % POINTS_PER_REV);
}

uint32_t pkt_hdl_Mech::timestamp_seconds() const {
    return timestamp_seconds_;
}

uint32_t pkt_hdl_Mech::timestamp_fraction() const {
    return timestamp_fraction_;
}

double pkt_hdl_Mech::timestamp() const {
    constexpr double fraction_scale = 4294967296.0;
    return static_cast<double>(timestamp_seconds_) +
           static_cast<double>(timestamp_fraction_) / fraction_scale;
}
} // namespace dephan_ros
