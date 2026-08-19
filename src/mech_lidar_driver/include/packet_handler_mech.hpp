/**
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov, Aleksandr Plukchi
 * All rights reserved.
 */

/**
 * @file packet_handler_mech.hpp
 * @brief Raw packets handler class for mechanical LiDar
 */

#ifndef PCK_HDL_MECH_HPP
#define PCK_HDL_MECH_HPP

#include "packet_raw.hpp"

#include <cstddef>
#include <cstdint>

namespace dephan_ros {
/**
 * Decodes one 936-byte M360 UDP packet according to the firmware contract.
 */
class pkt_hdl_Mech : public packet {

public:
    /**
     * Constructor for handled packet.
     */
    pkt_hdl_Mech(raw_packet_t pkt);

    /**
     * Disabled copy constructor.
     */
    pkt_hdl_Mech(const pkt_hdl_Mech&) = delete;

    /**
     * Disabled copy assign operator.
     */
    pkt_hdl_Mech& operator= (const pkt_hdl_Mech&) = delete;

    /**
     * Packet's magic byte.
     */
    static const uint8_t magic = 0x68;

    /**
     * Number of channels in one UDP packet.
     */
    static const int CHANELLS = 115;

    /**
     * Number of points in one full revolution.
     */
    static const int POINTS_PER_REV = 2300;

    /**
     * Version of the communications protocol.
     */
    static const uint8_t protocol_version = 0x00;

    /**
     * Angle resolution of the photodetection unit.
     */
    static constexpr float RAD_RESOLUTION =
        6.28318530717958647692f / POINTS_PER_REV;

    /** Revolution counter copied from the ROT packet field. */
    uint16_t rotation_counter() const;

    /** Target rotation frequency in hertz copied from the HZ packet field. */
    uint16_t rotation_frequency_hz() const;

    /**
     * First encoder point index contained in this packet.
     */
    uint16_t first_point_index() const;

    /**
     * Encoder point index for a packet channel.
     *
     * @throws std::out_of_range when the channel is outside this packet.
     */
    uint16_t point_index(size_t chnl) const;

    /** Whole seconds from the TS_SEC packet field. */
    uint32_t timestamp_seconds() const;

    /** Fractional seconds in 2^-32 units from the TS_FRAC packet field. */
    uint32_t timestamp_fraction() const;

    /** Packet timestamp represented as seconds with a fractional part. */
    double timestamp() const;

    /**
     * Ranges to the points within one scan packet.
     */
    float ranges[CHANELLS] = {0.0};

    /**
     * Intensities of the points within one scan packet.
     */
    float intensities[CHANELLS] = {0.0};

    /**
     * Angles of the points within one scan packet.
     */
    float angles[CHANELLS] = {0.0};

    /**
     * Raw packet that have to be handeled
     */
    raw_packet_t raw_pkt;

private:
    uint16_t rotation_counter_ = 0;
    uint16_t rotation_frequency_hz_ = 0;
    uint16_t enc_signal = 0;
    uint32_t timestamp_fraction_ = 0;
    uint32_t timestamp_seconds_ = 0;
};
} // namespace dephan_ros

#endif
