/**
 * @file full_scan_assembler.hpp
 * @brief Assembly of M360 packets into one complete revolution.
 */

#ifndef FULL_SCAN_ASSEMBLER_HPP
#define FULL_SCAN_ASSEMBLER_HPP

#include "packet_handler_mech.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace dephan_ros {
/**
 * Collects the 20 uniquely indexed packets that belong to one ROT value.
 *
 * A newer ROT value starts a new revolution and discards an incomplete one.
 * Duplicate and stale packets do not advance completion.
 */
class FullScanAssembler {
public:
    static constexpr size_t PACKETS_PER_REV =
        pkt_hdl_Mech::POINTS_PER_REV / pkt_hdl_Mech::CHANELLS;

    /** Add one packet and return true when a complete revolution is ready. */
    bool add_packet(const pkt_hdl_Mech& packet);

    /** Whether all packet positions for the active revolution were received. */
    bool complete() const;

    /** Number of unique packets currently stored. */
    size_t packet_count() const;

    /** Active firmware revolution counter. */
    uint16_t rotation_counter() const;

    /** Rotation frequency reported by the packet header. */
    uint16_t rotation_frequency_hz() const;

    /** Timestamp fields copied from packet ENC=0 when available. */
    uint32_t timestamp_seconds() const;
    uint32_t timestamp_fraction() const;

    /** Distance values in millimetres, indexed by encoder point. */
    const std::array<float, pkt_hdl_Mech::POINTS_PER_REV>& ranges_mm() const;

    /** Intensity values indexed by encoder point. */
    const std::array<float, pkt_hdl_Mech::POINTS_PER_REV>& intensities() const;

    /** Number of incomplete revolutions replaced by a newer ROT value. */
    size_t discarded_revolutions() const;

private:
    void begin_rotation(const pkt_hdl_Mech& packet);

    bool has_rotation_ = false;
    bool complete_ = false;
    uint16_t rotation_counter_ = 0;
    uint16_t rotation_frequency_hz_ = 0;
    uint32_t timestamp_seconds_ = 0;
    uint32_t timestamp_fraction_ = 0;
    size_t packet_count_ = 0;
    size_t discarded_revolutions_ = 0;
    std::array<bool, PACKETS_PER_REV> received_{};
    std::array<float, pkt_hdl_Mech::POINTS_PER_REV> ranges_mm_{};
    std::array<float, pkt_hdl_Mech::POINTS_PER_REV> intensities_{};
};
} // namespace dephan_ros

#endif
