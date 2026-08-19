#include "packet_handler_mech.hpp"
#include "full_scan_assembler.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {
using dephan_ros::packet;
using dephan_ros::pkt_hdl_Mech;

void write_u16_le(uint8_t* data, uint16_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
}

void write_u32_le(uint8_t* data, uint32_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}

packet::raw_packet_t make_packet(uint16_t encoder_index = 115) {
    auto raw = std::make_unique<uint8_t[]>(packet::PKT_LEN);
    raw[0] = pkt_hdl_Mech::magic;
    raw[1] = pkt_hdl_Mech::protocol_version;
    write_u16_le(raw.get() + 2, 0x1234);
    write_u16_le(raw.get() + 4, 10);
    write_u16_le(raw.get() + 6, encoder_index);
    write_u32_le(raw.get() + 8, 0x80000000U);
    write_u32_le(raw.get() + 12, 42U);
    write_u32_le(raw.get() + 16, 1250U);
    write_u32_le(raw.get() + 20, 77U);
    return raw;
}

packet::raw_packet_t make_packet(
    uint16_t rotation, uint16_t encoder_index, uint32_t distance_mm
) {
    auto raw = make_packet(encoder_index);
    write_u16_le(raw.get() + 2, rotation);
    write_u32_le(raw.get() + 16, distance_mm);
    return raw;
}
} // namespace

TEST(PacketHandlerMech, DecodesContractHeaderAndPointData) {
    pkt_hdl_Mech packet(make_packet());

    EXPECT_EQ(packet.rotation_counter(), 0x1234);
    EXPECT_EQ(packet.rotation_frequency_hz(), 10);
    EXPECT_EQ(packet.first_point_index(), 115);
    EXPECT_EQ(packet.point_index(114), 229);
    EXPECT_EQ(packet.timestamp_seconds(), 42U);
    EXPECT_EQ(packet.timestamp_fraction(), 0x80000000U);
    EXPECT_DOUBLE_EQ(packet.timestamp(), 42.5);
    EXPECT_FLOAT_EQ(packet.ranges[0], 1250.0F);
    EXPECT_FLOAT_EQ(packet.intensities[0], 77.0F);
}

TEST(PacketHandlerMech, ValidatesPacketChannelIndex) {
    pkt_hdl_Mech packet(make_packet(2185));

    EXPECT_EQ(packet.point_index(114), 2299);
    EXPECT_THROW(packet.point_index(115), std::out_of_range);
}

TEST(PacketHandlerMech, MapsNoObjectDistanceToInfinity) {
    auto raw = make_packet();
    write_u32_le(raw.get() + 16, std::numeric_limits<uint32_t>::max());

    pkt_hdl_Mech packet(std::move(raw));

    EXPECT_TRUE(std::isinf(packet.ranges[0]));
}

TEST(PacketHandlerMech, RejectsUnsupportedVersion) {
    auto raw = make_packet();
    raw[0] = 0x67;

    EXPECT_THROW(pkt_hdl_Mech packet(std::move(raw)), std::runtime_error);
}

TEST(PacketHandlerMech, RejectsMisalignedEncoderIndex) {
    EXPECT_THROW(pkt_hdl_Mech packet(make_packet(116)), std::runtime_error);
}

TEST(FullScanAssembler, CompletesOnlyAfterAllUniquePacketPositions) {
    dephan_ros::FullScanAssembler assembler;

    for (size_t index = 0; index < assembler.PACKETS_PER_REV; ++index) {
        const uint16_t encoder = static_cast<uint16_t>(
            (assembler.PACKETS_PER_REV - index - 1) * pkt_hdl_Mech::CHANELLS
        );
        pkt_hdl_Mech packet(make_packet(7, encoder, encoder + 1000U));
        EXPECT_EQ(
            assembler.add_packet(packet),
            index + 1 == assembler.PACKETS_PER_REV
        );
    }

    ASSERT_TRUE(assembler.complete());
    EXPECT_EQ(assembler.packet_count(), assembler.PACKETS_PER_REV);
    EXPECT_EQ(assembler.rotation_counter(), 7);
    EXPECT_FLOAT_EQ(assembler.ranges_mm()[0], 1000.0F);
    EXPECT_FLOAT_EQ(assembler.ranges_mm()[2185], 3185.0F);
}

TEST(FullScanAssembler, IgnoresDuplicatesAndDropsIncompleteRevolution) {
    dephan_ros::FullScanAssembler assembler;
    pkt_hdl_Mech first(make_packet(10, 0, 1000));
    pkt_hdl_Mech duplicate(make_packet(10, 0, 2000));
    pkt_hdl_Mech next_rotation(make_packet(11, 115, 3000));
    pkt_hdl_Mech stale(make_packet(10, 230, 4000));

    EXPECT_FALSE(assembler.add_packet(first));
    EXPECT_FALSE(assembler.add_packet(duplicate));
    EXPECT_EQ(assembler.packet_count(), 1U);
    EXPECT_FALSE(assembler.add_packet(next_rotation));
    EXPECT_EQ(assembler.discarded_revolutions(), 1U);
    EXPECT_EQ(assembler.rotation_counter(), 11);
    EXPECT_FALSE(assembler.add_packet(stale));
    EXPECT_EQ(assembler.rotation_counter(), 11);
    EXPECT_EQ(assembler.packet_count(), 1U);
}
