#ifndef PCK_HDL_MECH_HPP
#define PCK_HDL_MECH_HPP

#include "packet_raw.hpp"


namespace dephan {
    class pkt_hdl_Mech: public packet { 

    public:
        pkt_hdl_Mech(raw_packet_t pkt); 

        pkt_hdl_Mech(const pkt_hdl_Mech &) = delete; 

        pkt_hdl_Mech& operator=(const pkt_hdl_Mech &) = delete;

        static const uint8_t magic = 0x68;

        static const int CHANELLS = 125; 

        static const uint8_t protocol_version = 0x00; 

        static constexpr float RAD_RESOLUTION = 2 * 3.1415 / 2250;

        float ranges[CHANELLS] = {0.0};
        float angles[CHANELLS] = {0.0};

        raw_packet_t raw_pkt;

    private:
        uint16_t enc_signal = 0;

    };
}

#endif