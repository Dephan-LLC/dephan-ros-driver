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


namespace dephan_ros {
    /**
     * Class for handle raw packet derived from base packet class.
     */
    class pkt_hdl_Mech: public packet { 

    public:
        /**
         * Constructor for handled packet.
         */
        pkt_hdl_Mech(raw_packet_t pkt); 

        /**
         * Disabled copy constructor.
         */
        pkt_hdl_Mech(const pkt_hdl_Mech &) = delete; 

        /**
         * Disabled copy assign operator.
         */
        pkt_hdl_Mech& operator=(const pkt_hdl_Mech &) = delete;

        /**
         * Packet's magic byte.
         */
        static const uint8_t magic = 0x68;

        /**
         * Number of chanells of the photodetection unit. 
         */
        static const int CHANELLS = 125; 

        /**
         * Version of the communications protocol.
         */
        static const uint8_t protocol_version = 0x00; 

        /**
         * Angle resolution of the photodetection unit.
         */
        static constexpr float RAD_RESOLUTION = 2 * 3.1415 / 2250;

        /**
         * Ranges to the points within one scan packet.
         */
        float ranges[CHANELLS] = {0.0};

        /**
         * Angles of the points within one scan packet.
         */
        float angles[CHANELLS] = {0.0};

        /**
         * Raw packet that have to be handeled
         */
        raw_packet_t raw_pkt;

    private:
        // Signal from the LiDar's motor encoder.         
        uint16_t enc_signal = 0;
    };
}

#endif