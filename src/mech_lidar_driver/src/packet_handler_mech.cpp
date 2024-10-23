/** 
 * Copyright (c) 2024, DEPHAN LLC, Anton Ledrov, Aleksandr Plukchi
 * All rights reserved.
 */

/**
 * @file packet_handler_mech.cpp
 * @brief Raw packets handler class for mechanical LiDar
 */

#include "packet_handler_mech.hpp"
#include <iostream>


namespace dephan_ros {
    pkt_hdl_Mech::pkt_hdl_Mech(raw_packet_t pkt): 
        raw_pkt(std::move(pkt)) {

        if (!raw_pkt) 
            throw std::runtime_error("empty packet has been passed to ctor");
        
        enc_signal = ( (uint16_t)raw_pkt[7] << 8) + raw_pkt[6];

        for (size_t chnl = 0; chnl < CHANELLS; chnl++) {
            ranges[chnl] = ((uint32_t)raw_pkt[16 + chnl * 8 + 3] << 8 * 3) + 
                           ((uint32_t)raw_pkt[16 + chnl * 8 + 2] << 8 * 2) +
                           ((uint32_t)raw_pkt[16 + chnl * 8 + 1] << 8 * 1) + 
                           raw_pkt[16 + chnl * 8];

            angles[chnl] = (enc_signal + chnl) * RAD_RESOLUTION;
        }
    }
}

