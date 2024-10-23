/** 
 * Copyright (c) 2024, DEPHAN LLC, Aleksandr Plukchi, Anton Ledrov
 * All rights reserved.
 */

/**
 * @file packet_handler_mech.hpp
 * @brief Raw packet from the LiDar
 */

#ifndef PACKET_RAW_HPP
#define PACKET_RAW_HPP

#include <memory>


namespace dephan_ros {
    /**
     * Base Packet class
     */
    struct packet {
        public:
            typedef std::unique_ptr<uint8_t[]> raw_packet_t; 

            static const unsigned PKT_LEN = 1016; 
    };
}


#endif