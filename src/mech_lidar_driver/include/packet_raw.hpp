#ifndef PACKET_RAW_HPP
#define PACKET_RAW_HPP

#include <memory>


namespace dephan_ros {
    struct packet {
        public:
            typedef std::unique_ptr<uint8_t[]> raw_packet_t; 

            static const unsigned PKT_LEN = 1016; 
    };
}


#endif