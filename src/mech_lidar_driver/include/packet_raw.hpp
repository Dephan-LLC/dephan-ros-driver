#ifndef PACKET_RAW_HPP
#define PACKET_RAW_HPP

#include <memory>


namespace dephan {
    struct packet {
        public:
            typedef std::unique_ptr<uint8_t[]> raw_packet_t; 
    };
}


#endif