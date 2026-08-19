#include "driver_diagnostics.hpp"

namespace dephan_ros {
std::string diagnostic_hardware_id(
    const std::string& source_ip, const std::string& pcap_path,
    const std::string& http_host
) {
    if (!pcap_path.empty()) {
        return pcap_path;
    }
    if (source_ip.empty() || source_ip == "0.0.0.0") {
        return http_host;
    }
    return source_ip;
}
} // namespace dephan_ros
