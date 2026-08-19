/**
 * @file driver_cli.hpp
 * @brief Command-line interface for driver and LiDAR HTTP operations.
 */

#ifndef DRIVER_CLI_HPP
#define DRIVER_CLI_HPP

#include <nlohmann/json.hpp>

#include <iosfwd>
#include <string>
#include <vector>

namespace dephan_ros {
/** HTTP operation selected on the command line. */
enum class HttpCommandType {
    None,
    Get,
    Post,
    SetConfig,
    Download,
    Stream,
};

/** Parsed command-line options, independent from ROS initialization. */
struct CliOptions {
    std::string config_path;
    bool config_provided = false;
    bool help = false;

    std::string http_host = "192.168.0.120";
    int http_port = 80;
    int http_timeout_ms = 3000;

    HttpCommandType http_command = HttpCommandType::None;
    std::string http_path;
    std::string http_body;
    std::string output_path;
    std::string config_name;
    std::string config_value;
};

/** Parse driver arguments while ignoring ROS remapping arguments. */
CliOptions parse_cli(const std::vector<std::string>& arguments);

/** Return true when a one-shot LiDAR HTTP command was selected. */
bool has_http_command(const CliOptions& options);

/** Print driver and HTTP command help. */
void print_cli_help(std::ostream& output, const std::string& ros_command);

/** Load the configured JSON file, or the legacy default when omitted. */
nlohmann::json load_driver_configuration(
    const CliOptions& options, std::ostream& output
);

/** Execute one selected HTTP operation and return a process exit code. */
int run_http_command(const CliOptions& options, std::ostream& output);
} // namespace dephan_ros

#endif
