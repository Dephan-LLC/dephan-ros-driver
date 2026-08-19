/**
 * @file driver_cli.cpp
 * @brief Command-line interface for driver and LiDAR HTTP operations.
 */

#include "driver_cli.hpp"

#include "http_client.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dephan_ros {
namespace {
constexpr const char* DEFAULT_DRIVER_CONFIG =
    "./src/mech_lidar_driver/configs/default_udp_config.json";

using SimpleCommand = std::pair<HttpCommandType, const char*>;

const std::map<std::string, SimpleCommand> SIMPLE_COMMANDS = {
    {"--get-config", {HttpCommandType::Get, "/config.json"}},
    {"--get-status", {HttpCommandType::Get, "/status.json"}},
    {"--get-features", {HttpCommandType::Get, "/features.json"}},
    {"--get-version", {HttpCommandType::Get, "/version.txt"}},
    {"--get-version-string", {HttpCommandType::Get, "/version_string.txt"}},
    {"--get-log", {HttpCommandType::Get, "/log.txt"}},
    {"--get-timestamp", {HttpCommandType::Get, "/timestamp.txt"}},
    {"--log-events", {HttpCommandType::Stream, "/log_events"}},
    {"--get-contamination-status",
     {HttpCommandType::Get, "/contamination/status.json"}},
    {"--contamination-events",
     {HttpCommandType::Stream, "/contamination_events"}},
    {"--get-zones", {HttpCommandType::Get, "/zones.json"}},
    {"--get-zone-settings", {HttpCommandType::Get, "/zones/settings.json"}},
    {"--get-zone-status", {HttpCommandType::Get, "/zones/status.json"}},
    {"--save-zones", {HttpCommandType::Post, "/zones/save"}},
    {"--safety-events", {HttpCommandType::Stream, "/safety_events"}},
    {"--save-preset", {HttpCommandType::Post, "/save_preset.cgi"}},
    {"--jump-to-bootloader",
     {HttpCommandType::Post, "/jump_to_bootloader.cgi"}},
};

bool is_ros_argument(const std::string& value) {
    return value.find(":=") != std::string::npos || value.rfind("__", 0) == 0;
}

int parse_integer(const std::string& option, const std::string& value) {
    size_t parsed = 0;
    int result = std::stoi(value, &parsed);
    if (parsed != value.size()) {
        throw std::runtime_error(
            "Invalid integer for " + option + ": " + value
        );
    }
    return result;
}

void require_value(
    const std::vector<std::string>& arguments, size_t index,
    const std::string& option, size_t count = 1
) {
    if (index + count >= arguments.size()) {
        throw std::runtime_error("Missing value for " + option);
    }
}

void ensure_no_http_command(const CliOptions& options) {
    if (has_http_command(options)) {
        throw std::runtime_error("Only one HTTP command can be used at a time");
    }
}

std::string read_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

std::string read_body_argument(const std::string& value) {
    if (!value.empty() && value.front() == '@') {
        if (value.size() == 1) {
            throw std::runtime_error("Missing file path after @");
        }
        return read_file(value.substr(1));
    }
    return value;
}

std::string zone_name_body(const char* key, const std::string& name) {
    return nlohmann::json{{key, name}}.dump();
}

void select_http_command(
    CliOptions& options, HttpCommandType type, const std::string& path
) {
    ensure_no_http_command(options);
    options.http_command = type;
    options.http_path = path;
}

void write_binary_file(const std::string& path, const std::string& data) {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Failed to open output file: " + path);
    }
    output.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!output) {
        throw std::runtime_error("Failed to write output file: " + path);
    }
}

bool response_ok(const HttpResponse& response) {
    return response.status_code >= 200 && response.status_code < 300;
}

void print_response_header(std::ostream& output, const HttpResponse& response) {
    output << "HTTP " << response.status_code;
    if (!response.reason.empty()) {
        output << " " << response.reason;
    }
    output << '\n';
}

void print_response_body(std::ostream& output, const std::string& body) {
    if (body.empty()) {
        return;
    }
    output << body;
    if (body.back() != '\n') {
        output << '\n';
    }
}
} // namespace

CliOptions parse_cli(const std::vector<std::string>& arguments) {
    CliOptions options;

    for (size_t i = 1; i < arguments.size(); ++i) {
        const std::string& argument = arguments[i];
        if (argument == "--ros-args") {
            break;
        }
        if (is_ros_argument(argument)) {
            continue;
        }

        if (argument == "-h" || argument == "--help") {
            options.help = true;
        }
        else if (argument == "-c" || argument == "--config") {
            require_value(arguments, i, argument);
            options.config_path = arguments[++i];
            options.config_provided = true;
        }
        else if (argument == "--http-host" || argument == "--lidar-ip") {
            require_value(arguments, i, argument);
            options.http_host = arguments[++i];
        }
        else if (argument == "--http-port") {
            require_value(arguments, i, argument);
            options.http_port = parse_integer(argument, arguments[++i]);
            if (options.http_port < 1 || options.http_port > 65535) {
                throw std::runtime_error("--http-port must be in range 1..65535");
            }
        }
        else if (argument == "--http-timeout-ms") {
            require_value(arguments, i, argument);
            options.http_timeout_ms = parse_integer(argument, arguments[++i]);
            if (options.http_timeout_ms <= 0) {
                throw std::runtime_error("--http-timeout-ms must be positive");
            }
        }
        else if (const auto command = SIMPLE_COMMANDS.find(argument);
                 command != SIMPLE_COMMANDS.end()) {
            select_http_command(
                options, command->second.first, command->second.second
            );
        }
        else if (argument == "--set-zones" || argument == "--add-zone" ||
                 argument == "--set-zone-settings" ||
                 argument == "--set-config-json") {
            require_value(arguments, i, argument);
            const std::map<std::string, std::string> paths = {
                {"--set-zones", "/zones.json"},
                {"--add-zone", "/zones/add.json"},
                {"--set-zone-settings", "/zones/settings.json"},
                {"--set-config-json", "/config.json"},
            };
            select_http_command(options, HttpCommandType::Post, paths.at(argument));
            options.http_body = read_body_argument(arguments[++i]);
        }
        else if (argument == "--delete-zone") {
            require_value(arguments, i, argument);
            select_http_command(
                options, HttpCommandType::Post, "/zones/delete.json"
            );
            options.http_body = zone_name_body("name", arguments[++i]);
        }
        else if (argument == "--reset-zones") {
            select_http_command(options, HttpCommandType::Post, "/zones/reset");
            if (i + 1 < arguments.size() &&
                arguments[i + 1].rfind("-", 0) != 0) {
                options.http_body =
                    zone_name_body("zone_name", arguments[++i]);
            }
        }
        else if (argument == "--get-zones-lut") {
            require_value(arguments, i, argument);
            select_http_command(
                options, HttpCommandType::Download, "/zones/lut.bin"
            );
            options.output_path = arguments[++i];
        }
        else if (argument == "--set-config") {
            require_value(arguments, i, argument, 2);
            select_http_command(options, HttpCommandType::SetConfig, "");
            options.config_name = arguments[++i];
            options.config_value = arguments[++i];
            if (options.config_name.empty() ||
                options.config_name.find('/') != std::string::npos) {
                throw std::runtime_error("Invalid config parameter name");
            }
        }
        else {
            throw std::runtime_error("Bad command line flag: " + argument);
        }
    }

    return options;
}

bool has_http_command(const CliOptions& options) {
    return options.http_command != HttpCommandType::None;
}

void print_cli_help(std::ostream& output, const std::string& ros_command) {
    output
        << "\nDriver mode:\n"
        << "  -c, --config <path>          Path to UDP/PCAP JSON config\n\n"
        << "HTTP options:\n"
        << "  --http-host <ip-or-host>     Lidar HTTP host, default 192.168.0.120\n"
        << "  --lidar-ip <ip>              Alias for --http-host\n"
        << "  --http-port <port>           Lidar HTTP port, default 80\n"
        << "  --http-timeout-ms <ms>       HTTP timeout, default 3000\n\n"
        << "HTTP read commands:\n"
        << "  --get-config                 GET /config.json\n"
        << "  --get-status                 GET /status.json\n"
        << "  --get-features               GET /features.json\n"
        << "  --get-version                GET /version.txt\n"
        << "  --get-version-string         GET /version_string.txt\n"
        << "  --get-log                    GET /log.txt\n"
        << "  --get-timestamp              GET /timestamp.txt\n"
        << "  --log-events                 GET /log_events\n\n"
        << "Contamination commands:\n"
        << "  --get-contamination-status  GET /contamination/status.json\n"
        << "  --contamination-events      GET /contamination_events\n\n"
        << "Safety zones commands:\n"
        << "  --get-zones                  GET /zones.json\n"
        << "  --set-zones <json|@file>     POST /zones.json\n"
        << "  --add-zone <json|@file>      POST /zones/add.json\n"
        << "  --delete-zone <name>         POST /zones/delete.json\n"
        << "  --get-zone-settings          GET /zones/settings.json\n"
        << "  --set-zone-settings <json|@file>  POST /zones/settings.json\n"
        << "  --get-zone-status            GET /zones/status.json\n"
        << "  --reset-zones [name]         POST /zones/reset\n"
        << "  --save-zones                 POST /zones/save\n"
        << "  --get-zones-lut <path>       GET /zones/lut.bin\n"
        << "  --safety-events              GET /safety_events\n\n"
        << "HTTP write commands:\n"
        << "  --set-config <name> <json-token>  POST /config/{name}\n"
        << "  --set-config-json <json|@file>    POST /config.json\n"
        << "  --save-preset                POST /save_preset.cgi\n"
        << "  --jump-to-bootloader         POST /jump_to_bootloader.cgi\n\n"
        << "Examples:\n"
        << "  " << ros_command << " --get-config\n"
        << "  " << ros_command
        << " --lidar-ip 192.168.0.120 --set-config motor_speed 10\n"
        << "  " << ros_command << " --set-config preemptive_conns true\n"
        << "  " << ros_command
        << " --set-config-json '{\"motor_speed\":10}'\n"
        << "  " << ros_command << " --add-zone @zone.json\n"
        << "  " << ros_command << " --get-zones-lut /tmp/zones_lut.bin\n\n";
}

nlohmann::json load_driver_configuration(
    const CliOptions& options, std::ostream& output
) {
    std::string path = options.config_path;
    if (!options.config_provided) {
        output << "Config was not provided; using default config\n";
        path = DEFAULT_DRIVER_CONFIG;
    }

    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open config file: " + path);
    }
    if (input.peek() == std::ifstream::traits_type::eof()) {
        throw std::runtime_error("Config file is empty: " + path);
    }
    return nlohmann::json::parse(input);
}

int run_http_command(const CliOptions& options, std::ostream& output) {
    if (!has_http_command(options)) {
        throw std::runtime_error("No HTTP command selected");
    }

    HttpClient client(
        options.http_host, options.http_port, options.http_timeout_ms
    );
    HttpResponse response;

    switch (options.http_command) {
    case HttpCommandType::Get:
        response = client.get(options.http_path);
        break;
    case HttpCommandType::Post:
        response = client.post(options.http_path, options.http_body);
        break;
    case HttpCommandType::SetConfig:
        response = client.post(
            "/config/" + options.config_name, options.config_value
        );
        break;
    case HttpCommandType::Download:
        response = client.get(options.http_path);
        break;
    case HttpCommandType::Stream:
        response = client.stream_get(options.http_path, output, &output);
        return response_ok(response) ? 0 : 1;
    case HttpCommandType::None:
        throw std::runtime_error("No HTTP command selected");
    }

    print_response_header(output, response);
    if (options.http_command == HttpCommandType::Download && response_ok(response)) {
        write_binary_file(options.output_path, response.body);
        output << "Saved " << response.body.size() << " bytes to "
               << options.output_path << '\n';
    }
    else {
        print_response_body(output, response.body);
    }
    return response_ok(response) ? 0 : 1;
}
} // namespace dephan_ros
