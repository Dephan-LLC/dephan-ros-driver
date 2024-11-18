#include "colored_cout.h"

namespace utils {

std::string make_colored(const std::string& input, Color color, Style style) {
    std::string _str_color = std::to_string((int) color);
    std::string _str_style = std::to_string((int) style);

    return "\033[" + _str_style + ";" + _str_color + "m" + input + "\033[0m";
}

std::string make_colored(const std::string& input, Color color) {
    std::string _str_color = std::to_string((int) color);

    return "\033[;" + _str_color + "m" + input + "\033[0m";
}

} // namespace utils