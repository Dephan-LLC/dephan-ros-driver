#ifndef COLORED_COUT_H
#define COLORED_COUT_H

#include <string>
#include <iostream>

namespace utils {

enum class Color { black = 30, red, green, yellow, blue, magenta, cyan, white };

enum class Style { standard = 0, bold = 1, underline = 4 };

std::string make_colored(const std::string& input, Color color, Style style);
std::string make_colored(const std::string& input, Color color);
} // namespace utils

#endif