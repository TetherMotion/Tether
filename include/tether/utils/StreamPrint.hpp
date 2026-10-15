#pragma once

#include <format>
#include <iostream>
#include <string_view>

namespace Tether {
namespace Utils {

/**
 * @brief Emit one timestamped line — "t={t:7.3f}  {line}\n" + flush —
 *        for tools that stream periodic status to stdout.
 */
inline void printTimestamped(double t, std::string_view line) {
    std::cout << std::format("t={:7.3f}  {}\n", t, line);
    std::cout.flush();
}

} // namespace Utils
} // namespace Tether
