#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace MockSDO {
    void setString(uint16_t index, const std::string& value);
    void clear();
}
