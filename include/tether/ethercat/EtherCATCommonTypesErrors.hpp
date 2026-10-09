/**
 * @file EtherCATCommonTypesErrors.hpp
 * @brief EtherCAT types: Common Types & Errors
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// Common Types & Errors
// ============================================================================

using EthHandle = void*;
using ErrorCode = int;
constexpr ErrorCode EC_SUCCESS = 0;
constexpr ErrorCode EC_FAILURE = -1;

} // namespace EtherCAT
