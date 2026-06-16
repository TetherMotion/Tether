#pragma once

/**
 * @file SlaveIdentity.hpp
 * @brief Expected slave identity for verification against SII EEPROM
 */

#include <cstdint>
#include <optional>

namespace EtherCAT {
namespace Identity {

struct SlaveIdentity {
    std::optional<uint32_t> vendor_id;        ///< Vendor ID (ETG assigned)
    std::optional<uint32_t> product_code;     ///< Vendor-specific product code
    std::optional<uint32_t> revision_number;  ///< Revision (major.minor in high.low words)
    std::optional<uint32_t> serial_number;    ///< Device serial number
};

} // namespace Identity
} // namespace EtherCAT
