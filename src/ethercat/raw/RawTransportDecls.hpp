/**
 * @file RawTransportDecls.hpp
 * @brief EtherCAT raw transport function declarations
 *
 * Extracted from internal.hpp. Contains:
 * - adp_for_slave_index helper
 * - configure_mailbox_from_sii, sii_read_string
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

// ============================================================================
// Higher-Level Helpers
// ============================================================================

uint16_t adp_for_slave_index(uint16_t slave_index);

// ============================================================================
// EEPROM/SII and Mailbox Configuration
// ============================================================================

bool configure_mailbox_from_sii(
    Master& master,
    uint16_t slave_index,
    uint16_t *out_wr_addr,
    uint16_t *out_wr_len,
    uint16_t *out_rd_addr,
    uint16_t *out_rd_len,
    uint16_t *out_mbx_proto);

bool sii_read_string(Master& master, uint16_t slave_index, uint16_t string_number, char *out, size_t out_cap);

} // namespace Raw
} // namespace EtherCAT
