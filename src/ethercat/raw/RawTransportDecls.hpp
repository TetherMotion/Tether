/**
 * @file RawTransportDecls.hpp
 * @brief EtherCAT raw transport function declarations
 *
 * Extracted from internal.hpp. Contains:
 * - Log deduplication functions
 * - adp_for_slave_index helper
 * - configure_mailbox_from_sii, sii_read_string
 * - coe_sdo_upload, coe_sdo_download
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Raw {

// ============================================================================
// Log Deduplication
// ============================================================================

void log_dedup_key(int level, const char *key, const char *msg);
void log_dedup(int level, const char *msg);

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

// ============================================================================
// CoE SDO Communication
// ============================================================================

bool coe_sdo_upload(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    uint8_t *out,
    size_t out_cap,
    size_t *out_len,
    bool diag_enabled = false,
    unsigned int poll_interval_ms = 5,
    unsigned int transaction_timeout_ms = 1000);

bool coe_sdo_download(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    const uint8_t *data,
    size_t data_len,
    bool diag_enabled = false,
    unsigned int poll_interval_ms = 5,
    unsigned int transaction_timeout_ms = 1000);

} // namespace Raw
} // namespace EtherCAT
