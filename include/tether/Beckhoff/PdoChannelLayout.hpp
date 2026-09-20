/**
 * @file PdoChannelLayout.hpp
 * @brief Pure PDO->channel layout resolution shared by the value-terminal
 *        drivers (analog in, analog out, position/encoder)
 *
 * Beckhoff value terminals express their process image as a sequence of
 * PDOs assigned to a process-data sync manager.  Within each PDO the
 * entries form a status prefix (packed <16-bit status/control bits)
 * followed by one or more >=16-bit value entries (measurement, counter,
 * latch, setpoint).  resolveValueChannels() walks the SII PDO list for
 * one SM channel and returns the byte offsets of every value entry plus
 * the status-prefix offset per channel PDO.
 *
 * The function is pure — it depends only on the parsed SII structures —
 * so it is fully unit-testable with synthetic PDO data built from the
 * ESIs.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "tether/ethercat/EtherCATConfig.hpp"

#if TETHER_ENABLE_SII
#include "tether/sii/SIIParser.hpp"
#endif

namespace EtherCAT {
namespace Beckhoff {
namespace detail {

// -- Process-image field access ------------------------------------------------
// Pure bit/field primitives shared by all packed-image drivers.  They are
// unconditional: the no-SII fallback paths resolve offsets differently but
// read and write the process image identically.

/// Read bit `bit_off` from a process image; false when out of range.
inline bool imageBit(std::span<const uint8_t> image, uint32_t bit_off) {
    const size_t byte = bit_off / 8;
    return byte < image.size() &&
           ((image[byte] >> (bit_off % 8)) & 1u) != 0;
}

/// Write bit `bit_off` in a process image; no-op when out of range.
inline void setImageBit(std::span<uint8_t> image, uint32_t bit_off, bool v) {
    const size_t byte = bit_off / 8;
    if (byte >= image.size()) return;
    if (v) image[byte] |=  static_cast<uint8_t>(1u << (bit_off % 8));
    else   image[byte] &= ~static_cast<uint8_t>(1u << (bit_off % 8));
}

/// Sign-extend a `bits`-wide raw value to int64.  Bits above `bits` in
/// `raw` are masked off first, so callers may pass unmasked extractions.
inline int64_t signExtend64(uint64_t raw, uint8_t bits) {
    if (bits == 0)  return 0;
    if (bits >= 64) return static_cast<int64_t>(raw);
    raw &= (uint64_t{1} << bits) - 1;
    const uint64_t sign = uint64_t{1} << (bits - 1);
    return static_cast<int64_t>((raw ^ sign) - sign);
}

/// Read a little-endian field of `bits` bits starting at `data` and
/// sign-extend it to int64.  Reads ceil(bits/8) bytes — the caller must
/// have bounds-checked `data` against the image.
inline int64_t signExtendLE(const uint8_t* data, uint8_t bits) {
    uint64_t raw = 0;
    std::memcpy(&raw, data, (bits + 7) / 8);
    return signExtend64(raw, bits);
}

#if TETHER_ENABLE_SII

/// A resolved >=16-bit value entry inside one channel PDO.
struct ResolvedEntry {
    uint16_t byte_off = 0;    ///< byte offset within the SM image
    uint8_t  bit_len  = 0;    ///< entry width in bits (clamped to 64)
    uint16_t index    = 0;    ///< object dictionary index
    uint8_t  subindex = 0;    ///< object dictionary subindex
};

/// One channel PDO resolved into value entries plus a status prefix.
struct ResolvedChannel {
    /// All >= min_value_bits entries, in PDO order.  For a standard
    /// analog channel this is a single measurement; an encoder PDO
    /// typically resolves to {counter, latch}.
    std::vector<ResolvedEntry> values;
    /// Byte offset of the status prefix (the >=16-bit region between the
    /// channel start and the first value entry); -1 when absent.
    int16_t status_off = -1;
    /// PDO index this channel was resolved from.
    uint16_t pdo_index = 0;
};

/**
 * @brief Resolve the channel layout of one process-data sync manager.
 *
 * Walks `pdos` in order, keeps the entries whose SII record assigns them
 * to `sm_channel`, and interprets each PDO carrying at least one
 * >= min_value_bits entry as one channel.  PDOs without a value entry
 * (status-only blocks like the EL3356's "RMB Status") merge into the
 * status prefix of the following channel.
 *
 * @param pdos            SII PDO list for the direction (tx_pdos for
 *                        inputs, rx_pdos for outputs)
 * @param sm_channel      process-data SM channel index
 * @param min_value_bits  minimum entry width treated as a value
 *                        (16 for the standard families)
 * @param channels        out: resolved channel list
 * @param image_bits      out: total resolved image size in bits
 * @return false on a non-byte-aligned layout (unsupported); the outputs
 *         are cleared in that case.
 */
bool resolveValueChannels(std::span<const SII::SIIPDO> pdos,
                          uint8_t sm_channel,
                          uint8_t min_value_bits,
                          std::vector<ResolvedChannel>& channels,
                          uint32_t& image_bits);

/**
 * @brief Bit offsets of every 1-bit entry assigned to `sm_channel`,
 *        in PDO order.
 *
 * Used by the packed-bit drivers (combined I/O boxes, relay terminals,
 * breaker terminals) where each 1-bit entry is one logical channel.
 * Padding entries (index 0) are skipped, so `bit_offs[i]` is the offset
 * of the i-th real channel.
 *
 * @param bit_offs    out: absolute bit offset per channel
 * @param image_bits  out: total image size in bits
 */
void resolveBitChannels(std::span<const SII::SIIPDO> pdos,
                        uint8_t sm_channel,
                        std::vector<uint32_t>& bit_offs,
                        uint32_t& image_bits);

/// One resolved byte-FIFO channel inside a process-data SM image —
/// a leading 8/16-bit control/status register followed by N 8-bit
/// data entries (the EL6xxx serial/IO-Link shape).
struct ResolvedFifo {
    uint16_t ctrl_off = 0;    ///< byte offset of the ctrl/status field
    uint8_t  ctrl_bits = 0;   ///< ctrl/status width (0 = data-only PDO)
    uint16_t data_off = 0;    ///< byte offset of the first data byte
    uint16_t data_len = 0;    ///< data byte count
};

/**
 * @brief Resolve the FIFO channels of one process-data sync manager.
 *
 * Each PDO assigned to `sm_channel` is one channel: its first entry
 * (<=16 bit) is the control/status register, every following 8-bit
 * entry is one FIFO byte.  Padding entries (index 0) are skipped.
 * PDOs carrying only data entries produce channels with ctrl_bits = 0.
 *
 * @param max_data  cap on data bytes per channel (0 = unlimited)
 */
std::vector<ResolvedFifo> resolveFifoChannels(
    std::span<const SII::SIIPDO> pdos, uint8_t sm_channel,
    uint16_t max_data = 256);

/**
 * @brief First PDO index assigned to `sm_channel` in `pdos`, 0 when none.
 */
uint16_t firstPdoIndex(std::span<const SII::SIIPDO> pdos,
                       uint8_t sm_channel);

/**
 * @brief Absolute bit offset of a PDO entry inside `sm_channel`'s image.
 *
 * Locates the `occurrence`-th entry (0-based) whose object index and
 * subindex match.  Pass `subindex = -1` to ignore the subindex (some
 * ESI/SII revisions report 0 for every packed entry).
 *
 * The returned offset is in bits from the start of the SM image —
 * byte_offset = bit_off / 8, bit-in-byte = bit_off % 8 — so it addresses
 * packed 1-bit flags and wide fields uniformly.
 *
 * @return false when no such entry is assigned to `sm_channel`.
 */
bool findEntryBitOffset(std::span<const SII::SIIPDO> pdos,
                        uint8_t sm_channel,
                        uint16_t index, int subindex,
                        uint32_t occurrence,
                        uint32_t& bit_off);

/**
 * @brief Locate the `occurrence`-th entry with object `index` that is at
 *        least `min_bits` wide (the value/command fields).
 *
 * @param bit_off  out: absolute bit offset in the SM image
 * @param bit_len  out: entry width in bits
 */
bool findValueEntry(std::span<const SII::SIIPDO> pdos,
                    uint8_t sm_channel,
                    uint16_t index, int subindex,
                    uint32_t occurrence, uint8_t min_bits,
                    uint32_t& bit_off, uint8_t& bit_len);

#endif // TETHER_ENABLE_SII

} // namespace detail
} // namespace Beckhoff
} // namespace EtherCAT
