/**
 * @file PdoChannelLayout.cpp
 * @brief PDO->channel layout resolution (see PdoChannelLayout.hpp).
 */

#include "tether/Beckhoff/PdoChannelLayout.hpp"

#if TETHER_ENABLE_SII

namespace EtherCAT {
namespace Beckhoff {
namespace detail {

bool resolveValueChannels(std::span<const SII::SIIPDO> pdos,
                          uint8_t sm_channel,
                          uint8_t min_value_bits,
                          std::vector<ResolvedChannel>& channels,
                          uint32_t& image_bits) {
    channels.clear();
    image_bits = 0;

    uint32_t channel_start_bit = 0;
    uint32_t running_bit       = 0;

    for (const auto& pdo : pdos) {
        if (pdo.sync_manager != sm_channel) continue;

        // Locate every >= min_value_bits entry (in order).
        uint32_t entry_bit = 0;
        std::vector<ResolvedEntry> values;
        for (const auto& e : pdo.entries) {
            if (e.bit_length >= min_value_bits) {
                ResolvedEntry r{};
                r.byte_off = (running_bit + entry_bit) / 8;
                r.bit_len  = e.bit_length > 64 ? 64 : e.bit_length;
                r.index    = e.index;
                r.subindex = e.subindex;
                values.push_back(r);
            }
            entry_bit += e.bit_length;
        }

        if (!values.empty()) {
            // The whole image must stay byte-aligned for the value access
            // helpers to work — bail on exotic bit-packed layouts.  Check
            // the channel start, the PDO size, and every value entry's
            // start bit.
            entry_bit = 0;
            std::vector<uint32_t> value_bits;
            for (const auto& e : pdo.entries) {
                if (e.bit_length >= min_value_bits) {
                    value_bits.push_back(running_bit + entry_bit);
                }
                entry_bit += e.bit_length;
            }
            bool aligned = (running_bit % 8) == 0 &&
                           (pdo.totalBits() % 8) == 0;
            for (uint32_t vb : value_bits) {
                if (vb % 8) aligned = false;
            }
            if (!aligned) {
                channels.clear();
                return false;
            }

            ResolvedChannel ch{};
            ch.values   = std::move(values);
            ch.pdo_index = pdo.pdo_index;
            const uint32_t status_bits =
                value_bits.front() - channel_start_bit;
            ch.status_off = status_bits >= 16
                          ? static_cast<int16_t>(channel_start_bit / 8)
                          : -1;
            channels.push_back(std::move(ch));
            channel_start_bit = running_bit + pdo.totalBits();
        }
        running_bit += pdo.totalBits();
    }
    image_bits = running_bit;
    return true;
}

void resolveBitChannels(std::span<const SII::SIIPDO> pdos,
                        uint8_t sm_channel,
                        std::vector<uint32_t>& bit_offs,
                        uint32_t& image_bits) {
    bit_offs.clear();
    image_bits = 0;
    uint32_t running_bit = 0;
    for (const auto& pdo : pdos) {
        if (pdo.sync_manager != sm_channel) continue;
        for (const auto& e : pdo.entries) {
            if (e.bit_length == 1 && e.index != 0) {
                bit_offs.push_back(running_bit);
            }
            running_bit += e.bit_length;
        }
    }
    image_bits = running_bit;
}

std::vector<ResolvedFifo> resolveFifoChannels(
    std::span<const SII::SIIPDO> pdos, uint8_t sm_channel,
    uint16_t max_data) {
    std::vector<ResolvedFifo> out;
    uint32_t bit_off = 0;
    for (const auto& pdo : pdos) {
        if (pdo.sync_manager != sm_channel) continue;
        ResolvedFifo c;
        bool first = true;
        for (const auto& e : pdo.entries) {
            if (e.index == 0) { bit_off += e.bit_length; continue; }
            if (first && e.bit_length <= 16) {
                c.ctrl_off  = static_cast<uint16_t>(bit_off / 8);
                c.ctrl_bits = e.bit_length;
                first = false;
            } else if (e.bit_length == 8) {
                if (c.data_len == 0) {
                    c.data_off = static_cast<uint16_t>(bit_off / 8);
                }
                if (max_data == 0 || c.data_len < max_data) ++c.data_len;
            }
            bit_off += e.bit_length;
        }
        if (!first || c.data_len > 0) out.push_back(c);
    }
    return out;
}

uint16_t firstPdoIndex(std::span<const SII::SIIPDO> pdos,
                       uint8_t sm_channel) {
    for (const auto& pdo : pdos) {
        if (pdo.sync_manager == sm_channel) return pdo.pdo_index;
    }
    return 0;
}

bool findEntryBitOffset(std::span<const SII::SIIPDO> pdos,
                        uint8_t sm_channel,
                        uint16_t index, int subindex,
                        uint32_t occurrence,
                        uint32_t& bit_off) {
    uint32_t running_bit = 0;
    uint32_t seen        = 0;
    for (const auto& pdo : pdos) {
        if (pdo.sync_manager != sm_channel) continue;
        for (const auto& e : pdo.entries) {
            if (e.index == index &&
                (subindex < 0 || e.subindex == subindex)) {
                if (seen++ == occurrence) {
                    bit_off = running_bit;
                    return true;
                }
            }
            running_bit += e.bit_length;
        }
    }
    return false;
}

bool findValueEntry(std::span<const SII::SIIPDO> pdos,
                    uint8_t sm_channel,
                    uint16_t index, int subindex,
                    uint32_t occurrence, uint8_t min_bits,
                    uint32_t& bit_off, uint8_t& bit_len) {
    uint32_t running_bit = 0;
    uint32_t seen        = 0;
    for (const auto& pdo : pdos) {
        if (pdo.sync_manager != sm_channel) continue;
        for (const auto& e : pdo.entries) {
            if (e.index == index &&
                (subindex < 0 || e.subindex == subindex) &&
                e.bit_length >= min_bits) {
                if (seen++ == occurrence) {
                    bit_off = running_bit;
                    bit_len = e.bit_length;
                    return true;
                }
            }
            running_bit += e.bit_length;
        }
    }
    return false;
}

} // namespace detail
} // namespace Beckhoff
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
