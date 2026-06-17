#include "tether/ethercat/utils/PDO.hpp"

#include <cinttypes>
#include <cstdarg>
#include <cstdio>
#include <vector>
#include <algorithm>
#include "tether/ethercat/CoEManager.hpp"
#include "logging/Logger.hpp"  // for TETHER_LOGI/W

namespace EtherCAT {
namespace Utils {
namespace {

void appendf(std::string& out, const char* fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    if (static_cast<size_t>(n) < sizeof(buf)) {
        out.append(buf, static_cast<size_t>(n));
        return;
    }

    // Rare: message didn't fit; allocate exact size.
    std::string tmp;
    tmp.resize(static_cast<size_t>(n) + 1);
    va_start(ap, fmt);
    std::vsnprintf(tmp.data(), tmp.size(), fmt, ap);
    va_end(ap);
    out.append(tmp.c_str(), static_cast<size_t>(n));
}

uint64_t readLe(const uint8_t* p, uint8_t size)
{
    uint64_t v = 0;
    for (uint8_t i = 0; i < size && i < 8; ++i) {
        v |= (static_cast<uint64_t>(p[i]) << (8u * i));
    }
    return v;
}

int64_t signExtend(uint64_t v, uint8_t size)
{
    if (size == 0 || size >= 8) {
        return static_cast<int64_t>(v);
    }
    const uint64_t mask = 1ull << (static_cast<uint64_t>(size) * 8ull - 1ull);
    const uint64_t full = (1ull << (static_cast<uint64_t>(size) * 8ull)) - 1ull;
    const uint64_t clipped = v & full;
    if (clipped & mask) {
        return static_cast<int64_t>(clipped | ~full);
    }
    return static_cast<int64_t>(clipped);
}

void appendHexdump(std::string& out, const uint8_t* buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        out += "    <empty>\n";
        return;
    }

    for (size_t i = 0; i < buffer_size; i += 16) {
        appendf(out, "    %04" PRIX64 ":", static_cast<uint64_t>(i));
        const size_t line_end = (i + 16 < buffer_size) ? (i + 16) : buffer_size;
        for (size_t j = i; j < line_end; ++j) {
            appendf(out, " %02X", buffer[j]);
        }
        out += "\n";
    }
}

} // namespace

std::string pdoToString(bool is_tx,
                        uint16_t pdo_index,
                        const uint8_t* buffer,
                        size_t buffer_size,
                        const PDOFieldDescriptor* fields,
                        size_t field_count)
{
    std::string out;
    out.reserve(512);

    appendf(out, "  %sPDO 0x%04X (%zu bytes):\n", is_tx ? "Tx" : "Rx", pdo_index, buffer_size);

    if (!fields || field_count == 0) {
        appendHexdump(out, buffer, buffer_size);
        return out;
    }

    for (size_t i = 0; i < field_count; ++i) {
        const auto& f = fields[i];
        const char* desc = f.description ? f.description : "<unnamed>";

        if (!buffer || buffer_size == 0) {
            appendf(out, "    %s (0x%04X:%u, off=%u, sz=%u): <no buffer>\n",
                    desc, f.index, static_cast<unsigned>(f.subindex),
                    static_cast<unsigned>(f.offset), static_cast<unsigned>(f.size));
            continue;
        }

        const size_t end = static_cast<size_t>(f.offset) + static_cast<size_t>(f.size);
        if (end > buffer_size) {
            appendf(out, "    %s (0x%04X:%u, off=%u, sz=%u): <out of range>\n",
                    desc, f.index, static_cast<unsigned>(f.subindex),
                    static_cast<unsigned>(f.offset), static_cast<unsigned>(f.size));
            continue;
        }

        const uint64_t raw = readLe(buffer + f.offset, f.size);
        const int width = static_cast<int>(f.size) * 2;

        if (f.size == 1 || f.size == 2 || f.size == 4 || f.size == 8) {
            const int64_t sval = signExtend(raw, f.size);
            appendf(out, "    %s (0x%04X:%u, off=%u, sz=%u): 0x%0*" PRIX64 " (%" PRId64 ")\n",
                    desc, f.index, static_cast<unsigned>(f.subindex),
                    static_cast<unsigned>(f.offset), static_cast<unsigned>(f.size),
                    width, raw, sval);
        } else {
            appendf(out, "    %s (0x%04X:%u, off=%u, sz=%u): 0x%0*" PRIX64 "\n",
                    desc, f.index, static_cast<unsigned>(f.subindex),
                    static_cast<unsigned>(f.offset), static_cast<unsigned>(f.size),
                    width, raw);
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// New mapping helpers
// ---------------------------------------------------------------------------

bool readPDOMapping(CoE::CoEManager& coe,
                    uint16_t pdo_index,
                    std::vector<PDOMappingEntry>& out_entries,
                    uint32_t timeout_ms)
{
    out_entries.clear();

    // read count (subindex 0)
    auto count_res = coe.readU8(pdo_index, 0, {.timeout_ms = timeout_ms});
    if (!count_res.has_value()) {
        return false;
    }
    uint8_t count = count_res.value();

    uint16_t running_offset = 0;
    for (uint8_t si = 1; si <= count; ++si) {
        if (si > 0xFF) {
            // subindex is a single byte; stop if it rolls over
            break;
        }
        auto entry_res = coe.readU32(pdo_index, si, {.timeout_ms = timeout_ms});
        if (!entry_res.has_value()) {
            // stop on first failure but keep what we've gathered so far
            break;
        }
        uint32_t entry = entry_res.value();
        uint16_t obj_idx = static_cast<uint16_t>((entry >> 16) & 0xFFFF);
        uint8_t sub_idx = static_cast<uint8_t>((entry >> 8) & 0xFF);
        uint8_t bits    = static_cast<uint8_t>(entry & 0xFF);

        PDOMappingEntry e{};
        e.index = obj_idx;
        e.subindex = sub_idx;
        e.bit_length = bits;
        e.byte_offset = running_offset;
        out_entries.push_back(e);

        // increment by floor(bits/8) to match example behaviour
        running_offset += static_cast<uint16_t>(bits / 8);
    }

    return true;
}

std::string pdoMappingToString(bool is_tx,
                               uint16_t pdo_index,
                               const std::vector<PDOMappingEntry>& entries)
{
    std::string out;
    out.reserve(256);

    appendf(out, "  %sPDO 0x%04X Mapping Entries:\n", is_tx ? "Tx" : "Rx", pdo_index);
    uint16_t total_bytes = 0;
    for (size_t i = 0; i < entries.size(); ++i) {
        const auto& e = entries[i];
        appendf(out,
                "    0x%04X:%02zu → obj 0x%04X:%02u, %u bits (offset %u bytes)\n",
                pdo_index, static_cast<unsigned>(i + 1),
                e.index, static_cast<unsigned>(e.subindex),
                static_cast<unsigned>(e.bit_length),
                static_cast<unsigned>(e.byte_offset));
        total_bytes = static_cast<uint16_t>(e.byte_offset + (e.bit_length / 8));
    }
    appendf(out, "  Total mapped: %u bytes\n", total_bytes);
    return out;
}

// ---------------------------------------------------------------------------
// Convenience logging wrapper
// ---------------------------------------------------------------------------

void printPDOMapping(CoE::CoEManager& coe,
                     bool is_tx,
                     uint16_t pdo_index,
                     const char* tag,
                     uint32_t timeout_ms)
{
    std::vector<PDOMappingEntry> entries;
    if (readPDOMapping(coe, pdo_index, entries, timeout_ms)) {
        TETHER_LOGI(tag, "%s", pdoMappingToString(is_tx, pdo_index, entries).c_str());
    } else {
        TETHER_LOGW(tag, "PDO 0x%04X mapping read FAILED", pdo_index);
    }
}

} // namespace Utils
} // namespace EtherCAT
