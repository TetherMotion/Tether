#pragma once
/**
 * @file StateChangePrinter.hpp
 * @brief Per-bit change printer for sampled bitfields
 *
 * Complements StateChangeLogger: while that class fires on whole-value
 * changes, StateChangePrinter prints one line per *bit* that changed,
 * e.g. "X: DI3 0 -> 1".  Intended for digital-input / status-word style
 * bitfields polled on a cyclic path — quiet while stable, one compact
 * line per edge otherwise.
 *
 * Usage:
 * @code
 *   Tether::Utils::StateChangePrinter di("covestro", "X DI",
 *       {"neg_limit", "pos_limit", "home"});
 *   // per cycle:
 *   di.update(tx->digital_inputs);
 * @endcode
 */

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "logging/Logger.hpp"

namespace Tether {
namespace Utils {

class StateChangePrinter {
public:
    /// @param tag        Log tag passed to TETHER_LOGI.
    /// @param label      Prefix for each line (e.g. "X DI").
    /// @param bit_names  Optional names for bits 0..n; unnamed bits print
    ///                   as "bitN".
    StateChangePrinter(const char* tag, std::string label,
                       std::vector<std::string> bit_names = {})
        : tag_(tag), label_(std::move(label)), bit_names_(std::move(bit_names)) {}

    /// Record a new sample.  The first call prints the initial value once;
    /// subsequent calls print one line per changed bit.
    /// @return true if at least one bit changed.
    bool update(uint32_t value) {
        if (!last_.has_value()) {
            TETHER_LOGI(tag_, "{} initial state: 0x{:08X}", label_, value);
            last_ = value;
            return false;
        }
        const uint32_t diff = value ^ *last_;
        if (!diff)
            return false;
        for (uint32_t bit = 0; bit < 32; ++bit) {
            const uint32_t mask = 1u << bit;
            if (!(diff & mask))
                continue;
            const unsigned now = (value & mask) ? 1u : 0u;
            if (bit < bit_names_.size()) {
                TETHER_LOGI(tag_, "{}: {} {} -> {}", label_,
                            bit_names_[bit], now ^ 1u, now);
            } else {
                TETHER_LOGI(tag_, "{}: bit{} {} -> {}", label_, bit,
                            now ^ 1u, now);
            }
        }
        last_ = value;
        return true;
    }

    /// Forget the previous sample; the next update prints the initial state.
    void reset() { last_.reset(); }

    /// Last sampled value (undefined until the first update()).
    uint32_t last() const { return last_.value_or(0); }

private:
    const char* tag_;
    std::string label_;
    std::vector<std::string> bit_names_;
    std::optional<uint32_t> last_;
};

} // namespace Utils
} // namespace Tether
