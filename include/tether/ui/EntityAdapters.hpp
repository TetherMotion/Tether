#pragma once

/**
 * @file EntityAdapters.hpp
 * @brief Composable ISlaveEntityProvider / IRegisterReader adapters for
 *        the EntityBrowser model (`tether/ui/Panels.hpp`).
 *
 * Applications that only need to list physical devices wire
 * `MasterEntityProvider` + `SdoRegisterReader` (EthercatPanels.hpp).
 * Applications with virtual participants — emulated channels,
 * computed pseudo-devices, remote nodes — compose these adapters:
 *
 *   - `CompositeEntityProvider` appends entities returned by a callback
 *     to an inner provider's list.  The callback is invoked on every
 *     `entities()` call so virtual rows can reflect live state (e.g.
 *     a virtual entity whose `state`/`detail` fields mirror protocol
 *     counters).
 *   - `DispatchingRegisterReader` routes `read()` by entity index:
 *     indices accepted by a predicate go to an injected handler
 *     (synthesized values), everything else delegates to the inner
 *     reader (e.g. real SDO traffic).
 *
 * The ESC211 blackchannel uses them to list emulated FSoE channels
 * alongside bus slaves and to serve synthetic registers for them.
 */

#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "tether/ui/Panels.hpp"

namespace Tether {
namespace UI {

/// Provider that appends callback-supplied entities after an inner
/// provider's list.  The extra source is called on every `entities()`.
class CompositeEntityProvider : public ISlaveEntityProvider {
public:
    /// Produces the appended rows; called on every entities() — may
    /// return different state each time (live virtual status).
    using ExtraSource = std::function<std::vector<SlaveEntity>()>;

    CompositeEntityProvider(ISlaveEntityProvider& inner,
                            ExtraSource extra)
        : inner_(inner), extra_(std::move(extra)) {}

    std::vector<SlaveEntity> entities() override {
        std::vector<SlaveEntity> out = inner_.entities();
        if (extra_) {
            auto extra = extra_();
            out.insert(out.end(),
                       std::make_move_iterator(extra.begin()),
                       std::make_move_iterator(extra.end()));
        }
        return out;
    }

private:
    ISlaveEntityProvider& inner_;
    ExtraSource extra_;
};

/// Register reader that dispatches by entity index: entities accepted
/// by `is_virtual` are answered by `read_virtual` (synthetic data, no
/// bus traffic); all other indices delegate to `inner`.
class DispatchingRegisterReader : public IRegisterReader {
public:
    using Predicate = std::function<bool(uint16_t entity_index)>;
    using Handler   = std::function<RegisterResult(
        uint16_t entity_index, const NamedRegister& reg)>;

    DispatchingRegisterReader(IRegisterReader& inner,
                              Predicate is_virtual,
                              Handler read_virtual)
        : inner_(inner),
          is_virtual_(std::move(is_virtual)),
          read_virtual_(std::move(read_virtual)) {}

    RegisterResult read(uint16_t entity_index,
                        const NamedRegister& reg) override {
        if (is_virtual_ && is_virtual_(entity_index)) {
            return read_virtual_
                ? read_virtual_(entity_index, reg)
                : RegisterResult{false, {}, "no handler"};
        }
        return inner_.read(entity_index, reg);
    }

private:
    IRegisterReader& inner_;
    Predicate is_virtual_;
    Handler   read_virtual_;
};

} // namespace UI
} // namespace Tether
