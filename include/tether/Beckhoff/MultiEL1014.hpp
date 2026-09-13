/**
 * @file MultiEL1014.hpp
 * @brief Flat bit-level interface across all Beckhoff EL1014 terminals on a bus
 *
 * MultiEL1014 is the EL1014-only specialization of the generic MultiInputTerminal
 * chain: detection is restricted to the EL1014 vendor/product identity and
 * module() returns a typed EL1014&.  All modules share one logical address
 * space (see MultiInputTerminal.hpp) — the cyclic exchange is a single LRW
 * datagram for the whole chain.
 *
 * Module 0 occupies bits 0-3, module 1 bits 4-7, and so on (within a
 * module, bit N = channel N).
 *
 * @code
 *   MultiEL1014<> ins(master);
 *   if (auto n = ins.detect(); !n || *n == 0) { ... none found ... }
 *   ins.start();                   // configures every module, enters OP
 *
 *   bool on = ins.bit(6);          // module 1, channel 3 state
 *   auto field = ins.bits();       // flat std::bitset of all inputs
 * @endcode
 */

#pragma once

#include <span>
#include <vector>

#include "tether/Beckhoff/EL1014.hpp"
#include "tether/Beckhoff/MultiInputTerminal.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxChannels = 256>
class MultiEL1014 : public MultiInputTerminal<MaxChannels> {
    using Base = MultiInputTerminal<MaxChannels>;

public:
    using Bitset       = typename Base::Bitset;
    using StartOptions = typename Base::StartOptions;
    using Error        = typename Base::Error;
    template <typename T = void>
    using Result = typename Base::template Result<T>;

    static constexpr size_t kChannelsPerModule = EL1014::kNumChannels;
    static constexpr size_t kMaxModules = MaxChannels / kChannelsPerModule;

    explicit MultiEL1014(Master& master) : Base(master) {}

    // -- Detection (EL1014 only) ---------------------------------------------

    /**
     * @brief Scan the bus for EL1014 terminals (shallow vendor/product scan).
     * @return Number of modules found, or an Error.
     */
    Result<size_t> detect() {
        return Base::detect(el1014Matcher(), this->shallowScan());
    }

    /**
     * @brief Populate the module list from an existing discovery result
     *        instead of rescanning the bus.
     *
     * Passing a deep scan (DiscoveryOption::All) lets each module reuse its
     * SII data in configure(), avoiding per-terminal EEPROM re-reads.
     */
    Result<size_t> detect(std::span<const DiscoveredSlave> scan) {
        return Base::detect(el1014Matcher(), scan);
    }

    // -- Typed module access ---------------------------------------------------

    /// Access the EL1014 driver for one module (throws std::out_of_range).
    EL1014&       module(size_t i)       { return static_cast<EL1014&>(Base::module(i)); }
    const EL1014& module(size_t i) const { return static_cast<const EL1014&>(Base::module(i)); }

private:
    /// Matcher constructing typed EL1014 devices (so module() can downcast).
    static std::span<const typename Base::DeviceMatcher> el1014Matcher() {
        static const typename Base::DeviceMatcher m[]{
            typename Base::DeviceMatcher{
                EL1014::kIdentity,
                [](Master& master, const DiscoveredSlave& s)
                    -> std::unique_ptr<IInputTerminal> {
                    return std::make_unique<EL1014>(master, s);
                }}};
        return std::span<const typename Base::DeviceMatcher>(m, 1);
    }
};

} // namespace Beckhoff

} // namespace EtherCAT
