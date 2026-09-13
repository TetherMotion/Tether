/**
 * @file MultiEL2004.hpp
 * @brief Flat bit-level interface across all Beckhoff EL2004 terminals on a bus
 *
 * MultiEL2004 is the EL2004-only specialization of the generic MultiOutput
 * chain: detection is restricted to the EL2004 vendor/product identity and
 * module() returns a typed EL2004&.  All modules share one logical address
 * space (see MultiOutput.hpp) — the cyclic exchange is a single LRW
 * datagram for the whole chain.
 *
 * Module 0 occupies bits 0-3, module 1 bits 4-7, and so on (within a
 * module, bit N = channel N).
 *
 * @code
 *   MultiEL2004<> outs(master);
 *   if (auto n = outs.detect(); !n || *n == 0) { ... none found ... }
 *   outs.start();                  // configures every module, enters OP
 *
 *   outs.setBit(6, true);          // module 1, channel 3 ON
 *   outs.setOnly(9);               // only module 2 channel 2 ON
 *   outs.setBits(0b1010'0001);     // bitset bulk write
 *   bool on = outs.bit(6);
 *   outs.allOff();
 * @endcode
 */

#pragma once

#include <span>
#include <vector>

#include "tether/Beckhoff/EL2004.hpp"
#include "tether/Beckhoff/MultiOutput.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxChannels = 256>
class MultiEL2004 : public MultiOutput<MaxChannels> {
    using Base = MultiOutput<MaxChannels>;

public:
    using Bitset       = typename Base::Bitset;
    using StartOptions = typename Base::StartOptions;
    using Error        = typename Base::Error;
    template <typename T = void>
    using Result = typename Base::template Result<T>;

    static constexpr size_t kChannelsPerModule = EL2004::kNumChannels;
    static constexpr size_t kMaxModules = MaxChannels / kChannelsPerModule;

    explicit MultiEL2004(Master& master) : Base(master) {}

    // -- Detection (EL2004 only) ---------------------------------------------

    /**
     * @brief Scan the bus for EL2004 terminals (shallow vendor/product scan).
     * @return Number of modules found, or an Error.
     */
    Result<size_t> detect() {
        return Base::detect(el2004Matcher(), this->shallowScan());
    }

    /**
     * @brief Populate the module list from an existing discovery result
     *        instead of rescanning the bus.
     *
     * Passing a deep scan (DiscoveryOption::All) lets each module reuse its
     * SII data in configure(), avoiding per-terminal EEPROM re-reads.
     */
    Result<size_t> detect(std::span<const DiscoveredSlave> scan) {
        return Base::detect(el2004Matcher(), scan);
    }

    // -- Typed module access ---------------------------------------------------

    /// Access the EL2004 driver for one module (throws std::out_of_range).
    EL2004&       module(size_t i)       { return static_cast<EL2004&>(Base::module(i)); }
    const EL2004& module(size_t i) const { return static_cast<const EL2004&>(Base::module(i)); }

private:
    /// Matcher constructing typed EL2004 devices (so module() can downcast).
    static std::span<const typename Base::DeviceMatcher> el2004Matcher() {
        static const typename Base::DeviceMatcher m[]{
            typename Base::DeviceMatcher{
                EL2004::kIdentity,
                [](Master& master, const DiscoveredSlave& s)
                    -> std::unique_ptr<IChainableOutput> {
                    return std::make_unique<EL2004>(master, s);
                }}};
        return std::span<const typename Base::DeviceMatcher>(m, 1);
    }
};

} // namespace Beckhoff

} // namespace EtherCAT
