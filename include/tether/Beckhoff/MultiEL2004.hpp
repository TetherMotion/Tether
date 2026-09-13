/**
 * @file MultiEL2004.hpp
 * @brief Flat bit-level interface across all Beckhoff EL2004 terminals on a bus
 *
 * MultiEL2004 finds every EL2004 (vendor 0x00000002, product 0x07D43052) in
 * the discovered chain and presents their outputs as one contiguous set of
 * bits: module 0 occupies bits 0-3, module 1 bits 4-7, and so on (within a
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
 *
 * The template parameter is the maximum number of channel bits the interface
 * can address (default 256 = 64 terminals — far beyond what an E-Bus segment
 * can power).  If the bus contains more EL2004s than kMaxModules, detect()
 * keeps only the first kMaxModules terminals in bus order.
 */

#pragma once

#include <bitset>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

#include "tether/Beckhoff/EL2004.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxChannels = 256>
class MultiEL2004 {
public:
    /// Flat output bitset type — bit i = global channel i.
    using Bitset       = std::bitset<MaxChannels>;
    using StartOptions = EL2004::StartOptions;
    using Error        = EL2004::Error;
    template <typename T = void>
    using Result = std::expected<T, Error>;

    static constexpr size_t kChannelsPerModule = EL2004::kNumChannels;
    static constexpr size_t kMaxModules = MaxChannels / kChannelsPerModule;

    /**
     * @brief Construct bound to a started master.  Performs no bus I/O —
     *        call detect() to find the EL2004 terminals.
     */
    explicit MultiEL2004(Master& master) : master_(master) {}

    ~MultiEL2004() { stop(); }

    MultiEL2004(MultiEL2004&&)            = delete;
    MultiEL2004(const MultiEL2004&)       = delete;
    MultiEL2004& operator=(const MultiEL2004&) = delete;

    // -- Detection -----------------------------------------------------------

    /**
     * @brief Scan the bus for EL2004 terminals (shallow vendor/product scan).
     * @return Number of modules found, or an Error.
     */
    Result<size_t> detect() {
        auto scan = master_.discovery().discover(
            {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
        return detect(scan);
    }

    /**
     * @brief Populate the module list from an existing discovery result
     *        instead of rescanning the bus.
     *
     * Passing a deep scan (DiscoveryOption::All) lets each module reuse its
     * SII data in configure(), avoiding per-terminal EEPROM re-reads.
     */
    Result<size_t> detect(std::span<const DiscoveredSlave> scan) {
        modules_.clear();
        for (const auto& s : scan) {
            if (!EL2004::matches(s)) continue;
            if (modules_.size() >= kMaxModules) break;
            modules_.push_back(std::make_unique<EL2004>(master_, s));
        }
        return modules_.size();
    }

    // -- Bring-up --------------------------------------------------------------

    /// Number of detected EL2004 terminals.
    size_t moduleCount() const { return modules_.size(); }

    /// Total number of output bits (moduleCount() * 4).
    size_t channelCount() const { return modules_.size() * kChannelsPerModule; }

    /// Access the driver for one module (throws std::out_of_range).
    EL2004&       module(size_t i)       { return *modules_.at(i); }
    const EL2004& module(size_t i) const { return *modules_.at(i); }

    /// Bus position of module `i`.
    uint16_t slaveIndex(size_t i) const { return modules_.at(i)->slaveIndex(); }

    /**
     * @brief Configure every detected module up to SAFE-OP.
     * @return Ok, or the first module's error (see lastErrorModule()).
     */
    Result<> configure() {
        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->configure(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        return {};
    }

    /**
     * @brief Bring every module to OP and (by default) start the master's
     *        realtime loop with a default exchangeAll() callback.
     *
     * The loop is started once, before any OP request, so the terminals see
     * cyclic process data during their SAFE-OP -> OP transition.
     */
    Result<> start(const StartOptions& opts = StartOptions()) {
        if (auto r = configure(); !r) return r;

        if (opts.manage_realtime_loop &&
            !master_.isMotionControlLoopRunning()) {
            master_.setMotionControlCallback(
                [this](double) { master_.pdo().exchangeAll(); return true; });
            Master::RealtimeMotionLoopConfig cfg;
            cfg.cycle_period_us           = opts.cycle_period_us;
            cfg.enable_dc_synchronization = false;
            if (!master_.startRealtimeMotionControlLoop(cfg)) {
                return std::unexpected(Error::LoopStartFailed);
            }
            loop_started_ = true;
        }

        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->requestOp(opts.op_timeout_ms); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        return {};
    }

    /// All outputs off on every module; stops the RT loop if we started it.
    void stop() {
        allOff();
        if (loop_started_) {
            Tether::Platform::Clock::instance().delayMilliseconds(20);
            master_.stopMotionControlLoop();
            loop_started_ = false;
        }
    }

    /// Index of the module that produced the last error (SIZE_MAX if none).
    size_t lastErrorModule() const { return last_error_module_; }

    // -- Flat bit interface ------------------------------------------------------
    // Global channel numbering: bit i maps to module (i / 4), channel (i % 4).
    // Setters are lock-free — safe from any thread while the RT loop runs.

    /// Set one global channel bit.  Throws std::out_of_range if >= channelCount().
    void setBit(size_t channel, bool on) {
        modules_.at(channel / kChannelsPerModule)
            ->set(channel % kChannelsPerModule, on);
    }

    /// Read one global channel bit (throws std::out_of_range).
    bool bit(size_t channel) const {
        return modules_.at(channel / kChannelsPerModule)
            ->get(channel % kChannelsPerModule);
    }

    /// Turn exactly one global channel on, everything else off.
    /// Throws std::out_of_range if >= channelCount().
    void setOnly(size_t channel) {
        const size_t mod = channel / kChannelsPerModule;
        const size_t ch  = channel % kChannelsPerModule;
        if (mod >= modules_.size()) throw std::out_of_range("MultiEL2004::setOnly");
        for (size_t i = 0; i < modules_.size(); ++i) {
            modules_[i]->setChannels(
                i == mod ? EL2004::Channels{}.set(ch) : EL2004::Channels{});
        }
    }

    /// Bulk-write outputs from a bitset (bits >= channelCount() are ignored).
    void setBits(const Bitset& bits) {
        for (size_t i = 0; i < modules_.size(); ++i) {
            EL2004::Channels c;
            for (size_t k = 0; k < kChannelsPerModule; ++k) {
                c[k] = bits[i * kChannelsPerModule + k];
            }
            modules_[i]->setChannels(c);
        }
    }

    /// Current requested outputs as a flat bitset (bits >= channelCount() are 0).
    Bitset bits() const {
        Bitset b;
        for (size_t i = 0; i < modules_.size(); ++i) {
            const auto c = modules_[i]->channels();
            for (size_t k = 0; k < kChannelsPerModule; ++k) {
                b[i * kChannelsPerModule + k] = c[k];
            }
        }
        return b;
    }

    /// Every channel on every module on.
    void allOn()  { for (auto& m : modules_) m->allOn(); }

    /// Every channel on every module off.
    void allOff() { for (auto& m : modules_) m->allOff(); }

private:
    Master&                                master_;
    std::vector<std::unique_ptr<EL2004>>   modules_;
    bool                                   loop_started_      = false;
    size_t                                 last_error_module_ = SIZE_MAX;
};

} // namespace Beckhoff

} // namespace EtherCAT
