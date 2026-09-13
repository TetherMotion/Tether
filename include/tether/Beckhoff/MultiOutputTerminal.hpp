/**
 * @file MultiOutputTerminal.hpp
 * @brief Flat bit-level interface across a heterogeneous chain of output
 *        terminals sharing one logical address space
 *
 * MultiOutputTerminal collects devices implementing IOutputTerminal (any width —
 * 1, 2, 4, 8, ... bits) and presents their outputs as one contiguous set of
 * bits: the device at the lowest bus position occupies bits [0, w0), the
 * next device bits [w0, w0+w1), and so on (within a device, bit N = output
 * channel N).
 *
 * Efficiency: all devices are assigned to a dedicated PDO group
 * (Master::createPdoGroup) with its own LogicalAddressManager.  start()
 * builds one contiguous logical address map across the whole chain, so the
 * cyclic exchange is a *single LRW datagram* per cycle covering every
 * module — instead of one physical write per terminal.  Devices outside
 * the chain keep using the default pdo() manager and are unaffected.
 *
 * @code
 *   MultiOutputTerminal<> outs(master);
 *   outs.detect();                 // every known packed-output terminal
 *   outs.start();                  // shared LRW map + SAFE-OP + OP + RT loop
 *
 *   outs.setBit(6, true);
 *   outs.setOnly(9);
 *   outs.setBits(0b1010'0001);     // std::bitset bulk write
 *   bool on = outs.bit(6);
 *   outs.allOff();
 * @endcode
 *
 * Custom devices: pass a DeviceMatcher list to detect(), or attach() an
 * IOutputTerminal implementation directly:
 *
 * @code
 *   MultiOutputTerminal<>::DeviceMatcher myDevice{
 *       {0x00000002, 0x07D43052, 4, "EL2004"},
 *       [](Master& m, const DiscoveredSlave& s) {
 *           return std::make_unique<MyDevice>(m, s);
 *       }};
 *   outs.detect(std::span{&myDevice, 1});
 * @endcode
 *
 * The template parameter is the maximum number of output bits the flat
 * interface can address (default 256).  Devices that would overflow the
 * capacity are dropped from the chain in bus order.
 */

#pragma once

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

#include "tether/Beckhoff/IOutputTerminal.hpp"
#include "tether/Beckhoff/OutputTerminal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxBits = 256>
class MultiOutputTerminal {
public:
    /// Flat output bitset type — bit i = global channel i.
    using Bitset       = std::bitset<MaxBits>;
    using StartOptions = Beckhoff::StartOptions;
    using Error        = Beckhoff::Error;
    template <typename T = void>
    using Result = Beckhoff::Result<T>;

    /**
     * @brief How to recognize and construct a chained device.
     *
     * `identity` is matched against the discovered vendor/product IDs.
     * `create` constructs the IOutputTerminal; when empty, a generic
     * OutputTerminal is constructed for the identity.
     */
    struct DeviceMatcher {
        DeviceIdentity identity;
        std::function<std::unique_ptr<IOutputTerminal>(
            Master&, const DiscoveredSlave&)> create;

        /*implicit*/ DeviceMatcher(const DeviceIdentity& id)
            : identity(id) {}
        DeviceMatcher(const DeviceIdentity& id, decltype(create) f)
            : identity(id), create(std::move(f)) {}
    };

    /**
     * @brief Construct bound to a started master.  Performs no bus I/O —
     *        call detect()/attach() to populate the chain.
     */
    explicit MultiOutputTerminal(Master& master) : master_(master) {}

    ~MultiOutputTerminal() { stop(); }

    MultiOutputTerminal(MultiOutputTerminal&&)                 = delete;
    MultiOutputTerminal(const MultiOutputTerminal&)            = delete;
    MultiOutputTerminal& operator=(const MultiOutputTerminal&) = delete;

    /// All packed-output terminals known to this driver (ESI-verified).
    static std::span<const DeviceIdentity> knownDevices() {
        return std::span<const DeviceIdentity>(
            Devices::kOutputTerminals.data(),
            Devices::kOutputTerminals.size());
    }

    // -- Detection -----------------------------------------------------------

    /**
     * @brief Scan the bus for every known packed-output terminal
     *        (shallow vendor/product scan).
     * @return Number of devices in the chain, or an Error.
     */
    Result<size_t> detect() {
        return detect(knownDevices(), shallowScan());
    }

    /// Detect only the given identities (own bus scan).
    Result<size_t> detect(std::span<const DeviceIdentity> devices) {
        return detect(devices, shallowScan());
    }

    /// Detect with custom matchers (own bus scan).
    Result<size_t> detect(std::span<const DeviceMatcher> matchers) {
        return detect(matchers, shallowScan());
    }

    /**
     * @brief Populate the chain from an existing discovery result instead of
     *        rescanning the bus (all known identities).
     *
     * Passing a deep scan (DiscoveryOption::All) lets each device reuse its
     * SII data during bring-up, avoiding per-terminal EEPROM re-reads.
     */
    Result<size_t> detect(std::span<const DiscoveredSlave> scan) {
        return detect(knownDevices(), scan);
    }

    /// Detect only the given identities, reusing an existing scan.
    Result<size_t> detect(std::span<const DeviceIdentity> devices,
                          std::span<const DiscoveredSlave> scan) {
        const auto matchers = makeMatchers(devices);
        return detect(std::span<const DeviceMatcher>(matchers), scan);
    }

    /// Detect with custom matchers, reusing an existing scan.
    Result<size_t> detect(std::span<const DeviceMatcher> matchers,
                          std::span<const DiscoveredSlave> scan) {
        modules_.clear();
        configured_   = false;
        group_pdo_    = nullptr;
        group_lam_    = nullptr;
        for (const auto& s : scan) {
            for (const auto& m : matchers) {
                if (!OutputTerminal::matches(s, m.identity)) continue;
                if (m.create) modules_.push_back(m.create(master_, s));
                else modules_.push_back(
                    std::make_unique<OutputTerminal>(master_, s, m.identity));
                break;
            }
        }
        rebuildLayout();
        return modules_.size();
    }

    /**
     * @brief Append a pre-constructed chained device (any
     *        IOutputTerminal implementation).
     *
     * The flat bit layout is always sorted by bus position, so call order
     * does not matter.  Mixing attach() with detect() is allowed.
     */
    Result<> attach(std::unique_ptr<IOutputTerminal> device) {
        if (!device) return std::unexpected(Error::NoDeviceFound);
        modules_.push_back(std::move(device));
        rebuildLayout();
        return {};
    }

    // -- Chain layout ----------------------------------------------------------

    /// Number of devices in the chain.
    size_t moduleCount() const { return modules_.size(); }

    /// Total number of output bits across all devices.
    size_t channelCount() const { return total_bits_; }

    /// Access one chain device (throws std::out_of_range).
    IOutputTerminal&       module(size_t i)       { return *modules_.at(i); }
    const IOutputTerminal& module(size_t i) const { return *modules_.at(i); }

    /// Bus position of chain device `i`.
    uint16_t slaveIndex(size_t i) const { return modules_.at(i)->slaveIndex(); }

    /// Flat bit offset where device `i`'s outputs begin.
    size_t bitOffset(size_t i) const { return bit_offsets_.at(i); }

    // -- Bring-up --------------------------------------------------------------

    /**
     * @brief Configure the whole chain up to SAFE-OP.
     *
     * Creates a dedicated PDO group (own logical address space), prepares
     * every device, builds the shared logical address map once, then
     * programs each device's output FMMU and enters SAFE-OP.
     * @return Ok, or the first device's error (see lastErrorModule()).
     */
    Result<> configure() { return configureChain(StartOptions()); }

    /**
     * @brief Bring every device to OP and (by default) start the master's
     *        realtime loop exchanging the chain's PDO group — one LRW
     *        datagram per cycle for the whole chain.
     *
     * The loop is started once, before any OP request, so the terminals see
     * cyclic process data during their SAFE-OP -> OP transition.
     */
    Result<> start() { return start(StartOptions()); }
    Result<> start(const StartOptions& opts) {
        if (auto r = configureChain(opts); !r) return r;

        if (opts.manage_realtime_loop &&
            !master_.isMotionControlLoopRunning()) {
            master_.setMotionControlCallback(
                [this](double) {
                    master_.pdo().exchangeAll();
                    if (group_pdo_) group_pdo_->exchangeAll();
                    return true;
                });
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

    /// All outputs off on every device; stops the RT loop if we started it.
    void stop() {
        allOff();
        if (loop_started_) {
            Tether::Platform::Clock::instance().delayMilliseconds(20);
            master_.stopMotionControlLoop();
            loop_started_ = false;
        }
    }

    /// Index of the device that produced the last error (SIZE_MAX if none).
    size_t lastErrorModule() const { return last_error_module_; }

    // -- Introspection for app-managed loops -----------------------------------

    /// The chain's dedicated PDO manager (created by configure()/start()).
    /// Exchange it from your own cyclic loop when StartOptions::
    /// manage_realtime_loop is false: `outs.pdoManager()->exchangeAll()`.
    PDOManager* pdoManager() const { return group_pdo_; }

    /// The chain's logical address manager (same lifetime as pdoManager()).
    LogicalAddressManager* logicalAddressManager() const { return group_lam_; }

    /// Base of the chain's logical address space (0 before configure()).
    uint32_t baseLogicalAddress() const {
        return group_lam_ ? group_lam_->getBaseLogicalAddress() : 0;
    }

    // -- Flat bit interface ------------------------------------------------------
    // Global channel numbering: bit i maps to the device whose
    // [bitOffset, bitOffset + bitCount) range contains i.  Setters are
    // lock-free — safe from any thread while the RT loop runs.

    /// Set one global channel bit.  Throws std::out_of_range if >= channelCount().
    void setBit(size_t channel, bool on) {
        const size_t m = locate(channel);
        modules_[m]->setBit(channel - bit_offsets_[m], on);
    }

    /// Read one global channel bit (throws std::out_of_range).
    bool bit(size_t channel) const {
        const size_t m = locate(channel);
        return modules_[m]->bit(channel - bit_offsets_[m]);
    }

    /// Turn exactly one global channel on, everything else off.
    /// Throws std::out_of_range if >= channelCount().
    void setOnly(size_t channel) {
        const size_t m   = locate(channel);
        const uint64_t v = uint64_t{1} << (channel - bit_offsets_[m]);
        for (size_t i = 0; i < modules_.size(); ++i) {
            modules_[i]->setBits(i == m ? v : 0);
        }
    }

    /// Bulk-write outputs from a bitset (bits >= channelCount() are ignored).
    void setBits(const Bitset& bits) {
        for (size_t i = 0; i < modules_.size(); ++i) {
            const size_t w = modules_[i]->bitCount();
            uint64_t v = 0;
            for (size_t k = 0; k < w && k < 64; ++k) {
                if (bits[bit_offsets_[i] + k]) v |= uint64_t{1} << k;
            }
            modules_[i]->setBits(v);
        }
    }

    /// Current requested outputs as a flat bitset (bits >= channelCount() are 0).
    Bitset bits() const {
        Bitset b;
        for (size_t i = 0; i < modules_.size(); ++i) {
            const uint64_t v = modules_[i]->bits();
            const size_t w   = modules_[i]->bitCount();
            for (size_t k = 0; k < w && k < 64; ++k) {
                b[bit_offsets_[i] + k] = (v >> k) & 1u;
            }
        }
        return b;
    }

    /// Every channel on every device on.
    void allOn() { for (auto& m : modules_) m->setBits(~uint64_t{0}); }

    /// Every channel on every device off.
    void allOff() { for (auto& m : modules_) m->allOff(); }

protected:
    /// Shallow vendor/product scan used by detect() variants and subclasses.
    std::vector<DiscoveredSlave> shallowScan() {
        return master_.discovery().discover(
            {DiscoveryOption::VendorId, DiscoveryOption::ProductCode});
    }

    /// Bound master (for subclasses).
    Master& master() { return master_; }

private:
    static std::vector<DeviceMatcher> makeMatchers(
        std::span<const DeviceIdentity> devices) {
        std::vector<DeviceMatcher> out;
        out.reserve(devices.size());
        for (const auto& id : devices) out.emplace_back(id);
        return out;
    }

    /// Sort devices by bus position and rebuild the flat bit layout,
    /// dropping devices that would overflow MaxBits.
    void rebuildLayout() {
        std::stable_sort(modules_.begin(), modules_.end(),
            [](const auto& a, const auto& b) {
                return a->slaveIndex() < b->slaveIndex();
            });
        bit_offsets_.assign(modules_.size(), 0);
        total_bits_ = 0;
        size_t used = 0;
        for (size_t i = 0; i < modules_.size(); ++i) {
            const size_t w = modules_[i]->bitCount();
            if (total_bits_ + w > MaxBits) break;
            bit_offsets_[i] = total_bits_;
            total_bits_ += w;
            used = i + 1;
        }
        modules_.resize(used);
        bit_offsets_.resize(used);
    }

    /// Find the device covering flat bit `channel` (throws std::out_of_range).
    size_t locate(size_t channel) const {
        if (channel >= total_bits_) {
            throw std::out_of_range("MultiOutputTerminal bit index");
        }
        auto it = std::upper_bound(bit_offsets_.begin(), bit_offsets_.end(),
                                   channel);
        return static_cast<size_t>(
            std::distance(bit_offsets_.begin(), it)) - 1;
    }

    /// Group + shared logical map + FMMU programming, up to SAFE-OP.
    Result<> configureChain(const StartOptions& opts) {
        if (configured_) return {};
        if (modules_.empty()) return std::unexpected(Error::NoDeviceFound);
        rebuildLayout();

        // Assign the chain to a dedicated PDO group so it gets its own
        // PDOManager + LogicalAddressManager pair.  The shared logical
        // address space is then built once across the whole chain and
        // exchanged via a single LRW datagram — devices outside the chain
        // keep their own exchange path untouched.
        std::vector<uint16_t> idxs;
        idxs.reserve(modules_.size());
        for (const auto& m : modules_) idxs.push_back(m->slaveIndex());

        uint32_t base = opts.base_logical_addr;
        if (base == 0) {
            // Auto-select a range that does not collide with the default
            // LAM (0x10000) or other groups.
            base = 0x08000000u +
                   0x00010000u * uint32_t(master_.pdoGroups().size());
        }
        group_pdo_ = &master_.createPdoGroup(idxs, base);
        group_lam_ = &master_.logicalAddressManagerForSlave(idxs.front());

        // Phase 1: per-device SII/SM/PRE-OP + logical-mode PDO registration.
        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->prepareForLogicalExchange(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        rebuildLayout();  // widths may have been resolved from SII

        // Phase 2: build the shared logical address map once.
        uint16_t count = master_.getDiscoveredSlaveCount();
        if (count <= idxs.back()) {
            count = static_cast<uint16_t>(idxs.back() + 1);
        }
        if (!group_lam_->buildAddressMap(group_pdo_->slaveConfigs(), count)) {
            return std::unexpected(Error::LogicalMapMissing);
        }

        // Phase 3: per-device FMMU programming + SAFE-OP.
        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->mapLogicalAndEnterSafeOp(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        configured_ = true;
        return {};
    }

    Master&                                          master_;
    std::vector<std::unique_ptr<IOutputTerminal>>   modules_;
    std::vector<size_t>                              bit_offsets_;
    size_t                                           total_bits_        = 0;
    PDOManager*                                      group_pdo_         = nullptr;
    LogicalAddressManager*                           group_lam_         = nullptr;
    bool                                             configured_        = false;
    bool                                             loop_started_      = false;
    size_t                                           last_error_module_ = SIZE_MAX;
};

} // namespace Beckhoff

} // namespace EtherCAT
