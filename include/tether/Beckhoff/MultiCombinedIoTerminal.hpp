/**
 * @file MultiCombinedIoTerminal.hpp
 * @brief Flat bit-level interface across a heterogeneous chain of
 *        combined I/O devices sharing one logical address space
 *
 * MultiCombinedIoTerminal collects devices implementing
 * ICombinedIoTerminal (mixed digital I/O, breaker/relay terminals,
 * couplers with integrated I/O, LED drivers, ...) and presents their
 * channels as two contiguous flat ranges — inputs and outputs each
 * numbered from 0 across the whole chain in bus order.
 *
 * Efficiency: all devices are assigned to a dedicated PDO group
 * (Master::createPdoGroup) with its own LogicalAddressManager.  start()
 * builds one contiguous logical address map across the whole chain, so
 * the cyclic exchange is a *single LRW datagram* per cycle covering
 * every device — instead of one physical datagram per terminal.
 *
 * @code
 *   MultiCombinedIoTerminal<> io(master);
 *   io.detect();                  // every known combined I/O device
 *   io.start();                   // shared LRW map + SAFE-OP + OP + RT loop
 *
 *   bool v = io.input(3);            // flat input index
 *   io.setOutput(10, true);          // flat output index
 * @endcode
 *
 * Custom devices: pass a DeviceMatcher list to detect(), or attach() an
 * ICombinedIoTerminal implementation directly.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

#include "tether/Beckhoff/CombinedIoTerminal.hpp"
#include "tether/Beckhoff/ICombinedIoTerminal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxBits = 1024>
class MultiCombinedIoTerminal {
public:
    using StartOptions = Beckhoff::StartOptions;
    using Error        = Beckhoff::Error;
    template <typename T = void>
    using Result = Beckhoff::Result<T>;

    /**
     * @brief How to recognize and construct a chained device.
     */
    struct DeviceMatcher {
        DeviceIdentity identity;
        std::function<std::unique_ptr<ICombinedIoTerminal>(
            Master&, const DiscoveredSlave&)> create;

        /*implicit*/ DeviceMatcher(const DeviceIdentity& id)
            : identity(id) {}
        DeviceMatcher(const DeviceIdentity& id, decltype(create) f)
            : identity(id), create(std::move(f)) {}
    };

    /// Construct bound to a started master.  Performs no bus I/O —
    /// call detect()/attach() to populate the chain.
    explicit MultiCombinedIoTerminal(Master& master) : master_(master) {}

    ~MultiCombinedIoTerminal() { stop(); }

    MultiCombinedIoTerminal(MultiCombinedIoTerminal&&)                 = delete;
    MultiCombinedIoTerminal(const MultiCombinedIoTerminal&)            = delete;
    MultiCombinedIoTerminal& operator=(const MultiCombinedIoTerminal&) = delete;

    /// All combined-I/O devices known to this driver (ESI-verified).
    static std::span<const DeviceIdentity> knownDevices() {
        return std::span<const DeviceIdentity>(
            Devices::kCombinedIoTerminals.data(),
            Devices::kCombinedIoTerminals.size());
    }

    // -- Detection --------------------------------------------------------------

    /**
     * @brief Scan the bus for every known combined-I/O device
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

    /// Populate the chain from an existing discovery result (all known
    /// identities).
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
        configured_ = false;
        group_pdo_  = nullptr;
        group_lam_  = nullptr;
        for (const auto& s : scan) {
            for (const auto& m : matchers) {
                if (!CombinedIoTerminal::matches(s, m.identity)) continue;
                if (m.create) modules_.push_back(m.create(master_, s));
                else modules_.push_back(
                    std::make_unique<CombinedIoTerminal>(master_, s,
                                                         m.identity));
                break;
            }
        }
        rebuildLayout();
        return modules_.size();
    }

    /**
     * @brief Append a pre-constructed chained device (any
     *        ICombinedIoTerminal implementation).
     *
     * The flat channel layout is always sorted by bus position, so call
     * order does not matter.  Mixing attach() with detect() is allowed.
     */
    Result<> attach(std::unique_ptr<ICombinedIoTerminal> device) {
        if (!device) return std::unexpected(Error::NoDeviceFound);
        modules_.push_back(std::move(device));
        rebuildLayout();
        return {};
    }

    // -- Chain layout ---------------------------------------------------------------

    /// Number of devices in the chain.
    size_t moduleCount() const { return modules_.size(); }

    /// Total input/output channel counts across all devices.
    size_t inputChannelCount()  const { return total_in_; }
    size_t outputChannelCount() const { return total_out_; }

    /// Access one chain device (throws std::out_of_range).
    ICombinedIoTerminal&       module(size_t i)       { return *modules_.at(i); }
    const ICombinedIoTerminal& module(size_t i) const { return *modules_.at(i); }

    /// Bus position of chain device `i`.
    uint16_t slaveIndex(size_t i) const { return modules_.at(i)->slaveIndex(); }

    /// Flat offsets where device `i`'s inputs/outputs begin.
    size_t inputOffset(size_t i)  const { return in_offsets_.at(i); }
    size_t outputOffset(size_t i) const { return out_offsets_.at(i); }

    // -- Bring-up ---------------------------------------------------------------------

    /**
     * @brief Configure the whole chain up to SAFE-OP.
     */
    Result<> configure() { return configureChain(StartOptions()); }

    /**
     * @brief Bring every device to OP and (by default) start the master's
     *        realtime loop exchanging the chain's PDO group — one LRW
     *        datagram per cycle for the whole chain.
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

    /// Stop the RT loop if we started it.
    void stop() {
        if (loop_started_) {
            Tether::Platform::Clock::instance().delayMilliseconds(20);
            master_.stopMotionControlLoop();
            loop_started_ = false;
        }
    }

    /// Index of the device that produced the last error (SIZE_MAX if none).
    size_t lastErrorModule() const { return last_error_module_; }

    // -- Introspection for app-managed loops ---------------------------------------------

    /// The chain's dedicated PDO manager (created by configure()/start()).
    PDOManager* pdoManager() const { return group_pdo_; }

    /// The chain's logical address manager (same lifetime as pdoManager()).
    LogicalAddressManager* logicalAddressManager() const { return group_lam_; }

    /// Base of the chain's logical address space (0 before configure()).
    uint32_t baseLogicalAddress() const {
        return group_lam_ ? group_lam_->getBaseLogicalAddress() : 0;
    }

    // -- Flat channel interface --------------------------------------------------------------
    // Global numbering: flat bit i maps to the device whose
    // [offset, offset + count) range contains i.

    /// Latest state of flat input bit `i` (throws std::out_of_range).
    bool input(size_t i) const {
        const size_t m = locate(in_offsets_, total_in_, i);
        return modules_[m]->input(i - in_offsets_[m]);
    }

    /// Write flat output bit `i` (throws std::out_of_range).
    void setOutput(size_t i, bool on) {
        const size_t m = locate(out_offsets_, total_out_, i);
        modules_[m]->setOutput(i - out_offsets_[m], on);
    }

    /// Read-back of flat output bit `i` (throws std::out_of_range).
    bool output(size_t i) const {
        const size_t m = locate(out_offsets_, total_out_, i);
        return modules_[m]->output(i - out_offsets_[m]);
    }

    /// Clear every device's output image.
    void allOutputsOff() {
        for (auto& m : modules_) m->allOutputsOff();
    }

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

    /// Sort devices by bus position and rebuild the flat channel layout,
    /// dropping devices that would overflow MaxBits per direction.
    void rebuildLayout() {
        std::stable_sort(modules_.begin(), modules_.end(),
            [](const auto& a, const auto& b) {
                return a->slaveIndex() < b->slaveIndex();
            });
        in_offsets_.assign(modules_.size(), 0);
        out_offsets_.assign(modules_.size(), 0);
        total_in_ = total_out_ = 0;
        size_t used = 0;
        for (size_t i = 0; i < modules_.size(); ++i) {
            const size_t wi = modules_[i]->inputChannelCount();
            const size_t wo = modules_[i]->outputChannelCount();
            if (total_in_ + wi > MaxBits || total_out_ + wo > MaxBits) break;
            in_offsets_[i]  = total_in_;
            out_offsets_[i] = total_out_;
            total_in_  += wi;
            total_out_ += wo;
            used = i + 1;
        }
        modules_.resize(used);
        in_offsets_.resize(used);
        out_offsets_.resize(used);
    }

    /// Find the device covering flat bit `i` (throws std::out_of_range).
    static size_t locate(const std::vector<size_t>& offsets,
                         size_t total, size_t i) {
        if (i >= total) {
            throw std::out_of_range("MultiCombinedIoTerminal bit index");
        }
        auto it = std::upper_bound(offsets.begin(), offsets.end(), i);
        return static_cast<size_t>(std::distance(offsets.begin(), it)) - 1;
    }

    /// Group + shared logical map + FMMU programming, up to SAFE-OP.
    Result<> configureChain(const StartOptions& opts) {
        if (configured_) return {};
        if (modules_.empty()) return std::unexpected(Error::NoDeviceFound);
        rebuildLayout();

        std::vector<uint16_t> idxs;
        idxs.reserve(modules_.size());
        for (const auto& m : modules_) idxs.push_back(m->slaveIndex());

        uint32_t base = opts.base_logical_addr;
        if (base == 0) {
            base = 0x08000000u +
                   0x00010000u * uint32_t(master_.pdoGroups().size());
        }
        group_pdo_ = &master_.createPdoGroup(idxs, base);
        group_lam_ = &master_.logicalAddressManagerForSlave(idxs.front());

        // Phase 1: per-device SII/mailbox/SM/PRE-OP + logical-mode PDO
        // registration.
        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->prepareForLogicalExchange(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        rebuildLayout();  // channel counts may have been resolved from SII

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
    std::vector<std::unique_ptr<ICombinedIoTerminal>> modules_;
    std::vector<size_t>                              in_offsets_;
    std::vector<size_t>                              out_offsets_;
    size_t                                           total_in_          = 0;
    size_t                                           total_out_         = 0;
    PDOManager*                                      group_pdo_         = nullptr;
    LogicalAddressManager*                           group_lam_         = nullptr;
    bool                                             configured_        = false;
    bool                                             loop_started_      = false;
    size_t                                           last_error_module_ = SIZE_MAX;
};

} // namespace Beckhoff
} // namespace EtherCAT
