/**
 * @file MultiPositionInputTerminal.hpp
 * @brief Flat channel-level interface across a heterogeneous chain of
 *        position/encoder input terminals sharing one logical address
 *        space
 *
 * MultiPositionInputTerminal collects devices implementing
 * IPositionInputTerminal (any channel count) and presents their position
 * inputs as one contiguous set of channels: the device at the lowest bus
 * position occupies channels [0, c0), the next [c0, c0+c1), and so on.
 *
 * Efficiency: all devices are assigned to a dedicated PDO group
 * (Master::createPdoGroup) with its own LogicalAddressManager.  start()
 * builds one contiguous logical address map across the whole chain, so
 * the cyclic exchange is a *single LRW datagram* per cycle covering every
 * module — instead of one physical read per terminal.
 *
 * @code
 *   MultiPositionInputTerminal<> encs(master);
 *   encs.detect();                  // every known EL5xxx terminal
 *   encs.start();                   // shared LRW map + SAFE-OP + OP + RT
 *
 *   int64_t p = encs.position(3);   // flat channel index
 *   int64_t l = encs.latch(3);      // first latch value (0 if none)
 * @endcode
 *
 * The template parameter is the maximum number of channels the flat
 * interface can address (default 128).  Devices that would overflow the
 * capacity are dropped from the chain in bus order.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

#include "tether/Beckhoff/IPositionInputTerminal.hpp"
#include "tether/Beckhoff/PositionInputTerminal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/SlaveDiscoveryManager.hpp"
#include "tether/platform/Platform.hpp"

namespace EtherCAT {
namespace Beckhoff {

template <size_t MaxChannels = 128>
class MultiPositionInputTerminal {
public:
    using StartOptions = Beckhoff::StartOptions;
    using Error        = Beckhoff::Error;
    template <typename T = void>
    using Result = Beckhoff::Result<T>;

    /**
     * @brief How to recognize and construct a chained device.
     *
     * `identity` is matched against the discovered vendor/product IDs.
     * `create` constructs the IPositionInputTerminal; when empty, a
     * generic PositionInputTerminal is constructed for the identity.
     */
    struct DeviceMatcher {
        DeviceIdentity identity;
        std::function<std::unique_ptr<IPositionInputTerminal>(
            Master&, const DiscoveredSlave&)> create;

        /*implicit*/ DeviceMatcher(const DeviceIdentity& id)
            : identity(id) {}
        DeviceMatcher(const DeviceIdentity& id, decltype(create) f)
            : identity(id), create(std::move(f)) {}
    };

    /// Bind to a started master — no bus I/O until detect()/attach().
    explicit MultiPositionInputTerminal(Master& master) : master_(master) {}

    ~MultiPositionInputTerminal() { stop(); }

    MultiPositionInputTerminal(MultiPositionInputTerminal&&)            = delete;
    MultiPositionInputTerminal(const MultiPositionInputTerminal&)       = delete;
    MultiPositionInputTerminal& operator=(
        const MultiPositionInputTerminal&)                              = delete;

    /// All position-input terminals known to this driver (ESI-verified).
    static std::span<const DeviceIdentity> knownDevices() {
        return std::span<const DeviceIdentity>(
            Devices::kPositionInputTerminals.data(),
            Devices::kPositionInputTerminals.size());
    }

    // -- Detection ---------------------------------------------------------------

    /// Scan the bus for every known position-input terminal.
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

    /// Populate the chain from an existing discovery result.
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
                if (!PositionInputTerminal::matches(s, m.identity)) continue;
                if (m.create) modules_.push_back(m.create(master_, s));
                else modules_.push_back(
                    std::make_unique<PositionInputTerminal>(master_, s,
                                                            m.identity));
                break;
            }
        }
        rebuildLayout();
        return modules_.size();
    }

    /// Append a pre-constructed chained device (any IPositionInputTerminal).
    Result<> attach(std::unique_ptr<IPositionInputTerminal> device) {
        if (!device) return std::unexpected(Error::NoDeviceFound);
        modules_.push_back(std::move(device));
        rebuildLayout();
        return {};
    }

    // -- Chain layout ---------------------------------------------------------------

    /// Number of devices in the chain.
    size_t moduleCount() const { return modules_.size(); }

    /// Total number of position channels across all devices.
    size_t channelCount() const { return total_channels_; }

    /// Access one chain device (throws std::out_of_range).
    IPositionInputTerminal&       module(size_t i)       { return *modules_.at(i); }
    const IPositionInputTerminal& module(size_t i) const { return *modules_.at(i); }

    /// Bus position of chain device `i`.
    uint16_t slaveIndex(size_t i) const { return modules_.at(i)->slaveIndex(); }

    /// Flat channel offset where device `i`'s channels begin.
    size_t channelOffset(size_t i) const { return channel_offsets_.at(i); }

    // -- Bring-up ----------------------------------------------------------------------

    /// Configure the whole chain up to SAFE-OP (dedicated PDO group +
    /// shared logical map + per-device FMMU).
    Result<> configure() { return configureChain(StartOptions()); }

    /// Bring every device to OP and (by default) start the master's
    /// realtime loop exchanging the chain's PDO group — one LRW datagram
    /// per cycle for the whole chain.
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

    // -- Introspection for app-managed loops --------------------------------------------

    /// The chain's dedicated PDO manager (created by configure()/start()).
    PDOManager* pdoManager() const { return group_pdo_; }

    /// The chain's logical address manager (same lifetime).
    LogicalAddressManager* logicalAddressManager() const { return group_lam_; }

    /// Base of the chain's logical address space (0 before configure()).
    uint32_t baseLogicalAddress() const {
        return group_lam_ ? group_lam_->getBaseLogicalAddress() : 0;
    }

    // -- Flat channel interface ------------------------------------------------------------

    /// Position/counter of flat channel `i` (throws std::out_of_range).
    int64_t position(size_t channel) const { return value(channel, 0); }

    /// Raw (zero-extended) position of flat channel `i`.
    uint64_t rawPosition(size_t channel) const { return rawValue(channel, 0); }

    /// i-th value field of flat channel `ch`, sign-extended.
    int64_t value(size_t channel, size_t i) const {
        const size_t m = locate(channel);
        return modules_[m]->value(channel - channel_offsets_[m], i);
    }

    /// i-th value field of flat channel `ch`, zero-extended.
    uint64_t rawValue(size_t channel, size_t i) const {
        const size_t m = locate(channel);
        return modules_[m]->rawValue(channel - channel_offsets_[m], i);
    }

    /// First latch value of flat channel `i` (0 when none).
    int64_t latch(size_t channel) const { return value(channel, 1); }

    /// Number of value fields of flat channel `i`.
    size_t valueCount(size_t channel) const {
        const size_t m = locate(channel);
        return modules_[m]->valueCount(channel - channel_offsets_[m]);
    }

    /// Raw status word of flat channel `i` (0 if none).
    uint16_t status(size_t channel) const {
        const size_t m = locate(channel);
        return modules_[m]->status(channel - channel_offsets_[m]);
    }

    /// Whether flat channel `i` carries a status field.
    bool hasStatus(size_t channel) const {
        const size_t m = locate(channel);
        return modules_[m]->hasStatus(channel - channel_offsets_[m]);
    }

    /// Whether flat channel `i` carries a latch field.
    bool hasLatch(size_t channel) const {
        return valueCount(channel) > 1;
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
    /// dropping devices that would overflow MaxChannels.
    void rebuildLayout() {
        std::stable_sort(modules_.begin(), modules_.end(),
            [](const auto& a, const auto& b) {
                return a->slaveIndex() < b->slaveIndex();
            });
        channel_offsets_.assign(modules_.size(), 0);
        total_channels_ = 0;
        size_t used = 0;
        for (size_t i = 0; i < modules_.size(); ++i) {
            const size_t w = modules_[i]->channelCount();
            if (total_channels_ + w > MaxChannels) break;
            channel_offsets_[i] = total_channels_;
            total_channels_ += w;
            used = i + 1;
        }
        modules_.resize(used);
        channel_offsets_.resize(used);
    }

    /// Find the device covering flat channel `i` (throws std::out_of_range).
    size_t locate(size_t channel) const {
        if (channel >= total_channels_) {
            throw std::out_of_range("MultiPositionInputTerminal channel index");
        }
        auto it = std::upper_bound(channel_offsets_.begin(),
                                   channel_offsets_.end(), channel);
        return static_cast<size_t>(
            std::distance(channel_offsets_.begin(), it)) - 1;
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

        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->prepareForLogicalExchange(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        rebuildLayout();

        uint16_t count = master_.getDiscoveredSlaveCount();
        if (count <= idxs.back()) {
            count = static_cast<uint16_t>(idxs.back() + 1);
        }
        if (!group_lam_->buildAddressMap(group_pdo_->slaveConfigs(), count)) {
            return std::unexpected(Error::LogicalMapMissing);
        }

        for (size_t i = 0; i < modules_.size(); ++i) {
            if (auto r = modules_[i]->mapLogicalAndEnterSafeOp(); !r) {
                last_error_module_ = i;
                return r;
            }
        }
        configured_ = true;
        return {};
    }

    Master&                                             master_;
    std::vector<std::unique_ptr<IPositionInputTerminal>> modules_;
    std::vector<size_t>                                 channel_offsets_;
    size_t                                              total_channels_    = 0;
    PDOManager*                                         group_pdo_         = nullptr;
    LogicalAddressManager*                              group_lam_         = nullptr;
    bool                                                configured_        = false;
    bool                                                loop_started_      = false;
    size_t                                              last_error_module_ = SIZE_MAX;
};

} // namespace Beckhoff

} // namespace EtherCAT
