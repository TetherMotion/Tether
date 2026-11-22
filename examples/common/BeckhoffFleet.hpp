// SPDX-License-Identifier: MIT
/**
 * @file BeckhoffFleet.hpp
 * @brief Emulated Beckhoff I/O fleet behind two EK1100 couplers.
 *
 * Builds a 10-node EtherCAT chain that mirrors a real Beckhoff segment:
 *
 *   EK1100 #1 -> EL1004 -> EL2004 -> EL3104 -> EL4104
 *   EK1100 #2 -> EL1004 -> EL2004 -> EL3104 -> EL4104
 *
 * Every node is a slave::CiA401Slave carrying the real Beckhoff vendor /
 * product identity (vendor 0x00000002).  The EK1100 couplers have no
 * process data of their own, matching the real part.  Each I/O module
 * gets the PDO layout CiA401Slave produces for its channel counts:
 *
 *   TxPDO (slave -> master): [digital inputs][analog inputs int16 LE]
 *   RxPDO (master -> slave): [digital outputs][analog outputs int16 LE]
 *
 * PDO buffers are kept coherent with the profile I/O arrays by the
 * per-tick updateTxPDO()/processRxPDO() calls in tick(), so the fleet
 * works with every master exchange mode (position, configured address,
 * or logical LRW via the PDO exchange callback).
 *
 * The fleet description data (identities, catalog, fleet text
 * language, scan helpers) lives in the library header
 * <tether/ethercat/BeckhoffFleet.hpp>; this file keeps the emulated
 * slave machinery and re-exports the old tether::examples names.
 */
#pragma once

#include <tether/ethercat/BeckhoffFleet.hpp>

#include "slave/profiles/CiA401Slave.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tether::examples {

/* ---- compatibility aliases ----------------------------------------
   The fleet catalog/description moved to the library header
   tether::ethercat::beckhoff; these aliases keep every existing
   `tether::examples::{beckhoff_ids,FleetSlaveInfo,fleetInfos,
   detail::*}` call site compiling unchanged. */
namespace beckhoff_ids = tether::ethercat::beckhoff::ids;
using FleetSlaveInfo = tether::ethercat::beckhoff::FleetSlaveInfo;
using tether::ethercat::beckhoff::fleetInfos;

namespace detail {

using tether::ethercat::beckhoff::detail::FleetModuleType;
using tether::ethercat::beckhoff::detail::fleetModuleCatalog;
using tether::ethercat::beckhoff::detail::fleetModuleType;
using tether::ethercat::beckhoff::detail::lowerStr;
using tether::ethercat::beckhoff::detail::fleetInfosFromText;
using tether::ethercat::beckhoff::detail::fleetInfosFromFile;
using tether::ethercat::beckhoff::detail::FleetScanSlave;
using tether::ethercat::beckhoff::detail::FleetScanResult;
using tether::ethercat::beckhoff::detail::fleetModuleForProduct;
using tether::ethercat::beckhoff::detail::fleetIsCouplerProduct;
using tether::ethercat::beckhoff::detail::fleetTextFromScan;
using tether::ethercat::beckhoff::detail::fleetTextFromInfos;

inline EtherCAT::slave::CiA401SlaveConfig moduleConfig(
    const FleetSlaveInfo& inf, uint8_t di8, uint8_t do8,
    uint8_t ai, uint8_t ao, const char* devName, uint32_t serial)
{
    EtherCAT::slave::CiA401SlaveConfig c;
    c.identity.vendorId      = inf.vendorId;
    c.identity.productCode   = inf.productCode;
    c.identity.revisionNumber = beckhoff_ids::kRevision;
    c.identity.serialNumber  = serial;
    c.identity.deviceName    = devName;
    c.digitalInputs8   = di8;
    c.digitalOutputs8  = do8;
    c.analogInputs     = ai;
    c.analogOutputs    = ao;
    c.supportsDC       = true;
    c.defaultCycleTime = 1000000;  // 1 ms
    return c;
}

} // namespace detail

/**
 * @brief The emulated bus: two EK1100 couplers with four I/O modules
 *        behind each, chained in EtherCAT order.
 *
 * Usage:
 * @code
 *   BeckhoffFleet fleet;
 *   fleet.start();
 *   // per received frame:
 *   auto resp = fleet.processFrame(frame, len);
 *   // periodically (e.g. 1 kHz):
 *   fleet.tick(1'000'000);
 * @endcode
 */
class BeckhoffFleet {
public:
    /// Default two-coupler fleet (fleetInfos()).
    BeckhoffFleet() : BeckhoffFleet(fleetInfos()) {}

    /// Custom topology — e.g. detail::fleetInfosFromText/File output
    /// with a bunch of EL1014s behind one coupler.
    explicit BeckhoffFleet(std::vector<FleetSlaveInfo> infos)
        : infos_(std::move(infos)) {
        buildTopology();
        for (auto& s : slaves_) s->start();
        wireMailboxAndPdo();
    }

    ~BeckhoffFleet() { stop(); }

    BeckhoffFleet(const BeckhoffFleet&) = delete;
    BeckhoffFleet& operator=(const BeckhoffFleet&) = delete;

    /// Number of EtherCAT nodes (always 10 for the default topology).
    size_t slaveCount() const { return slaves_.size(); }

    /// Node at chain position @p i (0 = first slave after the master).
    EtherCAT::slave::CiA401Slave& slave(size_t i) { return *slaves_.at(i); }
    const EtherCAT::slave::CiA401Slave& slave(size_t i) const { return *slaves_.at(i); }

    const FleetSlaveInfo& info(size_t i) const { return infos_.at(i); }
    const std::vector<FleetSlaveInfo>& infos() const { return infos_; }

    /// Run a received Ethernet frame through every slave in chain order.
    /// Returns the response frame (empty for malformed input).
    std::vector<uint8_t> processFrame(const uint8_t* frame, size_t len) {
        std::vector<uint8_t> f(frame, frame + len);
        for (auto& s : slaves_) {
            f = s->getCore().processFrame(f.data(), f.size());
            if (f.empty()) return {};
        }
        return f;
    }

    /// Advance every slave: core simulate (mailbox/DC/watchdog), profile
    /// simulate, and PDO buffer sync.
    void tick(uint64_t deltaNs) {
        for (size_t i = 0; i < slaves_.size(); ++i) {
            slaves_[i]->simulate(deltaNs);
            slaves_[i]->getCore().simulate(deltaNs);
            slaves_[i]->updateTxPDO();
            slaves_[i]->processRxPDO();
        }
    }

    void start() {
        for (auto& s : slaves_) s->getCore().start();
    }

    void stop() {
        for (auto& s : slaves_) s->getCore().stop();
    }

private:
    void buildTopology() {
        for (const auto& inf : infos_) {
            const uint8_t di8 = static_cast<uint8_t>(inf.diSize);
            const uint8_t do8 = static_cast<uint8_t>(inf.doSize);
            const uint8_t ai  = static_cast<uint8_t>(inf.aiSize / 2);
            const uint8_t ao  = static_cast<uint8_t>(inf.aoSize / 2);
            const std::string dev = inf.name.substr(0, inf.name.find(" ("));
            auto cfg = detail::moduleConfig(inf, di8, do8, ai, ao,
                                             dev.c_str(),
                                             static_cast<uint32_t>(slaves_.size() + 1));
            slaves_.push_back(
                std::make_unique<EtherCAT::slave::CiA401Slave>(cfg));
        }
    }

    void wireMailboxAndPdo() {
        for (auto& s : slaves_) {
            auto& core = s->getCore();
            // CoE mailbox: share the profile's object dictionary with the
            // core through a non-owning alias (the profile owns it).
            auto& od = s->getObjectDictionary();
            std::shared_ptr<EtherCAT::slave::IObjectDictionary> odsp(
                &od, [](EtherCAT::slave::IObjectDictionary*) {});
            core.setObjectDictionary(odsp);
            core.addMailboxHandler(EtherCAT::slave::createCoEHandler(odsp));
            // Immediate PDO refresh on logical (LRD/LWR/LRW) exchanges;
            // tick() covers position/configured-address exchanges.
            auto* raw = s.get();
            core.setPDOExchangeCallback([raw]() {
                raw->updateTxPDO();
                raw->processRxPDO();
            });
        }
    }

    std::vector<std::unique_ptr<EtherCAT::slave::CiA401Slave>> slaves_;
    std::vector<FleetSlaveInfo> infos_;
};

} // namespace tether::examples
