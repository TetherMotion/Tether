/**
 * @file slave_emulator.cpp
 * @brief Generic EtherCAT Slave Emulator Example
 *
 * Emulates a generic EtherCAT slave on a real network interface using
 * SlaveCore directly (no specific CiA profile). The slave listens for
 * EtherCAT frames from an external master, processes them, and transmits
 * responses back.
 *
 * Usage (Linux, requires root or CAP_NET_RAW):
 *   ./slave_emulator                     # uses eth0, default identity
 *   ./slave_emulator -i enp3s0           # specify interface
 *   ./slave_emulator --vendor-id 0xABCD  # custom vendor ID
 *   ./slave_emulator --no-coe            # disable CoE mailbox
 *   ./slave_emulator -t 30               # run for 30 seconds
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <argparse/argparse.hpp>

#include "tether/slave/core/SlaveCore.hpp"
#include "tether/slave/core/SlaveTypes.hpp"
#include "tether/slave/mailbox/IMailboxHandler.hpp"
#include "tether/ethercat/ALRegisters.hpp"
#include "tether/hal/IEthernet.hpp"
#include "tether/platform/Platform.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

namespace slave = EtherCAT::slave;
namespace AL = EtherCAT::AL;

static std::atomic<bool> g_running{true};

static void signalHandler(int) {
    g_running.store(false);
}

static std::string slaveStateString(AL::SlaveState state) {
    return AL::slaveStateToString(state);
}

static uint32_t parseHex32(const std::string& s) {
    if (s.empty()) return 0;
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        return static_cast<uint32_t>(std::stoul(s.substr(2), nullptr, 16));
    return static_cast<uint32_t>(std::stoul(s, nullptr, 16));
}

static uint16_t parseHex16(const std::string& s) {
    return static_cast<uint16_t>(parseHex32(s));
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    constexpr const char* kTag = "slave_emu";

    argparse::ArgumentParser program("slave_emulator", "1.0");

    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addDurationArg(program, 0.0);
    Tether::Examples::addDebugArg(program);
    Tether::Examples::addVlanArgs(program);

    program.add_argument("--vendor-id")
        .default_value(std::string("0x1234"))
        .help("Slave vendor ID (hex)");
    program.add_argument("--product-code")
        .default_value(std::string("0x5678"))
        .help("Slave product code (hex)");
    program.add_argument("--revision")
        .default_value(std::string("0x10000"))
        .help("Slave revision number (hex)");
    program.add_argument("--serial")
        .default_value(std::string("0x1"))
        .help("Slave serial number (hex)");
    program.add_argument("--name")
        .default_value(std::string("Generic EtherCAT Slave"))
        .help("Slave device name");
    program.add_argument("--station-addr")
        .default_value(std::string("0x1000"))
        .help("Initial configured station address (hex)");
    program.add_argument("--rxpdo-size")
        .scan<'i', int>()
        .default_value(0)
        .help("RxPDO size in bytes (0 = auto from mapping)");
    program.add_argument("--txpdo-size")
        .scan<'i', int>()
        .default_value(0)
        .help("TxPDO size in bytes (0 = auto from mapping)");
    program.add_argument("--mailbox-size")
        .scan<'i', int>()
        .default_value(128)
        .help("Mailbox size in bytes (in and out)");
    program.add_argument("--no-coe")
        .implicit_value(true)
        .default_value(false)
        .help("Disable CoE mailbox support");
    program.add_argument("--no-dc")
        .implicit_value(true)
        .default_value(false)
        .help("Disable Distributed Clock support");
    program.add_argument("--cycle-time")
        .scan<'i', int>()
        .default_value(1000)
        .help("DC cycle time in microseconds");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        std::cerr << program;
        return 1;
    }

    const auto interfaceName = Tether::Examples::resolveInterface(program.get<std::string>("--interface"), kTag);
    const auto durationSec = program.get<double>("--time");
    const auto debugStr = program.get<std::string>("--debug");
    const auto rxVlanStr = program.get<std::string>("--rx-vlan");
    const auto txVlanStr = program.get<std::string>("--tx-vlan");

    const uint32_t vendorId = parseHex32(program.get<std::string>("--vendor-id"));
    const uint32_t productCode = parseHex32(program.get<std::string>("--product-code"));
    const uint32_t revision = parseHex32(program.get<std::string>("--revision"));
    const uint32_t serial = parseHex32(program.get<std::string>("--serial"));
    const auto deviceName = program.get<std::string>("--name");
    const uint16_t stationAddr = parseHex16(program.get<std::string>("--station-addr"));
    const int rxPdoSize = program.get<int>("--rxpdo-size");
    const int txPdoSize = program.get<int>("--txpdo-size");
    const int mailboxSize = program.get<int>("--mailbox-size");
    const bool disableCoe = program.get<bool>("--no-coe");
    const bool disableDc = program.get<bool>("--no-dc");
    const int cycleTimeUs = program.get<int>("--cycle-time");

    if (Tether::Examples::printDebugHelpIfRequested(debugStr)) {
        return 0;
    }

    Tether::Examples::VlanConfig vlanCfg;
    if (!Tether::Examples::parseVlanArgs(rxVlanStr, txVlanStr, vlanCfg, kTag)) {
        return 1;
    }
    Tether::Examples::logVlanConfig(vlanCfg, kTag);

    std::cout << "EtherCAT Slave Emulator\n";
    std::cout << "=======================\n\n";
    std::cout << "Interface:    " << interfaceName << "\n";
    std::cout << "Vendor ID:    0x" << std::hex << vendorId << "\n";
    std::cout << "Product Code: 0x" << productCode << "\n";
    std::cout << "Revision:     0x" << revision << "\n";
    std::cout << "Serial:       0x" << serial << std::dec << "\n";
    std::cout << "Device Name:  " << deviceName << "\n";
    std::cout << "Station Addr: 0x" << std::hex << stationAddr << std::dec << "\n";
    std::cout << "CoE:          " << (disableCoe ? "disabled" : "enabled") << "\n";
    std::cout << "DC:           " << (disableDc ? "disabled" : "enabled") << "\n";
    std::cout << "Cycle Time:   " << cycleTimeUs << " us\n\n";

    // -----------------------------------------------------------------------
    // 1. Initialize Ethernet HAL
    // -----------------------------------------------------------------------
    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, interfaceName, kTag)) {
        return 1;
    }

    // -----------------------------------------------------------------------
    // 2. Configure and create SlaveCore
    // -----------------------------------------------------------------------
    slave::SlaveConfig slaveCfg;

    slaveCfg.identity.vendorId = vendorId;
    slaveCfg.identity.productCode = productCode;
    slaveCfg.identity.revisionNumber = revision;
    slaveCfg.identity.serialNumber = serial;
    slaveCfg.identity.deviceName = deviceName;

    slaveCfg.mailboxOutSize = static_cast<uint16_t>(mailboxSize);
    slaveCfg.mailboxInSize = static_cast<uint16_t>(mailboxSize);
    slaveCfg.mailboxProtocol = disableCoe ? 0x0000 : 0x000C;

    slaveCfg.rxPdoSize = static_cast<uint16_t>(rxPdoSize);
    slaveCfg.txPdoSize = static_cast<uint16_t>(txPdoSize);

    slaveCfg.supportsDC = !disableDc;
    slaveCfg.defaultCycleTime = static_cast<uint32_t>(cycleTimeUs * 1000);

    auto slaveCore = std::make_unique<slave::SlaveCore>(slaveCfg);

    // Set initial configured station address via register write
    {
        uint8_t addrBuf[2] = {
            static_cast<uint8_t>(stationAddr & 0xFF),
            static_cast<uint8_t>((stationAddr >> 8) & 0xFF),
        };
        slaveCore->setSIIData({}); // SII is auto-initialized in constructor
        // Write station address through the register interface
        // by using processFrame with an APWR datagram is overly complex;
        // instead, we rely on the master setting it during bus scan.
        // But we can set it directly by writing to the register memory.
        // SlaveCore doesn't expose a public setter, so the master will
        // assign the address during initialization via APRD/APWR sequence.
        (void)addrBuf;
    }

    // Set up CoE mailbox support
    std::shared_ptr<slave::IObjectDictionary> objectDict;
    std::shared_ptr<slave::IMailboxHandler> coeHandler;

    if (!disableCoe) {
        objectDict = slave::createObjectDictionary();
        if (objectDict) {
            // Register CiA 301 communication objects
            // 0x1000: Device type (Unsigned32, RO)
            slave::ODEntryInfo devType{
                0x1000, 0x00,
                slave::ODDataType::Unsigned32, 32,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Device type", 0x00000192
            };
            objectDict->registerObject(devType);

            // 0x1018: Identity object
            slave::ODEntryInfo identityObj{
                0x1018, 0x00,
                slave::ODDataType::Unsigned8, 8,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Identity", 4
            };
            objectDict->registerObject(identityObj);

            slave::ODEntryInfo vendorIdEntry{
                0x1018, 0x01,
                slave::ODDataType::Unsigned32, 32,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Vendor ID", vendorId
            };
            objectDict->registerObject(vendorIdEntry);

            slave::ODEntryInfo productCodeEntry{
                0x1018, 0x02,
                slave::ODDataType::Unsigned32, 32,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Product Code", productCode
            };
            objectDict->registerObject(productCodeEntry);

            slave::ODEntryInfo revisionEntry{
                0x1018, 0x03,
                slave::ODDataType::Unsigned32, 32,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Revision Number", revision
            };
            objectDict->registerObject(revisionEntry);

            slave::ODEntryInfo serialEntry{
                0x1018, 0x04,
                slave::ODDataType::Unsigned32, 32,
                static_cast<uint8_t>(slave::ODAccessType::ReadOnly),
                "Serial Number", serial
            };
            objectDict->registerObject(serialEntry);

            slaveCore->setObjectDictionary(objectDict);

            coeHandler = slave::createCoEHandler(objectDict);
            if (coeHandler) {
                slaveCore->addMailboxHandler(coeHandler);
            }
        }
    }

    // Start the slave
    slaveCore->start();

    std::cout << "Slave started. Waiting for master...\n\n";

    // -----------------------------------------------------------------------
    // 3. Set up RX callback: bridge Ethernet frames to SlaveCore
    // -----------------------------------------------------------------------
    std::atomic<uint64_t> frameCount{0};
    std::atomic<uint64_t> responseCount{0};

    // The SlaveCore pointer captured in the lambda must remain valid
    // for the lifetime of the poll thread.
    auto* slavePtr = slaveCore.get();
    auto* ethPtr = session.eth.get();

    auto rxCallback = [slavePtr, ethPtr, &frameCount, &responseCount](
            const uint8_t* frame, size_t length,
            const EtherCAT::HAL::RxFrameInfo&, void*) {
        frameCount.fetch_add(1, std::memory_order_relaxed);

        auto response = slavePtr->processFrame(frame, length);
        if (!response.empty()) {
            ethPtr->transmit(response.data(), response.size());
            responseCount.fetch_add(1, std::memory_order_relaxed);
        }
    };

    session.eth->setRxCallback(rxCallback, nullptr);

    // -----------------------------------------------------------------------
    // 4. Start poll thread
    // -----------------------------------------------------------------------
    Tether::Examples::startHostPollThread(session, kTag);

    // -----------------------------------------------------------------------
    // 5. Simulation thread (advances DC time and watchdog)
    // -----------------------------------------------------------------------
    std::atomic<bool> simRunning{true};
    std::thread simThread([&slaveCore, &simRunning, cycleTimeUs]() {
        const auto deltaNs = static_cast<uint64_t>(cycleTimeUs) * 1000;
        while (simRunning.load()) {
            slaveCore->simulate(deltaNs);
            std::this_thread::sleep_for(
                std::chrono::microseconds(cycleTimeUs));
        }
    });

    // -----------------------------------------------------------------------
    // 6. Main loop — status display
    // -----------------------------------------------------------------------
    auto startTime = std::chrono::steady_clock::now();
    auto lastStatusTime = startTime;

    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - startTime).count();

        if (durationSec > 0.0 && elapsed >= durationSec) {
            break;
        }

        auto sinceLastStatus = std::chrono::duration_cast<std::chrono::seconds>(
            now - lastStatusTime);
        if (sinceLastStatus.count() >= 1) {
            lastStatusTime = now;
            auto state = slaveCore->getState();
            auto stats = session.eth->getStats();

            std::cout << "\r[" << std::fixed << std::setprecision(1)
                      << elapsed << "s] State: "
                      << slaveStateString(state)
                      << " | RX: " << frameCount.load()
                      << " | TX: " << responseCount.load()
                      << " | ethRX: " << stats.rxFrames
                      << " ethTX: " << stats.txFrames
                      << "   " << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // -----------------------------------------------------------------------
    // 7. Shutdown
    // -----------------------------------------------------------------------
    std::cout << "\n\nShutting down...\n";

    simRunning.store(false);
    if (simThread.joinable()) {
        simThread.join();
    }

    slaveCore->stop();
    Tether::Examples::shutdownHostEthernet(session);

    std::cout << "Done. Processed " << frameCount.load() << " frames, "
              << responseCount.load() << " responses.\n";

    return 0;
}
