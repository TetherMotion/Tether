/**
 * @file simple_slave.cpp
 * @brief Example: Simple EtherCAT slave implementation
 * 
 * This example demonstrates creating a basic EtherCAT slave using
 * the DirectLoopbackHAL for testing without real hardware.
 */

#include "slave/SlaveCore.hpp"
#include "slave/hal/LoopbackHAL.hpp"
#include "slave/profiles/CiA401Slave.hpp"
#include "shared/PcapLogger.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running.store(false);
}

int main(int argc, char* argv[]) {
    std::cout << "EtherCAT Slave Example - Simple Digital I/O\n";
    std::cout << "============================================\n\n";
    
    // Set up signal handler
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Optional: Create PcapNG logger
    std::unique_ptr<EtherCAT::PcapNg::IPcapLogger> logger;
    if (argc > 1 && std::string(argv[1]) == "--pcap") {
        std::string pcapFile = "slave_trace.pcapng";
        if (argc > 2) {
            pcapFile = argv[2];
        }
        
        EtherCAT::PcapNg::PcapNgConfig config;
        config.filePath = pcapFile;
        config.interfaceName = "EtherCAT Slave";
        config.interfaceDescription = "Virtual EtherCAT slave interface";
        config.enableBuffering = true;
        
        logger = std::make_unique<EtherCAT::PcapNg::PcapNgLogger>(config);
        std::cout << "Logging to: " << pcapFile << "\n";
    } else {
        logger = std::make_unique<EtherCAT::PcapNg::NullPcapLogger>();
    }
    
    // Create CiA 401 (Digital I/O) slave
    auto slave = std::make_unique<EtherCAT::Slave::CiA401Slave>(1);
    
    // Configure slave identity
    slave->setConfiguredAddress(0x1001);
    slave->setVendorId(0x12345678);
    slave->setProductCode(0x00401001);
    slave->setRevisionNumber(0x00010000);
    slave->setSerialNumber(0x00000001);
    
    // Configure SyncManagers for PDO exchange
    // SM2: Output (Master -> Slave) - Digital outputs
    slave->configureSyncManager(2, 0x1100, 2, 
        EtherCAT::Slave::SyncManagerType::Output, true);
    
    // SM3: Input (Slave -> Master) - Digital inputs  
    slave->configureSyncManager(3, 0x1000, 2,
        EtherCAT::Slave::SyncManagerType::Input, true);
    
    // Configure FMMU mapping
    // FMMU0: Map logical 0x1000 to physical 0x1100 (outputs)
    slave->configureFMMU(0, 0x1000, 2, 0x1100, 0x0, true, true);
    
    // FMMU1: Map logical 0x1002 to physical 0x1000 (inputs)
    slave->configureFMMU(1, 0x1002, 2, 0x1000, 0x0, true, false);
    
    // Create DirectLoopback HAL for testing
    auto hal = std::make_unique<EtherCAT::Slave::DirectLoopbackHAL>(
        [&slave](const uint8_t* data, size_t len) {
            return slave->processFrame(data, len);
        },
        logger.get()
    );
    
    std::cout << "Slave created with address 0x1001\n";
    std::cout << "  Vendor ID:     0x" << std::hex << slave->getVendorId() << "\n";
    std::cout << "  Product Code:  0x" << slave->getProductCode() << "\n";
    std::cout << "  Digital Inputs:  16\n";
    std::cout << "  Digital Outputs: 16\n\n";
    
    // Simulate digital input changes
    int inputCounter = 0;
    auto lastUpdate = std::chrono::steady_clock::now();
    
    std::cout << "Running slave simulation (Ctrl+C to stop)...\n\n";
    
    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate);
        
        if (elapsed.count() >= 100) {  // Update every 100ms
            lastUpdate = now;
            inputCounter++;
            
            // Set digital inputs based on counter
            for (int i = 0; i < 8; i++) {
                bool state = (inputCounter & (1 << i)) != 0;
                slave->setDigitalInput(i, state);
            }
            
            // Get current state
            auto state = slave->getState();
            std::string stateStr;
            switch (state) {
                case EtherCAT::Slave::SlaveState::Init: stateStr = "Init"; break;
                case EtherCAT::Slave::SlaveState::PreOp: stateStr = "Pre-Op"; break;
                case EtherCAT::Slave::SlaveState::SafeOp: stateStr = "Safe-Op"; break;
                case EtherCAT::Slave::SlaveState::Op: stateStr = "Op"; break;
                default: stateStr = "Unknown"; break;
            }
            
            // Read digital outputs (from master)
            uint16_t outputs = 0;
            for (int i = 0; i < 16; i++) {
                if (slave->getDigitalOutput(i)) {
                    outputs |= (1 << i);
                }
            }
            
            // Read digital inputs (to master)
            uint16_t inputs = 0;
            for (int i = 0; i < 16; i++) {
                if (slave->getDigitalInput(i)) {
                    inputs |= (1 << i);
                }
            }
            
            std::cout << "\rState: " << stateStr 
                      << " | Inputs: 0x" << std::hex << inputs
                      << " | Outputs: 0x" << outputs
                      << std::dec << "     " << std::flush;
        }
        
        // Update slave
        slave->update();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    std::cout << "\n\nShutting down...\n";
    
    return 0;
}
