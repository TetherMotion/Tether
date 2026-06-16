/**
 * @file fifo_communication.cpp
 * @brief Example: Master-slave communication via POSIX FIFOs
 * 
 * This example demonstrates using FIFOs for inter-process communication
 * between an EtherCAT master and slave running as separate processes.
 * 
 * Run two instances:
 *   ./fifo_communication --master
 *   ./fifo_communication --slave
 */

#include "slave/SlaveCore.hpp"
#include "slave/hal/LoopbackHAL.hpp"
#include "slave/profiles/CiA401Slave.hpp"
#include "pcap/PcapLogger.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/select.h>

std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running.store(false);
}

// EtherCAT frame utilities
class SimpleFrameBuilder {
public:
    static std::vector<uint8_t> buildAPRD(uint16_t position, uint16_t offset, uint16_t length) {
        std::vector<uint8_t> frame;
        
        // Ethernet header (14 bytes)
        frame.resize(14);
        std::memset(frame.data(), 0xFF, 6);  // Dest (broadcast)
        std::memset(frame.data() + 6, 0x00, 6);  // Source
        frame[12] = 0x88; frame[13] = 0xA4;  // EtherCAT type
        
        // EtherCAT header (2 bytes)
        uint16_t ecatLen = 10 + length + 2;  // datagram header + data + wkc
        frame.push_back(ecatLen & 0xFF);
        frame.push_back(((ecatLen >> 8) & 0x07) | 0x10);  // Type 1
        
        // Datagram header (10 bytes)
        frame.push_back(0x01);  // APRD command
        frame.push_back(0x00);  // Index
        frame.push_back(position & 0xFF);
        frame.push_back((position >> 8) & 0xFF);
        frame.push_back(offset & 0xFF);
        frame.push_back((offset >> 8) & 0xFF);
        frame.push_back(length & 0xFF);
        frame.push_back((length >> 8) & 0x07);  // No more datagrams
        frame.push_back(0x00); frame.push_back(0x00);  // IRQ
        
        // Data (zeros)
        for (uint16_t i = 0; i < length; i++) {
            frame.push_back(0);
        }
        
        // Working counter
        frame.push_back(0); frame.push_back(0);
        
        return frame;
    }
    
    static std::vector<uint8_t> buildAPWR(uint16_t position, uint16_t offset, 
                                          const std::vector<uint8_t>& data) {
        std::vector<uint8_t> frame;
        
        // Ethernet header
        frame.resize(14);
        std::memset(frame.data(), 0xFF, 6);
        std::memset(frame.data() + 6, 0x00, 6);
        frame[12] = 0x88; frame[13] = 0xA4;
        
        // EtherCAT header
        uint16_t ecatLen = 10 + data.size() + 2;
        frame.push_back(ecatLen & 0xFF);
        frame.push_back(((ecatLen >> 8) & 0x07) | 0x10);
        
        // Datagram header
        frame.push_back(0x02);  // APWR command
        frame.push_back(0x00);
        frame.push_back(position & 0xFF);
        frame.push_back((position >> 8) & 0xFF);
        frame.push_back(offset & 0xFF);
        frame.push_back((offset >> 8) & 0xFF);
        frame.push_back(data.size() & 0xFF);
        frame.push_back((data.size() >> 8) & 0x07);
        frame.push_back(0x00); frame.push_back(0x00);
        
        // Data
        frame.insert(frame.end(), data.begin(), data.end());
        
        // Working counter
        frame.push_back(0); frame.push_back(0);
        
        return frame;
    }
};

void runMaster(const std::string& m2sPath, const std::string& s2mPath) {
    std::cout << "=== Running as MASTER ===\n";
    std::cout << "Master->Slave FIFO: " << m2sPath << "\n";
    std::cout << "Slave->Master FIFO: " << s2mPath << "\n\n";
    
    // Create FIFOs
    mkfifo(m2sPath.c_str(), 0666);
    mkfifo(s2mPath.c_str(), 0666);
    
    std::cout << "Waiting for slave to connect...\n";
    
    // Open FIFOs (order matters for non-blocking)
    int fdWrite = open(m2sPath.c_str(), O_WRONLY);
    int fdRead = open(s2mPath.c_str(), O_RDONLY);
    
    if (fdWrite < 0 || fdRead < 0) {
        std::cerr << "Failed to open FIFOs\n";
        return;
    }
    
    std::cout << "Connected! Starting communication...\n\n";
    
    int frameCount = 0;
    
    while (g_running.load()) {
        // Build and send frame
        auto frame = SimpleFrameBuilder::buildAPRD(0, 0x0130, 2);  // Read AL Status
        
        ssize_t written = write(fdWrite, frame.data(), frame.size());
        if (written < 0) {
            std::cerr << "Write error\n";
            break;
        }
        
        // Read response with timeout
        std::vector<uint8_t> response(1500);
        
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fdRead, &readSet);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms
        
        if (select(fdRead + 1, &readSet, nullptr, nullptr, &timeout) > 0) {
            ssize_t bytesRead = read(fdRead, response.data(), response.size());
            if (bytesRead > 0) {
                response.resize(bytesRead);
                
                // Parse response
                if (response.size() >= 26) {  // Minimum frame size
                    uint16_t wkc = response[24] | (response[25] << 8);
                    uint16_t alStatus = response[22] | (response[23] << 8);
                    
                    std::cout << "\rFrame " << ++frameCount 
                              << " | AL Status: 0x" << std::hex << alStatus
                              << " | WKC: " << std::dec << wkc
                              << "     " << std::flush;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "\n\nClosing FIFOs...\n";
    close(fdWrite);
    close(fdRead);
}

void runSlave(const std::string& m2sPath, const std::string& s2mPath) {
    std::cout << "=== Running as SLAVE ===\n";
    std::cout << "Master->Slave FIFO: " << m2sPath << "\n";
    std::cout << "Slave->Master FIFO: " << s2mPath << "\n\n";
    
    // Create slave
    auto slave = std::make_unique<EtherCAT::Slave::CiA401Slave>(1);
    slave->setConfiguredAddress(0x1001);
    
    std::cout << "Waiting for master to connect...\n";
    
    // Open FIFOs (reverse order from master)
    int fdRead = open(m2sPath.c_str(), O_RDONLY);
    int fdWrite = open(s2mPath.c_str(), O_WRONLY);
    
    if (fdRead < 0 || fdWrite < 0) {
        std::cerr << "Failed to open FIFOs\n";
        return;
    }
    
    std::cout << "Connected! Processing frames...\n\n";
    
    int frameCount = 0;
    
    while (g_running.load()) {
        // Read frame with timeout
        std::vector<uint8_t> frame(1500);
        
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fdRead, &readSet);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        
        if (select(fdRead + 1, &readSet, nullptr, nullptr, &timeout) > 0) {
            ssize_t bytesRead = read(fdRead, frame.data(), frame.size());
            if (bytesRead > 0) {
                frame.resize(bytesRead);
                
                // Process frame through slave
                auto response = slave->processFrame(frame.data(), frame.size());
                
                if (!response.empty()) {
                    ssize_t written = write(fdWrite, response.data(), response.size());
                    if (written < 0) {
                        std::cerr << "Write error\n";
                        break;
                    }
                    
                    frameCount++;
                    
                    auto state = slave->getState();
                    std::string stateStr;
                    switch (state) {
                        case EtherCAT::Slave::SlaveState::Init: stateStr = "Init"; break;
                        case EtherCAT::Slave::SlaveState::PreOp: stateStr = "Pre-Op"; break;
                        case EtherCAT::Slave::SlaveState::SafeOp: stateStr = "Safe-Op"; break;
                        case EtherCAT::Slave::SlaveState::Op: stateStr = "Op"; break;
                        default: stateStr = "Unknown"; break;
                    }
                    
                    std::cout << "\rProcessed " << frameCount 
                              << " frames | State: " << stateStr
                              << "     " << std::flush;
                }
            }
        }
        
        // Update slave
        slave->update();
    }
    
    std::cout << "\n\nClosing FIFOs...\n";
    close(fdRead);
    close(fdWrite);
}

void printUsage(const char* progName) {
    std::cout << "Usage: " << progName << " [--master|--slave] [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --master         Run as EtherCAT master\n";
    std::cout << "  --slave          Run as EtherCAT slave\n";
    std::cout << "  --m2s <path>     Master-to-slave FIFO path (default: /tmp/ecat_m2s)\n";
    std::cout << "  --s2m <path>     Slave-to-master FIFO path (default: /tmp/ecat_s2m)\n";
    std::cout << "\nExample:\n";
    std::cout << "  Terminal 1: " << progName << " --slave\n";
    std::cout << "  Terminal 2: " << progName << " --master\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    bool isMaster = false;
    bool isSlave = false;
    std::string m2sPath = "/tmp/ecat_m2s";
    std::string s2mPath = "/tmp/ecat_s2m";
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--master") {
            isMaster = true;
        } else if (arg == "--slave") {
            isSlave = true;
        } else if (arg == "--m2s" && i + 1 < argc) {
            m2sPath = argv[++i];
        } else if (arg == "--s2m" && i + 1 < argc) {
            s2mPath = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }
    
    if (isMaster && isSlave) {
        std::cerr << "Error: Cannot be both master and slave\n";
        return 1;
    }
    
    if (!isMaster && !isSlave) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::cout << "EtherCAT FIFO Communication Example\n";
    std::cout << "====================================\n\n";
    
    if (isMaster) {
        runMaster(m2sPath, s2mPath);
    } else {
        runSlave(m2sPath, s2mPath);
    }
    
    return 0;
}
