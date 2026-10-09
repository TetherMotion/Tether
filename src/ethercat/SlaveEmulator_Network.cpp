/**
 * @file SlaveEmulator_Network.cpp
 * @brief NetworkEmulator — frame/datagram dispatch across emulated slaves.
 *
 * TU split out of SlaveEmulator.cpp.
 */

#include "SlaveEmulator.hpp"
#include "tether/ethercat/DCClass.hpp"
#include "tether/platform/EspCompat.hpp"

#include <bit>

#define LOG_TAG "EC_EMU"
#define LOGI(fmt, ...) TETHER_LOGI(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) TETHER_LOGW(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) TETHER_LOGE(LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) TETHER_LOGD(LOG_TAG, fmt, ##__VA_ARGS__)

namespace EtherCAT {
namespace Emulator {

// ============================================================================
// NetworkEmulator Implementation
// ============================================================================

NetworkEmulator::NetworkEmulator() {
}

NetworkEmulator::~NetworkEmulator() = default;

void NetworkEmulator::addSlave(std::unique_ptr<SlaveEmulator> slave) {
    std::lock_guard<std::mutex> lock(mutex_);
    slave->setPosition(slaves_.size());
    slaves_.push_back(std::move(slave));
}

SlaveEmulator* NetworkEmulator::getSlave(size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < slaves_.size()) {
        return slaves_[index].get();
    }
    return nullptr;
}

void NetworkEmulator::clearSlaves() {
    std::lock_guard<std::mutex> lock(mutex_);
    slaves_.clear();
}

std::vector<uint8_t> NetworkEmulator::processFrame(const uint8_t* frame_data, size_t frame_len) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (frame_len < 14 + 2) {  // Ethernet header + EtherCAT header
        return {};
    }
    
    // Check EtherType (0x88A4 for EtherCAT)
    uint16_t ethertype = (frame_data[12] << 8) | frame_data[13];
    if (ethertype != 0x88A4) {
        return {};
    }
    
    stats_.frames_processed++;
    
    // Build response frame (copy header)
    std::vector<uint8_t> response(frame_data, frame_data + frame_len);
    
    // Swap source/dest MAC for response
    std::swap_ranges(response.begin(), response.begin() + 6, response.begin() + 6);
    
    // EtherCAT header at offset 14
    uint16_t ecat_len = (frame_data[14]) | ((frame_data[15] & 0x07) << 8);
    (void)ecat_len;  // Length of datagram area
    
    // Parse datagrams starting at offset 16
    size_t offset = 16;
    while (offset + 10 <= frame_len) {  // Minimum datagram header size
        uint8_t cmd = frame_data[offset];
        uint8_t idx = frame_data[offset + 1];
        uint16_t adp = frame_data[offset + 2] | (frame_data[offset + 3] << 8);
        uint16_t ado = frame_data[offset + 4] | (frame_data[offset + 5] << 8);
        uint16_t len_flags = frame_data[offset + 6] | (frame_data[offset + 7] << 8);
        uint16_t datalen = len_flags & 0x07FF;
        bool more = (len_flags & 0x8000) != 0;
        
        if (offset + 10 + datalen + 2 > frame_len) {
            break;  // Invalid frame
        }
        
        // Get pointer to data in response
        uint8_t* data_ptr = &response[offset + 10];
        
        // Process datagram and get WKC
        uint16_t wkc = processDatagram(
            static_cast<Command>(cmd), idx, adp, ado, data_ptr, datalen);
        
        // Write WKC to response (after data)
        response[offset + 10 + datalen] = wkc & 0xFF;
        response[offset + 10 + datalen + 1] = (wkc >> 8) & 0xFF;
        
        stats_.datagrams_processed++;
        
        if (!more) break;
        offset += 10 + datalen + 2;
    }
    
    return response;
}

uint16_t NetworkEmulator::processDatagram(Command cmd, uint8_t idx,
                                           uint16_t adp, uint16_t ado,
                                           uint8_t* data, uint16_t datalen)
{
    (void)idx;
    
    switch (cmd) {
        case Command::APRD:
        case Command::APWR:
        case Command::APRW:
            return processAutoIncrement(cmd, static_cast<int16_t>(adp), ado, data, datalen);
            
        case Command::FPRD:
        case Command::FPWR:
        case Command::FPRW:
            return processConfiguredAddr(cmd, adp, ado, data, datalen);
            
        case Command::BRD:
        case Command::BWR:
        case Command::BRW:
            return processBroadcast(cmd, ado, data, datalen);
            
        case Command::LRD:
        case Command::LWR:
        case Command::LRW:
        {
            uint32_t logical_addr = (static_cast<uint32_t>(adp) << 16) | ado;
            return processLogical(cmd, logical_addr, data, datalen);
        }
            
        default:
            stats_.unknown_commands++;
            return 0;
    }
}

uint16_t NetworkEmulator::processAutoIncrement(Command cmd, int16_t adp, uint16_t ado,
                                                uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    // ADP starts negative, increments as frame passes through each slave
    // Slave processes when ADP == 0
    int16_t current_adp = adp;
    
    for (auto& slave : slaves_) {
        if (current_adp == 0) {
            // This slave processes the datagram
            bool ok = false;
            switch (cmd) {
                case Command::APRD:
                    ok = slave->processAPRD(ado, data, len);
                    break;
                case Command::APWR:
                    ok = slave->processAPWR(ado, data, len);
                    break;
                case Command::APRW:
                    // Read first, then write (in practice, simultaneous)
                    ok = slave->processAPRD(ado, data, len);
                    break;
                default:
                    break;
            }
            if (ok) {
                wkc += (cmd == Command::APRW) ? 3 : 1;
            }
        }
        current_adp++;
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processConfiguredAddr(Command cmd, uint16_t addr, uint16_t ado,
                                                 uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        // Check configured station address
        if (slave->getALStatus().state != SlaveState::INIT) {
            uint8_t addr_low, addr_high;
            slave->processFPRD(0x0010, &addr_low, 1);
            slave->processFPRD(0x0011, &addr_high, 1);
            uint16_t slave_addr = addr_low | (addr_high << 8);
            
            if (slave_addr == addr) {
                bool ok = false;
                switch (cmd) {
                    case Command::FPRD:
                        ok = slave->processFPRD(ado, data, len);
                        break;
                    case Command::FPWR:
                        ok = slave->processFPWR(ado, data, len);
                        break;
                    case Command::FPRW:
                        ok = slave->processFPRD(ado, data, len);
                        break;
                    default:
                        break;
                }
                if (ok) {
                    wkc += (cmd == Command::FPRW) ? 3 : 1;
                }
                break;  // Found the slave
            }
        }
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processBroadcast(Command cmd, uint16_t ado,
                                            uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        bool ok = false;
        switch (cmd) {
            case Command::BRD:
                ok = slave->processAPRD(ado, data, len);
                // For broadcast read, data is OR'd from all slaves
                break;
            case Command::BWR:
                ok = slave->processAPWR(ado, data, len);
                break;
            case Command::BRW:
                ok = slave->processAPRD(ado, data, len);
                break;
            default:
                break;
        }
        if (ok) {
            wkc++;
        }
    }
    
    return wkc;
}

uint16_t NetworkEmulator::processLogical(Command cmd, uint32_t logical_addr,
                                          uint8_t* data, uint16_t len)
{
    uint16_t wkc = 0;
    
    for (auto& slave : slaves_) {
        bool ok = false;
        switch (cmd) {
            case Command::LRD:
                ok = slave->processLogicalRead(logical_addr, data, len);
                if (ok) wkc += 1;
                break;
            case Command::LWR:
                ok = slave->processLogicalWrite(logical_addr, data, len);
                if (ok) wkc += 1;
                break;
            case Command::LRW:
                // Read from TxPDO (inputs), write to RxPDO (outputs)
                if (slave->processLogicalRead(logical_addr, data, len)) {
                    wkc += 1;
                }
                if (slave->processLogicalWrite(logical_addr, data, len)) {
                    wkc += 2;
                }
                break;
            default:
                break;
        }
    }
    
    return wkc;
}

void NetworkEmulator::simulate(uint32_t delta_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slave : slaves_) {
        slave->simulate(delta_us);
        slave->advanceDCTime(delta_us * 1000);  // Convert to ns
    }
}

void NetworkEmulator::setGlobalErrorInjection(const ErrorInjection& errors) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& slave : slaves_) {
        slave->setErrorInjection(errors);
    }
}

}  // namespace Emulator
}  // namespace EtherCAT
