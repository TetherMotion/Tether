/**
 * @file NetworkEmulator.hpp
 * @brief NetworkEmulator — multi-slave frame/datagram dispatch facade.
 *
 * Split out of SlaveEmulator.hpp.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace EtherCAT {
namespace Emulator {

class SlaveEmulator;
struct ErrorInjection;

// ============================================================================
// Network Emulator Class
// ============================================================================

/**
 * @brief Emulates an EtherCAT network with multiple slaves
 * 
 * Processes complete EtherCAT frames and returns responses as if
 * the frame traveled through all slaves in sequence.
 * 
 * Usage:
 * @code
 * NetworkEmulator network;
 * network.addSlave(std::make_unique<SlaveEmulator>());
 * network.addSlave(std::make_unique<SlaveEmulator>());
 * 
 * // Process a frame (returns response frame)
 * auto response = network.processFrame(frame_data, frame_len);
 * @endcode
 */
class NetworkEmulator {
public:
    NetworkEmulator();
    ~NetworkEmulator();

    /**
     * @brief Add a slave to the network (takes ownership)
     * @param slave  Slave emulator appended at the next bus position.
     */
    void addSlave(std::unique_ptr<SlaveEmulator> slave);

    /**
     * @brief Get slave by position (0-based)
     * @param index  Bus position.
     * @return slave pointer (borrowed), or nullptr out of range.
     */
    SlaveEmulator* getSlave(size_t index);

    /**
     * @brief Get number of slaves
     * @return slaves on the emulated bus.
     */
    size_t getSlaveCount() const { return slaves_.size(); }

    /**Clear all slaves */
    void clearSlaves();

    /**
     * @brief Process an EtherCAT frame
     * 
     * Parses the frame, routes datagrams to appropriate slaves,
     * and builds a response frame.
     * 
     * @param frame_data Raw Ethernet frame data
     * @param frame_len Length of frame
     * @return Response frame data (or empty if no response needed)
     */
    std::vector<uint8_t> processFrame(const uint8_t* frame_data, size_t frame_len);

    /**
     * @brief Advance simulation time for all slaves
     * @param delta_us  Time step in microseconds.
     */
    void simulate(uint32_t delta_us);

    /**
     * @brief Set global error injection (affects all slaves)
     * @param errors  Error-injection config applied network-wide.
     */
    void setGlobalErrorInjection(const ErrorInjection& errors);

    /**Get statistics */
    struct NetworkStats {
        uint64_t frames_processed = 0;
        uint64_t datagrams_processed = 0;
        uint64_t wkc_errors = 0;
        uint64_t unknown_commands = 0;
    };
    NetworkStats getStats() const { return stats_; }
    void resetStats() { stats_ = {}; }

private:
    // Process individual datagrams
    uint16_t processDatagram(Command cmd, uint8_t idx,
                             uint16_t adp, uint16_t ado,
                             uint8_t* data, uint16_t datalen);

    // Helpers for specific command types
    uint16_t processAutoIncrement(Command cmd, int16_t adp, uint16_t ado,
                                   uint8_t* data, uint16_t len);
    uint16_t processConfiguredAddr(Command cmd, uint16_t addr, uint16_t ado,
                                    uint8_t* data, uint16_t len);
    uint16_t processBroadcast(Command cmd, uint16_t ado,
                               uint8_t* data, uint16_t len);
    uint16_t processLogical(Command cmd, uint32_t logical_addr,
                             uint8_t* data, uint16_t len);

    std::vector<std::unique_ptr<SlaveEmulator>> slaves_;
    NetworkStats stats_;
    std::mutex mutex_;  // Thread safety
};

} // namespace Emulator
} // namespace EtherCAT
