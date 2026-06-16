/**
 * @file PCAPLogger.hpp
 * @brief HAL-facing PCAP packet logger adapter
 *
 * Implements the abstract Tether::PacketLoggers::PacketLogger interface using
 * the Tether::PacketLoggers::PCAP::PCAPWriter engine. This is the "drop in"
 * PCAP implementation for the HAL.
 */

#pragma once

#include "packetloggers/PacketLogger.hpp"
#include "packetloggers/pcap/PCAPLoggerConfig.hpp"

#include <cstdint>
#include <cstddef>
#include <memory>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

// Forward declaration of the internal PCAPNG writer.
class IPCAPWriter;

/**
 * @brief PCAP-backed implementation of the PacketLogger interface
 */
class PCAPLogger : public PacketLogger {
public:
    PCAPLogger();
    ~PCAPLogger() override;

    // PacketLogger interface
    Error init(const PCAPLoggerConfig& config) override;
    void close() override;
    bool isOpen() const override;
    Error logFrame(const uint8_t* frame, size_t length,
                   FrameDirection direction,
                   uint64_t timestamp = 0) override;
    void flush() override;
    uint64_t getFrameCount() const override;
    size_t getFileSize() const override;
    Stats getStats() const override;

private:
    std::unique_ptr<IPCAPWriter> writer_;
    PCAPLoggerConfig config_;
    uint64_t frameCount_ = 0;
    uint64_t totalBytes_ = 0;
    uint64_t droppedFrames_ = 0;
};

/**
 * @brief Create a PCAP-backed PacketLogger
 */
std::shared_ptr<PacketLogger> createPCAPLogger();

/**
 * @brief Create a PCAP-backed PacketLogger with the specified configuration
 */
std::shared_ptr<PacketLogger> createPCAPLogger(const PCAPLoggerConfig& config);

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
