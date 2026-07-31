/**
 * @file CanTransport.hpp
 * @brief Byte-stream transport tunneled over CAN frames.
 *
 * @details
 * CAN bus frames carry up to 8 bytes of payload (CAN 2.0A). To tunnel a
 * byte stream over CAN, this transport segments writes into 8-byte chunks
 * and reassembles them on the receive side. A simple framing scheme is used:
 *
 *   - Each CAN frame carries up to 8 bytes of raw stream data.
 *   - The CAN ID is fixed (configurable) so both ends agree on the stream
 *     channel. Two IDs are used: one for host->device, one for device->host.
 *   - No additional length header is needed because the Klipper message-block
 *     framing (sync byte + CRC) is self-delimiting on top of this byte stream.
 *
 * This transport requires a HAL `ICan` implementation (e.g. LinuxCan on Linux
 * via SocketCAN). It is only compiled when TETHER_ENABLE_KLIPPER_CAN is ON.
 */

#pragma once

#include "tether/klipper/transport/IByteStreamTransport.hpp"
#include "tether/hal/ICan.hpp"

#include <cstdint>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <memory>

namespace tether::klipper::transport {

/**
 * @brief Configuration for a CAN-based byte-stream transport.
 */
struct CanTransportConfig {
    /// HAL CAN interface to use (must be open).
    std::shared_ptr<tether::hal::ICan> can;
    /// CAN ID used for transmitting (this end -> peer).
    uint32_t txCanId = 0x100;
    /// CAN ID used for receiving (peer -> this end). Frames with other IDs
    /// are ignored. If 0, accepts all IDs.
    uint32_t rxCanId = 0x101;
    /// Poll interval in microseconds when blocking-read is requested.
    int pollIntervalUs = 1000;
};

/**
 * @brief Byte-stream transport over CAN frames.
 */
class CanTransport : public IByteStreamTransport {
public:
    CanTransport() = default;
    explicit CanTransport(CanTransportConfig config) : config_(std::move(config)) {}
    ~CanTransport() override;

    bool open() override;
    bool isOpen() const override;
    void close() override;

    size_t write(std::span<const uint8_t> data) override;
    size_t available() const override;
    size_t read(uint8_t* out, size_t maxLen, bool canBlock = false) override;

private:
    void pumpRx();

    CanTransportConfig config_;
    std::mutex rxMtx_;
    std::deque<uint8_t> rxBuf_;
    std::atomic<bool> open_{false};
};

} // namespace tether::klipper::transport
