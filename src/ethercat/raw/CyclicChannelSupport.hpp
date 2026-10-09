/**
 * @file CyclicChannelSupport.hpp
 * @brief Shared Linux cyclic-channel helpers: cyclic socket open, monotonic
 *        clocks, and poll.
 *
 * @internal Internal header — not installed, not part of the public API.
 * The LinuxSocketChannel / LinuxRingChannel class bodies live in their own
 * headers; this module holds the free helpers they and the factory share.
 */

#pragma once

#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/Types.hpp"

#include <cstddef>
#include <cstdint>

#if defined(__linux__)

namespace EtherCAT {

/// Open + configure a non-blocking AF_PACKET socket on `ifindex`, attach the
/// cyclic demux program (or a caller-composed one), and register it with the
/// NIC error-counter monitor.  Returns -1 on failure.
int openCyclicSocket(int ifindex,
                     const CBPFInsn* accept_prog = nullptr,
                     size_t accept_prog_len = 0);

/// CLOCK_MONOTONIC now, in nanoseconds.
uint64_t monoNowNs();

/// Convert a CLOCK_REALTIME kernel RX stamp to the CLOCK_MONOTONIC domain.
uint64_t rtStampToMonoNs(uint64_t rt_ns);

/// ppoll a single fd for POLLIN.  >0 ready, 0 timeout, <0 error.
int waitReadable(int fd, uint32_t timeout_ns);

} // namespace EtherCAT

#endif // __linux__

