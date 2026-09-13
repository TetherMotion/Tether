/**
 * @file TerminalTypes.hpp
 * @brief Shared types for the Beckhoff terminal drivers
 *
 * Common vocabulary used by the digital terminal drivers
 * (IOutputTerminal / OutputTerminal / MultiOutputTerminal,
 * IInputTerminal / InputTerminal / MultiInputTerminal) and the analog
 * terminal drivers (IAnalogInputTerminal / AnalogInputTerminal /
 * MultiAnalogInputTerminal, IAnalogOutputTerminal / AnalogOutputTerminal /
 * MultiAnalogOutputTerminal): the Error/Result pair, the DeviceIdentity
 * used for discovery matching, and the StartOptions shared by
 * single-device and chained bring-up.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>

namespace EtherCAT {

class Master;

namespace Beckhoff {

// ============================================================================
// Shared error type
// ============================================================================

enum class Error : uint8_t {
    Ok = 0,
    NoDeviceFound,          ///< detect()/findFirst(): no matching device on the bus
    WrongDevice,            ///< slave identity does not match vendor/product
    SlaveIndexOutOfRange,   ///< index exceeds the PDO manager's slave table
    SmConfigFailed,         ///< could not write the sync-manager registers
    PdoRegistrationFailed,  ///< add_rxpdo()/add_txpdo() rejected the buffer
    FmmuConfigFailed,       ///< could not program the FMMU
    LogicalMapMissing,      ///< no logical address was assigned to this slave
    PreOpFailed,            ///< INIT -> PRE-OP transition failed
    SafeOpFailed,           ///< PRE-OP -> SAFE-OP transition failed
    OpRequestFailed,        ///< SAFE-OP -> OP request could not be sent
    OpTimeout,              ///< slave did not reach OP in time
    LoopStartFailed,        ///< master realtime loop failed to start
    TooManyBits,            ///< chain exceeds the MaxBits capacity
    FsoeInitFailed,         ///< FSoE master connection could not be initialized
    Cancelled,              ///< aborted via master.requestCancel()
};

/// Human-readable description of an Error value.
const char* errorToString(Error e);

/// Expected-like result type used by all fallible operations.
template <typename T = void>
using Result = std::expected<T, Error>;

// ============================================================================
// Device identity
// ============================================================================

/**
 * @brief Static description of a supported packed-bit terminal
 *        (input or output direction).
 *
 * `vendor_id`/`product_code` are matched against discovery results.
 * `num_bits` declares the packed bit width for digital terminals and the
 * channel count for analog terminals; when it is 0 the count is derived
 * from the SII PDO data during bring-up (the Multi* chains need a
 * non-zero declared count to lay out the flat channel space before
 * start()).
 */
struct DeviceIdentity {
    uint32_t    vendor_id;
    uint32_t    product_code;
    uint16_t    num_bits    = 0;
    const char* name        = nullptr;
};

// ============================================================================
// Start options (shared by single-device and Multi bring-up)
// ============================================================================

struct StartOptions {
    /// When true (default), the driver installs a motion-control callback
    /// that exchanges the PDO manager(s) covering the device(s) and starts
    /// the master's realtime loop if it is not already running.  Set false
    /// when the application manages its own cyclic exchange.
    bool     manage_realtime_loop = true;
    /// Cycle period used when the driver starts the RT loop (microseconds).
    uint32_t cycle_period_us      = 1000;
    /// How long to wait for the OP state confirmation (milliseconds).
    int      op_timeout_ms        = 5000;
    /// Base of the chain's logical address space (Multi* only).
    /// 0 = auto-select a non-overlapping range.
    uint32_t base_logical_addr    = 0;
};

} // namespace Beckhoff

} // namespace EtherCAT
