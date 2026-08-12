/**
 * @file FSoEMasterConnection.hpp
 * @brief FSoE (Fail-Safe over EtherCAT) Master Connection — redesigned
 *
 * Replaces FSoEConnection with a correct, thread-safe, master-driven
 * FSoE protocol implementation that fixes all identified issues:
 * - Unified CRC (shared table-based implementation with final XOR)
 * - Master-driven protocol phases (Session → Connection → Parameter → Data)
 * - Parameter exchange with CRC verification
 * - Phase timeouts for all states
 * - Recovery from FailSafe/Error states
 * - Strict sequence validation
 * - Data length validation
 * - Per-state command validation
 * - Thread-safe via recursive mutex
 */

#pragma once

#include "fsoe/FSoEDefs.hpp"
#include "fsoe/FSoECRC.hpp"
#include "tether/utils/EventSource.hpp"
#include <cstdint>
#include <string>
#include <functional>
#include <array>
#include <span>
#include <vector>
#include <memory>
#include <mutex>
#include <random>

namespace FSoE {

class FSoESlave;

// ============================================================================
// Configuration
// ============================================================================

struct MasterConnectionConfig {
    uint16_t slave_addr = 0;
    uint16_t slave_safety_addr = 0;     // FSoE slave safety address
    uint16_t connection_id = 0;
    uint16_t master_addr = 0;
    uint16_t watchdog_timeout_ms = 100;
    uint16_t conn_timeout_ms = 1000;
    uint16_t session_timeout_ms = 5000;
    uint16_t recovery_delay_ms = 500;
    uint16_t reset_timeout_ms = 500;   ///< Reset state fallback timeout;
                                       ///< 0 = wait forever for slave response
    uint8_t  safety_level = SIL::SIL2;
    uint8_t  input_size = 0;
    uint8_t  output_size = 0;
    std::array<uint8_t, 16> fail_safe_values = {0};

    bool auto_recovery_enabled = true;
    bool auto_fail_safe_on_error = true;

    /// Number of PDO cycles the slave's FSoE response is delayed relative
    /// to the master's TX command.  In EtherCAT, the slave's TxPDO is
    /// always at least one cycle behind the master's RxPDO (master TX in
    /// cycle N → slave processes → slave TX in cycle N+1).  Some slaves
    /// have additional internal processing delay (e.g. the Synapticon
    /// SOMANET runs FSoE at a lower rate, resulting in ~8 cycles of
    /// delay).  When the master transitions to a new state (changing the
    /// TX command byte), the slave's TxPDO still contains the response
    /// to the PREVIOUS command for this many cycles.  The master skips
    /// RX processing during this period to avoid triggering a spurious
    /// "unexpected command" fail-safe from the stale response.
    /// Set to 0 to disable command-change skipping (useful for tests
    /// with an immediate-response slave).  Default is 1 (standard
    /// one-cycle EtherCAT pipeline delay).
    uint8_t slave_response_delay_cycles = 1;

    /// Initial sequence number used for the Reset frame and the
    /// expected slave Reset response.  ETG.5100 §8.1.3.4 says
    /// sequence numbers start at 1 (0 is never used), but some
    /// slaves (e.g. Synapticon SOMANET) expect seq=0 for the
    /// initial Reset frame.  Default is 0 (Synapticon convention).
    uint16_t initial_seq_no = 0;
};

struct MasterConnectionStatus {
    uint8_t  state = ConnectionState::Reset;
    uint16_t error_code = ErrorCode::NoError;
    uint16_t session_id = 0;
    uint8_t  sequence_number = 0;
    bool     data_valid = false;
    bool     fail_safe_active = false;
    uint32_t watchdog_counter = 0;
    uint64_t last_valid_frame_ms = 0;
    uint64_t state_entered_ms = 0;

    bool isOperational() const { return state == ConnectionState::Data; }
    bool isFailSafe() const { return state == ConnectionState::FailSafe || fail_safe_active; }
    bool hasError() const { return state == ConnectionState::Error || error_code != ErrorCode::NoError; }
};

// ============================================================================
// Callback Types
// ============================================================================

using StateChangeCallback = std::function<void(uint8_t old_state, uint8_t new_state)>;
using ErrorCallback = std::function<void(uint16_t error_code, const FSoEErrorDetail& detail)>;
using FailSafeCallback = std::function<void()>;
using DataCallback = std::function<void(const uint8_t* data, size_t len)>;

/// Human-readable protocol trace callback.
///
/// When installed via setTraceCallback(), the connection emits high-level
/// descriptions of protocol decisions, e.g.:
///   "TX Reset(0x2A): forcing slave to initial state"
///   "RX Session(0x4E): slave accepted session, moving to Connection"
/// This is the "what is the FSoE master trying to do and what did it get
/// back" view — no raw frame bytes.  For raw frame dumps, use the
/// txFrameEvents()/rxFrameEvents() event sources.
///
/// The callback is invoked synchronously from within prepareTxFrame() /
/// processRxFrame() while the connection's recursive mutex is held.
using TraceCallback = std::function<void(const char* message)>;

/// Per-cycle sequence trace info (--debug fsoe-sequence).
///
/// When installed via setSequenceTraceCallback(), the connection emits one
/// structured summary per exchangeViaPDO() call, describing:
///   - whether the RX frame was accepted or rejected (and why)
///   - whether the TX frame was rebuilt this cycle
///   - whether the FSoE state changed as a result
///
/// This is the "every cycle, what happened" view — useful for debugging
/// handshake stalls and understanding why the master is stuck in a state.
struct SequenceTraceInfo {
    uint32_t cycle;           ///< PDO cycle count (1-based)
    uint8_t  state_before;    ///< FSoE state at start of cycle
    uint8_t  state_after;     ///< FSoE state at end of cycle
    bool     state_changed;   ///< true if state_before != state_after
    bool     frame_accepted;  ///< true if RX frame was processed by processRxFrame
    bool     tx_rebuilt;      ///< true if TX frame was rebuilt (state change or output change)
    uint8_t  rx_cmd;          ///< Command byte from RX frame (0 if no RX processed)
    const char* reason;       ///< Short reason string for accept/reject
};

using SequenceTraceCallback = std::function<void(const SequenceTraceInfo&)>;

/// CRC trace info (--debug fsoe-crc).
///
/// When installed via setCrcTraceCallback(), the connection emits the
/// exact CRC parameters used when building (TX) and checking (RX) every
/// FSoE frame.  This is the low-level "what CRC inputs did the master
/// use" view — essential for diagnosing CRC/seq synchronization issues
/// with real slaves.
///
/// For TX frames, the callback is invoked from prepareTxFrame() after the
/// frame is built.  For RX frames, it is invoked from processRxFrame()
/// after the CRC check (whether it passed or failed).
struct CrcTraceInfo {
    enum class Direction : uint8_t { TX, RX };
    Direction direction;      ///< TX (master building) or RX (master checking)
    uint8_t  command;         ///< FSoE command byte (0x2A, 0x4E, 0x64, ...)
    uint8_t  state;           ///< FSoE state when this frame was built/checked
    uint16_t start_crc;       ///< start_crc used for CRC computation (inheritance)
    uint16_t seq_expected;    ///< seq initially tried (before collision avoidance)
    uint16_t seq_used;        ///< seq that actually matched/was used (after CA)
    uint16_t crc0;            ///< resulting CRC0 (0 if parse failed and no fallback)
    bool     crc_ok;          ///< true if CRC verified (RX) or frame built (TX)
    bool     fallback_used;   ///< true if seq±1 fallback was used (RX only)
    int      fallback_delta;  ///< -1, 0, or +1 (RX only; 0 = no fallback)
    uint16_t conn_id;         ///< Connection ID in the frame
    size_t   data_len;        ///< SafeData length in bytes
};

using CrcTraceCallback = std::function<void(const CrcTraceInfo&)>;

/// Event source for FSoE frame events.  Each listener receives an immutable
/// shared_ptr<const std::vector<uint8_t>> copy of the frame bytes.  When no
/// listeners are registered, emit() performs no allocation.
using FrameEventSource = Tether::Utils::EventSource<std::vector<uint8_t>>;

// ============================================================================
// FSoEMasterConnection — master-side FSoE connection to a single slave
// ============================================================================

class FSoEMasterConnection {
public:
    explicit FSoEMasterConnection(const MasterConnectionConfig& config);
    ~FSoEMasterConnection();

    // --- Initialization ---
    bool initialize();
    bool isInitialized() const;
    const MasterConnectionConfig& getConfig() const;

    // --- Connection Control ---
    bool startConnection();
    bool resetConnection();
    bool requestSessionReset();
    void triggerFailSafe(uint16_t error_code = ErrorCode::ApplicationError);
    bool clearError();

    // --- State Machine ---
    bool processRxFrame(const uint8_t* data, size_t len);
    size_t prepareTxFrame(uint8_t* data, size_t max_len);
    void update(uint64_t current_time_ms);

    // --- Safe Data Access ---
    bool setSafeOutputs(const uint8_t* data, size_t len);
    bool writeOutputProcessData(std::span<const uint8_t> data);
    size_t getSafeInputs(uint8_t* data, size_t len) const;
    size_t readInputProcessData(std::span<uint8_t> data) const;
    std::vector<uint8_t> inputProcessData() const;
    std::vector<uint8_t> outputProcessData() const;

    // --- Simulation helper (for testing with FSoESlave) ---
    bool exchangeWith(FSoESlave& slave, uint64_t current_time_ms);

    /// Exchange FSoE frames via EtherCAT PDO buffers (real drive communication).
    ///
    /// Runs the FSoE state machine, builds the master→slave frame into
    /// @p rx_pdo_out (the RxPDO buffer the master writes each cycle), and
    /// processes the slave→master frame from @p tx_pdo_in (the TxPDO buffer
    /// the drive populates each cycle).  This is the generic FSoE-over-PDO
    /// master exchange used by any drive profile that maps FSoE PDUs onto
    /// EtherCAT PDOs.
    ///
    /// @param rx_pdo_out      Output buffer for the master→slave FSoE frame
    /// @param rx_pdo_max      Capacity of rx_pdo_out
    /// @param tx_pdo_in       Input buffer with the slave→master FSoE frame
    /// @param tx_pdo_len      Number of valid bytes in tx_pdo_in
    /// @param current_time_ms Monotonic time in milliseconds
    /// @return true if the frame was processed successfully
    bool exchangeViaPDO(uint8_t* rx_pdo_out, size_t rx_pdo_max,
                        const uint8_t* tx_pdo_in, size_t tx_pdo_len,
                        uint64_t current_time_ms);

    bool areSafeInputsValid() const;

    // --- Bit-level Safe I/O ---
    bool getSafeInputBit(uint8_t bit_index) const;
    bool setSafeOutputBit(uint8_t bit_index, bool value);
    uint8_t getSafeInputByte(uint8_t byte_index) const;
    bool setSafeOutputByte(uint8_t byte_index, uint8_t value);

    // --- Status & Diagnostics ---
    MasterConnectionStatus getStatus() const;
    uint8_t getState() const;
    uint16_t getErrorCode() const;
    bool isOperational() const;
    bool isFailSafe() const;
    ConnectionStats getStats() const;
    void resetStats();
    std::string getDiagnostics() const;

    /// Get the current TX CRC state (self-inheriting TX).
    /// startCrc = last_tx_crc0_, seqNo = tx_seq_no_
    uint16_t getTxLastCrc0() const { return last_tx_crc0_; }
    uint16_t getTxSeqNo() const { return tx_seq_no_; }
    /// Get the current RX CRC state (self-inheriting RX — slave uses
    /// rx_seq_no_ after incrementing, master uses tx_seq_no_ after
    /// incrementing).
    /// startCrc = last_tx_crc0_, seqNo = tx_seq_no_
    uint16_t getRxLastCrc0() const { return last_tx_crc0_; }
    uint16_t getRxSeqNo() const { return tx_seq_no_; }

    // --- Callbacks ---
    void setStateChangeCallback(StateChangeCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setFailSafeCallback(FailSafeCallback callback);
    void setDataCallback(DataCallback callback);
    void setTraceCallback(TraceCallback callback);
    void setSequenceTraceCallback(SequenceTraceCallback callback);
    void setCrcTraceCallback(CrcTraceCallback callback);

    // --- Frame event sources ---
    // Listeners receive a shared_ptr<const std::vector<uint8_t>> copy of the
    // frame bytes.  When no listeners are registered, no allocation occurs.
    // Access is protected by the connection's internal mutex; listeners are
    // invoked synchronously inside prepareTxFrame()/processRxFrame() while
    // the lock is held (recursive — safe to re-enter the connection).
    FrameEventSource& txFrameEvents();
    FrameEventSource& rxFrameEvents();

private:
    // State machine handlers
    void handleResetState(uint8_t cmd, const uint8_t* data, size_t data_len);
    void handleSessionState(uint8_t cmd, const uint8_t* data, size_t data_len);
    void handleConnectionState(uint8_t cmd, const uint8_t* data, size_t data_len);
    void handleParameterState(uint8_t cmd, const uint8_t* data, size_t data_len);
    void handleDataState(uint8_t cmd, const uint8_t* data, size_t data_len);
    void handleFailSafeState(uint8_t cmd, const uint8_t* data, size_t data_len);

    // Frame building
    size_t buildResetFrame(uint8_t* data, size_t max_len);
    size_t buildSessionResetFrame(uint8_t* data, size_t max_len);
    size_t buildConnectionFrame(uint8_t* data, size_t max_len);
    size_t buildParameterFrame(uint8_t* data, size_t max_len);
    size_t buildDataFrame(uint8_t* data, size_t max_len);
    size_t buildFailSafeFrame(uint8_t* data, size_t max_len);

    // Frame validation
    bool validateCRC(const uint8_t* data, size_t len) const;
    bool validateSequence(uint8_t seq);
    bool validateConnectionID(uint16_t conn_id) const;
    static bool isValidCommand(uint8_t cmd);

    // State transitions
    void transitionTo(uint8_t new_state);
    void handleError(uint16_t error_code, const FSoEErrorDetail& detail);
    void checkWatchdog(uint64_t current_time_ms);
    void checkPhaseTimeout(uint64_t current_time_ms);
    void attemptAutoRecovery(uint64_t current_time_ms);

    // Protocol trace helper — formats a human-readable message and forwards
    // it to trace_callback_ (if installed).  No-op when no callback is set.
    void trace(const char* fmt, ...) const;

    // Parameter CRC computation
    uint16_t computeParameterCRC() const;

    MasterConnectionConfig config_;
    MasterConnectionStatus status_;
    ConnectionStats stats_;

    bool initialized_ = false;
    uint64_t current_time_ms_ = 0;

    std::array<uint8_t, 16> safe_inputs_{};
    std::array<uint8_t, 16> safe_outputs_{};

    // FSoE CRC inheritance and sequence number tracking.
    //
    // The FSoE CRC uses two mechanisms that require state across frames:
    //
    // 1. Cross-frame CRC inheritance: each frame's CRC0 is folded into the
    //    next frame's CRC computation as startCrc.  This creates a chain
    //    across frames — an attacker cannot replay an old frame because the
    //    CRC chain would break.
    //    See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
    //
    // 2. Sequence number (ETG.5100 §8.1.3.4): a virtual 16-bit counter folded
    //    into the CRC but NOT transmitted.  Range is 1..65535 (0 is never
    //    used; after 65535 it wraps to 1).  The master and slave each maintain
    //    their own independent counter.  CRC collision avoidance: if the new
    //    CRC0 equals the previous CRC0, the seq is incremented until they
    //    differ.  The checker replicates this algorithm.
    //
    // The master tracks separate CRC and sequence state for TX (frames it
    // sends) and RX (frames it receives).
    //
    // CRC inheritance model (verified on real Synapticon hardware):
    //   - Master TX: start_crc = last_rx_crc0_ (slave's last TX CRC0, cross-direction)
    //     seq = last_rx_seq_no_ (slave's last TX seq, cross-direction)
    //   - Master RX: start_crc = last_tx_crc0_ (own last TX CRC0, self-inheriting)
    //     seq = last_tx_seq_no_ (own last TX seq, self-inheriting)
    //   - Reset breaks the chain: both sides reset to start_crc=0
    //   - No seq increment between frames — seq only advances via collision
    //     avoidance (ETG.5100 §8.1.3.4).
    uint16_t last_tx_crc0_ = 0;   ///< CRC0 of the last frame we sent (for RX parsing)
    uint16_t last_rx_crc0_ = 0;   ///< CRC0 of the last frame we received (startCrc for TX)
    uint16_t tx_seq_no_ = 1;      ///< Kept for diagnostics (not used for TX — use last_rx_seq_no_)
    uint16_t rx_seq_no_ = 1;      ///< Kept for diagnostics (not used for RX — use last_tx_seq_no_)
    uint16_t last_tx_seq_no_ = 0; ///< Seq used in the last TX (for RX parsing)
    uint16_t last_rx_seq_no_ = 0; ///< Seq used by slave in its last TX (startSeq for TX)

    // Legacy sequence tracking (kept for API compatibility, no longer used
    // for actual sequence validation — see tx_seq_no_ / rx_seq_no_ above)
    uint8_t rx_sequence_ = 0;
    uint8_t tx_sequence_ = 0;

    // Parameter exchange state
    uint8_t current_param_index_ = 0;
    uint16_t parameter_crc_ = 0;

    // Recovery state
    uint64_t fail_safe_entered_ms_ = 0;

    // PDO startup state: number of Tx frames sent via exchangeViaPDO.
    // The slave cannot respond until it has received at least one frame,
    // so the first cycle(s) TxPDO is stale (all zeros or uninitialized).
    // We skip RxFrame processing until enough frames have been sent for
    // the slave to have produced a valid response.
    uint32_t pdo_tx_count_ = 0;

    // --- PDO exchange: TX frame caching ---
    // In the PDO path, the master sends the SAME frame bytes every cycle
    // (same CRC, same seq) while the state and safe outputs are unchanged.
    // This prevents CRC chain divergence with slaves that don't process
    // every frame (e.g. Synapticon's FSoE task runs every 8 cycles).
    // The frame is rebuilt only when the state transitions or safe outputs
    // change (tx_cache_dirty_).
    std::vector<uint8_t> cached_tx_pdo_;
    uint8_t cached_tx_pdo_state_ = 0xFF;  ///< State when cached frame was built
    bool tx_cache_dirty_ = true;  ///< True when safe outputs changed (rebuild needed)

    // --- PDO exchange: RX change detection ---
    // In a simultaneous PDO exchange, the RxPDO frame CANNOT be the
    // response to the TxPDO frame sent in the same cycle — the slave
    // has not had time to process it.  The RX is always a response to
    // a PREVIOUS TX (pipeline delay).
    //
    // When the master's TX changes (state transition or safe-output
    // change), the slave's RX will still be the response to the OLD TX
    // for some number of cycles.  The master uses change detection to
    // skip these stale responses:
    //
    //   1. When TX changes, store the current RX as "baseline_rx" (the
    //      last response to the old TX) and enter "expecting_change" mode.
    //   2. In "expecting_change" mode, compare each new RX to baseline_rx:
    //      - Same → stale, increment stale_counter, skip processing.
    //      - Different → slave has processed the new TX, process the RX.
    //      - stale_counter > slave_response_delay_cycles → error → fail-safe.
    //   3. The FSoE timeout (watchdog/conn_timeout) is the ultimate backstop.
    //
    // No hardcoded frame-count assumptions are made beyond the configured
    // FSoE timeout and the configurable stale-frame budget.
    std::vector<uint8_t> baseline_rx_;       ///< RX captured when TX changed (stale baseline)
    std::vector<uint8_t> last_rx_frame_;     ///< Last RX frame received (for diagnostics)
    bool expecting_rx_change_ = false;       ///< True when waiting for RX to reflect new TX
    uint32_t stale_rx_count_ = 0;            ///< Consecutive stale RX frames since TX change

    // Last raw TxPDO bytes received from the slave, for timeout diagnostics.
    // Updated on every exchangeViaPDO call, even when the frame is skipped
    // (startup grace, invalid command, duplicate).  On timeout, this shows
    // what the slave was actually sending.
    std::vector<uint8_t> last_txpdo_;

    // --- Diagnostic seq±1 fallback ---
    // When a CRC mismatch occurs, the master retries with seq-1 and seq+1
    // to detect off-by-one seq synchronization issues (common during
    // interoperability debugging with real slaves).  This is a DIAGNOSTIC
    // aid only — it must not mask persistent errors.  To prevent infinite
    // fallback, a consecutive-fallback counter limits how many times in a
    // row the fallback can rescue a frame.  When a frame verifies with the
    // expected seq (no fallback needed), the counter resets to 0.  When
    // the counter exceeds the configured limit, the fallback is disabled
    // and the CRC error is reported normally.
    static constexpr uint32_t kMaxConsecutiveSeqFallback = 3;
    uint32_t consecutive_seq_fallback_ = 0;
    bool seq_fallback_disabled_ = false;

    // Thread safety
    mutable std::recursive_mutex mutex_;

    // Callbacks
    StateChangeCallback state_change_callback_;
    ErrorCallback error_callback_;
    FailSafeCallback fail_safe_callback_;
    DataCallback data_callback_;
    TraceCallback trace_callback_;
    SequenceTraceCallback sequence_trace_callback_;
    CrcTraceCallback crc_trace_callback_;

    // Frame event sources (multi-listener, no-copy-if-empty)
    FrameEventSource tx_frame_events_;
    FrameEventSource rx_frame_events_;

    // Session ID generation
    std::mt19937 rng_{std::random_device{}()};
};

} // namespace FSoE
