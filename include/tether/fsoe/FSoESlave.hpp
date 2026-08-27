/**
 * @file FSoESlave.hpp
 * @brief FSoE (Fail-Safe over EtherCAT) Slave Implementation
 *
 * Provides complete FSoE safety protocol implementation for EtherCAT slaves.
 *
 * Features:
 * - Complete FSoE state machine (ETG.5100 compliant)
 * - CRC-16 calculation and verification
 * - Watchdog monitoring with configurable timeout
 * - Session management with sequence numbers
 * - Safe I/O data exchange
 * - Fail-safe state handling
 * - Error injection for testing
 * - Configurable safety integrity level (SIL1-SIL3)
 *
 * ## FSoE State Machine
 *
 * ```
 *                    ┌─────────────────┐
 *                    │      RESET      │◄────────────────┐
 *                    └────────┬────────┘                 │
 *                             │ (Init)                   │
 *                    ┌────────▼────────┐                 │
 *                    │     SESSION     │─────────────────┤
 *                    └────────┬────────┘                 │
 *                             │ (Session OK)             │
 *                    ┌────────▼────────┐                 │
 *                    │   CONNECTION    │─────────────────┤
 *                    └────────┬────────┘                 │ (Error)
 *                             │ (Params OK)              │
 *                    ┌────────▼────────┐                 │
 *                    │    PARAMETER    │─────────────────┤
 *                    └────────┬────────┘                 │
 *                             │ (Config OK)              │
 *                    ┌────────▼────────┐                 │
 *                    │      DATA       │◄──────┐         │
 *                    └────────┬────────┘       │         │
 *                             │ (Error)        │(Recover)│
 *                    ┌────────▼────────┐       │         │
 *                    │    FAIL-SAFE    │───────┘─────────┘
 *                    └─────────────────┘
 * ```
 */

#pragma once

#include "fsoe/FSoEDefs.hpp"
#include "fsoe/FSoEStatistics.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <span>
#include <vector>

namespace FSoE {

// ============================================================================
// Error Injection Types (for testing)
// ============================================================================

/**
 * @brief FSoE error injection configuration for testing
 */
struct FSoEErrorInjection {
    bool enabled = false;
    
    // CRC errors
    bool injectCRCError = false;
    uint32_t crcErrorRate = 0;       // 0 = every packet, 1 = every other, etc.
    uint32_t crcErrorCounter = 0;
    
    // Sequence errors
    bool injectSequenceError = false;
    int8_t sequenceOffset = 0;       // Add this to sequence number
    
    // Connection ID errors
    bool injectConnIdError = false;
    uint16_t fakeConnId = 0;
    
    // Watchdog simulation
    bool simulateWatchdogTimeout = false;
    uint32_t watchdogDelayMs = 0;
    
    // Data corruption
    bool corruptData = false;
    uint8_t corruptBitMask = 0xFF;
    size_t corruptByteIndex = 0;
    
    // Complete frame drop
    bool dropFrames = false;
    uint32_t dropRate = 0;
    uint32_t dropCounter = 0;
    
    // Delayed response
    bool delayResponse = false;
    uint32_t delayMs = 0;
    
    // State machine interference
    bool forceFailSafe = false;
    bool preventRecovery = false;
    
    void reset() {
        enabled = false;
        injectCRCError = false;
        injectSequenceError = false;
        injectConnIdError = false;
        simulateWatchdogTimeout = false;
        corruptData = false;
        dropFrames = false;
        delayResponse = false;
        forceFailSafe = false;
        preventRecovery = false;
        crcErrorCounter = 0;
        dropCounter = 0;
    }
};

// ============================================================================
// FSoE Slave Configuration
// ============================================================================

/**
 * @brief FSoE slave configuration
 */
struct FSoESlaveConfig {
    // Identity
    uint16_t slaveAddress = 0;         ///< EtherCAT slave address
    uint16_t connectionId = 0;         ///< FSoE connection ID (assigned by master)
    uint16_t safetyAddress = 0;        ///< FSoE safety address
    
    // Safety level
    uint8_t safetyLevel = SIL::SIL2;   ///< Required SIL level
    
    // Timing
    uint16_t watchdogTimeoutMs = 100;  ///< Watchdog timeout (ms)
    uint16_t connectionTimeoutMs = 1000; ///< Connection timeout (ms)
    uint16_t sessionTimeoutMs = 5000;  ///< Session establishment timeout (ms)
    
    // Data configuration
    uint8_t safeInputSize = 0;         ///< Safe input data size (bytes)
    uint8_t safeOutputSize = 0;        ///< Safe output data size (bytes)
    std::array<uint8_t, 16> failSafeInputs{};   ///< Fail-safe input values
    std::array<uint8_t, 16> failSafeOutputs{};  ///< Fail-safe output values

    /// Expected safety-related application parameters received from the
    /// master in the Parameter state (ETG.5100 S (D) V1.2.0, §8.2.2.5,
    /// Table 18, octets 6+).  The slave validates that the received app
    /// parameters match these expected values.  Empty = no app parameters
    /// expected (app param length = 0).
    /// See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    std::vector<uint8_t> expectedAppParameters;
    
    // Behavior configuration
    bool autoRecoveryEnabled = true;   ///< Automatically recover from fail-safe
    uint32_t recoveryDelayMs = 1000;   ///< Delay before attempting recovery
    bool strictCrcCheck = true;        ///< Reject all CRC errors (always enforced)
    bool strictSequenceCheck = true;   ///< Reject sequence errors (deprecated, no-op)

    /// Initial sequence number used for the Reset response and the
    /// expected master Reset frame.  ETG.5100 §8.1.3.4 says
    /// sequence numbers start at 1 (0 is never used), but some
    /// masters (e.g. for Synapticon SOMANET) use seq=0 for the
    /// initial Reset frame.  Default is 0 (Synapticon convention).
    uint16_t initialSeqNo = 0;
    
    // Error handling
    bool treatCrcErrorAsCritical = true;
    bool treatSequenceErrorAsCritical = true;
    bool treatTimeoutAsCritical = true;
    bool treatConnIdErrorAsCritical = true;

    // CRC model for state-transition responses (Session, Connection,
    // Parameter).  When true, the slave resets the CRC chain (start_crc=0,
    // seq=initialSeqNo) at each state transition, matching the ESC211
    // master's behavior.  When false (default), the slave uses
    // cross-direction CRC inheritance (start_crc=last_rx_crc0_), matching
    // the Synapticon master's behavior.
    bool resetCrcOnStateTransition = false;

    // Parameter CRC verification (0 = skip verification)
    uint16_t expectedParameterCRC = 0;
    
    // Diagnostics
    bool enableDiagnostics = true;
    uint32_t maxErrorLogEntries = 100;
};

// ============================================================================
// Callback Types
// ============================================================================

using FSoEStateCallback = std::function<void(uint8_t oldState, uint8_t newState)>;
using FSoEErrorCallback = std::function<void(uint16_t errorCode, bool isCritical, const FSoEErrorDetail& detail)>;
using FSoEFailSafeCallback = std::function<void()>;
using FSoEDataValidCallback = std::function<void(const uint8_t* data, size_t len)>;
using FSoERecoveryCallback = std::function<bool()>;  // Return true to allow recovery

// ============================================================================
// FSoE Slave Class
// ============================================================================

/**
 * @brief FSoE Slave Implementation
 *
 * Complete FSoE safety protocol handler for EtherCAT slaves.
 */
class FSoESlave {
public:
    explicit FSoESlave(const FSoESlaveConfig& config);
    ~FSoESlave();
    
    // Non-copyable
    FSoESlave(const FSoESlave&) = delete;
    FSoESlave& operator=(const FSoESlave&) = delete;
    
    // ========================================================================
    // Initialization
    // ========================================================================
    
    /**
     * @brief Initialize FSoE slave
     * @return true on success
     */
    bool initialize();
    
    /**
     * @brief Check if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get configuration
     */
    const FSoESlaveConfig& getConfig() const { return config_; }

    /**
     * @brief Reconfigure (must be in RESET state)
     */
    bool reconfigure(const FSoESlaveConfig& config);

    // ========================================================================
    // State Machine
    // ========================================================================

    /**
     * @brief Get current FSoE state
     */
    uint8_t getState() const { return state_.load(); }

    /**
     * @brief Get state name
     */
    const char* getStateName() const;

    /**
     * @brief Check if in operational (DATA) state
     */
    bool isOperational() const { return state_.load() == ConnectionState::Data; }

    /**
     * @brief Check if in fail-safe state
     */
    bool isFailSafe() const;

    /**
     * @brief Check if error occurred
     */
    bool hasError() const;

    /**
     * @brief Get last error code
     */
    uint16_t getLastError() const;
    
    /**
     * @brief Reset state machine
     */
    void reset();
    
    /**
     * @brief Force transition to fail-safe state
     */
    void triggerFailSafe(uint16_t errorCode = ErrorCode::ApplicationError);
    
    /**
     * @brief Attempt recovery from fail-safe
     * @return true if recovery initiated
     */
    bool attemptRecovery();
    
    // ========================================================================
    // Frame Processing
    // ========================================================================
    
    /**
     * @brief Process incoming FSoE frame from master
     * @param data Frame data
     * @param len Frame length
     * @return true if frame was valid
     */
    bool processRxFrame(const uint8_t* data, size_t len);
    
    /**
     * @brief Prepare outgoing FSoE frame to master
     * @param data Buffer for frame
     * @param maxLen Maximum buffer size
     * @return Actual frame size, 0 on error
     */
    size_t prepareTxFrame(uint8_t* data, size_t maxLen);
    
    /**
     * @brief Update state machine (call periodically)
     * @param currentTimeMs Current time in milliseconds
     */
    void update(uint64_t currentTimeMs);
    
    // ========================================================================
    // Safe Data Access
    // ========================================================================
    
    /**
     * @brief Set safe input data (slave -> master)
     * @param data Input data
     * @param len Length
     * @return true on success
     */
    bool setSafeInputs(const uint8_t* data, size_t len);

    /**
     * @brief Write the raw slave-to-master process image.
     */
    bool writeInputProcessData(std::span<const uint8_t> data);
    
    /**
     * @brief Get safe output data (master -> slave)
     * @param data Buffer for output data
     * @param len Buffer length
     * @return Actual data length, 0 if no valid data
     */
    size_t getSafeOutputs(uint8_t* data, size_t len) const;

    /**
     * @brief Read the raw master-to-slave process image.
     */
    size_t readOutputProcessData(std::span<uint8_t> data) const;

    /**
     * @brief Return the currently published slave-to-master process image.
     */
    std::vector<uint8_t> inputProcessData() const;

    /**
     * @brief Return the last valid master-to-slave process image.
     */
    std::vector<uint8_t> outputProcessData() const;
    
    /**
     * @brief Check if safe outputs are valid
     */
    bool areSafeOutputsValid() const;
    
    /**
     * @brief Apply fail-safe values to outputs
     */
    void applyFailSafeOutputs();
    
    // ========================================================================
    // Bit-level Safe I/O
    // ========================================================================
    
    bool getSafeOutputBit(uint8_t bitIndex) const;
    bool setSafeInputBit(uint8_t bitIndex, bool value);
    
    // ========================================================================
    // Callbacks
    // ========================================================================
    
    void setStateCallback(FSoEStateCallback callback) { stateCallback_ = callback; }
    void setErrorCallback(FSoEErrorCallback callback) { errorCallback_ = callback; }
    void setFailSafeCallback(FSoEFailSafeCallback callback) { failSafeCallback_ = callback; }
    void setDataValidCallback(FSoEDataValidCallback callback) { dataValidCallback_ = callback; }
    void setRecoveryCallback(FSoERecoveryCallback callback) { recoveryCallback_ = callback; }
    
    // ========================================================================
    // Statistics and Diagnostics
    // ========================================================================

    /**
     * @brief Get statistics (thread-safe snapshot by value)
     */
    FSoESlaveStats getStats() const;

    /// Get the RX CRC state that the MASTER should chain from when
    /// building frames to send TO this slave (cross-direction inheritance).
    /// The master's TX start_crc = the slave's own last TX CRC0.
    /// Tests that simulate master→slave TX should use this as start_crc.
    uint16_t getRxLastCrc0() const { return last_tx_crc0_; }
    uint16_t getRxSeqNo() const { return rx_seq_no_; }
    /// Get the TX CRC state that this slave chains from when building
    /// its own TX frames (cross-direction: slave TX inherits from the
    /// master's last TX CRC0 = last_rx_crc0_).
    /// startCrc = last_rx_crc0_, seqNo = last_rx_seq_no_ (echoes master's
    /// last TX seq; Reset response uses seq=initialSeqNo, same as master)
    uint16_t getTxLastCrc0() const { return last_rx_crc0_; }
    uint16_t getTxSeqNo() const { return last_rx_seq_no_; }

    /// Get the slave's own Session ID (ETG.5100 §8.2.2.3: the slave
    /// generates its own random Session ID, independent of the master's).
    uint16_t getSessionId() const { return sessionId_; }

    /**
     * @brief Reset statistics
     */
    void resetStats();

    /**
     * @brief Get diagnostic log (thread-safe copy by value)
     */
    std::vector<FSoEDiagnosticEntry> getDiagnostics() const;

    /**
     * @brief Clear diagnostic log
     */
    void clearDiagnostics();

    // ========================================================================
    // Error Injection (Testing)
    // ========================================================================

    /**
     * @brief Get error injection configuration (mutable, for test setup only)
     * @note Not thread-safe — call before starting cyclic processing.
     */
    FSoEErrorInjection& getErrorInjection() { return errorInjection_; }

    /**
     * @brief Set error injection configuration
     */
    void setErrorInjection(const FSoEErrorInjection& injection);

    /**
     * @brief Check if error injection is enabled
     */
    bool isErrorInjectionEnabled() const;

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================
    
    /// Transition to a new connection state.
    /// @note Invariant: caller must hold mutex_. All state-correlated fields
    ///       (stateEntryTimeMs_, lastError_, failSafeActive_, dataValid_)
    ///       are updated atomically with state_ under that lock.
    void transitionTo(uint8_t newState);
    bool validateFrame(const uint8_t* data, size_t len);
    bool validateCRC(const uint8_t* data, size_t len);
    bool validateSequence(uint8_t seqNum);
    bool validateConnectionId(uint16_t connId);
    uint16_t calculateCRC(const uint8_t* data, size_t len);
    
    void processSessionReset(const uint8_t* data, size_t len);
    void processConnection(const uint8_t* data, size_t len);
    void processParameter(const uint8_t* data, size_t len);
    void processData(const uint8_t* data, size_t len);
    
    size_t buildResetResponse(uint8_t* data, size_t maxLen);
    size_t buildSessionResponse(uint8_t* data, size_t maxLen);
    size_t buildConnectionResponse(uint8_t* data, size_t maxLen);
    size_t buildParameterResponse(uint8_t* data, size_t maxLen);
    size_t buildDataResponse(uint8_t* data, size_t maxLen);
    size_t buildFailSafeResponse(uint8_t* data, size_t maxLen);
    
    void handleWatchdog(uint64_t currentTimeMs);
    void handleTimeout(uint64_t currentTimeMs);
    void handleError(uint16_t errorCode, bool isCritical,
                     const FSoEErrorDetail& detail = {});
    void logDiagnostic(uint16_t errorCode, const char* message);
    
    bool shouldInjectCRCError();
    bool shouldDropFrame();
    void applyDataCorruption(uint8_t* data, size_t len);
    
    // ========================================================================
    // Member Variables
    // ========================================================================
    
    FSoESlaveConfig config_;
    bool initialized_ = false;
    
    // State machine
    std::atomic<uint8_t> state_{ConnectionState::Reset};
    uint16_t lastError_ = ErrorCode::NoError;
    bool failSafeActive_ = false;
    bool dataValid_ = false;
    
    // Session/sequence management
    // ETG.5100 S (D) V1.2.0, §8.2.2.3: The slave generates its OWN random
    // Slave Session ID — it must NOT echo the master's Session ID.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    uint16_t sessionId_ = 0;
    uint16_t currentConnectionId_ = 0;
    uint16_t receivedParameterCRC_ = 0;
    uint8_t expectedSequence_ = 0;
    uint8_t txSequence_ = 0;

    // Session ID octet index for 1-octet safety data.
    // ETG.5100 §8.2.2.3: When safety data length is 1 octet, the 16-bit
    // Session ID is transferred in two successive PDUs (low byte first,
    // then high byte).  For safety data length >= 2, both bytes fit in a
    // single PDU and this index stays at 0.
    // See: https://techoverflow.net/2026/08/12/fsoe-session-pdu-master-and-slave-structure/
    uint8_t sessionOctetIdx_ = 0;
    bool sessionOctetAdvancePending_ = false;  ///< Advance sessionOctetIdx_ after next buildSessionResponse
    bool sessionFirstRxDone_ = false;  ///< True after first Session RX (state-transition reset only on first)

    // Connection state multi-cycle transfer.
    // ETG.5100 S (D) V1.2.0, §8.2.2.4, Table 15:
    // The Connection state transfers 4 bytes (2-byte Connection ID +
    // 2-byte FSoE Slave Address) in SafeData.  When safety data length
    // is less than 4 octets, multiple cycles are needed:
    //   4 octets → 1 cycle, 2 octets → 2 cycles, 1 octet → 4 cycles
    // See: https://techoverflow.net/2026/08/12/fsoe-connection-pdu-master-and-slave-structure/
    uint8_t connectionRxIdx_ = 0;       ///< RX byte offset (advances by safeOutputSize)
    uint8_t connectionTxIdx_ = 0;       ///< TX echo byte offset (advances by safeInputSize)
    uint8_t connectionBuf_[4] = {0};    ///< Accumulated RX / echo buffer
    bool connectionTxAdvancePending_ = false;  ///< Advance connectionTxIdx_ after next buildConnectionResponse

    // Parameter state multi-cycle transfer.
    // ETG.5100 S (D) V1.2.0, §8.2.2.5, Table 18:
    // The Parameter state transfers a variable-length payload:
    //   octets 0-1: comm param length (always 2, LE)
    //   octets 2-3: FSoE watchdog (ms, LE)
    //   octets 4-5: app param length (LE)
    //   octets 6+:  app param bytes
    // The slave accumulates received bytes and echoes them back each cycle.
    // See: https://techoverflow.net/2026/08/12/fsoe-parameter-pdu-master-and-slave-structure/
    static constexpr uint16_t PARAM_BUF_SIZE = 256 + 6;
    uint16_t paramRxIdx_ = 0;           ///< RX byte offset (advances by safeOutputSize)
    uint16_t paramTxIdx_ = 0;           ///< TX echo byte offset (advances by safeInputSize)
    std::vector<uint8_t> paramBuf_;     ///< Accumulated RX / echo buffer
    bool paramTxAdvancePending_ = false; ///< Advance paramTxIdx_ after next buildParameterResponse

    // FSoE CRC inheritance and sequence number tracking.
    // See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
    // ETG.5100 §8.1.3.4: sequence numbers are 1..65535 (0 is never used).
    //
    // CRC inheritance model (verified on real Synapticon hardware):
    //   - Slave TX: start_crc = last_rx_crc0_ (master's last TX CRC0, cross-direction)
    //     seq = last_rx_seq_no_ (echoes master's last TX seq, cross-direction)
    //     Exception: Reset response uses seq = initialSeqNo (slave's own
    //     initial seq, same value as master's initial_seq_no)
    //   - Slave RX: start_crc = last_tx_crc0_ (own last TX CRC0, cross-direction)
    //     seq = rx_seq_no_ (expected next master TX seq)
    //   - Reset breaks the chain: both sides reset to start_crc=0
    //   - The slave echoes the master's last TX seq (no independent increment).
    //     The master expects slave TX seq = last_tx_seq_no_ (the seq it just used).
    uint16_t last_tx_crc0_ = 0;   ///< CRC0 of the last frame we sent (for RX parsing)
    uint16_t last_rx_crc0_ = 0;   ///< CRC0 of the master's last TX (startCrc for slave TX)
    uint16_t tx_seq_no_ = 1;      ///< Kept for diagnostics (TX uses last_rx_seq_no_)
    uint16_t rx_seq_no_ = 1;      ///< Expected next master TX seq (for RX validation)
    uint16_t last_tx_seq_no_ = 0; ///< Seq used in the last TX (for diagnostics)
    uint16_t last_rx_seq_no_ = 0; ///< Seq used by master in its last TX (echoed in slave TX)
    
    // Timing
    uint64_t lastValidFrameMs_ = 0;
    uint64_t stateEntryTimeMs_ = 0;
    uint64_t lastUpdateTimeMs_ = 0;
    uint64_t recoveryAttemptTimeMs_ = 0;
    uint64_t failSafeEnteredMs_ = 0;
    
    // Data buffers
    std::array<uint8_t, 16> safeInputs_{};
    std::array<uint8_t, 16> safeOutputs_{};
    std::array<uint8_t, 32> rxBuffer_{};
    std::array<uint8_t, 32> txBuffer_{};

    // Duplicate frame detection for the PDO path.  When the master
    // resends the same frame bytes every cycle (which is the correct
    // PDO behavior — the master only rebuilds the frame on state
    // transitions), the slave must detect duplicates and skip CRC
    // advancement.  Without this, the slave's RX CRC would advance
    // on every duplicate, causing CRC divergence with the master.
    std::vector<uint8_t> last_rx_frame_bytes_;

    // Cached TX response for the current state.  In the PDO path, the
    // slave should send the SAME response bytes every cycle while in the
    // same state (same CRC, same seq).  This prevents CRC chain divergence
    // when the master resends the same frame (duplicate detection).  The
    // response is rebuilt only when the state transitions or the fail-safe
    // flag changes.
    std::vector<uint8_t> cached_tx_response_;
    uint8_t cached_tx_state_ = 0xFF;       ///< State when cached response was built
    bool cached_tx_fail_safe_ = false;     ///< Fail-safe flag when cached
    bool tx_cache_valid_ = false;          ///< True when cache should be used (after RX duplicate)
    
    // Callbacks
    FSoEStateCallback stateCallback_;
    FSoEErrorCallback errorCallback_;
    FSoEFailSafeCallback failSafeCallback_;
    FSoEDataValidCallback dataValidCallback_;
    FSoERecoveryCallback recoveryCallback_;
    
    // Statistics and diagnostics (managed by FSoEStatistics)
    FSoEStatistics statistics_;
    
    // Error injection
    FSoEErrorInjection errorInjection_;

    // Thread safety
    mutable std::recursive_mutex mutex_;

    // Session ID generation (ETG.5100 §8.2.2.3: slave generates its own
    // random Session ID, independent of the master's)
    std::mt19937 rng_{std::random_device{}()};
};

// ============================================================================
// CRC-16 Utilities (FSoE specific polynomial)
// ============================================================================

} // namespace FSoE
