/**
 * @file EEPROMReactor.hpp
 * @brief Per-slave EEPROM state-machine reactor for concurrent multi-slave SII reads
 *
 * @details
 * The EEPROMReactor implements the single-worker, concurrent-router pattern
 * for reading SII EEPROM data from multiple EtherCAT slaves in parallel.
 *
 * ## Problem
 *
 * The synchronous SII reader (`SIIReader::readRaw32()`) performs a 4-step
 * EEPROM protocol for each 32-bit (2-word) read:
 *
 *   1. Write EEPADDR (0x0504) — set the EEPROM word address
 *   2. Write EEPCTL  (0x0502) — issue a READ command
 *   3. Poll  EEPSTAT (0x0502) — wait for the busy bit to clear
 *   4. Read  EEPDAT  (0x0508) — read the 32-bit data word
 *
 * Each step is a blocking register read/write that round-trips through the
 * EtherCAT frame. For N slaves, the synchronous reader does these steps
 * **sequentially per slave**, so the total time is N × (per-slave EEPROM
 * latency × word-pairs needed).
 *
 * ## Solution: reactor pattern
 *
 * The reactor decomposes the 4-step protocol into a per-slave state machine.
 * A single worker thread drives all state machines concurrently:
 *
 *   1. For each slave that is ready for its next protocol step, the reactor
 *      pre-registers a router slot, builds a datagram, and sends it.
 *   2. The reactor calls `TransactionRouter::waitForAny()` to block until
 *      any of the in-flight datagrams completes.
 *   3. The completed response is dispatched to the corresponding slave's
 *      state machine, which advances to its next step.
 *   4. Steps 1–3 repeat until all slaves have read all requested words.
 *
 * This overlaps the EEPROM latency across all slaves: the total time
 * approaches **one slave's EEPROM latency** (not N × one slave's latency).
 *
 * ## Memory safety
 *
 * - Each slave's state machine owns its response buffer (a `RxDatagram`
 *   stored by value in the state machine, not on the caller's stack).
 * - Router slots are cancelled on timeout, error, or reactor shutdown.
 * - The reactor owns all state machines by value (no heap allocation
 *   per slave, no dangling pointers).
 * - `waitForAny()` receives only slot indices (integers), never pointers.
 *
 * ## Reuse of existing SII code
 *
 * The reactor reuses the existing `SIIReader` infrastructure by:
 *   - Exposing the EEPROM register constants and protocol steps through
 *     the `EEPROMProtocol` helper (shared with `SIIReader`).
 *   - Using the same `SIISlaveCache` for word-level caching, so the
 *     reactor's results are immediately visible to `SIIManager::readWord()`
 *     and `SIIParser::parseCategories()`.
 *   - Delegating the "force EEPROM to ECAT control" one-time setup to
 *     `SIIReader::forceEepromToEcat()` (called synchronously before the
 *     reactor loop starts, since it is a one-time per-slave operation).
 *
 * @see TransactionRouter::waitForAny()
 * @see SIIReader
 * @see SIIParser::parseCategories()
 */

#pragma once

#include "tether/ethercat/TetherConfig.hpp"

#if TETHER_ENABLE_SII

#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/sii/SIIManager.hpp"
#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>

namespace EtherCAT {
namespace SII {

// ============================================================================
// EEPROM register constants (shared with SIIReader, per ETG.1000.4)
// ============================================================================

/**
 * @brief EEPROM register addresses and protocol constants.
 *
 * These are factored out of SIIReader so that both the synchronous reader
 * and the asynchronous reactor share the same definitions.
 */
struct EEPROMProtocol {
    static constexpr uint16_t REG_EEPCFG  = 0x0500;  ///< EEPROM configuration
    static constexpr uint16_t REG_EEPCTL  = 0x0502;  ///< EEPROM control/status
    static constexpr uint16_t REG_EEPSTAT = 0x0502;  ///< Same as EEPCTL
    static constexpr uint16_t REG_EEPADDR = 0x0504;  ///< EEPROM word address
    static constexpr uint16_t REG_EEPDAT  = 0x0508;  ///< EEPROM data (32-bit)

    static constexpr uint16_t ECMD_NOP   = 0x0000;  ///< No operation
    static constexpr uint16_t ECMD_READ  = 0x0100;  ///< Read command

    static constexpr uint16_t ESTAT_BUSY    = 0x8000;  ///< Bit 15: busy
    static constexpr uint16_t ESTAT_EMASK   = 0x7800;  ///< Bits 14-11: errors
    static constexpr uint16_t ESTAT_NACK    = 0x2000;  ///< Bit 13: not acknowledged
    static constexpr uint16_t ESTAT_CRC_ERR = 0x0800;  ///< Bit 11: CRC error

    /// Maximum busy-poll iterations before declaring timeout.
    static constexpr int MAX_BUSY_POLLS = 100;

    /// Maximum NACK retries per word read.
    static constexpr int MAX_NACK_RETRIES = 3;
};

// ============================================================================
// EEPROMReadStateMachine — per-slave EEPROM read state machine
// ============================================================================

/**
 * @brief States of the per-slave EEPROM read protocol.
 *
 * The state machine reads a contiguous range of EEPROM word-pairs (32-bit
 * each) from a single slave. Each state corresponds to one EtherCAT
 * register transaction (one datagram in flight).
 *
 * State transitions:
 *
 * @code
 *   IDLE → WRITE_EEPADDR → WRITE_EEPCTL_READ → POLL_EEPSTAT → READ_EEPDAT
 *                                                                 ↓
 *   (advance word address, loop back to WRITE_EEPADDR)
 *                                                                 ↓
 *   (all words read) → DONE
 *
 *   Any state → FAILED (on timeout, error, or cancellation)
 * @endcode
 */
enum class EEPROMState : uint8_t {
    IDLE,               ///< Not started
    WRITE_EEPADDR,      ///< Writing the EEPROM word address to 0x0504
    WRITE_EEPCTL_READ,  ///< Writing the READ command to 0x0502
    POLL_EEPSTAT,       ///< Reading EEPSTAT (0x0502) to check busy bit
    READ_EEPDAT,        ///< Reading 32-bit data from 0x0508
    DONE,               ///< All words read successfully
    FAILED,             ///< Error or timeout
};

/**
 * @brief Per-slave EEPROM read state machine.
 *
 * Each instance drives the 4-step EEPROM read protocol for one slave,
 * reading a contiguous range of word-pairs (32-bit values). The state
 * machine is advanced one step at a time by the reactor.
 *
 * The state machine owns:
 *   - Its response buffer (`RxDatagram`, stored by value — no heap
 *     allocation, no stack-pointer lifetime issues).
 *   - Its router slot index (an integer, safe to pass to `waitForAny()`).
 *   - Its current word address and remaining word count.
 *
 * Memory safety: the response buffer is a member of the state machine,
 * which is a member of the reactor. The reactor outlives all router
 * operations. On destruction, the reactor cancels all pending slots
 * before the state machines (and their buffers) are destroyed.
 */
class EEPROMReadStateMachine {
public:
    /**
     * @brief Construct an idle state machine.
     */
    EEPROMReadStateMachine() = default;

    /**
     * @brief Initialize the state machine for a new read operation.
     *
     * @param slave_index   Slave position on the bus (0-based).
     * @param start_word    First EEPROM word address (must be even).
     * @param word_pair_count  Number of 32-bit word-pairs to read.
     */
    void init(uint16_t slave_index, uint16_t start_word, uint16_t word_pair_count);

    /// Current state.
    EEPROMState state() const { return state_; }

    /// True if the state machine has finished (DONE or FAILED).
    bool isFinished() const {
        return state_ == EEPROMState::DONE || state_ == EEPROMState::FAILED;
    }

    /// True if the state machine needs a datagram to be sent.
    bool needsSend() const {
        return !in_flight_ &&
               (state_ == EEPROMState::WRITE_EEPADDR ||
                state_ == EEPROMState::WRITE_EEPCTL_READ ||
                state_ == EEPROMState::POLL_EEPSTAT ||
                state_ == EEPROMState::READ_EEPDAT);
    }

    /// True if this state machine has a datagram currently in flight.
    bool isInFlight() const { return in_flight_; }

    /// Mark this state machine as having a datagram in flight.
    /// Called by the reactor after issuing a datagram.
    void markInFlight() { in_flight_ = true; }

    /// Slave index this state machine reads from.
    uint16_t slaveIndex() const { return slave_index_; }

    /// Router slot index for the current in-flight datagram.
    size_t slot() const { return slot_; }

    /// Response buffer (owned by this state machine).
    RxDatagram& response() { return response_; }
    const RxDatagram& response() const { return response_; }

    /**
     * @brief Build the datagram spec for the current state.
     *
     * Called by the reactor when this state machine needs to send a
     * datagram. The reactor allocates the transaction index, pre-registers
     * the router slot, and stores the slot in `slot_`.
     *
     * @param idx  Transaction index (allocated by the reactor).
     * @return MultiDatagramSpec for the current protocol step.
     */
    MultiDatagramSpec buildDatagram(uint8_t idx);

    /**
     * @brief Set the router slot for the current in-flight datagram.
     *
     * Called by the reactor after pre-registering the waiter and before
     * sending the datagram.
     */
    void setSlot(size_t slot) { slot_ = slot; }

    /**
     * @brief Process the completion of the current datagram.
     *
     * Called by the reactor when `waitForAny()` reports this state
     * machine's slot has completed. Advances the state machine to its
     * next state (or to FAILED on error).
     *
     * @param result  The WaitResult from the router.
     * @param master  The master (for cache writes on READ_EEPDAT).
     */
    void onComplete(const WaitResult& result, Master& master);

    /**
     * @brief Cancel any pending operation.
     *
     * Called by the reactor on shutdown or cancellation. Transitions
     * to FAILED if not already finished.
     */
    void cancel();

    /// Number of word-pairs successfully read so far.
    uint16_t wordsRead() const { return words_read_; }

    /// Total number of word-pairs to read.
    uint16_t totalWords() const { return total_words_; }

    /// Current EEPROM word address being read (even).
    uint16_t currentWord() const { return current_word_; }

    /**
     * @brief Skip word-pairs that are already in the SII cache.
     *
     * Called by the reactor after init() and after each word-pair is
     * read. Advances current_word_ past any cached word-pairs (e.g.
     * those prefetched by initSlaves()), avoiding redundant bus reads.
     *
     * @param master  The master (for accessing the per-slave SII cache).
     */
    void skipCachedWords(Master& master);

private:
    uint16_t     slave_index_{0};
    uint16_t     current_word_{0};    ///< Current EEPROM word address (even)
    uint16_t     total_words_{0};     ///< Total word-pairs to read
    uint16_t     words_read_{0};      ///< Word-pairs completed
    int          nack_count_{0};      ///< NACK retries for current word
    int          busy_polls_{0};      ///< Busy-poll iterations for current step

    EEPROMState  state_{EEPROMState::IDLE};
    bool         in_flight_{false};   ///< True when a datagram is pending
    size_t       slot_{0};            ///< Router slot for current datagram
    RxDatagram   response_{};         ///< Response buffer (owned, stable address)

    /// Write payload for EEPADDR (kept as member to avoid stack-lifetime issues).
    uint16_t     eepaddr_payload_{0};
    /// Write payload for EEPCTL (kept as member).
    uint16_t     eepctl_payload_{0};

    /// Transition to FAILED state.
    void fail() { state_ = EEPROMState::FAILED; in_flight_ = false; }

    /// Advance to the next word-pair or DONE.
    void advanceWord();
};

// ============================================================================
// EEPROMReactor — single-worker concurrent multi-slave EEPROM reader
// ============================================================================

/**
 * @brief Single-worker reactor that reads EEPROM from multiple slaves
 *        concurrently using the packet router's `waitForAny()` primitive.
 *
 * @details
 * The reactor owns one `EEPROMReadStateMachine` per slave. It runs a
 * single-threaded event loop:
 *
 * 1. **Issue phase**: For each state machine that `needsSend()`, allocate
 *    a transaction index, pre-register a router slot, build the datagram,
 *    and add it to a batch frame.
 * 2. **Send phase**: Send all pending datagrams in one frame via
 *    `Master::sendMultiDatagram()`.
 * 3. **Wait phase**: Call `TransactionRouter::waitForAny()` with all
 *    in-flight slot indices. This blocks until any datagram completes.
 * 4. **Dispatch phase**: Route the completed response to the corresponding
 *    state machine's `onComplete()`, which advances it to its next state.
 * 5. Repeat until all state machines are `isFinished()`.
 *
 * The reactor uses **one thread** (the caller's thread) and keeps
 * **multiple datagrams in flight** across different slaves. This
 * overlaps the per-slave EEPROM latency.
 *
 * ## Memory safety
 *
 * - All state machines (and their response buffers) are owned by value
 *   in a `std::vector` inside the reactor. No heap allocation per slave.
 * - Router slots are cancelled before the vector is destroyed.
 * - `waitForAny()` receives only slot indices (integers).
 * - The reactor does not hold any router lock during I/O.
 *
 * ## Usage
 *
 * @code
 *   EEPROMReactor reactor(master);
 *   reactor.addSlave(0, 0x0040, 256);  // slave 0, 256 word-pairs from 0x0040
 *   reactor.addSlave(1, 0x0040, 256);  // slave 1, same range
 *   reactor.run();                      // blocks until all done
 * @endcode
 *
 * After `run()` completes, the EEPROM words are in each slave's
 * `SIISlaveCache`, accessible via `master.slave(i).sii().cache()`.
 */
class EEPROMReactor {
public:
    /**
     * @brief Construct a reactor bound to a master.
     * @param master  The EtherCAT master (must outlive the reactor).
     */
    explicit EEPROMReactor(Master& master);

    /**
     * @brief Destructor: cancels all pending router slots.
     *
     * Safe to call even if `run()` was never called or is in progress
     * (though `run()` should not be called concurrently with destruction).
     */
    ~EEPROMReactor();

    EEPROMReactor(const EEPROMReactor&) = delete;
    EEPROMReactor& operator=(const EEPROMReactor&) = delete;
    EEPROMReactor(EEPROMReactor&&) = delete;
    EEPROMReactor& operator=(EEPROMReactor&&) = delete;

    /**
     * @brief Add a slave to the reactor's read set.
     *
     * Must be called before `run()`. Each added slave will have its
     * EEPROM read concurrently with all other added slaves.
     *
     * @param slave_index     Slave position on the bus (0-based).
     * @param start_word      First EEPROM word address (will be aligned to even).
     * @param word_pair_count Number of 32-bit word-pairs to read.
     */
    void addSlave(uint16_t slave_index, uint16_t start_word,
                  uint16_t word_pair_count);

    /**
     * @brief Run the reactor event loop until all slaves are done.
     *
     * Blocks the calling thread. Issues datagrams for all slaves
     * concurrently, waits for completions via `waitForAny()`, and
     * dispatches results to the per-slave state machines.
     *
     * @param timeout_ms  Per-wait timeout in milliseconds (default 500).
     * @return true if all slaves completed successfully, false if any
     *         slave failed or timed out.
     */
    bool run(uint32_t timeout_ms = 500);

    /**
     * @brief Number of slaves that completed successfully.
     */
    size_t successCount() const { return success_count_; }

    /**
     * @brief Number of slaves that failed.
     */
    size_t failureCount() const { return failure_count_; }

    /**
     * @brief Get the state machine for a slave (for diagnostics).
     * @param i  Index in the add order (not slave_index).
     */
    const EEPROMReadStateMachine& stateMachine(size_t i) const {
        return state_machines_[i];
    }

    /// Number of state machines (slaves) in the reactor.
    size_t slaveCount() const { return state_machines_.size(); }

private:
    Master* master_{nullptr};
    std::vector<EEPROMReadStateMachine> state_machines_;
    size_t success_count_{0};
    size_t failure_count_{0};

    /// Cancel all pending router slots (called on destruction and error).
    void cancelAllPending();
};

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
