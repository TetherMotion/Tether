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
 * ## Solution: demand-driven reactor
 *
 * The reactor uses a `SIIDemandParser` per slave to determine exactly which
 * EEPROM word-pairs are needed. It then reads those word-pairs concurrently
 * across all slaves using the packet router's `waitForAny()` primitive:
 *
 *   1. For each slave, the demand parser is called to get the list of
 *      needed word-pair addresses.
 *   2. The reactor issues bus reads for those word-pairs across all slaves.
 *   3. When a read completes, the result is cached in the slave's
 *      `SIISlaveCache`, and the demand parser is called again to see
 *      if more word-pairs are needed or if parsing is complete.
 *   4. Steps 2–3 repeat until all slaves' parsers report COMPLETE.
 *
 * This approach fetches **only the words that are actually needed** — no
 * fixed-size prefetch, no assumptions about "typical" EEPROM sizes. The
 * demand parser drives the read set; the reactor drives the concurrency.
 *
 * ## Memory safety
 *
 * - Each slave's state machine owns its response buffer (a `RxDatagram`
 *   stored by value in the state machine, not on the caller's stack).
 * - Router slots are cancelled on timeout, error, or reactor shutdown.
 * - The reactor owns all state machines and parsers by value (no heap
 *   allocation per slave, no dangling pointers).
 * - `waitForAny()` receives only slot indices (integers), never pointers.
 *
 * ## Reuse of SII parsing code
 *
 * The reactor delegates all parsing to `SIIDemandParser`, which shares the
 * same category-data parsing functions (`parseStringsFromBuffer`,
 * `parseGeneralFromBuffer`, etc.) with the blocking `SIIParser`. Both the
 * blocking and reactor paths use the same low-level parsing code.
 *
 * @see SIIDemandParser
 * @see TransactionRouter::waitForAny()
 * @see SIIReader
 */

#pragma once

#include "tether/ethercat/TetherConfig.hpp"

#if TETHER_ENABLE_SII

#include "tether/ethercat/TransactionRouter.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/sii/SIIManager.hpp"
#include "tether/sii/SIIDemandParser.hpp"
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
// EEPROMReadStateMachine — per-slave single-word-pair EEPROM read
// ============================================================================

/**
 * @brief States of the per-slave EEPROM read protocol.
 *
 * The state machine reads a **single** 32-bit word-pair from one slave.
 * After completion (DONE), the reactor provides the next word-pair address
 * via `reinit()`. This allows the reactor to drive demand-driven reads
 * where the next address depends on the parsing result.
 *
 * State transitions:
 *
 * @code
 *   IDLE → WRITE_EEPADDR → WRITE_EEPCTL_READ → POLL_EEPSTAT → READ_EEPDAT → DONE
 *                                                                 ↓
 *   (NACK retry) → WRITE_EEPADDR
 *   (busy)       → POLL_EEPSTAT (stay)
 *   (error/timeout) → FAILED
 *
 *   reinit(addr) from DONE → WRITE_EEPADDR
 * @endcode
 */
enum class EEPROMState : uint8_t {
    IDLE,               ///< Not started
    WRITE_EEPADDR,      ///< Writing the EEPROM word address to 0x0504
    WRITE_EEPCTL_READ,  ///< Writing the READ command to 0x0502
    POLL_EEPSTAT,       ///< Reading EEPSTAT (0x0502) to check busy bit
    READ_EEPDAT,        ///< Reading 32-bit data from 0x0508
    DONE,               ///< Word-pair read successfully
    FAILED,             ///< Error or timeout
};

/**
 * @brief Per-slave EEPROM read state machine for a list of word-pairs.
 *
 * Each instance drives the 4-step EEPROM read protocol for one slave,
 * reading a list of 32-bit word-pairs. The state machine is advanced
 * one step at a time by the reactor. After each word-pair completes,
 * the state machine automatically advances to the next address in
 * its vector. When all word-pairs are read, it transitions to DONE.
 *
 * The list of word-pair addresses is stored in a `std::vector<uint16_t>`,
 * provided at `init()` time. The reactor refills this vector by calling
 * `setWordPairs()` when the demand parser reports more needed words.
 *
 * The state machine owns:
 *   - Its response buffer (`RxDatagram`, stored by value — no heap
 *     allocation, no stack-pointer lifetime issues).
 *   - Its router slot index (an integer, safe to pass to `waitForAny()`).
 *   - Its current word address.
 *   - Its vector of pending word-pair addresses.
 *
 * Memory safety: the response buffer is a member of the state machine,
 * which is a member of the reactor. The reactor outlives all router
 * operations. On destruction, the reactor cancels all pending slots
 * before the state machines (and their buffers) are destroyed.
 */
class EEPROMReadStateMachine {
public:
    EEPROMReadStateMachine() = default;

    /**
     * @brief Initialize the state machine with a list of word-pair addresses.
     *
     * @param slave_index  Slave position on the bus (0-based).
     * @param word_pairs   Vector of even EEPROM word addresses to read.
     *                     Each address starts a 2-word (32-bit) read.
     *                     The vector is moved into the state machine.
     */
    void init(uint16_t slave_index, std::vector<uint16_t> word_pairs);

    /**
     * @brief Reset to IDLE with no pending word-pairs.
     */
    void reset();

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
    void markInFlight() { in_flight_ = true; }

    /// Slave index this state machine reads from.
    uint16_t slaveIndex() const { return slave_index_; }

    /// Router slot index for the current in-flight datagram.
    size_t slot() const { return slot_; }

    /// Response buffer (owned by this state machine).
    RxDatagram& response() { return response_; }
    const RxDatagram& response() const { return response_; }

    /// Number of word-pairs remaining to read.
    size_t remaining() const { return word_pairs_.size() - current_index_; }

    /// Number of word-pairs already read.
    size_t wordsRead() const { return current_index_; }

    /// Total number of word-pairs in the current batch.
    size_t totalWords() const { return word_pairs_.size(); }

    /**
     * @brief Build the datagram spec for the current state.
     * @param idx  Transaction index (allocated by the reactor).
     * @return MultiDatagramSpec for the current protocol step.
     */
    MultiDatagramSpec buildDatagram(uint8_t idx);

    /**
     * @brief Set the router slot for the current in-flight datagram.
     */
    void setSlot(size_t slot) { slot_ = slot; }

    /**
     * @brief Process the completion of the current datagram.
     *
     * Called by the reactor when `waitForAny()` reports this state
     * machine's slot has completed. On READ_EEPDAT, caches the result
     * in the slave's SIISlaveCache and advances to the next word-pair
     * (or DONE if all word-pairs have been read).
     *
     * @param result  The WaitResult from the router.
     * @param master  The master (for cache writes).
     */
    void onComplete(const WaitResult& result, Master& master);

    /**
     * @brief Cancel any pending operation.
     */
    void cancel();

    /// Current EEPROM word address being read (even).
    uint16_t currentWord() const {
        return current_index_ < word_pairs_.size()
            ? word_pairs_[current_index_] : 0xFFFF;
    }

private:
    uint16_t     slave_index_{0};
    std::vector<uint16_t> word_pairs_{};  ///< Pending word-pair addresses
    size_t       current_index_{0};       ///< Index into word_pairs_

    int          nack_count_{0};      ///< NACK retries for current word
    int          busy_polls_{0};       ///< Busy-poll iterations for current step

    EEPROMState  state_{EEPROMState::IDLE};
    bool         in_flight_{false};   ///< True when a datagram is pending
    size_t       slot_{0};            ///< Router slot for current datagram
    RxDatagram   response_{};          ///< Response buffer (owned, stable address)

    /// Write payload for EEPADDR (kept as member to avoid stack-lifetime issues).
    uint16_t     eepaddr_payload_{0};
    /// Write payload for EEPCTL (kept as member).
    uint16_t     eepctl_payload_{0};

    /// Transition to FAILED state.
    void fail() { state_ = EEPROMState::FAILED; in_flight_ = false; }

    /// Advance to the next word-pair, or DONE if all read.
    void advanceWord();
};

// ============================================================================
// EEPROMReactor — demand-driven concurrent multi-slave EEPROM reader
// ============================================================================

/**
 * @brief Single-worker reactor that reads EEPROM from multiple slaves
 *        concurrently, driven by demand parsers.
 *
 * @details
 * The reactor owns one `EEPROMReadStateMachine` and one `SIIDemandParser`
 * per slave. It runs a single-threaded event loop:
 *
 * 1. **Demand phase**: For each slave that needs more words, call the
 *    demand parser to get the list of needed word-pair addresses.
 *    If the parser reports COMPLETE, mark the slave as done.
 * 2. **Issue phase**: For each state machine that `needsSend()`, allocate
 *    a transaction index, pre-register a router slot, build the datagram.
 * 3. **Send phase**: Send all pending datagrams in one frame.
 * 4. **Wait phase**: Call `TransactionRouter::waitForAny()` to block until
 *    any datagram completes.
 * 5. **Dispatch phase**: Route the completed response to the corresponding
 *    state machine's `onComplete()`, which caches the result. Then call
 *    the demand parser again to get the next word-pair for that slave.
 * 6. Repeat until all slaves' parsers report COMPLETE.
 *
 * The reactor uses **one thread** (the caller's thread) and keeps
 * **multiple datagrams in flight** across different slaves. This
 * overlaps the per-slave EEPROM latency.
 *
 * ## Usage
 *
 * @code
 *   EEPROMReactor reactor(master);
 *   reactor.addSlave(0, SII::CAT_MASK_ALL);
 *   reactor.addSlave(1, SII::CAT_MASK_ALL);
 *   reactor.run();  // blocks until all done
 *
 *   // Results are in each slave's SIISlaveCache:
 *   SII::SIIData data;
 *   master.slave(0).sii().parseCategories(data, SII::CAT_MASK_ALL);
 * @endcode
 *
 * After `run()` completes, the EEPROM words are in each slave's
 * `SIISlaveCache`, and the parsed SII data is available via
 * `reactor.result(i)`.
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
     */
    ~EEPROMReactor();

    EEPROMReactor(const EEPROMReactor&) = delete;
    EEPROMReactor& operator=(const EEPROMReactor&) = delete;
    EEPROMReactor(EEPROMReactor&&) = delete;
    EEPROMReactor& operator=(EEPROMReactor&&) = delete;

    /**
     * @brief Add a slave to the reactor's read set.
     *
     * Must be called before `run()`. The slave's EEPROM will be read
     * concurrently with all other added slaves, driven by the demand
     * parser which fetches only the words needed for the requested
     * category mask.
     *
     * @param slave_index  Slave position on the bus (0-based).
     * @param cat_mask     Bitmask of SIICategoryMask values selecting
     *                     which SII categories to parse.
     */
    void addSlave(uint16_t slave_index, uint32_t cat_mask);

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

    /// Number of slaves that completed successfully.
    size_t successCount() const { return success_count_; }

    /// Number of slaves that failed.
    size_t failureCount() const { return failure_count_; }

    /**
     * @brief Get the parsed SII data for a slave.
     * @param i  Index in the add order (not slave_index).
     * @return Parsed SII data (valid if run() succeeded for this slave).
     */
    const SIIData& result(size_t i) const { return slaves_[i].out_data; }

    /**
     * @brief Get the state machine for a slave (for diagnostics).
     * @param i  Index in the add order (not slave_index).
     */
    const EEPROMReadStateMachine& stateMachine(size_t i) const {
        return slaves_[i].sm;
    }

    /// Number of slaves in the reactor.
    size_t slaveCount() const { return slaves_.size(); }

private:
    /// Per-slave state: demand parser + state machine.
    struct SlaveEntry {
        uint16_t                  slave_index{0};
        SIIDemandParser           parser{};
        EEPROMReadStateMachine    sm{};
        SIIData                   out_data{};
        bool                      done{false};
    };

    Master* master_{nullptr};
    std::vector<SlaveEntry> slaves_;
    size_t success_count_{0};
    size_t failure_count_{0};

    /// Cancel all pending router slots (called on destruction and error).
    void cancelAllPending();

    /**
     * @brief Refill a slave's word-pair queue from its demand parser.
     *
     * Calls the demand parser to get the next set of needed word-pairs.
     * If the parser reports COMPLETE, marks the slave as done.
     * If the parser reports NEED_WORDS, fills the word-pair queue.
     * If the parser reports FAILED, marks the slave as failed.
     *
     * @param entry  The slave entry to refill.
     * @return true if the queue was filled (or slave is done),
     *         false on failure.
     */
    bool refillQueue(SlaveEntry& entry);
};

} // namespace SII
} // namespace EtherCAT

#endif // TETHER_ENABLE_SII
