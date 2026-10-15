#pragma once

/**
 * @file SdoObjectPoller.hpp
 * @brief Shared poll/snapshot/pause engine for SDO object-table pollers.
 *
 * Device status interfaces are often exposed as a flat table of CoE
 * objects (indices + optional record sub-indices) that a background
 * thread reads cyclically via SDO — best effort, keeping last known
 * values for entries that fail, and pausable so no SDO read overlaps a
 * mailbox-sensitive command sequence.
 *
 * `SdoObjectPoller` owns that mechanics half (previously hand-rolled
 * identically in ESC211::RSAPInformationProcessor and
 * ESC211::RSAPMonitoringProcessor): the caller supplies the
 * `SdoObjectEntry` table and an optional name lookup for error logging;
 * `poll()` carries previous values forward, delegates the read loop to
 * `pollSdoObjects`, publishes a copyable `SdoPollSnapshot` (raw value +
 * ok flag + read_ok + generation per entry), and honours
 * `pause()`/`resume()` between objects.
 *
 * Semantics:
 *  - Entries not attempted (paused poll, earlier failure) keep their
 *    previous value and ok flag — `generation` still increments.
 *  - `poll()` is safe to call concurrently with `snapshot()`.
 *
 * Typical derived class (statics + decode live in the subclass; the
 * poller only knows index/subindex/bytes):
 * @code
 *   inline constexpr size_t kMyPollObjects = 3;
 *   class MyPoller : public SdoObjectPoller<kMyPollObjects> {
 *   public:
 *       using Snapshot = SdoPollSnapshot<kObjectCount>;
 *       static constexpr size_t kObjectCount = kMyPollObjects;
 *       struct ObjectInfo { uint16_t index; uint8_t subindex;
 *                           const char* name; };
 *       static const std::array<ObjectInfo, kObjectCount> kObjects;
 *       explicit MyPoller(Slave& s)
 *           : SdoObjectPoller(s, kSdoEntries,
 *                             +[](size_t i) { return kObjects[i].name; },
 *                             "MyPoller") {}
 *   private:
 *       static const std::array<SdoObjectEntry, kObjectCount> kSdoEntries;
 *   };
 * @endcode
 *
 * Snapshots with extra accessors: pass a `SdoPollSnapshot<N>` subclass
 * as the second template parameter and alias it back inside the class
 * (`using Snapshot = MySnapshot;`), e.g. `RSAPMonitoringProcessor`'s
 * `get(index, sub)`/`getSigned` lookup helpers.
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <type_traits>

#include "logging/Logger.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {

/// Raw snapshot storage of one poll cycle — one slot per table entry,
/// copyable and safe to share across threads.  Derived structs may add
/// accessor methods (see SdoObjectPoller's SnapshotT parameter).
template <size_t N>
struct SdoPollSnapshot {
    std::array<uint32_t, N> value{};  ///< raw value per object
    std::array<bool,     N> ok{};     ///< SDO read succeeded
    size_t   read_ok    = 0;          ///< objects read this poll
    uint64_t generation = 0;          ///< incremented on every poll()
};

/**
 * @brief Cyclic best-effort SDO poller over a fixed object table.
 *
 * @tparam N         number of table entries
 * @tparam SnapshotT published snapshot type — must derive from
 *         `SdoPollSnapshot<N>` (add accessors by subclassing).
 */
template <size_t N, typename SnapshotT = SdoPollSnapshot<N>>
class SdoObjectPoller {
public:
    static_assert(std::is_base_of_v<SdoPollSnapshot<N>, SnapshotT>,
                  "SnapshotT must derive from SdoPollSnapshot<N>");

    using Snapshot  = SnapshotT;
    /// Maps a table position to a display name for error logging.
    /// Pass `nullptr` to log "?" names.
    using NameOfFn  = const char* (*)(size_t);

    /**
     * @param entries  poll table — must outlive the poller (a static
     *                 table built alongside the ObjectInfo list is the
     *                 intended shape).
     */
    SdoObjectPoller(Slave& slave,
                    const std::array<SdoObjectEntry, N>& entries,
                    NameOfFn name_of = nullptr,
                    const char* log_tag = "SdoObjectPoller")
        : slave_(slave), entries_(entries),
          name_of_(name_of), tag_(log_tag) {}

    SdoObjectPoller(const SdoObjectPoller&) = delete;
    SdoObjectPoller& operator=(const SdoObjectPoller&) = delete;

    /**
     * @brief Read every table entry via SDO (best effort).
     *
     * A failed read marks the slot !ok and polling continues; while
     * paused() the poll aborts between objects — entries not attempted
     * keep previous values and are not counted in read_ok.
     *
     * @return number of objects read successfully (0..N)
     */
    size_t poll() {
        // Carry previous values forward so objects not attempted in a
        // paused or partially-failed poll keep their last known state.
        Snapshot next;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            next = snap_;
        }
        next.generation++;

        next.read_ok = pollSdoObjects(
            slave_, entries_, next.value, next.ok,
            [this] { return paused_.load(std::memory_order_relaxed); },
            [this](size_t i, const SdoObjectEntry& e, SlaveError err) {
                TETHER_LOGD(tag_, "0x{:04X}:{} ({}): SDO read failed ({})",
                            e.index, e.subindex,
                            name_of_ ? name_of_(i) : "?",
                            static_cast<int>(err));
            });

        std::lock_guard<std::mutex> lock(mtx_);
        snap_ = next;
        return next.read_ok;
    }

    /// Copy of the latest snapshot (empty generation until first poll).
    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return snap_;
    }

    /**
     * @brief Pause/resume polling.
     *
     * While paused, poll() still runs but aborts before issuing any
     * (further) SDO reads — use to keep status polls from overlapping
     * mailbox-sensitive command sequences (e.g. ESC211 0xF100 commands).
     */
    void pause()  { paused_.store(true,  std::memory_order_relaxed); }
    void resume() { paused_.store(false, std::memory_order_relaxed); }
    bool isPaused() const {
        return paused_.load(std::memory_order_relaxed);
    }

private:
    Slave& slave_;
    const std::array<SdoObjectEntry, N>& entries_;
    NameOfFn name_of_;
    const char* tag_;
    mutable std::mutex mtx_;
    Snapshot snap_;
    std::atomic<bool> paused_{false};
};

} // namespace EtherCAT
