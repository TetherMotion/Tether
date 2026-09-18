#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/**
 * @brief Reads the ESC211 RSAP information objects 0x4000-0x4023 via SDO.
 *
 * The 0x4000 range is the live safety-application status interface (also
 * exposed cyclically as TxPDO 0x1A02 "TxPDO-Map-RSAP-Info", ESI v0.9:
 * 46 entries / 55 bytes).  Object layout per ESI v0.9:
 *
 *   0x4000  RSAP state                        (USINT, scalar)
 *   0x4001  Sub state in MONITORING state     (USINT, scalar)
 *   0x4002  Error code                        (INT,   scalar)
 *   0x4003  Operation mode                    (USINT, scalar)
 *   0x4004  Stop state                        (USINT, scalar)
 *   0x4005  Stop category                     (USINT, scalar)
 *   0x4006  Fault and violation status        (USINT, bitfield)
 *   0x4007  Drive status                      (USINT, bitfield)
 *   0x4008:1-5  Drive valid status            (USINT, bitfield each)
 *   0x4009:1-7  Drive N user defined bit      (USINT, bitfield each)
 *   0x400A  Input discrepancy status          (UINT,  bitfield)
 *   0x400B-0x4014  Input states (emergency stop, normal stop, protective
 *             stop, enabling device, operation mode, reset, collaborative,
 *             HGC, monitored position, SSM)   (USINT, bitfield each)
 *   0x4015  Output discrepancy status         (UINT,  bitfield)
 *   0x4016  Safety output state               (UINT,  bitfield)
 *   0x4017  Safety control function state     (UINT,  bitfield)
 *   0x4018  Safety limit function state       (UINT,  bitfield)
 *   0x4019-0x4023  Limit statuses (axis/TCP/endpoint position, speed,
 *             torque/force, TCP0 orientation, robot power)  (bitfield)
 *
 * For the bitfield objects each bit represents one configured element
 * (input channel, output channel, limiting function, axis, ...); a set bit
 * means that element's state is currently triggered/active.  The scalar
 * objects carry plain values; a non-zero 0x4002 error code counts as
 * triggered.
 *
 * Typical use (mirrors SystemErrorManager's poll/publish split):
 * @code
 *   RSAPInformationProcessor rsap(slave);
 *   // in a ~3 Hz background thread:
 *   rsap.poll(slave);
 *   auto snap = rsap.snapshot();
 *   for (const auto& line : RSAPInformationProcessor::decodeTriggered(snap))
 *       log(line);
 * @endcode
 */
class RSAPInformationProcessor {
public:
    /// Object dictionary metadata for one 0x40xx (sub)object.
    struct ObjectInfo {
        uint16_t    index;    ///< CoE index (0x4000..0x4023)
        uint8_t     subindex; ///< CoE subindex (0, or 1..n for records)
        const char* name;     ///< ESI object name
        uint8_t     bytes;    ///< 1 (USINT) or 2 (UINT/INT)
        bool        scalar;   ///< true = plain value, false = bitfield
    };

    /// All RSAP information objects, in index order.
    static constexpr size_t kObjectCount = 46;
    static const std::array<ObjectInfo, kObjectCount> kObjects;

    /// Copyable snapshot of one poll cycle — safe to share across threads.
    struct Snapshot {
        std::array<uint32_t, kObjectCount> value{};  ///< raw value per object
        std::array<bool,     kObjectCount> ok{};     ///< SDO read succeeded
        size_t   read_ok = 0;    ///< objects read successfully this poll
        uint64_t generation = 0; ///< incremented on every poll()
    };

    explicit RSAPInformationProcessor(EtherCAT::Slave& slave);

    RSAPInformationProcessor(const RSAPInformationProcessor&) = delete;
    RSAPInformationProcessor& operator=(const RSAPInformationProcessor&) = delete;

    /**
     * @brief Read all 46 RSAP information objects via SDO.
     *
     * Reads are best-effort: an object that fails is marked !ok and
     * polling continues.  If pause() was called, the poll aborts between
     * objects — objects not attempted keep their previous values and are
     * not counted in read_ok.  Safe to call from any thread; may be
     * called concurrently with snapshot()/decodeTriggered().
     *
     * @return number of objects read successfully (0..kObjectCount)
     */
    size_t poll();

    /// Copy of the latest snapshot (empty generation until first poll).
    Snapshot snapshot() const;

    /**
     * @brief Pause/resume polling.
     *
     * While paused, poll() still runs but aborts before issuing any
     * (further) SDO reads.  Use during safety state machine command
     * sequences (0xF100) to avoid SDO mailbox conflicts — same reason
     * SystemErrorManager is stopped there.
     */
    void pause()  { paused_.store(true,  std::memory_order_relaxed); }
    void resume() { paused_.store(false, std::memory_order_relaxed); }
    bool isPaused() const { return paused_.load(std::memory_order_relaxed); }

    /// True if any bitfield object has a set bit or the error code != 0.
    static bool anyTriggered(const Snapshot& snap);

    /**
     * @brief Decode a snapshot into human-readable lines.
     *
     * One line per object: bitfields show the set bit indices
     * ("… State: 0x05 [bits 0,2]"), scalars show the plain value.
     * Only objects that were read successfully appear.
     */
    static std::vector<std::string> decodeAll(const Snapshot& snap);

    /**
     * @brief Decode only the triggered states — the objects whose value
     *        indicates an active/triggered element.
     *
     * Bitfields: one summary line per non-zero object, listing each set bit
     * as "Name(bit)" (or "bit N" when no label is known), e.g.
     * "0x400B:0 Emergency stop input state [BasicEmergencyStopState(0)]".
     * Scalars: only 0x4002 Error code counts as triggered when non-zero.
     * Empty when nothing is triggered.
     */
    static std::vector<std::string> decodeTriggered(const Snapshot& snap);

private:
    EtherCAT::Slave& slave_;
    mutable std::mutex mtx_;
    Snapshot snap_;
    std::atomic<bool> paused_{false};
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
