/**
 * @file SlaveGroup.hpp
 * @brief Single-packet EtherCAT state (AL) control and query for slave groups
 *
 * A SlaveGroup binds a Master to a fixed set of slave indices — or to the
 * whole bus in broadcast mode — and drives the Application Layer state
 * machine of the whole set with a single Ethernet packet per operation:
 *
 *  - broadcast group → ONE BWR datagram to AL_CONTROL (0x0120)
 *  - index group     → ONE Ethernet frame carrying one APWR datagram per
 *                      slave
 *  - state query     → ONE Ethernet frame carrying two APRD datagrams per
 *                      slave (AL_STATUS 0x0130 + AL_STATUS_CODE 0x0134)
 *
 * The heavy lifting (packing multiple datagrams into one frame, response
 * demultiplexing) is provided by Master::sendMultiDatagram() and the
 * readRegistersBatch()/writeRegistersBatch() APIs.
 *
 * Typical multi-drive bring-up:
 * @code
 *   // ... per-slave mailbox / PDO configuration, all slaves in SAFE_OP ...
 *   EtherCAT::SlaveGroup actuators(master, {2, 3, 4});
 *   actuators.requestState(EtherCAT::SlaveState::OP, /*ack_error=* /true);
 *   if (!actuators.waitForState(EtherCAT::SlaveState::OP, 5000)) { ... }
 *
 *   // Whole bus variant — writes go out as a single BWR datagram:
 *   auto all = EtherCAT::SlaveGroup::all(master);
 *   all.requestState(EtherCAT::SlaveState::INIT);
 * @endcode
 */

#pragma once

#include <cstdint>
#include <initializer_list>
#include <span>
#include <vector>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

class Master;

/**
 * @brief Per-slave AL status snapshot returned by SlaveGroup::readStates().
 */
struct SlaveGroupState {
    uint16_t slave_index    = 0;
    bool     responded      = false;  ///< Both datagrams answered (WKC>0)
    uint16_t wkc            = 0;      ///< OR of both datagrams' working counters
    uint16_t al_status      = 0;      ///< Raw AL_STATUS register (0x0130)
    uint16_t al_status_code = 0;      ///< AL_STATUS_CODE register (0x0134)

    /// AL state (low nibble of AL_STATUS).
    SlaveState state() const {
        return static_cast<SlaveState>(al_status & 0x000F);
    }
    /// AL error flag (bit 4 of AL_STATUS).
    bool errorFlag() const { return (al_status & 0x0010) != 0; }
};

/**
 * @brief A fixed set of slaves driven/queried together, one packet per call.
 *
 * Holds no wire state; every method is a self-contained transaction
 * (allocate idx, pre-register waiters, send one frame, collect responses).
 */
class SlaveGroup {
public:
    /// Group over an explicit set of bus indices — writes use one
    /// APWR-per-slave multi-datagram frame.
    SlaveGroup(Master& master, std::span<const uint16_t> slave_indices)
        : master_(master), indices_(slave_indices.begin(), slave_indices.end()) {}

    SlaveGroup(Master& master, std::initializer_list<uint16_t> slave_indices)
        : SlaveGroup(master, std::span<const uint16_t>(
                                 slave_indices.begin(), slave_indices.size())) {}

    SlaveGroup(Master& master, const std::vector<uint16_t>& slave_indices)
        : SlaveGroup(master, std::span<const uint16_t>(slave_indices)) {}

    /// Whole-bus group: requestState*() emits a single BWR datagram;
    /// readStates() still queries every discovered slave individually
    /// (broadcast reads return only one merged result).
    static SlaveGroup all(Master& master);

    /// Whole-bus group with addressed writes: one APWR datagram per
    /// discovered slave in a single frame — useful when a broadcast write
    /// is undesirable (e.g. slaves that must not see the request).
    static SlaveGroup allAddressed(Master& master);

    // ---- Group description -------------------------------------------------

    const std::vector<uint16_t>& indices() const { return indices_; }
    size_t size()  const { return indices_.size(); }
    bool   empty() const { return indices_.empty(); }
    /// True when created via all() — writes go out as one BWR datagram.
    bool   isBroadcast() const { return broadcast_; }

    /// Build the AL_CONTROL byte: requested state | optional error-ack bit.
    static constexpr uint8_t controlByte(SlaveState target, bool ack_error) {
        return static_cast<uint8_t>(target) | (ack_error ? 0x10u : 0x00u);
    }

    // ---- State requests (single EtherCAT packet) --------------------------

    /**
     * @brief Write AL_CONTROL to every group member with ONE packet.
     *
     * Broadcast groups send a single BWR datagram; index groups send one
     * Ethernet frame carrying one APWR per member.
     *
     * @param al_control  AL_CONTROL value (state nibble | flags, e.g.
     *                    controlByte(SlaveState::OP, true))
     * @param timeout_ms  Response timeout
     * @return Number of slaves that acknowledged the write (BWR working
     *         counter / count of acknowledged datagrams), 0 on failure.
     */
    uint16_t requestAlControl(uint8_t al_control, uint32_t timeout_ms = 200);

    /// Convenience wrapper: request @p target on the whole group.
    uint16_t requestState(SlaveState target, bool ack_error = false,
                          uint32_t timeout_ms = 200) {
        return requestAlControl(controlByte(target, ack_error), timeout_ms);
    }

    // ---- State query (single EtherCAT packet) -----------------------------

    /**
     * @brief Read AL_STATUS + AL_STATUS_CODE of every member in ONE frame.
     *
     * Two APRD datagrams per slave (0x0130 and 0x0134, 2 bytes each) are
     * packed into a single Ethernet frame, so a whole-group status poll is
     * one wire round-trip.
     */
    std::vector<SlaveGroupState> readStates(uint32_t timeout_ms = 200);

    // ---- Convenience ------------------------------------------------------

    /**
     * @brief Poll readStates() until every member reports @p target.
     *
     * One Ethernet packet per poll.  Aborts early on cancellation and when
     * a slave signals an AL error (error flag + status code are logged).
     *
     * @param resend_interval_ms  If > 0, re-issue requestState() every
     *        interval while waiting — some slaves need repeated requests.
     * @return true when all members reached @p target.
     */
    bool waitForState(SlaveState target, uint32_t timeout_ms,
                      uint32_t poll_interval_ms = 10,
                      uint32_t resend_interval_ms = 0);

private:
    Master& master_;
    std::vector<uint16_t> indices_;
    bool broadcast_ = false;
};

} // namespace EtherCAT
