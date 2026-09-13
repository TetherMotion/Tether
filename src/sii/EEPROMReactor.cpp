/**
 * @file EEPROMReactor.cpp
 * @brief Implementation of the per-slave EEPROM state-machine reactor
 */

#include "tether/sii/EEPROMReactor.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/sii/SIIManager.hpp"
#include "ethercat/raw/internal.hpp"

#include <algorithm>
#include <cstring>

namespace EtherCAT {
namespace SII {

static const char* TAG = "eeprom_reactor";

// ============================================================================
// EEPROMReadStateMachine
// ============================================================================

void EEPROMReadStateMachine::init(uint16_t slave_index, uint16_t start_word,
                                   uint16_t word_pair_count) {
    slave_index_  = slave_index;
    current_word_ = start_word & 0xFFFEu;  // Align to even
    total_words_  = word_pair_count;
    words_read_   = 0;
    nack_count_   = 0;
    busy_polls_   = 0;
    state_        = EEPROMState::IDLE;
    slot_         = 0;
    response_     = RxDatagram{};

    // Start in WRITE_EEPADDR (the first protocol step).
    // The IDLE state is just the initial state; the reactor's issue
    // phase will pick up any state machine that needsSend().
    if (word_pair_count > 0) {
        state_ = EEPROMState::WRITE_EEPADDR;
    } else {
        state_ = EEPROMState::DONE;
    }
}

MultiDatagramSpec EEPROMReadStateMachine::buildDatagram(uint8_t idx) {
    MultiDatagramSpec spec{};
    spec.idx       = idx;
    spec.roundtrip = true;

    const uint16_t adp = Master::adpForSlaveIndex(slave_index_);
    // For auto-increment addressing: adp = 0 - slave_index
    // SlaveAddress(slave_index) maps to adp = 0 - slave_index for physical.
    // But sendMultiDatagram takes raw adp, so we compute it directly.
    // Actually, MultiDatagramSpec.adp is the raw ADP field.
    // For APRD/APWR, adp = 0 - slave_index (auto-increment position).
    spec.adp = adp;

    switch (state_) {
        case EEPROMState::WRITE_EEPADDR: {
            // Write the EEPROM word address to 0x0504 (APWR, 2 bytes)
            spec.cmd      = Command::APWR;
            spec.ado      = EEPROMProtocol::REG_EEPADDR;
            eepaddr_payload_ = Raw::host_to_le16(current_word_);
            spec.data     = &eepaddr_payload_;
            spec.datalen  = sizeof(eepaddr_payload_);
            break;
        }
        case EEPROMState::WRITE_EEPCTL_READ: {
            // Write the READ command to 0x0502 (APWR, 2 bytes)
            spec.cmd      = Command::APWR;
            spec.ado      = EEPROMProtocol::REG_EEPCTL;
            eepctl_payload_ = Raw::host_to_le16(EEPROMProtocol::ECMD_READ);
            spec.data     = &eepctl_payload_;
            spec.datalen  = sizeof(eepctl_payload_);
            break;
        }
        case EEPROMState::POLL_EEPSTAT: {
            // Read EEPSTAT from 0x0502 (APRD, 2 bytes)
            spec.cmd      = Command::APRD;
            spec.ado      = EEPROMProtocol::REG_EEPSTAT;
            spec.data     = nullptr;
            spec.datalen  = 2;
            break;
        }
        case EEPROMState::READ_EEPDAT: {
            // Read 32-bit data from 0x0508 (APRD, 4 bytes)
            spec.cmd      = Command::APRD;
            spec.ado      = EEPROMProtocol::REG_EEPDAT;
            spec.data     = nullptr;
            spec.datalen  = 4;
            break;
        }
        default:
            // Should not be called in IDLE/DONE/FAILED
            spec.cmd      = Command::APRD;
            spec.ado      = 0;
            spec.datalen  = 0;
            break;
    }

    return spec;
}

void EEPROMReadStateMachine::onComplete(const WaitResult& result, Master& master) {
    if (!result.success) {
        // Timeout or failure
        fail();
        return;
    }

    switch (state_) {
        case EEPROMState::WRITE_EEPADDR:
            // EEPADDR written successfully → issue READ command
            state_ = EEPROMState::WRITE_EEPCTL_READ;
            break;

        case EEPROMState::WRITE_EEPCTL_READ:
            // READ command issued → start polling for busy clear
            busy_polls_ = 0;
            state_ = EEPROMState::POLL_EEPSTAT;
            break;

        case EEPROMState::POLL_EEPSTAT: {
            // Check the busy bit in the response
            if (result.data_length < 2) {
                fail();
                return;
            }
            // The response data was copied into response_.data by the router.
            uint16_t estat = Raw::le16_to_host(
                *reinterpret_cast<const uint16_t*>(response_.data));

            if (estat & EEPROMProtocol::ESTAT_BUSY) {
                // Still busy — poll again
                busy_polls_++;
                if (busy_polls_ >= EEPROMProtocol::MAX_BUSY_POLLS) {
                    fail();
                    return;
                }
                // Stay in POLL_EEPSTAT
            } else if (estat & EEPROMProtocol::ESTAT_EMASK) {
                // Error flags set
                if (estat & EEPROMProtocol::ESTAT_NACK) {
                    // NACK — retry the whole read sequence
                    nack_count_++;
                    if (nack_count_ >= EEPROMProtocol::MAX_NACK_RETRIES) {
                        fail();
                        return;
                    }
                    // Go back to WRITE_EEPADDR to retry
                    state_ = EEPROMState::WRITE_EEPADDR;
                } else {
                    // Non-NACK error (CRC, loading, write) — fail
                    fail();
                }
            } else {
                // Not busy, no errors → read the data
                state_ = EEPROMState::READ_EEPDAT;
            }
            break;
        }

        case EEPROMState::READ_EEPDAT: {
            // Extract the 32-bit data and cache it
            if (result.data_length < 4) {
                fail();
                return;
            }
            uint32_t dword_le = 0;
            std::memcpy(&dword_le, response_.data, 4);
            uint32_t dword = Raw::le32_to_host(dword_le);

            // Cache the two words in the slave's per-slave SII cache
            auto& sii = master.slave(slave_index_).sii();
            sii.cache().set(current_word_,
                            static_cast<uint16_t>(dword & 0xFFFF));
            sii.cache().set(static_cast<uint16_t>(current_word_ + 1),
                            static_cast<uint16_t>((dword >> 16) & 0xFFFF));

            words_read_++;
            nack_count_ = 0;
            advanceWord();
            break;
        }

        default:
            // Should not happen
            fail();
            break;
    }
}

void EEPROMReadStateMachine::advanceWord() {
    current_word_ = static_cast<uint16_t>(current_word_ + 2);
    if (words_read_ >= total_words_) {
        state_ = EEPROMState::DONE;
    } else {
        state_ = EEPROMState::WRITE_EEPADDR;
    }
}

void EEPROMReadStateMachine::cancel() {
    if (!isFinished()) {
        state_ = EEPROMState::FAILED;
    }
}

// ============================================================================
// EEPROMReactor
// ============================================================================

EEPROMReactor::EEPROMReactor(Master& master)
    : master_(&master) {}

EEPROMReactor::~EEPROMReactor() {
    cancelAllPending();
}

void EEPROMReactor::addSlave(uint16_t slave_index, uint16_t start_word,
                              uint16_t word_pair_count) {
    state_machines_.emplace_back();
    state_machines_.back().init(slave_index, start_word, word_pair_count);
}

void EEPROMReactor::cancelAllPending() {
    if (!master_) return;
    auto& router = master_->packetRouter();
    for (auto& sm : state_machines_) {
        if (sm.needsSend() || (!sm.isFinished() && sm.slot() < TransactionRouter::kNumSlots)) {
            router.cancelPreRegistered(sm.slot());
            sm.cancel();
        }
    }
}

bool EEPROMReactor::run(uint32_t timeout_ms) {
    if (!master_ || state_machines_.empty()) return true;

    auto& router = master_->packetRouter();

    // Reusable buffers for the issue phase
    std::vector<MultiDatagramSpec> specs;
    std::vector<uint8_t>           idxs;
    std::vector<size_t>            slots;
    std::vector<size_t>            active_slots;  // slots currently in flight

    specs.reserve(state_machines_.size());
    idxs.reserve(state_machines_.size());
    slots.reserve(state_machines_.size());
    active_slots.reserve(state_machines_.size());

    int loop_count = 0;
    const int max_loops = 100000;  // Safety valve

    while (loop_count++ < max_loops) {
        // Check if all state machines are finished
        bool all_done = true;
        for (const auto& sm : state_machines_) {
            if (!sm.isFinished()) {
                all_done = false;
                break;
            }
        }
        if (all_done) break;

        // ---- Issue phase ----
        // For each state machine that needs to send, allocate an idx,
        // pre-register a router slot, and build the datagram.
        specs.clear();
        idxs.clear();
        slots.clear();
        active_slots.clear();

        for (auto& sm : state_machines_) {
            if (!sm.needsSend()) continue;

            uint8_t idx = master_->allocIdx();
            size_t  slot = master_->preRegisterResponseWaiter(
                idx, sm.response().data, sizeof(sm.response().data));

            if (slot >= TransactionRouter::kNumSlots) {
                // Slot allocation failed — fail this state machine
                sm.cancel();
                continue;
            }

            sm.setSlot(slot);
            slots.push_back(slot);
            active_slots.push_back(slot);
            idxs.push_back(idx);
            specs.push_back(sm.buildDatagram(idx));
        }

        if (specs.empty()) {
            // No datagrams to send — all remaining state machines are
            // either finished or failed.
            continue;
        }

        // ---- Send phase ----
        size_t sent = master_->sendMultiDatagram(specs.data(), specs.size());
        if (sent == 0) {
            // Send failed — cancel all pre-registered slots and fail
            // the corresponding state machines.
            for (size_t i = 0; i < slots.size(); ++i) {
                router.cancelPreRegistered(slots[i]);
            }
            for (auto& sm : state_machines_) {
                if (sm.needsSend()) sm.cancel();
            }
            continue;
        }

        // ---- Wait + Dispatch phase ----
        // Wait for all in-flight datagrams to complete, dispatching
        // each completion to its state machine. We loop here because
        // waitForAny() returns one completion at a time.
        while (!active_slots.empty()) {
            auto any = router.waitForAny(active_slots.data(),
                                          active_slots.size(), timeout_ms);
            if (any.timed_out) {
                // Timeout — cancel all remaining in-flight slots
                for (size_t s : active_slots) {
                    router.cancelPreRegistered(s);
                }
                // Fail all state machines that still need to send
                for (auto& sm : state_machines_) {
                    if (sm.needsSend()) sm.cancel();
                }
                active_slots.clear();
                break;
            }

            // Dispatch the completed slot to its state machine.
            // any.slot_index is the index into active_slots, which
            // corresponds to the state machine at the same position
            // in the issue phase. We need to find which state machine
            // owns this slot.
            size_t completed_slot = active_slots[any.slot_index];

            // Find the state machine that owns this slot
            for (auto& sm : state_machines_) {
                if (sm.slot() == completed_slot && sm.needsSend()) {
                    sm.onComplete(any.result, *master_);
                    break;
                }
            }

            // Remove the completed slot from active_slots
            active_slots[any.slot_index] = active_slots.back();
            active_slots.pop_back();
        }
    }

    // ---- Tally results ----
    for (const auto& sm : state_machines_) {
        if (sm.state() == EEPROMState::DONE) {
            success_count_++;
        } else {
            failure_count_++;
        }
    }

    return failure_count_ == 0;
}

} // namespace SII
} // namespace EtherCAT
