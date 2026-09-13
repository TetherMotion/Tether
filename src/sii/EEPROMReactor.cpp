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
    in_flight_    = false;
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

void EEPROMReadStateMachine::skipCachedWords(Master& master) {
    // Skip word-pairs that are already in the SII cache (e.g. from
    // initSlaves()'s 128-word prefetch). This avoids re-reading words
    // that are already available, reducing bus traffic significantly.
    //
    // Uses cachedContiguousFrom() to find the contiguous cached range
    // from the current word in a single lock acquisition, then skips
    // whole word-pairs at once.
    if (state_ != EEPROMState::WRITE_EEPADDR) return;

    auto& cache = master.slave(slave_index_).sii().cache();

    while (words_read_ < total_words_) {
        uint16_t cached = cache.cachedContiguousFrom(current_word_);
        if (cached < 2) break;  // Not enough for a word-pair

        // Skip whole word-pairs from the contiguous cached range.
        // Each word-pair is 2 words.
        uint16_t pairs_to_skip = cached / 2;
        uint16_t pairs_remaining = static_cast<uint16_t>(
            total_words_ - words_read_);
        if (pairs_to_skip > pairs_remaining)
            pairs_to_skip = pairs_remaining;

        current_word_ = static_cast<uint16_t>(
            current_word_ + pairs_to_skip * 2);
        words_read_ = static_cast<uint16_t>(words_read_ + pairs_to_skip);
    }

    if (words_read_ >= total_words_) {
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
    in_flight_ = false;  // Datagram completed — no longer in flight

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

            // Cache the word-pair atomically (single lock acquisition)
            master.slave(slave_index_).sii().cache().setWordPair(
                current_word_, dword);

            words_read_++;
            nack_count_ = 0;
            advanceWord();
            // After advancing, skip any subsequent cached word-pairs
            skipCachedWords(master);
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
    in_flight_ = false;
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
        if (sm.isInFlight() || (!sm.isFinished() && sm.slot() < TransactionRouter::kNumSlots)) {
            router.cancelPreRegistered(sm.slot());
            sm.cancel();
        }
    }
}

bool EEPROMReactor::run(uint32_t timeout_ms) {
    if (!master_ || state_machines_.empty()) return true;

    auto& router = master_->packetRouter();

    // Skip any already-cached word-pairs before starting.
    for (auto& sm : state_machines_) {
        sm.skipCachedWords(*master_);
    }

    // Track which slots are in flight, mapped to state machine indices.
    // slot_to_sm_[slot] = index into state_machines_, or SIZE_MAX if unused.
    std::vector<size_t> slot_to_sm_(TransactionRouter::kNumSlots, SIZE_MAX);
    std::vector<size_t> active_slots;      // slots currently in flight
    std::vector<MultiDatagramSpec> specs;  // batch of datagrams to send
    std::vector<uint8_t>           idxs;   // transaction indices
    std::vector<size_t>            slots;  // router slots for current batch

    active_slots.reserve(state_machines_.size());
    specs.reserve(state_machines_.size());
    idxs.reserve(state_machines_.size());
    slots.reserve(state_machines_.size());

    int loop_count = 0;
    const int max_loops = 500000;  // Safety valve

    while (loop_count++ < max_loops) {
        // ---- Issue phase ----
        // For each state machine that needs a send and has no datagram
        // in flight, allocate an idx, pre-register a router slot, and
        // build the datagram. This runs every iteration so that a slave
        // which just completed a step can immediately issue its next
        // step without waiting for other slaves.
        specs.clear();
        idxs.clear();
        slots.clear();

        for (size_t i = 0; i < state_machines_.size(); ++i) {
            auto& sm = state_machines_[i];
            if (!sm.needsSend()) continue;

            uint8_t idx = master_->allocIdx();
            size_t  slot = master_->preRegisterResponseWaiter(
                idx, sm.response().data, sizeof(sm.response().data));

            if (slot >= TransactionRouter::kNumSlots) {
                sm.cancel();
                continue;
            }

            sm.setSlot(slot);
            sm.markInFlight();
            slot_to_sm_[slot] = i;
            active_slots.push_back(slot);
            slots.push_back(slot);
            idxs.push_back(idx);
            specs.push_back(sm.buildDatagram(idx));
        }

        // ---- Send phase ----
        if (!specs.empty()) {
            size_t sent = master_->sendMultiDatagram(specs.data(), specs.size());
            if (sent == 0) {
                for (size_t s : slots) {
                    router.cancelPreRegistered(s);
                    slot_to_sm_[s] = SIZE_MAX;
                }
                for (auto& sm : state_machines_) {
                    if (sm.isInFlight()) sm.cancel();
                }
                // Remove failed slots from active_slots
                for (auto it = active_slots.begin(); it != active_slots.end(); ) {
                    if (slot_to_sm_[*it] == SIZE_MAX)
                        it = active_slots.erase(it);
                    else
                        ++it;
                }
            }
        }

        // ---- Check if all done ----
        if (active_slots.empty()) {
            bool all_done = true;
            for (const auto& sm : state_machines_) {
                if (!sm.isFinished()) { all_done = false; break; }
            }
            if (all_done) break;
            // No in-flight datagrams but not all done — shouldn't happen,
            // but continue to let issue phase pick up any stragglers.
            continue;
        }

        // ---- Wait for one completion ----
        auto any = router.waitForAny(active_slots.data(),
                                      active_slots.size(), timeout_ms);
        if (any.timed_out) {
            for (size_t s : active_slots) {
                router.cancelPreRegistered(s);
                slot_to_sm_[s] = SIZE_MAX;
            }
            for (auto& sm : state_machines_) {
                if (!sm.isFinished()) sm.cancel();
            }
            active_slots.clear();
            break;
        }

        // ---- Dispatch the completed slot ----
        size_t completed_slot = active_slots[any.slot_index];
        size_t sm_idx = slot_to_sm_[completed_slot];
        slot_to_sm_[completed_slot] = SIZE_MAX;

        if (sm_idx < state_machines_.size()) {
            state_machines_[sm_idx].onComplete(any.result, *master_);
        }

        // Remove the completed slot from active_slots (swap-and-pop)
        active_slots[any.slot_index] = active_slots.back();
        active_slots.pop_back();

        // Loop back to issue phase — the completed slave may now
        // needSend() for its next protocol step, and it will be
        // issued in the next frame alongside any other slaves that
        // are ready. This is the pipelining: different slaves can
        // be at different protocol steps simultaneously.
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
