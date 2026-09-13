/**
 * @file EEPROMReactor.cpp
 * @brief Implementation of the demand-driven EEPROM reactor
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

void EEPROMReadStateMachine::init(uint16_t slave_index,
                                   std::vector<uint16_t> word_pairs) {
    slave_index_   = slave_index;
    word_pairs_    = std::move(word_pairs);
    current_index_ = 0;
    nack_count_    = 0;
    busy_polls_    = 0;
    slot_          = 0;
    response_      = RxDatagram{};

    if (word_pairs_.empty()) {
        state_ = EEPROMState::DONE;
        in_flight_ = false;
    } else {
        state_     = EEPROMState::WRITE_EEPADDR;
        in_flight_ = false;
    }
}

void EEPROMReadStateMachine::reset() {
    word_pairs_.clear();
    current_index_ = 0;
    nack_count_    = 0;
    busy_polls_    = 0;
    state_         = EEPROMState::IDLE;
    in_flight_     = false;
    slot_          = 0;
    response_      = RxDatagram{};
}

void EEPROMReadStateMachine::advanceWord() {
    current_index_++;
    nack_count_ = 0;
    busy_polls_ = 0;
    response_ = RxDatagram{};

    if (current_index_ >= word_pairs_.size()) {
        state_ = EEPROMState::DONE;
    } else {
        state_ = EEPROMState::WRITE_EEPADDR;
    }
}

MultiDatagramSpec EEPROMReadStateMachine::buildDatagram(uint8_t idx) {
    MultiDatagramSpec spec{};
    spec.idx       = idx;
    spec.roundtrip = true;
    spec.adp       = Master::adpForSlaveIndex(slave_index_);

    // Current word address (aligned to even)
    uint16_t current_word = currentWord() & 0xFFFEu;

    switch (state_) {
        case EEPROMState::WRITE_EEPADDR: {
            spec.cmd      = Command::APWR;
            spec.ado      = EEPROMProtocol::REG_EEPADDR;
            eepaddr_payload_ = Raw::host_to_le16(current_word);
            spec.data     = &eepaddr_payload_;
            spec.datalen  = sizeof(eepaddr_payload_);
            break;
        }
        case EEPROMState::WRITE_EEPCTL_READ: {
            spec.cmd      = Command::APWR;
            spec.ado      = EEPROMProtocol::REG_EEPCTL;
            eepctl_payload_ = Raw::host_to_le16(EEPROMProtocol::ECMD_READ);
            spec.data     = &eepctl_payload_;
            spec.datalen  = sizeof(eepctl_payload_);
            break;
        }
        case EEPROMState::POLL_EEPSTAT: {
            spec.cmd      = Command::APRD;
            spec.ado      = EEPROMProtocol::REG_EEPSTAT;
            spec.data     = nullptr;
            spec.datalen  = 2;
            break;
        }
        case EEPROMState::READ_EEPDAT: {
            spec.cmd      = Command::APRD;
            spec.ado      = EEPROMProtocol::REG_EEPDAT;
            spec.data     = nullptr;
            spec.datalen  = 4;
            break;
        }
        default:
            spec.cmd      = Command::APRD;
            spec.ado      = 0;
            spec.datalen  = 0;
            break;
    }

    return spec;
}

void EEPROMReadStateMachine::onComplete(const WaitResult& result,
                                         Master& master) {
    in_flight_ = false;

    if (!result.success) {
        fail();
        return;
    }

    switch (state_) {
        case EEPROMState::WRITE_EEPADDR:
            state_ = EEPROMState::WRITE_EEPCTL_READ;
            break;

        case EEPROMState::WRITE_EEPCTL_READ:
            busy_polls_ = 0;
            state_ = EEPROMState::POLL_EEPSTAT;
            break;

        case EEPROMState::POLL_EEPSTAT: {
            if (result.data_length < 2) {
                fail();
                return;
            }
            uint16_t estat = Raw::le16_to_host(
                *reinterpret_cast<const uint16_t*>(response_.data));

            if (estat & EEPROMProtocol::ESTAT_BUSY) {
                busy_polls_++;
                if (busy_polls_ >= EEPROMProtocol::MAX_BUSY_POLLS) {
                    fail();
                    return;
                }
                // Stay in POLL_EEPSTAT
            } else if (estat & EEPROMProtocol::ESTAT_EMASK) {
                if (estat & EEPROMProtocol::ESTAT_NACK) {
                    nack_count_++;
                    if (nack_count_ >= EEPROMProtocol::MAX_NACK_RETRIES) {
                        fail();
                        return;
                    }
                    state_ = EEPROMState::WRITE_EEPADDR;
                } else {
                    fail();
                }
            } else {
                state_ = EEPROMState::READ_EEPDAT;
            }
            break;
        }

        case EEPROMState::READ_EEPDAT: {
            if (result.data_length < 4) {
                fail();
                return;
            }
            uint32_t dword_le = 0;
            std::memcpy(&dword_le, response_.data, 4);
            uint32_t dword = Raw::le32_to_host(dword_le);

            // Cache the word-pair atomically
            master.slave(slave_index_).sii().cache().setWordPair(
                currentWord(), dword);

            // Advance to next word-pair or DONE
            advanceWord();
            break;
        }

        default:
            fail();
            break;
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

void EEPROMReactor::addSlave(uint16_t slave_index, uint32_t cat_mask) {
    SlaveEntry entry;
    entry.slave_index = slave_index;
    entry.parser.init(cat_mask);
    // Initialize the SM with the slave index and an empty vector so
    // slaveIndex() is correct before run() starts.
    entry.sm.init(slave_index, {});
    slaves_.push_back(std::move(entry));
}

void EEPROMReactor::cancelAllPending() {
    if (!master_) return;
    auto& router = master_->packetRouter();
    for (auto& s : slaves_) {
        if (s.sm.isInFlight() || (!s.sm.isFinished() && s.sm.slot() < TransactionRouter::kNumSlots)) {
            router.cancelPreRegistered(s.sm.slot());
            s.sm.cancel();
        }
    }
}

bool EEPROMReactor::refillQueue(SlaveEntry& entry) {
    auto& cache = master_->slave(entry.slave_index).sii().cache();

    // Loop until the parser either completes/fails or reports words
    // that are not yet cached (which become the next batch for the SM).
    while (true) {
        auto result = entry.parser.parse(cache, entry.out_data);

        if (result.isComplete()) {
            entry.done = true;
            return true;
        }
        if (result.isFailed()) {
            entry.done = true;
            return false;
        }

        // NEED_WORDS — collect uncached word-pairs into a vector.
        std::vector<uint16_t> pairs;
        pairs.reserve(result.needed_word_pairs.size());
        for (uint16_t addr : result.needed_word_pairs) {
            uint32_t dummy;
            if (!cache.getWordPair(addr, dummy)) {
                pairs.push_back(addr);
            }
        }

        if (!pairs.empty()) {
            // Initialize the state machine with this batch.
            entry.sm.init(entry.slave_index, std::move(pairs));
            return true;
        }

        // All needed words are already cached — loop to let the parser
        // advance to the next phase.
    }
}

bool EEPROMReactor::run(uint32_t timeout_ms) {
    if (!master_ || slaves_.empty()) return true;

    auto& router = master_->packetRouter();

    // Track which slots are in flight, mapped to slave indices.
    std::vector<size_t> slot_to_sm(TransactionRouter::kNumSlots, SIZE_MAX);
    std::vector<size_t> active_slots;
    std::vector<MultiDatagramSpec> specs;
    std::vector<size_t>            slots;
    std::vector<uint8_t>           idxs;

    active_slots.reserve(slaves_.size());
    specs.reserve(slaves_.size());
    idxs.reserve(slaves_.size());
    slots.reserve(slaves_.size());

    // Initial demand: for each slave, call the parser to get needed words
    // and initialize the state machine with the word-pair vector.
    for (auto& s : slaves_) {
        refillQueue(s);
    }

    int loop_count = 0;
    const int max_loops = 500000;  // Safety valve

    while (loop_count++ < max_loops) {
        // ---- Issue phase ----
        // For each slave that is not done and whose state machine is
        // finished (DONE from a previous batch), refill from the demand
        // parser to get the next batch of needed word-pairs.
        for (auto& s : slaves_) {
            if (s.done) continue;
            if (s.sm.isFinished() && !s.sm.isInFlight()) {
                if (s.sm.state() == EEPROMState::DONE) {
                    // Previous batch complete — ask parser for more
                    if (!refillQueue(s)) {
                        if (!s.done) s.done = true;
                    }
                } else {
                    // FAILED
                    s.done = true;
                }
            }
        }

        // Build datagrams for all state machines that need a send
        specs.clear();
        idxs.clear();
        slots.clear();

        for (size_t i = 0; i < slaves_.size(); ++i) {
            auto& s = slaves_[i];
            if (s.done) continue;
            if (!s.sm.needsSend()) continue;

            uint8_t idx = master_->allocIdx();
            size_t  slot = master_->preRegisterResponseWaiter(
                idx, s.sm.response().data, sizeof(s.sm.response().data));

            if (slot >= TransactionRouter::kNumSlots) {
                s.sm.cancel();
                s.done = true;
                continue;
            }

            s.sm.setSlot(slot);
            s.sm.markInFlight();
            slot_to_sm[slot] = i;
            active_slots.push_back(slot);
            slots.push_back(slot);
            idxs.push_back(idx);
            specs.push_back(s.sm.buildDatagram(idx));
        }

        // ---- Send phase ----
        if (!specs.empty()) {
            size_t sent = master_->sendMultiDatagram(specs.data(), specs.size());
            if (sent == 0) {
                for (size_t sl : slots) {
                    router.cancelPreRegistered(sl);
                    slot_to_sm[sl] = SIZE_MAX;
                }
                for (auto& s : slaves_) {
                    if (s.sm.isInFlight()) s.sm.cancel();
                }
                active_slots.erase(
                    std::remove_if(active_slots.begin(), active_slots.end(),
                        [&](size_t sl) { return slot_to_sm[sl] == SIZE_MAX; }),
                    active_slots.end());
            }
        }

        // ---- Check if all done ----
        if (active_slots.empty()) {
            bool all_done = true;
            for (const auto& s : slaves_) {
                if (!s.done) { all_done = false; break; }
            }
            if (all_done) break;
            // No in-flight datagrams but not all done — check if any
            // slave needs a refill.
            bool any_pending = false;
            for (const auto& s : slaves_) {
                if (!s.done && !s.sm.isFinished()) {
                    any_pending = true;
                    break;
                }
            }
            if (!any_pending) break;
            continue;
        }

        // ---- Wait for one completion ----
        auto any = router.waitForAny(active_slots.data(),
                                      active_slots.size(), timeout_ms);
        if (any.timed_out) {
            for (size_t sl : active_slots) {
                router.cancelPreRegistered(sl);
                slot_to_sm[sl] = SIZE_MAX;
            }
            for (auto& s : slaves_) {
                if (!s.sm.isFinished()) s.sm.cancel();
                if (!s.done) s.done = true;
            }
            active_slots.clear();
            break;
        }

        // ---- Dispatch the completed slot ----
        size_t completed_slot = active_slots[any.slot_index];
        size_t sm_idx = slot_to_sm[completed_slot];
        slot_to_sm[completed_slot] = SIZE_MAX;

        if (sm_idx < slaves_.size()) {
            slaves_[sm_idx].sm.onComplete(any.result, *master_);
        }

        // Remove the completed slot from active_slots (swap-and-pop)
        active_slots[any.slot_index] = active_slots.back();
        active_slots.pop_back();

        // Loop back to issue phase — the completed slave may now
        // have its SM in DONE state, and the issue phase will refill
        // its queue and start the next batch of reads.
    }

    // ---- Tally results ----
    for (const auto& s : slaves_) {
        if (s.done && s.parser.isComplete()) {
            success_count_++;
        } else {
            failure_count_++;
        }
    }

    return failure_count_ == 0;
}

} // namespace SII
} // namespace EtherCAT
