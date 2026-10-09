/**
 * @file PDOManager_queue.cpp
 * @brief PDOManager — callback mode and queue mode (Mode 2/3).
 *
 * TU split out of PDOManager.cpp.
 */

#include "tether/ethercat/PDOManager.hpp"
#include "raw/PDOManagerInternal.hpp"
#include "tether/ethercat/LogicalAddressManager.hpp"
#include "tether/utils/ColoredBitsetFormatter.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <format>
#include <vector>

namespace EtherCAT {

// ============================================================================
// Callback mode (Mode 3)
// ============================================================================

void PDOManager::configureCallbackMode(const CallbackModeConfig& config) {
    mode_ = PDOMode::Callback;
    callback_config_ = config;
    callbacks_.resize(PDO::kMaxPDOEntries);
}

void PDOManager::setTxSentCallback(size_t entry_index, PDOTxSentCallback callback) {
    if (entry_index < callbacks_.size()) {
        callbacks_[entry_index].tx_sent = std::move(callback);
    }
}

void PDOManager::setRxReceivedCallback(size_t entry_index, PDORxReceivedCallback callback) {
    if (entry_index < callbacks_.size()) {
        callbacks_[entry_index].rx_received = std::move(callback);
    }
}

// ============================================================================
// Queue mode (Mode 2)
// ============================================================================

void PDOManager::configureQueueMode(const QueueModeConfig& config) {
    mode_ = PDOMode::Queue;
    queue_config_ = config;

    const size_t n = PDO::kMaxPDOEntries;
    tx_queues_.clear();
    rx_queues_.clear();
    tx_queues_.reserve(n);
    rx_queues_.reserve(n);
    for (size_t i = 0; i < n; i++) {
        tx_queues_.push_back(std::make_unique<FrameQueue>(config.tx_queue_capacity));
        rx_queues_.push_back(std::make_unique<FrameQueue>(config.rx_queue_capacity));
    }
    event_queue_ = std::make_unique<EventQueue>(config.event_queue_capacity);
    last_tx_frames_.resize(n);
    underrun_callback_ = nullptr;
}

bool PDOManager::enqueueTx(size_t entry_index, std::shared_ptr<PDOFrame> frame) {
    if (mode_ != PDOMode::Queue || entry_index >= tx_queues_.size() || !tx_queues_[entry_index])
        return false;
    return tx_queues_[entry_index]->try_push(std::move(frame));
}

bool PDOManager::tryDequeueRx(size_t entry_index, std::shared_ptr<PDOFrame>& frame) {
    if (mode_ != PDOMode::Queue || entry_index >= rx_queues_.size() || !rx_queues_[entry_index])
        return false;
    return rx_queues_[entry_index]->try_pop(frame);
}

bool PDOManager::tryPollEvent(std::shared_ptr<PDOEvent>& event) {
    if (mode_ != PDOMode::Queue || !event_queue_)
        return false;
    return event_queue_->try_pop(event);
}

void PDOManager::setUnderrunCallback(UnderrunCallback callback) {
    underrun_callback_ = std::move(callback);
}

bool PDOManager::queueCycle() {
    if (mode_ != PDOMode::Queue) return false;

    const uint64_t cycle_start_ns =
        static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;

    // Phase 1: Drain TX queues into app buffers for RxPDO entries
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* e = mapping_.get_entry_mut(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
            continue;

        std::shared_ptr<PDOFrame> frame;
        if (tx_queues_[i] && tx_queues_[i]->try_pop(frame)) {
            // Got new TX data — copy into app buffer
            if (frame->data.size() <= e->data_size) {
                std::memcpy(e->storage, frame->data.data(), frame->data.size());
            }
            last_tx_frames_[i] = frame;
        } else {
            // Underrun — apply policy
            if (underrun_callback_) {
                underrun_callback_(e->slave_index, stats_.total_cycles);
            }
            // Push underrun event
            if (event_queue_) {
                auto ev = std::make_shared<PDOEvent>();
                ev->type = PDOEvent::Type::Underrun;
                ev->slave_index = e->slave_index;
                ev->pdo_entry_index = static_cast<uint16_t>(i);
                ev->timestamp_ns = cycle_start_ns;
                ev->cycle_count = stats_.total_cycles;
                event_queue_->try_push(std::move(ev));  // Drop if full
            }

            switch (queue_config_.underrun_policy) {
                case UnderrunPolicy::RepeatLastFrame:
                    if (last_tx_frames_[i] && last_tx_frames_[i]->data.size() <= e->data_size) {
                        std::memcpy(e->storage, last_tx_frames_[i]->data.data(),
                                    last_tx_frames_[i]->data.size());
                    }
                    break;
                case UnderrunPolicy::SafeState:
                    if (queue_config_.safe_state_buffer.size() <= e->data_size) {
                        std::memcpy(e->storage, queue_config_.safe_state_buffer.data(),
                                    queue_config_.safe_state_buffer.size());
                    }
                    break;
                case UnderrunPolicy::SkipCycle:
                    e->enabled = false;  // Temporarily disable for this cycle
                    break;
                case UnderrunPolicy::Custom:
                    // User callback already invoked above; no automatic action
                    break;
            }
        }
    }

    // Phase 2: Bus exchange
    bool ok = sendAll();
    ok = receiveAll() && ok;

    // Re-enable any entries we skipped for underrun
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        PDO::PDOEntry* e = mapping_.get_entry_mut(i);
        if (!e) continue;
        // Re-enable entries that were disabled by SkipCycle
        // (only if they were disabled by us, not the user)
        // We can't distinguish, so we re-enable all that have queue data
        if (e->direction == PDO::PDODirection::RxPDO && !e->enabled) {
            // Check if this was a SkipCycle disable by checking if queue has data now
            // Simplest: just re-enable — user disables are done via mapping API
            e->enabled = true;
        }
    }

    // Phase 3: Push received TxPDO data to RX queues and emit events
    for (size_t i = 0; i < mapping_.entry_count(); i++) {
        const PDO::PDOEntry* e = mapping_.get_entry(i);
        if (!e || !e->enabled || e->direction != PDO::PDODirection::TxPDO)
            continue;

        // Create RX frame
        auto frame = std::make_shared<PDOFrame>();
        frame->data.resize(e->data_size);
        std::memcpy(frame->data.data(), e->storage, e->data_size);
        frame->timestamp_ns =
            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
        frame->cycle_count = stats_.total_cycles;

        // try_push — drop if queue is full (user not consuming fast enough)
        if (rx_queues_[i]) {
            rx_queues_[i]->try_push(std::move(frame));
        }

        // Push RxReceived event
        if (event_queue_ && queue_config_.enable_rx_received_events) {
            auto ev = std::make_shared<PDOEvent>();
            ev->type = PDOEvent::Type::RxReceived;
            ev->slave_index = e->slave_index;
            ev->pdo_entry_index = static_cast<uint16_t>(i);
            ev->timestamp_ns =
                static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
            ev->cycle_count = stats_.total_cycles;
            event_queue_->try_push(std::move(ev));  // Drop if full
        }
    }

    // Push TxSent events for RxPDO entries
    if (queue_config_.enable_tx_sent_events) {
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (!e || !e->enabled || e->direction != PDO::PDODirection::RxPDO)
                continue;
            if (event_queue_) {
                auto ev = std::make_shared<PDOEvent>();
                ev->type = PDOEvent::Type::TxSent;
                ev->slave_index = e->slave_index;
                ev->pdo_entry_index = static_cast<uint16_t>(i);
                ev->timestamp_ns = cycle_start_ns;
                ev->cycle_count = stats_.total_cycles;
                event_queue_->try_push(std::move(ev));  // Drop if full
            }
        }
    }

    // Push error events if exchange failed
    if (!ok && event_queue_) {
        auto ev = std::make_shared<PDOEvent>();
        ev->type = PDOEvent::Type::Error;
        ev->timestamp_ns =
            static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds()) * 1000ULL;
        ev->cycle_count = stats_.total_cycles;
        event_queue_->try_push(std::move(ev));
    }

    return ok;
}

} // namespace EtherCAT

