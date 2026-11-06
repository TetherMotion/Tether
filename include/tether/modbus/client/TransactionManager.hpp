/**
 * @file TransactionManager.hpp
 * @brief Pipelined TCP invoker: up to `maxInflight` transactions on the
 *        wire concurrently over one MbapChannel (plan §8.3).
 *
 * A single receiver thread reads ADUs and matches them to pending
 * transactions by MBAP transaction id. Callers block on per-request
 * promises; the inflight window is bounded by `maxInflight` (callers
 * wait for a slot when the window is full).
 *
 * RTU has no txn ids and a strict one-at-a-time wire model — this
 * class is MBAP-only. For serialized access use BusExecutor.
 */
#pragma once

#include <atomic>
#include <condition_variable>
#include <future>
#include <map>
#include <mutex>
#include <thread>

#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/link/Channel.hpp"
#include "tether/modbus/protocol/ExceptionCode.hpp"

namespace tether::modbus {

class TransactionManager final : public IModbusInvoker {
public:
    TransactionManager(MbapChannel& ch, int maxInflight = 16)
        : ch_(ch), maxInflight_(std::max(1, maxInflight)) {}
    ~TransactionManager() override { stop(); }

    /// Open the channel and start the receiver thread.
    Expected<void> start() {
        if (running_) return {};
        if (!ch_.isOpen())
            if (auto e = ch_.open(); !e) return e;
        stop_ = false;
        running_ = true;
        rx_ = std::thread([this] { rxLoop(); });
        return {};
    }
    void stop() {
        stop_ = true;
        ch_.close();
        if (rx_.joinable()) rx_.join();
        failAll(ModbusError::cancelled("txn manager stopped"));
        running_ = false;
    }
    bool running() const { return running_; }
    int  inflight() const {
        std::lock_guard g(m_);
        return inflight_;
    }

    Expected<void> invokeBroadcast(uint8_t, FunctionCode fc,
                                   std::span<const uint8_t> body,
                                   const InvokeOptions& = {}) override {
        std::lock_guard g(sendM_);
        return ch_.sendOnly(0, prepend(fc, body));
    }

    Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                         std::span<const uint8_t> body,
                         const InvokeOptions& o = {}) override {
        const int attempts = 1 + std::max(0, o.retries);
        ModbusError last = ModbusError::codec("no attempt");
        for (int i = 0; i < attempts; ++i) {
            if (i > 0 && o.retryDelayMs > 0)
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(o.retryDelayMs));
            auto r = round(unit, fc, body, o.timeoutMs);
            if (r) return r;
            last = r.error();
            const bool retryable = last.retryable() ||
                (last.isException() &&
                 exceptionRetryable(uint8_t(last.code() & 0xFF)));
            if (!retryable || i + 1 == attempts) break;
            if (o.onRetry) o.onRetry(i, last);
        }
        return err<Pdu>(last);
    }

private:
    struct Pending {
        std::promise<Expected<Adu>> pr;
    };

    /// One send+match round (no retry logic).
    Expected<Pdu> round(uint8_t unit, FunctionCode fc,
                        std::span<const uint8_t> body, int timeoutMs) {
        // Inflight window: wait for a slot.
        std::unique_lock lk(m_);
        if (!slotCv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                              [&] { return inflight_ < maxInflight_ ||
                                          stop_; }))
            return err<Pdu>(ModbusError::timedOut("inflight window full"));
        if (stop_) return err<Pdu>(ModbusError::cancelled("stopped"));
        const uint16_t txn = ch_.nextTxn();
        auto [it, _] = pending_.emplace(txn, Pending{});
        ++inflight_;
        auto fut = it->second.pr.get_future();  // must precede any erase
        lk.unlock();

        {   // Serialized send; full-duplex socket tolerates concurrent rx.
            std::lock_guard g(sendM_);
            Adu a; a.unit = unit; a.txn = txn;
            if (auto e = ch_.sendAdu(a, prepend(fc, body)); !e) {
                finishSlot(txn);
                return err<Pdu>(e.error());
            }
        }

        if (fut.wait_for(std::chrono::milliseconds(timeoutMs)) !=
            std::future_status::ready) {
            finishSlot(txn);
            return err<Pdu>(ModbusError::timedOut("response timeout"));
        }
        auto adu = fut.get();
        {
            std::lock_guard g(m_);
            --inflight_;
        }
        slotCv_.notify_all();
        if (!adu) return err<Pdu>(adu.error());
        return toPdu(fc, std::move(*adu));
    }

    void rxLoop() {
        while (!stop_) {
            auto r = ch_.receiveAny(200);
            if (!r) {
                if (r.error().isTimeout()) continue;
                // Link-level failure: fail everything in flight. If the
                // link is gone, back off instead of spinning.
                failAll(r.error());
                if (!ch_.isOpen())
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(50));
                continue;
            }
            std::lock_guard g(m_);
            if (auto it = pending_.find(r->txn); it != pending_.end()) {
                it->second.pr.set_value(std::move(*r));
                pending_.erase(it);
            }   // unknown txn: stray/late — drop
        }
    }

    /// Remove + fail one pending entry and release its window slot.
    void finishSlot(uint16_t txn) {
        std::lock_guard g(m_);
        pending_.erase(txn);
        --inflight_;
        slotCv_.notify_all();
    }
    void failAll(const ModbusError& e) {
        std::lock_guard g(m_);
        for (auto& [_, p] : pending_)
            p.pr.set_value(err<Adu>(e));
        pending_.clear();
        // inflight_ is released by each woken invoke()'s --inflight_.
    }

    static std::vector<uint8_t> prepend(FunctionCode fc,
                                        std::span<const uint8_t> body) {
        std::vector<uint8_t> p(body.size() + 1);
        p[0] = uint8_t(fc);
        std::copy(body.begin(), body.end(), p.begin() + 1);
        return p;
    }
    static Expected<Pdu> toPdu(FunctionCode fc, Adu a) {
        if (a.pdu.empty())
            return err<Pdu>(ModbusError::codec("empty pdu"));
        const uint8_t rfc = a.pdu[0];
        if (rfc == (uint8_t(fc) | 0x80))
            return err<Pdu>(exceptionToError(
                a.pdu.size() > 1 ? a.pdu[1] : 0));
        if (rfc != uint8_t(fc))
            return err<Pdu>(ModbusError::codec("function mismatch"));
        Pdu p{fc};
        if (!p.append(std::span(a.pdu.data() + 1, a.pdu.size() - 1)))
            return err<Pdu>(ModbusError::codec("pdu oversize"));
        return p;
    }

    MbapChannel& ch_;
    int          maxInflight_;
    mutable std::mutex m_;
    std::mutex   sendM_;
    std::condition_variable slotCv_;
    std::map<uint16_t, Pending> pending_;
    int             inflight_ = 0;
    std::thread     rx_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};
};

} // namespace tether::modbus
