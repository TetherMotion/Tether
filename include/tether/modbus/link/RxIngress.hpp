/**
 * @file RxIngress.hpp
 * @brief Interrupt-fed RX ingress: ISR pushes bytes, a processing
 *        thread drains — the "push" half of the serial model
 *        (plan §3 dual input path).
 *
 * Two ways bytes reach the RTU state machine:
 *   - Pull (Linux/RTOS): a thread calls ILink::read() on a buffered
 *     serial driver — SerialPortLink, EspUartLink.
 *   - Push (ISR-driven): a UART interrupt calls
 *     `ingress.onByte(b)` / `onBytes(...)` directly. No locks, no
 *     allocation — a lock-free SPSC ring carries bytes to the
 *     consumer. An atomic sequence counter is bumped + notified per
 *     enqueue, so a waiting thread can sleep on `seq().wait(old)`
 *     (std::atomic waiting) and wake the moment data lands.
 *
 * `IngressLink` adapts any RxIngress to the ILink contract (read =
 * wait + drain the ring; write = user TX hook), so RtuChannel,
 * ASCII framing and everything above it work unchanged on a bare
 * microcontroller UART ISR, an RTOS driver callback, or a Linux
 * host (via the IngressPump pull→push bridge).
 *
 * Wire-timing fidelity: the producer may stamp each byte's arrival
 * time (`onByte(b, tNs)`); IngressLink reports it through
 * ILink::rxByteAgeNs() so RtuChannel's t3.5 gap detection reflects
 * wire arrival, not drain time — important when the consumer drains
 * late under load.
 *
 * MCU wiring sketch (ESP-IDF):
 * @code
 *   static RxIngress<256> rx;
 *   // UART RX ISR / driver event → push path:
 *   IRAM_ATTR void uartIsr() {
 *       uint8_t b = uart_fifo_read(...);
 *       rx.onByte(b, esp_timer_get_time() * 1000);   // never blocks
 *   }
 *   // Processing thread:
 *   IngressLink link(rx, [](auto bytes, int) {
 *       return Expected<void>(uart_write_bytes(UART_NUM_1,
 *           bytes.data(), bytes.size()) >= 0 ? void() : void());
 *   });
 *   RtuChannel ch(std::move(link-ownership), RtuTiming::forSerial(cfg));
 * @endcode
 *
 * Platform knobs: IngressWait{nowNs,delayUs} defaults to
 * steady_clock + sleep_for on hosted builds, esp_timer +
 * esp_rom_delay_us on ESP_PLATFORM. Freestanding targets inject
 * their own (cycle counter + WFI/yield).
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

#include "tether/modbus/link/ILink.hpp"

#if defined(ESP_PLATFORM)
#include "esp_rom_sys.h"
#include "esp_timer.h"
#else
#include <thread>
#endif

namespace tether::modbus {

/// Platform clock + delay hooks for the bounded-wait loop.
struct IngressWait {
    int64_t (*nowNs)() noexcept = &ingressDefaultNowNs;
    void    (*delayUs)(uint32_t) noexcept = &ingressDefaultDelayUs;

    static int64_t ingressDefaultNowNs() noexcept {
#if defined(ESP_PLATFORM)
        return esp_timer_get_time() * 1000;
#else
        return std::chrono::steady_clock::now()
               .time_since_epoch().count();
#endif
    }
    static void ingressDefaultDelayUs(uint32_t us) noexcept {
#if defined(ESP_PLATFORM)
        esp_rom_delay_us(us);
#else
        std::this_thread::sleep_for(std::chrono::microseconds(us));
#endif
    }
};

/// Consumer-side view over an RxIngress — keeps IngressLink
/// non-templated while the ring itself stays a fixed-size template.
class RxIngressBase {
public:
    virtual ~RxIngressBase() = default;
    /// Producer side: queue a burst (ISR-safe); one seq notify/batch.
    virtual size_t onBytes(std::span<const uint8_t> bytes,
                           int64_t tNs = 0) noexcept = 0;
    /// Move up to out.size() bytes into out. When `splitGapNs` > 0 and
    /// entries are producer-timestamped, the drain STOPS before an
    /// entry whose timestamp is >splitGapNs after the previous one —
    /// wire gaps preserved across reads even when bursts coalesce in
    /// the ring. *firstTsNs / *lastTsNs get the edge timestamps (0 =
    /// untimed).
    virtual size_t drain(std::span<uint8_t> out, int64_t splitGapNs = 0,
                         int64_t* firstTsNs = nullptr,
                         int64_t* lastTsNs = nullptr) noexcept = 0;
    virtual size_t   pending()  const noexcept = 0;
    virtual void     clear()    noexcept = 0;
    virtual uint32_t overruns() const noexcept = 0;
    /// Enqueue counter — `seq().wait(old)` sleeps until a byte lands.
    /// Non-const: waiters notify on close; producers use it internally.
    virtual std::atomic<uint32_t>& seq() noexcept = 0;

    /// Bounded wait for data — polls pending() every ≤1 ms via the
    /// wait hooks; returns pending() > 0. For an indefinite atomic
    /// wait use seq().wait(old) directly.
    bool waitAvailable(int64_t deadlineNs,
                       const IngressWait& w = {}) const noexcept {
        for (;;) {
            if (pending() != 0) return true;
            const int64_t now = w.nowNs();
            if (now >= deadlineNs) return false;
            const int64_t remainUs = (deadlineNs - now) / 1000;
            w.delayUs(uint32_t(remainUs < 1000 ? remainUs : 1000));
        }
    }
};

/**
 * @brief Lock-free SPSC byte ring: producer (ISR) → consumer (thread).
 *
 * One producer, one consumer, no dynamic allocation — safe in an
 * interrupt handler. Capacity-1 bytes are usable (one slot marks full).
 * Ordering: producer stores the cell then releases head_; consumer
 * acquires head_ before reading cells — standard SPSC message passing.
 */
template<size_t Capacity>
class RxIngress final : public RxIngressBase {
public:
    static_assert(Capacity >= 4, "ring too small");
    static constexpr size_t capacity = Capacity;

    /// ISR-safe: queue one byte. `tNs` = optional arrival timestamp
    /// (producer's clock, e.g. esp_timer_get_time()*1000) used for
    /// precise inter-frame gap detection; 0 = untimed.
    /// Returns false (and counts an overrun) when the ring is full.
    bool onByte(uint8_t b, int64_t tNs = 0) noexcept {
        const size_t h = head_.load(std::memory_order_relaxed);
        const size_t n = (h + 1) % Capacity;
        if (n == tail_.load(std::memory_order_acquire)) {
            overruns_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        bytes_[h] = b;
        stamps_[h] = tNs;
        head_.store(n, std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);
        seq_.notify_all();               // wake atomic waiters
        return true;
    }

    /// ISR-safe: queue a burst; one notify for the batch.
    size_t onBytes(std::span<const uint8_t> bytes,
                   int64_t tNs = 0) noexcept {
        size_t pushed = 0;
        for (uint8_t b : bytes) {
            const size_t h = head_.load(std::memory_order_relaxed);
            const size_t n = (h + 1) % Capacity;
            if (n == tail_.load(std::memory_order_acquire)) {
                overruns_.fetch_add(
                    uint32_t(bytes.size() - pushed),
                    std::memory_order_relaxed);
                break;
            }
            bytes_[h] = b;
            stamps_[h] = tNs;
            head_.store(n, std::memory_order_release);
            ++pushed;
        }
        if (pushed != 0) {
            seq_.fetch_add(uint32_t(pushed), std::memory_order_release);
            seq_.notify_all();
        }
        return pushed;
    }

    size_t drain(std::span<uint8_t> out, int64_t splitGapNs = 0,
                 int64_t* firstTsNs = nullptr,
                 int64_t* lastTsNs = nullptr) noexcept override {
        size_t t = tail_.load(std::memory_order_relaxed);
        const size_t h = head_.load(std::memory_order_acquire);
        int64_t prevTs = 0;
        size_t n = 0;
        while (t != h && n < out.size()) {
            const int64_t ts = stamps_[t];
            if (splitGapNs > 0 && ts != 0 && prevTs != 0 &&
                ts - prevTs > splitGapNs)
                break;                       // wire gap → new batch
            out[n] = bytes_[t];
            if (n == 0 && firstTsNs) *firstTsNs = ts;
            if (lastTsNs) *lastTsNs = ts;
            if (ts != 0) prevTs = ts;
            t = (t + 1) % Capacity;
            ++n;
        }
        tail_.store(t, std::memory_order_release);
        return n;
    }

    size_t pending() const noexcept override {
        const size_t h = head_.load(std::memory_order_acquire);
        const size_t t = tail_.load(std::memory_order_acquire);
        return h >= t ? h - t : Capacity - t + h;
    }
    void clear() noexcept override {
        tail_.store(head_.load(std::memory_order_acquire),
                    std::memory_order_release);
    }
    uint32_t overruns() const noexcept override {
        return overruns_.load(std::memory_order_relaxed);
    }
    std::atomic<uint32_t>& seq() noexcept override { return seq_; }

private:
    std::atomic<size_t>   head_{0};
    std::atomic<size_t>   tail_{0};
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint32_t> overruns_{0};
    uint8_t bytes_[Capacity];
    int64_t stamps_[Capacity];
};

/**
 * @brief ILink adapter over an RxIngress: read() = bounded wait +
 *        drain; write() = user TX hook; flush(Rx) = clear ring.
 *
 * @param waitFn Optional custom wait (e.g. RTOS task notify /
 *        event-group wait). Default = waitAvailable() poll loop.
 */
class IngressLink final : public ILink {
public:
    using TxHook =
        std::function<Expected<void>(std::span<const uint8_t>, int)>;
    /// Wait until ingress has data or deadlineNs (wait clock domain)
    /// elapses → true = data available.
    using WaitFn =
        std::function<bool(RxIngressBase&, int64_t deadlineNs)>;

    /// @param intraGapNs when >0, drain() splits batches at producer-
    ///        timestamp gaps larger than this — pass the channel's
    ///        inter-frame bound (t3.5 ns) so a drained burst containing
    ///        a wire gap still splits into separate reads.
    IngressLink(RxIngressBase& ingress, TxHook tx,
                IngressWait wait = {}, WaitFn waitFn = {},
                int64_t intraGapNs = 0)
        : ingress_(ingress), tx_(std::move(tx)),
          wait_(wait), waitFn_(std::move(waitFn)),
          intraGapNs_(intraGapNs) {}

    Expected<void> open() override { open_ = true; return {}; }
    void close() override {
        open_ = false;
        ingress_.seq().notify_all();   // release any external waiter
    }
    bool isOpen() const override { return open_; }

    Expected<void> write(std::span<const uint8_t> bytes,
                         int timeoutMs) override {
        if (!open_) return err<void>(ModbusError::transport("closed"));
        if (!tx_)   return err<void>(ModbusError::unsupported("no tx"));
        return tx_(bytes, timeoutMs);
    }

    Expected<size_t> read(std::span<uint8_t> buf,
                          int timeoutMs) override {
        if (!open_) return err<size_t>(ModbusError::transport("closed"));
        if (ingress_.pending() == 0) {
            const int64_t dl =
                wait_.nowNs() + int64_t(timeoutMs) * 1'000'000;
            const bool got = waitFn_
                ? waitFn_(ingress_, dl)
                : ingress_.waitAvailable(dl, wait_);
            if (!got) return size_t{0};
        }
        int64_t firstTs = 0, lastTs = 0;
        const size_t n =
            ingress_.drain(buf, intraGapNs_, &firstTs, &lastTs);
        // Wire-arrival times of this batch's edge bytes — producer
        // timestamps when given, else drain time (this clock's domain).
        const int64_t now = wait_.nowNs();
        batchFirstNs_ = firstTs != 0 ? firstTs : now;
        batchLastNs_  = lastTs  != 0 ? lastTs  : now;
        return n;
    }

    int64_t rxBatchFirstAgeNs() const noexcept override {
        if (!batchFirstNs_) return 0;
        // Signed delta — may be negative when the producer stamps a
        // wire time ahead of this clock's drain instant; after-age
        // then recovers the stamp in the channel's clock domain.
        return wait_.nowNs() - batchFirstNs_;
    }
    int64_t rxBatchLastAgeNs() const noexcept override {
        if (!batchLastNs_) return 0;
        return wait_.nowNs() - batchLastNs_;
    }

    Expected<void> flush(Flush f) override {
        if (f != Flush::Tx) ingress_.clear();
        return {};
    }
    Expected<size_t> drain() override {
        uint8_t tmp[64];
        size_t total = 0;
        for (;;) {
            const size_t n = ingress_.drain(std::span(tmp), 0);
            if (!n) return total;
            total += n;
        }
    }
    LinkInfo info() const override {
        return {"ingress", "isr-fed", LinkKind::Serial, false, true};
    }

private:
    RxIngressBase& ingress_;
    TxHook         tx_;
    IngressWait    wait_;
    WaitFn         waitFn_;
    int64_t        intraGapNs_ = 0;
    int64_t        batchFirstNs_ = 0;
    int64_t        batchLastNs_ = 0;
    bool           open_ = false;
};

#if !defined(TETHER_MODBUS_NO_THREADS)
/**
 * @brief Pull→push bridge: a thread reads a buffered ILink and feeds
 *        an RxIngress — brings the ISR-style ingress model to hosts
 *        whose serial driver only offers blocking reads (e.g. running
 *        a master's receive path fully asynchronously on Linux).
 */
class IngressPump {
public:
    IngressPump(ILink& src, RxIngressBase& dst,
                IngressWait wait = {}, size_t chunk = 64)
        : src_(src), dst_(dst), wait_(wait), chunk_(chunk) {}
    ~IngressPump() { stop(); }

    void start() {
        stop_ = false;
        th_ = std::thread([this] { loop(); });
    }
    void stop() {
        stop_ = true;
        if (th_.joinable()) th_.join();
    }
    bool running() const { return !stop_; }

private:
    void loop() {
        std::vector<uint8_t> buf(chunk_);
        while (!stop_) {
            auto r = src_.read(std::span(buf), 50 /*ms*/);
            if (!r || *r == 0) continue;
            dst_.onBytes(std::span(buf.data(), *r), wait_.nowNs());
        }
    }
    ILink&          src_;
    RxIngressBase&  dst_;
    IngressWait     wait_;
    size_t          chunk_;
    std::thread     th_;
    std::atomic<bool> stop_{true};
};
#endif // TETHER_MODBUS_NO_THREADS

} // namespace tether::modbus
