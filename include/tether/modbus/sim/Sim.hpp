/**
 * @file Sim.hpp
 * @brief SimSlave (in-memory device model) + SimInvoker (direct dispatch).
 */
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <random>

#include "tether/modbus/client/Invoker.hpp"
#include "tether/modbus/link/ILink.hpp"
#include "tether/modbus/server/DataStore.hpp"
#include "tether/modbus/server/Dispatcher.hpp"

namespace tether::modbus {

struct SimTraits {
    /// Per-invoke fault injection probabilities [0..1].
    double dropRequest  = 0.0;   ///< request never arrives → timeout
    double dropResponse = 0.0;   ///< response lost → timeout
    double corrupt      = 0.0;   ///< flip one pdu byte
    double excBusy      = 0.0;   ///< respond 0x06 busy
    int    delayMs     = 0;      ///< fixed extra latency
    uint64_t seed      = 0x1234;
};

/**
 * @brief A simulated Modbus device: DataStore + dispatcher + quirks.
 *
 * `handlePdu` models the wire-visible behavior incl. faults; it returns
 * nullopt on "no response" (dropped) so the caller sees a timeout.
 */
class SimSlave {
public:
    SimSlave(uint8_t unit, SimTraits t = {})
        : unit_(unit), traits_(t), rng_(uint32_t(t.seed)),
          disp_(store_) {}

    DataStore& store() { return store_; }
    SimTraits& traits() { return traits_; }
    uint8_t unit() const { return unit_; }

    /// One transaction: request pdu (incl fc) → response pdu or nullopt.
    std::optional<std::vector<uint8_t>>
    handlePdu(std::span<const uint8_t> reqPdu);

    /// Convenience: preload holding regs.
    void preload(uint16_t addr, std::initializer_list<uint16_t> vals) {
        for (auto v : vals) (void)store_.setReg(
            TableId::HoldingRegister, addr++, v);
    }

private:
    uint8_t           unit_;
    SimTraits         traits_;
    std::mt19937      rng_;
    DataStore         store_;
    RequestDispatcher disp_;
    std::uniform_real_distribution<double> uni_{0.0, 1.0};
};

/**@brief Invoker that talks directly to a SimSlave (no bytes). */
class SimInvoker final : public IModbusInvoker {
public:
    SimInvoker(SimSlave& slave, bool checkUnit = true)
        : slave_(slave), checkUnit_(checkUnit) {}

    Expected<Pdu> invoke(uint8_t unit, FunctionCode fc,
                         std::span<const uint8_t> body,
                         const TransactionOptions& o = {}) override;
    Expected<void> invokeBroadcast(
        uint8_t, FunctionCode fc, std::span<const uint8_t> body,
        const TransactionOptions& o = {}) override {
        std::vector<uint8_t> pdu(body.size() + 1);
        pdu[0] = uint8_t(fc);
        std::copy(body.begin(), body.end(), pdu.begin() + 1);
        slave_.handlePdu(pdu);
        return {};
    }
private:
    SimSlave& slave_;
    bool      checkUnit_;
};

/**@brief A whole simulated bus: multiple SimSlaves keyed by unit. */
class SimBus {
public:
    SimSlave& add(uint8_t unit, SimTraits t = {});
    SimSlave* find(uint8_t unit);
    /// Invoker dispatching to the right slave by unit id.
    IModbusInvoker& invoker() { return inv_; }

private:
    class BusInvoker final : public IModbusInvoker {
    public:
        explicit BusInvoker(SimBus& b) : bus_(b) {}
        Expected<Pdu> invoke(uint8_t u, FunctionCode fc,
                             std::span<const uint8_t> b,
                             const TransactionOptions& o = {}) override;
        SimBus& bus_;
    } inv_{*this};
    std::map<uint8_t, std::unique_ptr<SimSlave>> slaves_;
};

// ---- wire-level sim link -------------------------------------------------------

struct SimLinkTraits {
    double dropRequest  = 0.0;   ///< request frame never reaches device
    double dropResponse = 0.0;   ///< whole response frame lost
    double corruptByte  = 0.0;   ///< xor one random response byte
    double dropByte     = 0.0;   ///< drop one random response byte
    double dupByte      = 0.0;   ///< duplicate one random response byte
    int    turnaroundMs = 0;     ///< device response latency
    int    dribbleBytes = -1;    ///< -1: rx all at once; ≥1: max per read()
    uint64_t seed = 0xBEEF;
    /// Deterministic fault hooks — e.g. byte-loss matrix:
    ///   mutateResponse = [i](auto& f){ f.erase(f.begin()+i); };
    std::function<void(std::vector<uint8_t>&)> mutateRequest;
    std::function<void(std::vector<uint8_t>&)> mutateResponse;
};

/**
 * @brief ILink that terminates inside a SimBus at the *wire* level:
 *        real encoded frames in, real encoded frames out — exercises
 *        framers, CRC checks, t3.5 gaps and channels without hardware.
 *
 * Use RTU mode (rtu=true) with RtuChannel or MBAP (rtu=false) with
 * MbapChannel. Fault hooks can delete/corrupt/reorder bytes
 * deterministically — the byte-loss matrix in tests is built on them.
 */
class SimLink final : public ILink {
public:
    SimLink(SimBus& bus, bool rtu = true, SimLinkTraits t = {})
        : bus_(bus), rtu_(rtu), traits_(std::move(t)),
          rng_(uint32_t(traits_.seed)) {}
    SimLinkTraits& traits() { return traits_; }

    Expected<void>   open() override { open_.store(true); return {}; }
    void             close() override { open_.store(false); }
    bool             isOpen() const override { return open_.load(); }

    Expected<void>   write(std::span<const uint8_t> bytes,
                           int timeoutMs) override;
    Expected<size_t> read(std::span<uint8_t> buf, int timeoutMs) override;

    Expected<void> flush(Flush f) override {
        if (f != Flush::Tx) {
            std::lock_guard lk(rx_mutex_);
            rx_.clear();
        }
        return {};
    }
    Expected<size_t> drain() override {
        std::lock_guard lk(rx_mutex_);
        const size_t n = rx_.size(); rx_.clear(); return n;
    }
    LinkInfo info() const override {
        return {"sim", rtu_ ? "sim-rtu" : "sim-mbap", LinkKind::Sim,
                false, true};
    }
    void setTap(std::function<void(bool, std::span<const uint8_t>)> t)
        override { tap_ = std::move(t); }

    /// Bytes staged for reading (test introspection).
    size_t rxPending() const {
        std::lock_guard lk(rx_mutex_);
        return rx_.size();
    }

private:
    /// rng_ draw under its lock — write() can run on multiple invoker threads.
    double   rollUniform();
    uint32_t rollIndex();

    SimBus&        bus_;
    bool           rtu_;
    SimLinkTraits  traits_;
    std::mt19937   rng_;
    std::deque<uint8_t> rx_;
    std::atomic<bool>   open_{false};
    mutable std::mutex rx_mutex_;
    std::mutex          rng_mutex_;
    std::function<void(bool, std::span<const uint8_t>)> tap_;
};

} // namespace tether::modbus
