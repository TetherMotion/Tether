/**
 * @file EtherCATDC.hpp
 * @brief EtherCAT Distributed Clock (DC) synchronization class
 *
 * Extracted from DCClass.hpp. Contains the EtherCATDC class and
 * NoDistributedClockConfigured sentinel.
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include "tether/platform/Platform.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/IDCTransport.hpp"
#include "tether/ethercat/DCTypes.hpp"

namespace EtherCAT {

class RealtimeLoop;

class EtherCATDC {
public:
    EtherCATDC(IDCTransport& transport,
               uint16_t slave_count,
               const DCConfig* config = nullptr);

    virtual ~EtherCATDC();

    EtherCATDC(const EtherCATDC&) = delete;
    EtherCATDC& operator=(const EtherCATDC&) = delete;
    EtherCATDC(EtherCATDC&&) = delete;
    EtherCATDC& operator=(EtherCATDC&&) = delete;

    virtual bool start();
    virtual bool start(std::function<bool()> pdo_exchange_fn);
    virtual bool init();
    virtual void stop();
    virtual DCState getState() const { return state_.load(std::memory_order_acquire); }
    virtual DCLoopStats getStats() const;
    virtual void forceSync();
    virtual void setPDOEnabled(bool enable);
    virtual bool isPDOEnabled() const;
    virtual bool isSlaveSupported(uint16_t slave_index) const;
    virtual int64_t getSlaveOffset(uint16_t slave_index) const;
    virtual void readSyncConfig(uint16_t slave_index);
    virtual bool reconfigureSync(uint16_t slave_index);
    virtual uint64_t getMasterTimeNs();
    virtual bool readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                      unsigned int timeout_ms = 200);
    virtual bool writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                       unsigned int timeout_ms = 200);
    virtual bool sendSyncFrame();

private:
    DCConfig config_;
    std::atomic<DCState> state_{DCState::Disabled};
    uint16_t slave_count_;
    SlaveTimeInfo slaves_[kMaxDCSlaves];
    uint64_t master_reference_time_ns_{0};
    uint64_t dc_start_time_ns_{0};
    uint64_t next_sync_time_ns_{0};
    std::atomic<bool> initialized_{false};
    DCLoopStats stats_{};
    mutable std::mutex stats_mutex_;
    IDCTransport& transport_;
    std::unique_ptr<RealtimeLoop> realtime_loop_;
    std::function<bool()> pdo_exchange_fn_;

    bool initialize();
    bool readSlaveCapabilities(uint16_t slave_index);
    bool calcPropagationDelay(uint16_t slave_index);
    bool writeSystemTimeOffset(uint16_t slave_index, int64_t offset);
    bool configureSyncSignals(uint16_t slave_index);
    bool updateSyncStartTime();
};

/**
 * @brief State-less no-op IDCTransport used as the transport for the
 *        NoDistributedClockConfigured sentinel.
 *
 * Every method returns a safe no-op result (false / 0). The sentinel
 * overrides every public EtherCATDC method and never delegates to the base,
 * so this transport is never actually exercised — it exists only to satisfy
 * the EtherCATDC base-class constructor's reference parameter without
 * introducing process-global state.
 *
 * It is held as a private base class of NoDistributedClockConfigured
 * (the "base-from-member" idiom) so that the EtherCATDC subobject can be
 * constructed with a reference to an already-constructed NoopDCTransport
 * subobject, avoiding both a global instance and a dangling reference.
 */
class NoopDCTransport : public IDCTransport {
public:
    bool readRegister(uint16_t, uint16_t, void*, uint16_t, unsigned int = 200) override { return false; }
    bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int = 200) override { return false; }
    bool sendSyncDatagram(uint16_t, uint16_t, const void*, uint16_t) override { return false; }
    uint64_t getMasterTimeNs() override { return 0; }
    void delayMs(uint32_t) override {}
};

class NoDistributedClockConfigured final : private NoopDCTransport, public EtherCATDC {
public:
    NoDistributedClockConfigured();

    bool start() override;
    bool start(std::function<bool()> pdo_exchange_fn) override;
    bool init() override;
    void stop() override;

    DCState getState() const override;
    DCLoopStats getStats() const override;

    void forceSync() override;
    void setPDOEnabled(bool enable) override;
    bool isPDOEnabled() const override;

    bool isSlaveSupported(uint16_t slave_index) const override;
    int64_t getSlaveOffset(uint16_t slave_index) const override;

    void readSyncConfig(uint16_t slave_index) override;
    bool reconfigureSync(uint16_t slave_index) override;

    uint64_t getMasterTimeNs() override;

    bool readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                      unsigned int timeout_ms = 200) override;
    bool writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                       unsigned int timeout_ms = 200) override;

    bool sendSyncFrame() override;

private:
    void logCritical(const char* method) const;
};

} // namespace EtherCAT
