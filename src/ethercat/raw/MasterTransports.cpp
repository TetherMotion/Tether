/**
 * @file MasterTransports.cpp
 * @brief Master transport-adapter classes (PDO, fault, SDO).
 */

#include "raw/MasterTransports.hpp"

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/CoeSDOChannel.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/platform/Platform.hpp"

#include <chrono>
#include <thread>

namespace EtherCAT {

// ============================================================================
// MasterPDOTransport — adapts Master to IPDOTransport
// ============================================================================

class MasterPDOTransport : public IPDOTransport {
public:
    explicit MasterPDOTransport(Master& master) : master_(master) {}

    bool writeRegister(uint16_t adp, uint16_t ado,
                       const void* data, uint16_t len,
                       unsigned int timeout_ms) override {
        return master_.writeRegister(Master::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool readRegister(uint16_t adp, uint16_t ado,
                      void* data, uint16_t len,
                      unsigned int timeout_ms) override {
        return master_.readRegister(Master::slaveAddressFromADP(adp), ado, data, len, timeout_ms);
    }

    bool sendSingleDatagram(Command cmd, uint8_t idx,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip) override {
        return master_.sendSingleDatagram(cmd, idx, adp, ado, data, datalen, roundtrip);
    }

    size_t sendMultiDatagram(const MultiDatagramSpec* specs, size_t count) override {
        return master_.sendMultiDatagram(specs, count);
    }

    bool waitForResponseIdx(uint8_t idx, unsigned int timeout_ms,
                            RxDatagram& out) override {
        return master_.waitForResponseIdx(idx, timeout_ms, out);
    }

    size_t preRegisterResponseWaiter(uint8_t idx,
                                     uint8_t* buffer, size_t buffer_size) override {
        return master_.preRegisterResponseWaiter(idx, buffer, buffer_size);
    }

    bool waitForPreRegistered(size_t slot, unsigned int timeout_ms,
                              RxDatagram& out) override {
        WaitResult wr = master_.waitForPreRegistered(slot, timeout_ms);
        if (!wr.success) return false;
        out.idx = wr.idx; out.cmd = wr.cmd; out.adp = wr.adp;
        out.ado = wr.ado;
        out.datalen = static_cast<uint16_t>(wr.data_length);
        out.wkc = wr.wkc;
        return true;
    }

    uint8_t allocIdx() override {
        return master_.allocIdx();
    }

    uint16_t adpForSlaveIndex(uint16_t slave_index) override {
        return Master::adpForSlaveIndex(slave_index);
    }

    bool isCancelRequested() const override {
        return master_.isCancelRequested();
    }

    size_t maxEtherCATPayloadPerFrame() const override {
        return master_.maxEtherCATPayloadPerFrame();
    }

    // ---- Cyclic fast path ----
    bool supportsCyclicFastPath() const override {
        return master_.supportsCyclicFastPath();
    }
    uint64_t cyclicSlotToken(uint8_t slot) override {
        return master_.cyclicSlotToken(slot);
    }
    uint8_t cyclicSlotGen(uint8_t slot) override {
        return master_.cyclicSlotGen(slot);
    }
    bool sendCyclicDatagram(Command cmd, uint8_t slot,
                            uint16_t adp, uint16_t ado,
                            const void* data, uint16_t datalen,
                            bool roundtrip) override {
        return master_.sendCyclicDatagram(cmd, slot, adp, ado,
                                          data, datalen, roundtrip);
    }
    bool waitCyclicSlot(uint8_t slot, uint64_t token,
                        uint32_t timeout_ns, RxDatagram& out) override {
        return master_.waitCyclicSlot(slot, token, timeout_ns, out);
    }
    bool waitCyclicSlotView(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns,
                            CyclicSlotView& out) override {
        return master_.waitCyclicSlotView(slot, token, timeout_ns, out);
    }
    uint32_t waitCyclicSlotMask(uint32_t slot_mask, const uint64_t* tokens,
                                uint32_t timeout_ns,
                                CyclicSlotView* views) override {
        return master_.waitCyclicSlotMask(slot_mask, tokens, timeout_ns,
                                          views);
    }
    uint8_t* acquireCyclicTxFrame() override {
        return master_.acquireCyclicTxFrame();
    }
    void composeCyclicHeader(uint8_t* frame, Command cmd,
                             uint8_t slot, uint16_t adp,
                             uint16_t ado, uint16_t datalen,
                             bool roundtrip) override {
        master_.composeCyclicHeader(frame, cmd, slot, adp, ado,
                                    datalen, roundtrip);
    }
    bool sendCyclicFrame(uint32_t frame_len) override {
        return master_.sendCyclicFrame(frame_len);
    }
    ICyclicChannel* cyclicChannel() override {
        return master_.cyclicChannel();
    }

private:
    Master& master_;
};

// ============================================================================
// MasterFaultTransport — adapts Master to IFaultTransport
// ============================================================================

class MasterFaultTransport : public IFaultTransport {
public:
    explicit MasterFaultTransport(Master& master) : master_(master) {}

    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size) override {
        return master_.readRegister(SlaveAddress(slave_index), reg_addr, data, size, 50);
    }

    bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                       const void* data, uint16_t size) override {
        return master_.writeRegister(SlaveAddress(slave_index), reg_addr, data, size, 50);
    }

    uint64_t getTimestampMs() override {
        auto now = std::chrono::steady_clock::now();
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count());
    }

    void delayMs(uint32_t ms) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

private:
    Master& master_;
};

// ============================================================================
// MasterSDOTransport — adapts Master to ISDOTransport
// ============================================================================

class Master::MasterSDOTransport : public ::EtherCAT::SDO::ISDOTransport {
public:
    explicit MasterSDOTransport(Master& master)
        : master_(master) {}

    bool sdoUpload(uint16_t slave_index, uint8_t* mbx_counter,
                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                   uint16_t index, uint8_t sub,
                   uint8_t* out, size_t out_cap, size_t* out_len,
                   bool diag_enabled = false,
                   unsigned int poll_interval_ms = 5,
                   unsigned int transaction_timeout_ms = 1000) override
    {
        uint16_t adp = Master::adpForSlaveIndex(slave_index);
        return master_.coeSdoUpload(adp, mbx_counter,
                                    mbx_wr_addr, mbx_wr_len,
                                    mbx_rd_addr, mbx_rd_len,
                                    index, sub, out, out_cap, out_len, diag_enabled,
                                    poll_interval_ms, transaction_timeout_ms);
    }

    bool sdoDownload(uint16_t slave_index, uint8_t* mbx_counter,
                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                     uint16_t index, uint8_t sub,
                     const uint8_t* data, size_t data_len,
                     bool diag_enabled = false,
                     unsigned int poll_interval_ms = 5,
                     unsigned int transaction_timeout_ms = 1000) override
    {
        uint16_t adp = Master::adpForSlaveIndex(slave_index);
        return master_.coeSdoDownload(adp, mbx_counter,
                                      mbx_wr_addr, mbx_wr_len,
                                      mbx_rd_addr, mbx_rd_len,
                                      index, sub, data, data_len, diag_enabled,
                                      poll_interval_ms, transaction_timeout_ms);
    }

    uint64_t getMicroseconds() override {
        return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds());
    }

    bool readSlaveRegister(uint16_t slave_index, uint16_t reg_addr,
                           void* out, uint16_t len,
                           unsigned int timeout_ms) override {
        return master_.readRegister(SlaveAddress(slave_index), reg_addr, out, len, timeout_ms);
    }

    uint32_t lastAbortCode() const override {
        return master_.lastCoeSdoAbortCode();
    }

    bool isCancelRequested() const override {
        return master_.isCancelRequested();
    }

private:
    Master& master_;
};

std::unique_ptr<IPDOTransport> makeMasterPDOTransport(Master& master) {
    return std::make_unique<MasterPDOTransport>(master);
}

std::unique_ptr<IFaultTransport> makeMasterFaultTransport(Master& master) {
    return std::make_unique<MasterFaultTransport>(master);
}

std::unique_ptr<SDO::ISDOTransport> makeMasterSDOTransport(Master& master) {
    return std::make_unique<Master::MasterSDOTransport>(master);
}

} // namespace EtherCAT
