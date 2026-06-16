/**
 * @file test_as715n_errc11_integration.cpp
 * @brief Integration test: AS715N emulated slave reporting ErC1.1 fault
 *
 * Wires a full Master to a software slave emulator via
 * LinuxPairedNetworkInterface and verifies that the AS715N fault-detection
 * stack correctly parses the ErC1.1 (Synchronization loss / 0x0C11) fault.
 *
 * All register values are taken directly from the hardware capture log.
 * Note: this specific AS715N device maps mailbox SMs per the standard ETG convention:
 *   MbxIn  (Receive, M→S) at SM0: addr=0x1000, len=256, ctrl=0x26
 *   MbxOut (Send,    S→M) at SM1: addr=0x1400, len=256, ctrl=0x22
 *
 *   SDO 0x6041:00 StatusWord  = 0x1638  (fault bit set)
 *   SDO 0x203F:00 U32         = 0x00000C11  (external=0x0C11, internal=0x0000)
 *   SDO 0x603F:00             = 0x8700  (CiA402 EtherCAT communication error)
 *
 * The slave emulator:
 *  - Responds to BRD / APRD / APWR commands for bus discovery and state management.
 *  - Emulates the SII EEPROM (returning correct mailbox words 20–24).
 *  - Implements CoE SDO upload (read) responses over the mailbox with
 *    the exact data values above.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715NErrors.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/ethercat/LinuxPairedNetworkInterface.hpp"
#include "tether/platform/EspCompat.hpp"

// Internal EtherCAT protocol structures (CoE/mailbox definitions)
#include "ethercat/raw/internal.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Raw;
using namespace EtherCAT::Drives;

// ============================================================================
// Emulated AS715N slave: ErC1.1 fault state
// ============================================================================

/**
 * @brief Software EtherCAT slave that presents the AS715N in ErC1.1 fault state.
 *
 * Sits on side-B of a LinuxPairedNetworkInterface and responds to all frames
 * sent by the master (side-A).  Implements just enough of the EtherCAT ESC
 * register map and CoE mailbox protocol to let AS715NFaultHandler work
 * end-to-end without real hardware.
 *
 * ## SDO objects emulated
 *
 * | Object   | Value      | Description                              |
 * |----------|-----------|------------------------------------------|
 * | 0x6041:0 | 0x1638    | StatusWord — FAULT bit (bit 3) = 1       |
 * | 0x203F:0 | 0x00000C11| Manufacturer fault — external code 0x0C11|
 * | 0x603F:0 | 0x8700    | CiA402 error — EtherCAT comm class       |
 *
 * ## Slave register mapping (mirrors real hardware)
 *
 * | Register | Value               | Description                 |
 * |----------|--------------------|-----------------------------|
 * | 0x0130   | al_state_ (U16)   | AL Status                   |
 * | 0x0800   | SM0 config (8 B)  | start=0x1000 len=256 ctrl=0x26 (device: MbxIn) |
 * | 0x0808   | SM1 config (8 B)  | start=0x1400 len=256 ctrl=0x22 (device: MbxOut) |
 * | 0x0502   | EEPROM ctrl/stat  | busy=0, errors=0            |
 * | 0x0508   | EEPROM data (4 B) | two SII words at sii_addr_  |
 * | 0x1000   | MbxIn  (WR)       | CoE SDO requests from master |
 * | 0x1400   | MbxOut (RD)       | CoE SDO responses to master  |
 */
class AS715NErC11SlaveResponder {
public:
    explicit AS715NErC11SlaveResponder(LinuxPairedNetworkInterface& pair)
        : pair_(pair)
    {
        pair_.setRxCallbackB([this](const uint8_t* data, size_t len) {
            handleFrame(data, len);
        });
    }

    ~AS715NErC11SlaveResponder() {
        pair_.setRxCallbackB(nullptr);
    }

private:
    // ---- Mailbox addresses (match real AS715N hardware) --------------------
    static constexpr uint16_t kSmRecvAddr = 0x1000;  ///< MbxIn:  master writes SDO request (on this device, mapped to SM0)
    static constexpr uint16_t kSmSendAddr = 0x1400;  ///< MbxOut: master reads SDO response (on this device, mapped to SM1)
    static constexpr uint16_t kSmLen      = 256;

    // ---- Fault register values from hardware capture -----------------------
    static constexpr uint16_t kStatusWord    = 0x1638u;     ///< 0x6041:0 — FAULT set
    static constexpr uint32_t kMfrFault203F  = 0x00000C11u; ///< 0x203F:0 — external=ErC1.1
    static constexpr uint16_t kCiA402Error   = 0x8700u;     ///< 0x603F:0 — comm error

    // ---- Frame handler -----------------------------------------------------

    void handleFrame(const uint8_t* data, size_t len) {
        constexpr size_t kMinLen = 14 + 2 + 10 + 2;
        if (len < kMinLen) return;

        std::vector<uint8_t> reply(data, data + len);

        // Swap Ethernet src ↔ dst so the master recognises the reply
        std::swap_ranges(reply.begin(), reply.begin() + 6, reply.begin() + 6);

        auto* dg = reinterpret_cast<EtherCATDatagramHeader*>(reply.data() + 14 + 2);
        const uint16_t datalen_raw = le16_to_host(dg->lenFlags.raw_le) & 0x07FFu;
        const size_t   wkc_offset  = 14 + 2 + 10 + datalen_raw;
        if (wkc_offset + 2 > len) return;

        auto*    wkc_ptr = reinterpret_cast<uint16_t*>(reply.data() + wkc_offset);
        uint8_t* payload = reply.data() + 14 + 2 + 10;

        const auto     cmd = static_cast<Command>(static_cast<uint8_t>(dg->cmd));
        const uint16_t adp = le16_to_host(dg->adp_le);
        const uint16_t ado = le16_to_host(dg->ado_le);
        uint16_t       wkc = 0;

        switch (cmd) {
        case Command::BRD:
            // Broadcast read — one slave responds
            wkc = 1;
            fillRegister(payload, datalen_raw, ado);
            break;

        case Command::BWR:
            // Broadcast write — one slave accepts
            wkc = 1;
            applyWrite(ado, payload, datalen_raw);
            break;

        case Command::APRD: {
            // Auto-increment read: slave_idx = (0 - adp)
            const uint16_t slave_idx = static_cast<uint16_t>(0u - adp);
            if (slave_idx == 0) {
                wkc = 1;
                fillRegister(payload, datalen_raw, ado);
                dg->adp_le = host_to_le16(static_cast<uint16_t>(adp + 1));
            }
            break;
        }

        case Command::APWR: {
            const uint16_t slave_idx = static_cast<uint16_t>(0u - adp);
            if (slave_idx == 0) {
                wkc = 1;
                applyWrite(ado, payload, datalen_raw);
                dg->adp_le = host_to_le16(static_cast<uint16_t>(adp + 1));
            }
            break;
        }

        default:
            break;
        }

        *wkc_ptr = host_to_le16(wkc);
        pair_.ifaceB().send(reply.data(), reply.size());
    }

    // ---- Register reads (payload fill for APRD / BRD) ---------------------

    void fillRegister(uint8_t* buf, uint16_t len, uint16_t ado) {
        if (!buf || len == 0) return;
        std::memset(buf, 0, len);

        // AL Status (0x0130) — current EtherCAT state
        if (ado == 0x0130 && len >= 2) {
            buf[0] = static_cast<uint8_t>(al_state_ & 0xFF);
            buf[1] = static_cast<uint8_t>((al_state_ >> 8) & 0xFF);
            return;
        }

        // DL Status (0x0110) — report port 0 active (needed for counting slaves)
        if (ado == 0x0110 && len >= 2) {
            buf[0] = 0xB4; buf[1] = 0x00;
            return;
        }

        // SM0 config (0x0800, 8 bytes) — on this device: MbxIn (M→S)
        if (ado == 0x0800 && len >= 8) {
            buf[0] = 0x00; buf[1] = 0x10;  // start = 0x1000
            buf[2] = 0x00; buf[3] = 0x01;  // len   = 256
            buf[4] = 0x26;                  // ctrl  = 0x26 (mailbox, M→S)
            buf[5] = 0x00;                  // status
            buf[6] = 0x01;                  // activate enabled
            buf[7] = 0x00;
            return;
        }

        // SM0 status byte (0x0805) — idle / not busy
        if (ado == 0x0805 && len >= 1) {
            buf[0] = 0x00;
            return;
        }

        // SM1 config (0x0808, 8 bytes) — on this device: MbxOut (S→M)
        if (ado == 0x0808 && len >= 8) {
            buf[0] = 0x00; buf[1] = 0x14;  // start = 0x1400
            buf[2] = 0x00; buf[3] = 0x01;  // len   = 256
            buf[4] = 0x22;                  // ctrl  = 0x22 (mailbox, S→M)
            buf[5] = 0x00;
            buf[6] = 0x01;
            buf[7] = 0x00;
            return;
        }

        // SM1 status byte (0x080D) — idle
        if (ado == 0x080D && len >= 1) {
            buf[0] = 0x00;
            return;
        }

        // EEPROM access mode (0x0500) — 0x01: word-addressed, ECAT master access OK
        if (ado == 0x0500 && len >= 2) {
            buf[0] = 0x01; buf[1] = 0x00;
            return;
        }

        // EEPROM ctrl/status (0x0502) — not busy (bit15=0), no errors
        if (ado == 0x0502 && len >= 2) {
            buf[0] = 0x00; buf[1] = 0x00;
            return;
        }

        // EEPROM data (0x0508, 4 bytes = 2 SII words at sii_addr_)
        if (ado == 0x0508 && len >= 4) {
            const uint16_t w0 = siiWord(sii_addr_);
            const uint16_t w1 = siiWord(static_cast<uint16_t>(sii_addr_ + 1u));
            buf[0] = static_cast<uint8_t>(w0 & 0xFF);
            buf[1] = static_cast<uint8_t>((w0 >> 8) & 0xFF);
            buf[2] = static_cast<uint8_t>(w1 & 0xFF);
            buf[3] = static_cast<uint8_t>((w1 >> 8) & 0xFF);
            return;
        }

        // SM1 mailbox read area (0x1400) — return SDO response if ready
        if (ado == kSmSendAddr) {
            if (sdo_response_ready_) {
                const size_t n = (sdo_resp_len_ < static_cast<size_t>(len))
                                 ? sdo_resp_len_ : static_cast<size_t>(len);
                std::memcpy(buf, sdo_response_, n);
                sdo_response_ready_ = false;  // consume — master will not re-read
            }
            // else: buffer stays zeroed → master sees length=0 and retries
            return;
        }
    }

    // ---- Register writes (APWR / BWR payload handling) --------------------

    void applyWrite(uint16_t ado, const uint8_t* data, uint16_t len) {
        // AL Control (0x0120) — accept state transition, mirror in al_state_
        if (ado == 0x0120 && len >= 2) {
            uint16_t val;
            std::memcpy(&val, data, 2);
            al_state_ = static_cast<uint16_t>(le16_to_host(val) & 0x0Fu);
            return;
        }

        // EEPROM control write (0x0502, 6 bytes) — extract embedded word address
        // Structure: [comm_le(2)] [addr_le(2)] [d2_le(2)]
        // Read command (EC_ECMD_READ=0x0100) embeds the word address at offset 2.
        if (ado == 0x0502 && len >= 4) {
            uint16_t word_addr;
            std::memcpy(&word_addr, data + 2, 2);
            sii_addr_ = le16_to_host(word_addr);
            return;
        }

        // SM0 mailbox write (0x1000) — CoE SDO request from master
        constexpr size_t kMbxMinLen =
            sizeof(MbxHeader) + sizeof(CoeHeader) + 8u;  // 6+2+8 = 16
        if (ado == kSmRecvAddr && len >= static_cast<uint16_t>(kMbxMinLen)) {
            processMailboxRequest(data, len);
            return;
        }

        // SM1 mailbox clear (master writes zeros to invalidate stale response)
        if (ado == kSmSendAddr) {
            sdo_response_ready_ = false;
            return;
        }

        // All other writes (SM config, FMMU, EEPCTL NOP, etc.) — silently accepted
    }

    // ---- CoE SDO mailbox protocol -----------------------------------------

    /**
     * @brief Parse an incoming CoE mailbox frame and dispatch to SDO handlers.
     *
     * The master writes the SDO upload (read) or download (write) request to
     * SM0 (0x1000).  We parse the frame, look up the requested object, and
     * pre-build a response that will be returned on the next APRD to SM1.
     */
    void processMailboxRequest(const uint8_t* data, uint16_t len) {
        const auto* mbx = reinterpret_cast<const MbxHeader*>(data);
        const uint16_t mbx_len  = le16_to_host(mbx->length_le);
        const uint8_t  mbx_type = mbx->mbxtype & 0x0Fu;
        const uint8_t  mbx_cnt  =
            static_cast<uint8_t>((mbx->mbxtype >> 4) & 0x0Fu);

        if (mbx_type != EC_MBXT_COE) return;
        if (static_cast<size_t>(sizeof(MbxHeader)) + mbx_len >
            static_cast<size_t>(len)) return;

        const auto* coe = reinterpret_cast<const CoeHeader*>(
            data + sizeof(MbxHeader));
        const uint16_t coe_raw     = le16_to_host(coe->raw_le);
        const uint16_t coe_number  = coe_raw & 0x01FFu;
        const uint8_t  coe_service =
            static_cast<uint8_t>((coe_raw >> 12) & 0x0Fu);

        if (coe_service != EC_COES_SDOREQ) return;
        if (mbx_len < static_cast<uint16_t>(sizeof(CoeHeader) + 8u)) return;

        const uint8_t* sdo    = data + sizeof(MbxHeader) + sizeof(CoeHeader);
        const uint8_t  cmd    = sdo[0];
        uint16_t       idx;
        std::memcpy(&idx, sdo + 1, 2);
        idx             = le16_to_host(idx);
        const uint8_t  sub    = sdo[3];

        if (cmd == 0x40u) {
            // SDO Upload Initiate (read) request
            buildUploadResponse(coe_number, mbx_cnt, idx, sub);
        } else if ((cmd & 0xE0u) == 0x20u) {
            // SDO Download Initiate (write) request — acknowledge only
            buildDownloadAck(coe_number, mbx_cnt, idx, sub);
        }
        // Other commands (segmented transfers etc.) not needed for this test
    }

    /**
     * @brief Build an expedited CoE SDO upload response for a known object.
     *
     * The SDO command byte encoding for expedited upload response:
     *   bits[7:5] scs=2 (server command specifier: upload response)
     *   bit[4]    reserved=0
     *   bits[3:2] n: number of unused data bytes (0=U32, 2=U16, 3=U8)
     *   bit[1]    e=1 (expedited)
     *   bit[0]    s=1 (size indicated)
     *   → U32: 0x43   U16: 0x4B   U8: 0x4F
     */
    void buildUploadResponse(uint16_t coe_number, uint8_t mbx_cnt,
                             uint16_t idx, uint8_t sub) {
        uint8_t data4[4]   = {};
        uint8_t n_unused   = 2;   // default: U16 (2 unused bytes in 4-byte field)
        bool    known      = true;

        if (idx == 0x6041u && sub == 0x00u) {
            // StatusWord U16 = 0x1638
            data4[0] = static_cast<uint8_t>(kStatusWord & 0xFF);
            data4[1] = static_cast<uint8_t>((kStatusWord >> 8) & 0xFF);
            n_unused = 2;
        } else if (idx == 0x203Fu && sub == 0x00u) {
            // Manufacturer fault U32 = 0x00000C11
            data4[0] = static_cast<uint8_t>(kMfrFault203F & 0xFF);
            data4[1] = static_cast<uint8_t>((kMfrFault203F >> 8) & 0xFF);
            data4[2] = static_cast<uint8_t>((kMfrFault203F >> 16) & 0xFF);
            data4[3] = static_cast<uint8_t>((kMfrFault203F >> 24) & 0xFF);
            n_unused = 0;  // U32 — all 4 bytes carry data
        } else if (idx == 0x603Fu && sub == 0x00u) {
            // CiA402 error U16 = 0x8700
            data4[0] = static_cast<uint8_t>(kCiA402Error & 0xFF);
            data4[1] = static_cast<uint8_t>((kCiA402Error >> 8) & 0xFF);
            n_unused = 2;
        } else {
            known = false;
        }

        if (!known) {
            // SDO Abort: object does not exist (0x06020000)
            buildAbortResponse(coe_number, mbx_cnt, idx, sub, 0x06020000u);
            return;
        }

        // scs=2, e=1, s=1, n=n_unused  →  0x40 | (n << 2) | 0x03
        const uint8_t sdo_cmd =
            static_cast<uint8_t>(0x40u | ((n_unused & 0x03u) << 2u) | 0x03u);
        buildMbxResponse(coe_number, mbx_cnt, EC_COES_SDORES, sdo_cmd, idx, sub, data4);
    }

    /// Build a download (write) acknowledgement response (cmd = 0x60).
    void buildDownloadAck(uint16_t coe_number, uint8_t mbx_cnt,
                          uint16_t idx, uint8_t sub) {
        constexpr uint8_t zeros[4] = {};
        buildMbxResponse(coe_number, mbx_cnt, EC_COES_SDORES, 0x60u, idx, sub, zeros);
    }

    /// Build a SDO abort response (cmd = 0x80, data = abort code LE).
    void buildAbortResponse(uint16_t coe_number, uint8_t mbx_cnt,
                            uint16_t idx, uint8_t sub, uint32_t abort_code) {
        uint8_t data4[4];
        const uint32_t ac = host_to_le32(abort_code);
        std::memcpy(data4, &ac, 4);
        buildMbxResponse(coe_number, mbx_cnt, EC_COES_SDORES, 0x80u, idx, sub, data4);
    }

    /**
     * @brief Serialise a CoE SDO response frame into sdo_response_[].
     *
     * Frame layout:
     *   [MbxHeader 6B] [CoeHeader 2B] [SDO body 8B] [padding to kSmLen]
     */
    void buildMbxResponse(uint16_t coe_number, uint8_t mbx_cnt,
                          uint8_t coe_service, uint8_t sdo_cmd,
                          uint16_t idx, uint8_t sub, const uint8_t data4[4]) {
        uint8_t* buf = sdo_response_;
        std::memset(buf, 0, kSmLen);

        // Mailbox header — 6 bytes
        const uint16_t payload_len =
            static_cast<uint16_t>(sizeof(CoeHeader) + 8u);
        MbxHeader mbx{};
        mbx.length_le  = host_to_le16(payload_len);
        mbx.address_le = host_to_le16(0);
        mbx.priority   = 0;
        mbx.mbxtype    = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);
        std::memcpy(buf, &mbx, sizeof(mbx));

        // CoE header — 2 bytes: echo coe_number, set service to SDORES
        CoeHeader coe{};
        coe.raw_le = host_to_le16(coe_make_raw(coe_number, coe_service));
        std::memcpy(buf + sizeof(mbx), &coe, sizeof(coe));

        // SDO body — 8 bytes: cmd, index LE (2), sub, data (4)
        uint8_t* sdo = buf + sizeof(mbx) + sizeof(coe);
        sdo[0] = sdo_cmd;
        sdo[1] = static_cast<uint8_t>(idx & 0xFF);
        sdo[2] = static_cast<uint8_t>((idx >> 8) & 0xFF);
        sdo[3] = sub;
        if (data4) std::memcpy(sdo + 4, data4, 4);

        sdo_resp_len_       = sizeof(mbx) + sizeof(coe) + 8u;
        sdo_response_ready_ = true;
    }

    // ---- Minimal SII EEPROM image -----------------------------------------

    /**
     * @brief Return the SII word at a given word address.
     *
     * Words 20–24 carry the standard mailbox configuration that the master
     * reads during discovery and autoConfigureMailbox().  All other words
     * return 0 (causing a CRC mismatch at word 7, which the SII reader
     * tolerates and treats as non-fatal).
     *
     * | Word | Value  | Meaning                         |
     * |------|--------|---------------------------------|
     * |  8   | 0x0000 | Vendor ID low  (0x00400000)     |
     * |  9   | 0x0040 | Vendor ID high                  |
     * | 10   | 0x0715 | Product Code low (0x00000715)   |
     * | 11   | 0x0000 | Product Code high               |
     * | 20   | 0x1000 | Std RX mailbox offset (MbxIn/M→S) |
     * | 21   | 0x0100 | Std RX mailbox size = 256          |
     * | 22   | 0x1400 | Std TX mailbox offset (MbxOut/S→M) |
     * | 23   | 0x0100 | Std TX mailbox size = 256          |
     * | 24   | 0x000C | Protocols: CoE (bit2) + FoE (bit3)|
     */
    static uint16_t siiWord(uint16_t word_addr) {
        switch (word_addr) {
            case  8: return 0x0000;  // Vendor ID low  (→ 0x00400000)
            case  9: return 0x0040;  // Vendor ID high
            case 10: return 0x0715;  // Product Code low (→ 0x00000715)
            case 11: return 0x0000;  // Product Code high
            case 20: return 0x1000;  // Std mailbox receive offset (MbxIn)
            case 21: return 0x0100;  // Std mailbox receive size = 256
            case 22: return 0x1400;  // Std mailbox transmit offset (MbxOut)
            case 23: return 0x0100;  // Std mailbox transmit size = 256
            case 24: return 0x000C;  // Mailbox protocols: CoE + FoE
            default: return 0x0000;
        }
    }

    // ---- State ---------------------------------------------------------------

    LinuxPairedNetworkInterface& pair_;

    /// Current AL state (written via 0x0120, read via 0x0130)
    uint16_t al_state_ = 0x0001;  // INIT at startup

    /// Last SII word address set by the EEPROM read command embedded in 0x0502
    uint16_t sii_addr_ = 0x0000;

    /// Pending SDO response buffer (served on next APRD to SM1/0x1400)
    uint8_t  sdo_response_[kSmLen] = {};
    size_t   sdo_resp_len_         = 0;
    bool     sdo_response_ready_   = false;
};

// ============================================================================
// Test fixture
// ============================================================================

class AS715NErC11IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        pair_ = std::make_unique<LinuxPairedNetworkInterface>();
    }

    void TearDown() override {
        if (master_) master_->stop();
        pair_.reset();
    }

    Master& startMaster() {
        master_ = std::make_unique<Master>();
        pair_->setRxCallbackA([this](const uint8_t* data, size_t len) {
            if (master_) master_->handleRxFrame(data, len);
        });
        master_->start(pair_->ifaceA(), kDummyMac);
        return *master_;
    }

    /// Spin until at least one slave appears or the timeout elapses.
    bool waitForDiscovery(Master& master, int max_ms = 3000) {
        for (int ms = 0; ms < max_ms; ms += 25) {
            if (master.getDiscoveredSlaveCount() > 0) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        return false;
    }

    std::unique_ptr<LinuxPairedNetworkInterface> pair_;
    std::unique_ptr<Master>              master_;
    static constexpr uint8_t kDummyMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
};

// ============================================================================
// Tests
// ============================================================================

/**
 * @test AS715NErC11IntegrationTest/Discovery
 *
 * Verifies that the master discovers exactly one slave when connected to
 * the AS715N emulator.
 */
TEST_F(AS715NErC11IntegrationTest, Discovery) {
    AS715NErC11SlaveResponder responder(*pair_);
    auto& master = startMaster();

    ASSERT_TRUE(waitForDiscovery(master))
        << "Master did not discover the emulated slave within the timeout";
    EXPECT_EQ(master.getDiscoveredSlaveCount(), 1u);
}

/**
 * @test AS715NErC11IntegrationTest/CheckFaultDetectsErC11
 *
 * End-to-end integration test of the full AS715N fault inspection path:
 *
 *  1. Master discovers the emulated slave.
 *  2. AS715NFaultHandler::checkFault() reads fault codes via CoE SDO.
 *  3. The raw register values match what the real hardware returns.
 *
 * This mirrors exactly what `as715n_check_error_code -i eth0` (or `--interface eth0`) does,
 * but with a software slave instead of real hardware.
 */
TEST_F(AS715NErC11IntegrationTest, CheckFaultDetectsErC11) {
    AS715NErC11SlaveResponder responder(*pair_);
    auto& master = startMaster();

    ASSERT_TRUE(waitForDiscovery(master)) << "No slaves discovered";
    ASSERT_EQ(master.getDiscoveredSlaveCount(), 1u);

    // Configure mailbox addresses in the SDO manager (may use SII or defaults)
    master.autoConfigureMailbox(0);

    // Ensure slave is in PRE_OP (required for SDO access via mailbox)
    ASSERT_TRUE(master.transitionSlaveToPreOperational(0))
        << "Failed to transition emulated slave to PRE_OP";

    uint16_t mfr_error    = 0xFFFFu;
    uint16_t cia402_error = 0xFFFFu;
    const bool has_fault = AS715NFaultHandler::checkFault(
        master.sdoManager(), 0, &mfr_error, &cia402_error);

    // ---- Correctness assertions ----
    EXPECT_TRUE(has_fault)
        << "checkFault() must detect a fault: StatusWord=0x1638 has bit3 (FAULT) set";
    EXPECT_EQ(mfr_error, 0x0C11u)
        << "Manufacturer error must be 0x0C11 (external code for ErC1.1)";
    EXPECT_EQ(cia402_error, 0x8700u)
        << "CiA402 error must be 0x8700 (EtherCAT communication error class)";
}

/**
 * @test AS715NErC11IntegrationTest/ErrorParsedCorrectly
 *
 * Verifies that the 0x0C11 raw code returned by the emulated slave is parsed
 * into the correct human-readable representation that matches:
 *  - The drive display:  "ErC1.1"
 *  - The fault manual:   "Synchronization loss" (resettable, DC sync class)
 *
 * This combines the integration test (real SDO exchange) with error-parsing
 * assertions that serve as a regression guard against future encoding changes.
 */
TEST_F(AS715NErC11IntegrationTest, ErrorParsedCorrectly) {
    AS715NErC11SlaveResponder responder(*pair_);
    auto& master = startMaster();

    ASSERT_TRUE(waitForDiscovery(master));
    master.autoConfigureMailbox(0);
    ASSERT_TRUE(master.transitionSlaveToPreOperational(0));

    uint16_t mfr_error    = 0;
    uint16_t cia402_error = 0;
    AS715NFaultHandler::checkFault(
        master.sdoManager(), 0, &mfr_error, &cia402_error);

    // Parse exactly as the production firmware path does
    const AS715NError err = AS715NError::parse(mfr_error);

    EXPECT_STREQ(err.name, "ErC1.1")
        << "Drive display shows ErC1.1 — name must match";
    EXPECT_EQ(err.class_code, 0xC1u)
        << "class_code = upper nibble pair (C1 = hex digits 'C' and '1')";
    EXPECT_EQ(err.sub_code, 0x01u)
        << "sub_code = lower nibble (1)";
    EXPECT_STREQ(err.description, "Synchronization loss")
        << "Description must match the manual entry for ErC1.1";
    EXPECT_TRUE(err.is_recoverable)
        << "ErC1.1 is marked 'Resettable' in the AS715N fault table";
    EXPECT_TRUE(err.isDCSyncError())
        << "ErC1.1 is an EtherCAT Class-C communication error (0x0Cxx)";
    EXPECT_TRUE(err.isNoSyncError())
        << "0x0C11 is the primary synchronization-loss fault code";

    // Also verify the CiA402 error is as observed on hardware
    EXPECT_EQ(cia402_error, 0x8700u)
        << "CiA402 Bus Fault Code 0x8700 per fault table column '603F'";
}
