/**
 * @file PDOManager_exchange.cpp
 * @brief PDOManager — LRW / separate / physical exchange modes.
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
// Exchange modes (formerly pdo_logical.cpp)
// ============================================================================

bool PDOManager::exchangeLRW(uint16_t slave_count) {
    if (logical_addr_mgr_ && logical_addr_mgr_->isInitialized()) {
        return logical_addr_mgr_->exchangeAllLRW(mapping_);
    }
    if (slave_count == 0) return true;
    lrw_stats_.lrw_send_errors++;
    TETHER_LOGW(TAG, "LRW exchange requires FMMU; use master-level API");
    return false;
}

bool PDOManager::exchangeSeparate(uint16_t slave_count) {
    if (slave_count == 0) return true;
    separate_stats_.send_errors++;
    TETHER_LOGW(TAG, "Separate exchange requires FMMU; use master-level API");
    return false;
}

bool PDOManager::exchangePhysical(uint16_t slave_count) {
    if (slave_count == 0) return true;
    bool fpwr_ok = true, fprd_ok = true;

    // Iterate over all configured slaves (not just index 0).
    // slave_count limits the range of indices to scan.
    const uint16_t max_scan = (slave_count < PDO::kMaxPDOSlaves)
                                  ? slave_count : PDO::kMaxPDOSlaves;

    for (uint16_t si = 0; si < max_scan; si++) {
        PDO::SlaveConfig* cfg = &slave_configs_[si];
        if (!cfg || !cfg->configured) continue;

        const auto& sm2 = cfg->sm[2];
        const auto& sm3 = cfg->sm[3];

    // Build the RxPDO output buffer
    uint8_t out_buf[PDO::kMaxPDOSize] = {0};
    bool have_write = false;
    bool should_log_wire = false;
    uint32_t wire_cycle = 0;
    if (sm2.type == PDO::SyncManagerType::ProcessOutput && sm2.length > 0) {
        have_write = true;
        for (size_t i = 0; i < mapping_.entry_count(); i++) {
            const PDO::PDOEntry* e = mapping_.get_entry(i);
            if (e && e->enabled && e->slave_index == si
                && e->direction == PDO::PDODirection::RxPDO
                && e->data_size > 0 && e->data_size <= sm2.length
                && e->physical_offset >= sm2.phys_start_addr
                && e->physical_offset - sm2.phys_start_addr + e->data_size
                       <= sm2.length) {
                std::memcpy(out_buf + (e->physical_offset - sm2.phys_start_addr),
                            e->storage, e->data_size);
            }
        }
        // Periodic wire log: log interpreted RxPDO output every 1000 cycles
        // (skip cycle 0 — fpwr_success==0 makes 0%1000==0 which would
        //  fire every cycle until the first successful write)
        if (physical_stats_.fpwr_success > 0 &&
            (physical_stats_.fpwr_success % 1000) == 0 &&
            (rxPDODebug(si) || txPDODebug(si))) {
            should_log_wire = true;
            wire_cycle = physical_stats_.fpwr_success;
        }
        if (rxPDODebug()) {
            char hex[128];
            size_t pos = 0;
            size_t dump_len = sm2.length < 32 ? sm2.length : 32;
            for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", out_buf[b]));
            }
            TETHER_LOGI(TAG, "[RxPDO-DEBUG] Physical write SM2 ({}): addr=0x{:04x} len={} data={}",
                        slavePrefix(si).c_str(), sm2.phys_start_addr, sm2.length, hex);
        }
    }

    bool have_read = (sm3.type == PDO::SyncManagerType::ProcessInput && sm3.length > 0);

    // Build multi-datagram specs: pack FPWR + FPRD into one frame
    if (have_write && have_read) {
        // Batch both write and read into a single frame.
        // Use the slave's configured station address for FPWR/FPRD (not the
        // auto-increment position).  The configured station address is read
        // from register 0x0010 during configureMultiPDOs().
        // Note: 0x0000 is a valid configured station address — only fall back
        // to auto-increment if the slave was not configured at all.
        const uint16_t adp = cfg->configured
                                 ? cfg->configured_address
                                 : transport_.adpForSlaveIndex(si);
        const uint8_t write_idx = transport_.allocIdx();
        const uint8_t read_idx = transport_.allocIdx();

        // Pre-register response waiter slots BEFORE sending to avoid the
        // send-then-register race: both datagrams share one frame, so both
        // responses arrive in one frame on the RX thread.  If we register
        // slots only after the send returns, the second response arrives
        // with no pending slot and is dropped ("unrouted"), causing a 50 ms
        // timeout per cycle.
        RxDatagram write_resp{};
        RxDatagram read_resp{};
        size_t write_slot = transport_.preRegisterResponseWaiter(
            write_idx, write_resp.data, sizeof(write_resp.data));
        size_t read_slot = transport_.preRegisterResponseWaiter(
            read_idx, read_resp.data, sizeof(read_resp.data));

        MultiDatagramSpec specs[2] = {
            {Command::FPWR, write_idx, adp, sm2.phys_start_addr, out_buf, sm2.length, true},
            {Command::FPRD, read_idx, adp, sm3.phys_start_addr, nullptr, sm3.length, true},
        };

        size_t frames_sent = transport_.sendMultiDatagram(specs, 2);
        if (frames_sent == 0) {
            if (write_slot < IPDOTransport::kPreRegInvalid)
                transport_.waitForPreRegistered(write_slot, 0, write_resp); // cancel
            if (read_slot < IPDOTransport::kPreRegInvalid)
                transport_.waitForPreRegistered(read_slot, 0, read_resp);  // cancel
            physical_stats_.fpwr_wkc_errors++;
            physical_stats_.fprd_wkc_errors++;
            fpwr_ok = false;
            fprd_ok = false;
            continue;
        }

        // Wait for write response
        bool write_ok;
        if (write_slot < IPDOTransport::kPreRegInvalid) {
            write_ok = transport_.waitForPreRegistered(write_slot, 50, write_resp);
        } else {
            write_ok = transport_.waitForResponseIdx(write_idx, 50, write_resp);
        }
        if (write_ok && write_resp.wkc > 0) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }

        // Wait for read response
        bool read_ok;
        if (read_slot < IPDOTransport::kPreRegInvalid) {
            read_ok = transport_.waitForPreRegistered(read_slot, 50, read_resp);
        } else {
            read_ok = transport_.waitForResponseIdx(read_idx, 50, read_resp);
        }
        if (read_ok && read_resp.wkc > 0) {
            physical_stats_.fprd_success++;
            if (txPDODebug()) {
                char hex[128];
                size_t pos = 0;
                size_t dump_len = sm3.length < 32 ? sm3.length : 32;
                for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                    pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", read_resp.data[b]));
                }
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 ({}): addr=0x{:04x} len={} data={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->slave_index == si
                    && e->direction == PDO::PDODirection::TxPDO
                    && e->data_size > 0 && e->data_size <= sm3.length
                    && e->physical_offset >= sm3.phys_start_addr
                    && e->physical_offset - sm3.phys_start_addr + e->data_size
                           <= sm3.length) {
                    std::memcpy(e->storage,
                                read_resp.data + (e->physical_offset - sm3.phys_start_addr),
                                e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied {} bytes to entry {} buf={:p}",
                                    e->data_size, i, static_cast<const void*>(e->storage));
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED ({}): addr=0x{:04x} len={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length);
            }
        }
        // Periodic interpreted RxPDO/TxPDO wire log
        if (should_log_wire && write_ok && (read_ok && read_resp.wkc > 0)) {
            static const Utils::BitLabel kCwLabels[] = {
                Utils::BitLabel::bit("SwOn",   0x0001),
                Utils::BitLabel::bit("EnV",    0x0002),
                Utils::BitLabel::bit("NoQS",   0x0004),
                Utils::BitLabel::bit("EnOp",   0x0008),
                Utils::BitLabel::bit("NewSP",  0x0010),
                Utils::BitLabel::bit("ChgSI",  0x0020),
                Utils::BitLabel::bit("AbsRel", 0x0040),
                Utils::BitLabel::bit("FltR",   0x0080),
                Utils::BitLabel::bit("Halt",   0x0100),
            };
            const std::span<const Utils::BitLabel> kCwSpan(kCwLabels);
            Utils::ColoredBitsetFormatter cw_fmt(kCwSpan);
            const uint32_t known_cw = Utils::ColoredBitsetFormatter::labelCoverage(kCwSpan);

            // Resolve the PDO indices actually assigned to this slave so the
            // decode matches the configured mapping (AS715N slave-defined
            // set: Rx 0x1701-0x1705, Tx 0x1B01-0x1B04).
            uint16_t rxpdo_index = 0;
            uint16_t txpdo_index = 0;
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                const PDO::PDOEntry* e = mapping_.get_entry(i);
                if (!e || !e->enabled || e->slave_index != si) continue;
                if (e->direction == PDO::PDODirection::RxPDO) rxpdo_index = e->pdo_index;
                else                                        txpdo_index = e->pdo_index;
            }

            // AS715N RxPDO layouts: 0x6071 TargetTorque and 0x6060
            // ModesOfOperation sit at different offsets (or are absent)
            // depending on the assigned PDO.  -1 = field not present.
            int tq_off = -1, opmode_off = -1;
            bool has_tv = true;
            switch (rxpdo_index) {
                case 0x1701: has_tv = false; break;                // CW,TP,TPF,DO
                case 0x1702:
                case 0x1704: tq_off = 10; opmode_off = 12; break;  // +TT,Mode,TPF,...
                case 0x1703:
                case 0x1705: opmode_off = 10; break;               // Mode,TPF,+/-TqLim,...
                default: break;
            }

            // Bounds-checked little-endian field readers.
            auto rd8 = [](const uint8_t* b, uint16_t len, int off) -> int8_t {
                return (off >= 0 && static_cast<uint16_t>(off) < len)
                    ? static_cast<int8_t>(b[off]) : static_cast<int8_t>(0);
            };
            auto rd16 = [](const uint8_t* b, uint16_t len, int off) -> uint16_t {
                return (off >= 0 && static_cast<uint16_t>(off) + 2 <= len)
                    ? static_cast<uint16_t>(b[off] | (b[off + 1] << 8))
                    : static_cast<uint16_t>(0);
            };
            auto rd32 = [](const uint8_t* b, uint16_t len, int off) -> int32_t {
                return (off >= 0 && static_cast<uint16_t>(off) + 4 <= len)
                    ? static_cast<int32_t>(b[off] | (b[off + 1] << 8) |
                                           (b[off + 2] << 16) | (b[off + 3] << 24))
                    : static_cast<int32_t>(0);
            };

            const uint16_t cw = rd16(out_buf, sm2.length, 0);
            const std::string cw_state = cw_fmt.format(cw, " ", known_cw);

            const int32_t tp = rd32(out_buf, sm2.length, 2);
            const int32_t tv = has_tv ? rd32(out_buf, sm2.length, 6) : 0;
            const int16_t tq = static_cast<int16_t>(rd16(out_buf, sm2.length, tq_off));
            const int8_t opmode = rd8(out_buf, sm2.length, opmode_off);

            TETHER_LOGI(TAG, "[RxPDO 0x{:04X}] {} Cycle {}:", rxpdo_index, slavePrefix(si).c_str(), wire_cycle);
            TETHER_LOGI(TAG, "  Controlword: {} (0x{:04X})", cw_state, cw);

            // Print the demand value matching the active operating mode plus
            // the raw setpoints.  PDOs without a mode field (e.g. 0x1701)
            // print the raw targets only.
            if (opmode_off >= 0 && opmode == 8) {
                TETHER_LOGI(TAG, "  Mode=CSP(8) DemandPosition={:>10} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tp, tp, tv, tq);
            } else if (opmode_off >= 0 && opmode == 9) {
                TETHER_LOGI(TAG, "  Mode=CSV(9) DemandVelocity={:>10} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tv, tp, tv, tq);
            } else if (opmode_off >= 0 && opmode == 10) {
                TETHER_LOGI(TAG, "  Mode=CST(10) DemandTorque={:>6} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tq, tp, tv, tq);
            } else if (opmode_off >= 0) {
                TETHER_LOGI(TAG, "  Mode={} | TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            opmode, tp, tv, tq);
            } else {
                TETHER_LOGI(TAG, "  TargetPosition={:>10} TargetVelocity={:>10} TargetTorque={:>6}",
                            tp, tv, tq);
            }

            static const Utils::BitLabel kSwLabels[] = {
                Utils::BitLabel::bit("Rdy",     0x0001),
                Utils::BitLabel::bit("SwOn",    0x0002),
                Utils::BitLabel::bit("EnOp",    0x0004),
                Utils::BitLabel::bit("Flt",     0x0008),
                Utils::BitLabel::bit("EnV",     0x0010),
                Utils::BitLabel::bit("NoQS",    0x0020),
                Utils::BitLabel::bit("SwOnDsbl",0x0040),
                Utils::BitLabel::bit("Wrn",     0x0080),
                Utils::BitLabel::bit("Rem",     0x0200),
                Utils::BitLabel::bit("TgtRec",  0x0400),
                Utils::BitLabel::bit("IntLim",  0x0800),
                Utils::BitLabel::bit("SetAck",  0x1000),
                Utils::BitLabel::bit("FolErr",  0x2000),
            };
            const std::span<const Utils::BitLabel> kSwSpan(kSwLabels);
            Utils::ColoredBitsetFormatter sw_fmt(kSwSpan);
            const uint32_t known_sw = Utils::ColoredBitsetFormatter::labelCoverage(kSwSpan);
            const uint16_t sw = rd16(read_resp.data, sm3.length, 2);
            const std::string sw_state = sw_fmt.format(sw, " ", known_sw);

            const int32_t ap = rd32(read_resp.data, sm3.length, 4);
            const int16_t at = static_cast<int16_t>(rd16(read_resp.data, sm3.length, 8));
            // The trailing 4-byte slot is 0x606C speed feedback on 0x1B04
            // and 0x60FD digital inputs on 0x1B01/0x1B02/0x1B03.
            const bool last_is_di = (txpdo_index == 0x1B01 ||
                                     txpdo_index == 0x1B02 ||
                                     txpdo_index == 0x1B03);
            const int32_t last_field = rd32(read_resp.data, sm3.length,
                                            static_cast<int>(sm3.length) - 4);

            TETHER_LOGI(TAG, "[TxPDO 0x{:04X}] {} Cycle {}:", txpdo_index, slavePrefix(si).c_str(), wire_cycle);
            TETHER_LOGI(TAG, "  Statusword: {} (0x{:04X})", sw_state, sw);
            if (last_is_di) {
                TETHER_LOGI(TAG, "  ActualPosition={:>10} DigitalInputs=0x{:08X} ActualTorque={:>6}",
                            ap, static_cast<uint32_t>(last_field), at);
            } else {
                TETHER_LOGI(TAG, "  ActualPosition={:>10} ActualVelocity={:>10} ActualTorque={:>6}",
                            ap, last_field, at);
            }
        }
    } else if (have_write) {
        // Write only — uses APWR via writeRegister (position-based addressing)
        if (transport_.writeRegister(transport_.adpForSlaveIndex(si),
                         sm2.phys_start_addr, out_buf, sm2.length, 50)) {
            physical_stats_.fpwr_success++;
        } else {
            physical_stats_.fpwr_wkc_errors++;
            fpwr_ok = false;
        }
    } else if (have_read) {
        // Read only — uses APRD via readRegister (position-based addressing)
        uint8_t in_buf[PDO::kMaxPDOSize] = {0};
        if (transport_.readRegister(transport_.adpForSlaveIndex(si),
                        sm3.phys_start_addr, in_buf, sm3.length, 50)) {
            physical_stats_.fprd_success++;
            if (txPDODebug()) {
                char hex[128];
                size_t pos = 0;
                size_t dump_len = sm3.length < 32 ? sm3.length : 32;
                for (size_t b = 0; b < dump_len && pos + 3 < sizeof(hex); b++) {
                    pos += static_cast<size_t>(std::snprintf(hex + pos, sizeof(hex) - pos, "%02X ", in_buf[b]));
                }
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 ({}): addr=0x{:04x} len={} data={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length, hex);
            }
            for (size_t i = 0; i < mapping_.entry_count(); i++) {
                PDO::PDOEntry* e = mapping_.get_entry_mut(i);
                if (e && e->enabled && e->slave_index == si
                    && e->direction == PDO::PDODirection::TxPDO
                    && e->data_size > 0 && e->data_size <= sm3.length
                    && e->physical_offset >= sm3.phys_start_addr
                    && e->physical_offset - sm3.phys_start_addr + e->data_size
                           <= sm3.length) {
                    std::memcpy(e->storage,
                                in_buf + (e->physical_offset - sm3.phys_start_addr),
                                e->data_size);
                    e->success_count++;
                    if (txPDODebug(e->slave_index)) {
                        TETHER_LOGI(TAG, "  [TxPDO-DEBUG] Copied {} bytes to entry {} buf={:p}",
                                    e->data_size, i, static_cast<const void*>(e->storage));
                    }
                }
            }
        } else {
            physical_stats_.fprd_wkc_errors++;
            fprd_ok = false;
            if (txPDODebug()) {
                TETHER_LOGI(TAG, "[TxPDO-DEBUG] Physical read SM3 FAILED ({}): addr=0x{:04x} len={}",
                            slavePrefix(si).c_str(), sm3.phys_start_addr, sm3.length);
            }
        }
    }
    }  // end for each slave
    stats_.total_cycles++;
    return fpwr_ok && fprd_ok;
}

} // namespace EtherCAT

