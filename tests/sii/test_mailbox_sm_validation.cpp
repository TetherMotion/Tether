#include <gtest/gtest.h>
#include "tether/platform/Platform.hpp"
#include "ethercat/raw/internal.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

using namespace EtherCAT::Raw;
using namespace Tether::Platform;

// Minimal mock EEPROM that also returns SM control bytes for SM0/SM1
class MockSIIEEPROMLocal {
public:
    MockSIIEEPROMLocal() { reset(); }

    void reset() {
        data_.clear();
        last_cmd_addr_ = 0xFFFF;
        simulate_failure_ = false;
        sm0_ctrl_ = 0x00;
        sm1_ctrl_ = 0x00;
    }

    void setValidMailbox(uint16_t rx_addr, uint16_t rx_size,
                         uint16_t tx_addr, uint16_t tx_size,
                         uint16_t protocols) {
        // std_rx = slave RX (master write)
        data_[0x0018] = rx_addr;   // std_rx_offset
        data_[0x0019] = rx_size;
        data_[0x001A] = tx_addr;   // std_tx_offset
        data_[0x001B] = tx_size;
        data_[0x001C] = protocols;
    }

    void setSMControl(uint16_t sm_index, uint8_t ctrl) {
        if (sm_index == 0) sm0_ctrl_ = ctrl;
        else if (sm_index == 1) sm1_ctrl_ = ctrl;
    }

    void installMocks(EtherCAT::EtherCATMaster& master) {
        master_ = &master;
        // APWR: capture last_cmd_addr for EEPCTL write
        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                           const void* data, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;
            // EEPCTL write command sets address for following EEPDAT reads
            if (ado == EC_REG_EEPCTL && data && len >= 4) {
                // The command word contains addr_le at offset 2 in tests
                uint16_t addr_le = 0;
                std::memcpy(&addr_le, reinterpret_cast<const uint8_t*>(data) + 2, sizeof(addr_le));
                last_cmd_addr_ = addr_le;
            }
            return !simulate_failure_;
        });

        // APRD: handle EEPSTAT/EEPDAT and SM control reads
        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                           void* out, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;
            // SM control reads
            if (ado == (EC_REG_SM0 + 0x04)) {
                if (out && len >= 1) {
                    uint8_t val = sm0_ctrl_;
                    std::memcpy(out, &val, 1);
                    return true;
                }
            }
            if (ado == (EC_REG_SM1 + 0x04)) {
                if (out && len >= 1) {
                    uint8_t val = sm1_ctrl_;
                    std::memcpy(out, &val, 1);
                    return true;
                }
            }

            // EEPROM status read
            if (ado == EC_REG_EEPSTAT) {
                if (out && len >= 2) {
                    uint16_t estat_le = 0; // not busy
                    std::memcpy(out, &estat_le, 2);
                    return true;
                }
            }

            // EEPDAT: return stored 32-bit value for last_cmd_addr_
            if (ado == EC_REG_EEPDAT) {
                if (out && len >= 4) {
                    uint16_t lo = data_.count(last_cmd_addr_) ? data_[last_cmd_addr_] : 0;
                    uint16_t hi = data_.count(last_cmd_addr_ + 1) ? data_[last_cmd_addr_ + 1] : 0;
                    uint32_t val = static_cast<uint32_t>(lo) | (static_cast<uint32_t>(hi) << 16);
                    std::memcpy(out, &val, 4);
                    return true;
                }
            }

            return false;
        });
    }

    void removeMocks() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }

private:
    std::map<uint16_t, uint16_t> data_;  // word addr -> value
    uint16_t last_cmd_addr_{0xFFFF};
    bool simulate_failure_{false};
    uint8_t sm0_ctrl_{0x00};
    uint8_t sm1_ctrl_{0x00};
    EtherCAT::EtherCATMaster* master_{nullptr};
};

TEST(SIIMailbox, SMControlValidationLogsWarning) {
    // Capture logs
    Logger& logger = Logger::instance();
    logger.setLevel(LogLevel::Info);
    std::vector<std::string> captured;
    logger.setHandler([&](LogLevel, const char* tag, const char* msg){ (void)tag; captured.emplace_back(msg); });

    // Install mock SII/EEPROM and make SM controls invalid (non-mailbox or wrong dir)
    MockSIIEEPROMLocal mock;
    mock.setValidMailbox(0x1400, 128, 0x1000, 64, 0x0004);
    // Set SM control bytes to 0x00 (buffered mode, master->slave) to trigger warnings
    mock.setSMControl(0, 0x00);
    mock.setSMControl(1, 0x00);

    EtherCAT::EtherCATMaster master;
    mock.installMocks(master);

    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;

    bool ok = configure_mailbox_from_sii(master, 0x0000, &wr_addr, &wr_len, &rd_addr, &rd_len, &proto);
    EXPECT_TRUE(ok);

    // We expect warnings about both SM0 and SM1 and that the message includes a decoded description
    bool saw_sm0 = false, saw_sm1 = false, saw_desc0 = false, saw_desc1 = false;
    for (auto &m : captured) {
        if (m.find("SM0 control") != std::string::npos) saw_sm0 = true;
        if (m.find("SM1 control") != std::string::npos) saw_sm1 = true;
        if (m.find("mode=") != std::string::npos && m.find("SM0") != std::string::npos) saw_desc0 = true;
        if (m.find("mode=") != std::string::npos && m.find("SM1") != std::string::npos) saw_desc1 = true;
    }

    EXPECT_TRUE(saw_sm0);
    EXPECT_TRUE(saw_sm1);
    // At least one of the messages should include the decoded ctrl description
    EXPECT_TRUE(saw_desc0 || saw_desc1);

    mock.removeMocks();
    logger.setHandler(nullptr);
}

TEST(SIIMailbox, SMControlValidationNoWarning) {
    // Capture logs
    Logger& logger = Logger::instance();
    logger.setLevel(LogLevel::Info);
    std::vector<std::string> captured;
    logger.setHandler([&](LogLevel, const char* tag, const char* msg){ (void)tag; captured.emplace_back(msg); });

    MockSIIEEPROMLocal mock_ok;
    mock_ok.setValidMailbox(0x1400, 128, 0x1000, 64, 0x0004);

    // SM0: mailbox mode + ECAT writes (master→slave, Receive/MbxIn): SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_WRITE | SM_CTRL_IRQ_PDI
    uint8_t sm0_ok = static_cast<uint8_t>(0x02u | 0x04u | 0x20u); // 0x26
    // SM1: mailbox mode + ECAT reads (slave→master, Send/MbxOut): SM_CTRL_MODE_MAILBOX | SM_CTRL_DIR_READ | SM_CTRL_IRQ_PDI
    uint8_t sm1_ok = static_cast<uint8_t>(0x02u | 0x00u | 0x20u); // 0x22
    mock_ok.setSMControl(0, sm0_ok);
    mock_ok.setSMControl(1, sm1_ok);

    EtherCAT::EtherCATMaster master;
    mock_ok.installMocks(master);

    uint16_t wr_addr = 0, wr_len = 0, rd_addr = 0, rd_len = 0, proto = 0;

    bool ok = configure_mailbox_from_sii(master, 0x0000, &wr_addr, &wr_len, &rd_addr, &rd_len, &proto);
    EXPECT_TRUE(ok);

    bool saw_any_warning = false;
    for (auto &m : captured) {
        if (m.find("SM0 control") != std::string::npos) saw_any_warning = true;
        if (m.find("SM1 control") != std::string::npos) saw_any_warning = true;
    }
    EXPECT_FALSE(saw_any_warning);

    mock_ok.removeMocks();
    logger.setHandler(nullptr);
}
