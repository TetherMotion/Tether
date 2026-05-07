#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

using namespace EtherCAT::SII;

static constexpr uint16_t EC_REG_EEPCTL   = 0x0502;
static constexpr uint16_t EC_REG_EEPSTAT  = 0x0502;
static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;
static constexpr uint16_t EC_ECMD_READ = 0x0100;
static constexpr uint16_t EC_ESTAT_BUSY  = 0x8000;

TEST(MoreRaw, ReadBytes_ZeroCount_ReturnsZero) {
    EtherCAT::EtherCATMaster master; SIIReader reader(master);
    uint8_t buf[4] = {0};
    size_t n = reader.readBytes(0, 0, buf, 0);
    EXPECT_EQ(n, 0u);
}

TEST(MoreRaw, ReadString_InvalidArgs_ReturnFalse) {
    EtherCAT::EtherCATMaster master; SIIReader reader(master);
    char buf[8];
    EXPECT_FALSE(reader.readString(0, 0, buf, sizeof(buf))); // string_index == 0 invalid
    EXPECT_FALSE(reader.readString(0, 1, nullptr, 4)); // null buffer
    EXPECT_FALSE(reader.readString(0, 1, buf, 0)); // zero buffer size
}

TEST(MoreRaw, WaitNotBusy_Timeout_ReadDWordFails) {
    // APRD always returns BUSY on EEPSTAT
    EtherCAT::EtherCATMaster master;

    master.setAprdTestCallback([&](uint16_t adp, uint16_t ado, void* out, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms;
        if (ado == EC_REG_EEPCTL || ado == EC_REG_EEPCTL) {}
        if (ado == EC_REG_EEPCTL) {}
        if (ado == EC_REG_EEPDAT) return false; // shouldn't be reached
        if (ado == EC_REG_EEPSTAT && out && len >= 2) {
            uint16_t busy = EC_ESTAT_BUSY; memcpy(out, &busy, 2); return true;
        }
        return false;
    });

    SIIReader reader(master);
    reader.setTimeout(10); // very short timeout
    uint32_t out = 0;
    bool ok = reader.readDWord(0, 0, out);
    EXPECT_FALSE(ok);

    master.setAprdTestCallback(nullptr);
}

TEST(MoreRaw, ApwrTriggersAprdResponse_ReadDWordSucceeds) {
    EtherCAT::EtherCATMaster master;
    master.clearAprdResponses();

    // APWR hook: when SIIReader writes read command to EEPCTL, push an APRD response with the data
    master.setApwrTestCallback([&](uint16_t adp, uint16_t ado, const void* data, uint16_t len, unsigned int ms)->bool {
        (void)adp; (void)ms;
        if (ado == EC_REG_EEPCTL && data && len >= 4) {
            struct __attribute__((packed)) EepromCmd { uint16_t comm_le; uint16_t addr_le; uint16_t d2_le; };
            const EepromCmd* cmd = reinterpret_cast<const EepromCmd*>(data);
            uint16_t comm = cmd->comm_le; // host is little-endian in tests
            uint16_t addr = cmd->addr_le;
            if (comm == EC_ECMD_READ) {
                // Construct value with lower 16 bits equal to the requested address for easy verification
                uint32_t val = (0xCAFEBABE & 0xFFFF0000u) | (static_cast<uint32_t>(addr) & 0xFFFFu);
                master.pushAprdResponse(true, adp, EC_REG_EEPDAT, &val, sizeof(val));
            }
        }
        return true;
    });

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_TRUE(reader.readDWord(0, 0x0042, out));
    EXPECT_EQ((out & 0xFFFFu), static_cast<uint32_t>(0x0042u));

    master.clearAprdResponses();
    master.setApwrTestCallback(nullptr);
}
