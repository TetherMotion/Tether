#include <gtest/gtest.h>
#include "sii/SIIReader.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

using namespace EtherCAT::SII;

static constexpr uint16_t EC_REG_EEPDAT   = 0x0508;

TEST(AprdSequence, InterleavedResponses_MultipleSlaves) {
    EtherCAT::Master master;
    master.clearAprdResponses();

    uint32_t v0 = 0x11111111u;
    uint32_t v1 = 0x22222222u;
    master.pushAprdResponse(true, 0x0000, EC_REG_EEPDAT, &v0, sizeof(v0));
    master.pushAprdResponse(true, 0xFFFF, EC_REG_EEPDAT, &v1, sizeof(v1));

    SIIReader r(master);
    uint32_t out0 = 0, out1 = 0;
    EXPECT_TRUE(r.readDWord(0, 0, out0));
    EXPECT_TRUE(r.readDWord(1, 0, out1));
    EXPECT_EQ(out0, v0);
    EXPECT_EQ(out1, v1);

    master.clearAprdResponses();
}

TEST(AprdSequence, PartialThenFull_ReadDWordObservesPartialThenFull) {
    EtherCAT::Master master;
    master.clearAprdResponses();

    // First response: partial (only low 2 bytes)
    uint16_t partial = 0x4444;
    master.pushAprdResponse(true, 0x0000, EC_REG_EEPDAT, &partial, sizeof(partial));

    // Second call: full 4 bytes
    uint32_t full = 0xAABBCCDDu;
    master.pushAprdResponse(true, 0x0000, EC_REG_EEPDAT, &full, sizeof(full));

    SIIReader reader(master);
    uint32_t out = 0;

    // First read observes partial data (lower 16 bits set)
    ASSERT_TRUE(reader.readDWord(0, 0, out));
    EXPECT_EQ(static_cast<uint16_t>(out & 0xFFFFu), partial);

    // Second read should get the full value
    ASSERT_TRUE(reader.readDWord(0, 0, out));
    EXPECT_EQ(out, full);

    master.clearAprdResponses();
}

TEST(AprdSequence, AprdTransportFailure_ReadDWordFails) {
    EtherCAT::Master master;
    master.clearAprdResponses();

    // Simulate transport failure when reading EEPDAT
    master.pushAprdResponse(false, 0x0000, EC_REG_EEPDAT, nullptr, 0);

    SIIReader reader(master);
    uint32_t out = 0;
    EXPECT_FALSE(reader.readDWord(0, 0, out));

    master.clearAprdResponses();
}
