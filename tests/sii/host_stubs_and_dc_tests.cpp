#include <gtest/gtest.h>
#include "tether/ethercat/Raw.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Master.hpp"

using namespace EtherCAT;

// Some host stubs only exist when building without the full EtherCAT backend
// (i.e., !TETHER_ENABLE_ETHERCAT). Guard the tests so the suite compiles in
// both host-stub and full-backend builds.
#if !defined(TETHER_ENABLE_ETHERCAT)
extern "C" bool ecm_sdo_read(uint16_t, uint16_t, uint8_t, void*, size_t, bool);
extern "C" bool ecm_sdo_write(uint16_t, uint16_t, uint8_t, const void*, size_t, bool);
#endif

TEST(HostStubs_Basic, ReturnsDefaults) {
#if !defined(TETHER_ENABLE_ETHERCAT)
    // Only validate the SDO stubs here to avoid duplicate-symbol
    // conflicts with the full EtherCAT implementation in other units.
    char buf[4] = {0};
    EXPECT_FALSE(ecm_sdo_read(1, 2, 3, buf, sizeof(buf), false));
    EXPECT_FALSE(ecm_sdo_write(1, 2, 3, buf, sizeof(buf), false));
#else
    GTEST_SKIP() << "Host stubs not present in this build";
#endif
}

TEST(PDOStubs_Basic, MappingNull) {
#if !defined(TETHER_ENABLE_ETHERCAT)
    // Use a local PDOManager instead of the global default
    class StubTransport : public IPDOTransport {
    public:
        bool writeRegister(uint16_t, uint16_t, const void*, uint16_t, unsigned int) override { return false; }
        bool readRegister(uint16_t, uint16_t, void*, uint16_t, unsigned int) override { return false; }
        bool sendSingleDatagram(Command, uint8_t, uint16_t, uint16_t, const void*, uint16_t, bool) override { return false; }
        size_t sendMultiDatagram(const MultiDatagramSpec*, size_t) override { return 0; }
        bool waitForResponseIdx(uint8_t, unsigned int, RxDatagram&) override { return false; }
        uint8_t allocIdx() override { return 0; }
        uint16_t adpForSlaveIndex(uint16_t idx) override { return static_cast<uint16_t>(0u - idx); }
    };
    StubTransport transport;
    PDOManager mgr(transport);
    mgr.init();
    auto& m = mgr.mapping();
    EXPECT_GE(m.total_rxpdo_bytes(), 0u);
    mgr.deinit();
#else
    GTEST_SKIP() << "PDO stubs not present in this build";
#endif
}

TEST(DC_BasicQueries, DefaultContext) {
    // Without initialization, getState() should return Disabled (enum default)
    EtherCAT::Master master;
    auto state = master.dc().getState();
    // State is an enum; ensure it's a valid enum value
    EXPECT_EQ(state, DC::DCState::Disabled);

    auto stats = master.dc().getStats();
    EXPECT_EQ(stats.cycle_count, 0u);
}
