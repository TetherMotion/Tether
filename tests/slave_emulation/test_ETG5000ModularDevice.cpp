#include <gtest/gtest.h>

#include "tether/etg5000/ETG5000ModularDevice.hpp"
#include "tether/etg5000/ETG5000Defs.hpp"
#include "tether/ethercat/SDOManager.hpp"

using namespace ETG5000;

namespace {

// Minimal no-op transport for tests that don't perform actual SDO operations
class NullSDOTransport : public EtherCAT::SDO::ISDOTransport {
public:
    bool sdoUpload(uint16_t, uint8_t*, uint16_t, uint16_t,
                   uint16_t, uint16_t, uint16_t, uint8_t,
                   uint8_t*, size_t, size_t*) override { return false; }
    bool sdoDownload(uint16_t, uint8_t*, uint16_t, uint16_t,
                     uint16_t, uint16_t, uint16_t, uint8_t,
                     const uint8_t*, size_t) override { return false; }
    uint64_t getMicroseconds() override { return 0; }
};

} // anonymous namespace

TEST(ETG5000ModularDevice, ModuleState_Accessors)
{
    ModuleState s;
    s.slot = 1;
    s.module_type = ModuleType::DigitalInput;
    s.status = ModuleStatus::Operational;
    s.diag_status = DiagStatus::ModuleOK | DiagStatus::Warning;
    s.last_error = ErrorCode::NoError;
    s.temperature = 255;    // 25.5°C
    s.supply_voltage = 330; // 33.0 V

    EXPECT_TRUE(s.isPresent());
    EXPECT_TRUE(s.isOperational());
    EXPECT_FALSE(s.hasError());
    EXPECT_NEAR(s.getTemperature_C(), 25.5f, 1e-3f);
    EXPECT_NEAR(s.getSupplyVoltage_V(), 33.0f, 1e-3f);
}

TEST(ETG5000ModularDevice, DeviceState_Flags)
{
    DeviceState d;
    d.statusword = 0;

    d.statusword |= StatuswordBits::Ready;
    EXPECT_TRUE(d.isReady());

    d.statusword |= StatuswordBits::OperationalMode;
    EXPECT_TRUE(d.isOperational());

    d.statusword |= StatuswordBits::ConfigMismatch;
    EXPECT_TRUE(d.hasConfigMismatch());

    d.statusword |= StatuswordBits::ModuleError;
    EXPECT_TRUE(d.hasModuleError());

    d.statusword |= StatuswordBits::DiagAvailable;
    EXPECT_TRUE(d.hasDiagAvailable());

    d.statusword |= StatuswordBits::HotSwapEvent;
    EXPECT_TRUE(d.hasHotSwapEvent());

    d.statusword |= StatuswordBits::Warning;
    EXPECT_TRUE(d.hasWarning());

    d.statusword |= StatuswordBits::Error;
    EXPECT_TRUE(d.hasError());
}

TEST(ETG5000ModularDevice, PrepareRxPDO_and_DefaultOffsets)
{
    NullSDOTransport transport;
    EtherCAT::SDO::SDOManager sdo(transport);
    ModularDevice dev(sdo, 0);

    // By default controlword_ == 0 -> prepareRxPDO should write zeros
    uint8_t buf[16] = {};
    size_t written = dev.prepareRxPDO(buf, sizeof(buf));
    EXPECT_EQ(written, sizeof(ModularOutputPDO));

    // controlword is little-endian uint16_t at start of PDO
    uint16_t control = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
    EXPECT_EQ(control, 0u);

    // Offsets / sizes should be zero for empty/uninitialized device
    EXPECT_EQ(dev.getModuleInputOffset(0), 0u);
    EXPECT_EQ(dev.getModuleOutputOffset(0), 0u);
    EXPECT_EQ(dev.getModuleInputSize(0), 0u);
    EXPECT_EQ(dev.getModuleOutputSize(0), 0u);
}

TEST(ETG5000ModularDevice, DiagnosticsString_Default)
{
    NullSDOTransport transport;
    EtherCAT::SDO::SDOManager sdo(transport);
    ModularDevice dev(sdo, 1);
    // Default-constructed state should produce a diagnostics string containing expected headers
    std::string diag = dev.getDiagnostics();
    EXPECT_NE(diag.find("Modular Device Diagnostics"), std::string::npos);
    EXPECT_NE(diag.find("Detected Modules"), std::string::npos);
    EXPECT_NE(diag.find("Module Details"), std::string::npos);
}
