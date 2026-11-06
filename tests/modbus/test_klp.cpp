// Klipper-object tests: ModbusSensorObject / ModbusOutputObject /
// ModbusGcodes — compiled only when klipper + modbus coexist.
#include <gtest/gtest.h>

#if defined(TETHER_MODBUS_KLIPPER)

#include "tether/modbus/klp/ModbusObjects.hpp"
#include "tether/modbus/sim/Sim.hpp"

using namespace tether::modbus;
using tether::modbus::klp::ModbusGcodes;
using tether::modbus::klp::ModbusOutputObject;
using tether::modbus::klp::ModbusSensorObject;
using tether::klipper::klippy::GcodeLine;

namespace {
struct KlpRig {
    SimSlave s{1};
    SimInvoker inv{s};
    ModbusClient cli{inv, 1};
    DeviceProfile prof;
    BoundDevice dev{cli, prof};

    KlpRig() {
        prof.id = "test/klp";
        prof.registers.push_back({"temp",
            Address{TableId::HoldingRegister, 0},
            ScalarSpec{ValueType::U16, WordOrder::BigEndian, 0.1, 0.0},
            0, false, 0.0, 0.0, "°C", "env", "chamber temp"});
        prof.registers.push_back({"run",
            Address{TableId::Coil, 0},
            ScalarSpec{ValueType::Bit, WordOrder::BigEndian, 1.0, 0.0},
            0, true, 0.0, 1.0, "", "ctl", "heater on"});

        (void)s.store().declare(TableId::HoldingRegister, 0, 4);
        (void)s.store().declare(TableId::Coil, 0, 4);
        s.preload(0, {235});               // 23.5 °C
    }
};
} // namespace

TEST(Klp, SensorStatus) {
    KlpRig rig;
    ModbusSensorObject obj("chamber", rig.dev, "temp");
    EXPECT_EQ(obj.name(), "modbus_sensor chamber");

    obj.update();                           // app/poller drives reads
    auto st = obj.status({});
    ASSERT_TRUE(st.count("value"));
    EXPECT_NEAR(st["value"].asDouble(), 23.5, 1e-6);
    EXPECT_TRUE(st["ok"].asBool());
    EXPECT_EQ(st["unit"].asString(), "°C");

    // Device dropped → error surfaces, ok=false.
    rig.s.traits().dropResponse = 1.0;
    obj.update();
    st = obj.status({});
    EXPECT_FALSE(st["ok"].asBool());
    EXPECT_TRUE(st.count("error"));
}

TEST(Klp, OutputSetAndQuery) {
    KlpRig rig;
    ModbusOutputObject obj("heater", rig.dev, "run");
    EXPECT_EQ(obj.name(), "modbus_output heater");

    ASSERT_TRUE(obj.set(1.0).has_value());
    EXPECT_EQ(obj.value(), 1.0);
    EXPECT_TRUE(*rig.s.store().getBit(TableId::Coil, 0));
}

TEST(Klp, Gcodes) {
    KlpRig rig;
    ModbusOutputObject out("heater", rig.dev, "run");
    std::vector<std::string> log;
    ModbusGcodes gc({{"bus0", &rig.dev}}, {{"heater", &out}},
                    [&](const std::string& s) { log.push_back(s); });

    GcodeLine l;
    l.code = "MODBUS_READ";
    l.namedParams = {{"BUS", "bus0"}, {"REG", "temp"}};
    ASSERT_TRUE(gc(l));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_NE(log[0].find("23.5"), std::string::npos) << log[0];

    log.clear();
    l.code = "MODBUS_WRITE";
    l.namedParams = {{"BUS", "bus0"}, {"REG", "run"}, {"VALUE", "1"}};
    ASSERT_TRUE(gc(l));
    EXPECT_TRUE(*rig.s.store().getBit(TableId::Coil, 0));

    log.clear();
    l.code = "SET_MODBUS_OUTPUT";
    l.namedParams = {{"NAME", "heater"}, {"VALUE", "0"}};
    ASSERT_TRUE(gc(l));
    EXPECT_EQ(out.value(), 0.0);

    log.clear();
    l.code = "QUERY_MODBUS_OUTPUT";
    l.namedParams = {{"NAME", "heater"}};
    ASSERT_TRUE(gc(l));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_NE(log[0].find("= 0"), std::string::npos) << log[0];

    // Unrelated codes pass through (return false).
    l.code = "BED_MESH_CALIBRATE";
    l.namedParams.clear();
    EXPECT_FALSE(gc(l));
}

#endif // TETHER_MODBUS_KLIPPER
