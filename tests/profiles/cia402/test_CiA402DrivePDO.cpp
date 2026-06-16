/**
 * @file test_CiA402DrivePDO.cpp
 * @brief Tests for CiA402 Drive PDO, state helpers, and AS715N PDO structs
 */
#include <gtest/gtest.h>
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/drives/PBLR81FGF/PBLR81FGFPDO.hpp"
#include "tether/ethercat/Controlword.hpp"
#include <cstring>
#include <string>

using namespace EtherCAT;

// ============================================================================
// ECState helpers
// ============================================================================

TEST(ECStateTest, GetName) {
    EXPECT_NE(getECStateName(ECState::Init), nullptr);
    EXPECT_NE(getECStateName(ECState::PreOp), nullptr);
    EXPECT_NE(getECStateName(ECState::SafeOp), nullptr);
    EXPECT_NE(getECStateName(ECState::Op), nullptr);
    EXPECT_NE(getECStateName(ECState::Unknown), nullptr);
}

TEST(ECStateTest, Values) {
    EXPECT_EQ(static_cast<uint8_t>(ECState::Init), 0x01u);
    EXPECT_EQ(static_cast<uint8_t>(ECState::PreOp), 0x02u);
    EXPECT_EQ(static_cast<uint8_t>(ECState::Bootstrap), 0x03u);
    EXPECT_EQ(static_cast<uint8_t>(ECState::SafeOp), 0x04u);
    EXPECT_EQ(static_cast<uint8_t>(ECState::Op), 0x08u);
    EXPECT_EQ(static_cast<uint8_t>(ECState::Unknown), 0x00u);
}

// ============================================================================
// DriveState helpers
// ============================================================================

TEST(DriveStateTest, GetName) {
    EXPECT_NE(getDriveStateName(DriveState::NotReadyToSwitchOn), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::SwitchOnDisabled), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::ReadyToSwitchOn), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::SwitchedOn), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::OperationEnabled), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::QuickStopActive), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::FaultReactionActive), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::Fault), nullptr);
    EXPECT_NE(getDriveStateName(DriveState::Unknown), nullptr);
}

// ============================================================================
// decodeDriveState
// ============================================================================

TEST(DecodeDriveStateTest, SwitchOnDisabled) {
    EXPECT_EQ(decodeDriveState(0x0040), DriveState::SwitchOnDisabled);
}

TEST(DecodeDriveStateTest, ReadyToSwitchOn) {
    EXPECT_EQ(decodeDriveState(0x0021), DriveState::ReadyToSwitchOn);
}

TEST(DecodeDriveStateTest, SwitchedOn) {
    EXPECT_EQ(decodeDriveState(0x0023), DriveState::SwitchedOn);
}

TEST(DecodeDriveStateTest, OperationEnabled) {
    EXPECT_EQ(decodeDriveState(0x0027), DriveState::OperationEnabled);
}

TEST(DecodeDriveStateTest, QuickStopActive) {
    EXPECT_EQ(decodeDriveState(0x0007), DriveState::QuickStopActive);
}

TEST(DecodeDriveStateTest, Fault) {
    EXPECT_EQ(decodeDriveState(0x0008), DriveState::Fault);
}

TEST(DecodeDriveStateTest, FaultReactionActive) {
    EXPECT_EQ(decodeDriveState(0x000F), DriveState::FaultReactionActive);
}

// ============================================================================
// formatStatuswordDiagnostics
// ============================================================================

TEST(FormatStatuswordTest, BasicFormat) {
    char buffer[256];
    const char* result = formatStatuswordDiagnostics(0x0027, buffer, sizeof(buffer));
    EXPECT_NE(result, nullptr);
    EXPECT_GT(strlen(result), 0u);
}

TEST(FormatStatuswordTest, FaultFormat) {
    char buffer[256];
    const char* result = formatStatuswordDiagnostics(0x0008, buffer, sizeof(buffer));
    EXPECT_NE(result, nullptr);
}

TEST(FormatStatuswordTest, SmallBuffer) {
    char buffer[4];
    const char* result = formatStatuswordDiagnostics(0x0027, buffer, sizeof(buffer));
    EXPECT_NE(result, nullptr);
    EXPECT_LE(strlen(result), sizeof(buffer) - 1);
}

// ---------------------------------------------------------------------------
// describeControlword tests (moved helper)
// ---------------------------------------------------------------------------

TEST(DescribeControlwordTest, BasicCommands) {
    std::string r = EtherCAT::describeControlword(0x0080u);
    EXPECT_GT(r.size(), 0u);
    EXPECT_NE(std::string::npos, r.find("FaultReset"));

    r = EtherCAT::describeControlword(0x000Fu);
    EXPECT_NE(std::string::npos, r.find("EnableOperation"));
}

TEST(DescribeControlwordTest, ReturnNotEmpty) {
    std::string r = EtherCAT::describeControlword(0x000Fu);
    EXPECT_GT(r.size(), 0u);
}

// ============================================================================
// ControlWord enum (EtherCAT namespace)
// ============================================================================

TEST(EtherCATControlWordTest, Values) {
    EXPECT_EQ(static_cast<uint16_t>(ControlWord::DISABLE_VOLTAGE), 0x0000u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWord::SHUTDOWN), 0x0006u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWord::SWITCH_ON), 0x0007u);
    EXPECT_EQ(static_cast<uint16_t>(ControlWord::ENABLE_OPERATION), 0x000Fu);
    EXPECT_EQ(static_cast<uint16_t>(ControlWord::FAULT_RESET), 0x0080u);
}

// ============================================================================
// Constants
// ============================================================================

TEST(PDOConstantsTest, MaxBufferSize) {
    EXPECT_EQ(CiA402Drive::kMaxPDOBufferSize, 256u);
}

TEST(PDOConstantsTest, MaxManagedDrives) {
    EXPECT_EQ(kMaxManagedDrives, 8u);
}

// ============================================================================
// AS715N Packed PDO Structs
// ============================================================================

TEST(AS715NPDOStructTest, RxPDO1705Size) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EXPECT_EQ(sizeof(AS715N_RxPDO_1705), RxPDO_1705.size);
    EXPECT_EQ(sizeof(AS715N_RxPDO_1705), 19u);
}

TEST(AS715NPDOStructTest, TxPDO1B04Size) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EXPECT_EQ(sizeof(AS715N_TxPDO_1B04), TxPDO_1B04.size);
    EXPECT_EQ(sizeof(AS715N_TxPDO_1B04), 29u);
}

TEST(AS715NPDOStructTest, RxPDO1702Size) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EXPECT_EQ(sizeof(AS715N_RxPDO_1702), RxPDO_1702.size);
    EXPECT_EQ(sizeof(AS715N_RxPDO_1702), 19u);
}

TEST(AS715NPDOStructTest, TxPDO1B02Size) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EXPECT_EQ(sizeof(AS715N_TxPDO_1B02), TxPDO_1B02.size);
    EXPECT_EQ(sizeof(AS715N_TxPDO_1B02), 25u);
}

// ---------------------------------------------------------------------------
// PBLR81FGF Packed PDO Structs
// ---------------------------------------------------------------------------

TEST(PBLR81FGFPDOStructTest, RxPDO1600Size) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    EXPECT_EQ(sizeof(PBLR81FGF_RxPDO_1600), RxPDO_1600.size);
    EXPECT_EQ(sizeof(PBLR81FGF_RxPDO_1600), 16u);
}

TEST(PBLR81FGFPDOStructTest, TxPDO1A00Size) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    EXPECT_EQ(sizeof(PBLR81FGF_TxPDO_1A00), TxPDO_1A00.size);
    EXPECT_EQ(sizeof(PBLR81FGF_TxPDO_1A00), 16u);
}

TEST(PBLR81FGFPDOStructTest, RxPDO1600FieldLayout) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    PBLR81FGF_RxPDO_1600 rx{};
    rx.controlword = 0x000F;
    rx.target_position = 0x12345678;
    rx.target_velocity = -1000;
    rx.target_torque = -200;
    rx.max_torque = 300;
    rx.modes_of_operation = 3;
    rx.reserved = 0xAA;

    EXPECT_EQ(rx.controlword, 0x000Fu);
    EXPECT_EQ(rx.target_position, 0x12345678);
    EXPECT_EQ(rx.target_velocity, -1000);
    EXPECT_EQ(rx.target_torque, -200);
    EXPECT_EQ(rx.max_torque, 300u);
    EXPECT_EQ(rx.modes_of_operation, 3);
    EXPECT_EQ(rx.reserved, 0xAA);
}

TEST(PBLR81FGFPDOStructTest, TxPDO1A00FieldLayout) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    PBLR81FGF_TxPDO_1A00 tx{};
    tx.statusword = 0x0027;
    tx.position_actual = 0x9abcdef0;
    tx.velocity_actual = 123456;
    tx.torque_actual = -50;
    tx.error_code = 0x100;
    tx.modes_of_operation_display = 5;
    tx.reserved = 0x55;

    EXPECT_EQ(tx.statusword, 0x0027u);
    EXPECT_EQ(tx.position_actual, 0x9abcdef0);
    EXPECT_EQ(tx.velocity_actual, 123456);
    EXPECT_EQ(tx.torque_actual, -50);
    EXPECT_EQ(tx.error_code, 0x100u);
    EXPECT_EQ(tx.modes_of_operation_display, 5);
    EXPECT_EQ(tx.reserved, 0x55);
}

TEST(PBLR81FGFPDOStructTest, RxPDO1600MemoryLayout) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    PBLR81FGF_RxPDO_1600 rx{};
    auto base = reinterpret_cast<uintptr_t>(&rx);

    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.controlword) - base, 0u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.target_position) - base, 2u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.target_velocity) - base, 6u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.target_torque) - base, 10u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.max_torque) - base, 12u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.modes_of_operation) - base, 14u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.reserved) - base, 15u);
}

TEST(PBLR81FGFPDOStructTest, TxPDO1A00MemoryLayout) {
    using namespace EtherCAT::Drives::PBLR81FGF;
    PBLR81FGF_TxPDO_1A00 tx{};
    auto base = reinterpret_cast<uintptr_t>(&tx);

    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.statusword) - base, 0u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.position_actual) - base, 2u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.velocity_actual) - base, 6u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.torque_actual) - base, 10u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.error_code) - base, 12u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.modes_of_operation_display) - base, 14u);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&tx.reserved) - base, 15u);
}

TEST(AS715NPDOStructTest, RxPDO1705FieldLayout) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    AS715N_RxPDO_1705 rx{};
    rx.controlword = 0x000F;
    rx.target_position = 12345;
    rx.target_velocity = -500;
    rx.modes_of_operation = 8;
    rx.touch_probe_function = 0;
    rx.positive_torque_limit = 1000;
    rx.negative_torque_limit = 1000;
    rx.torque_offset = 0;

    EXPECT_EQ(rx.controlword, 0x000Fu);
    EXPECT_EQ(rx.target_position, 12345);
    EXPECT_EQ(rx.target_velocity, -500);
    EXPECT_EQ(rx.modes_of_operation, 8);
    EXPECT_EQ(rx.positive_torque_limit, 1000u);
    EXPECT_EQ(rx.negative_torque_limit, 1000u);
}

TEST(AS715NPDOStructTest, TxPDO1B04FieldLayout) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    AS715N_TxPDO_1B04 tx{};
    tx.error_code = 0;
    tx.statusword = 0x0027;
    tx.position_actual = 65536;
    tx.torque_actual = 100;
    tx.modes_of_operation_display = 8;
    tx.position_deviation = 5;
    tx.touch_probe_status = 0;
    tx.touch_probe_pos1 = 0;
    tx.touch_probe_pos2 = 0;
    tx.speed_feedback = 1000;

    EXPECT_EQ(tx.statusword, 0x0027u);
    EXPECT_EQ(tx.position_actual, 65536);
    EXPECT_EQ(tx.torque_actual, 100);
    EXPECT_EQ(tx.modes_of_operation_display, 8);
    EXPECT_EQ(tx.speed_feedback, 1000);
}

TEST(AS715NPDOStructTest, RxPDO1705MemoryLayout) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    // Verify offsets match PDO descriptor expectations
    AS715N_RxPDO_1705 rx{};
    auto base = reinterpret_cast<uintptr_t>(&rx);

    // controlword at offset 0
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.controlword) - base, 0u);
    // target_position at offset 2
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.target_position) - base, 2u);
    // target_velocity at offset 6
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.target_velocity) - base, 6u);
    // modes_of_operation at offset 10
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.modes_of_operation) - base, 10u);
    // touch_probe_function at offset 11
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.touch_probe_function) - base, 11u);
    // positive_torque_limit at offset 13
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.positive_torque_limit) - base, 13u);
    // negative_torque_limit at offset 15
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.negative_torque_limit) - base, 15u);
    // torque_offset at offset 17
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&rx.torque_offset) - base, 17u);
}

// ============================================================================
// PDOQueue
// ============================================================================
#include "tether/profiles/cia402/PDOQueue.hpp"

TEST(PDOQueueTest, PushPopRx) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EtherCAT::PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04> q;

    AS715N_RxPDO_1705 cmd{};
    cmd.target_position = 42;
    EXPECT_TRUE(q.pushRx(cmd));
    EXPECT_EQ(q.rxPending(), 1u);

    AS715N_RxPDO_1705 out{};
    EXPECT_TRUE(q.popRx(out));
    EXPECT_EQ(out.target_position, 42);
    EXPECT_EQ(q.rxPending(), 0u);
}

TEST(PDOQueueTest, PopEmptyReturnsFalse) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EtherCAT::PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04> q;
    AS715N_RxPDO_1705 out{};
    EXPECT_FALSE(q.popRx(out));
}

TEST(PDOQueueTest, RxQueueFull) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EtherCAT::PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04, 4> q;
    AS715N_RxPDO_1705 cmd{};
    for (int i = 0; i < 4; ++i) {
        cmd.target_position = i;
        EXPECT_TRUE(q.pushRx(cmd));
    }
    EXPECT_FALSE(q.pushRx(cmd)); // full
    EXPECT_EQ(q.rxPending(), 4u);
}

TEST(PDOQueueTest, TxPublishAndRead) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EtherCAT::PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04> q;

    EXPECT_FALSE(q.hasTxData());

    AS715N_TxPDO_1B04 tx{};
    tx.position_actual = 99999;
    tx.statusword = 0x0027;
    q.publishTx(tx);

    EXPECT_TRUE(q.hasTxData());
    auto snap = q.latestTx();
    EXPECT_EQ(snap.position_actual, 99999);
    EXPECT_EQ(snap.statusword, 0x0027u);
}

TEST(PDOQueueTest, ClearRx) {
    using namespace EtherCAT::Drives::AS715N_pdo;
    EtherCAT::PDOQueue<AS715N_RxPDO_1705, AS715N_TxPDO_1B04> q;
    AS715N_RxPDO_1705 cmd{};
    q.pushRx(cmd);
    q.pushRx(cmd);
    EXPECT_EQ(q.rxPending(), 2u);
    q.clearRx();
    EXPECT_EQ(q.rxPending(), 0u);
}

// Verify that all of the built-in AS715N PDO descriptors refer to a valid
// object dictionary entry when one is expected.  The only field that is
// intentionally unpopulated is the "ForcedPhysicalDO" slot used by the
// default RxPDO 0x1701 which is a drive‑specific bitfield.
TEST(AS715NPDOStructTest, FieldsHaveEntries) {
    using namespace EtherCAT::Drives::AS715N_pdo;

    const std::set<std::string> ignore = {"ForcedPhysicalDO"};
    auto check = [&](const EtherCAT::Drives::AS715N_pdo::PDO &pdo) {
        for (size_t i = 0; i < pdo.field_count; ++i) {
            const EtherCAT::Drives::AS715N_pdo::PDOField &f = pdo.fields[i];
            if (ignore.count(f.description) != 0) continue;
            EXPECT_NE(f.entry, nullptr) << "PDO 0x" << std::hex << pdo.index
                                        << " field " << i << " (" << f.description
                                        << ") should have an entry";
        }
    };
    check(RxPDO_1701);
    check(RxPDO_1702);
    check(RxPDO_1703);
    check(RxPDO_1704);
    check(RxPDO_1705);
    check(TxPDO_1B01);
    check(TxPDO_1B02);
    check(TxPDO_1B03);
    check(TxPDO_1B04);
}
