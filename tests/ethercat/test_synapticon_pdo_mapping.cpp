/**
 * @file test_synapticon_pdo_mapping.cpp
 * @brief Unit tests for Synapticon SOMANET PDO mappings and multi-PDO assignment builders
 *
 * Verifies that the PDO structs, descriptors, and multi-PDO assignment builder
 * functions match the ESI file (SOMANET_CiA_402_v5.1.9.xml) layout.
 */

#include <gtest/gtest.h>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/drives/Synapticon.hpp"
#include "tether/ethercat/Slave.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Drives;
using namespace EtherCAT::Drives::Synapticon_pdo;

// ============================================================================
// Part 1: Standard PDO struct sizes (from ESI)
// ============================================================================

TEST(SynapticonPDOTest, RxPDO_1600_Size) {
    EXPECT_EQ(sizeof(SOMANET_RxPDO_1600), 19u);
    EXPECT_EQ(RxPDO_1600.index, 0x1600u);
    EXPECT_EQ(RxPDO_1600.size, 19u);
}

TEST(SynapticonPDOTest, RxPDO_1601_Size) {
    EXPECT_EQ(sizeof(SOMANET_RxPDO_1601), 8u);
    EXPECT_EQ(RxPDO_1601.index, 0x1601u);
    EXPECT_EQ(RxPDO_1601.size, 8u);
}

TEST(SynapticonPDOTest, RxPDO_1602_Size) {
    EXPECT_EQ(sizeof(SOMANET_RxPDO_1602), 8u);
    EXPECT_EQ(RxPDO_1602.index, 0x1602u);
    EXPECT_EQ(RxPDO_1602.size, 8u);
}

TEST(SynapticonPDOTest, TxPDO_1A00_Size) {
    EXPECT_EQ(sizeof(SOMANET_TxPDO_1A00), 13u);
    EXPECT_EQ(TxPDO_1A00.index, 0x1A00u);
    EXPECT_EQ(TxPDO_1A00.size, 13u);
}

TEST(SynapticonPDOTest, TxPDO_1A01_Size) {
    EXPECT_EQ(sizeof(SOMANET_TxPDO_1A01), 12u);
    EXPECT_EQ(TxPDO_1A01.index, 0x1A01u);
    EXPECT_EQ(TxPDO_1A01.size, 12u);
}

TEST(SynapticonPDOTest, TxPDO_1A02_Size) {
    EXPECT_EQ(sizeof(SOMANET_TxPDO_1A02), 4u);
    EXPECT_EQ(TxPDO_1A02.index, 0x1A02u);
    EXPECT_EQ(TxPDO_1A02.size, 4u);
}

TEST(SynapticonPDOTest, TxPDO_1A03_Size) {
    EXPECT_EQ(sizeof(SOMANET_TxPDO_1A03), 18u);
    EXPECT_EQ(TxPDO_1A03.index, 0x1A03u);
    EXPECT_EQ(TxPDO_1A03.size, 18u);
}

// ============================================================================
// Part 2: FSoE PDO struct sizes (from ESI)
// ============================================================================

TEST(SynapticonFSoEPDOTest, RxPDO_1700_Size) {
    EXPECT_EQ(sizeof(SOMANET_RxPDO_1700), 11u);
    EXPECT_EQ(RxPDO_1700.index, 0x1700u);
    EXPECT_EQ(RxPDO_1700.size, 11u);
}

TEST(SynapticonFSoEPDOTest, TxPDO_1B00_Size) {
    EXPECT_EQ(sizeof(SOMANET_TxPDO_1B00), 31u);
    EXPECT_EQ(TxPDO_1B00.index, 0x1B00u);
    EXPECT_EQ(TxPDO_1B00.size, 31u);
}

// ============================================================================
// Part 3: Sync Manager constants (from ESI)
// ============================================================================

TEST(SynapticonSMConstantsTest, SM2Constants) {
    EXPECT_EQ(kSM2ControlByte, 0x64u);
    EXPECT_EQ(kSM2PhysAddr, 0x1800u);
    EXPECT_EQ(kSM2TotalSize, 35u);  // 19 + 8 + 8
}

TEST(SynapticonSMConstantsTest, SM3Constants) {
    EXPECT_EQ(kSM3ControlByte, 0x20u);
    EXPECT_EQ(kSM3PhysAddr, 0x1C00u);
    EXPECT_EQ(kSM3TotalSize, 47u);  // 13 + 12 + 4 + 18
}

TEST(SynapticonSMConstantsTest, SynapticonNamespaceConstants) {
    EXPECT_EQ(Synapticon::kOutputsSmAddr, 0x1800u);
    EXPECT_EQ(Synapticon::kInputsSmAddr, 0x1C00u);
    EXPECT_EQ(Synapticon::kOutputsSmControlByte, 0x64u);
    EXPECT_EQ(Synapticon::kInputsSmControlByte, 0x20u);
    EXPECT_EQ(Synapticon::kOutputsSmSize, 35u);
    EXPECT_EQ(Synapticon::kInputsSmSize, 47u);
}

// ============================================================================
// Part 4: Multi-PDO Assignment Builders — Standard
// ============================================================================

TEST(SynapticonMultiPDOAssignmentTest, StandardAssignmentHasTwoSMs) {
    auto assignment = makeStandardPDOAssignment();
    ASSERT_EQ(assignment.sm_configs.size(), 2u);
}

TEST(SynapticonMultiPDOAssignmentTest, StandardAssignmentSM2) {
    auto assignment = makeStandardPDOAssignment();
    const auto& sm2 = assignment.sm_configs[0];
    EXPECT_EQ(sm2.sm_index, 2u);
    EXPECT_EQ(sm2.phys_start_addr, 0x1800u);
    EXPECT_EQ(sm2.control_byte, 0x64u);
    ASSERT_EQ(sm2.pdo_mappings.size(), 3u);

    EXPECT_EQ(sm2.pdo_mappings[0].pdo_index, 0x1600u);
    EXPECT_EQ(sm2.pdo_mappings[0].size_bytes, 19u);
    EXPECT_EQ(sm2.pdo_mappings[1].pdo_index, 0x1601u);
    EXPECT_EQ(sm2.pdo_mappings[1].size_bytes, 8u);
    EXPECT_EQ(sm2.pdo_mappings[2].pdo_index, 0x1602u);
    EXPECT_EQ(sm2.pdo_mappings[2].size_bytes, 8u);

    uint16_t total = 0;
    for (const auto& p : sm2.pdo_mappings) total += p.size_bytes;
    EXPECT_EQ(total, 35u);
}

TEST(SynapticonMultiPDOAssignmentTest, StandardAssignmentSM3) {
    auto assignment = makeStandardPDOAssignment();
    const auto& sm3 = assignment.sm_configs[1];
    EXPECT_EQ(sm3.sm_index, 3u);
    EXPECT_EQ(sm3.phys_start_addr, 0x1C00u);
    EXPECT_EQ(sm3.control_byte, 0x20u);
    ASSERT_EQ(sm3.pdo_mappings.size(), 4u);

    EXPECT_EQ(sm3.pdo_mappings[0].pdo_index, 0x1A00u);
    EXPECT_EQ(sm3.pdo_mappings[0].size_bytes, 13u);
    EXPECT_EQ(sm3.pdo_mappings[1].pdo_index, 0x1A01u);
    EXPECT_EQ(sm3.pdo_mappings[1].size_bytes, 12u);
    EXPECT_EQ(sm3.pdo_mappings[2].pdo_index, 0x1A02u);
    EXPECT_EQ(sm3.pdo_mappings[2].size_bytes, 4u);
    EXPECT_EQ(sm3.pdo_mappings[3].pdo_index, 0x1A03u);
    EXPECT_EQ(sm3.pdo_mappings[3].size_bytes, 18u);

    uint16_t total = 0;
    for (const auto& p : sm3.pdo_mappings) total += p.size_bytes;
    EXPECT_EQ(total, 47u);
}

// ============================================================================
// Part 5: Multi-PDO Assignment Builders — FSoE
// ============================================================================

TEST(SynapticonMultiPDOAssignmentTest, FSoEAssignmentHasTwoSMs) {
    auto assignment = makeFSoEPDOAssignment();
    ASSERT_EQ(assignment.sm_configs.size(), 2u);
}

TEST(SynapticonMultiPDOAssignmentTest, FSoEAssignmentSM2) {
    auto assignment = makeFSoEPDOAssignment();
    const auto& sm2 = assignment.sm_configs[0];
    EXPECT_EQ(sm2.sm_index, 2u);
    EXPECT_EQ(sm2.phys_start_addr, 0x1800u);
    EXPECT_EQ(sm2.control_byte, 0x64u);
    ASSERT_EQ(sm2.pdo_mappings.size(), 1u);
    EXPECT_EQ(sm2.pdo_mappings[0].pdo_index, 0x1700u);
    EXPECT_EQ(sm2.pdo_mappings[0].size_bytes, 11u);
}

TEST(SynapticonMultiPDOAssignmentTest, FSoEAssignmentSM3) {
    auto assignment = makeFSoEPDOAssignment();
    const auto& sm3 = assignment.sm_configs[1];
    EXPECT_EQ(sm3.sm_index, 3u);
    EXPECT_EQ(sm3.phys_start_addr, 0x1C00u);
    EXPECT_EQ(sm3.control_byte, 0x20u);
    ASSERT_EQ(sm3.pdo_mappings.size(), 1u);
    EXPECT_EQ(sm3.pdo_mappings[0].pdo_index, 0x1B00u);
    EXPECT_EQ(sm3.pdo_mappings[0].size_bytes, 31u);
}

// ============================================================================
// Part 6: Multi-PDO Assignment Builders — Combined
// ============================================================================

TEST(SynapticonMultiPDOAssignmentTest, CombinedAssignmentHasTwoSMs) {
    auto assignment = makeCombinedPDOAssignment();
    ASSERT_EQ(assignment.sm_configs.size(), 2u);
}

TEST(SynapticonMultiPDOAssignmentTest, CombinedAssignmentSM2) {
    auto assignment = makeCombinedPDOAssignment();
    const auto& sm2 = assignment.sm_configs[0];
    EXPECT_EQ(sm2.sm_index, 2u);
    ASSERT_EQ(sm2.pdo_mappings.size(), 4u);

    EXPECT_EQ(sm2.pdo_mappings[0].pdo_index, 0x1600u);
    EXPECT_EQ(sm2.pdo_mappings[1].pdo_index, 0x1601u);
    EXPECT_EQ(sm2.pdo_mappings[2].pdo_index, 0x1602u);
    EXPECT_EQ(sm2.pdo_mappings[3].pdo_index, 0x1700u);

    uint16_t total = 0;
    for (const auto& p : sm2.pdo_mappings) total += p.size_bytes;
    EXPECT_EQ(total, 46u);  // 35 + 11
}

TEST(SynapticonMultiPDOAssignmentTest, CombinedAssignmentSM3) {
    auto assignment = makeCombinedPDOAssignment();
    const auto& sm3 = assignment.sm_configs[1];
    EXPECT_EQ(sm3.sm_index, 3u);
    ASSERT_EQ(sm3.pdo_mappings.size(), 5u);

    EXPECT_EQ(sm3.pdo_mappings[0].pdo_index, 0x1A00u);
    EXPECT_EQ(sm3.pdo_mappings[1].pdo_index, 0x1A01u);
    EXPECT_EQ(sm3.pdo_mappings[2].pdo_index, 0x1A02u);
    EXPECT_EQ(sm3.pdo_mappings[3].pdo_index, 0x1A03u);
    EXPECT_EQ(sm3.pdo_mappings[4].pdo_index, 0x1B00u);

    uint16_t total = 0;
    for (const auto& p : sm3.pdo_mappings) total += p.size_bytes;
    EXPECT_EQ(total, 78u);  // 47 + 31
}

// ============================================================================
// Part 7: Multi-PDO Assignment Builders — CST Mode
// ============================================================================

TEST(SynapticonMultiPDOAssignmentTest, CSTModeAssignment) {
    auto assignment = makeCSTModePDOAssignment();
    ASSERT_EQ(assignment.sm_configs.size(), 2u);

    const auto& sm2 = assignment.sm_configs[0];
    ASSERT_EQ(sm2.pdo_mappings.size(), 1u);
    EXPECT_EQ(sm2.pdo_mappings[0].pdo_index, 0x1600u);
    EXPECT_EQ(sm2.pdo_mappings[0].size_bytes, 19u);

    const auto& sm3 = assignment.sm_configs[1];
    ASSERT_EQ(sm3.pdo_mappings.size(), 1u);
    EXPECT_EQ(sm3.pdo_mappings[0].pdo_index, 0x1A00u);
    EXPECT_EQ(sm3.pdo_mappings[0].size_bytes, 13u);
}

// ============================================================================
// Part 8: Multi-PDO Assignment Builders — Custom (explicit indices)
// ============================================================================

TEST(SynapticonMultiPDOAssignmentTest, CustomAssignmentKnownIndices) {
    auto assignment = makePDOAssignment({0x1600, 0x1601}, {0x1A00, 0x1A03});
    ASSERT_EQ(assignment.sm_configs.size(), 2u);

    const auto& sm2 = assignment.sm_configs[0];
    ASSERT_EQ(sm2.pdo_mappings.size(), 2u);
    EXPECT_EQ(sm2.pdo_mappings[0].pdo_index, 0x1600u);
    EXPECT_EQ(sm2.pdo_mappings[0].size_bytes, 19u);
    EXPECT_EQ(sm2.pdo_mappings[1].pdo_index, 0x1601u);
    EXPECT_EQ(sm2.pdo_mappings[1].size_bytes, 8u);

    const auto& sm3 = assignment.sm_configs[1];
    ASSERT_EQ(sm3.pdo_mappings.size(), 2u);
    EXPECT_EQ(sm3.pdo_mappings[0].pdo_index, 0x1A00u);
    EXPECT_EQ(sm3.pdo_mappings[0].size_bytes, 13u);
    EXPECT_EQ(sm3.pdo_mappings[1].pdo_index, 0x1A03u);
    EXPECT_EQ(sm3.pdo_mappings[1].size_bytes, 18u);
}

TEST(SynapticonMultiPDOAssignmentTest, CustomAssignmentSkipsUnknownIndices) {
    auto assignment = makePDOAssignment({0x1600, 0x9999}, {0x1A00, 0xFFFF});
    ASSERT_EQ(assignment.sm_configs.size(), 2u);

    // Unknown indices should be skipped
    EXPECT_EQ(assignment.sm_configs[0].pdo_mappings.size(), 1u);
    EXPECT_EQ(assignment.sm_configs[1].pdo_mappings.size(), 1u);
}

TEST(SynapticonMultiPDOAssignmentTest, CustomAssignmentFSoEIndices) {
    auto assignment = makePDOAssignment({0x1700}, {0x1B00});
    ASSERT_EQ(assignment.sm_configs.size(), 2u);

    EXPECT_EQ(assignment.sm_configs[0].pdo_mappings[0].pdo_index, 0x1700u);
    EXPECT_EQ(assignment.sm_configs[0].pdo_mappings[0].size_bytes, 11u);
    EXPECT_EQ(assignment.sm_configs[1].pdo_mappings[0].pdo_index, 0x1B00u);
    EXPECT_EQ(assignment.sm_configs[1].pdo_mappings[0].size_bytes, 31u);
}

TEST(SynapticonMultiPDOAssignmentTest, CustomAssignmentEmptyLists) {
    auto assignment = makePDOAssignment({}, {});
    EXPECT_TRUE(assignment.sm_configs.empty());
}

// ============================================================================
// Part 9: FSoE PDO field access tests
// ============================================================================

TEST(SynapticonFSoEPDOTest, RxPDO_1700_FieldAccess) {
    SOMANET_RxPDO_1700 pdo{};
    pdo.fsoe_command = 0x01;
    pdo.safety_flags = SOMANET_RxPDO_1700::kSTO | SOMANET_RxPDO_1700::kSOS;
    pdo.fsoe_crc_0 = 0x1234;
    pdo.safe_outputs = SOMANET_RxPDO_1700::kSafeOutput1;
    pdo.fsoe_crc_1 = 0x5678;
    pdo.fsoe_connection_id = 0xABCD;

    EXPECT_EQ(pdo.fsoe_command, 0x01u);
    EXPECT_TRUE(pdo.safety_flags & SOMANET_RxPDO_1700::kSTO);
    EXPECT_TRUE(pdo.safety_flags & SOMANET_RxPDO_1700::kSOS);
    EXPECT_FALSE(pdo.safety_flags & SOMANET_RxPDO_1700::kSS1);
    EXPECT_EQ(pdo.fsoe_crc_0, 0x1234u);
    EXPECT_TRUE(pdo.safe_outputs & SOMANET_RxPDO_1700::kSafeOutput1);
    EXPECT_FALSE(pdo.safe_outputs & SOMANET_RxPDO_1700::kSafeOutput2);
    EXPECT_EQ(pdo.fsoe_connection_id, 0xABCDu);
}

TEST(SynapticonFSoEPDOTest, TxPDO_1B00_FieldAccess) {
    SOMANET_TxPDO_1B00 pdo{};
    pdo.fsoe_command = 0x02;
    pdo.safety_state_flags = SOMANET_TxPDO_1B00::kSTOState | SOMANET_TxPDO_1B00::kSOSState;
    pdo.diagnostic_flags = SOMANET_TxPDO_1B00::kSafePositionValid | SOMANET_TxPDO_1B00::kSafeSpeedValid;
    pdo.safe_position_actual = 0x1000;
    pdo.safe_velocity_actual = 0x2000;
    pdo.fsoe_connection_id = 0x4321;

    EXPECT_EQ(pdo.fsoe_command, 0x02u);
    EXPECT_TRUE(pdo.safety_state_flags & SOMANET_TxPDO_1B00::kSTOState);
    EXPECT_TRUE(pdo.safety_state_flags & SOMANET_TxPDO_1B00::kSOSState);
    EXPECT_FALSE(pdo.safety_state_flags & SOMANET_TxPDO_1B00::kSS1State);
    EXPECT_TRUE(pdo.diagnostic_flags & SOMANET_TxPDO_1B00::kSafePositionValid);
    EXPECT_TRUE(pdo.diagnostic_flags & SOMANET_TxPDO_1B00::kSafeSpeedValid);
    EXPECT_FALSE(pdo.diagnostic_flags & SOMANET_TxPDO_1B00::kTemperatureWarning);
    EXPECT_EQ(pdo.safe_position_actual, 0x1000u);
    EXPECT_EQ(pdo.safe_velocity_actual, 0x2000u);
    EXPECT_EQ(pdo.fsoe_connection_id, 0x4321u);
}

// ============================================================================
// Part 10: Standard PDO field access tests
// ============================================================================

TEST(SynapticonPDOTest, RxPDO_1600_FieldAccess) {
    SOMANET_RxPDO_1600 pdo{};
    pdo.controlword = 0x000F;
    pdo.modes_of_operation = 8;  // CST mode
    pdo.target_torque = 100;
    pdo.target_position = 0x12345678;
    pdo.target_velocity = 0x9ABCDEF0;
    pdo.torque_offset = 50;
    pdo.tuning_command = 0xDEADBEEF;

    EXPECT_EQ(pdo.controlword, 0x000Fu);
    EXPECT_EQ(pdo.modes_of_operation, 8);
    EXPECT_EQ(pdo.target_torque, 100);
    EXPECT_EQ(pdo.target_position, 0x12345678);
    EXPECT_EQ(pdo.target_velocity, static_cast<int32_t>(0x9ABCDEF0));
    EXPECT_EQ(pdo.torque_offset, 50);
    EXPECT_EQ(pdo.tuning_command, 0xDEADBEEFu);
}

TEST(SynapticonPDOTest, TxPDO_1A00_FieldAccess) {
    SOMANET_TxPDO_1A00 pdo{};
    pdo.statusword = 0x0637;
    pdo.modes_of_operation_display = 8;
    pdo.position_actual = 0x12345678;
    pdo.velocity_actual = 0x9ABCDEF0;
    pdo.torque_actual = -50;

    EXPECT_EQ(pdo.statusword, 0x0637u);
    EXPECT_EQ(pdo.modes_of_operation_display, 8);
    EXPECT_EQ(pdo.position_actual, 0x12345678);
    EXPECT_EQ(pdo.velocity_actual, static_cast<int32_t>(0x9ABCDEF0));
    EXPECT_EQ(pdo.torque_actual, -50);
}
