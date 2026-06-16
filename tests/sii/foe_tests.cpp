/**
 * @file foe_tests.cpp
 * @brief Basic FoE tests (opcode/error string helpers)
 *
 * These are kept in sii/ for backward compat; comprehensive tests
 * are in ethercat/test_foe.cpp.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/FoE.hpp"

using namespace EtherCAT::FoE;

TEST(FoE_Header, OpcodeStrings) {
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::RRQ), "RRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::WRQ), "WRQ");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::DATA), "DATA");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ACK), "ACK");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::ERROR), "ERROR");
    EXPECT_STREQ(foe_opcode_string(FoEOpcode::BUSY), "BUSY");
}

TEST(FoE_Header, ErrorStrings) {
    EXPECT_STREQ(foe_error_string(FoEError::SUCCESS), "Success");
    EXPECT_STREQ(foe_error_string(FoEError::NOT_FOUND), "File not found");
    EXPECT_STREQ(foe_error_string(FoEError::ACCESS_DENIED), "Access denied");
    EXPECT_STREQ(foe_error_string(FoEError::DISK_FULL), "Disk full");
}
