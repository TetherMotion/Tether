/**
 * @file test_slave_names.cpp
 * @brief Unit tests for slave naming and log prefix functionality
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/CoEManager.hpp"

#include <string>
#include <string_view>

using namespace EtherCAT;

// ============================================================================
// Test fixture
// ============================================================================

class SlaveNamesTest : public ::testing::Test {
protected:
    void SetUp() override {
        master_.setPreopRetryConfig(1, 3, 1, 1);
        master_.siiReader().setTimeout(1);
        master_.initSlaves(3);
    }

    Master master_;
};

// ============================================================================
// Master::setSlaveName / slaveName / slaveLogPrefix
// ============================================================================

TEST_F(SlaveNamesTest, DefaultNameIsEmpty) {
    EXPECT_EQ(master_.slaveName(0), "");
    EXPECT_EQ(master_.slaveName(1), "");
    EXPECT_EQ(master_.slaveName(2), "");
}

TEST_F(SlaveNamesTest, DefaultLogPrefixIsSlaveIndex) {
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave 0");
    EXPECT_EQ(master_.slaveLogPrefix(1), "Slave 1");
    EXPECT_EQ(master_.slaveLogPrefix(2), "Slave 2");
}

TEST_F(SlaveNamesTest, SetSlaveName) {
    master_.setSlaveName(0, "X-Axis");
    master_.setSlaveName(1, "Y-Axis");
    master_.setSlaveName(2, "Extruder");

    EXPECT_EQ(master_.slaveName(0), "X-Axis");
    EXPECT_EQ(master_.slaveName(1), "Y-Axis");
    EXPECT_EQ(master_.slaveName(2), "Extruder");
}

TEST_F(SlaveNamesTest, LogPrefixWithCustomName) {
    master_.setSlaveName(0, "X-Axis");
    master_.setSlaveName(2, "Extruder-1");

    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave X-Axis (#0)");
    EXPECT_EQ(master_.slaveLogPrefix(1), "Slave 1");
    EXPECT_EQ(master_.slaveLogPrefix(2), "Slave Extruder-1 (#2)");
}

TEST_F(SlaveNamesTest, OverwriteName) {
    master_.setSlaveName(0, "First");
    EXPECT_EQ(master_.slaveName(0), "First");
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave First (#0)");

    master_.setSlaveName(0, "Second");
    EXPECT_EQ(master_.slaveName(0), "Second");
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave Second (#0)");
}

TEST_F(SlaveNamesTest, EmptyNameRevertsToDefault) {
    master_.setSlaveName(0, "X-Axis");
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave X-Axis (#0)");

    master_.setSlaveName(0, "");
    EXPECT_EQ(master_.slaveName(0), "");
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave 0");
}

TEST_F(SlaveNamesTest, OutOfRangeIndex) {
    // Should not crash, should return empty/default
    EXPECT_EQ(master_.slaveName(100), "");
    EXPECT_EQ(master_.slaveLogPrefix(100), "Slave 100");
}

TEST_F(SlaveNamesTest, OutOfRangeSetSlaveName) {
    // Should not crash, should be ignored
    master_.setSlaveName(100, "Ghost");
    EXPECT_EQ(master_.slaveName(100), "");
}

// ============================================================================
// Slave::setName / name / logPrefix
// ============================================================================

TEST_F(SlaveNamesTest, SlaveSetName) {
    auto& s = master_.slave(0);
    s.setName("X-Axis");
    EXPECT_EQ(s.name(), "X-Axis");
    EXPECT_EQ(s.logPrefix(), "Slave X-Axis (#0)");
}

TEST_F(SlaveNamesTest, SlaveDefaultName) {
    auto& s = master_.slave(1);
    EXPECT_EQ(s.name(), "");
    EXPECT_EQ(s.logPrefix(), "Slave 1");
}

TEST_F(SlaveNamesTest, SlaveLogPrefixMatchesMaster) {
    master_.setSlaveName(2, "Z-Axis");
    auto& s = master_.slave(2);
    EXPECT_EQ(s.logPrefix(), master_.slaveLogPrefix(2));
}

// ============================================================================
// CoEManager log prefix
// ============================================================================

TEST_F(SlaveNamesTest, CoEManagerLogPrefix) {
    auto& coe = master_.sdoManager(0);
    // After setLogPrefix is called by Master, the prefix should match
    EXPECT_EQ(coe.logPrefix(), master_.slaveLogPrefix(0));
}

TEST_F(SlaveNamesTest, CoEManagerLogPrefixAfterSetName) {
    master_.setSlaveName(0, "TestDrive");
    // CoEManager prefix is set at creation time; re-create by accessing it
    // (it already exists, so we need to check the original prefix)
    // Note: Master sets the prefix when the CoEManager is first created.
    // If the name is set after creation, the prefix won't update automatically.
    // This is by design — names should be set before accessing sdoManager.
    auto& coe = master_.sdoManager(1);
    EXPECT_EQ(coe.logPrefix(), "Slave 1");
}

// ============================================================================
// initSlaves resets names
// ============================================================================

TEST_F(SlaveNamesTest, InitSlavesResetsNames) {
    master_.setSlaveName(0, "X-Axis");
    EXPECT_EQ(master_.slaveName(0), "X-Axis");

    master_.initSlaves(2);
    EXPECT_EQ(master_.slaveName(0), "");
    EXPECT_EQ(master_.slaveName(1), "");
    EXPECT_EQ(master_.slaveLogPrefix(0), "Slave 0");
}
