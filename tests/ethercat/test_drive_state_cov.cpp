/**
 * @file test_drive_state_cov.cpp
 * @brief Coverage for Emulator::CiA402::DriveState
 *        (src/ethercat/SlaveEmulator_CiA402.cpp): status-word encoding per
 *        state + mode/homing bits, all processControlWord transition edges,
 *        and simulate() per operating mode.
 */

#include <gtest/gtest.h>

#include "tether/ethercat/SlaveEmulator.hpp"

using namespace EtherCAT::Emulator::CiA402;

namespace {

// ============================================================================
// getStatusWord — per-state encoding + extra bits
// ============================================================================

TEST(DriveStateCov, StatusWordPerState) {
    DriveState d;
    struct { State s; uint16_t sw; } cases[] = {
        {State::NOT_READY_TO_SWITCH_ON, 0x0000},
        {State::SWITCH_ON_DISABLED,     0x0040},
        {State::READY_TO_SWITCH_ON,     0x0021},
        {State::SWITCHED_ON,            0x0023},
        {State::OPERATION_ENABLED,      0x0027},
        {State::QUICK_STOP_ACTIVE,      0x0007},
        {State::FAULT_REACTION_ACTIVE,  0x000F},
        {State::FAULT,                  0x0008},
    };
    for (auto& c : cases) {
        d.state = c.s;
        d.operation_mode = 0;
        d.target_mode = 1;              // != → no mode-match bit
        d.homing_complete = false;
        EXPECT_EQ(d.getStatusWord(), c.sw) << static_cast<int>(c.s);
    }
}

TEST(DriveStateCov, StatusWordModeMatchBit) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 8;
    d.target_mode = 8;                  // match → bit10 set
    EXPECT_EQ(d.getStatusWord(), 0x0027 | 0x0400);
    d.target_mode = 9;                  // mismatch → bit10 clear
    EXPECT_EQ(d.getStatusWord(), 0x0027);
}

TEST(DriveStateCov, StatusWordHomingBit) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 6;               // homing mode
    d.target_mode = 6;
    d.homing_complete = true;
    EXPECT_EQ(d.getStatusWord(), 0x0027 | 0x0400 | 0x1000);
    // Homing bit requires mode 6 — clear otherwise.
    d.operation_mode = 8;
    EXPECT_EQ(d.getStatusWord() & 0x1000, 0u);
}

// ============================================================================
// processControlWord — all transition edges
// ============================================================================

TEST(DriveStateCov, NotReadyAutoTransitions) {
    DriveState d;
    d.state = State::NOT_READY_TO_SWITCH_ON;
    d.processControlWord(0);            // any cw → auto-advance
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

TEST(DriveStateCov, ShutdownFromSwitchOnDisabled) {
    DriveState d;
    d.state = State::SWITCH_ON_DISABLED;
    d.processControlWord(0x0006);       // Shutdown
    EXPECT_EQ(d.state, State::READY_TO_SWITCH_ON);
    // Non-shutdown stays put.
    d.state = State::SWITCH_ON_DISABLED;
    d.processControlWord(0x0000);
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

TEST(DriveStateCov, ReadyToSwitchOnEdges) {
    DriveState d;
    d.state = State::READY_TO_SWITCH_ON;
    d.processControlWord(0x0007);       // Switch On
    EXPECT_EQ(d.state, State::SWITCHED_ON);

    d.state = State::READY_TO_SWITCH_ON;
    d.processControlWord(0x0000);       // Disable Voltage
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

TEST(DriveStateCov, SwitchedOnEdges) {
    DriveState d;
    d.state = State::SWITCHED_ON;
    d.processControlWord(0x000F);       // Enable Operation
    EXPECT_EQ(d.state, State::OPERATION_ENABLED);

    d.state = State::SWITCHED_ON;
    d.processControlWord(0x0006);       // Shutdown
    EXPECT_EQ(d.state, State::READY_TO_SWITCH_ON);

    d.state = State::SWITCHED_ON;
    d.processControlWord(0x0000);       // Disable Voltage
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

TEST(DriveStateCov, OperationEnabledEdges) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.processControlWord(0x0002);       // Quick Stop (bit2=0)
    EXPECT_EQ(d.state, State::QUICK_STOP_ACTIVE);

    d.state = State::OPERATION_ENABLED;
    d.processControlWord(0x0007);       // Disable Operation (bit3=0)
    EXPECT_EQ(d.state, State::SWITCHED_ON);

    d.state = State::OPERATION_ENABLED;
    d.processControlWord(0x0006);       // Shutdown (bit3 also 0 → first match)
    EXPECT_EQ(d.state, State::SWITCHED_ON);  // disable-op hits before shutdown

    d.state = State::OPERATION_ENABLED;
    d.processControlWord(0x000F);       // stay enabled
    EXPECT_EQ(d.state, State::OPERATION_ENABLED);
}

TEST(DriveStateCov, QuickStopEdges) {
    DriveState d;
    d.state = State::QUICK_STOP_ACTIVE;
    d.processControlWord(0x000F);       // re-enable
    EXPECT_EQ(d.state, State::OPERATION_ENABLED);

    d.state = State::QUICK_STOP_ACTIVE;
    d.processControlWord(0x0000);       // disable voltage
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

TEST(DriveStateCov, FaultReactionAutoTransitions) {
    DriveState d;
    d.state = State::FAULT_REACTION_ACTIVE;
    d.processControlWord(0);
    EXPECT_EQ(d.state, State::FAULT);
}

TEST(DriveStateCov, FaultResetRisingEdge) {
    DriveState d;
    d.state = State::FAULT;
    d.error_code = 0x1234;
    d.prev_reset = false;

    d.processControlWord(0x0080);       // fault-reset rising edge
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
    EXPECT_EQ(d.error_code, 0);
    EXPECT_TRUE(d.prev_reset);

    // Reset latched high — no repeated transition.
    d.state = State::FAULT;
    d.error_code = 0x99;
    d.processControlWord(0x0080);       // still high, no edge
    EXPECT_EQ(d.state, State::FAULT);   // stays fault (edge consumed)
    EXPECT_EQ(d.error_code, 0x99);

    // Falling edge then rising → resets again.
    d.processControlWord(0x0000);       // bit7 low
    EXPECT_FALSE(d.prev_reset);
    d.processControlWord(0x0080);       // rising again
    EXPECT_EQ(d.state, State::SWITCH_ON_DISABLED);
}

// ============================================================================
// simulate — motor model per mode
// ============================================================================

TEST(DriveStateCov, SimulateNonEnabledZeroesVelocity) {
    DriveState d;
    d.state = State::SWITCHED_ON;
    d.actual_velocity = 500;
    d.simulate(1000);
    EXPECT_EQ(d.actual_velocity, 0);
}

TEST(DriveStateCov, SimulateProfilePosition) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 1;               // PP
    d.target_position = 500;
    d.actual_position = 0;
    d.simulate(1000);                   // max_step = 1000
    EXPECT_EQ(d.actual_position, 500);  // error <= max_step → full move
    EXPECT_GT(d.actual_velocity, 0);

    // Large error clamped to max_step.
    d.target_position = 100000;
    d.actual_position = 0;
    d.simulate(1000);
    EXPECT_EQ(d.actual_position, 1000);
}

TEST(DriveStateCov, SimulateProfilePositionNegative) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 1;
    d.target_position = -100000;
    d.actual_position = 0;
    d.simulate(1000);
    EXPECT_EQ(d.actual_position, -1000); // clamped negative
    EXPECT_LT(d.actual_velocity, 0);
}

TEST(DriveStateCov, SimulateProfileVelocity) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 3;               // PV
    d.target_velocity = 2000;
    d.actual_position = 0;
    d.simulate(1000);                   // 1 ms
    EXPECT_EQ(d.actual_velocity, 2000);
    EXPECT_EQ(d.actual_position, 2);    // v * dt(s)
}

TEST(DriveStateCov, SimulateProfileTorque) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 4;               // PT
    d.target_torque = 300;
    d.simulate(1000);
    EXPECT_EQ(d.actual_torque, 300);
}

TEST(DriveStateCov, SimulateCyclicModes) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;

    d.operation_mode = 8;               // CSP — position snaps to target
    d.target_position = 4242;
    d.simulate(1000);
    EXPECT_EQ(d.actual_position, 4242);

    d.operation_mode = 9;               // CSV
    d.actual_position = 0;
    d.target_velocity = 3000;
    d.simulate(1000);
    EXPECT_EQ(d.actual_velocity, 3000);
    EXPECT_EQ(d.actual_position, 3);

    d.operation_mode = 10;              // CST
    d.target_torque = -250;
    d.simulate(1000);
    EXPECT_EQ(d.actual_torque, -250);
}

TEST(DriveStateCov, SimulateUnknownModeNoop) {
    DriveState d;
    d.state = State::OPERATION_ENABLED;
    d.operation_mode = 99;              // no case → untouched
    d.actual_position = 111;
    d.simulate(1000);
    EXPECT_EQ(d.actual_position, 111);
}

}  // namespace
