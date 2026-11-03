/**
 * @file test_klipper_callbacks.cpp
 * @brief Coverage sweep for KlippyInstanceCallbacks.ipp — drives every
 *        stored GcodeCallbacks member (home/motor-enable/emergency-stop/
 *        retraction/sd-print/status/temps/TMC/spindle/coolant/rotation/
 *        scaling/print control) plus the dead branches inside
 *        handleExtendedCommand().
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/objects/Homing.hpp"

using namespace tether::klipper;

namespace {

std::string uniquePath(const char* stem) {
    return std::string("/tmp/") + stem + "_" + std::to_string(::getpid()) +
           "_" + std::to_string(reinterpret_cast<uintptr_t>(stem));
}

}  // namespace

class KlippyCallbacksTest : public ::testing::Test {
protected:
    void SetUp() override {
        sockPath_ = uniquePath("klippy_cb_sock");
        settingsPath_ = uniquePath("klippy_cb_set");
        cfgPath_ = uniquePath("klippy_cb_cfg");
        std::ofstream f(cfgPath_);
        f << R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[stepper_x]
position_max = 200
position_min = 0
position_endstop = 0
homing_speed = 40
second_homing_speed = 5
homing_positive_dir = false
[stepper_y]
position_max = 200
position_min = 0
position_endstop = 200
homing_speed = 40
homing_positive_dir = true
[stepper_z]
position_max = 200
position_min = 0
position_endstop = 10
homing_speed = 20
[extruder]
step_pin = PE0
dir_pin = PE1
enable_pin = PE2
[heater_bed]
[fan]
[probe]
pin = ^PC4
x_offset = 0
y_offset = 0
z_offset = 1.5
speed = 5
samples = 1
[bed_mesh]
probe_count = 3,3
mesh_min = 10,10
mesh_max = 190,190
[z_tilt]
z_positions = 0,0  200,0  100,200
points = 30,30  170,30  100,170
[bed_screws]
screw1 = 10,10
screw2 = 190,10
screw3 = 100,190
[screws_tilt_adjust]
screw1 = 10,10
screw2 = 190,10
[quad_gantry_level]
gantry_corners = 0,0 200,0 200,200 0,200
points = 50,50 50,150 150,150 150,50
[idle_timeout]
timeout = 600
gcode = M84
[pause_resume]
[display_status]
[virtual_sdcard]
path = /tmp/tether_test_sdcard_cb
[output_pin led]
pin = PD5
pwm = True
[servo s0]
pin = PD6
[temperature_fan tfan]
pin = PD7
sensor_type = EPCOS 100K B57560G104F
[heater_fan hfan]
pin = PB0
[controller_fan cfan]
pin = PB1
[tmc2209 stepper_x]
uart_pin = PC0
run_current = 0.8
[adxl345]
cs_pin = PC5
[input_shaper]
shaper_type_x = zv
shaper_freq_x = 40
[gcode_macro M_TEST]
gcode = M117 macro
[delayed_gcode dg0]
initial_duration = 0.1
gcode = M117 delayed
[firmware_retraction]
retract_length = 1.0
retract_speed = 30
[exclude_object]
[save_variables]
filename = /tmp/tether_test_cb_vars.cfg
[force_move]
enable_force_move = True
[endstop_phase stepper_z]
phase_count = 64
[safe_z_home]
home_xy_position = 100,100
[spindle_tool]
[coolant]
)";
        f.close();

        klippy::KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = sockPath_;
        cfg.settingsPath = settingsPath_;
        cfg.sdcardDir = "/tmp/tether_test_sdcard_cb";
        instance_ = std::make_unique<klippy::KlippyInstance>(cfg);
        ASSERT_TRUE(instance_->loadConfig(cfgPath_));

        probe_ = std::make_shared<objects::Probe>(0, []() { return false; });
        probe_->setZOffset(1.5);
        instance_->setProbe(probe_);
    }

    void TearDown() override {
        instance_.reset();
        ::unlink(sockPath_.c_str());
        std::filesystem::remove(cfgPath_);
        std::filesystem::remove(settingsPath_);
        std::filesystem::remove("/tmp/tether_test_cb_vars.cfg");
    }

    bool exec(const std::string& g) { return instance_->executeGcode(g); }

    std::string sockPath_, settingsPath_, cfgPath_;
    std::unique_ptr<klippy::KlippyInstance> instance_;
    std::shared_ptr<objects::Probe> probe_;
};

// Homing simulation path: axes with stepperHoming config spin up a
// HomingAxisSimulator; ticks advance it.
TEST_F(KlippyCallbacksTest, HomingSimulationPath) {
    EXPECT_TRUE(exec("G28 X Y Z"));
    EXPECT_TRUE(exec("G28 X"));
    EXPECT_TRUE(exec("G28 Y"));
    EXPECT_TRUE(exec("G28 Z"));
    // Drive the active homing axes through several ticks.
    for (int i = 0; i < 200; ++i) instance_->tick();
}

// Motor enable / disable, emergency stop.
TEST_F(KlippyCallbacksTest, MotorEnableAndEstop) {
    EXPECT_TRUE(exec("M17"));
    EXPECT_TRUE(exec("M84"));
    EXPECT_TRUE(exec("M18"));
    EXPECT_TRUE(exec("M17 XYZ"));
    EXPECT_TRUE(exec("M112"));
}

// Position set + status queries.
TEST_F(KlippyCallbacksTest, PositionAndStatusQueries) {
    EXPECT_TRUE(exec("G92 X10 Y20 Z5 E1.5"));
    EXPECT_TRUE(exec("M114"));
    EXPECT_TRUE(exec("M119"));
    EXPECT_TRUE(exec("M105"));
    EXPECT_TRUE(exec("M155 S2"));
    EXPECT_TRUE(exec("GET_POSITION"));
    EXPECT_TRUE(exec("QUERY_ENDSTOPS"));
}

// Firmware retraction + SD card print control.
TEST_F(KlippyCallbacksTest, RetractionAndSdPrint) {
    EXPECT_TRUE(exec("G10"));
    EXPECT_TRUE(exec("G11"));
    EXPECT_TRUE(exec("M23 print.gcode"));
    EXPECT_TRUE(exec("M24"));
    EXPECT_TRUE(exec("M25"));
    EXPECT_TRUE(exec("M27"));
    EXPECT_TRUE(exec("M73 P42"));
    EXPECT_TRUE(exec("M117 status msg"));
}

// Temperature waits + PID autotune + pressure advance + bed level.
TEST_F(KlippyCallbacksTest, TempWaitPidAndLeveling) {
    EXPECT_TRUE(exec("M104 S150"));
    EXPECT_TRUE(exec("M109 S150"));
    EXPECT_TRUE(exec("M140 S55"));
    EXPECT_TRUE(exec("M190 S55"));
    EXPECT_TRUE(exec("M400"));
    EXPECT_TRUE(exec("M303 E0 S180 C3"));
    EXPECT_TRUE(exec("M900 K0.05"));
    EXPECT_TRUE(exec("G29"));
    EXPECT_TRUE(exec("PID_CALIBRATE HEATER=extruder TARGET=200"));
}

// Offsets, TMC, spindle/coolant, coordinate rotation/scaling.
TEST_F(KlippyCallbacksTest, OffsetsTmcSpindleRotation) {
    EXPECT_TRUE(exec("M206 X1 Y-2"));
    EXPECT_TRUE(exec("M218 T0 X5"));
    EXPECT_TRUE(exec("M3 S1000"));
    EXPECT_TRUE(exec("M4 S500"));
    EXPECT_TRUE(exec("M5"));
    EXPECT_TRUE(exec("M7"));
    EXPECT_TRUE(exec("M8"));
    EXPECT_TRUE(exec("M9"));
    EXPECT_TRUE(exec("G68 X10 Y10 R30"));
    EXPECT_TRUE(exec("G69"));
    EXPECT_TRUE(exec("G51 X2 Y2 Z1"));
    EXPECT_TRUE(exec("G50"));
    EXPECT_TRUE(exec("G33"));
}

// Extended-command branches not hit by the existing sweep.
TEST_F(KlippyCallbacksTest, ExtendedBranchSweep) {
    EXPECT_TRUE(exec("ACCELEROMETER_MEASURE"));
    EXPECT_TRUE(exec("SET_SERVO SERVO=s0 ANGLE=30"));
    EXPECT_TRUE(exec("SET_SERVO SERVO=s0 WIDTH=1.5"));
    EXPECT_TRUE(exec("SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=70"));
    EXPECT_TRUE(exec("SET_HEATER_TEMPERATURE HEATER=unknown TARGET=50"));
    EXPECT_TRUE(exec("SET_TEMPERATURE_FAN FAN=tfan TARGET=45"));
    EXPECT_TRUE(exec("SET_TEMPERATURE_FAN FAN=missing TARGET=45"));
    EXPECT_TRUE(exec("SET_LED LED=led RED=0.1 GREEN=0.2 BLUE=0.3"));
    EXPECT_TRUE(exec("SET_NEOPIXEL LED=np RED=1 INDEX=2"));
    EXPECT_TRUE(exec("SET_DOTSTAR LED=ds RED=0.4"));
    EXPECT_TRUE(exec("SET_PROBE PROBE=probe Z_OFFSET=1.7"));
    EXPECT_TRUE(exec("SET_PROBE PROBE=probe VIRTUAL_ENDSTOP=1"));
    EXPECT_TRUE(exec("PROBE_CALIBRATE"));
    EXPECT_TRUE(exec("Z_OFFSET_APPLY_PROBE"));
    EXPECT_TRUE(exec("Z_OFFSET_APPLY_ENDSTOP"));
    EXPECT_TRUE(exec("SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=0"));
    EXPECT_TRUE(exec("SET_IDLE_TIMEOUT TIMEOUT=120"));
    EXPECT_TRUE(exec("DUMP_TMC STEPPER=stepper_x"));
    EXPECT_TRUE(exec("INIT_TMC STEPPER=stepper_x"));
    EXPECT_TRUE(exec("SET_TMC_FIELD STEPPER=stepper_x FIELD=GCONF VALUE=5"));
    EXPECT_TRUE(exec("MANUAL_PROBE"));
    EXPECT_TRUE(exec("ABORT"));
    EXPECT_TRUE(exec("ACCEPT"));
    EXPECT_TRUE(exec("ADJUSTED"));
    EXPECT_TRUE(exec("SYNC_EXTRUDER_STEPPER STEPPER=stepper_x"));
    EXPECT_TRUE(exec("TEST_RESONANCES AXIS=X"));
    EXPECT_TRUE(exec("SHAPER_CALIBRATE AXIS=X"));
    EXPECT_TRUE(exec("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.04"));
    EXPECT_TRUE(exec("SET_PRESSURE_ADVANCE SMOOTH_TIME=0.02"));
    EXPECT_TRUE(exec("SET_HEATER_FLOW_COMPENSATION HEATER=extruder COEFFICIENT=0.5"));
    EXPECT_TRUE(exec("SET_DECONVOLUTION_CONTROLLER STEPPER=stepper_x TYPE=pv"));
    EXPECT_TRUE(exec("SET_INPUT_SHAPER SHAPER_TYPE_X=mzv SHAPER_FREQ_X=55"));
    EXPECT_TRUE(exec("SET_EXTRUDER_STEP_DISTANCE DISTANCE=0.002"));
    EXPECT_TRUE(exec("ACTIVATE_EXTRUDER EXTRUDER=extruder"));
    EXPECT_TRUE(exec("SET_DIGITAL_PIN PIN=led VALUE=1"));
    EXPECT_TRUE(exec("SET_MULTI_PIN PIN=led VALUE=0.5"));
    EXPECT_TRUE(exec("SET_BUTTON_TEMPLATE NAME=btn TEXT='Hi'"));
    EXPECT_TRUE(exec("SET_SMART_EFFECTOR SENSITIVITY=25"));
    EXPECT_TRUE(exec("SET_KINEMATICS X=10"));
    EXPECT_TRUE(exec("CALIBRATE_PICOMM"));
    EXPECT_TRUE(exec("SET_EXTRUDER_ROTATION_DISTANCE DISTANCE=23"));
    EXPECT_TRUE(exec("SET_RETRACTION RETRACT_LENGTH=1.5 RETRACT_SPEED=35"));
    EXPECT_TRUE(exec("SET_CURRENT STEPPER=stepper_x CURRENT=0.7 HOLDCURRENT=0.3"));
    EXPECT_TRUE(exec("ENDSTOP_HOME"));
    EXPECT_TRUE(exec("ENDSTOP_PHASE STEPPER=stepper_z"));
    EXPECT_TRUE(exec("FILAMENT_LOAD"));
    EXPECT_TRUE(exec("FILAMENT_UNLOAD"));
    EXPECT_TRUE(exec("FILAMENT_PURGE LENGTH=5"));
    EXPECT_TRUE(exec("FORCE_MOVE STEPPER=stepper_x DISTANCE=1 VELOCITY=5 ACCEL=50"));
    EXPECT_TRUE(exec("STEPPER_BUZZ STEPPER=stepper_x"));
    EXPECT_TRUE(exec("MANUAL_STEPPER STEPPER=stepper_x SET=5"));
    EXPECT_TRUE(exec("MANUAL_STEPPER STEPPER=stepper_x MOVE=2"));
    EXPECT_TRUE(exec("SET_GCODE_POSITION X=1"));
    EXPECT_TRUE(exec("SAVE_GCODE_STATE NAME=s1"));
    EXPECT_TRUE(exec("RESTORE_GCODE_STATE NAME=s1"));
    EXPECT_TRUE(exec("SAVE_GCODE_STATE NAME=s2"));
    EXPECT_TRUE(exec("RESTORE_GCODE_STATE NAME=missing"));
    EXPECT_TRUE(exec("SET_GCODE_VARIABLE MACRO=M_TEST VARIABLE=v VALUE=1"));
    EXPECT_TRUE(exec("SAVE_VARIABLE VARIABLE=v2 VALUE=9"));
    EXPECT_TRUE(exec("SAVE_CONFIG"));
    EXPECT_TRUE(exec("SET_SKEW XY=140,100,140"));
    EXPECT_TRUE(exec("SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY"));
    EXPECT_TRUE(exec("SET_POSITION X=1 Y=2"));
    EXPECT_TRUE(exec("SET_HOME_POSITION X=0"));
    EXPECT_TRUE(exec("SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=1000"));
    EXPECT_TRUE(exec("BED_MESH_OFFSET X=0.2"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_DEFINE NAME=o1 CENTER=50,50"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_DEFINE RESET=1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_START NAME=o1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT NAME=o1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_END"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT RESET=1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_RESET"));
    EXPECT_TRUE(exec("UPDATE_DELAYED_GCODE ID=dg0 DELAY=0.2"));
    EXPECT_TRUE(exec("SET_DELAYED_GCODE ID=dg0 DELAY=0.2"));
    EXPECT_TRUE(exec("Z_TILT_ADJUST"));
    EXPECT_TRUE(exec("QUAD_GANTRY_LEVEL"));
    EXPECT_TRUE(exec("SCREWS_TILT_ADJUST"));
    EXPECT_TRUE(exec("BED_SCREWS_ADJUST"));
    EXPECT_TRUE(exec("DELTA_CALIBRATE"));
    EXPECT_TRUE(exec("DELTA_ANALYZE"));
    EXPECT_TRUE(exec("PROBE_ACCURACY SAMPLES=2"));
    EXPECT_TRUE(exec("RESPOND MSG='x'"));
    EXPECT_TRUE(exec("ECHO yo"));
    EXPECT_TRUE(exec("SET_PRINT_STATS_INFO TOTAL_LAYER=5 CURRENT_LAYER=2"));
    EXPECT_TRUE(exec("SET_DISPLAY_GROUP GROUP=g"));
    EXPECT_TRUE(exec("QUERY_ADC NAME=x"));
    EXPECT_TRUE(exec("ACCELEROMETER_QUERY"));
}

// Multi-extruder branch of setHotendTemp (T-parameter selects a
// configured extruder heater index).
TEST_F(KlippyCallbacksTest, ExtruderIndexPaths) {
    EXPECT_TRUE(exec("M104 T0 S180"));
    EXPECT_TRUE(exec("M104 T1 S180"));
    EXPECT_TRUE(exec("M109 T0 S170"));
    EXPECT_TRUE(exec("ACTIVATE_EXTRUDER EXTRUDER=extruder"));
    EXPECT_TRUE(exec("M104 S160"));
}

// Interactive probe session: PROBE_CALIBRATE opens a session that
// ACCEPT/ABORT/TESTZ/ADJUSTED drive.
TEST_F(KlippyCallbacksTest, InteractiveProbeSession) {
    EXPECT_TRUE(exec("PROBE_CALIBRATE"));
    EXPECT_TRUE(exec("TESTZ Z=+0.1"));
    EXPECT_TRUE(exec("TESTZ Z=-0.2"));
    EXPECT_TRUE(exec("ACCEPT"));
    EXPECT_TRUE(exec("PROBE_CALIBRATE"));
    EXPECT_TRUE(exec("ABORT"));
    EXPECT_TRUE(exec("MANUAL_PROBE"));
    EXPECT_TRUE(exec("TESTZ Z=+0.5"));
    EXPECT_TRUE(exec("ABORT"));
}

// Print start/pause/resume/cancel handler paths.
TEST_F(KlippyCallbacksTest, PrintControlHandlers) {
    EXPECT_TRUE(exec("SDCARD_PRINT_FILE FILENAME=print.gcode"));
    EXPECT_TRUE(exec("PAUSE"));
    EXPECT_TRUE(exec("RESUME"));
    EXPECT_TRUE(exec("CANCEL_PRINT"));
    EXPECT_TRUE(exec("SDCARD_RESET_FILE"));
    EXPECT_TRUE(exec("M24"));
    EXPECT_TRUE(exec("M25"));
}
