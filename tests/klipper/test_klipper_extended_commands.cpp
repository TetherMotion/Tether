/**
 * @file test_klipper_extended_commands.cpp
 * @brief Coverage for KlippyInstanceExtendedCommands.ipp — the
 *        handleExtendedCommand() dispatch surface (bed mesh, probe
 *        sessions, TMC, pins/LEDs, filament, exclude-object, delayed
 *        gcode, kinematics, skew, input shaper, etc.).
 *
 * Every command is driven through the public executeGcode() path with
 * a rich printer.cfg so per-object guards resolve against real config.
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

namespace {

using namespace tether::klipper;

static std::string uniqueSock() {
    return "/tmp/tether_test_extcmd_" + std::to_string(getpid()) + ".sock";
}
static std::string uniqueCfg() {
    return "/tmp/tether_test_extcmd_" + std::to_string(getpid()) + ".cfg";
}
static std::string uniqueSettings() {
    return "/tmp/tether_test_extcmd_" + std::to_string(getpid()) + ".settings.cfg";
}

class ExtendedCommandsTest : public ::testing::Test {
protected:
    void SetUp() override {
        cfgPath_ = uniqueCfg();
        std::ofstream f(cfgPath_);
        f << R"(
[printer]
kinematics = cartesian
max_velocity = 300
max_accel = 2000
[stepper_x]
position_max = 200
position_min = 0
[stepper_y]
position_max = 200
position_min = 0
[stepper_z]
position_max = 200
position_min = 0
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
path = /tmp/tether_test_sdcard
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
filename = /tmp/tether_test_extcmd_vars.cfg
[force_move]
enable_force_move = True
[endstop_phase stepper_z]
phase_count = 64
[safe_z_home]
home_xy_position = 100,100
)";
        f.close();

        klippy::KlippyInstanceConfig cfg;
        cfg.udsConfig.socketPath = sockPath_;
        cfg.settingsPath = uniqueSettings();
        cfg.sdcardDir = "/tmp/tether_test_sdcard";
        instance_ = std::make_unique<klippy::KlippyInstance>(cfg);
        ASSERT_TRUE(instance_->loadConfig(cfgPath_));

        // Commands that need a probe object get a fake one.
        probe_ = std::make_shared<objects::Probe>(0, []() { return false; });
        probe_->setZOffset(1.5);
        instance_->setProbe(probe_);
    }

    void TearDown() override {
        instance_.reset();
        ::unlink(sockPath_.c_str());
        std::filesystem::remove(cfgPath_);
        std::filesystem::remove(uniqueSettings());
        std::filesystem::remove("/tmp/tether_test_extcmd_vars.cfg");
    }

    bool exec(const std::string& g) { return instance_->executeGcode(g); }

    std::string sockPath_ = uniqueSock();
    std::string cfgPath_;
    std::unique_ptr<klippy::KlippyInstance> instance_;
    std::shared_ptr<objects::Probe> probe_;
};

// ---------------------------------------------------------------------------
// Bed mesh commands
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, BedMeshCommandFamily) {
    EXPECT_TRUE(exec("BED_MESH_CALIBRATE"));
    EXPECT_TRUE(exec("BED_MESH_OUTPUT"));
    EXPECT_TRUE(exec("BED_MESH_MAP"));
    EXPECT_TRUE(exec("BED_MESH_OFFSET X=0.1 Y=-0.2"));
    EXPECT_TRUE(exec("BED_MESH_PROFILE SAVE=mymesh"));
    EXPECT_TRUE(exec("BED_MESH_PROFILE LOAD=mymesh"));
    EXPECT_TRUE(exec("BED_MESH_PROFILE REMOVE=mymesh"));
    EXPECT_TRUE(exec("BED_MESH_CLEAR"));
}

// ---------------------------------------------------------------------------
// Probe / calibration commands
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, ProbeCommands) {
    EXPECT_TRUE(exec("PROBE_ACCURACY SAMPLES=2"));
    EXPECT_TRUE(exec("Z_OFFSET_APPLY_PROBE"));
    EXPECT_TRUE(exec("Z_OFFSET_APPLY_ENDSTOP"));
    EXPECT_TRUE(exec("QUERY_ENDSTOPS"));
}

TEST_F(ExtendedCommandsTest, InteractiveSessionCommands) {
    EXPECT_TRUE(exec("MANUAL_PROBE"));
    EXPECT_TRUE(exec("ACCEPT"));
    EXPECT_TRUE(exec("PROBE_CALIBRATE"));
    EXPECT_TRUE(exec("ABORT"));
    EXPECT_TRUE(exec("MANUAL_STEPPER STEPPER=stepper_x SET=10"));
    EXPECT_TRUE(exec("MANUAL_STEPPER STEPPER=stepper_x MOVE=5"));
    EXPECT_TRUE(exec("MANUAL_STEPPER STEPPER=stepper_x SYNC=1"));
}

TEST_F(ExtendedCommandsTest, LevelingCommands) {
    EXPECT_TRUE(exec("Z_TILT_ADJUST"));
    EXPECT_TRUE(exec("BED_SCREWS_ADJUST"));
    EXPECT_TRUE(exec("SCREWS_TILT_ADJUST"));
    EXPECT_TRUE(exec("QUAD_GANTRY_LEVEL"));
    EXPECT_TRUE(exec("DELTA_ANALYZE"));
    EXPECT_TRUE(exec("DELTA_CALIBRATE"));
    EXPECT_TRUE(exec("ENDSTOP_HOME"));
    EXPECT_TRUE(exec("ENDSTOP_PHASE STEPPER=stepper_z"));
}

// ---------------------------------------------------------------------------
// Pin / LED / servo / display commands
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, PinAndLedCommands) {
    EXPECT_TRUE(exec("SET_PIN PIN=led VALUE=0.5"));
    EXPECT_TRUE(exec("SET_PWM_PIN PIN=led VALUE=0.3"));
    EXPECT_TRUE(exec("SET_DIGITAL_PIN PIN=led VALUE=1"));
    EXPECT_TRUE(exec("SET_SERVO SERVO=s0 ANGLE=45"));
    EXPECT_TRUE(exec("SET_SERVO SERVO=s0 WIDTH=2.0"));
    EXPECT_TRUE(exec("SET_MULTI_PIN PIN=led VALUE=0.7"));
    EXPECT_TRUE(exec("SET_NEOPIXEL LED=np RED=1 GREEN=0 BLUE=0 INDEX=1"));
    EXPECT_TRUE(exec("SET_DOTSTAR LED=ds RED=0.5"));
    EXPECT_TRUE(exec("SET_LED LED=led RED=0.2 GREEN=0.3 BLUE=0.4 WHITE=0.1"));
    EXPECT_TRUE(exec("SET_BUTTON_TEMPLATE NAME=btn TEXT='OK'"));
    EXPECT_TRUE(exec("SET_DISPLAY_GROUP GROUP=g1"));
}

// ---------------------------------------------------------------------------
// Motion / state commands
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, MotionStateCommands) {
    EXPECT_TRUE(exec("FORCE_MOVE STEPPER=stepper_x DISTANCE=1 VELOCITY=10 ACCEL=100"));
    EXPECT_TRUE(exec("SET_POSITION X=10 Y=20"));
    EXPECT_TRUE(exec("SET_GCODE_POSITION X=5"));
    EXPECT_TRUE(exec("SET_GCODE_OFFSET X=1 Y=2 Z=0.1"));
    EXPECT_TRUE(exec("SET_HOME_POSITION X=0 Y=0 Z=0"));
    EXPECT_TRUE(exec("SET_KINEMATICS X=10"));
    EXPECT_TRUE(exec("SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=0"));
    EXPECT_TRUE(exec("SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=1"));
    EXPECT_TRUE(exec("STEPPER_BUZZ STEPPER=stepper_x"));
    EXPECT_TRUE(exec("SET_VELOCITY_LIMIT VELOCITY=250 ACCEL=1500 SQUARE_CORNER_VELOCITY=6"));
    EXPECT_TRUE(exec("SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY"));
    EXPECT_TRUE(exec("SYNC_EXTRUDER_STEPPER STEPPER=stepper_x"));
    EXPECT_TRUE(exec("ACTIVATE_EXTRUDER EXTRUDER=extruder"));
    EXPECT_TRUE(exec("SET_EXTRUDER_STEP_DISTANCE DISTANCE=0.001"));
    EXPECT_TRUE(exec("SET_EXTRUDER_ROTATION_DISTANCE DISTANCE=23.5"));
    EXPECT_TRUE(exec("SET_SKEW XY=140.4,100.5,140 XZ=141.2,100.3,150"));
    EXPECT_TRUE(exec("SET_IDLE_TIMEOUT TIMEOUT=300"));
    EXPECT_TRUE(exec("SET_PRESSURE_ADVANCE ADVANCE=0.05 SMOOTH_TIME=0.02"));
    EXPECT_TRUE(exec("SET_RETRACTION RETRACT_LENGTH=0.8 RETRACT_SPEED=25"));
    EXPECT_TRUE(exec("SET_HEATER_FLOW_COMPENSATION ENABLE=1"));
    EXPECT_TRUE(exec("SET_DECONVOLUTION_CONTROLLER ENABLE=0"));
    EXPECT_TRUE(exec("SET_INPUT_SHAPER SHAPER_TYPE_X=mzv SHAPER_FREQ_X=45"));
}

// ---------------------------------------------------------------------------
// TMC commands
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, TmcCommands) {
    EXPECT_TRUE(exec("INIT_TMC STEPPER=stepper_x"));
    EXPECT_TRUE(exec("DUMP_TMC STEPPER=stepper_x"));
    EXPECT_TRUE(exec("SET_TMC_FIELD STEPPER=stepper_x FIELD=SGTHRS VALUE=64"));
    EXPECT_TRUE(exec("SET_CURRENT STEPPER=stepper_x CURRENT=0.9"));
}

// ---------------------------------------------------------------------------
// State save / variables / macros
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, StateAndVariableCommands) {
    EXPECT_TRUE(exec("SAVE_GCODE_STATE NAME=mystate"));
    EXPECT_TRUE(exec("RESTORE_GCODE_STATE NAME=mystate"));
    EXPECT_TRUE(exec("SET_GCODE_VARIABLE MACRO=M_TEST VARIABLE=count VALUE=3"));
    EXPECT_TRUE(exec("SAVE_VARIABLE VARIABLE=myvar VALUE=42"));
    EXPECT_TRUE(exec("SAVE_CONFIG"));
}

// ---------------------------------------------------------------------------
// Filament / objects / misc
// ---------------------------------------------------------------------------

TEST_F(ExtendedCommandsTest, FilamentCommands) {
    EXPECT_TRUE(exec("FILAMENT_LOAD"));
    EXPECT_TRUE(exec("FILAMENT_UNLOAD"));
    EXPECT_TRUE(exec("FILAMENT_PURGE LENGTH=10"));
    EXPECT_TRUE(exec("SET_SMART_EFFECTOR SENSITIVITY=50"));
    EXPECT_TRUE(exec("CALIBRATE_PICOMM"));
}

TEST_F(ExtendedCommandsTest, ExcludeObjectCommands) {
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_DEFINE NAME=obj1 CENTER=100,100 POLYGON=10,10,20,20"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_START NAME=obj1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT NAME=obj1"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_END"));
    EXPECT_TRUE(exec("EXCLUDE_OBJECT_RESET"));
}

TEST_F(ExtendedCommandsTest, DelayedGcodeCommands) {
    EXPECT_TRUE(exec("SET_DELAYED_GCODE ID=dg0 DELAY=0.05"));
    EXPECT_TRUE(exec("UPDATE_DELAYED_GCODE ID=dg0 DELAY=0.05"));
}

TEST_F(ExtendedCommandsTest, HeaterAndFanCommands) {
    EXPECT_TRUE(exec("SET_FAN_SPEED FAN=fan SPEED=0.5"));
    EXPECT_TRUE(exec("SET_HEATER_TEMPERATURE HEATER=extruder TARGET=180"));
    EXPECT_TRUE(exec("SET_TEMPERATURE_FAN FAN=tfan TARGET=40"));
    EXPECT_TRUE(exec("M106 S128"));
    EXPECT_TRUE(exec("M140 S60"));
}

TEST_F(ExtendedCommandsTest, MiscCommands) {
    EXPECT_TRUE(exec("ECHO hello world"));
    EXPECT_TRUE(exec("RESPOND MSG='hi'"));
    EXPECT_TRUE(exec("QUERY_ADC NAME=x"));
    EXPECT_TRUE(exec("ACCELEROMETER_QUERY"));
    EXPECT_TRUE(exec("SET_PRINT_STATS_INFO TOTAL_LAYER=100 CURRENT_LAYER=3"));
    EXPECT_TRUE(exec("SET_PROBE PROBE=probe"));
}

}  // namespace
