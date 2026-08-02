/**
 * @file test_klipper_gcode_parser.cpp
 * @brief Direct unit tests for GCodeParser / parseGcodeLine / GcodeLine.
 */

#include <gtest/gtest.h>
#include "tether/klipper/klippy/GCodeParser.hpp"

using namespace tether::klipper::klippy;

TEST(GcodeLineTest, ParseSimpleG1) {
    auto g = parseGcodeLine("G1 X10 Y20 Z30 F1500");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_TRUE(g->has('X'));
    EXPECT_TRUE(g->has('Y'));
    EXPECT_TRUE(g->has('Z'));
    EXPECT_TRUE(g->has('F'));
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->get('Y'), 20.0);
    EXPECT_EQ(g->get('Z'), 30.0);
    EXPECT_EQ(g->get('F'), 1500.0);
}

TEST(GcodeLineTest, ParseWithComment) {
    auto g = parseGcodeLine("G1 X10 ; move to X10");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G1");
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->comment, " move to X10");
}

TEST(GcodeLineTest, ParseNegativeValues) {
    auto g = parseGcodeLine("G1 X-10.5 Y-20");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->get('X'), -10.5);
    EXPECT_EQ(g->get('Y'), -20.0);
}

TEST(GcodeLineTest, ParseMCode) {
    auto g = parseGcodeLine("M104 S200");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "M104");
    EXPECT_EQ(g->get('S'), 200.0);
}

TEST(GcodeLineTest, ParseExtendedCommand) {
    auto g = parseGcodeLine("BED_MESH_CALIBRATE");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "BED_MESH_CALIBRATE");
    EXPECT_TRUE(g->isExtendedCommand());
}

TEST(GcodeLineTest, ParseExtendedCommandWithNamedParams) {
    auto g = parseGcodeLine("SET_SERVO SERVO=my_servo ANGLE=45");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "SET_SERVO");
    EXPECT_TRUE(g->isExtendedCommand());
    EXPECT_TRUE(g->hasNamed("SERVO"));
    EXPECT_EQ(g->getNamed("SERVO"), "my_servo");
    EXPECT_TRUE(g->hasNamed("ANGLE"));
    EXPECT_EQ(g->getNamedDouble("ANGLE"), 45.0);
    EXPECT_EQ(g->getNamedInt("ANGLE"), 45);
}

TEST(GcodeLineTest, ParseNamedParamCaseInsensitive) {
    auto g = parseGcodeLine("SET_SERVO servo=my_servo angle=45");
    ASSERT_TRUE(g.has_value());
    EXPECT_TRUE(g->hasNamed("SERVO"));
    EXPECT_TRUE(g->hasNamed("servo"));
    EXPECT_EQ(g->getNamed("SERVO"), "my_servo");
}

TEST(GcodeLineTest, GetWithDefault) {
    auto g = parseGcodeLine("G1 X10");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->get('X'), 10.0);
    EXPECT_EQ(g->get('Y', 99.0), 99.0);
    EXPECT_FALSE(g->has('Y'));
}

TEST(GcodeLineTest, GetNamedWithDefault) {
    auto g = parseGcodeLine("G1 X10");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->getNamed("MISSING", "default"), "default");
    EXPECT_EQ(g->getNamedDouble("MISSING", 42.0), 42.0);
    EXPECT_EQ(g->getNamedInt("MISSING", 7), 7);
}

TEST(GcodeLineTest, ParseEmptyLine) {
    auto g = parseGcodeLine("");
    EXPECT_FALSE(g.has_value());
}

TEST(GcodeLineTest, ParseCommentOnly) {
    auto g = parseGcodeLine("; just a comment");
    EXPECT_FALSE(g.has_value());
}

TEST(GcodeLineTest, ParseG28) {
    auto g = parseGcodeLine("G28 X Y");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "G28");
}

TEST(GcodeLineTest, ParseGcodeWithEParam) {
    auto g = parseGcodeLine("G1 X10 E5 F100");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->get('E'), 5.0);
}

TEST(GcodeLineTest, ParseTemperatureCommand) {
    auto g = parseGcodeLine("M104 S210 T0");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->code, "M104");
    EXPECT_EQ(g->get('S'), 210.0);
    EXPECT_EQ(g->get('T'), 0.0);
}

TEST(GcodeLineTest, IsExtendedCommandFalseForRegularGcode) {
    auto g = parseGcodeLine("G1 X10");
    ASSERT_TRUE(g.has_value());
    EXPECT_FALSE(g->isExtendedCommand());
}

TEST(GcodeLineTest, IsExtendedCommandFalseForMCode) {
    auto g = parseGcodeLine("M104 S200");
    ASSERT_TRUE(g.has_value());
    EXPECT_FALSE(g->isExtendedCommand());
}

TEST(GcodeLineTest, ParseMultipleNamedParams) {
    auto g = parseGcodeLine("SET_PIN VALUE=1 PIN=led");
    ASSERT_TRUE(g.has_value());
    EXPECT_EQ(g->getNamed("VALUE"), "1");
    EXPECT_EQ(g->getNamed("PIN"), "led");
}
