#include <gtest/gtest.h>

#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeTypes.hpp"

using namespace GCode;

TEST(GCodeParser, Parse_G0_linear_move)
{
    VariableSystem vars;
    Parser parser(vars);

    Block block;
    Error err = parser.parseLine("G0 X10 Y20", block);
    EXPECT_FALSE(err);

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_TRUE(block.hasWord(WordLetter::Y));
    EXPECT_NEAR(block.getWord(WordLetter::X), 10.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Y), 20.0, 1e-9);

    // G0 stored as internal code 0
    bool foundG0 = false;
    for (uint8_t i = 0; i < block.gCodeCount; ++i) if (block.gCodes[i] == 0) foundG0 = true;
    EXPECT_TRUE(foundG0);
}

TEST(GCodeParser, Parse_G1_with_feed)
{
    VariableSystem vars;
    Parser parser(vars);

    Block block;
    Error err = parser.parseLine("G1 X100.5 Y-50.25 F1500", block);
    EXPECT_FALSE(err);

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_TRUE(block.hasWord(WordLetter::Y));
    EXPECT_TRUE(block.hasWord(WordLetter::F));
    EXPECT_NEAR(block.getWord(WordLetter::X), 100.5, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Y), -50.25, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::F), 1500.0, 1e-9);

    // G1 stored as internal code 10
    bool foundG1 = false;
    for (uint8_t i = 0; i < block.gCodeCount; ++i) if (block.gCodes[i] == 10) foundG1 = true;
    EXPECT_TRUE(foundG1);
}

TEST(GCodeParser, Parse_G2_arc_IJ_and_R)
{
    VariableSystem vars;
    Parser parser(vars);

    Block block;
    Error err = parser.parseLine("G2 X0 Y0 I10 J0 R5", block);
    EXPECT_FALSE(err);

    EXPECT_TRUE(block.hasWord(WordLetter::I));
    EXPECT_TRUE(block.hasWord(WordLetter::J));
    EXPECT_TRUE(block.hasWord(WordLetter::R));
    EXPECT_NEAR(block.getWord(WordLetter::I), 10.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::J), 0.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::R), 5.0, 1e-9);

    // G2 stored as internal code 20
    bool foundG2 = false;
    for (uint8_t i = 0; i < block.gCodeCount; ++i) if (block.gCodes[i] == 20) foundG2 = true;
    EXPECT_TRUE(foundG2);
}
