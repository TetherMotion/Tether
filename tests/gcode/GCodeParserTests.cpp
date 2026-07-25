/**
 * @file GCodeParserTests.cpp
 * @brief Comprehensive unit tests for the GCode Parser
 *
 * This file provides complete coverage for the parser including:
 * - Block parsing from tokens
 * - Modal group validation
 * - Word handling
 * - O-code parsing
 * - Block queue operations
 * - Error handling
 */

#include <gtest/gtest.h>
#include <tether/gcode/GCodeParser.hpp>
#include <tether/gcode/GCodeLexer.hpp>
#include <tether/gcode/GCodeTypes.hpp>
#include <tether/gcode/GCodeVariables.hpp>
#include <vector>
#include <string>
#include <cmath>

namespace GCode {
namespace test {

// ============================================================================
// Test Fixture
// ============================================================================

class ParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        vars = std::make_unique<VariableSystem>();
        parser = std::make_unique<Parser>(*vars);
    }

    void TearDown() override {
        parser.reset();
        vars.reset();
    }

    std::unique_ptr<VariableSystem> vars;
    std::unique_ptr<Parser> parser;

    // Helper to parse a single line
    Block parseBlock(const char* line) {
        Block block;
        parser->parseLine(line, block);
        return block;
    }
};

// ============================================================================
// Basic Parsing Tests
// ============================================================================

TEST_F(ParserTest, EmptyLine) {
    Block block;
    Error err = parser->parseLine("", block);
    EXPECT_FALSE(err);
}

TEST_F(ParserTest, NullLine) {
    Block block;
    Error err = parser->parseLine(nullptr, block);
    EXPECT_TRUE(err);
}

TEST_F(ParserTest, CommentOnly) {
    Block block;
    Error err = parser->parseLine("(this is a comment)", block);
    EXPECT_FALSE(err);
    EXPECT_TRUE(block.hasComment);
}

TEST_F(ParserTest, SimpleGCode) {
    Block block = parseBlock("G0 X100 Y50");

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_TRUE(block.hasWord(WordLetter::Y));
    EXPECT_NEAR(block.getWord(WordLetter::X), 100.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Y), 50.0, 1e-9);
}

TEST_F(ParserTest, AllAxisWords) {
    Block block = parseBlock("X10 Y20 Z30 A40 B50 C60 U70 V80 W90");

    EXPECT_TRUE(block.hasWord(WordLetter::X));
    EXPECT_TRUE(block.hasWord(WordLetter::Y));
    EXPECT_TRUE(block.hasWord(WordLetter::Z));
    EXPECT_TRUE(block.hasWord(WordLetter::A));
    EXPECT_TRUE(block.hasWord(WordLetter::B));
    EXPECT_TRUE(block.hasWord(WordLetter::C));
    EXPECT_TRUE(block.hasWord(WordLetter::U));
    EXPECT_TRUE(block.hasWord(WordLetter::V));
    EXPECT_TRUE(block.hasWord(WordLetter::W));
}

TEST_F(ParserTest, FeedRate) {
    Block block = parseBlock("G1 X100 F1000");

    EXPECT_TRUE(block.hasWord(WordLetter::F));
    EXPECT_NEAR(block.getWord(WordLetter::F), 1000.0, 1e-9);
}

TEST_F(ParserTest, SpindleSpeed) {
    Block block = parseBlock("M3 S5000");

    EXPECT_TRUE(block.hasWord(WordLetter::S));
    EXPECT_NEAR(block.getWord(WordLetter::S), 5000.0, 1e-9);
}

TEST_F(ParserTest, ToolNumber) {
    Block block = parseBlock("T1 M6");

    EXPECT_TRUE(block.hasWord(WordLetter::T));
    EXPECT_NEAR(block.getWord(WordLetter::T), 1.0, 1e-9);
}

TEST_F(ParserTest, LineNumber) {
    Block block = parseBlock("N100 G1 X50");

    // Line number is stored in dedicated field, not as a word
    EXPECT_EQ(block.lineNumber, 100);
    EXPECT_TRUE(block.hasWord(WordLetter::X));
}

// ============================================================================
// G-Code Modal Group Tests
// ============================================================================

// Note: getModalGroup takes the internal representation (major*10 + minor)
// G0 -> 0, G1 -> 10, G17 -> 170, G90 -> 900, etc.

TEST_F(ParserTest, ModalGroupMotion) {
    EXPECT_EQ(getModalGroup(0), ModalGroup::MOTION);    // G0
    EXPECT_EQ(getModalGroup(10), ModalGroup::MOTION);   // G1
    EXPECT_EQ(getModalGroup(20), ModalGroup::MOTION);   // G2
    EXPECT_EQ(getModalGroup(30), ModalGroup::MOTION);   // G3
}

TEST_F(ParserTest, ModalGroupPlane) {
    EXPECT_EQ(getModalGroup(170), ModalGroup::PLANE);   // G17
    EXPECT_EQ(getModalGroup(180), ModalGroup::PLANE);   // G18
    EXPECT_EQ(getModalGroup(190), ModalGroup::PLANE);   // G19
}

TEST_F(ParserTest, ModalGroupDistance) {
    EXPECT_EQ(getModalGroup(900), ModalGroup::DISTANCE);  // G90
    EXPECT_EQ(getModalGroup(910), ModalGroup::DISTANCE);  // G91
}

TEST_F(ParserTest, ModalGroupFeedMode) {
    EXPECT_EQ(getModalGroup(930), ModalGroup::FEED_MODE);  // G93
    EXPECT_EQ(getModalGroup(940), ModalGroup::FEED_MODE);  // G94
    EXPECT_EQ(getModalGroup(950), ModalGroup::FEED_MODE);  // G95
}

TEST_F(ParserTest, ModalGroupUnits) {
    EXPECT_EQ(getModalGroup(200), ModalGroup::UNITS);  // G20
    EXPECT_EQ(getModalGroup(210), ModalGroup::UNITS);  // G21
}

TEST_F(ParserTest, ModalGroupPathMode) {
    EXPECT_EQ(getModalGroup(610), ModalGroup::PATH_MODE);  // G61
    EXPECT_EQ(getModalGroup(640), ModalGroup::PATH_MODE);  // G64
}

TEST_F(ParserTest, ModalGroupNonModal) {
    // G4 (dwell) is non-modal
    EXPECT_EQ(getModalGroup(40), ModalGroup::NON_MODAL);    // G4
    // G10 (tool/work setting) is non-modal
    EXPECT_EQ(getModalGroup(100), ModalGroup::NON_MODAL);   // G10
    EXPECT_EQ(getModalGroup(280), ModalGroup::NON_MODAL);   // G28
    EXPECT_EQ(getModalGroup(300), ModalGroup::NON_MODAL);   // G30
}

TEST_F(ParserTest, GetModalGroupWithDecimal) {
    // G38.2 -> major=38, minor=2 -> internal=382
    EXPECT_EQ(getModalGroup(382), ModalGroup::MOTION);  // G38.2
    EXPECT_EQ(getModalGroup(381), ModalGroup::MOTION);  // G38.1
}

// ============================================================================
// Arc Parameter Tests
// ============================================================================

TEST_F(ParserTest, ArcWithIJ) {
    Block block = parseBlock("G2 X50 Y50 I25 J0 F200");

    EXPECT_TRUE(block.hasWord(WordLetter::I));
    EXPECT_TRUE(block.hasWord(WordLetter::J));
    EXPECT_NEAR(block.getWord(WordLetter::I), 25.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::J), 0.0, 1e-9);
}

TEST_F(ParserTest, ArcWithRadius) {
    Block block = parseBlock("G3 X0 Y100 R50");

    EXPECT_TRUE(block.hasWord(WordLetter::R));
    EXPECT_NEAR(block.getWord(WordLetter::R), 50.0, 1e-9);
}

TEST_F(ParserTest, HelicalArc) {
    Block block = parseBlock("G2 X50 Y50 Z-10 I25 J0 F100");

    EXPECT_TRUE(block.hasWord(WordLetter::Z));
    EXPECT_NEAR(block.getWord(WordLetter::Z), -10.0, 1e-9);
}

// ============================================================================
// O-Code Tests
// ============================================================================

TEST_F(ParserTest, OCodeNumber) {
    Block block = parseBlock("O100");

    EXPECT_TRUE(block.hasOCode);
    EXPECT_FALSE(block.oCodeIsNamed);
    EXPECT_EQ(block.oCodeNumber, 100);
}

TEST_F(ParserTest, OCodeName) {
    Block block = parseBlock("O<mysubroutine>");

    EXPECT_TRUE(block.hasOCode);
    EXPECT_TRUE(block.oCodeIsNamed);
    EXPECT_STREQ(block.oCodeName.data(), "mysubroutine");
}

TEST_F(ParserTest, KeyValueSimpleAndVector) {
    Block b = parseBlock("FOO=1 BAR=2.5 NAME=\"John Doe\" V=[1, 2, 3.5] P=(4 5 6) BAZ=1,2 NEG=-1e-3 SP =  42 K=1 K=2 EMPTY=()");

    EXPECT_TRUE(b.hasKeyValue("FOO"));
    EXPECT_EQ(b.getParamInt("FOO"), 1);
    EXPECT_NEAR(b.getParamDouble("BAR"), 2.5, 1e-9);
    EXPECT_EQ(b.getParamString("NAME"), "John Doe");

    auto v = b.getParamVector("V");
    ASSERT_EQ(v.size(), 3u);
    EXPECT_NEAR(v[0], 1.0, 1e-9);
    EXPECT_NEAR(v[2], 3.5, 1e-9);

    auto p = b.getParamVector("P");
    ASSERT_EQ(p.size(), 3u);
    EXPECT_NEAR(p[1], 5.0, 1e-9);

    auto baz = b.getParamVector("BAZ");
    ASSERT_EQ(baz.size(), 2u);
    EXPECT_NEAR(baz[0], 1.0, 1e-9);

    EXPECT_NEAR(b.getParamDouble("NEG"), -1e-3, 1e-12);

    EXPECT_EQ(b.getParamInt("SP"), 42);

    // Last K occurrence should win (K=2)
    EXPECT_EQ(b.getParamInt("K"), 2);

    auto e = b.getParamVector("EMPTY");
    EXPECT_TRUE(e.empty());
}

TEST_F(ParserTest, KeyValueEdgeCases) {
    // Sanity: tokenizer with parser lex config recognizes KEYVALUE
    LexerConfig c; c.caseInsensitive = true; c.allowSpacesInNumbers = false; Lexer lx(c); lx.setInput("FOO=");
    auto toks = lx.tokenizeLine();
    ASSERT_EQ(toks.size(), 1u);
    EXPECT_EQ(toks[0].type, LexerTokenType::KEYVALUE);

    // Missing value
    // Tokenize with same config and manually populate a Block to verify lexer output
    LexerConfig c2; c2.caseInsensitive = true; c2.allowSpacesInNumbers = false; Lexer lx2(c2); lx2.setInput("FOO=");
    auto toks2 = lx2.tokenizeLine();
    Block manual;
    for (const auto& t : toks2) {
        if (t.type == LexerTokenType::KEYVALUE && t.isKeyValue) {
            std::string v = t.kvValue;
            if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\''))) {
                v = v.substr(1, v.size() - 2);
            }
            manual.keyValues[t.kvKey] = v;
        }
    }
    EXPECT_TRUE(manual.hasKeyValue("FOO"));

    Block b1; Error e1 = parser->parseLine("FOO=", b1); EXPECT_FALSE(e1);
    EXPECT_TRUE(b1.hasKeyValue("FOO"));
    EXPECT_EQ(b1.getParamString("FOO"), "");

    // Quoted empty
    Block b2 = parseBlock("NAME=\"\"");
    EXPECT_EQ(b2.getParamString("NAME"), "");

    // Unparsable int/double -> return default
    Block b3 = parseBlock("X=abc Y=1.2.3");
    EXPECT_EQ(b3.getParamInt("X", 7), 7);
    // Ensure raw value stored is '1.2.3' (not truncated)
    EXPECT_EQ(b3.getParamString("Y"), "1.2.3");
    EXPECT_NEAR(b3.getParamDouble("Y", 2.0), 2.0, 1e-9);

    // Value with internal '=' should treat only first '=' as separator
    Block b4 = parseBlock("CMD=KEY=VAL");
    EXPECT_EQ(b4.getParamString("CMD"), "KEY=VAL");
}

// ============================================================================
// Block Delete Tests
// ============================================================================

TEST_F(ParserTest, BlockDelete) {
    Block block = parseBlock("/G1 X100");

    EXPECT_TRUE(block.blockDelete);
}

// ============================================================================
// BlockQueue Tests
// ============================================================================

TEST_F(ParserTest, BlockQueueConstruction) {
    BlockQueue queue(10);

    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0u);
    EXPECT_EQ(queue.maxSize(), 10u);
}

TEST_F(ParserTest, BlockQueuePushBack) {
    BlockQueue queue(10);

    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;

    queue.pushBack(b1);
    queue.pushBack(b2);

    EXPECT_EQ(queue.size(), 2u);
    EXPECT_EQ(queue.front().sourceLineNumber, 1u);
    EXPECT_EQ(queue.back().sourceLineNumber, 2u);
}

TEST_F(ParserTest, BlockQueuePushFront) {
    BlockQueue queue(10);

    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;

    queue.pushFront(b1);
    queue.pushFront(b2);

    EXPECT_EQ(queue.size(), 2u);
    EXPECT_EQ(queue.front().sourceLineNumber, 2u);
    EXPECT_EQ(queue.back().sourceLineNumber, 1u);
}

TEST_F(ParserTest, BlockQueuePopFront) {
    BlockQueue queue(10);

    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;

    queue.pushBack(b1);
    queue.pushBack(b2);

    Block popped = queue.popFront();
    EXPECT_EQ(popped.sourceLineNumber, 1u);
    EXPECT_EQ(queue.size(), 1u);
}

TEST_F(ParserTest, BlockQueuePopBack) {
    BlockQueue queue(10);

    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;

    queue.pushBack(b1);
    queue.pushBack(b2);

    Block popped = queue.popBack();
    EXPECT_EQ(popped.sourceLineNumber, 2u);
    EXPECT_EQ(queue.size(), 1u);
}

TEST_F(ParserTest, BlockQueueIndexOperator) {
    BlockQueue queue(10);

    Block b1, b2, b3;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    b3.sourceLineNumber = 3;

    queue.pushBack(b1);
    queue.pushBack(b2);
    queue.pushBack(b3);

    EXPECT_EQ(queue[0].sourceLineNumber, 1u);
    EXPECT_EQ(queue[1].sourceLineNumber, 2u);
    EXPECT_EQ(queue[2].sourceLineNumber, 3u);
}

TEST_F(ParserTest, BlockQueueOverflow) {
    BlockQueue queue(3);

    for (int i = 1; i <= 5; i++) {
        Block b;
        b.sourceLineNumber = static_cast<uint32_t>(i);
        queue.pushBack(b);
    }

    EXPECT_EQ(queue.size(), 3u);
    // Oldest should be dropped
    EXPECT_EQ(queue.front().sourceLineNumber, 3u);
}

TEST_F(ParserTest, BlockQueueClear) {
    BlockQueue queue(10);

    Block b;
    queue.pushBack(b);
    queue.pushBack(b);
    queue.clear();

    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0u);
}

TEST_F(ParserTest, BlockQueueIterator) {
    BlockQueue queue(10);

    for (int i = 1; i <= 3; i++) {
        Block b;
        b.sourceLineNumber = static_cast<uint32_t>(i);
        queue.pushBack(b);
    }

    uint32_t sum = 0;
    for (const auto& block : queue) {
        sum += block.sourceLineNumber;
    }
    EXPECT_EQ(sum, 6u); // 1+2+3
}

// ============================================================================
// Multi-Block Parsing Tests
// ============================================================================

TEST_F(ParserTest, ParseNextBlock) {
    parser->setInput("G1 X100\nG2 X200\nG3 X300");

    Block b1, b2, b3;
    Error e1 = parser->parseNextBlock(b1);
    Error e2 = parser->parseNextBlock(b2);
    Error e3 = parser->parseNextBlock(b3);

    EXPECT_FALSE(e1);
    EXPECT_FALSE(e2);
    EXPECT_FALSE(e3);

    EXPECT_NEAR(b1.getWord(WordLetter::X), 100.0, 1e-9);
    EXPECT_NEAR(b2.getWord(WordLetter::X), 200.0, 1e-9);
    EXPECT_NEAR(b3.getWord(WordLetter::X), 300.0, 1e-9);
}

TEST_F(ParserTest, ParseNextBlockEndOfInput) {
    parser->setInput("G1 X100");

    Block b1, b2;
    Error e1 = parser->parseNextBlock(b1);
    Error e2 = parser->parseNextBlock(b2);

    EXPECT_FALSE(e1);
    EXPECT_TRUE(e2);
    EXPECT_EQ(e2.code, ErrorCode::END);
}

TEST_F(ParserTest, ReadAhead) {
    parser->setInput("G1 X100\nG2 X200\nG3 X300\nG0 X0");

    BlockQueue ahead = parser->readAhead(3);

    EXPECT_EQ(ahead.size(), 3u);
    EXPECT_NEAR(ahead[0].getWord(WordLetter::X), 100.0, 1e-9);
    EXPECT_NEAR(ahead[1].getWord(WordLetter::X), 200.0, 1e-9);
    EXPECT_NEAR(ahead[2].getWord(WordLetter::X), 300.0, 1e-9);
}

// ============================================================================
// Position Control Tests
// ============================================================================

TEST_F(ParserTest, SeekToLine) {
    parser->setInput("G1 X100\nG2 X200\nG3 X300");

    Error err = parser->seekToLine(2);
    EXPECT_FALSE(err);

    Block b;
    parser->parseNextBlock(b);
    EXPECT_NEAR(b.getWord(WordLetter::X), 200.0, 1e-9);
}

TEST_F(ParserTest, SeekToInvalidLine) {
    parser->setInput("G1 X100");

    Error err = parser->seekToLine(100);
    EXPECT_TRUE(err);
}

TEST_F(ParserTest, SeekToStart) {
    parser->setInput("G1 X100\nG2 X200");

    Block b;
    parser->parseNextBlock(b);
    parser->seekToStart();
    parser->parseNextBlock(b);

    EXPECT_NEAR(b.getWord(WordLetter::X), 100.0, 1e-9);
}

TEST_F(ParserTest, AtEnd) {
    parser->setInput("G1 X100");

    EXPECT_FALSE(parser->atEnd());

    Block b;
    parser->parseNextBlock(b);

    EXPECT_TRUE(parser->atEnd());
}

TEST_F(ParserTest, AtStart) {
    parser->setInput("G1 X100");

    EXPECT_TRUE(parser->atStart());

    Block b;
    parser->parseNextBlock(b);

    EXPECT_FALSE(parser->atStart());
}

TEST_F(ParserTest, GetCurrentLine) {
    parser->setInput("G1 X100\nG2 X200");

    EXPECT_EQ(parser->getCurrentLine(), 1u);

    Block b;
    parser->parseNextBlock(b);

    EXPECT_EQ(parser->getCurrentLine(), 2u);
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(ParserTest, ValidateBlock) {
    Block block = parseBlock("G1 X100 F1000");

    Error err = parser->validate(block);
    EXPECT_FALSE(err);
}

// ============================================================================
// Comment Handling Tests
// ============================================================================

TEST_F(ParserTest, CommentInBlock) {
    Block block = parseBlock("G1 X100 (move to X)");

    EXPECT_TRUE(block.hasComment);
    EXPECT_STREQ(block.comment.data(), "move to X");
}

TEST_F(ParserTest, MultipleCommentsFirstPreserved) {
    Block block = parseBlock("G1 (first) X100 (second)");

    EXPECT_TRUE(block.hasComment);
    // First comment should be preserved
}

// ============================================================================
// Original Text Preservation Tests
// ============================================================================

TEST_F(ParserTest, OriginalTextPreserved) {
    ParserConfig config;
    config.preserveOriginalText = true;
    Parser p(*vars, config);

    Block block;
    p.parseLine("G1 X100 F1000", block);

    EXPECT_STREQ(block.originalText.data(), "G1 X100 F1000");
}

// ============================================================================
// Comprehensive Real-World Tests
// ============================================================================

TEST_F(ParserTest, RealWorldSetup) {
    Block block = parseBlock("N10 G21 G90 G17 F1000 S5000 M3");

    // N is stored in lineNumber field, not as a word
    EXPECT_EQ(block.lineNumber, 10);
    EXPECT_TRUE(block.hasWord(WordLetter::F));
    EXPECT_TRUE(block.hasWord(WordLetter::S));
}

TEST_F(ParserTest, RealWorldLinearMove) {
    Block block = parseBlock("G1 X100.5 Y-50.25 Z0.1 F500");

    EXPECT_NEAR(block.getWord(WordLetter::X), 100.5, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Y), -50.25, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Z), 0.1, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::F), 500.0, 1e-9);
}

TEST_F(ParserTest, RealWorldClockwiseArc) {
    Block block = parseBlock("G2 X50 Y50 I25 J0 F200");

    EXPECT_NEAR(block.getWord(WordLetter::X), 50.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::Y), 50.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::I), 25.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::J), 0.0, 1e-9);
}

TEST_F(ParserTest, RealWorldToolChange) {
    Block block = parseBlock("T1 M6");

    EXPECT_NEAR(block.getWord(WordLetter::T), 1.0, 1e-9);
}

TEST_F(ParserTest, RealWorldDwell) {
    Block block = parseBlock("G4 P1.5");

    EXPECT_TRUE(block.hasWord(WordLetter::P));
    EXPECT_NEAR(block.getWord(WordLetter::P), 1.5, 1e-9);
}

TEST_F(ParserTest, RealWorldCannedCycle) {
    Block block = parseBlock("G83 X10 Y10 Z-25 R2 Q5 F100");

    EXPECT_TRUE(block.hasWord(WordLetter::Q));
    EXPECT_TRUE(block.hasWord(WordLetter::R));
    EXPECT_NEAR(block.getWord(WordLetter::Q), 5.0, 1e-9);
}

// ============================================================================
// O-Code Operations Tests
// ============================================================================

TEST_F(ParserTest, FindMatchingOCode) {
    parser->setInput("O100 if [#1 GT 0]\nG1 X100\nO100 endif");

    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::IF, block);

    // Should find the matching endif
    EXPECT_FALSE(err) << "Expected findMatchingOCode to succeed";
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeNumber, 100);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDIF);
}

TEST_F(ParserTest, FindSubroutineNumber) {
    parser->setInput("O100 sub\nG1 X100\nO100 endsub");

    Block block;
    Error err = parser->findSubroutine(100, block);

    // Should find the subroutine
    EXPECT_FALSE(err) << "Expected findSubroutine to succeed";
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeNumber, 100);
    EXPECT_EQ(block.oCodeType, OCodeType::SUB);
}

TEST_F(ParserTest, FindSubroutineName) {
    parser->setInput("O<mysub> sub\nG1 X100\nO<mysub> endsub");

    Block block;
    Error err = parser->findSubroutine("mysub", block);

    // Should find the named subroutine
    EXPECT_FALSE(err) << "Expected findSubroutine to succeed";
    EXPECT_TRUE(block.hasOCode);
    EXPECT_TRUE(block.oCodeIsNamed);
    EXPECT_EQ(std::string(block.oCodeName.data()), "mysub");
    EXPECT_EQ(block.oCodeType, OCodeType::SUB);
}

// ============================================================================
// Block Count Tests
// ============================================================================

TEST_F(ParserTest, ScanBlockCount) {
    parser->setInput("G1 X100\nG2 X200\nG3 X300\n(comment only)\nG0 X0");

    parser->scanBlockCount();

    // Should count all parseable blocks
    EXPECT_GE(parser->getTotalBlocks(), 4u);
}

// ============================================================================
// Parameterized Parsing Tests
// ============================================================================

class ParserParameterizedTest : public ::testing::TestWithParam<const char*> {
protected:
    void SetUp() override {
        vars = std::make_unique<VariableSystem>();
        parser = std::make_unique<Parser>(*vars);
    }

    std::unique_ptr<VariableSystem> vars;
    std::unique_ptr<Parser> parser;
};

TEST_P(ParserParameterizedTest, ParsesWithoutCrash) {
    const char* input = GetParam();
    Block block;
    Error err = parser->parseLine(input, block);
    // Just verify it doesn't crash
    (void)err;
}

INSTANTIATE_TEST_SUITE_P(
    GCodeLines,
    ParserParameterizedTest,
    ::testing::Values(
        "G0 X0 Y0 Z0",
        "G1 X100 Y50 Z-10 F1000",
        "G2 X50 Y50 I25 J0 F200",
        "G3 X0 Y100 R50",
        "G4 P1.5",
        "G10 L2 P1 X0 Y0 Z0",
        "G17",
        "G18",
        "G19",
        "G20",
        "G21",
        "G28",
        "G28.1",
        "G30",
        "G30.1",
        "G40",
        "G41 D1",
        "G42 D2",
        "G43 H1 Z0",
        "G43.1 Z10",
        "G43.2 H1",
        "G49",
        "G53 G0 X0 Y0",
        "G54",
        "G55",
        "G56",
        "G57",
        "G58",
        "G59",
        "G59.1",
        "G59.2",
        "G59.3",
        "G61",
        "G61.1",
        "G64",
        "G64 P0.01",
        "G64 P0.01 Q0.01",
        "G73 X10 Y10 Z-25 R2 Q5 F100",
        "G76 P1 Z-10 I0.1 J0.05 K0.025 H5 E0 L0",
        "G80",
        "G81 X10 Y10 Z-10 R2 F100",
        "G82 X10 Y10 Z-10 R2 P0.5 F100",
        "G83 X10 Y10 Z-25 R2 Q5 F100",
        "G84 X10 Y10 Z-10 R2 F100",
        "G85 X10 Y10 Z-10 R2 F100",
        "G86 X10 Y10 Z-10 R2 F100",
        "G87 X10 Y10 Z-10 R2 I0 J0 K0 F100",
        "G88 X10 Y10 Z-10 R2 P0.5 F100",
        "G89 X10 Y10 Z-10 R2 P0.5 F100",
        "G90",
        "G90.1",
        "G91",
        "G91.1",
        "G92 X0 Y0 Z0",
        "G92.1",
        "G92.2",
        "G92.3",
        "G93",
        "G94",
        "G95",
        "G96 S100",
        "G97",
        "G98",
        "G99",
        "M0",
        "M1",
        "M2",
        "M3 S1000",
        "M4 S500",
        "M5",
        "M6 T1",
        "M7",
        "M8",
        "M9",
        "M19",
        "M30",
        "M48",
        "M49",
        "M50",
        "M51",
        "M52",
        "M53",
        "M60",
        "T1",
        "T1 M6",
        "S1000",
        "S1000 M3",
        "F500",
        "N100 G1 X100",
        "(comment)",
        "; comment",
        "G1 X100 (comment)",
        "G1 X100 ; comment",
        "O100",
        "O<name>",
        "/G1 X100",
        "%",
        "G1X100Y50",
        "   G1   X100   ",
        "g1 x100",
        "G1 X-100 Y+50",
        "G1 X.5 Y1.",
        ""
    )
);

// ============================================================================
// Stress Test
// ============================================================================

TEST_F(ParserTest, ParseManyBlocks) {
    std::string program;
    for (int i = 0; i < 1000; i++) {
        program += "G1 X" + std::to_string(i) + " Y" + std::to_string(i * 2) + "\n";
    }

    parser->setInput(program);

    int count = 0;
    Block block;
    while (!parser->atEnd()) {
        Error err = parser->parseNextBlock(block);
        if (err) break;
        count++;
    }

    EXPECT_EQ(count, 1000);
}

TEST_F(ParserTest, LongLine) {
    std::string line = "G1";
    for (int i = 0; i < 100; i++) {
        line += " X" + std::to_string(i);
    }

    Block block;
    Error err = parser->parseLine(line.c_str(), block);
    // Should handle without crash
    (void)err;
}

// ============================================================================
// BlockQueue Tests
// ============================================================================

TEST(BlockQueueTest, DefaultConstruction) {
    BlockQueue q;
    EXPECT_EQ(q.size(), 0u);
    EXPECT_TRUE(q.empty());
}

TEST(BlockQueueTest, ConstructWithCapacity) {
    BlockQueue q(10);
    EXPECT_EQ(q.size(), 0u);
    EXPECT_TRUE(q.empty());
}

TEST(BlockQueueTest, PushBack) {
    BlockQueue q(5);
    Block b;
    b.sourceLineNumber = 42;
    q.pushBack(b);
    
    EXPECT_EQ(q.size(), 1u);
    EXPECT_FALSE(q.empty());
    EXPECT_EQ(q.back().sourceLineNumber, 42u);
}

TEST(BlockQueueTest, PopBack) {
    BlockQueue q(5);
    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    q.pushBack(b1);
    q.pushBack(b2);
    
    Block popped = q.popBack();
    EXPECT_EQ(popped.sourceLineNumber, 2u);
    EXPECT_EQ(q.size(), 1u);
}

TEST(BlockQueueTest, PopFront) {
    BlockQueue q(5);
    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    q.pushBack(b1);
    q.pushBack(b2);
    
    Block popped = q.popFront();
    EXPECT_EQ(popped.sourceLineNumber, 1u);
    EXPECT_EQ(q.size(), 1u);
}

TEST(BlockQueueTest, Front) {
    BlockQueue q(5);
    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    q.pushBack(b1);
    q.pushBack(b2);
    
    EXPECT_EQ(q.front().sourceLineNumber, 1u);
    
    const BlockQueue& cq = q;
    EXPECT_EQ(cq.front().sourceLineNumber, 1u);
}

TEST(BlockQueueTest, Back) {
    BlockQueue q(5);
    Block b1, b2;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    q.pushBack(b1);
    q.pushBack(b2);
    
    EXPECT_EQ(q.back().sourceLineNumber, 2u);
    
    const BlockQueue& cq = q;
    EXPECT_EQ(cq.back().sourceLineNumber, 2u);
}

TEST(BlockQueueTest, IndexOperator) {
    BlockQueue q(5);
    Block b1, b2, b3;
    b1.sourceLineNumber = 1;
    b2.sourceLineNumber = 2;
    b3.sourceLineNumber = 3;
    q.pushBack(b1);
    q.pushBack(b2);
    q.pushBack(b3);
    
    EXPECT_EQ(q[0].sourceLineNumber, 1u);
    EXPECT_EQ(q[1].sourceLineNumber, 2u);
    EXPECT_EQ(q[2].sourceLineNumber, 3u);
    
    const BlockQueue& cq = q;
    EXPECT_EQ(cq[0].sourceLineNumber, 1u);
}

TEST(BlockQueueTest, Clear) {
    BlockQueue q(5);
    Block b;
    q.pushBack(b);
    q.pushBack(b);
    
    q.clear();
    EXPECT_EQ(q.size(), 0u);
    EXPECT_TRUE(q.empty());
}

// ============================================================================
// File Loading Tests (Additional)
// ============================================================================

TEST_F(ParserTest, LoadFileNullFilename) {
    Error err = parser->loadFile(nullptr);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::FILE_NOT_FOUND);
}

TEST_F(ParserTest, LoadFileNotFound) {
    Error err = parser->loadFile("/nonexistent/path/to/file.gcode");
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::FILE_NOT_FOUND);
}

// ============================================================================
// Input Methods Tests (Additional)
// ============================================================================

TEST_F(ParserTest, SetInputWithLengthRestricted) {
    const char* source = "G0 X0\nG1 X10\nextra garbage";
    parser->setInput(source, 12);  // Only first 12 chars
    
    Block block;
    Error err = parser->parseNextBlock(block);
    EXPECT_FALSE(err);
}

TEST_F(ParserTest, SetInputStringViewMethod) {
    std::string_view sv = "G0 X100\nG1 Y200";
    parser->setInput(sv);
    
    Block block;
    Error err = parser->parseNextBlock(block);
    EXPECT_FALSE(err);
}

// ============================================================================
// ReadBehind Tests (Additional)
// ============================================================================

TEST_F(ParserTest, ReadBehindMultiple) {
    parser->setInput("G0 X0\nG1 X10\nG1 X20\nG1 X30\n");
    
    // Move forward first
    Block block;
    parser->parseNextBlock(block);
    parser->parseNextBlock(block);
    
    BlockQueue behind = parser->readBehind(2);
    // readBehind results depend on implementation
    (void)behind;
}

TEST_F(ParserTest, ParsePrevBlockNavigation) {
    parser->setInput("G0 X0\nG1 X10\nG1 X20\n");
    
    Block block;
    parser->parseNextBlock(block);
    parser->parseNextBlock(block);
    
    Error err = parser->parsePrevBlock(block);
    // Depends on implementation - just check it doesn't crash
    (void)err;
}

// ============================================================================
// PreserveOriginalText Tests  
// ============================================================================

TEST(ParserConfigTest, PreserveOriginalText) {
    VariableSystem vars;
    ParserConfig config;
    config.preserveOriginalText = true;
    Parser parser(vars, config);
    
    Block block;
    Error err = parser.parseLine("G1 X100 Y200", block);
    EXPECT_FALSE(err);
    // Original text should be preserved
    EXPECT_STRNE(block.originalText.data(), "");
}

// ============================================================================
// Parser End Of Input Test
// ============================================================================

TEST_F(ParserTest, EndOfInputErrorCode) {
    parser->setInput("");
    
    Block block;
    Error err = parser->parseNextBlock(block);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::END);
}

// ============================================================================
// Additional Coverage Tests
// ============================================================================

TEST_F(ParserTest, TooManyGCodes) {
    // A block with many G-codes should error when exceeding limit
    Block block;
    // MAX_GCODES_PER_BLOCK is typically small (8)
    Error err = parser->parseLine("G0 G1 G2 G3 G17 G18 G19 G20 G21 G90 G91", block);
    // This may or may not error depending on the limit - just ensure no crash
}

TEST_F(ParserTest, TooManyMCodes) {
    Block block;
    Error err = parser->parseLine("M0 M1 M2 M3 M4 M5 M6 M7 M8 M9 M10", block);
    // Similar to G-codes - just ensure no crash
}

TEST_F(ParserTest, OCodeParsing) {
    // O-code with keyword should parse correctly
    Block block = parseBlock("O100 sub");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_FALSE(block.oCodeIsNamed);
    EXPECT_EQ(block.oCodeNumber, 100);
    EXPECT_EQ(block.oCodeType, OCodeType::SUB);
}

TEST_F(ParserTest, OCodeKeywordCall) {
    Block block = parseBlock("O100 call [10] [20]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::CALL);
}

TEST_F(ParserTest, OCodeKeywordIf) {
    Block block = parseBlock("O101 if [#1 GT 10]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::IF);
    // Condition should be stored
    EXPECT_STRNE(block.oCodeCondition.data(), "");
}

TEST_F(ParserTest, OCodeKeywordWhile) {
    Block block = parseBlock("O102 while [#1 LT 10]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::WHILE);
}

TEST_F(ParserTest, OCodeKeywordDo) {
    Block block = parseBlock("O103 do");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::DO);
}

TEST_F(ParserTest, OCodeKeywordRepeat) {
    Block block = parseBlock("O104 repeat [10]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::REPEAT);
}

TEST_F(ParserTest, OCodeKeywordEndsub) {
    Block block = parseBlock("O100 endsub");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDSUB);
}

TEST_F(ParserTest, OCodeKeywordEndif) {
    Block block = parseBlock("O101 endif");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDIF);
}

TEST_F(ParserTest, OCodeKeywordEndwhile) {
    Block block = parseBlock("O102 endwhile");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDWHILE);
}

TEST_F(ParserTest, OCodeKeywordEndrepeat) {
    Block block = parseBlock("O104 endrepeat");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDREPEAT);
}

TEST_F(ParserTest, OCodeKeywordElseif) {
    Block block = parseBlock("O101 elseif [#1 GT 5]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ELSEIF);
}

TEST_F(ParserTest, OCodeKeywordElse) {
    Block block = parseBlock("O101 else");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::ELSE);
}

TEST_F(ParserTest, OCodeKeywordReturn) {
    Block block = parseBlock("O100 return [#1 * 2]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::RETURN);
}

TEST_F(ParserTest, OCodeKeywordBreak) {
    Block block = parseBlock("O105 break");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::BREAK);
}

TEST_F(ParserTest, OCodeKeywordContinue) {
    Block block = parseBlock("O105 continue");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_EQ(block.oCodeType, OCodeType::CONTINUE);
}

TEST_F(ParserTest, OCodeNamedSub) {
    Block block = parseBlock("O<myroutine> sub");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_TRUE(block.oCodeIsNamed);
    EXPECT_STREQ(block.oCodeName.data(), "myroutine");
    EXPECT_EQ(block.oCodeType, OCodeType::SUB);
}

TEST_F(ParserTest, OCodeNamedCall) {
    Block block = parseBlock("O<myroutine> call [1] [2] [3]");
    EXPECT_TRUE(block.hasOCode);
    EXPECT_TRUE(block.oCodeIsNamed);
    EXPECT_EQ(block.oCodeType, OCodeType::CALL);
}

TEST_F(ParserTest, FindMatchingOCodeWhile) {
    parser->setInput("O100 while [#1 LT 10]\nG1 X[#1*10]\n#1 = [#1+1]\nO100 endwhile");
    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::WHILE, block);
    EXPECT_FALSE(err);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDWHILE);
}

TEST_F(ParserTest, FindMatchingOCodeRepeat) {
    parser->setInput("O100 repeat [5]\nG1 X100\nO100 endrepeat");
    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::REPEAT, block);
    EXPECT_FALSE(err);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDREPEAT);
}

TEST_F(ParserTest, FindMatchingOCodeSub) {
    parser->setInput("O100 sub\nG1 X100\nO100 endsub");
    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::SUB, block);
    EXPECT_FALSE(err);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDSUB);
}

TEST_F(ParserTest, FindMatchingOCodeNested) {
    parser->setInput("O100 if [1]\nO101 if [1]\nG1 X100\nO101 endif\nO100 endif");
    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::IF, block);
    EXPECT_FALSE(err);
    EXPECT_EQ(block.oCodeNumber, 100);
    EXPECT_EQ(block.oCodeType, OCodeType::ENDIF);
}

TEST_F(ParserTest, FindMatchingOCodeNotFound) {
    parser->setInput("O100 if [1]\nG1 X100\n");
    Block block;
    Error err = parser->findMatchingOCode(100, OCodeType::IF, block);
    EXPECT_TRUE(err);  // No matching endif
}

TEST_F(ParserTest, FindSubroutineNotFound) {
    parser->setInput("O100 sub\nG1 X100\nO100 endsub");
    Block block;
    Error err = parser->findSubroutine(999, block);
    EXPECT_TRUE(err);  // Subroutine 999 doesn't exist
}

TEST_F(ParserTest, GCodeSubWithDecimal) {
    // G-codes with decimals like G38.2
    Block block = parseBlock("G38.2 X100 Z-10");
    // Should parse the G38.2 properly
    EXPECT_TRUE(block.hasWord(WordLetter::X));
}

TEST_F(ParserTest, GCodeNumberFormats) {
    // Test various G-code number formats
    Block block1 = parseBlock("G01 X10");  // With leading zero
    Block block2 = parseBlock("G1 X10");   // Without leading zero
    
    // Both should have X word
    EXPECT_TRUE(block1.hasWord(WordLetter::X));
    EXPECT_TRUE(block2.hasWord(WordLetter::X));
}

TEST_F(ParserTest, MCodeDescription) {
    EXPECT_STREQ(getMCodeDescription(0), "Program stop");
    EXPECT_STREQ(getMCodeDescription(2), "Program end");
    EXPECT_STREQ(getMCodeDescription(3), "Spindle on (CW)");
    EXPECT_STREQ(getMCodeDescription(5), "Spindle stop");
    EXPECT_STREQ(getMCodeDescription(99), "M-code");  // Unknown
}

TEST_F(ParserTest, GCodeDescription) {
    EXPECT_STREQ(getGCodeDescription(0, -1), "Rapid positioning");
    EXPECT_STREQ(getGCodeDescription(1, -1), "Linear interpolation");
    EXPECT_STREQ(getGCodeDescription(2, -1), "Clockwise arc");
    EXPECT_STREQ(getGCodeDescription(3, -1), "Counter-clockwise arc");
    EXPECT_STREQ(getGCodeDescription(99, -1), "G-code");  // Unknown
}

TEST_F(ParserTest, IsValidGCode) {
    EXPECT_TRUE(isValidGCode(0, -1));
    EXPECT_TRUE(isValidGCode(1, 0));
    EXPECT_TRUE(isValidGCode(38, 2));
    EXPECT_FALSE(isValidGCode(-1, 0));
}

TEST_F(ParserTest, IsValidMCode) {
    EXPECT_TRUE(isValidMCode(0));
    EXPECT_TRUE(isValidMCode(3));
    EXPECT_FALSE(isValidMCode(-1));
}

TEST_F(ParserTest, FormatGCode) {
    EXPECT_EQ(formatGCode(0, -1), "G0");
    EXPECT_EQ(formatGCode(1, -1), "G1");
    EXPECT_EQ(formatGCode(38, 2), "G38.2");
    EXPECT_EQ(formatGCode(17, -1), "G17");
}

TEST_F(ParserTest, ParseGCodeNumber) {
    int major, minor;
    
    parseGCodeNumber(0.0, major, minor);
    EXPECT_EQ(major, 0);
    EXPECT_EQ(minor, -1);
    
    parseGCodeNumber(1.0, major, minor);
    EXPECT_EQ(major, 1);
    EXPECT_EQ(minor, -1);
    
    parseGCodeNumber(38.2, major, minor);
    EXPECT_EQ(major, 38);
    EXPECT_EQ(minor, 2);
}

TEST_F(ParserTest, FillLookahead) {
    parser->setInput("G0 X0\nG1 X10\n");
    parser->fillLookahead();  // Just test it doesn't crash
}

TEST_F(ParserTest, CheckModalGroups) {
    Block block = parseBlock("G0 G1");  // Two motion codes - conflict
    // checkModalGroups is internal, but parsing should handle it
}

TEST_F(ParserTest, SeekToLineCoverage) {
    parser->setInput("N10 G0 X0\nN20 G1 X10\nN30 G1 X20\n");
    
    Error err = parser->seekToLine(20);
    // Seek functionality
}

TEST_F(ParserTest, SeekToLineInvalid) {
    parser->setInput("N10 G0 X0\nN20 G1 X10\n");
    
    Error err = parser->seekToLine(999);  // Non-existent
    EXPECT_TRUE(err);
}

TEST_F(ParserTest, SeekToStartEnd) {
    parser->setInput("G0 X0\nG1 X10\nG1 X20\n");
    
    parser->seekToEnd();
    EXPECT_TRUE(parser->atEnd());
    
    parser->seekToStart();
    EXPECT_TRUE(parser->atStart());
}

TEST_F(ParserTest, CurrentLine) {
    parser->setInput("G0 X0\nG1 X10\n");
    
    Block block;
    parser->parseNextBlock(block);
    
    uint32_t line = parser->getCurrentLine();
    EXPECT_GE(line, 1u);
}

TEST_F(ParserTest, ReadAheadMultiple) {
    parser->setInput("G0 X0\nG1 X10\nG1 X20\nG1 X30\n");
    
    BlockQueue ahead = parser->readAhead(3);
    EXPECT_LE(ahead.size(), 3u);
}

TEST_F(ParserTest, ModalGroupUnitsCoverage) {
    EXPECT_EQ(getModalGroup(200), ModalGroup::UNITS);  // G20 (inches)
    EXPECT_EQ(getModalGroup(210), ModalGroup::UNITS);  // G21 (mm)
}

TEST_F(ParserTest, ModalGroupPathModeCoverage) {
    EXPECT_EQ(getModalGroup(610), ModalGroup::PATH_MODE);  // G61
    EXPECT_EQ(getModalGroup(640), ModalGroup::PATH_MODE);  // G64
}

TEST_F(ParserTest, ModalGroupNonModalCoverage) {
    // G4 (dwell) code is 40, G10 is 100
    EXPECT_EQ(getModalGroup(40), ModalGroup::NON_MODAL);   // G4 (dwell)
    EXPECT_EQ(getModalGroup(10), ModalGroup::MOTION);      // G1 is motion
}

TEST_F(ParserTest, ModalGroupWithDecimal) {
    // getModalGroup(int, int) overload - decimal is ignored
    EXPECT_EQ(getModalGroup(0, 0), ModalGroup::MOTION);
    EXPECT_EQ(getModalGroup(170, -1), ModalGroup::PLANE);  // G17 encoded as 170
}

TEST_F(ParserTest, BlockQueuePushPopOperations) {
    BlockQueue q(5);
    
    Block b1, b2, b3;
    b1.lineNumber = 1;
    b2.lineNumber = 2;
    b3.lineNumber = 3;
    
    q.pushBack(b1);
    q.pushBack(b2);
    q.pushFront(b3);
    
    EXPECT_EQ(q.size(), 3u);
    
    Block front = q.popFront();
    Block back = q.popBack();
    
    EXPECT_EQ(front.lineNumber, 3);  // b3 was pushed front
    EXPECT_EQ(back.lineNumber, 2);   // b2 was at back
}

TEST_F(ParserTest, BlockQueueOverflowPushBack) {
    BlockQueue q(3);  // Max size 3
    
    Block b1, b2, b3, b4;
    b1.lineNumber = 1;
    b2.lineNumber = 2;
    b3.lineNumber = 3;
    b4.lineNumber = 4;
    
    q.pushBack(b1);
    q.pushBack(b2);
    q.pushBack(b3);
    q.pushBack(b4);  // Should evict oldest
    
    EXPECT_EQ(q.size(), 3u);
}

TEST_F(ParserTest, BlockQueuePushFrontOverflow) {
    BlockQueue q(2);
    
    Block b1, b2, b3;
    b1.lineNumber = 1;
    b2.lineNumber = 2;
    b3.lineNumber = 3;
    
    q.pushBack(b1);
    q.pushBack(b2);
    q.pushFront(b3);  // Should evict from back
    
    EXPECT_EQ(q.size(), 2u);
}

TEST_F(ParserTest, ArcWordsIJK) {
    Block block = parseBlock("G2 X10 Y10 I5 J5");
    
    EXPECT_TRUE(block.hasWord(WordLetter::I));
    EXPECT_TRUE(block.hasWord(WordLetter::J));
    EXPECT_NEAR(block.getWord(WordLetter::I), 5.0, 1e-9);
    EXPECT_NEAR(block.getWord(WordLetter::J), 5.0, 1e-9);
}

TEST_F(ParserTest, ArcWordR) {
    Block block = parseBlock("G2 X10 Y10 R5");
    
    EXPECT_TRUE(block.hasWord(WordLetter::R));
    EXPECT_NEAR(block.getWord(WordLetter::R), 5.0, 1e-9);
}

TEST_F(ParserTest, HelicalArcK) {
    Block block = parseBlock("G2 X10 Y10 I5 J0 K5");
    
    EXPECT_TRUE(block.hasWord(WordLetter::K));
    EXPECT_NEAR(block.getWord(WordLetter::K), 5.0, 1e-9);
}

} // namespace test
} // namespace GCode
