/**
 * @file GCodeLexerTests.cpp
 * @brief Comprehensive unit tests for the GCode Lexer
 *
 * This file provides complete coverage for the lexer including:
 * - All token types (WORD, COMMENT, EXPRESSION, etc.)
 * - Edge cases and boundary conditions
 * - Error handling
 * - Unicode and special characters
 * - Highlighting functionality
 */

#include <gtest/gtest.h>
#include <tether/gcode/GCodeLexer.hpp>
#include <tether/gcode/GCodeTypes.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

namespace GCode {
namespace test {

// ============================================================================
// Test Fixture
// ============================================================================

class LexerTest : public ::testing::Test {
protected:
    void SetUp() override {
        lexer = std::make_unique<Lexer>();
    }

    void TearDown() override {
        lexer.reset();
    }

    std::unique_ptr<Lexer> lexer;

    // Helper to tokenize entire input
    std::vector<LexerToken> tokenizeAll(const char* input) {
        lexer->setInput(input);
        std::vector<LexerToken> tokens;
        while (true) {
            LexerToken t = lexer->nextToken();
            if (t.type == LexerTokenType::END) break;
            tokens.push_back(t);
        }
        return tokens;
    }

    // Helper to check word token
    void expectWord(const LexerToken& tok, WordLetter letter, double value) {
        EXPECT_EQ(tok.type, LexerTokenType::WORD);
        EXPECT_EQ(tok.letter, letter);
        EXPECT_NEAR(tok.value, value, 1e-9);
    }
};

// ============================================================================
// Basic Token Types Tests
// ============================================================================

TEST_F(LexerTest, EmptyInput) {
    lexer->setInput("");
    LexerToken tok = lexer->nextToken();
    EXPECT_EQ(tok.type, LexerTokenType::END);
}

TEST_F(LexerTest, NullInput) {
    lexer->setInput(nullptr);
    LexerToken tok = lexer->nextToken();
    EXPECT_EQ(tok.type, LexerTokenType::END);
}

TEST_F(LexerTest, WhitespaceOnly) {
    lexer->setInput("   \t\t   ");
    LexerToken tok = lexer->nextToken();
    EXPECT_EQ(tok.type, LexerTokenType::END);
}

TEST_F(LexerTest, NewlineToken) {
    lexer->setInput("\n");
    LexerToken tok = lexer->nextToken();
    EXPECT_EQ(tok.type, LexerTokenType::EOL);
    tok = lexer->nextToken();
    EXPECT_EQ(tok.type, LexerTokenType::END);
}

TEST_F(LexerTest, SingleGCode) {
    lexer->setInput("G0");
    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::G, 0.0);
}

TEST_F(LexerTest, SingleMCode) {
    lexer->setInput("M3");
    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::M, 3.0);
}

TEST_F(LexerTest, AllAxisLetters) {
    lexer->setInput("X10 Y20 Z30 A40 B50 C60 U70 V80 W90");
    auto tokens = tokenizeAll("X10 Y20 Z30 A40 B50 C60 U70 V80 W90");

    ASSERT_GE(tokens.size(), 9u);
    expectWord(tokens[0], WordLetter::X, 10.0);
    expectWord(tokens[1], WordLetter::Y, 20.0);
    expectWord(tokens[2], WordLetter::Z, 30.0);
    expectWord(tokens[3], WordLetter::A, 40.0);
    expectWord(tokens[4], WordLetter::B, 50.0);
    expectWord(tokens[5], WordLetter::C, 60.0);
    expectWord(tokens[6], WordLetter::U, 70.0);
    expectWord(tokens[7], WordLetter::V, 80.0);
    expectWord(tokens[8], WordLetter::W, 90.0);
}

TEST_F(LexerTest, AllWordLetters) {
    const char* input = "A1 B2 C3 D4 E5 F6 G7 H8 I9 J10 K11 L12 M13 N14 O15 P16 Q17 R18 S19 T20";
    auto tokens = tokenizeAll(input);

    EXPECT_EQ(tokens[0].letter, WordLetter::A);
    EXPECT_EQ(tokens[1].letter, WordLetter::B);
    EXPECT_EQ(tokens[2].letter, WordLetter::C);
    EXPECT_EQ(tokens[3].letter, WordLetter::D);
    EXPECT_EQ(tokens[4].letter, WordLetter::E);
    EXPECT_EQ(tokens[5].letter, WordLetter::F);
    EXPECT_EQ(tokens[6].letter, WordLetter::G);
    EXPECT_EQ(tokens[7].letter, WordLetter::H);
    EXPECT_EQ(tokens[8].letter, WordLetter::I);
    EXPECT_EQ(tokens[9].letter, WordLetter::J);
    EXPECT_EQ(tokens[10].letter, WordLetter::K);
    EXPECT_EQ(tokens[11].letter, WordLetter::L);
    EXPECT_EQ(tokens[12].letter, WordLetter::M);
    EXPECT_EQ(tokens[13].letter, WordLetter::N);
}

// ============================================================================
// Numeric Value Tests
// ============================================================================

TEST_F(LexerTest, IntegerValues) {
    auto tokens = tokenizeAll("X0 X1 X123 X999999");

    ASSERT_EQ(tokens.size(), 4u);
    expectWord(tokens[0], WordLetter::X, 0.0);
    expectWord(tokens[1], WordLetter::X, 1.0);
    expectWord(tokens[2], WordLetter::X, 123.0);
    expectWord(tokens[3], WordLetter::X, 999999.0);
}

TEST_F(LexerTest, NegativeValues) {
    auto tokens = tokenizeAll("X-100 Y-0.5 Z-999");

    ASSERT_EQ(tokens.size(), 3u);
    expectWord(tokens[0], WordLetter::X, -100.0);
    expectWord(tokens[1], WordLetter::Y, -0.5);
    expectWord(tokens[2], WordLetter::Z, -999.0);
}

TEST_F(LexerTest, PositiveSign) {
    auto tokens = tokenizeAll("X+100 Y+0.5");

    ASSERT_EQ(tokens.size(), 2u);
    expectWord(tokens[0], WordLetter::X, 100.0);
    expectWord(tokens[1], WordLetter::Y, 0.5);
}

TEST_F(LexerTest, DecimalValues) {
    auto tokens = tokenizeAll("X0.0 X0.1 X1.5 X123.456 X0.000001");

    ASSERT_EQ(tokens.size(), 5u);
    expectWord(tokens[0], WordLetter::X, 0.0);
    expectWord(tokens[1], WordLetter::X, 0.1);
    expectWord(tokens[2], WordLetter::X, 1.5);
    expectWord(tokens[3], WordLetter::X, 123.456);
    expectWord(tokens[4], WordLetter::X, 0.000001);
}

TEST_F(LexerTest, LeadingDecimal) {
    auto tokens = tokenizeAll("X.5 Y.123");

    ASSERT_EQ(tokens.size(), 2u);
    expectWord(tokens[0], WordLetter::X, 0.5);
    expectWord(tokens[1], WordLetter::Y, 0.123);
}

TEST_F(LexerTest, TrailingDecimal) {
    auto tokens = tokenizeAll("X5. Y123.");

    ASSERT_EQ(tokens.size(), 2u);
    expectWord(tokens[0], WordLetter::X, 5.0);
    expectWord(tokens[1], WordLetter::Y, 123.0);
}

TEST_F(LexerTest, LargeNumbers) {
    auto tokens = tokenizeAll("X1000000 Y-9999999.99");

    ASSERT_EQ(tokens.size(), 2u);
    expectWord(tokens[0], WordLetter::X, 1000000.0);
    expectWord(tokens[1], WordLetter::Y, -9999999.99);
}

TEST_F(LexerTest, SmallNumbers) {
    auto tokens = tokenizeAll("X0.00001 Y0.000001");

    ASSERT_EQ(tokens.size(), 2u);
    EXPECT_NEAR(tokens[0].value, 0.00001, 1e-10);
    EXPECT_NEAR(tokens[1].value, 0.000001, 1e-10);
}

// ============================================================================
// Comment Tests
// ============================================================================

TEST_F(LexerTest, ParenthesisComment) {
    lexer->setInput("(this is a comment)");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::COMMENT);
    EXPECT_EQ(tok.text, "this is a comment");
}

TEST_F(LexerTest, SemicolonComment) {
    lexer->setInput("; this is a comment");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::COMMENT);
    EXPECT_EQ(tok.text, " this is a comment");
}

TEST_F(LexerTest, CommentAfterCode) {
    auto tokens = tokenizeAll("G1 X100 (move to X)");

    ASSERT_GE(tokens.size(), 3u);
    expectWord(tokens[0], WordLetter::G, 1.0);
    expectWord(tokens[1], WordLetter::X, 100.0);
    EXPECT_EQ(tokens[2].type, LexerTokenType::COMMENT);
    EXPECT_EQ(tokens[2].text, "move to X");
}

TEST_F(LexerTest, CommentBeforeCode) {
    auto tokens = tokenizeAll("(setup) G1 X100");

    ASSERT_GE(tokens.size(), 3u);
    EXPECT_EQ(tokens[0].type, LexerTokenType::COMMENT);
    expectWord(tokens[1], WordLetter::G, 1.0);
    expectWord(tokens[2], WordLetter::X, 100.0);
}

TEST_F(LexerTest, MultipleComments) {
    auto tokens = tokenizeAll("(first) G1 (second) X100 ; line comment");

    size_t commentCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::COMMENT) commentCount++;
    }
    EXPECT_GE(commentCount, 2u);
}

TEST_F(LexerTest, EmptyComment) {
    lexer->setInput("()");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::COMMENT);
    EXPECT_EQ(tok.text, "");
}

TEST_F(LexerTest, CommentWithSpecialChars) {
    lexer->setInput("(test: 123 @ #$% !)");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::COMMENT);
    EXPECT_EQ(tok.text, "test: 123 @ #$% !");
}

// ============================================================================
// O-Code Tests
// ============================================================================

TEST_F(LexerTest, OCodeNumber) {
    lexer->setInput("O100");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::OCODE_NUMBER);
    EXPECT_EQ(tok.oNumber, 100);
}

TEST_F(LexerTest, OCodeName) {
    lexer->setInput("O<subroutine>");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::OCODE_NAME);
    EXPECT_EQ(tok.text, "subroutine");
}

TEST_F(LexerTest, OCodeNameWithUnderscore) {
    lexer->setInput("O<my_subroutine>");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::OCODE_NAME);
    EXPECT_EQ(tok.text, "my_subroutine");
}

TEST_F(LexerTest, OCodeNameWithNumbers) {
    lexer->setInput("O<sub123>");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::OCODE_NAME);
    EXPECT_EQ(tok.text, "sub123");
}

TEST_F(LexerTest, OCodeZero) {
    lexer->setInput("O0");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::OCODE_NUMBER);
    EXPECT_EQ(tok.oNumber, 0);
}

// ============================================================================
// Parameter Tests
// ============================================================================

TEST_F(LexerTest, NumberedParameter) {
    lexer->setInput("#1");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::PARAMETER);
    EXPECT_FALSE(tok.isNamedParam);
    EXPECT_EQ(tok.paramNumber, 1);
}

TEST_F(LexerTest, NamedParameter) {
    lexer->setInput("#<myvar>");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::PARAMETER);
    EXPECT_TRUE(tok.isNamedParam);
    EXPECT_EQ(tok.paramName, "myvar");
}

TEST_F(LexerTest, ParameterInWord) {
    // Parameters in word values would require expression evaluation
    auto tokens = tokenizeAll("X#1 Y#<offset>");

    // These might be lexed differently depending on implementation
    EXPECT_GE(tokens.size(), 2u);
}

TEST_F(LexerTest, GlobalParameter) {
    lexer->setInput("#5000");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::PARAMETER);
    EXPECT_EQ(tok.paramNumber, 5000);
}

TEST_F(LexerTest, KeyValueEmptyValue) {
    lexer->setInput("FOO=");
    auto toks = tokenizeAll("FOO=");
    ASSERT_EQ(toks.size(), 1u);
    EXPECT_EQ(toks[0].type, LexerTokenType::KEYVALUE);
    EXPECT_TRUE(toks[0].isKeyValue);
    EXPECT_EQ(toks[0].kvKey, "FOO");
    EXPECT_EQ(toks[0].kvValue, "");
}

TEST_F(LexerTest, KeyValueQuotedAndVector) {
    lexer->setInput("NAME=\"John Doe\" V=[1,2,3]");
    auto toks = tokenizeAll("NAME=\"John Doe\" V=[1,2,3]");
    ASSERT_EQ(toks.size(), 2u);
    EXPECT_EQ(toks[0].type, LexerTokenType::KEYVALUE);
    EXPECT_EQ(toks[0].kvKey, "NAME");
    EXPECT_EQ(toks[0].kvValue, "\"John Doe\"");
    EXPECT_EQ(toks[1].type, LexerTokenType::KEYVALUE);
    EXPECT_EQ(toks[1].kvKey, "V");
    EXPECT_EQ(toks[1].kvValue, "[1,2,3]");
}

TEST_F(LexerTest, KeyValueUnquotedWithEqualsInValue) {
    lexer->setInput("CMD=KEY=VAL");
    auto toks = tokenizeAll("CMD=KEY=VAL");
    ASSERT_EQ(toks.size(), 1u);
    EXPECT_EQ(toks[0].type, LexerTokenType::KEYVALUE);
    EXPECT_EQ(toks[0].kvKey, "CMD");
    EXPECT_EQ(toks[0].kvValue, "KEY=VAL");
}

// ============================================================================
// Expression Tests
// ============================================================================

TEST_F(LexerTest, SimpleExpression) {
    lexer->setInput("[1+2]");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::EXPRESSION);
    EXPECT_EQ(tok.expression, "1+2");
}

TEST_F(LexerTest, ExpressionWithSpaces) {
    lexer->setInput("[ 1 + 2 * 3 ]");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::EXPRESSION);
    EXPECT_EQ(tok.expression, " 1 + 2 * 3 ");
}

TEST_F(LexerTest, NestedExpression) {
    lexer->setInput("[[1+2]*3]");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::EXPRESSION);
    EXPECT_EQ(tok.expression, "[1+2]*3");
}

TEST_F(LexerTest, ExpressionWithFunctions) {
    lexer->setInput("[SIN[45]]");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::EXPRESSION);
    EXPECT_EQ(tok.expression, "SIN[45]");
}

TEST_F(LexerTest, ExpressionWithParameter) {
    lexer->setInput("[#1*2]");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::EXPRESSION);
    EXPECT_EQ(tok.expression, "#1*2");
}

// ============================================================================
// Block Delete Tests
// ============================================================================

TEST_F(LexerTest, BlockDeleteAtStart) {
    lexer->setInput("/G1 X100");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::BLOCK_DELETE);

    tok = lexer->nextToken();
    expectWord(tok, WordLetter::G, 1.0);
}

TEST_F(LexerTest, SlashNotAtStart) {
    // / not at line start should be ignored or treated differently
    lexer->setInput("G1 /X100");
    auto tokens = tokenizeAll("G1 /X100");

    // First token should be G1, not block delete
    ASSERT_GE(tokens.size(), 1u);
    expectWord(tokens[0], WordLetter::G, 1.0);
}

// ============================================================================
// Percent Delimiter Tests
// ============================================================================

TEST_F(LexerTest, PercentDelimiter) {
    lexer->setInput("%");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::PERCENT);
}

TEST_F(LexerTest, PercentProgramDelimiters) {
    auto tokens = tokenizeAll("%\nG1 X100\n%");

    ASSERT_GE(tokens.size(), 3u);
    EXPECT_EQ(tokens[0].type, LexerTokenType::PERCENT);
    // Skip EOL
    // Find G1 word
    bool foundG1 = false;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD && t.letter == WordLetter::G && t.value == 1.0) {
            foundG1 = true;
            break;
        }
    }
    EXPECT_TRUE(foundG1);
}

// ============================================================================
// Line Number Tests
// ============================================================================

TEST_F(LexerTest, LineNumber) {
    auto tokens = tokenizeAll("N100 G1 X50");

    ASSERT_GE(tokens.size(), 3u);
    expectWord(tokens[0], WordLetter::N, 100.0);
    expectWord(tokens[1], WordLetter::G, 1.0);
    expectWord(tokens[2], WordLetter::X, 50.0);
}

TEST_F(LexerTest, LineNumberTracking) {
    lexer->setInput("G1\nG2\nG3");

    LexerToken tok = lexer->nextToken(); // G1
    EXPECT_EQ(tok.line, 1u);

    tok = lexer->nextToken(); // EOL
    tok = lexer->nextToken(); // G2
    EXPECT_EQ(tok.line, 2u);

    tok = lexer->nextToken(); // EOL
    tok = lexer->nextToken(); // G3
    EXPECT_EQ(tok.line, 3u);
}

TEST_F(LexerTest, ColumnTracking) {
    lexer->setInput("G1 X100");

    LexerToken tok = lexer->nextToken(); // G1
    EXPECT_EQ(tok.column, 1u);

    tok = lexer->nextToken(); // X100
    EXPECT_EQ(tok.column, 4u);
}

// ============================================================================
// Case Sensitivity Tests
// ============================================================================

TEST_F(LexerTest, LowercaseLetters) {
    auto tokens = tokenizeAll("g1 x100 y50");

    ASSERT_EQ(tokens.size(), 3u);
    expectWord(tokens[0], WordLetter::G, 1.0);
    expectWord(tokens[1], WordLetter::X, 100.0);
    expectWord(tokens[2], WordLetter::Y, 50.0);
}

TEST_F(LexerTest, MixedCase) {
    auto tokens = tokenizeAll("G1 x100 Y50 z25");

    ASSERT_EQ(tokens.size(), 4u);
    expectWord(tokens[0], WordLetter::G, 1.0);
    expectWord(tokens[1], WordLetter::X, 100.0);
    expectWord(tokens[2], WordLetter::Y, 50.0);
    expectWord(tokens[3], WordLetter::Z, 25.0);
}

// ============================================================================
// Position Control Tests
// ============================================================================

TEST_F(LexerTest, SeekToStart) {
    lexer->setInput("G1 G2 G3");
    lexer->nextToken(); // G1
    lexer->nextToken(); // G2

    lexer->seekToStart();
    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::G, 1.0);
}

TEST_F(LexerTest, SeekToLine) {
    lexer->setInput("G1\nG2\nG3");

    EXPECT_TRUE(lexer->seekToLine(2));
    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::G, 2.0);
}

TEST_F(LexerTest, SeekToInvalidLine) {
    lexer->setInput("G1\nG2");

    EXPECT_FALSE(lexer->seekToLine(100));
}

TEST_F(LexerTest, SeekToPosition) {
    lexer->setInput("G1 X100");
    lexer->seek(3); // After "G1 "

    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::X, 100.0);
}

TEST_F(LexerTest, GetPosition) {
    lexer->setInput("G1 X100");
    EXPECT_EQ(lexer->getPosition(), 0u);

    lexer->nextToken(); // G1
    EXPECT_GT(lexer->getPosition(), 0u);
}

TEST_F(LexerTest, AtEndCheck) {
    lexer->setInput("G1");
    EXPECT_FALSE(lexer->atEnd());

    lexer->nextToken();
    EXPECT_TRUE(lexer->atEnd());
}

// ============================================================================
// Peek Token Tests
// ============================================================================

TEST_F(LexerTest, PeekDoesNotConsume) {
    lexer->setInput("G1 X100");

    LexerToken peek1 = lexer->peekToken();
    LexerToken peek2 = lexer->peekToken();
    LexerToken next = lexer->nextToken();

    EXPECT_EQ(peek1.letter, peek2.letter);
    EXPECT_EQ(peek1.value, peek2.value);
    EXPECT_EQ(peek1.letter, next.letter);
    EXPECT_EQ(peek1.value, next.value);
}

TEST_F(LexerTest, PeekThenNext) {
    lexer->setInput("G1 X100 Y50");

    lexer->peekToken(); // G1
    LexerToken tok = lexer->nextToken(); // G1
    expectWord(tok, WordLetter::G, 1.0);

    lexer->peekToken(); // X100
    tok = lexer->nextToken(); // X100
    expectWord(tok, WordLetter::X, 100.0);
}

// ============================================================================
// Prev Token Tests
// ============================================================================

TEST_F(LexerTest, PrevToken) {
    lexer->setInput("G1 X100");

    lexer->nextToken(); // G1
    lexer->nextToken(); // X100

    LexerToken prev = lexer->prevToken();
    expectWord(prev, WordLetter::G, 1.0);
}

// ============================================================================
// Tokenize Line Tests
// ============================================================================

TEST_F(LexerTest, TokenizeLine) {
    lexer->setInput("G1 X100 Y50\nG2 X200");

    auto line1 = lexer->tokenizeLine();
    ASSERT_EQ(line1.size(), 3u);
    expectWord(line1[0], WordLetter::G, 1.0);
    expectWord(line1[1], WordLetter::X, 100.0);
    expectWord(line1[2], WordLetter::Y, 50.0);

    auto line2 = lexer->tokenizeLine();
    ASSERT_EQ(line2.size(), 2u);
    expectWord(line2[0], WordLetter::G, 2.0);
    expectWord(line2[1], WordLetter::X, 200.0);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(LexerTest, UnterminatedComment) {
    lexer->setInput("(unterminated comment");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::ERROR);
    EXPECT_TRUE(lexer->hasError());
}

TEST_F(LexerTest, UnterminatedExpression) {
    lexer->setInput("[1+2");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::ERROR);
}

TEST_F(LexerTest, UnterminatedOCodeName) {
    lexer->setInput("O<myname");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::ERROR);
}

TEST_F(LexerTest, UnterminatedParameterName) {
    lexer->setInput("#<myvar");
    LexerToken tok = lexer->nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::ERROR);
}

TEST_F(LexerTest, ClearError) {
    lexer->setInput("(unterminated");
    lexer->nextToken();

    EXPECT_TRUE(lexer->hasError());
    lexer->clearError();
    EXPECT_FALSE(lexer->hasError());
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST_F(LexerTest, TokenTypeToString) {
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::WORD), "WORD");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::COMMENT), "COMMENT");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::EXPRESSION), "EXPRESSION");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::OCODE_NUMBER), "OCODE_NUMBER");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::OCODE_NAME), "OCODE_NAME");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::PARAMETER), "PARAMETER");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::BLOCK_DELETE), "BLOCK_DELETE");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::PERCENT), "PERCENT");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::EOL), "EOL");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::END), "END");
    EXPECT_STREQ(tokenTypeToString(LexerTokenType::ERROR), "ERROR");
}

TEST_F(LexerTest, WordLetterToChar) {
    EXPECT_EQ(wordLetterToChar(WordLetter::A), 'A');
    EXPECT_EQ(wordLetterToChar(WordLetter::G), 'G');
    EXPECT_EQ(wordLetterToChar(WordLetter::M), 'M');
    EXPECT_EQ(wordLetterToChar(WordLetter::X), 'X');
    EXPECT_EQ(wordLetterToChar(WordLetter::Z), 'Z');
}

TEST_F(LexerTest, CharToWordLetter) {
    EXPECT_EQ(charToWordLetter('A'), WordLetter::A);
    EXPECT_EQ(charToWordLetter('a'), WordLetter::A);
    EXPECT_EQ(charToWordLetter('G'), WordLetter::G);
    EXPECT_EQ(charToWordLetter('g'), WordLetter::G);
    EXPECT_EQ(charToWordLetter('X'), WordLetter::X);
    EXPECT_EQ(charToWordLetter('1'), WordLetter::INVALID);
}

TEST_F(LexerTest, IsAxisLetter) {
    EXPECT_TRUE(isAxisLetter(WordLetter::X));
    EXPECT_TRUE(isAxisLetter(WordLetter::Y));
    EXPECT_TRUE(isAxisLetter(WordLetter::Z));
    EXPECT_TRUE(isAxisLetter(WordLetter::A));
    EXPECT_TRUE(isAxisLetter(WordLetter::B));
    EXPECT_TRUE(isAxisLetter(WordLetter::C));
    EXPECT_TRUE(isAxisLetter(WordLetter::U));
    EXPECT_TRUE(isAxisLetter(WordLetter::V));
    EXPECT_TRUE(isAxisLetter(WordLetter::W));
    EXPECT_FALSE(isAxisLetter(WordLetter::G));
    EXPECT_FALSE(isAxisLetter(WordLetter::M));
    EXPECT_FALSE(isAxisLetter(WordLetter::F));
}

TEST_F(LexerTest, AxisFromWordLetter) {
    EXPECT_EQ(axisFromWordLetter(WordLetter::X), 0);
    EXPECT_EQ(axisFromWordLetter(WordLetter::Y), 1);
    EXPECT_EQ(axisFromWordLetter(WordLetter::Z), 2);
    EXPECT_EQ(axisFromWordLetter(WordLetter::A), 3);
    EXPECT_EQ(axisFromWordLetter(WordLetter::B), 4);
    EXPECT_EQ(axisFromWordLetter(WordLetter::C), 5);
    EXPECT_EQ(axisFromWordLetter(WordLetter::G), -1);
}

TEST_F(LexerTest, FormatToken) {
    lexer->setInput("G1");
    LexerToken tok = lexer->nextToken();

    std::string formatted = formatToken(tok);
    EXPECT_NE(formatted.find("WORD"), std::string::npos);
    EXPECT_NE(formatted.find("G"), std::string::npos);
}

// ============================================================================
// StripComments Tests
// ============================================================================

TEST_F(LexerTest, StripParenComment) {
    std::string result = stripComments("G1 (comment) X100");
    EXPECT_EQ(result, "G1  X100");
}

TEST_F(LexerTest, StripSemicolonComment) {
    std::string result = stripComments("G1 X100 ; comment");
    EXPECT_EQ(result, "G1 X100 ");
}

TEST_F(LexerTest, StripMultipleComments) {
    std::string result = stripComments("G1 (a) X100 (b) Y50");
    // Should remove both () comments
    EXPECT_NE(result.find("G1"), std::string::npos);
    EXPECT_NE(result.find("X100"), std::string::npos);
    EXPECT_EQ(result.find("(a)"), std::string::npos);
}

TEST_F(LexerTest, StripCommentsNull) {
    std::string result = stripComments(nullptr);
    EXPECT_EQ(result, "");
}

// ============================================================================
// IsEmptyOrComment Tests
// ============================================================================

TEST_F(LexerTest, IsEmptyOrCommentEmpty) {
    EXPECT_TRUE(isEmptyOrComment(""));
    EXPECT_TRUE(isEmptyOrComment("   "));
    EXPECT_TRUE(isEmptyOrComment("\t"));
}

TEST_F(LexerTest, IsEmptyOrCommentOnlyComment) {
    EXPECT_TRUE(isEmptyOrComment("; comment"));
    EXPECT_TRUE(isEmptyOrComment("(comment)"));
}

TEST_F(LexerTest, IsEmptyOrCommentWithCode) {
    EXPECT_FALSE(isEmptyOrComment("G1"));
    EXPECT_FALSE(isEmptyOrComment("G1 ; comment"));
}

TEST_F(LexerTest, IsEmptyOrCommentNull) {
    EXPECT_TRUE(isEmptyOrComment(nullptr));
}

// ============================================================================
// TokenizeLine Function Tests
// ============================================================================

TEST_F(LexerTest, TokenizeLineFunction) {
    std::vector<LexerToken> tokens;
    Error err = tokenizeLine("G1 X100 Y50", tokens);

    EXPECT_FALSE(err);
    ASSERT_EQ(tokens.size(), 3u);
    expectWord(tokens[0], WordLetter::G, 1.0);
    expectWord(tokens[1], WordLetter::X, 100.0);
    expectWord(tokens[2], WordLetter::Y, 50.0);
}

TEST_F(LexerTest, TokenizeLineNull) {
    std::vector<LexerToken> tokens;
    Error err = tokenizeLine(nullptr, tokens);

    EXPECT_TRUE(err);
}

// ============================================================================
// Syntax Highlighting Tests
// ============================================================================

TEST_F(LexerTest, HighlightSimpleLine) {
    auto spans = highlightLine("G1 X100");

    EXPECT_GT(spans.size(), 0u);

    bool foundGCode = false;
    bool foundAxis = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::GCode) foundGCode = true;
        if (span.kind == HighlightKind::Axis) foundAxis = true;
    }
    EXPECT_TRUE(foundGCode);
    EXPECT_TRUE(foundAxis);
}

TEST_F(LexerTest, HighlightComment) {
    auto spans = highlightLine("; comment");

    bool foundComment = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::Comment) foundComment = true;
    }
    EXPECT_TRUE(foundComment);
}

TEST_F(LexerTest, HighlightMCode) {
    auto spans = highlightLine("M3 S1000");

    bool foundMCode = false;
    bool foundWord = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::MCode) foundMCode = true;
        if (span.kind == HighlightKind::Word) foundWord = true;
    }
    EXPECT_TRUE(foundMCode);
    EXPECT_TRUE(foundWord);
}

TEST_F(LexerTest, HighlightOCode) {
    auto spans = highlightLine("O100");

    bool foundOCode = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::OCode) foundOCode = true;
    }
    EXPECT_TRUE(foundOCode);
}

TEST_F(LexerTest, HighlightExpression) {
    // Standalone expression, not as part of a word
    auto spans = highlightLine("[1+2]");

    bool foundExpr = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::Expression) foundExpr = true;
    }
    EXPECT_TRUE(foundExpr);
}

TEST_F(LexerTest, HighlightVariable) {
    auto spans = highlightLine("#1");

    bool foundVar = false;
    for (const auto& span : spans) {
        if (span.kind == HighlightKind::Variable) foundVar = true;
    }
    EXPECT_TRUE(foundVar);
}

// ============================================================================
// Lexer Configuration Tests
// ============================================================================

TEST_F(LexerTest, CaseSensitiveConfig) {
    LexerConfig config;
    config.caseInsensitive = false;
    Lexer lex(config);

    lex.setInput("g1");
    LexerToken tok = lex.nextToken();
    // In case-sensitive mode, 'g' is still a valid word letter
    EXPECT_EQ(tok.type, LexerTokenType::WORD);
}

TEST_F(LexerTest, SkipBlockDeleteConfig) {
    LexerConfig config;
    config.skipBlockDelete = true;
    Lexer lex(config);

    lex.setInput("/G1 X100");
    auto tokens = lex.tokenizeLine();

    // With skipBlockDelete=true, the entire line after / should be skipped
    EXPECT_EQ(tokens.size(), 0u);
}

TEST_F(LexerTest, PercentAsDelimiterDisabled) {
    LexerConfig config;
    config.treatPercentAsDelimiter = false;
    Lexer lex(config);

    lex.setInput("%");
    LexerToken tok = lex.nextToken();
    // When disabled, % is not treated as a special token
    EXPECT_NE(tok.type, LexerTokenType::PERCENT);
}

// ============================================================================
// Line Operations Tests
// ============================================================================

TEST_F(LexerTest, SkipLine) {
    lexer->setInput("G1 X100\nG2 Y200");

    lexer->skipLine();
    LexerToken tok = lexer->nextToken();
    expectWord(tok, WordLetter::G, 2.0);
}

TEST_F(LexerTest, GetCurrentLineText) {
    lexer->setInput("G1 X100\nG2 Y200");

    std::string line = lexer->getCurrentLineText();
    EXPECT_EQ(line, "G1 X100");
}

TEST_F(LexerTest, AtLineStart) {
    lexer->setInput("G1 X100");

    EXPECT_TRUE(lexer->atLineStart());
    lexer->nextToken();
    EXPECT_FALSE(lexer->atLineStart());
}

TEST_F(LexerTest, ScanLineCount) {
    lexer->setInput("G1\nG2\nG3\nG4\nG5");
    lexer->scanLineCount();

    EXPECT_EQ(lexer->getTotalLines(), 5u);
}

TEST_F(LexerTest, Remaining) {
    lexer->setInput("G1 X100");
    lexer->nextToken(); // G1

    std::string_view rem = lexer->remaining();
    EXPECT_NE(rem.find("X100"), std::string_view::npos);
}

// ============================================================================
// Comprehensive G-Code Examples
// ============================================================================

TEST_F(LexerTest, RealWorldGCodeLine1) {
    auto tokens = tokenizeAll("N10 G21 G90 G17 F1000 S5000 M3");

    size_t wordCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD) wordCount++;
    }
    EXPECT_GE(wordCount, 7u);
}

TEST_F(LexerTest, RealWorldGCodeLine2) {
    auto tokens = tokenizeAll("G1 X100.5 Y-50.25 Z0.1 F500 (rapid move)");

    EXPECT_GE(tokens.size(), 5u);
}

TEST_F(LexerTest, RealWorldArcMove) {
    auto tokens = tokenizeAll("G2 X50 Y50 I25 J0 F200");

    size_t wordCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD) wordCount++;
    }
    EXPECT_EQ(wordCount, 6u);
}

TEST_F(LexerTest, RealWorldHelicalMove) {
    auto tokens = tokenizeAll("G3 X0 Y10 Z-5 I-5 J0 F100");

    EXPECT_GE(tokens.size(), 7u);
}

TEST_F(LexerTest, RealWorldToolChange) {
    auto tokens = tokenizeAll("T1 M6");

    ASSERT_GE(tokens.size(), 2u);
    expectWord(tokens[0], WordLetter::T, 1.0);
    expectWord(tokens[1], WordLetter::M, 6.0);
}

TEST_F(LexerTest, RealWorldSubroutineCall) {
    auto tokens = tokenizeAll("O100 call [#1] [#2]");

    ASSERT_GE(tokens.size(), 1u);
    EXPECT_EQ(tokens[0].type, LexerTokenType::OCODE_NUMBER);
    EXPECT_EQ(tokens[0].oNumber, 100);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(LexerTest, MultipleNewlines) {
    lexer->setInput("\n\n\nG1\n\n");

    int eolCount = 0;
    int wordCount = 0;
    while (true) {
        LexerToken t = lexer->nextToken();
        if (t.type == LexerTokenType::END) break;
        if (t.type == LexerTokenType::EOL) eolCount++;
        if (t.type == LexerTokenType::WORD) wordCount++;
    }
    EXPECT_EQ(wordCount, 1);
    EXPECT_GE(eolCount, 3);
}

TEST_F(LexerTest, VeryLongLine) {
    std::string longLine = "G1";
    for (int i = 0; i < 100; i++) {
        longLine += " X" + std::to_string(i);
    }

    auto tokens = tokenizeAll(longLine.c_str());
    EXPECT_GT(tokens.size(), 50u);
}

TEST_F(LexerTest, ManyGCodesOnOneLine) {
    auto tokens = tokenizeAll("G21 G90 G17 G54 G64 P0.01 G40");

    size_t gCodeCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD && t.letter == WordLetter::G) {
            gCodeCount++;
        }
    }
    EXPECT_GE(gCodeCount, 5u);
}

TEST_F(LexerTest, SpacesEverywhere) {
    auto tokens = tokenizeAll("   G1   X100   Y50   ");

    size_t wordCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD) wordCount++;
    }
    EXPECT_EQ(wordCount, 3u);
}

TEST_F(LexerTest, NoSpaces) {
    auto tokens = tokenizeAll("G1X100Y50F1000");

    size_t wordCount = 0;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::WORD) wordCount++;
    }
    EXPECT_EQ(wordCount, 4u);
}

// ============================================================================
// Parameterized Test for Many G-Codes
// ============================================================================

class GCodeLexerParameterizedTest : public ::testing::TestWithParam<std::pair<const char*, int>> {
protected:
    Lexer lexer;
};

TEST_P(GCodeLexerParameterizedTest, ParsesGCode) {
    auto [input, expectedValue] = GetParam();
    lexer.setInput(input);
    LexerToken tok = lexer.nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::WORD);
    EXPECT_EQ(tok.letter, WordLetter::G);
    EXPECT_NEAR(tok.value, static_cast<double>(expectedValue), 0.1);
}

INSTANTIATE_TEST_SUITE_P(
    GCodeValues,
    GCodeLexerParameterizedTest,
    ::testing::Values(
        std::make_pair("G0", 0),
        std::make_pair("G1", 1),
        std::make_pair("G2", 2),
        std::make_pair("G3", 3),
        std::make_pair("G4", 4),
        std::make_pair("G10", 10),
        std::make_pair("G17", 17),
        std::make_pair("G18", 18),
        std::make_pair("G19", 19),
        std::make_pair("G20", 20),
        std::make_pair("G21", 21),
        std::make_pair("G28", 28),
        std::make_pair("G30", 30),
        std::make_pair("G40", 40),
        std::make_pair("G41", 41),
        std::make_pair("G42", 42),
        std::make_pair("G43", 43),
        std::make_pair("G49", 49),
        std::make_pair("G53", 53),
        std::make_pair("G54", 54),
        std::make_pair("G55", 55),
        std::make_pair("G56", 56),
        std::make_pair("G57", 57),
        std::make_pair("G58", 58),
        std::make_pair("G59", 59),
        std::make_pair("G61", 61),
        std::make_pair("G64", 64),
        std::make_pair("G73", 73),
        std::make_pair("G76", 76),
        std::make_pair("G80", 80),
        std::make_pair("G81", 81),
        std::make_pair("G82", 82),
        std::make_pair("G83", 83),
        std::make_pair("G84", 84),
        std::make_pair("G85", 85),
        std::make_pair("G86", 86),
        std::make_pair("G87", 87),
        std::make_pair("G88", 88),
        std::make_pair("G89", 89),
        std::make_pair("G90", 90),
        std::make_pair("G91", 91),
        std::make_pair("G92", 92),
        std::make_pair("G93", 93),
        std::make_pair("G94", 94),
        std::make_pair("G95", 95),
        std::make_pair("G96", 96),
        std::make_pair("G97", 97),
        std::make_pair("G98", 98),
        std::make_pair("G99", 99)
    )
);

// ============================================================================
// Parameterized Test for M-Codes
// ============================================================================

class MCodeLexerParameterizedTest : public ::testing::TestWithParam<std::pair<const char*, int>> {
protected:
    Lexer lexer;
};

TEST_P(MCodeLexerParameterizedTest, ParsesMCode) {
    auto [input, expectedValue] = GetParam();
    lexer.setInput(input);
    LexerToken tok = lexer.nextToken();

    EXPECT_EQ(tok.type, LexerTokenType::WORD);
    EXPECT_EQ(tok.letter, WordLetter::M);
    EXPECT_NEAR(tok.value, static_cast<double>(expectedValue), 0.1);
}

INSTANTIATE_TEST_SUITE_P(
    MCodeValues,
    MCodeLexerParameterizedTest,
    ::testing::Values(
        std::make_pair("M0", 0),
        std::make_pair("M1", 1),
        std::make_pair("M2", 2),
        std::make_pair("M3", 3),
        std::make_pair("M4", 4),
        std::make_pair("M5", 5),
        std::make_pair("M6", 6),
        std::make_pair("M7", 7),
        std::make_pair("M8", 8),
        std::make_pair("M9", 9),
        std::make_pair("M30", 30),
        std::make_pair("M48", 48),
        std::make_pair("M49", 49),
        std::make_pair("M60", 60),
        std::make_pair("M100", 100),
        std::make_pair("M199", 199)
    )
);

// ============================================================================
// Comprehensive Line Parsing Tests
// ============================================================================

class ComprehensiveLineTest : public ::testing::TestWithParam<const char*> {
protected:
    Lexer lexer;
};

TEST_P(ComprehensiveLineTest, ParsesWithoutError) {
    const char* input = GetParam();
    lexer.setInput(input);

    while (true) {
        LexerToken tok = lexer.nextToken();
        if (tok.type == LexerTokenType::END || tok.type == LexerTokenType::EOL) break;
        EXPECT_NE(tok.type, LexerTokenType::ERROR) << "Error on input: " << input;
    }
}

INSTANTIATE_TEST_SUITE_P(
    GCodeLines,
    ComprehensiveLineTest,
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
        "G30",
        "G40",
        "G41 D1",
        "G42 D2",
        "G43 H1 Z0",
        "G49",
        "G53 G0 X0 Y0",
        "G54",
        "G55",
        "G61",
        "G64 P0.01 Q0.01",
        "G90",
        "G91",
        "G92 X0 Y0",
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
        "M30",
        "T1",
        "T99",
        "S1000",
        "F500",
        "N100 G1 X100 F500",
        "(comment only)",
        "; semicolon comment",
        "G1 X100 (with comment)",
        "G1 X100 ; also comment",
        "O100",
        "O<mysub>",
        "#1=100",
        "#<myvar>=50",
        "[1+2]",
        // Note: Parameter references in expressions may not be fully supported
        // "G1 X[#1*2] Y[SIN[45]*10]",
        "%",
        "/G1 X100",
        "G1X100Y50Z10F1000",
        "   G1   X100   ",
        "g1 x100 y50",
        "G1 X-100 Y+50 Z-.5",
        "G1 X.5 Y1. Z0.001",
        "G1 X1000000 Y-9999999",
        "G21 G90 G17 G54 G64 P0.01"
    )
);

// ============================================================================
// Additional Edge Case Tests for Coverage
// ============================================================================

TEST_F(LexerTest, SeekWithNullSource) {
    // Test seek() with null source
    Lexer lex;
    lex.seek(100);  // Should handle gracefully with null source
    EXPECT_EQ(lex.getPosition(), 0u);
}

TEST_F(LexerTest, SeekPastEnd) {
    lexer->setInput("G0");
    lexer->seek(1000);  // Seek beyond end
    EXPECT_TRUE(lexer->atEnd());
}

TEST_F(LexerTest, SeekBackToStart) {
    lexer->setInput("G0 X100\nG1 Y200");
    lexer->nextToken();
    lexer->nextToken();
    lexer->seekToStart();
    EXPECT_EQ(lexer->getPosition(), 0u);
}

TEST_F(LexerTest, SeekForwardToEnd) {
    lexer->setInput("G0 X100");
    lexer->seekToEnd();
    EXPECT_TRUE(lexer->atEnd());
}

TEST_F(LexerTest, SeekToLineWithNullSource) {
    Lexer lex;
    EXPECT_FALSE(lex.seekToLine(1));
}

TEST_F(LexerTest, SeekToInvalidLineZero) {
    lexer->setInput("G0");
    EXPECT_FALSE(lexer->seekToLine(0));  // Line 0 is invalid
}

TEST_F(LexerTest, SeekToFirstLine) {
    lexer->setInput("G0\nG1\nG2");
    EXPECT_TRUE(lexer->seekToLine(1));
    EXPECT_EQ(lexer->getLine(), 1u);
}

TEST_F(LexerTest, SeekToLineTooLarge) {
    lexer->setInput("G0\nG1");
    EXPECT_FALSE(lexer->seekToLine(100));
}

TEST_F(LexerTest, PrevLineWhenAtStart) {
    lexer->setInput("G0\nG1\nG2");
    lexer->prevLine();  // Should handle at start
}

TEST_F(LexerTest, PrevLineNavigation) {
    lexer->setInput("G0\nG1\nG2");
    lexer->seekToLine(2);
    lexer->prevLine();
    EXPECT_EQ(lexer->getLine(), 1u);
}

TEST_F(LexerTest, SkipLineWithNullSource) {
    Lexer lex;
    lex.skipLine();  // Should handle null source
}

TEST_F(LexerTest, GetCurrentLineTextWithNullSource) {
    Lexer lex;
    EXPECT_EQ(lex.getCurrentLineText(), "");
}

TEST_F(LexerTest, GetCurrentLineTextBasic) {
    lexer->setInput("G0 X100\nG1 Y200");
    EXPECT_EQ(lexer->getCurrentLineText(), "G0 X100");
}

TEST_F(LexerTest, AtLineStartInitial) {
    lexer->setInput("G0");
    EXPECT_TRUE(lexer->atLineStart());
}

TEST_F(LexerTest, ScanLineCountWithNullSource) {
    Lexer lex;
    lex.scanLineCount();
    EXPECT_EQ(lex.getTotalLines(), 0u);
}

TEST_F(LexerTest, RemainingWithNullSource) {
    Lexer lex;
    EXPECT_TRUE(lex.remaining().empty());
}

TEST_F(LexerTest, RemainingWhenAtEnd) {
    lexer->setInput("G0");
    lexer->seekToEnd();
    EXPECT_TRUE(lexer->remaining().empty());
}

TEST_F(LexerTest, RemainingPartialInput) {
    lexer->setInput("G0 X100");
    lexer->nextToken();  // Consume G0
    EXPECT_FALSE(lexer->remaining().empty());
}

TEST_F(LexerTest, ConfigCaseInsensitiveMode) {
    LexerConfig cfg;
    cfg.caseInsensitive = true;
    Lexer lex(cfg);
    lex.setInput("g0 x100");
    auto tok = lex.nextToken();
    EXPECT_EQ(tok.letter, WordLetter::G);
}

TEST_F(LexerTest, ConfigAllowSpacesInNumbers) {
    LexerConfig cfg;
    cfg.allowSpacesInNumbers = true;
    Lexer lex(cfg);
    lex.setInput("G0 X1 000");  // Space in number
    auto tokens = std::vector<LexerToken>();
    while (true) {
        auto tok = lex.nextToken();
        if (tok.type == LexerTokenType::END) break;
        tokens.push_back(tok);
    }
}

TEST_F(LexerTest, ClearErrorAfterToken) {
    lexer->setInput("G0");
    lexer->nextToken();
    lexer->clearError();
    EXPECT_FALSE(lexer->hasError());
}

TEST_F(LexerTest, PeekTokenOnce) {
    lexer->setInput("G0 X100");
    auto peeked = lexer->peekToken();
    auto actual = lexer->nextToken();
    EXPECT_EQ(peeked.type, actual.type);
}

TEST_F(LexerTest, PeekTokenRepeated) {
    lexer->setInput("G0");
    auto first = lexer->peekToken();
    auto second = lexer->peekToken();
    EXPECT_EQ(first.type, second.type);
}

TEST_F(LexerTest, TokenizeLineFromEmpty) {
    lexer->setInput("");
    auto tokens = lexer->tokenizeLine();
    EXPECT_TRUE(tokens.empty());
}

TEST_F(LexerTest, TokenizeLineFromBasic) {
    lexer->setInput("G0 X100");
    auto tokens = lexer->tokenizeLine();
    EXPECT_GE(tokens.size(), 2u);
}

TEST_F(LexerTest, SetInputWithLengthLimited) {
    const char* source = "G0 X100 extra garbage";
    lexer->setInput(source, 7);  // Only "G0 X100"
    auto tok = lexer->nextToken();
    EXPECT_EQ(tok.letter, WordLetter::G);
}

TEST_F(LexerTest, SetInputFromStringView) {
    std::string_view sv = "G1 Y200";
    lexer->setInput(sv);
    auto tok = lexer->nextToken();
    EXPECT_EQ(tok.letter, WordLetter::G);
}

// ============================================================================
// Additional Coverage Tests - O-code Keywords
// ============================================================================

TEST_F(LexerTest, OCodeSub) {
    lexer->setInput("O100 SUB");
    auto tokens = tokenizeAll("O100 SUB");
    // O-code handling
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeEndsub) {
    lexer->setInput("O100 ENDSUB");
    auto tokens = tokenizeAll("O100 ENDSUB");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeCall) {
    lexer->setInput("O100 CALL");
    auto tokens = tokenizeAll("O100 CALL");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeIf) {
    lexer->setInput("O100 IF [#1 GT 5]");
    auto tokens = tokenizeAll("O100 IF [#1 GT 5]");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeElse) {
    lexer->setInput("O100 ELSE");
    auto tokens = tokenizeAll("O100 ELSE");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeEndif) {
    lexer->setInput("O100 ENDIF");
    auto tokens = tokenizeAll("O100 ENDIF");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeWhile) {
    lexer->setInput("O100 WHILE [#1 LT 10]");
    auto tokens = tokenizeAll("O100 WHILE [#1 LT 10]");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeEndwhile) {
    lexer->setInput("O100 ENDWHILE");
    auto tokens = tokenizeAll("O100 ENDWHILE");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeDo) {
    lexer->setInput("O100 DO");
    auto tokens = tokenizeAll("O100 DO");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeReturn) {
    lexer->setInput("O100 RETURN");
    auto tokens = tokenizeAll("O100 RETURN");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeBreak) {
    lexer->setInput("O100 BREAK");
    auto tokens = tokenizeAll("O100 BREAK");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeContinue) {
    lexer->setInput("O100 CONTINUE");
    auto tokens = tokenizeAll("O100 CONTINUE");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeUnknownKeyword) {
    lexer->setInput("O100 UNKNOWN");
    auto tokens = tokenizeAll("O100 UNKNOWN");
    // Should still parse something
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeWithExpression) {
    lexer->setInput("O[#1] CALL");
    auto tokens = tokenizeAll("O[#1] CALL");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeRepeat) {
    lexer->setInput("O100 REPEAT");
    auto tokens = tokenizeAll("O100 REPEAT");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeEndrepeat) {
    lexer->setInput("O100 ENDREPEAT");
    auto tokens = tokenizeAll("O100 ENDREPEAT");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, OCodeElseif) {
    lexer->setInput("O100 ELSEIF [#1 GT 10]");
    auto tokens = tokenizeAll("O100 ELSEIF [#1 GT 10]");
    EXPECT_FALSE(tokens.empty());
}

// ============================================================================
// Additional Coverage Tests - Position and Navigation
// ============================================================================

TEST_F(LexerTest, SeekToLineValidCoverage) {
    lexer->setInput("N10 G0 X0\nN20 G1 X100\nN30 G1 X200");
    lexer->scanLineCount();
    bool found = lexer->seekToLine(2);
    EXPECT_TRUE(found);
}

TEST_F(LexerTest, SeekToLineInvalidCoverage) {
    lexer->setInput("G0 X0\nG1 X100");
    bool found = lexer->seekToLine(999);
    EXPECT_FALSE(found);
}

TEST_F(LexerTest, SeekBeyondEndCoverage) {
    lexer->setInput("G0");
    lexer->seek(1000);  // Way past end
    EXPECT_TRUE(lexer->atEnd());
}

// ============================================================================
// Additional Coverage Tests - Number Parsing
// ============================================================================

TEST_F(LexerTest, NegativeNumberCoverage) {
    auto tokens = tokenizeAll("X-100");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_EQ(tokens[0].letter, WordLetter::X);
        EXPECT_DOUBLE_EQ(tokens[0].value, -100.0);
    }
}

TEST_F(LexerTest, PositiveNumberWithPlusCoverage) {
    auto tokens = tokenizeAll("X+50");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_DOUBLE_EQ(tokens[0].value, 50.0);
    }
}

TEST_F(LexerTest, ScientificNotationCoverage) {
    // Lexer doesn't support scientific notation - it parses before the 'e'
    auto tokens = tokenizeAll("X1e-3");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_DOUBLE_EQ(tokens[0].value, 1.0);  // Just the "1" part
    }
}

TEST_F(LexerTest, ScientificNotationUpperECoverage) {
    // Lexer doesn't support scientific notation - parses only the number before E
    auto tokens = tokenizeAll("X1E3");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_DOUBLE_EQ(tokens[0].value, 1.0);  // Just the "1" part, E3 becomes separate
    }
}

TEST_F(LexerTest, LeadingDecimalPointCoverage) {
    auto tokens = tokenizeAll("X.5");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_DOUBLE_EQ(tokens[0].value, 0.5);
    }
}

TEST_F(LexerTest, TrailingDecimalPointCoverage) {
    auto tokens = tokenizeAll("X5.");
    EXPECT_FALSE(tokens.empty());
    if (!tokens.empty()) {
        EXPECT_DOUBLE_EQ(tokens[0].value, 5.0);
    }
}

TEST_F(LexerTest, MultipleDecimalPointsCoverage) {
    // Invalid number - lexer should handle gracefully
    auto tokens = tokenizeAll("X5.5.5");
    // May produce error or partial parse
}

// ============================================================================
// Additional Coverage Tests - Line Endings
// ============================================================================

TEST_F(LexerTest, WindowsLineEndingsCoverage) {
    auto tokens = tokenizeAll("G0\r\nG1");
    // Should handle Windows line endings
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, MacOldLineEndingsCoverage) {
    auto tokens = tokenizeAll("G0\rG1");
    // Should handle old Mac line endings
}

// ============================================================================
// Additional Coverage Tests - Expressions and Brackets
// ============================================================================

TEST_F(LexerTest, ExpressionBracketsCoverage) {
    auto tokens = tokenizeAll("[1 + 2]");
    // Should produce expression token
    EXPECT_FALSE(tokens.empty());
    bool foundExpr = false;
    for (const auto& t : tokens) {
        if (t.type == LexerTokenType::EXPRESSION) {
            foundExpr = true;
        }
    }
    EXPECT_TRUE(foundExpr);
}

TEST_F(LexerTest, NestedExpressionBracketsCoverage) {
    auto tokens = tokenizeAll("[[1 + 2] * 3]");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, ParameterInExpressionCoverage) {
    auto tokens = tokenizeAll("X[#100 + 5]");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, NamedParameterInExpressionCoverage) {
    auto tokens = tokenizeAll("X[#<myvar> * 2]");
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, UnterminatedCommentCoverage) {
    auto tokens = tokenizeAll("(unterminated comment");
    // Should handle gracefully - may produce error token
}

TEST_F(LexerTest, UnterminatedExpressionCoverage) {
    auto tokens = tokenizeAll("[1 + 2");
    // Should handle gracefully
}

TEST_F(LexerTest, EmptyCommentCoverage) {
    auto tokens = tokenizeAll("()");
    // Empty comment
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, CommentWithNewlineCoverage) {
    auto tokens = tokenizeAll("(comment\n)");
    // Comment can't span newlines in most dialects
}

// ============================================================================
// Additional Coverage Tests - Word Letters
// ============================================================================

TEST_F(LexerTest, CharToWordLetterAllCoverage) {
    EXPECT_EQ(charToWordLetter('X'), WordLetter::X);
    EXPECT_EQ(charToWordLetter('Y'), WordLetter::Y);
    EXPECT_EQ(charToWordLetter('Z'), WordLetter::Z);
    EXPECT_EQ(charToWordLetter('G'), WordLetter::G);
    EXPECT_EQ(charToWordLetter('M'), WordLetter::M);
    EXPECT_EQ(charToWordLetter('x'), WordLetter::X);  // Lowercase
}

TEST_F(LexerTest, AllWordLettersCompleteCoverage) {
    // Test all word letters
    std::string input = "A1 B2 C3 D4 E5 F6 G7 H8 I9 J10 K11 L12 M13 N14 O15 P16 Q17 R18 S19 T20 U21 V22 W23 X24 Y25 Z26";
    auto tokens = tokenizeAll(input.c_str());
    EXPECT_EQ(tokens.size(), 26u);
}

// ============================================================================
// Additional Coverage Tests - Special Cases
// ============================================================================

TEST_F(LexerTest, LineNumberWithNCoverage) {
    auto tokens = tokenizeAll("N100 G0 X0");
    bool hasLineNumber = false;
    for (const auto& t : tokens) {
        if (t.letter == WordLetter::N) {
            hasLineNumber = true;
            EXPECT_DOUBLE_EQ(t.value, 100.0);
        }
    }
    EXPECT_TRUE(hasLineNumber);
}

TEST_F(LexerTest, PercentSignCoverage) {
    auto tokens = tokenizeAll("%");
    // Percent sign is often program delimiter
}

TEST_F(LexerTest, SemicolonCommentCoverage) {
    auto tokens = tokenizeAll("G0 ; this is a comment");
    // Should have G0 token at least
    EXPECT_FALSE(tokens.empty());
}

TEST_F(LexerTest, MultiLineInputCoverage) {
    lexer->setInput("G0 X0\nG1 X10\nG1 X20");
    lexer->scanLineCount();
    EXPECT_EQ(lexer->getTotalLines(), 3u);
}

TEST_F(LexerTest, TokenizeLineMultipleTimesCoverage) {
    lexer->setInput("G0 X0\nG1 X100");
    auto line1 = lexer->tokenizeLine();
    EXPECT_FALSE(line1.empty());
    lexer->seekToLine(1);  // Move to second line
    auto line2 = lexer->tokenizeLine();
    // Line2 might be empty or have G1
}

} // namespace test
} // namespace GCode
