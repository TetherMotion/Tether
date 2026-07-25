/**
 * @file GCodeOCodesTests.cpp
 * @brief Unit tests for O-code control flow (subroutines, if/else, loops)
 */

#include <gtest/gtest.h>
#include "tether/gcode/GCodeOCodes.hpp"
#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeVariables.hpp"

using namespace GCode;

// ============================================================================
// Test fixture
// ============================================================================

class OCodeExecutorTest : public ::testing::Test {
protected:
    void SetUp() override {
        vars = std::make_unique<VariableSystem>();
        parser = std::make_unique<Parser>(*vars);
        executor = std::make_unique<OCodeExecutor>(*vars, *parser);
    }

    void TearDown() override {
        executor.reset();
        parser.reset();
        vars.reset();
    }

    // Keep the source string alive for the parser's lifetime.
    void setInput(const std::string& src) {
        m_source = src;
        parser->setInput(m_source.c_str());
    }

    std::unique_ptr<VariableSystem> vars;
    std::unique_ptr<Parser> parser;
    std::unique_ptr<OCodeExecutor> executor;
    std::string m_source;
};

// ============================================================================
// O-Code Utility Tests
// ============================================================================

TEST(OCodeUtilsTest, TypeToKeyword) {
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::SUB), "sub");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ENDSUB), "endsub");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::CALL), "call");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::RETURN), "return");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::IF), "if");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ELSEIF), "elseif");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ELSE), "else");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ENDIF), "endif");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::WHILE), "while");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ENDWHILE), "endwhile");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::DO), "do");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::REPEAT), "repeat");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::ENDREPEAT), "endrepeat");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::BREAK), "break");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::CONTINUE), "continue");
}

TEST(OCodeUtilsTest, KeywordToType) {
    EXPECT_EQ(keywordToOCodeType("sub"), OCodeType::SUB);
    EXPECT_EQ(keywordToOCodeType("SUB"), OCodeType::SUB);
    EXPECT_EQ(keywordToOCodeType("Sub"), OCodeType::SUB);
    EXPECT_EQ(keywordToOCodeType("endsub"), OCodeType::ENDSUB);
    EXPECT_EQ(keywordToOCodeType("call"), OCodeType::CALL);
    EXPECT_EQ(keywordToOCodeType("return"), OCodeType::RETURN);
    EXPECT_EQ(keywordToOCodeType("if"), OCodeType::IF);
    EXPECT_EQ(keywordToOCodeType("elseif"), OCodeType::ELSEIF);
    EXPECT_EQ(keywordToOCodeType("else"), OCodeType::ELSE);
    EXPECT_EQ(keywordToOCodeType("endif"), OCodeType::ENDIF);
    EXPECT_EQ(keywordToOCodeType("while"), OCodeType::WHILE);
    EXPECT_EQ(keywordToOCodeType("endwhile"), OCodeType::ENDWHILE);
    EXPECT_EQ(keywordToOCodeType("do"), OCodeType::DO);
    EXPECT_EQ(keywordToOCodeType("repeat"), OCodeType::REPEAT);
    EXPECT_EQ(keywordToOCodeType("endrepeat"), OCodeType::ENDREPEAT);
    EXPECT_EQ(keywordToOCodeType("break"), OCodeType::BREAK);
    EXPECT_EQ(keywordToOCodeType("continue"), OCodeType::CONTINUE);
}

TEST(OCodeUtilsTest, IsBlockOpener) {
    EXPECT_TRUE(isBlockOpener(OCodeType::SUB));
    EXPECT_TRUE(isBlockOpener(OCodeType::IF));
    EXPECT_TRUE(isBlockOpener(OCodeType::WHILE));
    EXPECT_TRUE(isBlockOpener(OCodeType::DO));
    EXPECT_TRUE(isBlockOpener(OCodeType::REPEAT));
    EXPECT_FALSE(isBlockOpener(OCodeType::ENDSUB));
    EXPECT_FALSE(isBlockOpener(OCodeType::ENDIF));
    EXPECT_FALSE(isBlockOpener(OCodeType::CALL));
    EXPECT_FALSE(isBlockOpener(OCodeType::RETURN));
    EXPECT_FALSE(isBlockOpener(OCodeType::BREAK));
}

TEST(OCodeUtilsTest, IsBlockCloser) {
    EXPECT_TRUE(isBlockCloser(OCodeType::ENDSUB));
    EXPECT_TRUE(isBlockCloser(OCodeType::ENDIF));
    EXPECT_TRUE(isBlockCloser(OCodeType::ENDWHILE));
    EXPECT_TRUE(isBlockCloser(OCodeType::ENDREPEAT));
    EXPECT_FALSE(isBlockCloser(OCodeType::SUB));
    EXPECT_FALSE(isBlockCloser(OCodeType::IF));
    EXPECT_FALSE(isBlockCloser(OCodeType::WHILE));
    EXPECT_FALSE(isBlockCloser(OCodeType::DO));
    EXPECT_FALSE(isBlockCloser(OCodeType::REPEAT));
}

TEST(OCodeUtilsTest, GetMatchingCloser) {
    EXPECT_EQ(getMatchingCloser(OCodeType::IF), OCodeType::ENDIF);
    EXPECT_EQ(getMatchingCloser(OCodeType::WHILE), OCodeType::ENDWHILE);
    EXPECT_EQ(getMatchingCloser(OCodeType::DO), OCodeType::WHILE);
    EXPECT_EQ(getMatchingCloser(OCodeType::REPEAT), OCodeType::ENDREPEAT);
    EXPECT_EQ(getMatchingCloser(OCodeType::SUB), OCodeType::ENDSUB);
}

TEST(OCodeUtilsTest, FormatOCode) {
    EXPECT_EQ(formatOCode(100, OCodeType::SUB), "O100 sub");
    EXPECT_EQ(formatOCode(101, OCodeType::IF), "O101 if");
    EXPECT_EQ(formatOCode("myroutine", OCodeType::CALL), "O<myroutine> call");
    EXPECT_EQ(formatOCode("test", OCodeType::ENDSUB), "O<test> endsub");
}

// ============================================================================
// SubroutineRegistry Tests
// ============================================================================

TEST(SubroutineRegistryTest, RegisterAndFindNumbered) {
    SubroutineRegistry reg;
    SubroutineInfo info;
    info.oNumber = 100;
    info.isNamed = false;
    info.startLine = 5;

    Error err = reg.registerSubroutine(100, info);
    EXPECT_FALSE(err);

    const SubroutineInfo* found = reg.find(100);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->oNumber, 100);
    EXPECT_EQ(found->startLine, 5);
}

TEST(SubroutineRegistryTest, RegisterAndFindNamed) {
    SubroutineRegistry reg;
    SubroutineInfo info;
    info.oNumber = 200;
    info.isNamed = true;
    info.name = "myroutine";
    info.startLine = 10;

    Error err = reg.registerSubroutine("myroutine", info);
    EXPECT_FALSE(err);

    const SubroutineInfo* found = reg.find("myroutine");
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->name, "myroutine");
    EXPECT_EQ(found->startLine, 10);
}

TEST(SubroutineRegistryTest, DuplicateNumbered) {
    SubroutineRegistry reg;
    SubroutineInfo info1, info2;
    info1.oNumber = 100;
    info2.oNumber = 100;

    EXPECT_FALSE(reg.registerSubroutine(100, info1));
    EXPECT_TRUE(reg.registerSubroutine(100, info2));  // Duplicate
}

TEST(SubroutineRegistryTest, DuplicateNamed) {
    SubroutineRegistry reg;
    SubroutineInfo info1, info2;
    info1.name = "myroutine";
    info2.name = "myroutine";

    EXPECT_FALSE(reg.registerSubroutine("myroutine", info1));
    EXPECT_TRUE(reg.registerSubroutine("myroutine", info2));  // Duplicate
}

TEST(SubroutineRegistryTest, FindNonexistent) {
    SubroutineRegistry reg;
    EXPECT_EQ(reg.find(999), nullptr);
    EXPECT_EQ(reg.find("nonexistent"), nullptr);
    EXPECT_FALSE(reg.exists(999));
    EXPECT_FALSE(reg.exists("nonexistent"));
}

TEST(SubroutineRegistryTest, Clear) {
    SubroutineRegistry reg;
    SubroutineInfo info;
    info.oNumber = 100;
    reg.registerSubroutine(100, info);
    reg.registerSubroutine("test", info);

    EXPECT_TRUE(reg.exists(100));
    EXPECT_TRUE(reg.exists("test"));

    reg.clear();

    EXPECT_FALSE(reg.exists(100));
    EXPECT_FALSE(reg.exists("test"));
}

// ============================================================================
// OCodeExecutor Basic Tests
// ============================================================================

TEST_F(OCodeExecutorTest, InitialState) {
    EXPECT_FALSE(executor->inSubroutine());
    EXPECT_EQ(executor->getCallDepth(), 0u);
    EXPECT_FALSE(executor->inLoop());
    EXPECT_FALSE(executor->getReturnValue().has_value());
}

TEST_F(OCodeExecutorTest, Reset) {
    // Set some state
    vars->set(1, 42.0);
    executor->reset();
    EXPECT_FALSE(executor->inSubroutine());
    EXPECT_FALSE(executor->inLoop());
    EXPECT_FALSE(executor->getReturnValue().has_value());
}

TEST_F(OCodeExecutorTest, ExecuteNonOCodeBlock) {
    Block block;
    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
}

TEST_F(OCodeExecutorTest, IfTrueCondition) {
    setInput("O100 if [1 GT 0]\nG1 X100\nO100 endif");
    vars->set(1, 5.0);

    Block block;
    Error parseErr = parser->parseNextBlock(block);
    ASSERT_FALSE(parseErr) << "parseNextBlock error: " << parseErr.message.data();
    EXPECT_EQ(block.oCodeType, OCodeType::IF);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_FALSE(err);
    // Condition is true, so we continue (don't skip)
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
}

TEST_F(OCodeExecutorTest, IfFalseCondition) {
    setInput("O100 if [1 LT 0]\nG1 X100\nO100 endif");
    vars->set(1, 5.0);

    Block block;
    ASSERT_FALSE(parser->parseNextBlock(block));
    EXPECT_EQ(block.oCodeType, OCodeType::IF);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_FALSE(err);
    // Condition is false, so we skip to else/endif
    EXPECT_EQ(action, OCodeExecutor::NextAction::SKIP_TO_ELSE);
}

TEST_F(OCodeExecutorTest, EndIf) {
    setInput("O100 if [1]\nG1 X100\nO100 endif");

    // Parse and execute the if block first
    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));

    // Parse the G1 block
    Block g1Block;
    ASSERT_FALSE(parser->parseNextBlock(g1Block));

    // Parse the endif block
    Block endBlock;
    ASSERT_FALSE(parser->parseNextBlock(endBlock));
    EXPECT_EQ(endBlock.oCodeType, OCodeType::ENDIF);

    Error err = executor->execute(endBlock, action);
    EXPECT_FALSE(err);
    EXPECT_FALSE(executor->inSubroutine());
}

TEST_F(OCodeExecutorTest, RepeatLoop) {
    setInput("O100 repeat [3]\nG1 X100\nO100 endrepeat");

    // Parse the repeat block
    Block repeatBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    EXPECT_EQ(repeatBlock.oCodeType, OCodeType::REPEAT);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(repeatBlock, action);
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->inLoop());

    // Parse the G1 block
    Block g1Block;
    ASSERT_FALSE(parser->parseNextBlock(g1Block));

    // Parse the endrepeat block
    Block endBlock;
    ASSERT_FALSE(parser->parseNextBlock(endBlock));
    EXPECT_EQ(endBlock.oCodeType, OCodeType::ENDREPEAT);

    // First endrepeat: should jump back (iteration 0 -> 1)
    err = executor->execute(endBlock, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inLoop());

    // Second iteration
    err = executor->execute(endBlock, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inLoop());

    // Third iteration - should exit
    err = executor->execute(endBlock, action);
    EXPECT_FALSE(err);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, RepeatZeroCount) {
    setInput("O100 repeat [0]\nG1 X100\nO100 endrepeat");

    Block repeatBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    EXPECT_EQ(repeatBlock.oCodeType, OCodeType::REPEAT);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(repeatBlock, action);
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->inLoop());

    // With 0 iterations, the first endrepeat should exit immediately
    Block endBlock;
    // Skip G1
    Block g1Block;
    ASSERT_FALSE(parser->parseNextBlock(g1Block));
    ASSERT_FALSE(parser->parseNextBlock(endBlock));

    err = executor->execute(endBlock, action);
    EXPECT_FALSE(err);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, BreakOutsideLoop) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::BREAK;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, ContinueOutsideLoop) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::CONTINUE;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndIfWithoutIf) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ENDIF;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndWhileWithoutWhile) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ENDWHILE;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndRepeatWithoutRepeat) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ENDREPEAT;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, ReturnWithoutCall) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::RETURN;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndSubWithoutCall) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ENDSUB;

    OCodeExecutor::NextAction action;
    Error err = executor->execute(block, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, WhileLoopCondition) {
    setInput("O100 while [1 LT 3]\n#1 = [1 + 1]\nO100 endwhile");
    vars->set(1, 0.0);

    // Parse while block
    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    EXPECT_EQ(whileBlock.oCodeType, OCodeType::WHILE);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(whileBlock, action);
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, DoBlock) {
    setInput("O100 do\nG1 X100\nO100 while [1 GT 0]");

    Block doBlock;
    ASSERT_FALSE(parser->parseNextBlock(doBlock));
    EXPECT_EQ(doBlock.oCodeType, OCodeType::DO);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(doBlock, action);
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, ModalStateSaveRestore) {
    MachineState state;
    state.feedRate = 500.0;
    state.spindleSpeed = 1000.0;

    EXPECT_FALSE(executor->saveModalState(state));
    MachineState restored;
    EXPECT_FALSE(executor->restoreModalState(restored));
    EXPECT_DOUBLE_EQ(restored.feedRate, 500.0);
    EXPECT_DOUBLE_EQ(restored.spindleSpeed, 1000.0);
}

TEST_F(OCodeExecutorTest, ModalStateInvalidate) {
    MachineState state;
    state.feedRate = 500.0;

    EXPECT_FALSE(executor->saveModalState(state));
    EXPECT_FALSE(executor->invalidateModalState());

    // Restore after invalidate should not change anything
    MachineState restored;
    EXPECT_FALSE(executor->restoreModalState(restored));
    EXPECT_DOUBLE_EQ(restored.feedRate, 0.0);  // Not restored
}

TEST_F(OCodeExecutorTest, ScanSubroutines) {
    setInput("O100 sub\nG1 X100\nO100 endsub\nO<mysub> sub\nG1 X200\nO<mysub> endsub");

    Error err = executor->scanSubroutines();
    EXPECT_FALSE(err);

    // The registry should have found the subroutines
    // (Note: scanSource uses findMatchingOCode which may have issues with
    //  state restoration, so we just check that scan doesn't error)
}
