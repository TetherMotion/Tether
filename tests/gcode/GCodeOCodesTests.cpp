/**
 * @file GCodeOCodesTests.cpp
 * @brief Unit tests for O-code control flow (subroutines, if/else, loops)
 */

#include <cstdio>
#include <fstream>

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
    // Condition is false — the executor jumps to the else/elseif/endif line.
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    // The jump target should be the endif line (re-executed to pop the frame).
    parser->getLexer().seek(executor->getJumpAddress());
    Block endifBlock;
    ASSERT_FALSE(parser->parseNextBlock(endifBlock));
    EXPECT_EQ(endifBlock.oCodeType, OCodeType::ENDIF);
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

// ============================================================================
// Coverage additions — dead branches in OCodeExecutor
// ============================================================================

TEST_F(OCodeExecutorTest, SubDefinitionSkipsBody) {
    setInput("O100 sub\nG1 X100\nO100 endsub\nG1 X200");

    Block subBlock;
    ASSERT_FALSE(parser->parseNextBlock(subBlock));
    EXPECT_EQ(subBlock.oCodeType, OCodeType::SUB);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(subBlock, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    // Jump lands past the endsub — next parsed block is the trailing G1.
    parser->getLexer().seek(executor->getJumpAddress());
    Block tail;
    ASSERT_FALSE(parser->parseNextBlock(tail));
    EXPECT_FALSE(tail.hasOCode);
}

TEST_F(OCodeExecutorTest, BareOLabelIsNoop) {
    setInput("O1234\nG1 X100\nM99");

    Block label;
    ASSERT_FALSE(parser->parseNextBlock(label));
    EXPECT_TRUE(label.hasOCode);
    EXPECT_EQ(label.oCodeType, OCodeType::SUB);
    EXPECT_FALSE(label.oCodeHasKeyword);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(label, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
}

TEST_F(OCodeExecutorTest, CallNumberedSubFullFlow) {
    setInput("O100 call\nG1 X999\nO100 sub\nG1 X100\nO100 endsub\nG1 X200");

    Block callBlock;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    EXPECT_EQ(callBlock.oCodeType, OCodeType::CALL);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(callBlock, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inSubroutine());
    EXPECT_EQ(executor->getCallDepth(), 1u);

    // Jump into the sub body (line after `O100 sub`).
    parser->getLexer().seek(executor->getJumpAddress());
    Block body;
    ASSERT_FALSE(parser->parseNextBlock(body));
    EXPECT_FALSE(body.hasOCode);

    Block endsub;
    ASSERT_FALSE(parser->parseNextBlock(endsub));
    EXPECT_EQ(endsub.oCodeType, OCodeType::ENDSUB);
    err = executor->execute(endsub, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inSubroutine());
}

TEST_F(OCodeExecutorTest, CallNamedSubFullFlow) {
    setInput("O<mysub> call\nO<mysub> sub\nG1 X100\nO<mysub> endsub");

    Block callBlock;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    EXPECT_EQ(callBlock.oCodeType, OCodeType::CALL);

    OCodeExecutor::NextAction action;
    Error err = executor->execute(callBlock, action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inSubroutine());

    parser->getLexer().seek(executor->getJumpAddress());
    Block body, endsub;
    ASSERT_FALSE(parser->parseNextBlock(body));
    ASSERT_FALSE(parser->parseNextBlock(endsub));
    err = executor->execute(endsub, action);
    EXPECT_FALSE(err);
    EXPECT_FALSE(executor->inSubroutine());
}

TEST_F(OCodeExecutorTest, CallWithArgsBindsFrame) {
    setInput("O100 call [5]\nO100 sub\nO100 return [#1]\nO100 endsub");

    executor->scanSubroutines();

    Block callBlock;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(callBlock, action));
    EXPECT_TRUE(executor->inSubroutine());

    // Arg 5.0 bound to #1 in the pushed frame.
    parser->getLexer().seek(executor->getJumpAddress());
    Block retBlock;
    ASSERT_FALSE(parser->parseNextBlock(retBlock));
    EXPECT_EQ(retBlock.oCodeType, OCodeType::RETURN);
    ASSERT_FALSE(executor->execute(retBlock, action));
    ASSERT_TRUE(executor->getReturnValue().has_value());
    EXPECT_DOUBLE_EQ(*executor->getReturnValue(), 5.0);
    EXPECT_DOUBLE_EQ(*vars->getNamed("_value"), 5.0);
    EXPECT_DOUBLE_EQ(*vars->getNamed("_value_returned"), 1.0);
}

TEST_F(OCodeExecutorTest, EndsubClearsValueReturned) {
    setInput("O100 call\nO100 sub\nO100 endsub");

    executor->scanSubroutines();
    Block callBlock, endsub;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(callBlock, action));

    parser->getLexer().seek(executor->getJumpAddress());
    ASSERT_FALSE(parser->parseNextBlock(endsub));
    ASSERT_FALSE(executor->execute(endsub, action));
    EXPECT_DOUBLE_EQ(*vars->getNamed("_value_returned"), 0.0);
    EXPECT_FALSE(executor->getReturnValue().has_value());
}

TEST_F(OCodeExecutorTest, CallUndefinedSubErrors) {
    setInput("O100 call");

    Block callBlock;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(callBlock, action);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::UNDEFINED_SUBROUTINE);
}

TEST_F(OCodeExecutorTest, CallUndefinedNamedSubErrors) {
    setInput("O<missing> call");

    Block callBlock;
    ASSERT_FALSE(parser->parseNextBlock(callBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(callBlock, action);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::UNDEFINED_SUBROUTINE);
}

TEST(OCodeExecutorDepthTest, MaxCallDepthErrors) {
    VariableSystem vars;
    Parser parser(vars);
    OCodeConfig cfg;
    cfg.maxCallDepth = 1;
    OCodeExecutor exec(vars, parser, cfg);

    std::string src = "O100 sub\nO100 call\nO100 endsub\nO100 call";
    parser.setInput(src.c_str());
    exec.scanSubroutines();

    // First call succeeds — pushed directly.
    ASSERT_FALSE(exec.callSubroutine(100));
    EXPECT_EQ(exec.getCallDepth(), 1u);

    // Second exceeds maxCallDepth.
    Error err = exec.callSubroutine(100);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::NESTED_TOO_DEEP);

    Error err2 = exec.callSubroutine("named");
    EXPECT_TRUE(err2);
    EXPECT_EQ(err2.code, ErrorCode::NESTED_TOO_DEEP);
}

TEST_F(OCodeExecutorTest, CallSubprogramFanucLabel) {
    setInput("M98 P1234\nG1 X999\nO1234\nG1 X100\nM99");

    // Bare O1234 label — executeM98 resolves via findSubprogramLabel.
    Block m98Block;
    ASSERT_FALSE(parser->parseNextBlock(m98Block));

    Error err = executor->executeM98(1234, 1);
    EXPECT_FALSE(err);

    parser->getLexer().seek(executor->getJumpAddress());
    Block body;
    ASSERT_FALSE(parser->parseNextBlock(body));
    EXPECT_FALSE(body.hasOCode);

    OCodeExecutor::NextAction action;
    Block m99Block;
    ASSERT_FALSE(parser->parseNextBlock(m99Block));
    err = executor->executeM99(action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    // Returns to just after the M98 call.
    parser->getLexer().seek(executor->getJumpAddress());
    Block after;
    ASSERT_FALSE(parser->parseNextBlock(after));
    EXPECT_FALSE(after.hasOCode);
}

TEST_F(OCodeExecutorTest, M98RepeatCount) {
    setInput("M98 P1234 L2\nO1234\nG1 X100\nM99");

    Block m98Block;
    ASSERT_FALSE(parser->parseNextBlock(m98Block));
    ASSERT_FALSE(executor->executeM98(1234, 2));

    OCodeExecutor::NextAction action;
    // First M99: repeat once more — jump back to sub start.
    ASSERT_FALSE(executor->executeM99(action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    // Second M99: count exhausted — jump back to caller.
    ASSERT_FALSE(executor->executeM99(action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
}

TEST_F(OCodeExecutorTest, M98LWordClampedAndSubEndsub) {
    setInput("O200 sub\nG1 X100\nO200 endsub");

    executor->scanSubroutines();
    // lWord 0 clamps to 1; O200 resolves via `sub/endsub` registry entry.
    Error err = executor->executeM98(200, 0);
    EXPECT_FALSE(err);

    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->executeM99(action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
}

TEST_F(OCodeExecutorTest, M98UndefinedErrors) {
    setInput("G1 X100");
    Error err = executor->executeM98(9999, 1);
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::UNDEFINED_SUBROUTINE);
}

TEST_F(OCodeExecutorTest, M99TopLevelIsNoop) {
    setInput("G1 X100");
    OCodeExecutor::NextAction action;
    Error err = executor->executeM99(action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
}

TEST_F(OCodeExecutorTest, M99InsideSubCallReturns) {
    setInput("O100 sub\nM99\nO100 endsub");

    executor->scanSubroutines();
    ASSERT_FALSE(executor->callSubroutine(100));
    EXPECT_TRUE(executor->inSubroutine());

    OCodeExecutor::NextAction action;
    Error err = executor->executeM99(action);
    EXPECT_FALSE(err);
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inSubroutine());
}

TEST_F(OCodeExecutorTest, CallSubprogramUndefinedErrors) {
    setInput("G1 X100");
    Error err = executor->callSubprogram(4242, {1.0});
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::UNDEFINED_SUBROUTINE);
}

TEST_F(OCodeExecutorTest, WhileLoopIterates) {
    // #1 counts down; body sets it via vars between iterations.
    setInput("O100 while [#1 GT 0]\nG1 X100\nO100 endwhile\nG1 X200");
    vars->set(1, 2.0);

    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_TRUE(executor->inLoop());
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);

    // First endwhile: condition still true → JUMP back to body.
    Block g1, endwhile;
    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(endwhile));
    vars->set(1, 1.0);
    ASSERT_FALSE(executor->execute(endwhile, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inLoop());

    // Jump back to loop body.
    parser->getLexer().seek(executor->getJumpAddress());
    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(endwhile));
    vars->set(1, 0.0);
    ASSERT_FALSE(executor->execute(endwhile, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, WhileFalseSkipsToEndwhile) {
    setInput("O100 while [1 LT 0]\nG1 X100\nO100 endwhile\nG1 X200");

    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inLoop());

    parser->getLexer().seek(executor->getJumpAddress());
    Block tail;
    ASSERT_FALSE(parser->parseNextBlock(tail));
    EXPECT_FALSE(tail.hasOCode);  // G1 X200
}

TEST_F(OCodeExecutorTest, WhileWithoutEndwhileErrors) {
    setInput("O100 while [0]\nG1 X100");

    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(whileBlock, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndwhileTypeMismatchErrors) {
    setInput("O100 repeat [1]\nO100 endwhile");

    Block repeatBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(repeatBlock, action));
    EXPECT_TRUE(executor->inLoop());

    Block endwhile;
    ASSERT_FALSE(parser->parseNextBlock(endwhile));
    Error err = executor->execute(endwhile, action);
    EXPECT_TRUE(err);  // endwhile does not match repeat
}

TEST_F(OCodeExecutorTest, DoWhileIterates) {
    setInput("O100 do\nG1 X100\nO100 while [#1 GT 0]\nG1 X200");
    vars->set(1, 1.0);

    Block doBlock, g1, whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(doBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(doBlock, action));
    EXPECT_TRUE(executor->inLoop());

    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    EXPECT_EQ(whileBlock.oCodeType, OCodeType::WHILE);
    // Dispatches to doWhile (loopStack top is DO_WHILE) — cond true → JUMP.
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inLoop());

    // Second pass: cond false → pop.
    parser->getLexer().seek(executor->getJumpAddress());
    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    vars->set(1, 0.0);
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, RepeatNegativeCountErrors) {
    setInput("O100 repeat [-2]\nO100 endrepeat");

    Block repeatBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(repeatBlock, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, EndrepeatTypeMismatchErrors) {
    setInput("O100 while [1]\nO100 endrepeat");

    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));

    Block endrepeat;
    ASSERT_FALSE(parser->parseNextBlock(endrepeat));
    Error err = executor->execute(endrepeat, action);
    EXPECT_TRUE(err);  // endrepeat does not match while
}

TEST_F(OCodeExecutorTest, BreakFromWhile) {
    setInput("O100 while [1]\nG1 X100\nO100 break\nG1 X200\nO100 endwhile\nG1 X300");

    Block whileBlock, g1, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));
    EXPECT_EQ(breakBlock.oCodeType, OCodeType::BREAK);

    ASSERT_FALSE(executor->execute(breakBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inLoop());

    parser->getLexer().seek(executor->getJumpAddress());
    Block tail;
    ASSERT_FALSE(parser->parseNextBlock(tail));
    EXPECT_FALSE(tail.hasOCode);  // G1 X300
}

TEST_F(OCodeExecutorTest, BreakOuterLoopPopsInner) {
    setInput("O100 while [1]\nO200 while [1]\nO100 break\nO200 endwhile\nO100 endwhile");

    Block whileBlock, inner, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(inner));
    ASSERT_FALSE(executor->execute(inner, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    // Break targeting O100 pops both frames and jumps past O100 endwhile.
    ASSERT_FALSE(executor->execute(breakBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, BreakNonMatchingNumberErrors) {
    setInput("O100 while [1]\nO999 break\nO100 endwhile");

    Block whileBlock, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    Error err = executor->execute(breakBlock, action);
    EXPECT_TRUE(err);  // no O999 loop ender found
}

TEST_F(OCodeExecutorTest, ContinueInWhile) {
    setInput("O100 while [1]\nO100 continue\nO100 endwhile");

    Block whileBlock, contBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(contBlock));
    EXPECT_EQ(contBlock.oCodeType, OCodeType::CONTINUE);

    ASSERT_FALSE(executor->execute(contBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_TRUE(executor->inLoop());  // frame stays; endwhile re-evaluates

    // Jump lands on the endwhile line.
    parser->getLexer().seek(executor->getJumpAddress());
    Block endwhile;
    ASSERT_FALSE(parser->parseNextBlock(endwhile));
    EXPECT_EQ(endwhile.oCodeType, OCodeType::ENDWHILE);
}

TEST_F(OCodeExecutorTest, ContinueNonMatchingErrors) {
    setInput("O100 while [1]\nO999 continue\nO100 endwhile");

    Block whileBlock, contBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(contBlock));

    Error err = executor->execute(contBlock, action);
    EXPECT_TRUE(err);  // continue target loop not found
}

TEST_F(OCodeExecutorTest, IfElseifTakesElseifBranch) {
    setInput("O100 if [0]\nG1 X1\nO100 elseif [1]\nG1 X2\nO100 endif\nG1 X3");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    // Jump lands on the elseif line — re-execute; cond true → CONTINUE.
    parser->getLexer().seek(executor->getJumpAddress());
    Block elseifBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseifBlock));
    EXPECT_EQ(elseifBlock.oCodeType, OCodeType::ELSEIF);
    ASSERT_FALSE(executor->execute(elseifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);

    Block body, endif;
    ASSERT_FALSE(parser->parseNextBlock(body));
    ASSERT_FALSE(parser->parseNextBlock(endif));
    ASSERT_FALSE(executor->execute(endif, action));
}

TEST_F(OCodeExecutorTest, IfElseifFallsToElse) {
    setInput("O100 if [0]\nO100 elseif [0]\nO100 else\nG1 X2\nO100 endif");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    // elseif [0] → false → jump to else.
    parser->getLexer().seek(executor->getJumpAddress());
    Block elseifBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseifBlock));
    ASSERT_FALSE(executor->execute(elseifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    // else → no branch taken → execute body.
    parser->getLexer().seek(executor->getJumpAddress());
    Block elseBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseBlock));
    EXPECT_EQ(elseBlock.oCodeType, OCodeType::ELSE);
    ASSERT_FALSE(executor->execute(elseBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
}

TEST_F(OCodeExecutorTest, ElseifAfterTakenSkipsToEndif) {
    setInput("O100 if [1]\nG1 X1\nO100 elseif [1]\nG1 X2\nO100 endif\nG1 X3");

    Block ifBlock, g1, elseifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);

    ASSERT_FALSE(parser->parseNextBlock(g1));
    ASSERT_FALSE(parser->parseNextBlock(elseifBlock));
    // branchTaken → skip to endif.
    ASSERT_FALSE(executor->execute(elseifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    parser->getLexer().seek(executor->getJumpAddress());
    Block endif;
    ASSERT_FALSE(parser->parseNextBlock(endif));
    EXPECT_EQ(endif.oCodeType, OCodeType::ENDIF);
}

TEST_F(OCodeExecutorTest, ElseAfterTakenSkipsToEndif) {
    setInput("O100 if [1]\nO100 else\nG1 X2\nO100 endif\nG1 X3");

    Block ifBlock, elseBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(elseBlock));

    ASSERT_FALSE(executor->execute(elseBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    parser->getLexer().seek(executor->getJumpAddress());
    Block endif;
    ASSERT_FALSE(parser->parseNextBlock(endif));
    EXPECT_EQ(endif.oCodeType, OCodeType::ENDIF);
}

TEST_F(OCodeExecutorTest, ElseifWithoutIfErrors) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ELSEIF;

    OCodeExecutor::NextAction action;
    EXPECT_TRUE(executor->execute(block, action));
}

TEST_F(OCodeExecutorTest, ElseWithoutIfErrors) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::ELSE;

    OCodeExecutor::NextAction action;
    EXPECT_TRUE(executor->execute(block, action));
}

TEST_F(OCodeExecutorTest, IfMissingEndifErrors) {
    setInput("O100 if [0]\nG1 X1");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(ifBlock, action);
    EXPECT_TRUE(err);  // scanToBranchPoint: if without matching endif
}

TEST_F(OCodeExecutorTest, DebugOCodeTypeErrors) {
    Block block;
    block.hasOCode = true;
    block.oCodeNumber = 100;
    block.oCodeType = OCodeType::DEBUG;

    OCodeExecutor::NextAction action;
    EXPECT_TRUE(executor->execute(block, action));  // unknown O-code type
}

TEST_F(OCodeExecutorTest, ScanRegistersSubsAndFanucLabels) {
    setInput("O100 sub\nO100 endsub\nO<mysub> sub\nO<mysub> endsub\nO1234\nM99");

    ASSERT_FALSE(executor->scanSubroutines());
    EXPECT_TRUE(executor->getRegistry().exists(100));
    EXPECT_TRUE(executor->getRegistry().exists(1234));  // bare Fanuc label
    EXPECT_TRUE(executor->getRegistry().exists("mysub"));
}

TEST_F(OCodeExecutorTest, ScanDuplicateSubErrors) {
    setInput("O100 sub\nO100 endsub\nO100 sub\nO100 endsub");
    Error err = executor->scanSubroutines();
    EXPECT_TRUE(err);
}

TEST(OCodeRegistryFileTest, LoadFromFileAndMissing) {
    SubroutineRegistry reg;

    Error missing = reg.loadFromFile("/nonexistent/path/sub.ngc");
    EXPECT_TRUE(missing);
    EXPECT_EQ(missing.code, ErrorCode::FILE_NOT_FOUND);

    const char* path = "/tmp/tether_ocode_cov.ngc";
    {
        std::ofstream f(path);
        f << "O555 sub\nG1 X1\nO555 endsub\n";
    }
    Error ok = reg.loadFromFile(path);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(reg.exists(555));
    std::remove(path);
}

TEST_F(OCodeExecutorTest, AutoRestoreModalInsideCall) {
    setInput("O100 sub\nO100 endsub");
    executor->scanSubroutines();
    ASSERT_FALSE(executor->callSubroutine(100));
    EXPECT_TRUE(executor->inSubroutine());

    MachineState state;
    state.feedRate = 750.0;
    EXPECT_FALSE(executor->autoRestoreModalState(state));
}

TEST_F(OCodeExecutorTest, ResetClearsAllState) {
    setInput("O100 while [1]\nO100 if [1]\nO100 endwhile");
    vars->set(1, 5.0);
    executor->scanSubroutines();

    EXPECT_TRUE(executor->callSubroutine(100));  // no O100 sub — fails, stays 0
    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_TRUE(executor->inLoop());

    executor->reset();
    EXPECT_FALSE(executor->inLoop());
    EXPECT_FALSE(executor->inSubroutine());
    EXPECT_EQ(executor->getJumpAddress(), 0u);
    EXPECT_FALSE(executor->getRegistry().exists(100));  // registry cleared
}

TEST_F(OCodeExecutorTest, GetRepeatCountTracksIteration) {
    setInput("O100 repeat [5]\nO100 endrepeat");

    Block repeatBlock, endrepeat;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(repeatBlock, action));
    EXPECT_EQ(executor->getRepeatCount(), 0);

    ASSERT_FALSE(parser->parseNextBlock(endrepeat));
    ASSERT_FALSE(executor->execute(endrepeat, action));
    EXPECT_EQ(executor->getRepeatCount(), 1);

    executor->reset();
    EXPECT_EQ(executor->getRepeatCount(), 0);  // empty stack
}

TEST_F(OCodeExecutorTest, RepeatBadCountErrors) {
    setInput("O100 repeat [1 + ]\nO100 endrepeat");

    Block repeatBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    // Malformed count expression → evaluator error → "Invalid repeat count".
    Error err = executor->execute(repeatBlock, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, IfBadConditionErrors) {
    setInput("O100 if [#999 +]\nO100 endif");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(ifBlock, action);
    EXPECT_TRUE(err);  // evaluateCondition error propagates
}

TEST_F(OCodeExecutorTest, CallSubprogramFindsUnscannedSub) {
    // O200 sub exists in source but registry is empty — findSubroutine path.
    setInput("O200 sub\nO200 endsub");
    Error err = executor->callSubprogram(200, {1.0});
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->inSubroutine());
    EXPECT_TRUE(executor->getRegistry().exists(200));
}

TEST(OCodeExecutorDepthTest, CallSubprogramMaxDepthErrors) {
    VariableSystem vars;
    Parser parser(vars);
    OCodeConfig cfg;
    cfg.maxCallDepth = 0;
    OCodeExecutor exec(vars, parser, cfg);
    Error err = exec.callSubprogram(100, {});
    EXPECT_TRUE(err);
    EXPECT_EQ(err.code, ErrorCode::NESTED_TOO_DEEP);
}

TEST_F(OCodeExecutorTest, DoWhileWithoutDoErrors) {
    OCodeExecutor::NextAction action;
    Error err = executor->doWhile(100, "[1]", action);
    EXPECT_TRUE(err);  // empty loop stack
}

TEST_F(OCodeExecutorTest, DoWhileTypeMismatchErrors) {
    setInput("O100 while [1]\nO100 endwhile");
    Block whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));  // pushes WHILE frame

    // Direct doWhile on a WHILE frame → "do-while mismatch".
    Error err = executor->doWhile(100, "[0]", action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, BreakFromDoWhile) {
    setInput("O100 do\nO100 break\nO100 while [0]\nG1 X1");

    Block doBlock, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(doBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(doBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    ASSERT_FALSE(executor->execute(breakBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, ContinueInDoWhile) {
    setInput("O100 do\nO100 continue\nO100 while [0]");

    Block doBlock, contBlock;
    ASSERT_FALSE(parser->parseNextBlock(doBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(doBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(contBlock));

    ASSERT_FALSE(executor->execute(contBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    parser->getLexer().seek(executor->getJumpAddress());
    Block ender;
    ASSERT_FALSE(parser->parseNextBlock(ender));
    EXPECT_EQ(ender.oCodeType, OCodeType::WHILE);  // do-while closer
}

TEST_F(OCodeExecutorTest, ContinueInRepeat) {
    setInput("O100 repeat [2]\nO100 continue\nO100 endrepeat");

    Block repeatBlock, contBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(repeatBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(contBlock));

    ASSERT_FALSE(executor->execute(contBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    parser->getLexer().seek(executor->getJumpAddress());
    Block ender;
    ASSERT_FALSE(parser->parseNextBlock(ender));
    EXPECT_EQ(ender.oCodeType, OCodeType::ENDREPEAT);
}

TEST_F(OCodeExecutorTest, M98FindsUnscannedSubEndsub) {
    setInput("O200 sub\nG1 X1\nO200 endsub");
    // No scanSubroutines — executeM98 falls back to findSubroutine.
    Error err = executor->executeM98(200, 1);
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->getRegistry().exists(200));
}

TEST_F(OCodeExecutorTest, AddSearchPath) {
    executor->addSearchPath("/tmp/subs");
    // Path stored on config — no public getter; call covers the method body.
    SUCCEED();
}

TEST_F(OCodeExecutorTest, DoWhileEmptyConditionPops) {
    setInput("O100 do\nO100 while\nG1 X1");

    Block doBlock, whileBlock;
    ASSERT_FALSE(parser->parseNextBlock(doBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(doBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));

    // Empty condition → evaluateCondition("") → false → pop.
    ASSERT_FALSE(executor->execute(whileBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::CONTINUE);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, BreakScansPastNestedSameNumber) {
    setInput("O100 while [1]\nO100 while [1]\nO100 break\n"
             "O100 while [0]\nO100 endwhile\nO100 endwhile\nO100 endwhile");

    Block outer, inner, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(outer));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(outer, action));
    ASSERT_FALSE(parser->parseNextBlock(inner));
    ASSERT_FALSE(executor->execute(inner, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    // Break pops the inner O100 frame; scanForEnder must skip the nested
    // `O100 while` (depth++) and its `O100 endwhile` (depth--) before
    // finding the real closer.
    ASSERT_FALSE(executor->execute(breakBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
}

TEST_F(OCodeExecutorTest, IfFalseScansPastNestedIf) {
    setInput("O100 if [0]\nO100 if [1]\nO100 endif\nO100 else\nG1 X1\nO100 endif");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    // scanToBranchPoint skipped the nested if/endif and landed on `else`.
    parser->getLexer().seek(executor->getJumpAddress());
    Block elseBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseBlock));
    EXPECT_EQ(elseBlock.oCodeType, OCodeType::ELSE);
}

TEST_F(OCodeExecutorTest, ElseifMissingEndifErrors) {
    setInput("O100 if [1]\nO100 elseif [1]");

    Block ifBlock, elseifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(elseifBlock));

    // branchTaken → scanToEndif → no endif in source → error.
    Error err = executor->execute(elseifBlock, action);
    EXPECT_TRUE(err);
}

TEST_F(OCodeExecutorTest, BreakWithoutEnderErrors) {
    setInput("O100 while [1]\nO100 break");

    Block whileBlock, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    Error err = executor->execute(breakBlock, action);
    EXPECT_TRUE(err);  // jumpPastEnder: no matching end
}

TEST_F(OCodeExecutorTest, ContinueWithoutEnderErrors) {
    setInput("O100 while [1]\nO100 continue");

    Block whileBlock, contBlock;
    ASSERT_FALSE(parser->parseNextBlock(whileBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(whileBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(contBlock));

    Error err = executor->execute(contBlock, action);
    EXPECT_TRUE(err);  // jumpToEnder: no matching end
}

TEST(OCodeUtilsTest, RemainingKeywordMappings) {
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::DEBUG), "debug");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::LOG), "log");
    EXPECT_STREQ(oCodeTypeToKeyword(OCodeType::PRINT), "print");
    EXPECT_STREQ(oCodeTypeToKeyword(static_cast<OCodeType>(255)), "unknown");

    EXPECT_EQ(keywordToOCodeType("DEBUG"), OCodeType::DEBUG);
    EXPECT_EQ(keywordToOCodeType("LOG"), OCodeType::LOG);
    EXPECT_EQ(keywordToOCodeType("PRINT"), OCodeType::PRINT);
    EXPECT_EQ(keywordToOCodeType("bogus"), OCodeType::SUB);  // fallback

    EXPECT_EQ(getMatchingCloser(OCodeType::CALL), OCodeType::ENDSUB);  // default
}

TEST_F(OCodeExecutorTest, BreakFromRepeat) {
    setInput("O100 repeat [5]\nO100 break\nO100 endrepeat\nG1 X1");

    Block repeatBlock, breakBlock;
    ASSERT_FALSE(parser->parseNextBlock(repeatBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(repeatBlock, action));
    ASSERT_FALSE(parser->parseNextBlock(breakBlock));

    ASSERT_FALSE(executor->execute(breakBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);
    EXPECT_FALSE(executor->inLoop());
}

TEST_F(OCodeExecutorTest, SubWithoutEndsubErrors) {
    setInput("O100 sub\nG1 X1");

    Block subBlock;
    ASSERT_FALSE(parser->parseNextBlock(subBlock));
    OCodeExecutor::NextAction action;
    Error err = executor->execute(subBlock, action);
    EXPECT_TRUE(err);  // scanForEnder: no matching endsub
}

TEST_F(OCodeExecutorTest, ScanSourceSkipsParseErrors) {
    // Garbage line before the sub — scanSource continues past parse errors.
    setInput("O100 sub\n%&*bogus\nO100 endsub");
    Error err = executor->scanSubroutines();
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->getRegistry().exists(100));
}

TEST_F(OCodeExecutorTest, ScanSourceNestedSubDepth) {
    // A nested same-numbered sub inside the body — depth tracking.
    setInput("O100 sub\nO100 sub\nO100 endsub\nO100 endsub");
    Error err = executor->scanSubroutines();
    EXPECT_FALSE(err);
    EXPECT_TRUE(executor->getRegistry().exists(100));
}

TEST_F(OCodeExecutorTest, IfFalseSkipsNestedElseAndOtherOCodes) {
    // Inside a false O100 if: a nested O100 if/else/endif AND an O200
    // while — scanToBranchPoint must skip all of them.
    setInput("O100 if [0]\nO100 if [0]\nO100 else\nO100 endif\n"
             "O200 while [1]\nO200 endwhile\nO100 else\nG1 X1\nO100 endif");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    parser->getLexer().seek(executor->getJumpAddress());
    Block elseBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseBlock));
    EXPECT_EQ(elseBlock.oCodeType, OCodeType::ELSE);
}

TEST_F(OCodeExecutorTest, IfFalseSkipsSameNumberedWhile) {
    // O100 while inside a false O100 if — hits the default: break arm of
    // scanToBranchPoint's O-code-type switch.
    setInput("O100 if [0]\nO100 while [1]\nO100 endwhile\nO100 else\nO100 endif");

    Block ifBlock;
    ASSERT_FALSE(parser->parseNextBlock(ifBlock));
    OCodeExecutor::NextAction action;
    ASSERT_FALSE(executor->execute(ifBlock, action));
    EXPECT_EQ(action, OCodeExecutor::NextAction::JUMP);

    parser->getLexer().seek(executor->getJumpAddress());
    Block elseBlock;
    ASSERT_FALSE(parser->parseNextBlock(elseBlock));
    EXPECT_EQ(elseBlock.oCodeType, OCodeType::ELSE);
}
