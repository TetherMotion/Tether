/**
 * @file ModalStateAnalyzer.hpp
 * @brief Track modal state changes and analyze modal group transitions.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/analysis/GcodeParseUtils.hpp"
#include <map>
#include <string>
#include <vector>

namespace tether::analysis {

/// @brief Complete modal state snapshot.
struct ModalState {
    std::string motionMode = "G0";
    std::string plane = "G17";
    std::string distanceMode = "G90";
    std::string units = "G21";
    std::string feedMode = "G94";
    std::string cutterComp = "G40";
    std::string toolLengthComp = "G49";
    std::string coordSystem = "G54";
    std::string spindleState = "M5";
    std::string coolant = "M9";
};

/// @brief A single modal state change.
struct ModalStateChange {
    int lineNumber = 0;
    std::string property;
    std::string oldValue;
    std::string newValue;
};

/// @brief A modal transition (group-level).
struct ModalTransition {
    int line = 0;
    std::string group;
    std::string from;
    std::string to;
};

/// @brief Modal group state (for group analysis).
struct ModalGroupState {
    std::string group;
    std::string state;
    int changeCount = 0;
    int lastChangeLine = -1;
};

/// @brief Combined modal state analysis result.
struct ModalStateResult {
    // From trackModalStates
    ModalState finalState;
    std::vector<ModalStateChange> changes;
    bool isProperlyReset = false;
    std::vector<std::string> warnings;
    std::map<std::string, int> changeCounts;

    // From analyzeModalGroups
    std::vector<ModalGroupState> groups;
    int totalChanges = 0;
    int redundantCommands = 0;
    double modalScore = 100;

    // From analyzeModalStateTransitions
    std::vector<ModalTransition> transitions;
    int transitionCount = 0;
    std::map<std::string, int> transitionsPerGroup;
    std::string mostChangedGroup = "none";
    double stabilityScore = 100;

    std::vector<std::string> recommendations;
};

/// @brief Track and analyze modal state changes in G-code.
class ModalStateAnalyzer {
public:
    /// @brief Analyze modal states, groups, and transitions.
    ModalStateResult analyze(const std::vector<std::string>& gcodeLines) const;

private:
    void checkAndRecord(ModalState& state, std::vector<ModalStateChange>& changes,
                        std::map<std::string, int>& changeCounts,
                        int lineNum, const std::string& property,
                        const std::string& newValue) const;
};

} // namespace tether::analysis
