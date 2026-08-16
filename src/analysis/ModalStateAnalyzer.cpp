/**
 * @file ModalStateAnalyzer.cpp
 * @brief Track modal state changes and analyze modal group transitions.
 */
#include "tether/analysis/ModalStateAnalyzer.hpp"

#include <algorithm>
#include <numeric>
#include <format>

namespace tether::analysis {

void ModalStateAnalyzer::checkAndRecord(
    ModalState& state, std::vector<ModalStateChange>& changes,
    std::map<std::string, int>& changeCounts,
    int lineNum, const std::string& property,
    const std::string& newValue) const {

    std::string* currentVal = nullptr;
    if (property == "motionMode") currentVal = &state.motionMode;
    else if (property == "plane") currentVal = &state.plane;
    else if (property == "distanceMode") currentVal = &state.distanceMode;
    else if (property == "units") currentVal = &state.units;
    else if (property == "feedMode") currentVal = &state.feedMode;
    else if (property == "cutterComp") currentVal = &state.cutterComp;
    else if (property == "toolLengthComp") currentVal = &state.toolLengthComp;
    else if (property == "coordSystem") currentVal = &state.coordSystem;
    else if (property == "spindleState") currentVal = &state.spindleState;
    else if (property == "coolant") currentVal = &state.coolant;
    else return;

    if (*currentVal != newValue) {
        changes.push_back({lineNum, property, *currentVal, newValue});
        *currentVal = newValue;
        changeCounts[property]++;
    }
}

ModalStateResult ModalStateAnalyzer::analyze(
    const std::vector<std::string>& gcodeLines) const {

    ModalStateResult result;
    ModalState state;
    std::map<std::string, int>& changeCounts = result.changeCounts;

    // Group tracking for modal groups analysis.
    std::map<std::string, std::string> groupStates = {
        {"motion", "G1"}, {"plane", "G17"}, {"units", "G21"},
        {"distance", "G90"}, {"feed_mode", "G94"}, {"spindle_mode", "G97"},
        {"coolant", "off"}, {"tool_length", "G43"}
    };
    std::map<std::string, ModalGroupState> groups;
    for (const auto& [g, s] : groupStates)
        groups[g] = {g, s, 0, -1};

    // Transition tracking.
    std::map<std::string, std::string> transitionStates = {
        {"motion", "G0"}, {"distance", "G90"}, {"units", "G21"},
        {"feedrate", "G94"}, {"plane", "G17"}, {"wcs", "G54"},
        {"compensation", "G40"}
    };

    for (int i = 0; i < static_cast<int>(gcodeLines.size()); ++i) {
        auto code = stripComments(gcodeLines[i]);
        if (code.empty()) continue;

        // Motion mode
        if (hasWord(code, "G0")) checkAndRecord(state, result.changes, changeCounts, i, "motionMode", "G0");
        if (hasWord(code, "G1")) checkAndRecord(state, result.changes, changeCounts, i, "motionMode", "G1");
        if (hasWord(code, "G2")) checkAndRecord(state, result.changes, changeCounts, i, "motionMode", "G2");
        if (hasWord(code, "G3")) checkAndRecord(state, result.changes, changeCounts, i, "motionMode", "G3");

        // Plane
        if (hasWord(code, "G17")) checkAndRecord(state, result.changes, changeCounts, i, "plane", "G17");
        if (hasWord(code, "G18")) checkAndRecord(state, result.changes, changeCounts, i, "plane", "G18");
        if (hasWord(code, "G19")) checkAndRecord(state, result.changes, changeCounts, i, "plane", "G19");

        // Distance mode
        if (hasWord(code, "G90")) checkAndRecord(state, result.changes, changeCounts, i, "distanceMode", "G90");
        if (hasWord(code, "G91")) checkAndRecord(state, result.changes, changeCounts, i, "distanceMode", "G91");

        // Units
        if (hasWord(code, "G20")) checkAndRecord(state, result.changes, changeCounts, i, "units", "G20");
        if (hasWord(code, "G21")) checkAndRecord(state, result.changes, changeCounts, i, "units", "G21");

        // Feed mode
        if (hasWord(code, "G94")) checkAndRecord(state, result.changes, changeCounts, i, "feedMode", "G94");
        if (hasWord(code, "G95")) checkAndRecord(state, result.changes, changeCounts, i, "feedMode", "G95");

        // Cutter comp
        if (hasWord(code, "G40")) checkAndRecord(state, result.changes, changeCounts, i, "cutterComp", "G40");
        if (hasWord(code, "G41")) checkAndRecord(state, result.changes, changeCounts, i, "cutterComp", "G41");
        if (hasWord(code, "G42")) checkAndRecord(state, result.changes, changeCounts, i, "cutterComp", "G42");

        // Tool length comp
        if (hasWord(code, "G43")) checkAndRecord(state, result.changes, changeCounts, i, "toolLengthComp", "G43");
        if (hasWord(code, "G44")) checkAndRecord(state, result.changes, changeCounts, i, "toolLengthComp", "G44");
        if (hasWord(code, "G49")) checkAndRecord(state, result.changes, changeCounts, i, "toolLengthComp", "G49");

        // Coordinate system
        for (const char* cs : {"G54", "G55", "G56", "G57", "G58", "G59"}) {
            if (hasWord(code, cs)) checkAndRecord(state, result.changes, changeCounts, i, "coordSystem", cs);
        }

        // Spindle
        if (hasWord(code, "M3") || hasWord(code, "M03")) checkAndRecord(state, result.changes, changeCounts, i, "spindleState", "M3");
        if (hasWord(code, "M4") || hasWord(code, "M04")) checkAndRecord(state, result.changes, changeCounts, i, "spindleState", "M4");
        if (hasWord(code, "M5") || hasWord(code, "M05")) checkAndRecord(state, result.changes, changeCounts, i, "spindleState", "M5");

        // Coolant
        if (hasWord(code, "M7") || hasWord(code, "M07")) checkAndRecord(state, result.changes, changeCounts, i, "coolant", "M7");
        if (hasWord(code, "M8") || hasWord(code, "M08")) checkAndRecord(state, result.changes, changeCounts, i, "coolant", "M8");
        if (hasWord(code, "M9") || hasWord(code, "M09")) checkAndRecord(state, result.changes, changeCounts, i, "coolant", "M9");

        // === Modal groups analysis (redundancy) ===
        auto checkGroup = [&](const std::string& word, const std::string& groupName) {
            if (hasWord(code, word)) {
                auto& gs = groups[groupName];
                if (gs.state == word) {
                    result.redundantCommands++;
                } else {
                    gs.state = word;
                    gs.changeCount++;
                    gs.lastChangeLine = i;
                }
            }
        };

        checkGroup("G0", "motion"); checkGroup("G1", "motion");
        checkGroup("G2", "motion"); checkGroup("G3", "motion");
        checkGroup("G17", "plane"); checkGroup("G18", "plane"); checkGroup("G19", "plane");
        checkGroup("G20", "units"); checkGroup("G21", "units");
        checkGroup("G90", "distance"); checkGroup("G91", "distance");

        if (hasWord(code, "M7") || hasWord(code, "M8")) {
            auto& gs = groups["coolant"];
            if (gs.state == "on") result.redundantCommands++;
            else { gs.state = "on"; gs.changeCount++; gs.lastChangeLine = i; }
        }
        if (hasWord(code, "M9")) {
            auto& gs = groups["coolant"];
            if (gs.state == "off") result.redundantCommands++;
            else { gs.state = "off"; gs.changeCount++; gs.lastChangeLine = i; }
        }

        // === Transition tracking ===
        auto checkTransition = [&](const std::string& word, const std::string& groupName) {
            if (hasWord(code, word)) {
                auto& ts = transitionStates[groupName];
                if (ts != word) {
                    result.transitions.push_back({i, groupName, ts, word});
                    result.transitionsPerGroup[groupName]++;
                    ts = word;
                }
            }
        };

        checkTransition("G0", "motion"); checkTransition("G1", "motion");
        checkTransition("G2", "motion"); checkTransition("G3", "motion");
        checkTransition("G90", "distance"); checkTransition("G91", "distance");
        checkTransition("G20", "units"); checkTransition("G21", "units");
        checkTransition("G17", "plane"); checkTransition("G18", "plane"); checkTransition("G19", "plane");
        for (const char* cs : {"G54", "G55", "G56", "G57", "G58", "G59"})
            checkTransition(cs, "wcs");
        checkTransition("G40", "compensation");
        checkTransition("G41", "compensation");
        checkTransition("G42", "compensation");
    }

    // Finalize.
    result.finalState = state;
    result.isProperlyReset = (state.cutterComp == "G40" && state.toolLengthComp == "G49" &&
                              state.spindleState == "M5" && state.coolant == "M9");

    if (state.cutterComp != "G40")
        result.warnings.push_back("Cutter compensation not cancelled (G40) at program end");
    if (state.spindleState != "M5")
        result.warnings.push_back("Spindle not stopped (M5) at program end");
    if (state.coolant != "M9")
        result.warnings.push_back("Coolant not turned off (M9) at program end");
    if (state.units == "G20")
        result.warnings.push_back("Program ends in inch mode (G20)");
    if (changeCounts["units"] > 5)
        result.warnings.push_back("Frequent unit mode changes — potential for errors");

    // Groups.
    for (const auto& [name, gs] : groups)
        result.groups.push_back(gs);
    result.totalChanges = std::accumulate(result.groups.begin(), result.groups.end(), 0,
        [](int s, const ModalGroupState& g) { return s + g.changeCount; });
    result.modalScore = std::max(0.0, 100.0 - result.redundantCommands * 2.0);

    // Transitions.
    result.transitionCount = static_cast<int>(result.transitions.size());
    int maxCount = 0;
    for (const auto& [group, cnt] : result.transitionsPerGroup) {
        if (cnt > maxCount) { maxCount = cnt; result.mostChangedGroup = group; }
    }
    result.stabilityScore = std::max(0.0, 100.0 - result.transitionCount * 2.0);

    // Recommendations.
    result.recommendations.push_back(
        std::format("{} modal changes, {} transitions, {} redundant",
                    result.changes.size(), result.transitionCount, result.redundantCommands));
    if (result.redundantCommands > 10) {
        result.recommendations.push_back(
            std::format("{} redundant modal commands — remove for cleaner code",
                        result.redundantCommands));
    }
    if (!result.isProperlyReset) {
        result.recommendations.push_back("Modal states not properly reset at program end");
    }
    if (result.stabilityScore > 85) {
        result.recommendations.push_back("Stable modal states — minimal transitions");
    }

    return result;
}

} // namespace tether::analysis
