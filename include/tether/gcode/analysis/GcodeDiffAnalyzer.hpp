/// @file GcodeDiffAnalyzer.hpp
/// @brief Naive line-by-line G-code diff analyzer.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace tether::gcode::analysis {

struct DiffLine {
    int32_t lineNumber;
    std::string content;
};

struct DiffModified {
    int32_t oldLineNumber;
    int32_t newLineNumber;
    std::string oldContent;
    std::string newContent;
};

struct DiffWordChange {
    int32_t lineNumber;
    std::string word;
    std::string oldValue;
    std::string newValue;
};

struct DiffSummary {
    int32_t totalAdded = 0;
    int32_t totalRemoved = 0;
    int32_t totalModified = 0;
    int32_t totalUnchanged = 0;
    double similarityScore = 1.0;
};

struct GcodeDiffResult {
    std::vector<DiffLine> added;
    std::vector<DiffLine> removed;
    std::vector<DiffModified> modified;
    int32_t unchanged = 0;
    DiffSummary summary;
    std::vector<DiffWordChange> wordChanges;
};

class GcodeDiffAnalyzer {
public:
    static GcodeDiffResult diff(const std::vector<std::string>& oldLines,
                                const std::vector<std::string>& newLines);
};

} // namespace tether::gcode::analysis
