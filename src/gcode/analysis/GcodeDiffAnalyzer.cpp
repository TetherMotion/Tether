/// @file GcodeDiffAnalyzer.cpp
/// @brief Naive line-by-line G-code diff with word-level change detection.

#include "tether/gcode/analysis/GcodeDiffAnalyzer.hpp"
#include "AnalysisUtil.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <map>
#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace tether::gcode::analysis {

namespace {

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    if (text.empty()) {
        lines.emplace_back("");
        return lines;
    }
    size_t start = 0;
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\n') {
            lines.emplace_back(text.substr(start, i - start));
            start = i + 1;
        }
    }
    lines.emplace_back(text.substr(start));
    return lines;
}

char toUpperChar(char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

std::map<char, std::string> tokenizeWords(const std::string& line) {
    static const std::regex wordRegex(R"(([A-Za-z])(-?\d*\.?\d+))");
    std::map<char, std::string> words;
    for (std::sregex_iterator it(line.begin(), line.end(), wordRegex), end; it != end; ++it) {
        const std::smatch& m = *it;
        char letter = toUpperChar(m[1].str()[0]);
        words[letter] = m[0].str();
    }
    return words;
}

} // namespace

GcodeDiffResult GcodeDiffAnalyzer::diff(const std::vector<std::string>& oldLines,
                                        const std::vector<std::string>& newLines) {
    GcodeDiffResult result;

    const size_t oldSize = oldLines.size();
    const size_t newSize = newLines.size();
    const size_t minLen = std::min(oldSize, newSize);
    const size_t maxLen = std::max(oldSize, newSize);

    int32_t unchanged = 0;
    std::vector<DiffModified> modified;
    std::vector<DiffWordChange> wordChanges;

    for (size_t i = 0; i < minLen; ++i) {
        const std::string oldTrimmed = trim(oldLines[i]);
        const std::string newTrimmed = trim(newLines[i]);
        if (oldTrimmed == newTrimmed) {
            ++unchanged;
        } else {
            modified.push_back(DiffModified{
                static_cast<int32_t>(i),
                static_cast<int32_t>(i),
                oldLines[i],
                newLines[i],
            });

            auto oldWords = tokenizeWords(oldTrimmed);
            auto newWords = tokenizeWords(newTrimmed);
            std::vector<char> keys;
            keys.reserve(oldWords.size() + newWords.size());
            for (const auto& [k, _] : oldWords) keys.push_back(k);
            for (const auto& [k, _] : newWords) keys.push_back(k);
            std::sort(keys.begin(), keys.end());
            keys.erase(std::unique(keys.begin(), keys.end()), keys.end());

            for (char key : keys) {
                const auto oldIt = oldWords.find(key);
                const auto newIt = newWords.find(key);
                const std::string oldVal = (oldIt != oldWords.end()) ? oldIt->second : "(none)";
                const std::string newVal = (newIt != newWords.end()) ? newIt->second : "(none)";
                if (oldVal != newVal) {
                    wordChanges.push_back(DiffWordChange{
                        static_cast<int32_t>(i),
                        std::string(1, key),
                        oldVal,
                        newVal,
                    });
                }
            }
        }
    }

    for (size_t i = minLen; i < newSize; ++i) {
        result.added.push_back(DiffLine{static_cast<int32_t>(i), newLines[i]});
    }
    for (size_t i = minLen; i < oldSize; ++i) {
        result.removed.push_back(DiffLine{static_cast<int32_t>(i), oldLines[i]});
    }

    result.modified = std::move(modified);
    result.wordChanges = std::move(wordChanges);
    result.unchanged = unchanged;

    result.summary.totalAdded = static_cast<int32_t>(result.added.size());
    result.summary.totalRemoved = static_cast<int32_t>(result.removed.size());
    result.summary.totalModified = static_cast<int32_t>(result.modified.size());
    result.summary.totalUnchanged = unchanged;
    result.summary.similarityScore = (maxLen > 0)
        ? static_cast<double>(unchanged) / static_cast<double>(maxLen)
        : 1.0;

    return result;
}

} // namespace tether::gcode::analysis
