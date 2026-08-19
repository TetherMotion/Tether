/// @file AnalysisTypes.hpp
/// @brief Pure C++ result types for G-code analysis.

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <variant>

namespace tether::gcode::analysis {

/// @brief Severity level for analysis events, aligned with the viewer protobuf.
enum class Severity {
    Info = 0,
    Low = 1,
    Medium = 2,
    High = 3,
};

/// @brief A single named metric value.
struct Metric {
    std::string key;
    std::variant<double, int64_t> value;
};

inline Metric makeMetric(const std::string& key, double value) {
    return Metric{key, value};
}

inline Metric makeMetric(const std::string& key, int64_t value) {
    return Metric{key, value};
}

/// @brief A single analysis event / finding.
struct Event {
    std::string id;
    int32_t lineNumber = 0;
    std::string type;
    Severity severity = Severity::Info;
    std::string message;
    double metricValue = 0.0;
    std::string detailsJson;
};

/// @brief One analysis section (e.g. overhang, first layer, thermal).
struct Section {
    std::string name;
    std::string displayName;
    double score = 0.0;
    std::vector<Metric> metrics;
    std::vector<Event> events;
    std::size_t totalEventCount = 0;
    bool hasMoreEvents = false;
};

/// @brief Common options controlling how much detail each analyzer returns.
struct Options {
    std::string detailLevel = "standard"; ///< "summary", "standard", "full"
    std::size_t topEventLimit = 64;       ///< max top events for standard mode (0 = 64)
};

} // namespace tether::gcode::analysis
