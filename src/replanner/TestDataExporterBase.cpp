/**
 * @file TestDataExporterBase.cpp
 * @brief Base exporter classes, JSON builder, and streaming exporter
 */

#include "tether/motion_replanner/TestDataExporter.hpp"
#include <ctime>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <cmath>

namespace MotionReplanner {

//=============================================================================
// DataExporter Base Implementation
//=============================================================================

DataExporter::DataExporter(const ExportConfig& config) : config_(config) {}

std::string DataExporter::formatDouble(double value) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(config_.precision) << value;
    return ss.str();
}

std::string DataExporter::escapeCSV(const std::string& str) const {
    if (str.find(config_.delimiter) != std::string::npos ||
        str.find('"') != std::string::npos ||
        str.find('\n') != std::string::npos) {
        std::string escaped = str;
        size_t pos = 0;
        while ((pos = escaped.find('"', pos)) != std::string::npos) {
            escaped.insert(pos, "\"");
            pos += 2;
        }
        return "\"" + escaped + "\"";
    }
    return str;
}

std::string DataExporter::escapeJSON(const std::string& str) const {
    std::string result;
    for (char c : str) {
        switch (c) {
            case '"':  result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default: result += c;
        }
    }
    return result;
}

std::string DataExporter::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&time), config_.timestampFormat.c_str());
    return ss.str();
}

//=============================================================================
// StreamingExporter Implementation
//=============================================================================

StreamingExporter::StreamingExporter(const std::string& filename, const ExportConfig& config)
    : filename_(filename), config_(config), bytesWritten_(0), headerWritten_(false) {}

StreamingExporter::~StreamingExporter() {
    close();
}

bool StreamingExporter::open() {
    file_.open(filename_);
    return file_.is_open();
}

void StreamingExporter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

void StreamingExporter::writeHeader(const std::vector<std::string>& columns) {
    if (!file_.is_open() || headerWritten_) return;
    
    for (size_t i = 0; i < columns.size(); ++i) {
        if (i > 0) file_ << config_.delimiter;
        file_ << columns[i];
    }
    file_ << "\n";
    
    headerWritten_ = true;
    bytesWritten_ = file_.tellp();
}

void StreamingExporter::writeRow(const std::vector<double>& values) {
    if (!file_.is_open()) return;
    
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) file_ << config_.delimiter;
        file_ << std::fixed << std::setprecision(config_.precision) << values[i];
    }
    file_ << "\n";
    
    bytesWritten_ = file_.tellp();
}

void StreamingExporter::writeRow(const std::vector<std::string>& values) {
    if (!file_.is_open()) return;
    
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) file_ << config_.delimiter;
        file_ << values[i];
    }
    file_ << "\n";
    
    bytesWritten_ = file_.tellp();
}

void StreamingExporter::writeSample(const GCodeExport::TrajectorySample& desired,
                                    const GCodeExport::TrajectorySample& actual) {
    if (!file_.is_open()) return;
    
    if (!headerWritten_) {
        writeHeader({"time", "dx", "dy", "dz", "dvx", "dvy", "dvz",
                     "ax", "ay", "az", "avx", "avy", "avz"});
    }
    
    writeRow({desired.time,
              desired.position[0], desired.position[1], desired.position[2],
              desired.velocity[0], desired.velocity[1], desired.velocity[2],
              actual.position[0], actual.position[1], actual.position[2],
              actual.velocity[0], actual.velocity[1], actual.velocity[2]});
}

void StreamingExporter::writeError(const TrackingError& error) {
    if (!file_.is_open()) return;
    
    if (!headerWritten_) {
        writeHeader({"time", "ex", "ey", "ez", "emag", "verr", "lag", "contour", "critical"});
    }
    
    file_ << std::fixed << std::setprecision(config_.precision)
          << error.timestamp << config_.delimiter
          << error.positionError[0] << config_.delimiter
          << error.positionError[1] << config_.delimiter
          << error.positionError[2] << config_.delimiter
          << error.combinedPositionError << config_.delimiter
          << error.velocityError[0] << config_.delimiter
          << error.lagError << config_.delimiter
          << error.contourError << config_.delimiter
          << (error.isCriticalPoint ? 1 : 0) << "\n";
    
    bytesWritten_ = file_.tellp();
}

void StreamingExporter::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

//=============================================================================
// JSONBuilder Implementation
//=============================================================================

JSONBuilder::JSONBuilder(bool pretty)
    : pretty_(pretty), indent_(0), needComma_(false) {}

void JSONBuilder::newline() {
    if (pretty_) {
        ss_ << "\n" << std::string(indent_ * 2, ' ');
    }
}

void JSONBuilder::comma() {
    if (needComma_) {
        ss_ << ",";
    }
    needComma_ = true;
}

void JSONBuilder::beginObject() {
    comma();
    ss_ << "{";
    indent_++;
    needComma_ = false;
}

void JSONBuilder::endObject() {
    indent_--;
    newline();
    ss_ << "}";
    needComma_ = true;
}

void JSONBuilder::beginArray() {
    comma();
    ss_ << "[";
    indent_++;
    needComma_ = false;
}

void JSONBuilder::endArray() {
    indent_--;
    if (pretty_) ss_ << "\n" << std::string(indent_ * 2, ' ');
    ss_ << "]";
    needComma_ = true;
}

void JSONBuilder::key(const std::string& k) {
    comma();
    newline();
    ss_ << "\"" << k << "\": ";
    needComma_ = false;
}

void JSONBuilder::value(const std::string& v) {
    comma();
    if (pretty_ && needComma_) newline();
    ss_ << "\"" << v << "\"";
}

void JSONBuilder::value(double v) {
    comma();
    if (pretty_ && needComma_) newline();
    if (std::isnan(v) || std::isinf(v)) {
        ss_ << "null";
    } else {
        ss_ << std::fixed << std::setprecision(6) << v;
    }
}

void JSONBuilder::value(int v) {
    comma();
    if (pretty_ && needComma_) newline();
    ss_ << v;
}

void JSONBuilder::value(bool v) {
    comma();
    if (pretty_ && needComma_) newline();
    ss_ << (v ? "true" : "false");
}

void JSONBuilder::valueNull() {
    comma();
    if (pretty_ && needComma_) newline();
    ss_ << "null";
}

void JSONBuilder::keyValue(const std::string& k, const std::string& v) {
    key(k);
    ss_ << "\"" << v << "\"";
    needComma_ = true;
}

void JSONBuilder::keyValue(const std::string& k, double v) {
    key(k);
    if (std::isnan(v) || std::isinf(v)) {
        ss_ << "null";
    } else {
        ss_ << std::fixed << std::setprecision(6) << v;
    }
    needComma_ = true;
}

void JSONBuilder::keyValue(const std::string& k, int v) {
    key(k);
    ss_ << v;
    needComma_ = true;
}

void JSONBuilder::keyValue(const std::string& k, bool v) {
    key(k);
    ss_ << (v ? "true" : "false");
    needComma_ = true;
}

void JSONBuilder::doubleArray(const std::string& k, const std::vector<double>& values) {
    key(k);
    doubleArray(values);
}

void JSONBuilder::doubleArray(const std::vector<double>& values) {
    ss_ << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) ss_ << ", ";
        if (std::isnan(values[i]) || std::isinf(values[i])) {
            ss_ << "null";
        } else {
            ss_ << std::fixed << std::setprecision(6) << values[i];
        }
    }
    ss_ << "]";
    needComma_ = true;
}

} // namespace MotionReplanner
