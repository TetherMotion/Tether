/**
 * @file TestDataExporterTestResult.cpp
 * @brief Test result exporter implementation
 */

#include "tether/motion_replanner/TestDataExporter.hpp"
#include <algorithm>

namespace MotionReplanner {

//=============================================================================
// TestResultExporter Implementation
//=============================================================================

bool TestResultExporter::exportTestResult(const std::string& filename, const TestResult& result) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    if (config_.format == ExportFormat::JSON || config_.format == ExportFormat::JSONPretty) {
        writeJSONTestResult(file, result);
    } else {
        writeCSVTestResult(file, result);
    }
    
    return file.good();
}

void TestResultExporter::writeCSVTestResult(std::ostream& out, const TestResult& result) {
    char d = config_.delimiter;
    
    // Metadata section
    out << "# Test: " << result.testName << "\n";
    out << "# Type: " << result.testType << "\n";
    out << "# Passed: " << (result.passed ? "Yes" : "No") << "\n\n";
    
    // Summary statistics
    out << "Metric" << d << "Value\n";
    out << "MaxError" << d << formatDouble(result.positionError.maxError) << "\n";
    out << "MeanError" << d << formatDouble(result.positionError.meanError) << "\n";
    out << "RMSError" << d << formatDouble(result.positionError.rmsError) << "\n";
    out << "SettlingTime" << d << formatDouble(result.settlingTime) << "\n";
    out << "MaxVelocity" << d << formatDouble(result.maxVelocityAchieved) << "\n\n";
    
    // Trajectory data
    out << "time" << d
        << "desired_x" << d << "desired_y" << d << "desired_z" << d
        << "actual_x" << d << "actual_y" << d << "actual_z\n";
    
    size_t n = std::min(result.desiredSamples.size(), result.actualSamples.size());
    for (size_t i = 0; i < n; ++i) {
        const auto& des = result.desiredSamples[i];
        const auto& act = result.actualSamples[i];
        
        out << formatDouble(des.timestamp) << d
            << formatDouble(des.position[0]) << d
            << formatDouble(des.position[1]) << d
            << formatDouble(des.position[2]) << d
            << formatDouble(act.position[0]) << d
            << formatDouble(act.position[1]) << d
            << formatDouble(act.position[2]) << "\n";
    }
}

void TestResultExporter::writeJSONTestResult(std::ostream& out, const TestResult& result) {
    bool pretty = (config_.format == ExportFormat::JSONPretty);
    JSONBuilder json(pretty);
    
    json.beginObject();
    
    json.keyValue("testName", result.testName);
    json.keyValue("testType", result.testType);
    json.keyValue("passed", result.passed);
    
    json.key("statistics");
    json.beginObject();
    json.keyValue("maxPositionError", result.positionError.maxError);
    json.keyValue("meanPositionError", result.positionError.meanError);
    json.keyValue("rmsError", result.positionError.rmsError);
    json.keyValue("settlingTime", result.settlingTime);
    json.keyValue("maxVelocity", result.maxVelocityAchieved);
    json.endObject();
    
    // Trajectory arrays
    json.key("desired");
    json.beginObject();
    {
        std::vector<double> t, x, y, z;
        for (const auto& s : result.desiredSamples) {
            t.push_back(s.timestamp);
            x.push_back(s.position[0]);
            y.push_back(s.position[1]);
            z.push_back(s.position[2]);
        }
        json.doubleArray("time", t);
        json.doubleArray("x", x);
        json.doubleArray("y", y);
        json.doubleArray("z", z);
    }
    json.endObject();
    
    json.key("actual");
    json.beginObject();
    {
        std::vector<double> t, x, y, z;
        for (const auto& s : result.actualSamples) {
            t.push_back(s.timestamp);
            x.push_back(s.position[0]);
            y.push_back(s.position[1]);
            z.push_back(s.position[2]);
        }
        json.doubleArray("time", t);
        json.doubleArray("x", x);
        json.doubleArray("y", y);
        json.doubleArray("z", z);
    }
    json.endObject();
    
    json.endObject();
    
    out << json.str();
}

bool TestResultExporter::exportFrictionModel(const std::string& filename,
    const FrictionIdentificationResult& result) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    
    json.key("bestModel");
    json.beginObject();
    json.keyValue("type", result.bestModel.modelName());
    json.keyValue("coulombForce", result.bestModel.coulombForce);
    json.keyValue("staticFriction", result.bestModel.staticFriction);
    json.keyValue("viscousCoeff", result.bestModel.viscousCoeff);
    json.keyValue("stribeckVelocity", result.bestModel.stribeckVelocity);
    json.keyValue("rSquared", result.bestModel.rSquared);
    json.endObject();
    
    json.keyValue("isSymmetric", result.isSymmetric);
    json.keyValue("asymmetryRatio", result.asymmetryRatio);
    
    json.doubleArray("velocities", result.velocities);
    json.doubleArray("forces", result.forces);
    json.doubleArray("fittedForces", result.fittedForces);
    
    json.endObject();
    
    file << json.str();
    return file.good();
}

bool TestResultExporter::exportDelayIdentification(const std::string& filename,
    const DelayIdentificationResult& result) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    json.keyValue("transportDelay", result.transportDelay);
    json.keyValue("confidence", result.delayConfidence);
    json.keyValue("crossCorrelation", result.crossCorrelation);
    json.keyValue("crossCorrelationLag", result.crossCorrelationLag);
    json.keyValue("samplingPeriod", result.samplingPeriod);
    json.keyValue("riseTime", result.riseTime);
    json.keyValue("settlingTime", result.settlingTime);
    json.keyValue("overshoot", result.overshoot);
    json.endObject();
    
    file << json.str();
    return file.good();
}

bool TestResultExporter::exportPIDAssessment(const std::string& filename,
    const PIDTuningAssessment& assessment) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    
    json.key("observedGains");
    json.beginObject();
    json.keyValue("Kp", assessment.observedKp);
    json.keyValue("Ki", assessment.observedKi);
    json.keyValue("Kd", assessment.observedKd);
    json.endObject();
    
    json.key("performance");
    json.beginObject();
    json.keyValue("steadyStateError", assessment.steadyStateError);
    json.keyValue("riseTime", assessment.riseTime);
    json.keyValue("settlingTime", assessment.settlingTime);
    json.keyValue("overshoot", assessment.overshoot);
    json.keyValue("dampingRatio", assessment.dampingRatio);
    json.keyValue("naturalFrequency", assessment.naturalFrequency);
    json.endObject();
    
    json.key("scores");
    json.beginObject();
    json.keyValue("stability", assessment.stabilityScore);
    json.keyValue("response", assessment.responseScore);
    json.keyValue("accuracy", assessment.accuracyScore);
    json.keyValue("overall", assessment.overallScore);
    json.endObject();
    
    json.key("suggestedGains");
    json.beginObject();
    json.keyValue("Kp", assessment.suggestedKp);
    json.keyValue("Ki", assessment.suggestedKi);
    json.keyValue("Kd", assessment.suggestedKd);
    json.endObject();
    
    json.keyValue("tuningAdvice", assessment.tuningAdvice);
    
    json.key("issues");
    json.beginArray();
    for (const auto& issue : assessment.issues) {
        json.value(issue);
    }
    json.endArray();
    
    json.key("recommendations");
    json.beginArray();
    for (const auto& rec : assessment.recommendations) {
        json.value(rec);
    }
    json.endArray();
    
    json.endObject();
    
    file << json.str();
    return file.good();
}

bool TestResultExporter::exportDynamicsModel(const std::string& filename,
    const DynamicsIdentificationResult& result) {
    
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    JSONBuilder json(true);
    
    json.beginObject();
    
    json.key("firstOrder");
    json.beginObject();
    json.keyValue("gain", result.gain);
    json.keyValue("timeConstant", result.timeConstant);
    json.endObject();
    
    json.key("secondOrder");
    json.beginObject();
    json.keyValue("naturalFrequency", result.naturalFrequency);
    json.keyValue("dampingRatio", result.dampingRatio);
    json.endObject();
    
    json.keyValue("systemOrder", result.systemOrder);
    json.keyValue("bandwidthHz", result.bandwidthHz);
    json.keyValue("phaseMarginDeg", result.phaseMarginDeg);
    json.keyValue("gainMarginDb", result.gainMarginDb);
    
    if (!result.frequencies.empty()) {
        json.doubleArray("frequencies", result.frequencies);
        json.doubleArray("magnitudeDb", result.magnitudeDb);
        json.doubleArray("phaseDeg", result.phaseDeg);
    }
    
    json.endObject();
    
    file << json.str();
    return file.good();
}

} // namespace MotionReplanner
