/**
 * @file simple_gcode_example.cpp
 * @brief Simple example demonstrating G-code parsing with the tether_gcode library
 * 
 * This example shows how to:
 * - Parse G-code using the GCode namespace (full API)
 * - Process blocks and extract motion information
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>

#include <tether/gcode/GCodeParser.hpp>
#include <tether/gcode/GCodeTypes.hpp>
#include <tether/gcode/GCodeLexer.hpp>

int main() {
    std::cout << "=== Tether G-Code Example ===\n\n";
    
    // Sample G-code
    const char* gcode = R"(
G21 ; Metric mode
G90 ; Absolute positioning
G0 X0 Y0 Z5 ; Rapid to start
G1 Z-1 F500 ; Plunge
G1 X100 Y0 F1000 ; Line
G1 X100 Y100 ; Square corner
G1 X0 Y100
G1 X0 Y0
G0 Z5 ; Retract
M30 ; End program
)";
    
    // Parse G-code using the full API
    GCode::VariableSystem variables;
    GCode::Lexer lexer;
    GCode::Parser parser(variables);
    
    std::vector<GCode::Block> blocks;
    std::istringstream stream(gcode);
    std::string line;
    
    while (std::getline(stream, line)) {
        if (line.empty()) continue;
        
        GCode::Block block;
        GCode::Error err = parser.parseLine(line.c_str(), block);
        if (err) {
            std::cerr << "Failed to parse line: " << err.message.data() << "\n";
            return 1;
        }
        
        blocks.push_back(std::move(block));
    }
    
    std::cout << "Parsed " << blocks.size() << " G-code blocks\n";
    
    // Print parsed blocks
    std::cout << "\nParsed blocks:\n";
    for (size_t i = 0; i < blocks.size(); ++i) {
        std::cout << "  " << i << ": " << blocks[i].originalText.data() << "\n";
    }
    
    // Process blocks to extract motion segments
    struct MotionSegment {
        GCode::Position start;
        GCode::Position end;
        double segmentLength;
        double feedRate;
        bool isRapid;
        GCode::MotionMode motionType;
    };
    
    std::vector<MotionSegment> segments;
    GCode::Position currentPos{};
    GCode::MotionMode activeMotionMode = GCode::MotionMode::LINEAR;
    double activeFeedRate = 1000.0;  // default feed
    
    for (const auto& block : blocks) {
        // Update motion mode from G-codes
        for (uint8_t i = 0; i < block.gCodeCount; ++i) {
            int16_t g = block.gCodes[i];
            if (g == 0) activeMotionMode = GCode::MotionMode::RAPID;
            else if (g == 1) activeMotionMode = GCode::MotionMode::LINEAR;
            else if (g == 2) activeMotionMode = GCode::MotionMode::CW_ARC;
            else if (g == 3) activeMotionMode = GCode::MotionMode::CCW_ARC;
        }
        
        // Get feed rate if specified
        if (block.hasWord(GCode::WordLetter::F)) {
            activeFeedRate = block.getWord(GCode::WordLetter::F);
        }
        
        // Build target position from axis words
        GCode::Position targetPos = currentPos;
        bool hasAxisWords = false;
        
        if (block.hasWord(GCode::WordLetter::X)) { targetPos.coords[0] = block.getWord(GCode::WordLetter::X); hasAxisWords = true; }
        if (block.hasWord(GCode::WordLetter::Y)) { targetPos.coords[1] = block.getWord(GCode::WordLetter::Y); hasAxisWords = true; }
        if (block.hasWord(GCode::WordLetter::Z)) { targetPos.coords[2] = block.getWord(GCode::WordLetter::Z); hasAxisWords = true; }
        
        if (hasAxisWords) {
            MotionSegment seg;
            seg.start = currentPos;
            seg.end = targetPos;
            
            double dx = seg.end.coords[0] - seg.start.coords[0];
            double dy = seg.end.coords[1] - seg.start.coords[1];
            double dz = seg.end.coords[2] - seg.start.coords[2];
            seg.segmentLength = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            seg.isRapid = (activeMotionMode == GCode::MotionMode::RAPID);
            seg.feedRate = seg.isRapid ? 6000.0 : activeFeedRate;
            seg.motionType = activeMotionMode;
            
            segments.push_back(seg);
            currentPos = targetPos;
        }
    }
    
    std::cout << "\nGenerated " << segments.size() << " motion segments\n";
    
    // Print motion segments
    std::cout << "\nMotion segments:\n";
    double totalLength = 0.0;
    double totalTime = 0.0;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        const char* typeStr = seg.isRapid ? "RAPID" : 
                              (seg.motionType == GCode::MotionMode::LINEAR ? "LINEAR" :
                               (seg.motionType == GCode::MotionMode::CW_ARC ? "ARC_CW" :
                                (seg.motionType == GCode::MotionMode::CCW_ARC ? "ARC_CCW" : "OTHER")));
        
        double segTime = (seg.segmentLength > 0 && seg.feedRate > 0) 
            ? (seg.segmentLength / seg.feedRate * 60.0) : 0.0;
        
        std::cout << "  " << i << ": " << typeStr 
                  << " X" << seg.end.coords[0] 
                  << " Y" << seg.end.coords[1]
                  << " Z" << seg.end.coords[2]
                  << " len=" << std::fixed << std::setprecision(2) << seg.segmentLength << "mm"
                  << " t=" << std::setprecision(3) << segTime << "s"
                  << "\n";
        
        totalLength += seg.segmentLength;
        totalTime += segTime;
    }
    
    std::cout << "\nTotal path length: " << std::fixed << std::setprecision(2) << totalLength << " mm\n";
    std::cout << "Estimated machining time: " << std::setprecision(2) << totalTime << " seconds\n";
    
    std::cout << "\nDone!\n";
    return 0;
}
