/**
 * @file g64_demo_export.cpp
 * @brief Minimal G-Code visualization helper that parses simple X/Y linear moves
 *        and exports them to SVG using the project's SVGExporter.
 *
 * This example purposely avoids depending on the full motion planner and
 * demonstrates the exporter usage with a minimal set of trajectory samples.
 *
 * Usage:
 *   g64_demo_export <gcode_file> -o <output_base>
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <regex>

#include <tether/export/SVGExporter.hpp>
#include <tether/export/TrajectoryAnalyzer.hpp>

using namespace GCodeExport;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <gcode_file> [-o output_base]" << std::endl;
        return 1;
    }

    std::string infile = argv[1];
    std::string outbase = "outputs/g64_demo";

    for (int i = 2; i + 1 < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-o" && (i + 1) < argc) {
            outbase = argv[++i];
        }
    }

    std::ifstream ifs(infile);
    if (!ifs.is_open()) {
        std::cerr << "Could not open " << infile << std::endl;
        return 2;
    }

    std::vector<TrajectorySample> samples;
    samples.reserve(256);

    std::string line;
    double curX = 0.0, curY = 0.0, curZ = 0.0;
    std::regex reXY("[XY]\\s*([-+]?[0-9]*\\.?[0-9]+)");

    while (std::getline(ifs, line)) {
        // Quick find of X and Y values in the line
        std::smatch m;
        double x = curX, y = curY;
        std::string l = line;
        // find X
        std::regex reX("X\\s*([-+]?[0-9]*\\.?[0-9]+)");
        if (std::regex_search(l, m, reX)) {
            x = std::stod(m[1].str());
        }
        std::regex reY("Y\\s*([-+]?[0-9]*\\.?[0-9]+)");
        if (std::regex_search(l, m, reY)) {
            y = std::stod(m[1].str());
        }

        // If coordinates changed, emit sample
        if (x != curX || y != curY) {
            TrajectorySample s;
            s.position[0] = x; s.position[1] = y; s.position[2] = curZ;
            s.pathPosition = samples.empty() ? 0.0 : samples.back().pathPosition + std::hypot(x - curX, y - curY);
            samples.push_back(s);
            curX = x; curY = y;
        }
    }

    // Ensure outputs dir exists (best-effort)
    system("mkdir -p outputs");

    std::cout << "Parsed samples: " << samples.size() << "\n";
    for (size_t i = 0; i < samples.size() && i < 8; ++i) {
        std::cout << "  sample[" << i << "] = (" << samples[i].position[0] << ", " << samples[i].position[1] << ")\n";
    }

    SVGConfig cfg;
    cfg.width = 1024;
    cfg.height = 1024;
    cfg.autoScale = true;

    SVGExporter svg(cfg);
    std::string filename = outbase + "_parsed.svg";
    if (svg.exportToFile(samples, filename)) {
        std::cout << "Exported parsed trajectory to: " << filename << std::endl;
        return 0;
    }

    std::cerr << "Failed to export SVG" << std::endl;
    return 3;
}
