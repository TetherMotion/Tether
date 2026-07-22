/**
 * @file SVGExporter.cpp
 * @brief SVG export implementation
 */

#include "SVGExporter.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace GCodeExport {

/// Reject filenames containing path traversal sequences or null bytes.
/// This prevents an attacker who controls the filename from writing to
/// arbitrary locations via "../" traversal. Absolute paths are allowed.
static bool isSafeFilename(const std::string& filename) {
    if (filename.empty()) return false;
    if (filename.find('\0') != std::string::npos) return false;
    if (filename.find("../") != std::string::npos) return false;
    if (filename.find("..\\") != std::string::npos) return false;
    return true;
}

SVGExporter::SVGExporter(const SVGConfig& config)
    : config_(config) {}

bool SVGExporter::exportToFile(const std::vector<TrajectorySample>& samples, const std::string& filename) {
    if (!isSafeFilename(filename)) return false;
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    exportToStream(samples, file);
    return file.good();
}

void SVGExporter::exportToStream(const std::vector<TrajectorySample>& samples, std::ostream& out) {
    if (samples.empty()) return;
    
    Bounds bounds = computeBounds(samples);
    
    // Calculate scale and offset
    double dataWidth = bounds.maxX - bounds.minX;
    double dataHeight = bounds.maxY - bounds.minY;
    
    if (dataWidth < 1e-9) dataWidth = 1.0;
    if (dataHeight < 1e-9) dataHeight = 1.0;
    
    double availWidth = config_.width - 2 * config_.margin;
    double availHeight = config_.height - 2 * config_.margin;
    
    double scale = config_.autoScale ? 
        std::min(availWidth / dataWidth, availHeight / dataHeight) : 1.0;
    
    double offsetX = config_.margin - bounds.minX * scale + 
                     (availWidth - dataWidth * scale) / 2;
    double offsetY = config_.margin - bounds.minY * scale + 
                     (availHeight - dataHeight * scale) / 2;
    
    writeHeader(out, bounds);
    
    if (config_.showGrid) {
        writeGrid(out, bounds, scale, offsetX, offsetY);
    }
    
    writePath(out, samples, scale, offsetX, offsetY);
    writeMarkers(out, samples, scale, offsetX, offsetY);
    
    writeFooter(out);
}

bool SVGExporter::exportSegments(const std::vector<GCode::PlanningSegment>& segments, const std::string& filename) {
    // Convert segments to samples for export
    TrajectoryAnalyzer analyzer;
    auto samples = analyzer.analyze(segments, nullptr);
    return exportToFile(samples, filename);
}

SVGExporter::Bounds SVGExporter::computeBounds(const std::vector<TrajectorySample>& samples) {
    Bounds b{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::lowest()
    };
    
    for (const auto& s : samples) {
        double x = s.position[config_.primaryAxis1];
        double y = s.position[config_.primaryAxis2];
        
        b.minX = std::min(b.minX, x);
        b.maxX = std::max(b.maxX, x);
        b.minY = std::min(b.minY, y);
        b.maxY = std::max(b.maxY, y);
    }
    
    // Add small margin if bounds are equal
    if (b.maxX - b.minX < 1e-9) {
        b.minX -= 1.0;
        b.maxX += 1.0;
    }
    if (b.maxY - b.minY < 1e-9) {
        b.minY -= 1.0;
        b.maxY += 1.0;
    }
    
    return b;
}

void SVGExporter::writeHeader(std::ostream& out, const Bounds& bounds) {
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << config_.width << "\" "
        << "height=\"" << config_.height << "\" "
        << "viewBox=\"0 0 " << config_.width << " " << config_.height << "\">\n";
    
    // Background
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    
    // Title
    out << "  <title>G-code Toolpath</title>\n";
    
    // Style definitions
    out << "  <defs>\n";
    out << "    <marker id=\"arrow\" markerWidth=\"10\" markerHeight=\"10\" "
        << "refX=\"8\" refY=\"3\" orient=\"auto\" markerUnits=\"strokeWidth\">\n";
    out << "      <path d=\"M0,0 L0,6 L9,3 z\" fill=\"#666\"/>\n";
    out << "    </marker>\n";
    out << "  </defs>\n";
    
    // Metadata comment
    out << "  <!-- Bounds: X[" << std::fixed << std::setprecision(3) 
        << bounds.minX << ", " << bounds.maxX << "] Y[" 
        << bounds.minY << ", " << bounds.maxY << "] -->\n";
}

void SVGExporter::writeGrid(std::ostream& out, const Bounds& bounds, 
                            double scale, double offsetX, double offsetY) {
    out << "  <g id=\"grid\" stroke=\"" << config_.gridColor << "\" stroke-width=\"0.5\">\n";
    
    double spacing = config_.gridSpacing;
    
    // Vertical lines
    double startX = std::floor(bounds.minX / spacing) * spacing;
    for (double x = startX; x <= bounds.maxX; x += spacing) {
        double sx = transformX(x, scale, offsetX);
        double sy1 = transformY(bounds.minY, scale, offsetY);
        double sy2 = transformY(bounds.maxY, scale, offsetY);
        out << "    <line x1=\"" << sx << "\" y1=\"" << sy1 
            << "\" x2=\"" << sx << "\" y2=\"" << sy2 << "\"/>\n";
    }
    
    // Horizontal lines
    double startY = std::floor(bounds.minY / spacing) * spacing;
    for (double y = startY; y <= bounds.maxY; y += spacing) {
        double sy = transformY(y, scale, offsetY);
        double sx1 = transformX(bounds.minX, scale, offsetX);
        double sx2 = transformX(bounds.maxX, scale, offsetX);
        out << "    <line x1=\"" << sx1 << "\" y1=\"" << sy 
            << "\" x2=\"" << sx2 << "\" y2=\"" << sy << "\"/>\n";
    }
    
    out << "  </g>\n";
}

void SVGExporter::writePath(std::ostream& out, const std::vector<TrajectorySample>& samples,
                            double scale, double offsetX, double offsetY) {
    out << "  <g id=\"toolpath\">\n";
    
    // Group samples by motion type and create paths
    size_t i = 0;
    while (i < samples.size()) {
        uint8_t currentType = samples[i].motionType;
        bool isRapid = (currentType == 0);
        
        if (isRapid && !config_.showRapids) {
            ++i;
            continue;
        }
        
        // Determine color
        std::string color;
        double strokeWidth;
        std::string dashArray;
        
        if (config_.colorByVelocity && !isRapid) {
            color = velocityToColor(samples[i].linearVelocity);
        } else {
            switch (currentType) {
                case 0: color = config_.rapidColor; break;
                case 1: color = config_.linearColor; break;
                case 2: color = config_.arcCWColor; break;
                case 3: color = config_.arcCCWColor; break;
                default: color = config_.linearColor; break;
            }
        }
        
        strokeWidth = isRapid ? config_.rapidStrokeWidth : config_.feedStrokeWidth;
        dashArray = isRapid ? config_.rapidDash : "";
        
        // Start path
        out << "    <path d=\"M";
        
        double x = transformX(samples[i].position[config_.primaryAxis1], scale, offsetX);
        double y = transformY(samples[i].position[config_.primaryAxis2], scale, offsetY);
        out << std::fixed << std::setprecision(2) << x << "," << y;
        
        // Continue path while same motion type
        while (i < samples.size() && samples[i].motionType == currentType) {
            x = transformX(samples[i].position[config_.primaryAxis1], scale, offsetX);
            y = transformY(samples[i].position[config_.primaryAxis2], scale, offsetY);
            out << " L" << x << "," << y;
            ++i;
        }
        
        out << "\" stroke=\"" << color << "\" stroke-width=\"" << strokeWidth 
            << "\" fill=\"none\"";
        
        if (!dashArray.empty()) {
            out << " stroke-dasharray=\"" << dashArray << "\"";
        }
        
        if (config_.showDirectionArrows) {
            out << " marker-mid=\"url(#arrow)\"";
        }
        
        out << "/>\n";
    }
    
    out << "  </g>\n";
}

void SVGExporter::writeMarkers(std::ostream& out, const std::vector<TrajectorySample>& samples,
                                double scale, double offsetX, double offsetY) {
    if (samples.empty()) return;
    
    out << "  <g id=\"markers\">\n";
    
    // Start point
    if (config_.showStartPoint) {
        double x = transformX(samples.front().position[config_.primaryAxis1], scale, offsetX);
        double y = transformY(samples.front().position[config_.primaryAxis2], scale, offsetY);
        out << "    <circle cx=\"" << x << "\" cy=\"" << y 
            << "\" r=\"5\" fill=\"#00ff00\" stroke=\"#006600\"/>\n";
        out << "    <text x=\"" << (x + 8) << "\" y=\"" << (y + 4) 
            << "\" font-size=\"10\" fill=\"#006600\">Start</text>\n";
    }
    
    // End point
    if (config_.showEndPoint) {
        double x = transformX(samples.back().position[config_.primaryAxis1], scale, offsetX);
        double y = transformY(samples.back().position[config_.primaryAxis2], scale, offsetY);
        out << "    <circle cx=\"" << x << "\" cy=\"" << y 
            << "\" r=\"5\" fill=\"#ff0000\" stroke=\"#660000\"/>\n";
        out << "    <text x=\"" << (x + 8) << "\" y=\"" << (y + 4) 
            << "\" font-size=\"10\" fill=\"#660000\">End</text>\n";
    }
    
    out << "  </g>\n";
}

void SVGExporter::writeFooter(std::ostream& out) {
    out << "</svg>\n";
}

std::string SVGExporter::velocityToColor(double velocity) {
    // Map velocity to color (blue -> green -> yellow -> red)
    double normalized = (velocity - config_.velocityColorMin) / 
                       (config_.velocityColorMax - config_.velocityColorMin);
    normalized = std::max(0.0, std::min(1.0, normalized));
    
    int r, g, b;
    if (normalized < 0.5) {
        // Blue to green
        double t = normalized * 2.0;
        r = 0;
        g = static_cast<int>(255 * t);
        b = static_cast<int>(255 * (1.0 - t));
    } else {
        // Green to red
        double t = (normalized - 0.5) * 2.0;
        r = static_cast<int>(255 * t);
        g = static_cast<int>(255 * (1.0 - t));
        b = 0;
    }
    
    std::ostringstream oss;
    oss << "#" << std::hex << std::setfill('0')
        << std::setw(2) << r
        << std::setw(2) << g
        << std::setw(2) << b;
    return oss.str();
}

std::string SVGExporter::escapeXml(const std::string& str) {
    std::string result;
    for (char c : str) {
        switch (c) {
            case '&': result += "&amp;"; break;
            case '<': result += "&lt;"; break;
            case '>': result += "&gt;"; break;
            case '"': result += "&quot;"; break;
            case '\'': result += "&apos;"; break;
            default: result += c; break;
        }
    }
    return result;
}

double SVGExporter::transformX(double x, double scale, double offset) const {
    return x * scale + offset;
}

double SVGExporter::transformY(double y, double scale, double offset) const {
    if (config_.flipY) {
        return config_.height - (y * scale + offset);
    }
    return y * scale + offset;
}

bool SVGExporter::exportBezierPaths(const std::vector<GCodeExport::RenderableBezierPath>& paths, const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) return false;
    
    // Bounds calculation
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    bool hasPoints = false;

    for(const auto& rp : paths) {
        for(const auto& curve : rp.path) {
             const auto& pts = curve.controlPoints();
             for(const auto& p : pts) {
                 if(p[0] < minX) minX = p[0];
                 if(p[0] > maxX) maxX = p[0];
                 if(p[1] < minY) minY = p[1];
                 if(p[1] > maxY) maxY = p[1];
                 hasPoints = true;
             }
        }
    }
    
    if(!hasPoints) { minX = 0; maxX = 100; minY = 0; maxY = 100; }
    
    // Add margin
    double rangeX = maxX - minX;
    double rangeY = maxY - minY;
    double margin = std::max(rangeX, rangeY) * 0.1 + 1.0;
    
    minX -= margin; maxX += margin;
    minY -= margin; maxY += margin;
    
    double width = config_.width;
    double height = config_.height;
    
    double scaleX = width / (maxX - minX);
    double scaleY = height / (maxY - minY);
    double scale = std::min(scaleX, scaleY);
    
    auto transformX = [&](double x) { return (x - minX) * scale; };
    auto transformY = [&](double y) { 
        if(config_.flipY) return (maxY - y) * scale; 
        else return (y - minY) * scale; 
    };

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width << "\" height=\"" << height << "\" "
        << "viewBox=\"0 0 " << width << " " << height << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    
    if(config_.showGrid) {
        out << "  <g id=\"grid\" stroke=\"" << config_.gridColor << "\" stroke-width=\"0.5\">\n";
        double spacing = config_.gridSpacing;
        if(spacing <= 0) spacing = 10.0;
        
        int startX = std::floor(minX / spacing) * spacing;
        int startY = std::floor(minY / spacing) * spacing;
        
        for(double x = startX; x <= maxX; x += spacing) {
            double sx = transformX(x);
            out << "    <line x1=\"" << sx << "\" y1=\"" << transformY(minY) << "\" x2=\"" << sx << "\" y2=\"" << transformY(maxY) << "\"/>\n";
        }
        for(double y = startY; y <= maxY; y += spacing) {
            double sy = transformY(y);
            out << "    <line x1=\"" << transformX(minX) << "\" y1=\"" << sy << "\" x2=\"" << transformX(maxX) << "\" y2=\"" << sy << "\"/>\n";
        }
        out << "  </g>\n";
    }

    for (const auto& rpath : paths) {
        out << "  <path d=\"";
        
        bool first = true;
        for (const auto& curve : rpath.path) {
            const auto& pts = curve.controlPoints(); 
            
            double x0 = transformX(pts[0][0]);
            double y0 = transformY(pts[0][1]);
            
            if (first) {
                out << "M" << x0 << "," << y0;
                first = false;
            } else {
                 out << " L " << x0 << "," << y0;
            }
    
            if (curve.degree() == 3) {
                out << " C " << transformX(pts[1][0]) << "," << transformY(pts[1][1]) << " " 
                    << transformX(pts[2][0]) << "," << transformY(pts[2][1]) << " " 
                    << transformX(pts[3][0]) << "," << transformY(pts[3][1]);
            } else if (curve.degree() == 1) {
                out << " L " << transformX(pts[1][0]) << "," << transformY(pts[1][1]);
            } else {
                 const int samples = 32; 
                 for(int k=1; k<=samples; ++k) {
                     double t = (double)k / samples;
                     auto pt = curve.evaluate(t);
                     out << " L " << transformX(pt[0]) << "," << transformY(pt[1]);
                 }
            }
        }
        
        out << "\" stroke=\"" << rpath.color << "\" stroke-width=\"" << rpath.width << "\" fill=\"none\"/>\n";
    }
    out << "</svg>\n";
    return true;
}

bool SVGExporter::exportNURBSPath(const MotionPlanner::PiecewiseNURBSPath<2, double>& nurbsPath,
                                   const std::string& filename,
                                   const std::string& color,
                                   double width,
                                   double maxApproxError) {
    // Decompose NURBS to cubic Bézier curves
    auto cubics = nurbsPath.approximateWithCubicBeziers(maxApproxError);

    // Convert to RenderableBezierPath and delegate
    RenderableBezierPath rp;
    rp.path = std::move(cubics);
    rp.color = color;
    rp.width = width;

    return exportBezierPaths({rp}, filename);
}

} // namespace GCodeExport
