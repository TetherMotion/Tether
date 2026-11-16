// hmi_vulkan.cpp — CLI frontend for the shared HMI Vulkan backend.
//
// Consumes the same scene JSON document that src/hmi/scene.ts renders
// to SVG on the web, so both backends produce identical views by
// construction. Rendering lives in examples/common/HmiVulkan.cpp
// (shared with hmi_remote).
//
// Text uses the same glyph engine as VolcanoPlot: glyb
// (font_manager_ft + HarfBuzz shaping + FreeType rasterization into a
// grayscale atlas), rendered as textured quads.
//
// Usage:
//   hmi_vulkan <scene.json> <out.ppm> [--scale N] [--font PATH]
//
// Offscreen only (no windowing dependency): renders to an image and
// writes binary PPM (P6). Runs headless under Mesa lavapipe.

#include "common/HmiScene.hpp"
#include "common/HmiVulkan.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    std::string scenePath, outPath, fontPath;
    int scale = 1;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--scale" && i + 1 < argc) scale = std::atoi(argv[++i]);
        else if (a == "--font" && i + 1 < argc) fontPath = argv[++i];
        else if (scenePath.empty()) scenePath = a;
        else if (outPath.empty()) outPath = a;
    }
    if (scenePath.empty() || outPath.empty()) {
        std::cerr << "usage: hmi_vulkan <scene.json> <out.ppm> "
                     "[--scale N] [--font PATH]\n";
        return 2;
    }

    std::ifstream in(scenePath, std::ios::binary);
    if (!in) { std::cerr << "cannot open " << scenePath << "\n"; return 1; }
    std::stringstream ss;
    ss << in.rdbuf();
    tether::hmi::HmiScene scene;
    try {
        scene = tether::hmi::parseScene(ss.str());
    } catch (const std::exception& e) {
        std::cerr << "scene parse error: " << e.what() << "\n";
        return 1;
    }
    if (scene.w <= 0 || scene.h <= 0) {
        std::cerr << "scene has no w/h\n";
        return 1;
    }

    tether::hmi::HmiRenderer renderer;
    if (!renderer.init(fontPath)) {
        std::cerr << "renderer init failed (vulkan/font)\n";
        return 1;
    }
    if (!renderer.renderPpm(scene, scale, outPath)) {
        std::cerr << "render failed\n";
        return 1;
    }
    std::cerr << "wrote " << outPath << " ("
              << uint32_t(scene.w) * scale << "x"
              << uint32_t(scene.h) * scale << ")\n";
    return 0;
}
