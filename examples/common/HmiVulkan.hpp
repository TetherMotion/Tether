// HmiVulkan.hpp — reusable Vulkan backend for the HMI scene IR.
//
// Shares the exact geometry/text pipeline with examples/hmi_vulkan.cpp
// (which is now a thin CLI over this class): glyb/FreeType+HarfBuzz
// glyph atlas, solid-triangle scene ops, alpha blending, scene-space
// push constants. Used by hmi_remote for live frames and by any
// native panel that renders HmiScene documents.
#pragma once

#include "HmiScene.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace tether { namespace hmi {

/** RGBA8 pixel image in row-major order. */
struct HmiImage {
    uint32_t w = 0, h = 0;
    std::vector<uint8_t> rgba;   // w*h*4
};

class HmiRenderer {
public:
    HmiRenderer();
    ~HmiRenderer();
    HmiRenderer(const HmiRenderer&) = delete;
    HmiRenderer& operator=(const HmiRenderer&) = delete;

    /** Initialize Vulkan device, glyb text engine, pipelines, atlas.
     *  @param fontPath TTF/OTF path; empty → DejaVu Sans Mono search. */
    bool init(const std::string& fontPath = {});

    /** Windowed variant (requires the SDL2 build — TETHER_HMI_SDL2):
     *  creates an SDL window + VkSurfaceKHR + swapchain on top of the
     *  same device/pipelines as init(). @p w/@p h are the initial
     *  window size in pixels; the scene is letterboxed to fit. */
    bool initWindowed(const std::string& fontPath, int w, int h,
                      const std::string& title = "hmi");

    /** True when a windowed swapchain is active. */
    bool windowed() const;

    /** Pump SDL events (close/resize). Call once per frame. */
    void pumpWindowEvents();

    /** False after the user closed the window or the surface was lost. */
    bool windowAlive() const;

    /** Current drawable size in pixels (for aspect/fit decisions). */
    std::pair<int, int> windowExtent() const;

    /** Render @p scene and present it to the window. The scene is
     *  uniformly scaled to fit the drawable with letterbox margins in
     *  the scene background color. Returns false on device loss. */
    bool renderWindowed(const HmiScene& scene);

    /** Tear down window/swapchain resources (device stays alive). */
    void closeWindow();

    /** Render @p scene at @p scale → RGBA pixels.
     *  Output size = round(scene.w*scale) × round(scene.h*scale).
     *  Returns false on GPU/alloc failure. */
    bool render(const HmiScene& scene, int scale, HmiImage& out);

    /** Convenience: render and write binary PPM (P6, RGB). */
    bool renderPpm(const HmiScene& scene, int scale,
                   const std::string& path);

    /** Measured advance of @p s at @p px font size (scene units when
     *  px == size). Used by callers that pre-anchor text. */
    float measureText(const std::string& s, float size) const;

    bool ok() const { return impl_ != nullptr; }

    /* Impl is public so the TU-local swapchain helpers in
       HmiVulkan.cpp can name it (examples-internal header). */
    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}} // namespace tether::hmi
