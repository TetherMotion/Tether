// HmiVulkan.cpp — Vulkan backend for the HMI scene IR (shared).
//
// Implementation of HmiRenderer (HmiVulkan.hpp). Text uses the same
// glyph engine as VolcanoPlot: glyb font_manager_ft + HarfBuzz shaping
// + FreeType rasterization into a grayscale atlas, drawn as textured
// quads. Solid ops triangulate on the CPU; filled paths are fans
// (star-shaped contract from scene.ts).
//
// SPDX-License-Identifier: Apache-2.0 OR MPL-2.0

#include "HmiVulkan.hpp"

#include <vulkan/vulkan.hpp>
#include <shaderc/shaderc.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>
#ifdef TETHER_HMI_SDL2
#include <SDL.h>
#include <SDL_vulkan.h>
#endif
#include <hb.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include "binpack.h"
#include "utf8.h"
#include "image.h"
#include "draw.h"
#include "font.h"
#include "glyph.h"
#include "msdf.h"

namespace tether { namespace hmi {
namespace {

using namespace ::tether::hmi;

// ==========================================================================
// glyb text — same glyph engine as VolcanoPlot
// ==========================================================================

// Content-guessed direction/script (upstream text_shaper_hb hardcodes
// LTR/Latin) — same improvement VolcanoPlot applies.
struct ShaperGuess : text_shaper {
    void shape(std::vector<glyph_shape>& shapes,
               text_segment& segment) override {
        auto* face = static_cast<font_face_ft*>(segment.face);
        face->get_metrics(segment.font_size);
        hb_font_t* hbfont = face->get_hbfont(segment.font_size);
        hb_language_t hblang = hb_language_from_string(
            segment.language.c_str(), (int)segment.language.size());
        hb_buffer_t* buf = hb_buffer_create();
        hb_buffer_set_language(buf, hblang);
        hb_buffer_add_utf8(buf, segment.text.c_str(),
                           (int)segment.text.size(), 0,
                           (int)segment.text.size());
        hb_buffer_guess_segment_properties(buf);
        hb_shape(hbfont, buf, nullptr, 0);
        unsigned n = 0;
        auto* info = hb_buffer_get_glyph_infos(buf, &n);
        auto* pos = hb_buffer_get_glyph_positions(buf, &n);
        for (unsigned i = 0; i < n; ++i)
            shapes.push_back({info[i].codepoint, info[i].cluster,
                              pos[i].x_offset, pos[i].y_offset,
                              pos[i].x_advance, pos[i].y_advance});
        hb_buffer_destroy(buf);
    }
};

struct TextEngine {
    std::unique_ptr<font_manager_ft> fm;
    std::unique_ptr<ShaperGuess> shaper;
    std::unique_ptr<text_renderer_ft> renderer;
    font_face* face = nullptr;

    bool init(const std::string& fontPath) {
        fm = std::make_unique<font_manager_ft>();
        // DejaVu Sans Mono — same family the web SVG backend requests
        // ('DejaVu Sans Mono','Liberation Mono',monospace) and the font
        // hmi_font_metrics.cpp measures for hmiTextWidth().
        static const char* kCandidates[] = {
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
            "/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        };
        std::vector<std::string> paths;
        if (!fontPath.empty()) paths.push_back(fontPath);
        for (const char* c : kCandidates) paths.push_back(c);
        for (const auto& p : paths) {
            fm->scanFontPath(p);
            face = fm->findFontByPath(p);
            if (face) break;
        }
        if (!face) {
            fm->scanFontDir("/usr/share/fonts");
            face = fm->findFontByName("DejaVu Sans Mono");
        }
        if (!face && fm->fontCount()) face = fm->findFontById(0);
        if (!face) return false;
        shaper = std::make_unique<ShaperGuess>();
        renderer = std::make_unique<text_renderer_ft>(fm.get());
        return true;
    }

    font_atlas* atlas() { return fm ? fm->getCurrentAtlas(face) : nullptr; }

    float measure(const std::string& s, int px) {
        if (s.empty() || !face) return 0;
        std::vector<glyph_shape> shapes;
        // glyb font_size is 26.6 fixed-point pixels
        text_segment seg(s, "en", face, px * 64, 0, 0, 0);
        shaper->shape(shapes, seg);
        float adv = 0;
        for (auto& g : shapes) adv += g.x_advance / 64.f;
        return adv;
    }
};

// ==========================================================================
// Geometry expansion — scene ops → solid tris + glyb text quads
// ==========================================================================

struct SolidVert { float x, y, r, g, b, a; };
struct TextVert { float x, y, u, v, r, g, b, a; };

struct Geom {
    std::vector<SolidVert> solid;
    std::vector<uint32_t> solidIdx;
    std::vector<TextVert> text;
    std::vector<uint32_t> textIdx;
};

inline void solidTri(Geom& g, Pt a, Pt b, Pt c,
                     float r, float gg, float bl, float al) {
    uint32_t i = uint32_t(g.solid.size());
    g.solid.push_back({a.x, a.y, r, gg, bl, al});
    g.solid.push_back({b.x, b.y, r, gg, bl, al});
    g.solid.push_back({c.x, c.y, r, gg, bl, al});
    g.solidIdx.insert(g.solidIdx.end(), {i, i + 1, i + 2});
}

inline void solidQuad(Geom& g, float x, float y, float w, float h,
                      float r, float gg, float bl, float al) {
    if (w <= 0 || h <= 0) return;
    Pt p[4] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
    solidTri(g, p[0], p[1], p[2], r, gg, bl, al);
    solidTri(g, p[0], p[2], p[3], r, gg, bl, al);
}

void solidDisc(Geom& g, float cx, float cy, float r,
               float cr, float cg, float cb, float ca, int n = 10) {
    if (r <= 0) return;
    uint32_t base = uint32_t(g.solid.size());
    g.solid.push_back({cx, cy, cr, cg, cb, ca});
    for (int i = 0; i <= n; ++i) {
        float a = float(i) / n * 6.2831853f;
        g.solid.push_back({cx + r * std::cos(a), cy + r * std::sin(a),
                           cr, cg, cb, ca});
    }
    for (int i = 0; i < n; ++i)
        g.solidIdx.insert(g.solidIdx.end(),
                          {base, base + 1 + i, base + 2 + i});
}

void solidSeg(Geom& g, Pt a, Pt b, float w,
              float r, float gg, float bl, float al) {
    float dx = b.x - a.x, dy = b.y - a.y;
    float len = std::hypot(dx, dy);
    if (len < 1e-6f) return;
    float nx = -dy / len * w * 0.5f, ny = dx / len * w * 0.5f;
    Pt p[4] = {{a.x + nx, a.y + ny}, {b.x + nx, b.y + ny},
               {b.x - nx, b.y - ny}, {a.x - nx, a.y - ny}};
    solidTri(g, p[0], p[1], p[2], r, gg, bl, al);
    solidTri(g, p[0], p[2], p[3], r, gg, bl, al);
}

void solidFill(Geom& g, const std::vector<Pt>& pts,
               float r, float gg, float b, float al) {
    if (pts.size() < 3) return;
    for (size_t i = 1; i + 1 < pts.size(); ++i)
        solidTri(g, pts[0], pts[i], pts[i + 1], r, gg, b, al);
}

// glyphs rasterize at size*k px then map back to scene units —
// supersampled so text stays crisp at any output resolution.
constexpr float kTextScale = 4.0f;

void emitText(Geom& g, TextEngine& te, const SceneOp& op) {
    if (op.s.empty() || !te.face) return;
    int px = std::max(8, int(std::lround(op.size * kTextScale)));
    float advScene = te.measure(op.s, px) / kTextScale;
    float x = op.x;
    if (op.anchor == 1) x -= advScene * 0.5f;
    else if (op.anchor == 2) x -= advScene;

    std::vector<glyph_shape> shapes;
    uint32_t packed =
        (uint32_t(std::lround(op.cr * 255)) << 24) |
        (uint32_t(std::lround(op.cg * 255)) << 16) |
        (uint32_t(std::lround(op.cb * 255)) << 8) |
        uint32_t(std::lround(op.ca * 255));
    // glyb font_size is 26.6 fixed-point; x/y are pixel coords
    text_segment seg(op.s, "en", te.face, px * 64, x * kTextScale,
                     op.y * kTextScale, packed);
    te.shaper->shape(shapes, seg);

    auto* batch = new draw_list();
    te.renderer->render(*batch, shapes, seg);

    uint32_t vbase = uint32_t(g.text.size());
    for (const auto& v : batch->vertices) {
        uint32_t c = v.color;
        g.text.push_back({v.pos[0] / kTextScale, v.pos[1] / kTextScale,
                          v.uv[0], v.uv[1],
                          ((c >> 24) & 0xff) / 255.f,
                          ((c >> 16) & 0xff) / 255.f,
                          ((c >> 8) & 0xff) / 255.f,
                          (c & 0xff) / 255.f});
    }
    for (uint32_t idx : batch->indices)
        g.textIdx.push_back(vbase + idx);
    delete batch;
}

void buildGeom(const HmiScene& sc, Geom& g, TextEngine& te) {
    for (const SceneOp& op : sc.ops) {
        switch (op.kind) {
        case SceneOp::Rect:
            if (op.hasFill)
                solidQuad(g, op.x, op.y, op.w, op.h,
                          op.fr, op.fg, op.fb, op.fa);
            if (op.hasStroke) {
                float t = op.sw;
                solidQuad(g, op.x, op.y, op.w, t, op.cr, op.cg, op.cb, op.ca);
                solidQuad(g, op.x, op.y + op.h - t, op.w, t,
                          op.cr, op.cg, op.cb, op.ca);
                solidQuad(g, op.x, op.y + t, t, op.h - 2 * t,
                          op.cr, op.cg, op.cb, op.ca);
                solidQuad(g, op.x + op.w - t, op.y + t, t, op.h - 2 * t,
                          op.cr, op.cg, op.cb, op.ca);
            }
            break;
        case SceneOp::Line:
            solidSeg(g, {op.x, op.y}, {op.w, op.h}, op.sw,
                     op.cr, op.cg, op.cb, op.ca);
            solidDisc(g, op.x, op.y, op.sw * 0.5f,
                      op.cr, op.cg, op.cb, op.ca, 8);
            solidDisc(g, op.w, op.h, op.sw * 0.5f,
                      op.cr, op.cg, op.cb, op.ca, 8);
            break;
        case SceneOp::Path: {
            /* scene contract: a path is fill XOR stroke (the SVG
               backend emits fill-only when `fill` is set) */
            size_t n = op.pts.size();
            if (op.hasFill) {
                solidFill(g, op.pts, op.fr, op.fg, op.fb, op.fa);
            } else {
                size_t segs = op.close ? n : n - 1;
                for (size_t i = 0; i < segs; ++i)
                    solidSeg(g, op.pts[i], op.pts[(i + 1) % n], op.sw,
                             op.cr, op.cg, op.cb, op.ca);
                /* round joins/caps — only for opaque strokes to avoid
                   double-blending translucent overlaps */
                if (op.ca >= 0.99f)
                    for (const Pt& p : op.pts)
                        solidDisc(g, p.x, p.y, op.sw * 0.5f,
                                  op.cr, op.cg, op.cb, op.ca, 6);
            }
            break;
        }
        case SceneOp::Text:
            emitText(g, te, op);
            break;
        }
    }
}

// ==========================================================================
// Vulkan helpers
// ==========================================================================

uint32_t findMemoryType(vk::PhysicalDevice phys, uint32_t typeBits,
                        vk::MemoryPropertyFlags props) {
    auto memProps = phys.getMemoryProperties();
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i)
        if ((typeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    throw std::runtime_error("no suitable memory type");
}

struct Buf {
    vk::Buffer buf;
    vk::DeviceMemory mem;
    VkDeviceSize size = 0;
};

Buf makeBuffer(vk::Device dev, vk::PhysicalDevice phys,
               const void* data, VkDeviceSize size,
               vk::BufferUsageFlags usage, bool hostVisible) {
    Buf b;
    b.size = size;
    b.buf = dev.createBuffer({{}, size, usage,
                              vk::SharingMode::eExclusive});
    auto req = dev.getBufferMemoryRequirements(b.buf);
    vk::MemoryPropertyFlags props =
        hostVisible
            ? vk::MemoryPropertyFlags(vk::MemoryPropertyFlagBits::eHostVisible |
                                      vk::MemoryPropertyFlagBits::eHostCoherent)
            : vk::MemoryPropertyFlags(vk::MemoryPropertyFlagBits::eDeviceLocal);
    b.mem = dev.allocateMemory(
        {req.size,
         findMemoryType(phys, req.memoryTypeBits, props)},
        nullptr);
    dev.bindBufferMemory(b.buf, b.mem, 0);
    if (data && hostVisible) {
        void* p = dev.mapMemory(b.mem, 0, size);
        std::memcpy(p, data, size);
        dev.unmapMemory(b.mem);
    }
    return b;
}

// device-local buffer filled via a one-shot command buffer
Buf uploadBuffer(vk::Device dev, vk::PhysicalDevice phys, vk::Queue queue,
                 vk::CommandPool pool, const void* data, VkDeviceSize size,
                 vk::BufferUsageFlags usage) {
    Buf staging = makeBuffer(dev, phys, data, size,
                             vk::BufferUsageFlagBits::eTransferSrc, true);
    Buf dst = makeBuffer(dev, phys, nullptr, size, usage, false);
    vk::CommandBufferAllocateInfo cai{pool,
                                      vk::CommandBufferLevel::ePrimary, 1};
    auto cmds = dev.allocateCommandBuffers(cai);
    auto c = cmds[0];
    c.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    vk::BufferCopy region{0, 0, size};
    c.copyBuffer(staging.buf, dst.buf, region);
    c.end();
    vk::SubmitInfo si{{}, {}, c, {}};
    queue.submit(si, nullptr);
    queue.waitIdle();
    dev.freeCommandBuffers(pool, cmds);
    dev.destroyBuffer(staging.buf);
    dev.freeMemory(staging.mem);
    return dst;
}

std::vector<uint32_t> compileShader(shaderc_shader_kind kind,
                                    const char* src, const char* name) {
    shaderc::Compiler comp;
    shaderc::CompileOptions opts;
    opts.SetOptimizationLevel(shaderc_optimization_level_performance);
    auto res = comp.CompileGlslToSpv(src, kind, name, "main", opts);
    if (res.GetCompilationStatus() != shaderc_compilation_status_success) {
        std::cerr << "shader error (" << name << "): "
                  << res.GetErrorMessage() << "\n";
        throw std::runtime_error("shader compile failed");
    }
    const uint32_t* b = res.cbegin();
    return std::vector<uint32_t>(b, res.cend());
}

const char* kSolidVert = R"(#version 450
layout(push_constant) uniform PC { vec2 scene; } pc;
layout(location=0) in vec2 pos;
layout(location=1) in vec4 color;
layout(location=0) out vec4 vColor;
void main() {
    vColor = color;
    /* Vulkan NDC +y points down the framebuffer — scene y=0 (top)
       maps to ndc -1 */
    vec2 ndc = vec2(pos.x / pc.scene.x * 2.0 - 1.0,
                    pos.y / pc.scene.y * 2.0 - 1.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
})";

const char* kSolidFrag = R"(#version 450
layout(location=0) in vec4 vColor;
layout(location=0) out vec4 o;
void main() { o = vColor; })";

const char* kTextVert = R"(#version 450
layout(push_constant) uniform PC { vec2 scene; } pc;
layout(location=0) in vec2 pos;
layout(location=1) in vec2 uv;
layout(location=2) in vec4 color;
layout(location=0) out vec2 vUV;
layout(location=1) out vec4 vColor;
void main() {
    vUV = uv; vColor = color;
    /* Vulkan NDC +y points down the framebuffer — scene y=0 (top)
       maps to ndc -1 */
    vec2 ndc = vec2(pos.x / pc.scene.x * 2.0 - 1.0,
                    pos.y / pc.scene.y * 2.0 - 1.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
})";

const char* kTextFrag = R"(#version 450
layout(set=0, binding=0) uniform sampler2D atlas;
layout(location=0) in vec2 vUV;
layout(location=1) in vec4 vColor;
layout(location=0) out vec4 o;
void main() {
    float cov = texture(atlas, vUV).r;
    o = vec4(vColor.rgb, vColor.a * cov);
})";

struct Gpu {
    vk::Instance instance;
    vk::PhysicalDevice phys;
    vk::Device dev;
    vk::Queue queue;
    uint32_t qfamily = 0;
    vk::CommandPool pool;

    bool init(const std::vector<const char*>& instExts = {},
              const std::vector<const char*>& devExts = {}) {
        vk::ApplicationInfo app;
        app.pApplicationName = "hmi_vulkan";
        app.applicationVersion = 1;
        app.apiVersion = VK_API_VERSION_1_2;
        vk::InstanceCreateInfo ici;
        ici.pApplicationInfo = &app;
        ici.enabledExtensionCount = uint32_t(instExts.size());
        ici.ppEnabledExtensionNames = instExts.data();
        instance = vk::createInstance(ici);
        auto physDevs = instance.enumeratePhysicalDevices();
        if (physDevs.empty()) {
            std::cerr << "no vulkan physical device\n";
            return false;
        }
        phys = physDevs[0];
        auto qf = phys.getQueueFamilyProperties();
        for (uint32_t i = 0; i < qf.size(); ++i)
            if (qf[i].queueFlags & vk::QueueFlagBits::eGraphics) {
                qfamily = i;
                break;
            }
        float prio = 1.f;
        vk::DeviceQueueCreateInfo dq;
        dq.queueFamilyIndex = qfamily;
        dq.queueCount = 1;
        dq.pQueuePriorities = &prio;
        vk::DeviceCreateInfo dci;
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &dq;
        dci.enabledExtensionCount = uint32_t(devExts.size());
        dci.ppEnabledExtensionNames = devExts.data();
        dev = phys.createDevice(dci);
        queue = dev.getQueue(qfamily, 0);
        vk::CommandPoolCreateInfo cpi;
        cpi.flags = vk::CommandPoolCreateFlags{};
        cpi.queueFamilyIndex = qfamily;
        pool = dev.createCommandPool(cpi);
        return true;
    }
};

vk::CommandBuffer allocCmd(Gpu& gpu) {
    vk::CommandBufferAllocateInfo cai;
    cai.commandPool = gpu.pool;
    cai.level = vk::CommandBufferLevel::ePrimary;
    cai.commandBufferCount = 1;
    return gpu.dev.allocateCommandBuffers(cai)[0];
}

} // namespace

// ==========================================================================
// HmiRenderer::Impl — persistent device/pipelines; per-frame targets.
// ==========================================================================

struct HmiWin;

struct HmiRenderer::Impl {
    Gpu gpu;
    TextEngine te;
    bool inited = false;
    std::unique_ptr<HmiWin> win;

    // persistent GPU objects
    vk::ShaderModule sVert, sFrag, tVert, tFrag;
    vk::PipelineLayout pcLayout, textLayout;
    vk::DescriptorSetLayout dsl;
    vk::Pipeline solidPipe, textPipe;
    vk::RenderPass rp;
    vk::Sampler sampler;
    vk::Image atlasImg{};
    vk::ImageView atlasView{};
    vk::DeviceMemory atlasMem{};
    uint32_t atlasW = 0, atlasH = 0;

    ~Impl() {
        shutdownWindow();
        if (!inited || !gpu.dev) return;
        vk::Device dev = gpu.dev;
        if (atlasW) {
            dev.destroyImageView(atlasView);
            dev.destroyImage(atlasImg);
            dev.freeMemory(atlasMem);
        }
        if (sampler) dev.destroySampler(sampler);
        if (solidPipe) dev.destroyPipeline(solidPipe);
        if (textPipe) dev.destroyPipeline(textPipe);
        if (pcLayout) dev.destroyPipelineLayout(pcLayout);
        if (textLayout) dev.destroyPipelineLayout(textLayout);
        if (dsl) dev.destroyDescriptorSetLayout(dsl);
        if (sVert) dev.destroyShaderModule(sVert);
        if (sFrag) dev.destroyShaderModule(sFrag);
        if (tVert) dev.destroyShaderModule(tVert);
        if (tFrag) dev.destroyShaderModule(tFrag);
        if (rp) dev.destroyRenderPass(rp);
    }

    void shutdownWindow();

    bool ensureAtlasImage() {
        font_atlas* atlas = te.atlas();
        if (!atlas || !atlas->pixels) return false;
        if (atlasImg && atlasW == uint32_t(atlas->width)
                      && atlasH == uint32_t(atlas->height))
            return true;
        vk::Device dev = gpu.dev;
        if (atlasW) {
            dev.destroyImageView(atlasView);
            dev.destroyImage(atlasImg);
            dev.freeMemory(atlasMem);
        }
        atlasView = vk::ImageView{};
        atlasImg = vk::Image{};
        atlasMem = vk::DeviceMemory{};

        vk::ImageCreateInfo ai;
        ai.imageType = vk::ImageType::e2D;
        ai.format = vk::Format::eR8Unorm;
        ai.extent = vk::Extent3D{uint32_t(atlas->width),
                                 uint32_t(atlas->height), 1};
        ai.mipLevels = 1;
        ai.arrayLayers = 1;
        ai.samples = vk::SampleCountFlagBits::e1;
        ai.tiling = vk::ImageTiling::eOptimal;
        ai.usage = vk::ImageUsageFlagBits::eTransferDst |
                   vk::ImageUsageFlagBits::eSampled;
        ai.sharingMode = vk::SharingMode::eExclusive;
        ai.initialLayout = vk::ImageLayout::eUndefined;
        atlasImg = dev.createImage(ai);
        auto aReq = dev.getImageMemoryRequirements(atlasImg);
        atlasMem = dev.allocateMemory(
            vk::MemoryAllocateInfo{aReq.size,
                findMemoryType(gpu.phys, aReq.memoryTypeBits,
                               vk::MemoryPropertyFlagBits::eDeviceLocal)},
            nullptr);
        dev.bindImageMemory(atlasImg, atlasMem, 0);
        vk::ImageViewCreateInfo avi;
        avi.image = atlasImg;
        avi.viewType = vk::ImageViewType::e2D;
        avi.format = vk::Format::eR8Unorm;
        avi.subresourceRange =
            {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
        atlasView = dev.createImageView(avi);
        atlasW = uint32_t(atlas->width);
        atlasH = uint32_t(atlas->height);
        return true;
    }

    bool render(const HmiScene& scene, int scale, HmiImage& out) {
        if (!inited) return false;
        vk::Device dev = gpu.dev;
        font_atlas* atlas = te.atlas();
        if (!atlas || !atlas->pixels || !ensureAtlasImage()) return false;

        Geom geom;
        buildGeom(scene, geom, te);

        const uint32_t W = std::max(1u, uint32_t(scene.w) * uint32_t(scale));
        const uint32_t H = std::max(1u, uint32_t(scene.h) * uint32_t(scale));
        out.w = W; out.h = H;
        out.rgba.assign(size_t(W) * H * 4, 0);

        // ---- per-frame target image ----
        vk::ImageCreateInfo ici;
        ici.imageType = vk::ImageType::e2D;
        ici.format = vk::Format::eR8G8B8A8Unorm;
        ici.extent = vk::Extent3D{W, H, 1};
        ici.mipLevels = 1;
        ici.arrayLayers = 1;
        ici.samples = vk::SampleCountFlagBits::e1;
        ici.tiling = vk::ImageTiling::eOptimal;
        ici.usage = vk::ImageUsageFlagBits::eColorAttachment |
                    vk::ImageUsageFlagBits::eTransferSrc;
        ici.sharingMode = vk::SharingMode::eExclusive;
        ici.initialLayout = vk::ImageLayout::eUndefined;
        vk::Image img = dev.createImage(ici);
        auto imgReq = dev.getImageMemoryRequirements(img);
        vk::DeviceMemory imgMem = dev.allocateMemory(
            vk::MemoryAllocateInfo{imgReq.size,
                findMemoryType(gpu.phys, imgReq.memoryTypeBits,
                               vk::MemoryPropertyFlagBits::eDeviceLocal)},
            nullptr);
        dev.bindImageMemory(img, imgMem, 0);
        vk::ImageViewCreateInfo ivi;
        ivi.image = img;
        ivi.viewType = vk::ImageViewType::e2D;
        ivi.format = vk::Format::eR8G8B8A8Unorm;
        ivi.subresourceRange =
            {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
        vk::ImageView imgView = dev.createImageView(ivi);

        // ---- atlas staging + per-frame descriptor ----
        VkDeviceSize atlasBytes =
            VkDeviceSize(atlas->width) * atlas->height * atlas->depth;
        std::vector<uint8_t> atlasPx(atlas->pixels,
                                     atlas->pixels + atlasBytes);
        Buf atlasStage = makeBuffer(dev, gpu.phys, atlasPx.data(),
                                    atlasBytes,
                                    vk::BufferUsageFlagBits::eTransferSrc,
                                    true);

        vk::DescriptorPool dpool;
        vk::DescriptorSet dset;
        {
            vk::DescriptorPoolSize dps;
            dps.type = vk::DescriptorType::eCombinedImageSampler;
            dps.descriptorCount = 1;
            vk::DescriptorPoolCreateInfo dpci;
            dpci.maxSets = 1;
            dpci.poolSizeCount = 1;
            dpci.pPoolSizes = &dps;
            dpool = dev.createDescriptorPool(dpci);
            vk::DescriptorSetAllocateInfo dai;
            dai.descriptorPool = dpool;
            dai.descriptorSetCount = 1;
            dai.pSetLayouts = &dsl;
            dset = dev.allocateDescriptorSets(dai)[0];
            vk::DescriptorImageInfo dii;
            dii.sampler = sampler;
            dii.imageView = atlasView;
            dii.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
            vk::WriteDescriptorSet wds;
            wds.dstSet = dset;
            wds.dstBinding = 0;
            wds.descriptorCount = 1;
            wds.descriptorType = vk::DescriptorType::eCombinedImageSampler;
            wds.pImageInfo = &dii;
            dev.updateDescriptorSets(wds, nullptr);
        }

        vk::Framebuffer fb;
        {
            vk::ImageView attaches[1] = {imgView};
            vk::FramebufferCreateInfo fbci;
            fbci.renderPass = rp;
            fbci.attachmentCount = 1;
            fbci.pAttachments = attaches;
            fbci.width = W;
            fbci.height = H;
            fbci.layers = 1;
            fb = dev.createFramebuffer(fbci);
        }

        // ---- geometry buffers ----
        Buf solidVB, solidIB, textVB, textIB;
        if (!geom.solid.empty()) {
            solidVB = uploadBuffer(dev, gpu.phys, gpu.queue, gpu.pool,
                                   geom.solid.data(),
                                   geom.solid.size() * sizeof(SolidVert),
                                   vk::BufferUsageFlagBits::eVertexBuffer);
            solidIB = uploadBuffer(dev, gpu.phys, gpu.queue, gpu.pool,
                                   geom.solidIdx.data(),
                                   geom.solidIdx.size() * 4,
                                   vk::BufferUsageFlagBits::eIndexBuffer);
        }
        if (!geom.text.empty()) {
            textVB = uploadBuffer(dev, gpu.phys, gpu.queue, gpu.pool,
                                  geom.text.data(),
                                  geom.text.size() * sizeof(TextVert),
                                  vk::BufferUsageFlagBits::eVertexBuffer);
            textIB = uploadBuffer(dev, gpu.phys, gpu.queue, gpu.pool,
                                  geom.textIdx.data(),
                                  geom.textIdx.size() * 4,
                                  vk::BufferUsageFlagBits::eIndexBuffer);
        }

        // ---- record: atlas upload + render + readback ----
        vk::CommandBuffer cmd = allocCmd(gpu);
        vk::CommandBufferBeginInfo cbi;
        cbi.flags = vk::CommandBufferUsageFlags{};
        cmd.begin(cbi);

        auto barrier = [&](vk::Image image, vk::ImageLayout oldL,
                           vk::ImageLayout newL,
                           vk::AccessFlags srcA, vk::AccessFlags dstA,
                           vk::PipelineStageFlags srcS,
                           vk::PipelineStageFlags dstS) {
            vk::ImageMemoryBarrier b;
            b.oldLayout = oldL;
            b.newLayout = newL;
            b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            b.image = image;
            b.subresourceRange =
                {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
            b.srcAccessMask = srcA;
            b.dstAccessMask = dstA;
            vk::ImageMemoryBarrier arr[1] = {b};
            cmd.pipelineBarrier(srcS, dstS, vk::DependencyFlags{},
                                {}, {}, arr);
        };

        barrier(atlasImg, vk::ImageLayout::eUndefined,
                vk::ImageLayout::eTransferDstOptimal,
                vk::AccessFlags{}, vk::AccessFlagBits::eTransferWrite,
                vk::PipelineStageFlagBits::eTopOfPipe,
                vk::PipelineStageFlagBits::eTransfer);
        vk::BufferImageCopy region;
        region.imageSubresource = {vk::ImageAspectFlagBits::eColor, 0, 0, 1};
        region.imageExtent = vk::Extent3D{atlasW, atlasH, 1};
        cmd.copyBufferToImage(atlasStage.buf, atlasImg,
                              vk::ImageLayout::eTransferDstOptimal, region);
        barrier(atlasImg, vk::ImageLayout::eTransferDstOptimal,
                vk::ImageLayout::eShaderReadOnlyOptimal,
                vk::AccessFlagBits::eTransferWrite,
                vk::AccessFlagBits::eShaderRead,
                vk::PipelineStageFlagBits::eTransfer,
                vk::PipelineStageFlagBits::eFragmentShader);

        barrier(img, vk::ImageLayout::eUndefined,
                vk::ImageLayout::eColorAttachmentOptimal,
                vk::AccessFlags{}, vk::AccessFlagBits::eColorAttachmentWrite,
                vk::PipelineStageFlagBits::eTopOfPipe,
                vk::PipelineStageFlagBits::eColorAttachmentOutput);

        vk::ClearValue clear;
        if (scene.hasBg) {
            clear.color = vk::ClearColorValue{scene.br, scene.bg,
                                              scene.bb, scene.ba};
        } else {
            clear.color = vk::ClearColorValue{0.07f, 0.07f, 0.09f, 1.f};
        }
        vk::RenderPassBeginInfo rpbi;
        rpbi.renderPass = rp;
        rpbi.framebuffer = fb;
        rpbi.renderArea = vk::Rect2D{{{0, 0}, {W, H}}};
        rpbi.clearValueCount = 1;
        rpbi.pClearValues = &clear;
        cmd.beginRenderPass(rpbi, vk::SubpassContents::eInline);

        vk::Viewport viewport;
        viewport.x = 0; viewport.y = 0;
        viewport.width = float(W); viewport.height = float(H);
        viewport.minDepth = 0; viewport.maxDepth = 1;
        vk::Rect2D scissor{{{0, 0}, {W, H}}};
        cmd.setViewport(0, viewport);
        cmd.setScissor(0, scissor);
        float pcData[2] = {scene.w, scene.h};

        if (!geom.solid.empty()) {
            vk::Buffer vbs[1] = {solidVB.buf};
            VkDeviceSize offs[1] = {0};
            cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, solidPipe);
            cmd.pushConstants(pcLayout,
                              vk::ShaderStageFlagBits::eVertex, 0, 8, pcData);
            cmd.bindVertexBuffers(0, vbs, offs);
            cmd.bindIndexBuffer(solidIB.buf, 0, vk::IndexType::eUint32);
            cmd.drawIndexed(uint32_t(geom.solidIdx.size()), 1, 0, 0, 0);
        }
        if (!geom.text.empty()) {
            vk::Buffer vbs[1] = {textVB.buf};
            VkDeviceSize offs[1] = {0};
            vk::DescriptorSet dsets[1] = {dset};
            cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, textPipe);
            cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                                   textLayout, 0, dsets, nullptr);
            cmd.pushConstants(textLayout,
                              vk::ShaderStageFlagBits::eVertex, 0, 8,
                              pcData);
            cmd.bindVertexBuffers(0, vbs, offs);
            cmd.bindIndexBuffer(textIB.buf, 0, vk::IndexType::eUint32);
            cmd.drawIndexed(uint32_t(geom.textIdx.size()), 1, 0, 0, 0);
        }
        cmd.endRenderPass();

        barrier(img, vk::ImageLayout::eColorAttachmentOptimal,
                vk::ImageLayout::eTransferSrcOptimal,
                vk::AccessFlagBits::eColorAttachmentWrite,
                vk::AccessFlagBits::eTransferRead,
                vk::PipelineStageFlagBits::eColorAttachmentOutput,
                vk::PipelineStageFlagBits::eTransfer);
        VkDeviceSize rbSize = VkDeviceSize(W) * H * 4;
        Buf rb = makeBuffer(dev, gpu.phys, nullptr, rbSize,
                            vk::BufferUsageFlagBits::eTransferDst, true);
        vk::BufferImageCopy cp;
        cp.imageSubresource = {vk::ImageAspectFlagBits::eColor, 0, 0, 1};
        cp.imageExtent = vk::Extent3D{W, H, 1};
        cmd.copyImageToBuffer(img, vk::ImageLayout::eTransferSrcOptimal,
                              rb.buf, cp);
        cmd.end();

        vk::SubmitInfo si;
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd;
        vk::Fence fence;
        {
            vk::FenceCreateInfo fci;
            fence = dev.createFence(fci);
        }
        gpu.queue.submit(si, fence);
        dev.waitForFences(fence, true, UINT64_MAX);
        dev.destroyFence(fence);

        void* mapped = dev.mapMemory(rb.mem, 0, rbSize);
        const uint8_t* px = static_cast<const uint8_t*>(mapped);
        out.rgba.resize(size_t(W) * H * 4);
        std::memcpy(out.rgba.data(), px, size_t(W) * H * 4);
        dev.unmapMemory(rb.mem);

        // ---- cleanup per-frame objects ----
        dev.destroyBuffer(atlasStage.buf);
        dev.freeMemory(atlasStage.mem);
        dev.destroyDescriptorPool(dpool);
        dev.destroyFramebuffer(fb);
        dev.destroyImageView(imgView);
        dev.destroyImage(img);
        dev.freeMemory(imgMem);
        dev.destroyBuffer(rb.buf);
        dev.freeMemory(rb.mem);
        if (!geom.solid.empty()) {
            dev.destroyBuffer(solidVB.buf); dev.freeMemory(solidVB.mem);
            dev.destroyBuffer(solidIB.buf); dev.freeMemory(solidIB.mem);
        }
        if (!geom.text.empty()) {
            dev.destroyBuffer(textVB.buf); dev.freeMemory(textVB.mem);
            dev.destroyBuffer(textIB.buf); dev.freeMemory(textIB.mem);
        }
        dev.freeCommandBuffers(gpu.pool, cmd);
        return true;
    }
};

// ==========================================================================
// Windowed swapchain backend (SDL2 surface → VkSwapchainKHR → present)
// ==========================================================================

#ifdef TETHER_HMI_SDL2

struct HmiWin {
    SDL_Window* window = nullptr;
    vk::SurfaceKHR surface;
    vk::SwapchainKHR swapchain;
    vk::Format format = vk::Format::eUndefined;
    vk::Extent2D extent{};
    std::vector<vk::Image> images;
    std::vector<vk::ImageView> views;
    std::vector<vk::Framebuffer> fbs;
    std::vector<vk::Semaphore> imgAvail, renderDone;
    std::vector<vk::Fence> inFlight;
    vk::DescriptorPool dpool;
    vk::DescriptorSet dset;
    uint32_t frame = 0;
    bool dead = false;
    int drawW = 0, drawH = 0;
};

namespace {

void destroySwapchainObjects(HmiRenderer::Impl& im, HmiWin& w) {
    vk::Device dev = im.gpu.dev;
    for (auto fb : w.fbs) dev.destroyFramebuffer(fb);
    for (auto v : w.views) dev.destroyImageView(v);
    w.fbs.clear(); w.views.clear(); w.images.clear();
    if (w.swapchain) {
        dev.destroySwapchainKHR(w.swapchain);
        w.swapchain = vk::SwapchainKHR();
    }
    for (auto s : w.imgAvail) dev.destroySemaphore(s);
    for (auto s : w.renderDone) dev.destroySemaphore(s);
    for (auto f : w.inFlight) dev.destroyFence(f);
    w.imgAvail.clear(); w.renderDone.clear(); w.inFlight.clear();
    if (w.dpool) { dev.destroyDescriptorPool(w.dpool); w.dpool = vk::DescriptorPool(); }
    w.dset = vk::DescriptorSet();
    w.extent = vk::Extent2D{0, 0};
}

bool createSwapchain(HmiRenderer::Impl& im, HmiWin& w, int reqW, int reqH) {
    vk::Device dev = im.gpu.dev;
    vk::PhysicalDevice phys = im.gpu.phys;

    vk::SurfaceCapabilitiesKHR caps =
        phys.getSurfaceCapabilitiesKHR(w.surface);
    std::vector<vk::SurfaceFormatKHR> fmts =
        phys.getSurfaceFormatsKHR(w.surface);
    std::vector<vk::PresentModeKHR> modes =
        phys.getSurfacePresentModesKHR(w.surface);

    // The shared render pass (created in finishInit) is R8G8B8A8Unorm —
    // the swapchain must use the same format or rendering is invalid.
    vk::SurfaceFormatKHR fmt = fmts.empty()
        ? vk::SurfaceFormatKHR{vk::Format::eR8G8B8A8Unorm,
                               vk::ColorSpaceKHR::eSrgbNonlinear}
        : fmts[0];
    bool fmtOk = fmts.empty();   // empty list = any format supported
    for (const auto& f : fmts) {
        if (f.format == vk::Format::eR8G8B8A8Unorm) { fmt = f; fmtOk = true; break; }
    }
    if (!fmtOk) {
        std::fprintf(stderr, "hmi: surface lacks R8G8B8A8Unorm format\n");
        return false;
    }
    vk::PresentModeKHR pm = vk::PresentModeKHR::eFifo;
    for (auto m : modes)
        if (m == vk::PresentModeKHR::eMailbox) { pm = m; break; }

    vk::Extent2D ext = caps.currentExtent;
    if (ext.width == 0xFFFFFFFFu) {
        ext.width = uint32_t(std::clamp(reqW, 1, 16384));
        ext.height = uint32_t(std::clamp(reqH, 1, 16384));
    }
    ext.width = std::clamp(ext.width, caps.minImageExtent.width,
                           caps.maxImageExtent.width);
    ext.height = std::clamp(ext.height, caps.minImageExtent.height,
                            caps.maxImageExtent.height);

    uint32_t count = std::max(2u, caps.minImageCount);
    if (caps.maxImageCount > 0) count = std::min(count, caps.maxImageCount);

    vk::CompositeAlphaFlagBitsKHR ca =
        vk::CompositeAlphaFlagBitsKHR::eOpaque;
    if (!(caps.supportedCompositeAlpha &
          vk::CompositeAlphaFlagBitsKHR::eOpaque)) {
        ca = caps.supportedCompositeAlpha &
                 vk::CompositeAlphaFlagBitsKHR::eInherit
             ? vk::CompositeAlphaFlagBitsKHR::eInherit
             : vk::CompositeAlphaFlagBitsKHR::ePreMultiplied;
    }

    vk::SwapchainCreateInfoKHR sci;
    sci.surface = w.surface;
    sci.minImageCount = count;
    sci.imageFormat = fmt.format;
    sci.imageColorSpace = fmt.colorSpace;
    sci.imageExtent = ext;
    sci.imageArrayLayers = 1;
    sci.imageUsage = vk::ImageUsageFlagBits::eColorAttachment;
    sci.imageSharingMode = vk::SharingMode::eExclusive;
    sci.preTransform = caps.currentTransform;
    sci.compositeAlpha = ca;
    sci.presentMode = pm;
    sci.clipped = VK_TRUE;
    sci.oldSwapchain = w.swapchain;
    w.swapchain = dev.createSwapchainKHR(sci);
    w.format = fmt.format;
    w.extent = ext;

    w.images = dev.getSwapchainImagesKHR(w.swapchain);
    count = uint32_t(w.images.size());
    w.views.resize(count);
    w.fbs.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        vk::ImageViewCreateInfo ivi;
        ivi.image = w.images[i];
        ivi.viewType = vk::ImageViewType::e2D;
        ivi.format = w.format;
        ivi.subresourceRange =
            {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
        w.views[i] = dev.createImageView(ivi);
        vk::FramebufferCreateInfo fbci;
        fbci.renderPass = im.rp;
        fbci.attachmentCount = 1;
        fbci.pAttachments = &w.views[i];
        fbci.width = ext.width;
        fbci.height = ext.height;
        fbci.layers = 1;
        w.fbs[i] = dev.createFramebuffer(fbci);
    }

    w.imgAvail.resize(count);
    w.renderDone.resize(count);
    w.inFlight.resize(count);
    vk::SemaphoreCreateInfo semci;
    vk::FenceCreateInfo fci;
    fci.flags = vk::FenceCreateFlagBits::eSignaled;
    for (uint32_t i = 0; i < count; ++i) {
        w.imgAvail[i] = dev.createSemaphore(semci);
        w.renderDone[i] = dev.createSemaphore(semci);
        w.inFlight[i] = dev.createFence(fci);
    }
    w.frame = 0;
    w.dpool = vk::DescriptorPool();
    w.dset = vk::DescriptorSet();
    return true;
}

} // namespace

#endif // TETHER_HMI_SDL2

void HmiRenderer::Impl::shutdownWindow() {
#ifdef TETHER_HMI_SDL2
    if (!win) return;
    if (gpu.dev && win->swapchain) {
        gpu.dev.waitIdle();
        destroySwapchainObjects(*this, *win);
    }
    if (win->surface && gpu.instance) {
        gpu.instance.destroySurfaceKHR(win->surface);
        win->surface = vk::SurfaceKHR{};
    }
    if (win->window) { SDL_DestroyWindow(win->window); win->window = nullptr; }
    win.reset();
#endif
}

// ==========================================================================
// HmiRenderer public API
// ==========================================================================

HmiRenderer::HmiRenderer() = default;
HmiRenderer::~HmiRenderer() = default;

/** Shared post-device init: text engine, shaders, pipelines, render
 *  pass, atlas sampler. gpu.init() must have run already. */
static bool finishInit(HmiRenderer::Impl& im,
                       const std::string& fontPath) {
    if (!im.te.init(fontPath)) return false;
    vk::Device dev = im.gpu.dev;

    // compile shaders
    auto solidSpv = compileShader(shaderc_glsl_vertex_shader, kSolidVert,
                                  "solid.vert");
    auto solidFspv = compileShader(shaderc_glsl_fragment_shader, kSolidFrag,
                                   "solid.frag");
    auto textSpv = compileShader(shaderc_glsl_vertex_shader, kTextVert,
                                 "text.vert");
    auto textFspv = compileShader(shaderc_glsl_fragment_shader, kTextFrag,
                                  "text.frag");
    auto mkShader = [&](const std::vector<uint32_t>& spv) {
        vk::ShaderModuleCreateInfo smci;
        smci.codeSize = spv.size() * 4;
        smci.pCode = spv.data();
        return dev.createShaderModule(smci);
    };
    im.sVert = mkShader(solidSpv); im.sFrag = mkShader(solidFspv);
    im.tVert = mkShader(textSpv);   im.tFrag = mkShader(textFspv);

    vk::PushConstantRange pcr;
    pcr.stageFlags = vk::ShaderStageFlagBits::eVertex;
    pcr.offset = 0;
    pcr.size = 8;

    {
        vk::PipelineLayoutCreateInfo lci;
        lci.pushConstantRangeCount = 1;
        lci.pPushConstantRanges = &pcr;
        im.pcLayout = dev.createPipelineLayout(lci);
    }

    vk::DescriptorSetLayoutBinding binding;
    binding.binding = 0;
    binding.descriptorType = vk::DescriptorType::eCombinedImageSampler;
    binding.descriptorCount = 1;
    binding.stageFlags = vk::ShaderStageFlagBits::eFragment;
    {
        vk::DescriptorSetLayoutCreateInfo dci;
        dci.bindingCount = 1;
        dci.pBindings = &binding;
        im.dsl = dev.createDescriptorSetLayout(dci);
    }
    {
        vk::PipelineLayoutCreateInfo lci;
        lci.setLayoutCount = 1;
        lci.pSetLayouts = &im.dsl;
        lci.pushConstantRangeCount = 1;
        lci.pPushConstantRanges = &pcr;
        im.textLayout = dev.createPipelineLayout(lci);
    }

    vk::PipelineColorBlendAttachmentState blendAtt;
    blendAtt.blendEnable = true;
    blendAtt.srcColorBlendFactor = vk::BlendFactor::eSrcAlpha;
    blendAtt.dstColorBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAtt.colorBlendOp = vk::BlendOp::eAdd;
    blendAtt.srcAlphaBlendFactor = vk::BlendFactor::eOne;
    blendAtt.dstAlphaBlendFactor = vk::BlendFactor::eOneMinusSrcAlpha;
    blendAtt.alphaBlendOp = vk::BlendOp::eAdd;
    blendAtt.colorWriteMask = vk::ColorComponentFlagBits::eR |
        vk::ColorComponentFlagBits::eG |
        vk::ColorComponentFlagBits::eB |
        vk::ColorComponentFlagBits::eA;

    vk::AttachmentDescription att;
    att.format = vk::Format::eR8G8B8A8Unorm;
    att.samples = vk::SampleCountFlagBits::e1;
    att.loadOp = vk::AttachmentLoadOp::eClear;
    att.storeOp = vk::AttachmentStoreOp::eStore;
    att.stencilLoadOp = vk::AttachmentLoadOp::eDontCare;
    att.stencilStoreOp = vk::AttachmentStoreOp::eDontCare;
    att.initialLayout = vk::ImageLayout::eUndefined;
    att.finalLayout = vk::ImageLayout::eColorAttachmentOptimal;
    vk::AttachmentReference colorRef;
    colorRef.attachment = 0;
    colorRef.layout = vk::ImageLayout::eColorAttachmentOptimal;
    vk::SubpassDescription sub;
    sub.pipelineBindPoint = vk::PipelineBindPoint::eGraphics;
    sub.colorAttachmentCount = 1;
    sub.pColorAttachments = &colorRef;
    vk::SubpassDependency dep;
    dep.srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    dep.dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    dep.srcAccessMask = vk::AccessFlags{};
    dep.dstAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;
    vk::RenderPassCreateInfo rpci;
    rpci.attachmentCount = 1;
    rpci.pAttachments = &att;
    rpci.subpassCount = 1;
    rpci.pSubpasses = &sub;
    rpci.dependencyCount = 1;
    rpci.pDependencies = &dep;
    im.rp = dev.createRenderPass(rpci);

    auto makePipeline = [&](vk::ShaderModule vs, vk::ShaderModule fs,
                            vk::PipelineLayout layout,
                            const std::vector<vk::VertexInputBindingDescription>& binds,
                            const std::vector<vk::VertexInputAttributeDescription>& attrs) {
        vk::PipelineShaderStageCreateInfo stages[2];
        stages[0].stage = vk::ShaderStageFlagBits::eVertex;
        stages[0].module = vs;
        stages[0].pName = "main";
        stages[1].stage = vk::ShaderStageFlagBits::eFragment;
        stages[1].module = fs;
        stages[1].pName = "main";
        vk::PipelineVertexInputStateCreateInfo vi;
        vi.vertexBindingDescriptionCount = uint32_t(binds.size());
        vi.pVertexBindingDescriptions = binds.data();
        vi.vertexAttributeDescriptionCount = uint32_t(attrs.size());
        vi.pVertexAttributeDescriptions = attrs.data();
        vk::PipelineInputAssemblyStateCreateInfo ia;
        ia.topology = vk::PrimitiveTopology::eTriangleList;
        vk::PipelineViewportStateCreateInfo vp;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        vk::PipelineRasterizationStateCreateInfo rs;
        rs.polygonMode = vk::PolygonMode::eFill;
        rs.cullMode = vk::CullModeFlagBits::eNone;
        rs.frontFace = vk::FrontFace::eCounterClockwise;
        rs.lineWidth = 1.f;
        vk::PipelineMultisampleStateCreateInfo ms;
        ms.rasterizationSamples = vk::SampleCountFlagBits::e1;
        vk::PipelineDepthStencilStateCreateInfo ds;
        ds.depthTestEnable = false;
        ds.depthWriteEnable = false;
        ds.depthCompareOp = vk::CompareOp::eAlways;
        vk::PipelineColorBlendStateCreateInfo cb;
        cb.logicOpEnable = false;
        cb.logicOp = vk::LogicOp::eCopy;
        cb.attachmentCount = 1;
        cb.pAttachments = &blendAtt;
        vk::DynamicState dyns[2] = {vk::DynamicState::eViewport,
                                    vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dyn;
        dyn.dynamicStateCount = 2;
        dyn.pDynamicStates = dyns;
        vk::GraphicsPipelineCreateInfo pi;
        pi.stageCount = 2;
        pi.pStages = stages;
        pi.pVertexInputState = &vi;
        pi.pInputAssemblyState = &ia;
        pi.pViewportState = &vp;
        pi.pRasterizationState = &rs;
        pi.pMultisampleState = &ms;
        pi.pDepthStencilState = &ds;
        pi.pColorBlendState = &cb;
        pi.pDynamicState = &dyn;
        pi.layout = layout;
        pi.renderPass = im.rp;
        auto res = dev.createGraphicsPipelines(vk::PipelineCache{}, pi);
        if (res.result != vk::Result::eSuccess) {
            std::cerr << "pipeline creation failed: "
                      << vk::to_string(res.result) << "\n";
            return vk::Pipeline{};
        }
        return res.value[0];
    };

    std::vector<vk::VertexInputBindingDescription> sb;
    sb.push_back({0, sizeof(SolidVert), vk::VertexInputRate::eVertex});
    std::vector<vk::VertexInputAttributeDescription> sa;
    sa.push_back({0, 0, vk::Format::eR32G32Sfloat,
                  offsetof(SolidVert, x)});
    sa.push_back({1, 0, vk::Format::eR32G32B32A32Sfloat,
                  offsetof(SolidVert, r)});
    im.solidPipe = makePipeline(im.sVert, im.sFrag, im.pcLayout, sb, sa);

    std::vector<vk::VertexInputBindingDescription> tb;
    tb.push_back({0, sizeof(TextVert), vk::VertexInputRate::eVertex});
    std::vector<vk::VertexInputAttributeDescription> ta;
    ta.push_back({0, 0, vk::Format::eR32G32Sfloat,
                  offsetof(TextVert, x)});
    ta.push_back({1, 0, vk::Format::eR32G32Sfloat,
                  offsetof(TextVert, u)});
    ta.push_back({2, 0, vk::Format::eR32G32B32A32Sfloat,
                  offsetof(TextVert, r)});
    im.textPipe = makePipeline(im.tVert, im.tFrag, im.textLayout, tb, ta);

    vk::SamplerCreateInfo sci;
    sci.magFilter = vk::Filter::eLinear;
    sci.minFilter = vk::Filter::eLinear;
    sci.mipmapMode = vk::SamplerMipmapMode::eNearest;
    sci.addressModeU = vk::SamplerAddressMode::eClampToEdge;
    sci.addressModeV = vk::SamplerAddressMode::eClampToEdge;
    sci.addressModeW = vk::SamplerAddressMode::eClampToEdge;
    sci.maxLod = 0.f;
    im.sampler = dev.createSampler(sci);

    if (!im.solidPipe || !im.textPipe) return false;
    im.inited = true;
    return true;
}

bool HmiRenderer::init(const std::string& fontPath) {
    impl_ = std::make_unique<Impl>();
    Impl& im = *impl_;
    if (!im.gpu.init()) { impl_.reset(); return false; }
    if (!finishInit(im, fontPath)) { impl_.reset(); return false; }
    return true;
}

#ifdef TETHER_HMI_SDL2

bool HmiRenderer::initWindowed(const std::string& fontPath, int w, int h,
                               const std::string& title) {
    impl_ = std::make_unique<Impl>();
    Impl& im = *impl_;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init: " << SDL_GetError() << "\n";
        impl_.reset();
        return false;
    }
    im.win = std::make_unique<HmiWin>();
    HmiWin& win = *im.win;
    win.window = SDL_CreateWindow(
        title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        w, h,
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE |
            SDL_WINDOW_SHOWN);
    if (!win.window) {
        std::cerr << "SDL_CreateWindow: " << SDL_GetError() << "\n";
        impl_.reset();
        return false;
    }

    uint32_t extCount = 0;
    SDL_Vulkan_GetInstanceExtensions(win.window, &extCount, nullptr);
    std::vector<const char*> extNames(extCount);
    SDL_Vulkan_GetInstanceExtensions(win.window, &extCount,
                                     extNames.data());
    std::vector<const char*> instExts(extNames.begin(), extNames.end());
    instExts.push_back(VK_KHR_SURFACE_EXTENSION_NAME);
#ifdef VK_USE_PLATFORM_XLIB_KHR
    instExts.push_back(VK_KHR_XLIB_SURFACE_EXTENSION_NAME);
#endif
#ifdef VK_USE_PLATFORM_WAYLAND_KHR
    instExts.push_back(VK_KHR_WAYLAND_SURFACE_EXTENSION_NAME);
#endif

    if (!im.gpu.init(instExts, {VK_KHR_SWAPCHAIN_EXTENSION_NAME})) {
        impl_.reset();
        return false;
    }
    VkSurfaceKHR surf = VK_NULL_HANDLE;
    SDL_bool sres = SDL_Vulkan_CreateSurface(
        win.window, static_cast<VkInstance>(im.gpu.instance), &surf);
    if (!sres || surf == VK_NULL_HANDLE) {
        std::cerr << "SDL_Vulkan_CreateSurface failed\n";
        impl_.reset();
        return false;
    }
    win.surface = vk::SurfaceKHR(surf);

    if (!finishInit(im, fontPath)) { impl_.reset(); return false; }
    if (!createSwapchain(im, win, w, h)) { impl_.reset(); return false; }
    SDL_Vulkan_GetDrawableSize(win.window, &win.drawW, &win.drawH);
    return true;
}

bool HmiRenderer::windowed() const {
    return impl_ && impl_->win && impl_->win->swapchain;
}

void HmiRenderer::pumpWindowEvents() {
#ifdef TETHER_HMI_SDL2
    if (!impl_ || !impl_->win) return;
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT ||
            (ev.type == SDL_WINDOWEVENT &&
             ev.window.event == SDL_WINDOWEVENT_CLOSE))
            impl_->win->dead = true;
    }
#endif
}

bool HmiRenderer::windowAlive() const {
    return impl_ && impl_->win && !impl_->win->dead &&
           impl_->win->swapchain;
}

std::pair<int, int> HmiRenderer::windowExtent() const {
    if (!impl_ || !impl_->win || !impl_->win->window)
        return {0, 0};
    int w = 0, h = 0;
    SDL_Vulkan_GetDrawableSize(impl_->win->window, &w, &h);
    return {w, h};
}

void HmiRenderer::closeWindow() {
#ifdef TETHER_HMI_SDL2
    if (!impl_ || !impl_->win) return;
    HmiWin& w = *impl_->win;
    if (w.swapchain || !w.views.empty())
        destroySwapchainObjects(*impl_, w);
    if (w.surface) {
        vk::Instance inst = impl_->gpu.instance;
        inst.destroySurfaceKHR(w.surface);
        w.surface = vk::SurfaceKHR();
    }
    if (w.window) { SDL_DestroyWindow(w.window); w.window = nullptr; }
    impl_->win.reset();
#endif
}

bool HmiRenderer::renderWindowed(const HmiScene& scene) {
#ifdef TETHER_HMI_SDL2
    if (!impl_ || !impl_->win || !impl_->win->swapchain) return false;
    Impl& im = *impl_;
    HmiWin& w = *im.win;
    vk::Device dev = im.gpu.dev;

    if (!im.inited || !im.ensureAtlasImage()) return false;
    font_atlas* atlas = im.te.atlas();
    if (!atlas || !atlas->pixels) return false;

    // drawable resize → recreate swapchain
    int dw = 0, dh = 0;
    SDL_Vulkan_GetDrawableSize(w.window, &dw, &dh);
    if (dw <= 0 || dh <= 0) return true;   // minimized — skip frame
    if (dw != w.drawW || dh != w.drawH) {
        // wait idle before tearing down images in use
        dev.waitIdle();
        destroySwapchainObjects(im, w);
        if (!createSwapchain(im, w, dw, dh)) { w.dead = true; return false; }
        w.drawW = dw; w.drawH = dh;
    }

    const uint32_t W = w.extent.width, H = w.extent.height;
    const uint32_t frame = w.frame % uint32_t(w.fbs.size());

    // wait for this slot's previous frame, then acquire
    dev.waitForFences(w.inFlight[frame], true, UINT64_MAX);
    {
        vk::Fence f = w.inFlight[frame];
        dev.resetFences(1, &f);
    }
    uint32_t imgIdx = 0;
    vk::Result ar = static_cast<vk::Result>(dev.acquireNextImageKHR(
        w.swapchain, UINT64_MAX, w.imgAvail[frame],
        vk::Fence{}, &imgIdx));
    if (ar == vk::Result::eErrorOutOfDateKHR ||
        ar == vk::Result::eSuboptimalKHR) {
        dev.waitIdle();
        destroySwapchainObjects(im, w);
        if (!createSwapchain(im, w, dw, dh)) { w.dead = true; return false; }
        w.drawW = dw; w.drawH = dh;
        return true; // next frame retries
    }
    if (ar != vk::Result::eSuccess) {
        if (ar == vk::Result::eErrorDeviceLost ||
            ar == vk::Result::eErrorSurfaceLostKHR) w.dead = true;
        return false;
    }

    // ---- geometry ----
    Geom geom;
    buildGeom(scene, geom, im.te);

    // persistent text descriptor (atlas is stable for the session)
    if (!w.dset) {
        vk::DescriptorPoolSize dps;
        dps.type = vk::DescriptorType::eCombinedImageSampler;
        dps.descriptorCount = 1;
        vk::DescriptorPoolCreateInfo dpci;
        dpci.maxSets = 1;
        dpci.poolSizeCount = 1;
        dpci.pPoolSizes = &dps;
        w.dpool = dev.createDescriptorPool(dpci);
        vk::DescriptorSetAllocateInfo dai;
        dai.descriptorPool = w.dpool;
        dai.descriptorSetCount = 1;
        dai.pSetLayouts = &im.dsl;
        w.dset = dev.allocateDescriptorSets(dai)[0];
        vk::DescriptorImageInfo dii;
        dii.sampler = im.sampler;
        dii.imageView = im.atlasView;
        dii.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
        vk::WriteDescriptorSet wds;
        wds.dstSet = w.dset;
        wds.dstBinding = 0;
        wds.descriptorCount = 1;
        wds.descriptorType = vk::DescriptorType::eCombinedImageSampler;
        wds.pImageInfo = &dii;
        dev.updateDescriptorSets(wds, nullptr);
    }

    // ---- per-frame geometry buffers ----
    Buf solidVB, solidIB, textVB, textIB;
    if (!geom.solid.empty()) {
        solidVB = uploadBuffer(dev, im.gpu.phys, im.gpu.queue, im.gpu.pool,
                               geom.solid.data(),
                               geom.solid.size() * sizeof(SolidVert),
                               vk::BufferUsageFlagBits::eVertexBuffer);
        solidIB = uploadBuffer(dev, im.gpu.phys, im.gpu.queue, im.gpu.pool,
                               geom.solidIdx.data(),
                               geom.solidIdx.size() * 4,
                               vk::BufferUsageFlagBits::eIndexBuffer);
    }
    if (!geom.text.empty()) {
        textVB = uploadBuffer(dev, im.gpu.phys, im.gpu.queue, im.gpu.pool,
                              geom.text.data(),
                              geom.text.size() * sizeof(TextVert),
                              vk::BufferUsageFlagBits::eVertexBuffer);
        textIB = uploadBuffer(dev, im.gpu.phys, im.gpu.queue, im.gpu.pool,
                              geom.textIdx.data(),
                              geom.textIdx.size() * 4,
                              vk::BufferUsageFlagBits::eIndexBuffer);
    }

    // ---- record ----
    vk::CommandBuffer cmd = allocCmd(im.gpu);
    vk::CommandBufferBeginInfo cbi;
    cmd.begin(cbi);

    vk::ClearValue clear;
    if (scene.hasBg) {
        clear.color = vk::ClearColorValue{scene.br, scene.bg,
                                          scene.bb, scene.ba};
    } else {
        clear.color = vk::ClearColorValue{0.07f, 0.07f, 0.09f, 1.f};
    }
    vk::RenderPassBeginInfo rpbi;
    rpbi.renderPass = im.rp;
    rpbi.framebuffer = w.fbs[imgIdx];
    rpbi.renderArea = vk::Rect2D{{{0, 0}, {W, H}}};
    rpbi.clearValueCount = 1;
    rpbi.pClearValues = &clear;
    cmd.beginRenderPass(rpbi, vk::SubpassContents::eInline);

    // letterbox: uniform-fit the scene into the drawable
    float s = std::min(float(W) / std::max(1.f, scene.w),
                       float(H) / std::max(1.f, scene.h));
    float vw = scene.w * s, vh = scene.h * s;
    float vx = (float(W) - vw) * 0.5f, vy = (float(H) - vh) * 0.5f;
    vk::Viewport viewport;
    viewport.x = vx; viewport.y = vy;
    viewport.width = vw; viewport.height = vh;
    viewport.minDepth = 0; viewport.maxDepth = 1;
    vk::Rect2D scissor{{{int(vx), int(vy)},
                        {uint32_t(vw), uint32_t(vh)}}};
    cmd.setViewport(0, viewport);
    cmd.setScissor(0, scissor);
    float pcData[2] = {scene.w, scene.h};

    if (!geom.solid.empty()) {
        vk::Buffer vbs[1] = {solidVB.buf};
        VkDeviceSize offs[1] = {0};
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, im.solidPipe);
        cmd.pushConstants(im.pcLayout,
                          vk::ShaderStageFlagBits::eVertex, 0, 8, pcData);
        cmd.bindVertexBuffers(0, vbs, offs);
        cmd.bindIndexBuffer(solidIB.buf, 0, vk::IndexType::eUint32);
        cmd.drawIndexed(uint32_t(geom.solidIdx.size()), 1, 0, 0, 0);
    }
    if (!geom.text.empty()) {
        vk::Buffer vbs[1] = {textVB.buf};
        VkDeviceSize offs[1] = {0};
        vk::DescriptorSet dsets[1] = {w.dset};
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, im.textPipe);
        cmd.bindDescriptorSets(vk::PipelineBindPoint::eGraphics,
                               im.textLayout, 0, dsets, nullptr);
        cmd.pushConstants(im.textLayout,
                          vk::ShaderStageFlagBits::eVertex, 0, 8,
                          pcData);
        cmd.bindVertexBuffers(0, vbs, offs);
        cmd.bindIndexBuffer(textIB.buf, 0, vk::IndexType::eUint32);
        cmd.drawIndexed(uint32_t(geom.textIdx.size()), 1, 0, 0, 0);
    }
    cmd.endRenderPass();

    // attachment → present source
    vk::ImageMemoryBarrier pres;
    pres.oldLayout = vk::ImageLayout::eColorAttachmentOptimal;
    pres.newLayout = vk::ImageLayout::ePresentSrcKHR;
    pres.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    pres.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    pres.image = w.images[imgIdx];
    pres.subresourceRange =
        {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1};
    pres.srcAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;
    pres.dstAccessMask = vk::AccessFlags{};
    vk::ImageMemoryBarrier barr[1] = {pres};
    cmd.pipelineBarrier(
        vk::PipelineStageFlagBits::eColorAttachmentOutput,
        vk::PipelineStageFlagBits::eBottomOfPipe,
        vk::DependencyFlags{}, {}, {}, barr);
    cmd.end();

    vk::PipelineStageFlags waitStage =
        vk::PipelineStageFlagBits::eColorAttachmentOutput;
    vk::SubmitInfo si;
    si.waitSemaphoreCount = 1;
    si.pWaitSemaphores = &w.imgAvail[frame];
    si.pWaitDstStageMask = &waitStage;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores = &w.renderDone[frame];
    im.gpu.queue.submit(si, w.inFlight[frame]);

    vk::PresentInfoKHR pi;
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores = &w.renderDone[frame];
    pi.swapchainCount = 1;
    pi.pSwapchains = &w.swapchain;
    pi.pImageIndices = &imgIdx;
    vk::Result pr = static_cast<vk::Result>(
        im.gpu.queue.presentKHR(pi));
    if (pr == vk::Result::eErrorOutOfDateKHR ||
        pr == vk::Result::eSuboptimalKHR) {
        // recreate on next frame
    } else if (pr != vk::Result::eSuccess) {
        if (pr == vk::Result::eErrorSurfaceLostKHR ||
            pr == vk::Result::eErrorDeviceLost ||
            pr == vk::Result::eErrorOutOfDateKHR) {
            // surface lost — try recreate next call
        }
    }

    // ---- cleanup per-frame objects ----
    if (!geom.solid.empty()) {
        dev.destroyBuffer(solidVB.buf); dev.freeMemory(solidVB.mem);
        dev.destroyBuffer(solidIB.buf); dev.freeMemory(solidIB.mem);
    }
    if (!geom.text.empty()) {
        dev.destroyBuffer(textVB.buf); dev.freeMemory(textVB.mem);
        dev.destroyBuffer(textIB.buf); dev.freeMemory(textIB.mem);
    }
    dev.freeCommandBuffers(im.gpu.pool, cmd);
    w.frame++;
    return true;
#else
    (void)scene;
    return false;
#endif
}

#else // !TETHER_HMI_SDL2

bool HmiRenderer::initWindowed(const std::string&, int, int,
                               const std::string&) {
    return false;
}
bool HmiRenderer::windowed() const { return false; }
void HmiRenderer::pumpWindowEvents() {}
bool HmiRenderer::windowAlive() const { return false; }
std::pair<int, int> HmiRenderer::windowExtent() const { return {0, 0}; }
void HmiRenderer::closeWindow() {}
bool HmiRenderer::renderWindowed(const HmiScene&) { return false; }

#endif // TETHER_HMI_SDL2

bool HmiRenderer::render(const HmiScene& scene, int scale, HmiImage& out) {
    if (!impl_) return false;
    return impl_->render(scene, scale, out);
}

bool HmiRenderer::renderPpm(const HmiScene& scene, int scale,
                            const std::string& path) {
    HmiImage img;
    if (!render(scene, scale, img) || img.w == 0) return false;
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out << "P6\n" << img.w << " " << img.h << "\n255\n";
    for (uint32_t y = 0; y < img.h; ++y)
        for (uint32_t x = 0; x < img.w; ++x) {
            const uint8_t* p = &img.rgba[(size_t(y) * img.w + x) * 4];
            out.put(char(p[0]));
            out.put(char(p[1]));
            out.put(char(p[2]));
        }
    return bool(out);
}

float HmiRenderer::measureText(const std::string& s, float size) const {
    if (!impl_) return 0.f;
    return impl_->te.measure(s, std::max(8, int(std::lround(size))));
}

}} // namespace tether::hmi
