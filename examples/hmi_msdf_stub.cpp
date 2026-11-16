// hmi_msdf_stub.cpp — satisfies glyb's glyph_renderer_msdf vtable for
// builds that use the plain grayscale atlas (msdf_enabled = false).
// Ported from VolcanoPlot's approach of replacing glyb's msdf.cc (which
// requires the msdfgen dependency tree we don't need).
//
// The MSDF renderer is never invoked when msdf_enabled is false, so a
// no-op implementation is sufficient — it exists only to satisfy the
// `static glyph_renderer_msdf msdf;` instance in font_manager_ft::
// getGlyphRenderer().

// glyb headers are not self-contained — they rely on the includer
// providing the std headers (same convention as hmi_vulkan.cpp).
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <hb.h>

#include "binpack.h"
#include "utf8.h"
#include "image.h"
#include "draw.h"
#include "font.h"
#include "glyph.h"
#include "msdf.h"

atlas_entry glyph_renderer_msdf::render(font_atlas*, font_face_ft*,
                                        int, int) {
    return atlas_entry();
}
