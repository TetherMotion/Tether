/**
 * @file tether_io_schema_packed.cpp
 * @brief Packed structs: schema-authored fixed layout that is byte-
 *        identical to a C struct.
 *
 * A SchemaKind::Struct with StructEncoding::Packed lays fields out at
 * fixed offsets in declaration order — no per-field headers on the
 * wire. computeSchemaLayout() derives the offsets; when
 * SchemaLayout::memcpyImage is true the wire value IS the C packed
 * struct image, so producers can memcpy a row into a LatestSignal/ring
 * and consumers can reinterpret the bytes directly.
 *
 * This example authors a struct mixing scalars, a FixedArray and a
 * FixedString, prints the computed layout, encodes via the nv::
 * builders, and proves the wire bytes equal a plain C struct.
 *
 *   ./tether_io_schema_packed
 */

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaLayout.hpp"
#include "tether/io/SchemaValue.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "tether/io/SchemaValueCodec.hpp"

#include <cstring>
#include <cstdio>
#include <string>

using namespace tether::io;

namespace {

SchemaKey k(uint8_t tag) { SchemaKey x{}; x[0] = 0xDE; x[1] = tag; return x; }

SchemaNode scalar(SchemaKey key, const char* name, ValueType t) {
    SchemaNode n;
    n.key = key; n.kind = SchemaKind::Scalar;
    n.scalarType = t; n.name = name;
    return n;
}

// C twin of the schema below — the whole point of Packed.
#pragma pack(push, 1)
struct PointRow {
    uint64_t id;
    double pos[3];
    char name[16];
    double weight;
};
#pragma pack(pop)

} // namespace

int main() {
    SchemaGraph graph;

    SchemaNode vec3;
    vec3.key = k(3); vec3.kind = SchemaKind::FixedArray;
    vec3.fixedCount = 3; vec3.name = "demo.Vec3";
    vec3.element = SchemaRef{k(2), {}};

    SchemaNode name16;
    name16.key = k(4); name16.kind = SchemaKind::Scalar;
    name16.scalarType = ValueType::FixedString;
    name16.fixedCount = 16; name16.name = "demo.Name16";

    SchemaNode point;
    point.key = k(5); point.kind = SchemaKind::Struct;
    point.structEncoding = StructEncoding::Packed;
    point.name = "demo.Point";
    point.fields = {
        {1, 0, {k(1), {}}, "id", ""},
        {2, 0, {k(3), {}}, "pos", ""},
        {3, 0, {k(4), {}}, "name", ""},
        {4, 0, {k(2), {}}, "weight", ""},
    };

    graph.nodes = {scalar(k(1), "demo.U64", ValueType::U64),
                   scalar(k(2), "demo.F64", ValueType::F64),
                   std::move(vec3), std::move(name16), std::move(point)};
    if (!resolveSchemaDigests(graph)) return 1;
    const auto valid = validateSchemaGraph(graph);
    if (!valid) { std::fprintf(stderr, "%s\n", valid.message.c_str()); return 1; }

    const SchemaNode& pointNode = *graph.find(k(5));

    // ---- Layout ----------------------------------------------------
    const auto layout = computeSchemaLayout(graph, pointNode);
    if (!layout) { std::fprintf(stderr, "%s\n", layout.error().c_str()); return 1; }
    std::printf("wireSize=%zu fixedLayout=%d memcpyImage=%d (C sizeof=%zu)\n",
                layout->wireSize, layout->fixedLayout, layout->memcpyImage,
                sizeof(PointRow));
    for (const auto& h : layout->head)
        std::printf("  field %-7s offset=%2zu size=%zu\n",
                    h.field->name.c_str(), h.offset, h.size);
    if (layout->wireSize != sizeof(PointRow)) return 1;

    // ---- Encode via nv:: builders ----------------------------------
    const std::string nm = "origin";
    const double pos[3] = {1.0, -2.5, 3.25};
    const auto wire = encodeNative(
        graph, pointNode,
        nv::fields(nv::field("id", nv::u64(7)),
                   nv::field("pos", nv::f64v(pos)),
                   nv::field("name", nv::str(nm)),
                   nv::field("weight", nv::f64(0.5))));
    if (!wire) { std::fprintf(stderr, "encode: %s\n", wire.error().c_str()); return 1; }
    std::printf("encoded %zu wire bytes\n", wire->size());

    // The packed wire value must be byte-identical to the C image.
    PointRow row{};
    row.id = 7;
    std::memcpy(row.pos, pos, sizeof(pos));
    std::memcpy(row.name, nm.c_str(), nm.size() + 1);
    row.weight = 0.5;
    if (wire->size() != sizeof(row) ||
        std::memcmp(wire->data(), &row, sizeof(row)) != 0) {
        std::fprintf(stderr, "wire image != C image\n");
        return 1;
    }
    std::printf("wire bytes == C packed struct image (memcpy-capable)\n");

    // ---- Decode back -----------------------------------------------
    const auto value = decodeSchemaValue(graph, pointNode,
                                         wire->data(), wire->size());
    if (!value) { std::fprintf(stderr, "decode: %s\n", value.error().c_str()); return 1; }
    const auto& st = std::get<SchemaValue::Struct>(value->payload);
    for (const auto& f : st.fields)
        std::printf("  decoded key=%u name=%s\n", f.key,
                    f.schemaField ? f.schemaField->name.c_str() : "?");
    return st.fields.size() == 4 ? 0 : 1;
}
