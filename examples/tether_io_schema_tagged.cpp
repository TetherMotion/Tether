/**
 * @file tether_io_schema_tagged.cpp
 * @brief Tagged structs: self-describing wire format for dynamic shapes.
 *
 * StructEncoding::Tagged carries each field as a keyed, length-delimited
 * record on the wire — the opposite of Packed's fixed-offset image. The
 * cost is per-field overhead; the wins are what this example shows:
 *
 *   - variable-length members: String, DynamicArray, Map
 *   - optional fields that occupy zero bytes when absent
 *   - fields may appear in any order and unknown keys can be skipped —
 *     the encoding survives schema evolution
 *
 *   ./tether_io_schema_tagged
 */

#include "tether/io/Schema.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaValue.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "tether/io/SchemaValueCodec.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace tether::io;

namespace {

SchemaKey k(uint8_t tag) { SchemaKey x{}; x[0] = 0xDE; x[1] = tag; return x; }

void dump(const SchemaValue& v) {
    const auto& st = std::get<SchemaValue::Struct>(v.payload);
    for (const auto& f : st.fields) {
        const char* name = f.schemaField ? f.schemaField->name.c_str() : "?";
        if (const auto* s = std::get_if<SchemaValue::Scalar>(&f.value->payload))
            std::printf("    %-6s scalar bits=0x%llx\n", name,
                        static_cast<unsigned long long>(s->bits));
        else if (const auto* s = std::get_if<SchemaValue::Str>(&f.value->payload))
            std::printf("    %-6s \"%s\"\n", name, s->value.c_str());
        else if (const auto* a = std::get_if<SchemaValue::Array>(&f.value->payload))
            std::printf("    %-6s array[%zu]\n", name, a->elements.size());
        else if (const auto* m = std::get_if<SchemaValue::Map>(&f.value->payload))
            std::printf("    %-6s map{%zu}\n", name, m->entries.size());
        else if (const auto* o = std::get_if<SchemaValue::Optional>(&f.value->payload))
            std::printf("    %-6s optional(present=%d)\n", name, o->present);
    }
}

} // namespace

int main() {
    SchemaNode u64n;
    u64n.key = k(1); u64n.kind = SchemaKind::Scalar;
    u64n.scalarType = ValueType::U64; u64n.name = "demo.U64";
    SchemaNode f64n;
    f64n.key = k(2); f64n.kind = SchemaKind::Scalar;
    f64n.scalarType = ValueType::F64; f64n.name = "demo.F64";
    SchemaNode strn;
    strn.key = k(3); strn.kind = SchemaKind::String;
    strn.maxBytes = 256; strn.name = "demo.Str";
    SchemaNode tags;
    tags.key = k(4); tags.kind = SchemaKind::DynamicArray;
    tags.name = "demo.Tags"; tags.element = SchemaRef{k(3), {}};
    SchemaNode scores;
    scores.key = k(5); scores.kind = SchemaKind::Map;
    scores.name = "demo.Scores";
    scores.mapKey = SchemaRef{k(3), {}};
    scores.mapValue = SchemaRef{k(2), {}};
    // Optional<T>: a value-level presence flag (1 byte + element on the
    // wire when present). nv::none() encodes the absent case.
    SchemaNode optStr;
    optStr.key = k(7); optStr.kind = SchemaKind::Optional;
    optStr.name = "demo.OptStr"; optStr.element = SchemaRef{k(3), {}};

    SchemaNode rec;
    rec.key = k(6); rec.kind = SchemaKind::Struct;
    rec.structEncoding = StructEncoding::Tagged;
    rec.name = "demo.Record";
    rec.fields = {
        {1, 0, {k(1), {}}, "id", ""},
        {2, 0, {k(3), {}}, "label", ""},
        {3, 0, {k(4), {}}, "tags", ""},
        {4, 0, {k(5), {}}, "scores", ""},
        {5, 0, {k(7), {}}, "note", ""},
    };

    SchemaGraph graph;
    graph.nodes = {std::move(u64n), std::move(f64n), std::move(strn),
                   std::move(tags), std::move(scores), std::move(optStr),
                   std::move(rec)};
    if (!resolveSchemaDigests(graph)) return 1;
    const auto valid = validateSchemaGraph(graph);
    if (!valid) { std::fprintf(stderr, "%s\n", valid.message.c_str()); return 1; }
    const SchemaNode& recNode = *graph.find(k(6));

    // Full value: string, string array, string->f64 map, present note.
    const std::vector<std::string> tagv = {"alpha", "beta", "gamma"};
    const auto full = encodeNative(
        graph, recNode,
        nv::fields(
            nv::field("id", nv::u64(9)),
            nv::field("label", nv::str("sample")),
            nv::field("tags", nv::strv(tagv)),
            nv::field("scores",
                      nv::map(nv::mapEntry(nv::str("a"), nv::f64(1.5)),
                              nv::mapEntry(nv::str("b"), nv::f64(-2.0)))),
            nv::field("note", nv::str("first"))));
    if (!full) { std::fprintf(stderr, "encode: %s\n", full.error().c_str()); return 1; }

    // Same shape, optional field absent (nv::none()).
    const auto minimal = encodeNative(
        graph, recNode,
        nv::fields(
            nv::field("id", nv::u64(10)),
            nv::field("label", nv::str("lean")),
            nv::field("tags", nv::items()),
            nv::field("scores", nv::map()),
            nv::field("note", nv::none())));
    if (!minimal) { std::fprintf(stderr, "encode: %s\n", minimal.error().c_str()); return 1; }

    std::printf("full=%zu bytes, absent-optional=%zu bytes\n",
                full->size(), minimal->size());

    const auto dv = decodeSchemaValue(graph, recNode, full->data(), full->size());
    const auto dm = decodeSchemaValue(graph, recNode,
                                      minimal->data(), minimal->size());
    if (!dv || !dm) return 1;
    std::printf("full value:\n");
    dump(*dv);
    std::printf("minimal value (note absent -> no bytes spent):\n");
    dump(*dm);
    return 0;
}
