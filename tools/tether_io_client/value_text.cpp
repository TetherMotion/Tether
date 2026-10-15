/**
 * @file value_text.cpp
 * @brief Text→SchemaValue encoding implementation.
 */
#include "value_text.hpp"
#include "client_session.hpp"

#include <charconv>
#include <cstring>
#include <format>

#if defined(TETHER_IO_HAS_GLAZE)
#include <glaze/glaze.hpp>
#endif

namespace tether::io::client {

namespace {

const SchemaNode* resolve(const SchemaGraph& g, const SchemaRef& ref) {
    const SchemaNode* n = g.find(ref.key);
    while (n && n->kind == SchemaKind::Alias && n->target.has_value()) {
        n = g.find(n->target->key);
    }
    return n;
}


bool isIntScalar(ValueType t) {
    return t >= ValueType::U8 && t <= ValueType::U64;
}
bool isSignedScalar(ValueType t) {
    return t >= ValueType::I8 && t <= ValueType::I64;
}

std::expected<uint64_t, std::string>
scalarBits(const SchemaNode& node, const std::string& text) {
    std::string t = text;
    // trim
    const auto ws = " \t\n";
    t.erase(0, t.find_first_not_of(ws));
    t.erase(t.find_last_not_of(ws) + 1);
    if (t.empty()) return std::unexpected("empty value");

    const ValueType vt = node.scalarType;
    uint64_t bits = 0;
    if (vt == ValueType::Bool) {
        if (t == "true" || t == "1" || t == "on")  bits = 1;
        else if (t == "false" || t == "0" || t == "off") bits = 0;
        else return std::unexpected("invalid bool '" + t + "'");
        return bits;
    }
    if (vt == ValueType::F32 || vt == ValueType::F64) {
        char* end = nullptr;
        const double d = std::strtod(t.c_str(), &end);
        if (end != t.c_str() + t.size())
            return std::unexpected("invalid float '" + t + "'");
        if (vt == ValueType::F64) { std::memcpy(&bits, &d, 8); }
        else { float f = static_cast<float>(d); uint32_t fb; std::memcpy(&fb, &f, 4); bits = fb; }
        return bits;
    }
    // integers
    bool neg = false;
    std::string_view v = t;
    if (v.front() == '-') { neg = true; v.remove_prefix(1); }
    int base = 10;
    if (v.size() > 2 && (v.substr(0, 2) == "0x" || v.substr(0, 2) == "0X")) {
        base = 16; v.remove_prefix(2);
    }
    if (v.empty()) return std::unexpected("invalid integer");
    uint64_t mag = 0;
    auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), mag, base);
    if (ec != std::errc() || ptr != v.data() + v.size())
        return std::unexpected("invalid integer '" + t + "'");
    const int64_t s64 = neg ? -static_cast<int64_t>(mag)
                            : static_cast<int64_t>(mag);
    if (isSignedScalar(vt)) {
        int64_t iv = neg ? s64 : static_cast<int64_t>(mag);
        // range check
        switch (vt) {
            case ValueType::I8:  if (iv < -128 || iv > 127) goto range; break;
            case ValueType::I16: if (iv < -32768 || iv > 32767) goto range; break;
            case ValueType::I32: if (iv < INT32_MIN || iv > INT32_MAX) goto range; break;
            default: break;
        }
        bits = static_cast<uint64_t>(iv);
        return bits;
    }
    if (isIntScalar(vt)) {
        if (neg) goto range;
        switch (vt) {
            case ValueType::U8:  if (mag > 0xFF) goto range; break;
            case ValueType::U16: if (mag > 0xFFFF) goto range; break;
            case ValueType::U32: if (mag > 0xFFFFFFFF) goto range; break;
            default: break;
        }
        return mag;
    }
range:
    return std::unexpected(std::format("value '{}' out of range for {}",
                                     t, vtName(vt)));
}

std::string unquote(const std::string& t) {
    if (t.size() >= 2 && t.front() == '"' && t.back() == '"') {
        return t.substr(1, t.size() - 2);
    }
    return t;
}

std::expected<std::vector<uint8_t>, std::string> parseHex(const std::string& t) {
    std::string_view v = t;
    if (v.rfind("hex:", 0) == 0) v.remove_prefix(4);
    else if (v.rfind("0x", 0) == 0) v.remove_prefix(2);
    if (v.empty() || (v.size() % 2))
        return std::unexpected("hex needs an even digit count");
    std::vector<uint8_t> out(v.size() / 2);
    for (size_t i = 0; i < v.size(); i += 2) {
        unsigned byte;
        auto [ptr, ec] = std::from_chars(v.data() + i, v.data() + i + 2, byte, 16);
        if (ec != std::errc() || ptr != v.data() + i + 2)
            return std::unexpected("invalid hex '" + t + "'");
        out[i / 2] = static_cast<uint8_t>(byte);
    }
    return out;
}

#if defined(TETHER_IO_HAS_GLAZE)
// ---- JSON (glz::generic) → SchemaValue ------------------------------------

std::expected<SchemaValue, std::string>
jsonToValue(const SchemaGraph& g, const SchemaNode* node,
            const glz::generic& j);

std::expected<SchemaValue, std::string>
jsonScalar(const SchemaNode& node, const glz::generic& j) {
    if (const auto* d = j.get_if<double>()) {
        if (node.scalarType == ValueType::F32 ||
            node.scalarType == ValueType::F64) {
            double v = *d;
            if (node.scalarType == ValueType::F64) {
                uint64_t b; std::memcpy(&b, &v, 8);
                return makeScalarValue(&node, b);
            }
            float f = static_cast<float>(v); uint32_t fb; std::memcpy(&fb, &f, 4);
            return makeScalarValue(&node, fb);
        }
        // ints via double — only if exact
        if (*d != std::floor(*d))
            return std::unexpected(std::string("non-integer for ") +
                                   vtName(node.scalarType));
        return makeScalarValue(&node, static_cast<uint64_t>(
            static_cast<int64_t>(*d)));
    }
    if (const auto* b = j.get_if<bool>()) {
        if (node.scalarType != ValueType::Bool)
            return std::unexpected("bool literal for non-bool scalar");
        return makeScalarValue(&node, *b ? 1 : 0);
    }
    if (const auto* s = j.get_if<std::string>()) {
        if (node.scalarType == ValueType::Enum) {
            return std::unexpected("enum by name not supported in JSON — "
                                   "use the numeric value");
        }
        return std::unexpected("string for numeric scalar — "
                               "use bare literal form");
    }
    return std::unexpected("unsupported JSON type for scalar");
}

std::expected<SchemaValue, std::string>
jsonToValue(const SchemaGraph& g, const SchemaNode* node, const glz::generic& j) {
    if (!node) return std::unexpected("unresolved schema node");
    switch (node->kind) {
    case SchemaKind::Scalar:
        return jsonScalar(*node, j);
    case SchemaKind::String: {
        const auto* s = j.get_if<std::string>();
        if (!s) return std::unexpected("expected JSON string");
        return makeStringValue(node, *s);
    }
    case SchemaKind::Bytes: {
        const auto* s = j.get_if<std::string>();
        if (!s) return std::unexpected("expected hex string for bytes");
        auto b = parseHex(*s);
        if (!b) return std::unexpected(b.error());
        return makeBytesValue(node, std::move(*b));
    }
    case SchemaKind::Optional: {
        if (j.is_null()) return makeAbsentValue(node);
        const SchemaNode* el = resolve(g, *node->element);
        auto inner = jsonToValue(g, el, j);
        if (!inner) return std::unexpected(inner.error());
        return makeOptionalValue(node, std::move(*inner));
    }
    case SchemaKind::OneOf: {
        const auto* obj = j.get_if<glz::generic::object_t>();
        if (!obj || obj->size() != 1)
            return std::unexpected("oneof value must be {\"member\": v}");
        const auto& [name, val] = *obj->begin();
        for (const auto& [memberKey, memberRef] : node->oneOfMembers) {
            const SchemaNode* mn = resolve(g, memberRef);
            if (mn && mn->name == name) {
                auto inner = jsonToValue(g, mn, val);
                if (!inner) return std::unexpected(inner.error());
                return makeOneOfValue(node, memberKey, std::move(*inner));
            }
        }
        return std::unexpected("unknown oneof member '" + name + "'");
    }
    case SchemaKind::FixedArray:
    case SchemaKind::DynamicArray: {
        const auto* arr = j.get_if<glz::generic::array_t>();
        if (!arr) return std::unexpected("expected JSON array");
        const SchemaNode* el = resolve(g, *node->element);
        std::vector<SchemaValue> elems;
        for (const auto& item : *arr) {
            auto v = jsonToValue(g, el, item);
            if (!v) return std::unexpected(v.error());
            elems.push_back(std::move(*v));
        }
        return makeArrayValue(node, std::move(elems));
    }
    case SchemaKind::Struct: {
        const auto* obj = j.get_if<glz::generic::object_t>();
        if (!obj) return std::unexpected("expected JSON object for struct");
        std::vector<SchemaValue::Struct::Field> fields;
        for (const auto& [name, val] : *obj) {
            const SchemaField* fld = nullptr;
            for (const auto& f : node->fields) {
                if (f.name == name || std::to_string(f.key) == name) {
                    fld = &f; break;
                }
            }
            if (!fld)
                return std::unexpected("unknown field '" + name + "'");
            const SchemaNode* fn = resolve(g, fld->schema);
            auto v = jsonToValue(g, fn, val);
            if (!v) return std::unexpected(std::string(name) + ": " + v.error());
            fields.push_back(makeStructField(fld->key, fld, std::move(*v)));
        }
        return makeStructValue(node, std::move(fields));
    }
    case SchemaKind::Map: {
        const auto* obj = j.get_if<glz::generic::object_t>();
        if (!obj) return std::unexpected("expected JSON object for map");
        const SchemaNode* kn = resolve(g, *node->mapKey);
        const SchemaNode* vn = resolve(g, *node->mapValue);
        if (!kn || kn->kind != SchemaKind::String)
            return std::unexpected("JSON map requires string keys");
        std::vector<SchemaValue::Map::Entry> entries;
        for (const auto& [name, val] : *obj) {
            auto k = makeStringValue(kn, name);
            auto v = jsonToValue(g, vn, val);
            if (!v) return std::unexpected(v.error());
            entries.push_back(makeMapEntry(std::move(k), std::move(*v)));
        }
        return makeMapValue(node, std::move(entries));
    }
    case SchemaKind::Alias:
        return jsonToValue(g, resolve(g, *node->target), j);
    case SchemaKind::Enum: {
        const auto* d = j.get_if<double>();
        if (!d) return std::unexpected("expected enum number in JSON");
        return makeScalarValue(node, static_cast<uint64_t>(
            static_cast<int64_t>(*d)));
    }
    }
    return std::unexpected("unsupported schema kind");
}
#endif // TETHER_IO_HAS_GLAZE

} // namespace

// ---------------------------------------------------------------------------
// Bare-literal path
// ---------------------------------------------------------------------------

std::expected<SchemaValue, std::string>
parseValueLiteral(const SchemaGraph& graph, const SchemaNode& nodeIn,
                  const std::string& text) {
    const SchemaNode* node = &nodeIn;
    while (node && node->kind == SchemaKind::Alias && node->target) {
        node = graph.find(node->target->key);
    }
    if (!node) return std::unexpected("unresolved schema node");

    const bool looksJson = !text.empty() && (text[0] == '{' || text[0] == '[');
    if (looksJson) {
#if defined(TETHER_IO_HAS_GLAZE)
        auto parsed = glz::read_json<glz::generic>(text);
        if (!parsed) return std::unexpected("invalid JSON literal");
        return jsonToValue(graph, node, *parsed);
#else
        return std::unexpected("JSON input requires the glaze library");
#endif
    }

    switch (node->kind) {
    case SchemaKind::Scalar: {
        if (node->scalarType == ValueType::Enum) {
            // Enum literal: #index or plain number.
            std::string_view v = text;
            if (v.size() > 1 && v[0] == '#') v.remove_prefix(1);
            uint64_t idx;
            auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), idx);
            if (ec == std::errc() && ptr == v.data() + v.size())
                return makeScalarValue(node, idx);
            return std::unexpected("invalid enum literal '" + text +
                                   "' (use #N)");
        }
        auto bits = scalarBits(*node, text);
        if (!bits) return std::unexpected(bits.error());
        return makeScalarValue(node, *bits);
    }
    case SchemaKind::Enum: {
        std::string_view v = text;
        if (v.size() > 1 && v[0] == '#') v.remove_prefix(1);
        uint64_t idx;
        auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), idx);
        if (ec == std::errc() && ptr == v.data() + v.size())
            return makeScalarValue(node, idx);
        return std::unexpected("invalid enum literal '" + text + "' (use #N)");
    }
    case SchemaKind::String:
        return makeStringValue(node, unquote(text));
    case SchemaKind::Bytes: {
        auto b = parseHex(text);
        if (!b) return std::unexpected(b.error());
        return makeBytesValue(node, std::move(*b));
    }
    case SchemaKind::Optional:
        if (text == "none" || text == "null") return makeAbsentValue(node);
        {
            const SchemaNode* el = resolve(graph, *node->element);
            auto inner = parseValueLiteral(graph, *el, text);
            if (!inner) return std::unexpected(inner.error());
            return makeOptionalValue(node, std::move(*inner));
        }
    default:
        return std::unexpected(
            std::format("'{}' requires a JSON literal",
                        node->name.empty() ? "value" : node->name));
    }
}

std::expected<ParsedArg, std::string>
parseFunctionArg(const SchemaGraph& graph,
                 const ClientFunctionEntry& fn,
                 const std::vector<SchemaManifestEntry>& manifest,
                 const std::string& text,
                 uint32_t positionalIndex) {
    ParsedArg out;
    std::string name, val = text;
    if (auto eq = text.find('='); eq != std::string::npos) {
        name = text.substr(0, eq);
        val = text.substr(eq + 1);
    }

    // Escape hatch: `name=raw:deadbeef` sends literal bytes — needed when a
    // function's schema slot can't express the declared wire type.
    const bool rawBytes = val.rfind("raw:", 0) == 0;

    const ClientFunctionParam* param = nullptr;
    if (!name.empty()) {
        // name= or #key= form
        if (name.size() > 1 && name[0] == '#') {
            uint64_t k;
            if (std::from_chars(name.data() + 1, name.data() + name.size(), k).ec
                    == std::errc()) {
                for (const auto& p : fn.parameters)
                    if (p.key == k) { param = &p; break; }
            }
        } else {
            for (const auto& p : fn.parameters)
                if (p.name == name) { param = &p; break; }
        }
        if (!param)
            return std::unexpected("unknown parameter '" + name + "'");
    } else {
        if (positionalIndex >= fn.parameters.size())
            return std::unexpected("too many positional arguments");
        param = &fn.parameters[positionalIndex];
    }

    const auto idx = static_cast<uint32_t>(
        param - fn.parameters.data());
    out.position = idx;
    out.key = param->key;
    out.type = param->type;

    if (rawBytes) {
        auto b = parseHex(val.substr(4));
        if (!b) return std::unexpected(b.error());
        out.value = std::move(*b);
        return out;
    }

    // Schema-slot path: the catalog type for this parameter.
    const SchemaNode* n = nullptr;
    if (param->schemaSlot < manifest.size())
        n = graph.find(manifest[param->schemaSlot].ref.key);
    if (!n) {
        return std::unexpected(std::format(
            "argument '{}' has no usable schema slot ({}) — "
            "use name=raw:<hex> to send literal bytes",
            name.empty() ? val : name, param->schemaSlot));
    }
    auto v = parseValueLiteral(graph, *n, val);
    if (!v) {
        return std::unexpected(std::format(
            "{}: {} (or use name=raw:<hex>)", val, v.error()));
    }
    auto bytes = encodeSchemaValue(graph, *v);
    if (!bytes) return std::unexpected(val + ": " + bytes.error());
    out.value = std::move(*bytes);
    return out;
}

} // namespace tether::io::client
