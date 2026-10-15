/**
 * @file client_session.cpp
 * @brief URL→transport factory and catalog/name resolution for
 *        tether_io_client.
 */
#include "client_session.hpp"

#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"
#include "tether/io/WsTransport.hpp"
#if !defined(ESP_PLATFORM)
#include "tether/io/SerialTransport.hpp"
#endif

#include <algorithm>
#include <charconv>
#include <cctype>
#include <format>
#include <regex>

namespace tether::io::client {

namespace {

struct Url {
    std::string scheme;
    std::string host;
    uint16_t port = 0;
    std::string path;      // includes leading '/'
    std::string query;     // after '?'
    std::string resource;  // serial device path
};

bool splitOnce(std::string_view s, char c, std::string_view& a, std::string_view& b) {
    const auto pos = s.find(c);
    if (pos == std::string_view::npos) return false;
    a = s.substr(0, pos);
    b = s.substr(pos + 1);
    return true;
}

std::expected<Url, std::string> parseUrl(std::string_view in) {
    Url u;
    std::string_view rest = in;
    if (auto p = rest.find("://"); p != std::string_view::npos) {
        u.scheme = std::string(rest.substr(0, p));
        rest = rest.substr(p + 3);
    } else {
        u.scheme = "tcp";   // bare host[:port]
    }

    if (auto q = rest.find('?'); q != std::string_view::npos) {
        u.query = std::string(rest.substr(q + 1));
        rest = rest.substr(0, q);
    }

    if (u.scheme == "serial") {
        // serial:///dev/ttyUSB0 — the whole remainder is the device path
        u.resource = std::string(rest);
        if (u.resource.empty())
            return std::unexpected("serial: missing device path");
        return u;
    }

    std::string_view hostport = rest, path;
    if (auto p = rest.find('/'); p != std::string_view::npos) {
        hostport = rest.substr(0, p);
        path = rest.substr(p);
    }
    u.path = path.empty() ? "/" : std::string(path);

    std::string_view host, portStr;
    if (!hostport.empty() && hostport.front() == '[') {
        auto close = hostport.find(']');
        if (close == std::string_view::npos)
            return std::unexpected("bad IPv6 literal");
        host = hostport.substr(1, close - 1);
        if (close + 1 < hostport.size() && hostport[close + 1] == ':')
            portStr = hostport.substr(close + 2);
    } else if (!splitOnce(hostport, ':', host, portStr)) {
        host = hostport;
    }
    if (host.empty()) return std::unexpected("missing host");
    u.host = std::string(host);
    if (!portStr.empty()) {
        unsigned long p = 0;
        auto [ptr, ec] = std::from_chars(
            portStr.data(), portStr.data() + portStr.size(), p);
        if (ec != std::errc() || ptr != portStr.data() + portStr.size() ||
            p == 0 || p > 65535)
            return std::unexpected("bad port");
        u.port = static_cast<uint16_t>(p);
    }
    if (u.port == 0) u.port = (u.scheme == "ws") ? 80 : DEFAULT_PORT;
    return u;
}

std::string urlEncode(const std::string& s) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : s) {
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
            out.push_back(static_cast<char>(c));
        else {
            out.push_back('%');
            out.push_back(kHex[c >> 4]);
            out.push_back(kHex[c & 0xF]);
        }
    }
    return out;
}

} // namespace

std::expected<std::unique_ptr<ITransport>, std::string>
makeIoTransport(const ConnectSpec& spec) {
    auto url = parseUrl(spec.url);
    if (!url) return std::unexpected(url.error());

    if (url->scheme == "ws") {
        WsClientTransport::Options opts;
        opts.host = url->host;
        opts.handshakeTimeoutMs = spec.connectTimeoutMs;
        std::string target = url->path;
        std::string query = url->query;
        auto addParam = [&](const char* k, const std::string& v) {
            if (v.empty()) return;
            query += query.empty() ? "" : "&";
            query += std::string(k) + "=" + urlEncode(v);
        };
        addParam("token", spec.token);
        addParam("role", spec.role);
        if (!query.empty()) {
            target += (target.find('?') == std::string::npos) ? "?" : "&";
            target += query;
        }
        opts.target = target;
        if (!spec.token.empty())
            opts.headers.push_back({"Authorization", "Bearer " + spec.token});
        return WsClientTransport::connect(url->host.c_str(), url->port, opts);
    }

    if (url->scheme == "tcp") {
        if (!spec.token.empty() || !spec.role.empty())
            return std::unexpected("--token/--role apply only to ws:// URLs");
        auto tcp = tcpConnect(url->host.c_str(), url->port, spec.connectTimeoutMs);
        if (!tcp)
            return std::unexpected(std::format("cannot connect to {}:{}",
                                             url->host, url->port));
        return std::unique_ptr<ITransport>(
            std::make_unique<SlipFramedTransport>(std::move(tcp)).release());
    }

    if (url->scheme == "serial") {
#if !defined(ESP_PLATFORM)
        uint32_t baud = 115200;
        if (!url->query.empty()) {
            std::string_view k, v;
            if (splitOnce(url->query, '=', k, v) && k == "baud") {
                std::from_chars(v.data(), v.data() + v.size(), baud);
            }
        }
        auto drv = std::make_unique<PosixSerialDriver>();
        if (!drv->open(url->resource.c_str(), baud))
            return std::unexpected(std::format("cannot open serial port {}",
                                             url->resource));
        return std::unique_ptr<ITransport>(
            std::make_unique<SlipFramedTransport>(
                std::make_unique<SerialTransport>(std::move(drv))).release());
#else
        return std::unexpected("serial transport unavailable on ESP");
#endif
    }

    return std::unexpected(
        std::format("unknown URL scheme '{}'", url->scheme));
}

// ---------------------------------------------------------------------------
// IoSession — catalog + resolution
// ---------------------------------------------------------------------------

std::expected<void, std::string> IoSession::refreshCatalog() {
    params_.clear();
    signals_.clear();
    functions_.clear();
    for (uint32_t off = 0;;) {
        auto r = client_.listParams(off, 1000);
        if (!r) return std::unexpected(r.error().message);
        params_.insert(params_.end(), r->begin(), r->end());
        if (r->size() < 1000) break;
        off += static_cast<uint32_t>(r->size());
    }
    for (uint32_t off = 0;;) {
        auto r = client_.listSignals(off, 1000);
        if (!r) return std::unexpected(r.error().message);
        signals_.insert(signals_.end(), r->begin(), r->end());
        if (r->size() < 1000) break;
        off += static_cast<uint32_t>(r->size());
    }
    for (uint32_t off = 0;;) {
        auto r = client_.listFunctions(off, 1000);
        if (!r) return std::unexpected(r.error().message);
        functions_.insert(functions_.end(), r->begin(), r->end());
        if (r->size() < 1000) break;
        off += static_cast<uint32_t>(r->size());
    }
    return {};
}

namespace {

bool parseId(const std::string& s, uint64_t& id) {
    std::string_view v = s;
    int base = 10;
    if (v.size() > 2 && v.substr(0, 2) == "0x") { base = 16; v.remove_prefix(2); }
    if (v.empty()) return false;
    auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), id, base);
    return ec == std::errc() && ptr == v.data() + v.size();
}

const ClientCatalogEntry* findIn(const std::vector<ClientCatalogEntry>& list,
                                 const std::string& name, uint64_t id) {
    for (const auto& e : list) {
        if (e.name == name) return &e;
        if (!e.group.empty() && e.group + "." + e.name == name) return &e;
    }
    for (const auto& e : list) if (e.id == id) return &e;
    return nullptr;
}

} // namespace

const ClientCatalogEntry* IoSession::findEntry(const std::string& name,
                                               EntryKindSel kind) const {
    std::string n = name;
    if (n.rfind("p:", 0) == 0)      { kind = EntryKindSel::Param;   n.erase(0, 2); }
    else if (n.rfind("s:", 0) == 0) { kind = EntryKindSel::Signal;  n.erase(0, 2); }
    else if (n.rfind("f:", 0) == 0) { return nullptr; /* handled by findFunction */ }

    uint64_t id = 0;
    const bool numeric = parseId(n, id);

    const ClientCatalogEntry* found = nullptr;
    const ClientCatalogEntry* other = nullptr;   // for ambiguity reporting
    if (kind != EntryKindSel::Signal) {
        if (auto* e = findIn(params_, n, id); e) { found = e; }
    }
    if (kind != EntryKindSel::Param) {
        if (auto* e = findIn(signals_, n, id); e) {
            if (found) { other = e; } else { found = e; }
        }
    }
    (void)other;   // ambiguity is resolved by the caller via prefix
    return found;
}

const ClientFunctionEntry* IoSession::findFunction(const std::string& name) const {
    std::string n = name;
    if (n.rfind("f:", 0) == 0) n.erase(0, 2);
    uint64_t id = 0;
    const bool numeric = parseId(n, id);
    for (const auto& f : functions_) {
        if (f.name == n ||
            (!f.group.empty() && f.group + "." + f.name == n) ||
            (numeric && f.id == id)) {
            return &f;
        }
    }
    return nullptr;
}

namespace {


std::string kindName(const SchemaNode* n) {
    if (!n) return "?";
    switch (n->kind) {
        case SchemaKind::Scalar:
        case SchemaKind::Enum:
            return vtName(n->scalarType);
        case SchemaKind::String:    return "string";
        case SchemaKind::Bytes:     return "bytes";
        case SchemaKind::FixedArray:
            return std::format("{}[{}]", vtName(ValueType::Array),
                               n->fixedCount);
        case SchemaKind::DynamicArray: return "array";
        case SchemaKind::Map:       return "map";
        case SchemaKind::Struct:    return "struct";
        case SchemaKind::Optional:  return "optional";
        case SchemaKind::OneOf:     return "oneof";
        case SchemaKind::Alias:     return "alias";
    }
    return "?";
}

const SchemaNode* resolveSlot(const IoSession&, const SchemaGraph& graph,
                              const std::vector<SchemaManifestEntry>& manifest,
                              uint32_t slot) {
    if (slot >= manifest.size()) return nullptr;
    return graph.find(manifest[slot].ref.key);
}

} // namespace

std::string IoSession::typeNameOf(const ClientCatalogEntry& e) const {
    return kindName(resolveSlot(*this, client_.schemaGraph(),
                                client_.schemaManifest(), e.schemaSlot));
}

std::string IoSession::functionSignature(const ClientFunctionEntry& f) const {
    std::string sig = f.name + "(";
    for (size_t i = 0; i < f.parameters.size(); ++i) {
        const auto& p = f.parameters[i];
        if (i) sig += ", ";
        sig += p.name;
        sig += ": ";
        sig += kindName(resolveSlot(*this, client_.schemaGraph(),
                                    client_.schemaManifest(), p.schemaSlot));
        if (p.hasDefault) sig += " = default";
        if (p.flags != 0) sig += "*";
    }
    sig += ")";
    if (f.hasReturnValue) {
        sig += " -> ";
        sig += kindName(resolveSlot(*this, client_.schemaGraph(),
                                    client_.schemaManifest(), f.returnSchemaSlot));
    }
    return sig;
}

// ---------------------------------------------------------------------------
// Filtering helpers
// ---------------------------------------------------------------------------

bool globMatch(std::string_view pat, std::string_view text) {
    // Iterative * ? matcher.
    size_t p = 0, t = 0, starP = std::string_view::npos, starT = 0;
    while (t < text.size()) {
        if (p < pat.size() && (pat[p] == '?' || pat[p] == text[t])) {
            ++p; ++t;
        } else if (p < pat.size() && pat[p] == '*') {
            starP = p++; starT = t;
        } else if (starP != std::string_view::npos) {
            p = starP + 1; t = ++starT;
        } else {
            return false;
        }
    }
    while (p < pat.size() && pat[p] == '*') ++p;
    return p == pat.size();
}

bool regexMatch(const std::string& pattern, std::string_view text) {
    if (pattern.empty()) return true;
    try {
        return std::regex_search(std::string(text), std::regex(pattern));
    } catch (...) {
        return false;   // bad regex — treat as no match
    }
}

std::vector<const ClientCatalogEntry*>
filterEntries(const std::vector<ClientCatalogEntry>& entries,
              const std::string& glob, const std::string& substr,
              const std::string& regex) {
    std::vector<const ClientCatalogEntry*> out;
    for (const auto& e : entries) {
        if (!glob.empty() && !globMatch(glob, e.name) &&
            (e.group.empty() || !globMatch(glob, e.group + "." + e.name)))
            continue;
        if (!substr.empty() && e.name.find(substr) == std::string::npos &&
            e.group.find(substr) == std::string::npos)
            continue;
        if (!regex.empty() && !regexMatch(regex, e.name) &&
            !regexMatch(regex, e.group + "." + e.name))
            continue;
        out.push_back(&e);
    }
    return out;
}

void IoSession::startPump(uint32_t pollMs) {
    if (pumpRunning()) return;
    pumpStop_ = false;
    pumpThread_ = std::thread([this, pollMs] {
        while (!pumpStop_.load(std::memory_order_relaxed)) {
            // Serialized vs request() by the client's internal mutex —
            // dispatch only; errors are ignored (timeout is normal).
            client_.receiveMessage(pollMs);
        }
    });
}

void IoSession::stopPump() {
    pumpStop_ = true;
    if (pumpThread_.joinable()) pumpThread_.join();
}

const char* vtName(ValueType t) {
    switch (t) {
        case ValueType::U8: return "u8";   case ValueType::U16: return "u16";
        case ValueType::U32: return "u32"; case ValueType::U64: return "u64";
        case ValueType::I8: return "i8";   case ValueType::I16: return "i16";
        case ValueType::I32: return "i32"; case ValueType::I64: return "i64";
        case ValueType::F32: return "f32"; case ValueType::F64: return "f64";
        case ValueType::Bool: return "bool"; case ValueType::String: return "string";
        case ValueType::Binary: return "bytes"; case ValueType::IPv4: return "ipv4";
        case ValueType::IPv6: return "ipv6"; case ValueType::MAC: return "mac";
        case ValueType::Enum: return "enum"; case ValueType::UVarint: return "uvarint";
        case ValueType::IVarint: return "ivarint"; case ValueType::Struct: return "struct";
        case ValueType::Array: return "array"; case ValueType::Stream: return "stream";
    }
    return "?";
}

} // namespace tether::io::client
