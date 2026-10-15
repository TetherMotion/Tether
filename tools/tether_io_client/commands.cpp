/**
 * @file commands.cpp
 * @brief Command implementations for tether_io_client.
 */
#include "commands.hpp"
#include "value_text.hpp"

#include "tether/io/Datalogging.hpp"
#include "tether/io/ThresholdFilter.hpp"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <format>
#include <fstream>
#include <thread>

namespace tether::io::client {

namespace {

std::atomic<bool> g_stop{false};

void onSigint(int) { g_stop = true; }

struct SigintGuard {
    struct sigaction old;
    SigintGuard() {
        g_stop = false;
        struct sigaction sa{};
        sa.sa_handler = onSigint;
        sigaction(SIGINT, &sa, &old);
    }
    ~SigintGuard() { sigaction(SIGINT, &old, nullptr); }
};

// ---- output helpers -------------------------------------------------------

/// Minimal aligned table writer.
void printTable(std::ostream& out,
                const std::vector<std::string>& headers,
                const std::vector<std::vector<std::string>>& rows) {
    std::vector<size_t> w(headers.size());
    for (size_t i = 0; i < headers.size(); ++i) w[i] = headers[i].size();
    for (const auto& r : rows)
        for (size_t i = 0; i < r.size() && i < w.size(); ++i)
            w[i] = std::max(w[i], r[i].size());
    auto printRow = [&](const std::vector<std::string>& r) {
        for (size_t i = 0; i < r.size(); ++i) {
            if (i) out << "  ";
            out << r[i];
            for (size_t s = r[i].size(); s < w[i]; ++s) out << ' ';
        }
        out << '\n';
    };
    printRow(headers);
    std::vector<std::string> sep;
    for (size_t i = 0; i < w.size(); ++i) sep.push_back(std::string(w[i], '-'));
    printRow(sep);
    for (const auto& r : rows) printRow(r);
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20)
                    out += std::format("\\u{:04x}", c);
                else out += c;
        }
    }
    return out;
}

bool argFlag(const std::vector<std::string>& args, const std::string& f) {
    return std::find(args.begin(), args.end(), f) != args.end();
}

std::string argValue(const std::vector<std::string>& args,
                     const std::string& f, const std::string& def = {}) {
    auto it = std::find(args.begin(), args.end(), f);
    if (it != args.end() && it + 1 != args.end()) return *(it + 1);
    for (const auto& a : args)
        if (a.rfind(f + "=", 0) == 0) return a.substr(f.size() + 1);
    return def;
}

std::vector<std::string> positional(const std::vector<std::string>& args) {
    std::vector<std::string> out;
    for (size_t i = 1; i < args.size(); ++i) {
        const auto& a = args[i];
        if (a.empty() || a[0] == '-') {
            // skip the value of --flag value pairs
            if ((a == "--regex" || a == "--glob" || a == "--filter" ||
                 a == "--interval-ms" || a == "--interval-ms" ||
                 a == "--duration" || a == "--count" || a == "--chunk" ||
                 a == "--deadline-us" || a == "--severity" || a == "--rate-hz" ||
                 a == "--name" || a == "--threshold" || a == "--type" ||
                 a == "--rule" || a == "--keys" || a == "--id" ||
                 a == "--record" || a == "--ops") &&
                i + 1 < args.size()) ++i;
            continue;
        }
        out.push_back(a);
    }
    return out;
}

std::expected<const ClientCatalogEntry*, std::string>
needEntry(Ctx& ctx, const std::string& name) {
    const auto* e = ctx.session.findEntry(name);
    if (!e)
        return std::unexpected(std::format("unknown entry '{}'", name));
    return e;
}

std::expected<void, std::string> ensureCatalog(Ctx& ctx) {
    if (ctx.session.params().empty() && ctx.session.signals().empty() &&
        ctx.session.functions().empty()) {
        return ctx.session.refreshCatalog();
    }
    return {};
}

// ---- commands --------------------------------------------------------------

int cmdPing(Ctx& ctx, const std::vector<std::string>& args) {
    int count = 1;
    if (auto c = argValue(args, "--count"); !c.empty())
        std::from_chars(c.data(), c.data() + c.size(), count);
    const uint32_t intervalMs = static_cast<uint32_t>(
        std::stoul(argValue(args, "--interval", "0")));
    for (int i = 0; i < count && !g_stop; ++i) {
        if (i && intervalMs)
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        const auto t0 = std::chrono::steady_clock::now();
        auto r = ctx.session.client().ping(0xC0FFEE + i);
        if (!r) {
            ctx.err << "ping: " << r.error().message << '\n';
            return 1;
        }
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        ctx.out << std::format("pong nonce={:#x} time={} us\n", *r, us);
    }
    return 0;
}

int cmdInfo(Ctx& ctx, const std::vector<std::string>&) {
    const auto& c = ctx.session.client();
    ctx.out << std::format("schema epoch    : {}\n", c.schemaEpoch());
    ctx.out << std::format("schema slots    : {}\n", c.schemaManifest().size());
    ctx.out << std::format("schema nodes    : {}\n", c.schemaGraph().nodes.size());
    const auto& lim = c.serverLimits();
    ctx.out << std::format("limits          : msg={}B desc={}B defs={} "
                           "value={}B depth={} entries={}\n",
                           lim.maxMessageBytes, lim.maxDescriptorBytes,
                           lim.maxDefinitions, lim.maxValueBytes,
                           lim.maxValueDepth, lim.maxCollectionEntries);
    ctx.out << std::format("features        : {:#010x}\n", c.negotiatedFeatures());
    if (auto r = ensureCatalog(ctx); !r) {
        ctx.err << "catalog: " << r.error() << '\n';
        return 1;
    }
    ctx.out << std::format("catalog         : {} params, {} signals, {} functions\n",
                           ctx.session.params().size(),
                           ctx.session.signals().size(),
                           ctx.session.functions().size());
    return 0;
}

int cmdList(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const std::string what = positional(args).empty() ? "params"
                                                     : positional(args)[0];
    const std::string glob = argValue(args, "--glob");
    const std::string substr = argValue(args, "--filter");
    const std::string re = argValue(args, "--regex");

    if (what == "functions") {
        const auto& fns = ctx.session.functions();
        if (ctx.json) {
            ctx.out << "[";
            for (size_t i = 0; i < fns.size(); ++i) {
                const auto& f = fns[i];
                if (i) ctx.out << ",";
                ctx.out << std::format(
                    "{{\"id\":{},\"name\":\"{}\",\"group\":\"{}\"}}",
                    f.id, jsonEscape(f.name), jsonEscape(f.group));
            }
            ctx.out << "]\n";
            return 0;
        }
        std::vector<std::vector<std::string>> rows;
        for (const auto& f : fns) {
            if (!glob.empty() && !globMatch(glob, f.name) &&
                !globMatch(glob, f.group + "." + f.name)) continue;
            if (!substr.empty() && f.name.find(substr) == std::string::npos &&
                f.group.find(substr) == std::string::npos) continue;
            if (!re.empty() && !regexMatch(re, f.name) &&
                !regexMatch(re, f.group + "." + f.name)) continue;
            rows.push_back({std::format("{}", f.id),
                            f.group.empty() ? f.name
                                            : f.group + "." + f.name,
                            ctx.session.functionSignature(f)});
        }
        printTable(ctx.out, {"ID", "NAME", "SIGNATURE"}, rows);
        return 0;
    }

    const auto& entries = (what == "signals") ? ctx.session.signals()
                                              : ctx.session.params();
    const bool writable = (what == "params");
    const auto filtered = filterEntries(entries, glob, substr, re);
    if (ctx.json) {
        ctx.out << "[";
        for (size_t i = 0; i < filtered.size(); ++i) {
            const auto* e = filtered[i];
            if (i) ctx.out << ",";
            ctx.out << std::format(
                "{{\"id\":{},\"name\":\"{}\",\"group\":\"{}\",\"type\":\"{}\"}}",
                e->id, jsonEscape(e->name), jsonEscape(e->group),
                ctx.session.typeNameOf(*e));
        }
        ctx.out << "]\n";
        return 0;
    }
    std::vector<std::vector<std::string>> rows;
    for (const auto* e : filtered) {
        rows.push_back({std::format("{}", e->id),
                        e->group.empty() ? e->name : e->group + "." + e->name,
                        ctx.session.typeNameOf(*e),
                        e->description});
    }
    printTable(ctx.out, {"ID", "NAME", "TYPE",
                         writable ? "DESCRIPTION" : "DESCRIPTION"}, rows);
    return 0;
}

int cmdGet(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const bool raw = argFlag(args, "--raw") || argFlag(args, "-r");
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "get: need at least one name\n"; return 2; }

    const bool multi = names.size() > 1;
    for (const auto& n : names) {
        auto e = needEntry(ctx, n);
        if (!e) { ctx.err << e.error() << '\n'; return 1; }
        const bool isParam =
            std::find_if(ctx.session.params().begin(),
                         ctx.session.params().end(),
                         [&](const ClientCatalogEntry& p) {
                             return p.id == (*e)->id;
                         }) != ctx.session.params().end();
        auto rawResult = isParam ? ctx.session.client().getParam((*e)->id)
                                 : ctx.session.client().getSignal((*e)->id);
        if (!rawResult) {
            ctx.err << std::format("{}: {}\n", n, rawResult.error().message);
            return 1;
        }
        if (raw) {
            for (uint8_t b : *rawResult) ctx.out << std::format("{:02x}", b);
            ctx.out << '\n';
            continue;
        }
        auto v = ctx.session.decodeEntry(**e, *rawResult);
        if (!v) {
            ctx.err << std::format("{}: decode: {}\n", n, v.error().message);
            return 1;
        }
        if (ctx.json) {
            ctx.out << std::format("{{\"name\":\"{}\",\"value\":{}}}\n",
                                   jsonEscape(n),
                                   schemaValueToJson(*v, &ctx.session.client().schemaGraph()));
        } else if (multi) {
            ctx.out << std::format("{:>24} = {}\n", n, displayValue(*v));
        } else {
            ctx.out << displayValue(*v) << '\n';
        }
    }
    return 0;
}

int cmdSet(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const auto names = positional(args);
    if (names.size() != 2) { ctx.err << "usage: set NAME VALUE\n"; return 2; }
    auto e = ctx.session.findEntry(names[0], EntryKindSel::Param);
    if (!e) {
        ctx.err << std::format("unknown param '{}' (params only; use s: "
                               "prefix is not writable)\n", names[0]);
        return 1;
    }
    const SchemaNode* node = ctx.session.client().resolveSchemaSlot(e->schemaSlot);
    if (!node) { ctx.err << "no schema for entry\n"; return 1; }
    auto v = parseValueLiteral(ctx.session.client().schemaGraph(), *node,
                               names[1]);
    if (!v) { ctx.err << names[1] << ": " << v.error() << '\n'; return 1; }
    auto bytes = encodeSchemaValue(ctx.session.client().schemaGraph(), *v);
    if (!bytes) { ctx.err << "encode: " << bytes.error() << '\n'; return 1; }
    auto r = ctx.session.client().setParamVar(e->id, bytes->data(), bytes->size());
    if (!r) { ctx.err << "set: " << r.error().message << '\n'; return 1; }
    ctx.out << std::format("{} = {}\n", names[0], displayValue(*v));
    return 0;
}

int cmdSnapshot(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const std::string glob = argValue(args, "--glob", "*");
    const std::string substr = argValue(args, "--filter");
    const bool params = argFlag(args, "params") || !argFlag(args, "signals");
    const auto filtered = filterEntries(
        params ? ctx.session.params() : ctx.session.signals(), glob, substr);

    std::vector<uint64_t> ids;
    for (const auto* e : filtered) ids.push_back(e->id);
    if (ids.empty()) { ctx.out << "(no entries)\n"; return 0; }
    auto res = params ? ctx.session.client().snapshotParams(ids)
                      : ctx.session.client().snapshotSignals(ids);
    if (!res) { ctx.err << "snapshot: " << res.error().message << '\n'; return 1; }

    // name lookup
    auto nameOf = [&](uint64_t id) -> const ClientCatalogEntry* {
        for (const auto& e : (params ? ctx.session.params()
                                     : ctx.session.signals()))
            if (e.id == id) return &e;
        return nullptr;
    };
    if (ctx.json) {
        ctx.out << std::format("{{\"timestamp_us\":{},\"values\":{{",
                               res->first);
        bool first = true;
        for (const auto& sv : res->second) {
            const auto* e = nameOf(sv.id);
            auto v = ctx.session.decodeEntry(
                *e, std::vector<uint8_t>(sv.value.begin(), sv.value.end()));
            if (!first) ctx.out << ",";
            first = false;
            ctx.out << std::format("\"{}\":{}",
                jsonEscape(e ? e->name : std::to_string(sv.id)),
                v ? schemaValueToJson(*v, &ctx.session.client().schemaGraph())
                  : "null");
        }
        ctx.out << "}}\n";
        return 0;
    }
    std::vector<std::vector<std::string>> rows;
    for (const auto& sv : res->second) {
        const auto* e = nameOf(sv.id);
        auto v = ctx.session.decodeEntry(
            *e, std::vector<uint8_t>(sv.value.begin(), sv.value.end()));
        rows.push_back({e ? e->name : std::to_string(sv.id),
                        e ? ctx.session.typeNameOf(*e) : "?",
                        v ? displayValue(*v) : "<decode error>"});
    }
    printTable(ctx.out, {"NAME", "TYPE", "VALUE"}, rows);
    return 0;
}

int cmdMetadata(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "metadata: need a name\n"; return 2; }
    auto e = needEntry(ctx, names[0]);
    if (!e) { ctx.err << e.error() << '\n'; return 1; }
    auto m = ctx.session.client().getMetadata((*e)->id);
    if (!m) { ctx.err << "metadata: " << m.error().message << '\n'; return 1; }
    if (ctx.json) {
        ctx.out << "[";
        for (size_t i = 0; i < m->size(); ++i) {
            if (i) ctx.out << ",";
            ctx.out << std::format("{{\"key\":\"{}\",\"value\":\"{}\"}}",
                                   jsonEscape((*m)[i].key),
                                   jsonEscape((*m)[i].value));
        }
        ctx.out << "]\n";
        return 0;
    }
    for (const auto& kv : *m)
        ctx.out << std::format("{:>16} = {}\n", kv.key, kv.value);
    if (m->empty()) ctx.out << "(no metadata)\n";
    return 0;
}

int cmdCall(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const bool ex = argFlag(args, "--invoke-ex");
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "usage: call NAME [k=v ...] [--invoke-ex]\n"; return 2; }
    const auto* fn = ctx.session.findFunction(names[0]);
    if (!fn) {
        ctx.err << std::format("unknown function '{}' (try 'list functions')\n",
                               names[0]);
        return 1;
    }
    // Real access.danger check: fetch the function's metadata (the wire
    // catalog doesn't carry it — GetMetadata does).
    {
        auto md = ctx.session.client().getMetadata(fn->id);
        bool dangerous = false;
        std::string role;
        if (md) {
            for (const auto& [k, v] : *md) {
                if (k == "access.danger" &&
                    (v == "1" || v == "true" || v == "yes")) dangerous = true;
                if (k == "access.role") role = v;
            }
        }
        if (dangerous) {
            ctx.out << std::format("!! '{}' is marked access.danger{}\n",
                                   names[0],
                                   role.empty() ? "" :
                                       " (role " + role + ")");
            if (!argFlag(args, "--yes") && !argFlag(args, "-y") &&
                !argFlag(args, "--no-confirm")) {
                ctx.err << "aborted (pass --yes to confirm)\n";
                return 1;
            }
        }
    }
    std::vector<TetherIOClient::FunctionArg> fnArgs;
    uint32_t positional = 0;
    for (size_t i = 1; i < names.size(); ++i) {
        auto pa = parseFunctionArg(ctx.session.client().schemaGraph(), *fn,
                                   ctx.session.client().schemaManifest(),
                                   names[i], positional++);
        if (!pa) { ctx.err << pa.error() << '\n'; return 1; }
        TetherIOClient::FunctionArg fa;
        fa.position = pa->position;
        fa.key = pa->key;
        fa.type = pa->type;
        fa.value = std::move(pa->value);
        fnArgs.push_back(std::move(fa));
    }
    const uint64_t deadline = static_cast<uint64_t>(
        std::stoull(argValue(args, "--deadline-us", "0")));
    auto r = ex ? ctx.session.client().invokeEx(fn->id, fnArgs, deadline)
                : ctx.session.client().callFunction(fn->id, fnArgs);
    if (!r) { ctx.err << "call: " << r.error().message << '\n'; return 1; }
    if (!r->success) {
        ctx.err << std::format("call: remote error {}: {}\n",
                               r->errorCode, r->errorMessage);
        return 1;
    }
    if (r->hasReturnValue && !r->returnValue.empty()) {
        auto v = ctx.session.client().decodeValue(fn->returnSchemaSlot,
                                                  r->returnValue.data(),
                                                  r->returnValue.size());
        if (v) {
            ctx.out << (ctx.json
                ? schemaValueToJson(*v, &ctx.session.client().schemaGraph())
                : displayValue(*v)) << '\n';
            return 0;
        }
        ctx.out << "(unparseable return: " << r->returnValue.size() << " B)\n";
    } else {
        ctx.out << "ok\n";
    }
    return 0;
}

int cmdStream(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "stream: need at least one entry\n"; return 2; }
    std::vector<uint64_t> ids;
    std::vector<const ClientCatalogEntry*> entries;
    for (const auto& n : names) {
        auto e = needEntry(ctx, n);
        if (!e) { ctx.err << e.error() << '\n'; return 1; }
        ids.push_back((*e)->id);
        entries.push_back(*e);
    }
    const uint32_t intervalMs = static_cast<uint32_t>(
        std::stoul(argValue(args, "--interval-ms", "20")));
    const uint32_t chunk = static_cast<uint32_t>(
        std::stoul(argValue(args, "--chunk", "1")));
    const double duration = std::stod(argValue(args, "--duration", "0"));
    const uint64_t maxRows = std::stoull(argValue(args, "--count", "0"));
    const std::string record = argValue(args, "--record");
    const bool stats = argFlag(args, "--stats");
    const bool csv = argFlag(args, "--csv");
    const uint32_t skip = static_cast<uint32_t>(
        std::stoul(argValue(args, "--skip", "0")));
    const bool onChange = argFlag(args, "--on-change");
    const std::string triggerName = argValue(args, "--trigger");
    uint64_t triggerId = 0;
    if (onChange && !triggerName.empty()) {
        auto te = needEntry(ctx, triggerName);
        if (!te) { ctx.err << te.error() << '\n'; return 1; }
        triggerId = (*te)->id;
    }
    std::ofstream rec;
    if (!record.empty()) {
        rec.open(record);
        if (!rec) { ctx.err << "cannot open " << record << '\n'; return 1; }
    }

    auto spec = ctx.session.client().configureStream(
        ids, intervalMs, chunk, skip,
        static_cast<uint8_t>(onChange ? TriggerMode::OnChange
                                      : TriggerMode::Time),
        triggerId);
    if (!spec) {
        ctx.err << "configureStream: " << spec.error().message << '\n';
        return 1;
    }
    auto started = ctx.session.client().startStream();
    if (!started) {
        ctx.err << "startStream: " << started.error().message << '\n';
        return 1;
    }
    const auto& layout = spec->layout;

    // layout entries carry ids — map back to catalog names for display
    auto entryName = [&](uint64_t id) -> std::string {
        for (const auto& e : ctx.session.signals())
            if (e.id == id) return e.name;
        for (const auto& e : ctx.session.params())
            if (e.id == id) return e.name;
        return std::to_string(id);
    };

    // header
    if (ctx.json) {
        ctx.out << "[\n";
    } else {
        std::vector<std::string> cols = {"timestamp_us"};
        for (const auto& e : layout) cols.push_back(entryName(e.id));
        printTable(ctx.out, cols, {});
    }

    std::atomic<uint64_t> rows{0};
    std::atomic<bool> stopping{false};
    const auto t0 = std::chrono::steady_clock::now();
    // gap stats: non-increasing timestamps + inter-row interval spread
    uint64_t lastTs = 0, gaps = 0, minGapUs = UINT64_MAX, maxGapUs = 0;
    const char sep = csv ? ',' : '\t';
    ctx.session.client().setStreamDataCallback(
        [&](const ClientStreamRow& row) {
            if (ctx.json) {
                ctx.out << (rows ? "," : "") << std::format(
                    "{{\"t\":{}", row.timestampUs);
                for (size_t i = 0; i < row.values.size() && i < layout.size(); ++i) {
                    auto v = ctx.session.client().decodeValue(
                        layout[i].schemaSlot, row.values[i].data(),
                        row.values[i].size());
                    ctx.out << std::format(",\"{}\":{}",
                        jsonEscape(entryName(layout[i].id)),
                        v ? schemaValueToJson(*v, &ctx.session.client().schemaGraph())
                          : "null");
                }
                ctx.out << "}\n";
            } else {
                ctx.out << row.timestampUs;
                for (size_t i = 0; i < row.values.size() && i < layout.size(); ++i) {
                    auto v = ctx.session.client().decodeValue(
                        layout[i].schemaSlot, row.values[i].data(),
                        row.values[i].size());
                    ctx.out << sep
                            << (v ? displayValue(*v) : std::string("<err>"));
                }
                ctx.out << '\n';
            }
            // gap detection: non-increasing timestamp = drop/out-of-order
            if (lastTs && row.timestampUs > lastTs) {
                const uint64_t g = row.timestampUs - lastTs;
                minGapUs = std::min(minGapUs, g);
                maxGapUs = std::max(maxGapUs, g);
            } else if (lastTs) {
                ++gaps;
            }
            lastTs = row.timestampUs;
            ++rows;
            if (rec) {
                rec << row.timestampUs;
                for (size_t i = 0; i < row.values.size() && i < layout.size(); ++i) {
                    auto v = ctx.session.client().decodeValue(
                        layout[i].schemaSlot, row.values[i].data(),
                        row.values[i].size());
                    rec << '\t' << (v ? displayValue(*v) : "<err>");
                }
                rec << '\n';
            }
            ctx.out.flush();
            const bool done =
                (maxRows && rows >= maxRows) ||
                (duration > 0 &&
                 std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - t0).count() >= duration);
            if (done && !stopping.exchange(true)) {
                // receiveMessage() keeps dispatching async frames while
                // data flows — only stopping the stream lets it return.
                // Safe to call inside the callback (recursive mutex).
                ctx.session.client().stopStream();
            }
        });

    SigintGuard guard;
    while (!g_stop && !stopping &&
           ctx.session.client().isConnected()) {
        ctx.session.client().receiveMessage(100);
    }
    if (!stopping) ctx.session.client().stopStream();
    ctx.session.client().setStreamDataCallback(nullptr);
    if (ctx.json) ctx.out << "]\n";
    if (rec) rec.close();
    ctx.err << std::format("stream: {} rows\n", rows.load());
    if (stats && rows > 1) {
        const double secs = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        ctx.err << std::format(
            "stats : {:.1f} rows/s over {:.2f} s | gaps={} "
            "interval={}–{}us\n",
            rows / secs, secs, gaps,
            minGapUs == UINT64_MAX ? 0 : minGapUs, maxGapUs);
    }
    if (!record.empty())
        ctx.err << std::format("recorded to {}\n", record);
    return 0;
}

int cmdInstream(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    if (names.empty()) {
        ctx.err << "usage: instream create NAME | send ID hex:…|file | close ID\n";
        return 2;
    }
    if (names[0] == "create") {
        if (names.size() < 2) { ctx.err << "instream create NAME\n"; return 2; }
        // Resolve the entry for its schema slot + size hint.
        auto e = ctx.session.findEntry(names[1], EntryKindSel::Any);
        if (!e) { ctx.err << "unknown entry\n"; return 1; }
        const auto* node = ctx.session.client().resolveSchemaSlot(e->schemaSlot);
        uint32_t maxVal = 0;
        if (node && node->kind == SchemaKind::Scalar)
            maxVal = valueTypeSize(node->scalarType);
        auto r = ctx.session.client().createInputStream(
            e->schemaSlot, maxVal, 1);
        if (!r || !r->success) {
            ctx.err << "createInputStream failed\n"; return 1;
        }
        ctx.out << std::format("streamId={}\n", r->streamId);
        return 0;
    }
    if (names[0] == "send") {
        if (names.size() < 3) { ctx.err << "instream send ID raw:<hex>\n"; return 2; }
        uint64_t id = 0;
        std::from_chars(names[1].data(), names[1].data() + names[1].size(), id);
        // values: hex:… literals → one row of bytes
        std::vector<uint8_t> bytes;
        const std::string& hex = names[2];
        std::string_view hv = hex.rfind("raw:",0)==0||hex.rfind("hex:",0)==0
            ? std::string_view(hex).substr(4) : std::string_view(hex);
        for (size_t i = 0; i + 1 < hv.size(); i += 2) {
            unsigned b;
            if (std::from_chars(hv.data()+i, hv.data()+i+2, b, 16).ec
                    != std::errc()) { ctx.err << "bad hex\n"; return 1; }
            bytes.push_back(uint8_t(b));
        }
        auto r = ctx.session.client().inputStreamData(
            static_cast<uint32_t>(id), {bytes});
        if (!r) { ctx.err << r.error().message << '\n'; return 1; }
        ctx.out << "sent\n"; return 0;
    }
    if (names[0] == "close") {
        if (names.size() < 2) { ctx.err << "instream close ID\n"; return 2; }
        uint64_t id = 0;
        std::from_chars(names[1].data(), names[1].data()+names[1].size(), id);
        auto r = ctx.session.client().closeInputStream(static_cast<uint32_t>(id));
        if (!r) { ctx.err << r.error().message << '\n'; return 1; }
        ctx.out << std::format("closed ({})\n", r->second ? "ok" : "rejected");
        return 0;
    }
    ctx.err << "usage: instream create|send|close\n"; return 2;
}

int cmdPeer(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    if (names.size() < 2 || names[0] != "register") {
        ctx.err << "usage: peer register NAME [--id N]\n"; return 2;
    }
    // Register a minimal echo/stub function hosted by this client.
    FunctionEntry fe;
    fe.id = std::stoull(argValue(args, "--id", "9000"));
    fe.name = names[1];
    fe.description = "tether_io_client peer function";
    fe.group = "cli";
    FunctionParameter out;
    out.name = "result"; out.type = ValueType::String;
    fe.returnValue.present = true;
    fe.returnValue.name = "result";
    fe.returnValue.type = ValueType::String;
    fe.callback = [](const std::vector<FunctionArgument>& a) {
        FunctionCallResult r;
        r.success = true;
        const std::string s = std::format("echo({} args)", a.size());
        r.returnValue.assign(s.begin(), s.end());
        return r;
    };
    auto r = ctx.session.client().registerFunctions({fe});
    if (!r) { ctx.err << "register: " << r.error().message << '\n'; return 1; }
    ctx.out << std::format("registered '{}' id={} — keep this process alive\n",
                           fe.name, fe.id);
    return 0;
}

int cmdStress(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    const uint32_t n = static_cast<uint32_t>(
        std::stoul(argValue(args, "--ops", "1000")));
    const std::string what = names.empty() ? "ping" : names[0];
    SigintGuard guard;
    uint64_t ok = 0, err = 0;
    uint64_t minUs = UINT64_MAX, maxUs = 0, sumUs = 0;
    for (uint32_t i = 0; i < n && !g_stop; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        bool good = false;
        if (what == "ping") good = ctx.session.client().ping(i).has_value();
        else if (what == "catalog") {
            auto r = ctx.session.client().listSignals(0, 1000);
            good = r.has_value();
        } else good = ctx.session.client().ping(i).has_value();
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        ++(good ? ok : err);
        sumUs += us;
        minUs = std::min(minUs, static_cast<uint64_t>(us));
        maxUs = std::max(maxUs, static_cast<uint64_t>(us));
    }
    ctx.out << std::format(
        "stress {}: {} ok, {} err | min={}us avg={}us max={}us\n",
        what, ok, err, minUs, ok ? sumUs / (ok + err) : 0, maxUs);
    return err ? 1 : 0;
}

/// `describe NAME` — dump the schema tree for an entry/function so users
/// know what shape to pass to set/call (fields, kinds, member names).
int cmdDescribe(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "usage: describe NAME\n"; return 2; }
    const SchemaGraph& g = ctx.session.client().schemaGraph();

    std::function<void(const SchemaNode&, int)> dump =
        [&](const SchemaNode& n, int depth) {
            const std::string pad(static_cast<size_t>(depth) * 2, ' ');
            ctx.out << std::format("{}{}", pad, n.name.empty() ? "<anon>" : n.name);
            switch (n.kind) {
                case SchemaKind::Scalar: ctx.out << " : " << vtName(n.scalarType); break;
                case SchemaKind::String: ctx.out << " : string"; break;
                case SchemaKind::Bytes:  ctx.out << " : bytes"; break;
                case SchemaKind::FixedArray:    ctx.out << " : array"; break;
                case SchemaKind::DynamicArray:  ctx.out << " : array"; break;
                case SchemaKind::Map:    ctx.out << " : map"; break;
                case SchemaKind::Struct: ctx.out << " : struct"; break;
                case SchemaKind::Optional: ctx.out << " : optional"; break;
                case SchemaKind::OneOf:  ctx.out << " : oneof"; break;
                case SchemaKind::Enum:   ctx.out << " : enum"; break;
                case SchemaKind::Alias:  ctx.out << " : alias"; break;
                default: break;
            }
            ctx.out << '\n';
            for (const auto& f : n.fields) {
                ctx.out << pad << "  " << f.name << ": ";
                if (const auto* fn = g.find(f.schema.key))
                    dump(*fn, depth + 2);
                else ctx.out << "<unresolved>\n";
            }
            for (const auto& [memberKey, memberRef] : n.oneOfMembers) {
                ctx.out << pad << "  | " << memberKey << ": ";
                if (const auto* mn = g.find(memberRef.key))
                    dump(*mn, depth + 2);
                else ctx.out << "<unresolved>\n";
            }
        };

    // Try catalog entry first, then function.
    if (const auto* e = ctx.session.findEntry(names[0])) {
        if (const auto* n = ctx.session.client().resolveSchemaSlot(e->schemaSlot))
            { dump(*n, 0); return 0; }
        ctx.err << "no schema for '" << names[0] << "'\n"; return 1;
    }
    if (const auto* fn = ctx.session.findFunction(names[0])) {
        ctx.out << ctx.session.functionSignature(*fn) << "\n";
        if (fn->returnSchemaSlot < ctx.session.client().schemaManifest().size()) {
            const auto& ref = ctx.session.client()
                .schemaManifest()[fn->returnSchemaSlot].ref;
            if (const auto* n = g.find(ref.key)) {
                ctx.out << "  returns:\n";
                dump(*n, 2);
            }
        }
        return 0;
    }
    ctx.err << "unknown entry/function '" << names[1] << "'\n";
    return 1;
}

int cmdWatch(Ctx& ctx, const std::vector<std::string>& args) {
    if (auto r = ensureCatalog(ctx); !r) { ctx.err << r.error() << '\n'; return 1; }
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "watch: need at least one name\n"; return 2; }
    const uint32_t intervalMs = static_cast<uint32_t>(
        std::stoul(argValue(args, "--interval-ms", "500")));
    SigintGuard guard;
    while (!g_stop) {
        ctx.out << "\x1b[H\x1b[J";   // clear screen
        for (const auto& n : names) {
            auto e = needEntry(ctx, n);
            if (!e) { ctx.err << e.error() << '\n'; return 1; }
            const bool isParam =
                std::find_if(ctx.session.params().begin(),
                             ctx.session.params().end(),
                             [&](const ClientCatalogEntry& p) {
                                 return p.id == (*e)->id;
                             }) != ctx.session.params().end();
            auto raw = isParam ? ctx.session.client().getParam((*e)->id)
                               : ctx.session.client().getSignal((*e)->id);
            if (!raw) {
                ctx.out << std::format("{:>24} = <error: {}>\n",
                                       n, raw.error().message);
                continue;
            }
            auto v = ctx.session.decodeEntry(**e, *raw);
            ctx.out << std::format("{:>24} = {}\n",
                                   n, v ? displayValue(*v) : "<decode error>");
        }
        ctx.out.flush();
        for (uint32_t i = 0; i < intervalMs / 10 && !g_stop; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int cmdLog(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    if (names.empty() || names[0] != "tail") {
        ctx.err << "usage: log tail [--severity L] [--component C] "
                   "[--message M] [--location L]\n"; return 2;
    }
    const int severity = std::stoi(argValue(args, "--severity", "0"));
    const std::string comp = argValue(args, "--component");
    const std::string msgf = argValue(args, "--message");
    const std::string loc  = argValue(args, "--location");
    auto sub = ctx.session.client().subscribeLog(
        static_cast<uint8_t>(severity), comp, msgf, loc);
    if (!sub) { ctx.err << "subscribeLog: " << sub.error().message << '\n'; return 1; }
    if (sub->failed) {
        ctx.err << "subscribeLog rejected: " << sub->errorMessage << '\n';
        return 1;
    }
    ctx.session.client().setLogDataCallback(
        [&](const ClientLogRecord& r) {
            ctx.out << std::format("[{}] {}: {}\n",
                static_cast<int>(r.severity), r.component, r.message);
            ctx.out.flush();
            if (g_stop) ctx.session.client().unsubscribeLog(sub->subscriptionId);
        });
    SigintGuard guard;
    while (!g_stop && ctx.session.client().isConnected()) {
        ctx.session.client().receiveMessage(100);
    }
    ctx.session.client().setLogDataCallback(nullptr);
    return 0;
}

int cmdDatalog(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    if (names.empty()) { ctx.err << "usage: datalog status|configure ...\n"; return 2; }
    if (names[0] == "status") {
        // Raw request for DatalogStatus (message 0x2F → 0x30); the
        // response frame still carries its type byte — skip it.
        const std::vector<uint8_t> req{
            static_cast<uint8_t>(MessageType::DatalogStatusReq)};
        auto r = ctx.session.client().request(req, MessageType::DatalogStatusResp);
        if (!r) { ctx.err << "datalog status: " << r.error().message << '\n'; return 1; }
        BufReader br(r->data(), r->size());
        br.getU8();
        DatalogStatus st;
        if (!DatalogStatus::decode(br, st)) {
            ctx.err << "datalog status: undecodable response\n";
            return 1;
        }
        static const char* kState[] = {"idle", "recording", "paused", "error"};
        ctx.out << std::format("state={} records={} bytes={}",
                               kState[static_cast<int>(st.state)],
                               st.recordsWritten, st.bytesWritten);
        return 0;
    }
    if (names[0] == "configure") {
        const std::string name = argValue(args, "--name", "default");
        const uint32_t rate = static_cast<uint32_t>(
            std::stoul(argValue(args, "--rate-hz", "100")));
        const bool enabled = !argFlag(args, "--disable");
        std::vector<uint64_t> ids;
        if (auto idsStr = argValue(args, "--ids"); !idsStr.empty()) {
            size_t pos = 0;
            while (pos < idsStr.size()) {
                auto comma = idsStr.find(',', pos);
                uint64_t id;
                std::from_chars(idsStr.data() + pos,
                    idsStr.data() + (comma == std::string::npos
                                       ? idsStr.size() : comma), id);
                ids.push_back(id);
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        }
        auto r = ctx.session.client().configureDatalog(name, rate, enabled, ids);
        if (!r) { ctx.err << "datalog configure: " << r.error().message << '\n'; return 1; }
        ctx.out << "datalog configured\n";
        return 0;
    }
    ctx.err << "usage: datalog status|configure [--name N] [--rate-hz N] "
               "[--ids a,b,c] [--disable]\n";
    return 2;
}

int cmdThreshold(Ctx& ctx, const std::vector<std::string>& args) {
    const auto names = positional(args);
    if (names.size() < 2 || names[0] != "set") {
        ctx.err << "usage: threshold set NAME [--rule type=abs id=N val=X] ...\n";
        return 2;
    }
    ThresholdConfig cfg;
    cfg.name = names[1];
    cfg.isWhitelist = !argFlag(args, "--blacklist");
    for (const auto& a : args) {
        if (a.rfind("--rule=", 0) == 0) {
            // --rule=type=abs,id=5,val=1.5
            ThresholdRule rule{};
            std::string_view kv = a.substr(7);
            size_t p = 0;
            while (p < kv.size()) {
                auto comma = kv.find(',', p);
                auto part = kv.substr(p, comma == std::string_view::npos
                                           ? kv.size() : comma - p);
                auto eq = part.find('=');
                if (eq != std::string_view::npos) {
                    auto k = part.substr(0, eq), v = part.substr(eq + 1);
                    if (k == "type") {
                        if (v == "abs") rule.type = ThresholdType::Absolute;
                        else if (v == "rel") rule.type = ThresholdType::Relative;
                        else if (v == "none") rule.type = ThresholdType::None;
                        else rule.type = ThresholdType::Custom;
                    } else if (k == "id") {
                        std::from_chars(v.data(), v.data() + v.size(), rule.entryId);
                    } else if (k == "val") {
                        rule.threshold = std::stod(std::string(v));
                    }
                }
                if (comma == std::string_view::npos) break;
                p = comma + 1;
            }
            cfg.rules.push_back(std::move(rule));
        }
    }
    // Encode via ThresholdConfig::encode into a buffer.
    std::vector<uint8_t> buf(4096);
    BufWriter w(buf.data(), buf.size());
    cfg.encode(w);
    buf.resize(w.pos);
    const auto r = ctx.session.client().configureThreshold(
        cfg.name, cfg.isWhitelist, buf);
    if (!r) { ctx.err << "configureThreshold: " << r.error().message << '\n'; return 1; }
    ctx.out << std::format("threshold '{}' configured ({} rules)\n",
                           cfg.name, cfg.rules.size());
    return 0;
}

} // namespace

// ---------------------------------------------------------------------------

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    char quote = 0;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quote) {
            if (c == quote) { quote = 0; continue; }
            if (c == '\\' && i + 1 < line.size()) { cur += line[++i]; continue; }
            cur += c;
            continue;
        }
        if (c == '"' || c == '\'') { quote = c; continue; }
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (!cur.empty()) { out.push_back(cur); cur.clear(); }
            continue;
        }
        if (c == '\\' && i + 1 < line.size()) { cur += line[++i]; continue; }
        cur += c;
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

int dispatch(Ctx& ctx, const std::vector<std::string>& args) {
    if (args.empty()) return 0;
    const auto& cmd = args[0];
    if (cmd == "describe") return cmdDescribe(ctx, args);
    if (cmd == "ping")     return cmdPing(ctx, args);
    if (cmd == "info")     return cmdInfo(ctx, args);
    if (cmd == "list")     return cmdList(ctx, args);
    if (cmd == "get")      return cmdGet(ctx, args);
    if (cmd == "set")      return cmdSet(ctx, args);
    if (cmd == "snapshot") return cmdSnapshot(ctx, args);
    if (cmd == "metadata") return cmdMetadata(ctx, args);
    if (cmd == "call")     return cmdCall(ctx, args);
    if (cmd == "stream")   return cmdStream(ctx, args);
    if (cmd == "watch")    return cmdWatch(ctx, args);
    if (cmd == "log")      return cmdLog(ctx, args);
    if (cmd == "datalog")  return cmdDatalog(ctx, args);
    if (cmd == "threshold") return cmdThreshold(ctx, args);
    if (cmd == "instream") return cmdInstream(ctx, args);
    if (cmd == "peer")     return cmdPeer(ctx, args);
    if (cmd == "stress")   return cmdStress(ctx, args);
    ctx.err << std::format("unknown command '{}' (see --help)\n", cmd);
    return 2;
}

/// Wrap a command invocation with --verbose timing + stale-catalog notice.
int dispatchTimed(Ctx& ctx, const std::vector<std::string>& args) {
    if (ctx.session.catalogStale())
        ctx.err << "note: server pushed CatalogChanged — catalog refreshed\n";
    if (ctx.session.schemaStale())
        ctx.err << "note: server pushed SchemaUpdate — schema epoch changed\n";
    const auto t0 = std::chrono::steady_clock::now();
    const int rc = dispatch(ctx, args);
    if (ctx.verbose) {
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        ctx.err << std::format("[verbose] '{}' → rc={} in {}us\n",
                               args.empty() ? "" : args[0], rc, us);
    }
    return rc;
}

} // namespace tether::io::client
