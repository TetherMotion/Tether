/**
 * @file client_session.hpp
 * @brief Transport factory + cached-catalog session wrapper for the
 *        tether_io_client tool.
 *
 * URL schemes:
 *   ws://host[:port][/path][?token=..&role=..]   (default port 80)
 *   tcp://host[:port]                            (default port 4000, SLIP)
 *   serial:///dev/ttyX[?baud=N]                  (SLIP over serial)
 *   host[:port]                                  (bare = tcp://)
 */
#pragma once

#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Transport.hpp"

#include <atomic>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace tether::io::client {

struct ConnectSpec {
    std::string url;
    std::string token;          ///< ws only: sent as Bearer + ?token=
    std::string role;           ///< ws only: appended as ?role= (dev mode)
    uint32_t connectTimeoutMs = 5000;
    uint32_t requestTimeoutMs = 5000;
};

/// Parse a URL into a connected, message-oriented ITransport.
/// Errors carry a human-readable reason.
std::expected<std::unique_ptr<ITransport>, std::string>
makeIoTransport(const ConnectSpec& spec);

enum class EntryKindSel { Any, Param, Signal, Function };

/// A connected client + lazily loaded catalog with name resolution.
class IoSession {
public:
    explicit IoSession(std::unique_ptr<ITransport> transport,
                       uint32_t timeoutMs = 5000)
        : client_(std::move(transport), timeoutMs) {
        // §5 push notifications → staleness flags, read-and-cleared by
        // the dispatch layer (one-shot notice per change).
        client_.setCatalogChangedCallback(
            [this] { catalogStale_ = true; });
        client_.setSchemaUpdateCallback(
            [this] { schemaStale_ = true; });
    }
    ~IoSession() { stopPump(); }

    /// True once after a server CatalogChanged/SchemaUpdate push; clears
    /// on read so callers can surface a single notice.
    bool catalogStale() { return catalogStale_.exchange(false); }
    bool schemaStale()  { return schemaStale_.exchange(false); }

    TetherIOClient& client() { return client_; }
    const TetherIOClient& client() const { return client_; }

    bool isConnected() const { return client_.isConnected(); }

    /// Background pump thread: loops `client().receiveMessage(pollMs)`
    /// so StreamData/LogData/CatalogChanged callbacks fire without an
    /// in-flight request. Safe — the client's internal recursive_mutex
    /// serializes the pump against `request()` calls on other threads.
    /// Used by the TUI so stream/log views update live.
    void startPump(uint32_t pollMs = 20);
    void stopPump();
    bool pumpRunning() const { return pumpThread_.joinable(); }

    /// (Re)fetch all catalog entries.
    std::expected<void, std::string> refreshCatalog();

    const std::vector<ClientCatalogEntry>& params() const { return params_; }
    const std::vector<ClientCatalogEntry>& signals() const { return signals_; }
    const std::vector<ClientFunctionEntry>& functions() const { return functions_; }

    /// Resolve "name", "group.name", numeric id ("123", "0x1F"), or
    /// "p:name"/"s:name"/"f:name" to a catalog entry / function.
    /// Ambiguous or missing names produce an error naming the candidates.
    const ClientCatalogEntry* findEntry(const std::string& name,
                                        EntryKindSel kind = EntryKindSel::Any) const;
    const ClientFunctionEntry* findFunction(const std::string& name) const;

    /// Decode raw value bytes for a catalog entry.
    std::expected<SchemaValue, ClientError>
    decodeEntry(const ClientCatalogEntry& e, const std::vector<uint8_t>& raw) const {
        return client_.decodeValue(e.schemaSlot, raw.data(), raw.size());
    }

    /// Decode raw bytes for a stream layout entry.
    std::expected<SchemaValue, ClientError>
    decodeStreamValue(const ClientStreamLayoutEntry& e,
                      const std::vector<uint8_t>& raw) const {
        return client_.decodeValue(e.schemaSlot, raw.data(), raw.size());
    }

    /// Pretty type name for a catalog entry (schema kind/scalar type).
    std::string typeNameOf(const ClientCatalogEntry& e) const;
    std::string functionSignature(const ClientFunctionEntry& f) const;

private:
    TetherIOClient client_;
    std::vector<ClientCatalogEntry> params_, signals_;
    std::vector<ClientFunctionEntry> functions_;
    std::thread pumpThread_;
    std::atomic<bool> pumpStop_{false};
    std::atomic<bool> catalogStale_{false};
    std::atomic<bool> schemaStale_{false};
};

/// Render a decoded value for table output (compact text).
inline std::string displayValue(const SchemaValue& v) {
    return formatSchemaValueText(v, /*maxElements*/16, /*maxString*/64);
}

/// glob match (* ? [..]) — no regex dependency needed for CLI filtering.
bool globMatch(std::string_view pattern, std::string_view text);

/// Human-readable name for a ValueType.
const char* vtName(ValueType t);

/// Filter a catalog by optional glob, substring, and/or ECMAScript regex.
std::vector<const ClientCatalogEntry*>
filterEntries(const std::vector<ClientCatalogEntry>& entries,
              const std::string& glob, const std::string& substr,
              const std::string& regex = {});

/// Match a name against a regex pattern (empty = match all).
bool regexMatch(const std::string& pattern, std::string_view text);

/// Run the full-screen ncurses TUI (P4). Requires a connected session.
int runTui(IoSession& session);

} // namespace tether::io::client
