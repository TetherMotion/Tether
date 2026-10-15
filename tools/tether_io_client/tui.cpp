/**
 * @file tui.cpp
 * @brief Interactive TUI for tether_io_client: catalog explorer, watch
 *        panel, live stream table, function caller, log tail — built on
 *        tether_terminal_ui (PanelScreen + TableModel + Overlay).
 *
 * Single-threaded: the client is pumped with nonblocking
 * `receiveMessage(0)` polls each frame (~50 ms) — stream/log records are
 * delivered through the client's callbacks into the UI models.
 */
#include "client_session.hpp"
#include "commands.hpp"

#include "tether/terminal_ui/PanelScreen.hpp"
#include "tether/terminal_ui/TablePanel.hpp"
#include "tether/terminal_ui/StyledText.hpp"
#include "tether/ui/TableModel.hpp"
#include "tether/ui/Overlay.hpp"

#include <ncurses.h>
#undef OK
#undef ERR

#include <algorithm>
#include <atomic>
#include <deque>
#include <format>
#include <mutex>
#include <sstream>

using namespace tether::io;
using namespace tether::io::client;
namespace UI = Tether::UI;
namespace TUI = Tether::TUI;

namespace {

// ---------------------------------------------------------------------------
// Overlay showing text lines (value details, metadata, command output).
// ---------------------------------------------------------------------------

class LinesOverlay : public UI::Overlay {
public:
    LinesOverlay(std::string title, std::vector<std::string> lines)
        : title_(std::move(title)), lines_(std::move(lines)) {}
    std::string title() const override { return title_; }
    std::vector<std::string> lines() override { return lines_; }
private:
    std::string title_;
    std::vector<std::string> lines_;
};

// ---------------------------------------------------------------------------
// `:` command prompt overlay — runs dispatch() with output captured.
// ---------------------------------------------------------------------------

class PromptOverlay : public UI::Overlay {
public:
    PromptOverlay(std::string initial, Ctx* ctx,
                  std::function<void(const std::string&)> onResult)
        : input_(std::move(initial)), ctx_(ctx),
          onResult_(std::move(onResult)) {}

    std::string title() const override { return ":"; }
    std::vector<std::string> lines() override {
        return {":" + input_ + "_"};
    }
    bool handleKey(int key) override {
        if (key == 27) { close_ = true; return true; }
        if (key == '\n' || key == KEY_ENTER) {
            const auto args = tokenize(input_);
            std::ostringstream out, err;
            Ctx sub{ctx_->session, ctx_->json, ctx_->verbose, out, err};
            dispatch(sub, args);
            std::string result = out.str() + err.str();
            if (onResult_) onResult_(result.empty() ? "(ok)" : result);
            close_ = true;
            return true;
        }
        if (key == KEY_BACKSPACE || key == 127 || key == 8) {
            if (!input_.empty()) input_.pop_back();
            return true;
        }
        if (key >= 32 && key < 127) { input_ += static_cast<char>(key); return true; }
        return false;
    }
    bool wantsClose() const override { return close_; }

private:
    std::string input_;
    Ctx* ctx_;
    std::function<void(const std::string&)> onResult_;
    bool close_ = false;
};

// ---------------------------------------------------------------------------
// The app
// ---------------------------------------------------------------------------

struct App {
    IoSession& session;
    explicit App(IoSession& s) : session(s) {}

    // -- models --
    UI::TableModel catalog_;
    UI::TableModel funcs_;
    UI::TableModel watch_;
    UI::TableModel stream_;
    std::deque<std::vector<std::string>> streamRows_;
    static constexpr size_t kMaxStreamRows = 2000;

    // watch set + stream set (entry ids)
    std::vector<const ClientCatalogEntry*> watchList_;
    std::vector<uint64_t> streamIds_;
    std::vector<ClientStreamLayoutEntry> streamLayout_;
    std::mutex streamMutex_;
    bool streaming_ = false;
    uint64_t streamCount_ = 0;
    uint64_t rowCount_ = 0;
    std::string note_;
    double noteUntil_ = 0;

    TUI::PanelScreen* screen_ = nullptr;

    // ---------- helpers ----------

    void flash(const std::string& msg) { if (screen_) screen_->flash(msg); }
    void log(const std::string& line) {
        if (screen_) screen_->log().addLine(line);
    }
    void logBlock(const std::string& text) {
        std::istringstream is(text);
        std::string l;
        while (std::getline(is, l)) log(l);
    }

    void rebuildCatalog() {
        std::vector<UI::Row> rows;
        auto add = [&](const ClientCatalogEntry& e, const char* k) {
            rows.push_back({{std::to_string(e.id),
                             e.group.empty() ? e.name : e.group + "." + e.name,
                             k, session.typeNameOf(e), e.description},
                            {}, static_cast<int>(e.id)});
        };
        for (const auto& e : session.params())   add(e, "param");
        for (const auto& e : session.signals())  add(e, "signal");
        catalog_.setRows(std::move(rows));
    }

    void rebuildFuncs() {
        std::vector<UI::Row> rows;
        for (const auto& f : session.functions()) {
            rows.push_back({{std::to_string(f.id),
                             f.group.empty() ? f.name : f.group + "." + f.name,
                             session.functionSignature(f)},
                            {}, static_cast<int>(f.id)});
        }
        funcs_.setRows(std::move(rows));
    }

    const ClientCatalogEntry* catalogEntry(int tag) const {
        for (const auto& e : session.params())  if ((int)e.id == tag) return &e;
        for (const auto& e : session.signals()) if ((int)e.id == tag) return &e;
        return nullptr;
    }

    bool isParam(const ClientCatalogEntry& e) const {
        for (const auto& p : session.params()) if (p.id == e.id) return true;
        return false;
    }

    std::string readEntryText(const ClientCatalogEntry& e) {
        auto raw = isParam(e) ? session.client().getParam(e.id)
                              : session.client().getSignal(e.id);
        if (!raw) return "error: " + raw.error().message;
        auto v = session.decodeEntry(e, *raw);
        if (!v) return "decode: " + v.error().message;
        return displayValue(*v);
    }

    void pushValueOverlay(const ClientCatalogEntry& e) {
        std::string val = readEntryText(e);
        std::vector<std::string> lines;
        lines.push_back(std::format("{} (id {} slot {})",
                                    e.name, e.id, e.schemaSlot));
        lines.push_back(std::format("type : {}", session.typeNameOf(e)));
        lines.push_back(std::format("value: {}", val));
        if (auto raw = isParam(e) ? session.client().getParam(e.id)
                                  : session.client().getSignal(e.id); raw) {
            auto v = session.decodeEntry(e, *raw);
            if (v) {
                lines.push_back(std::format("json : {}",
                    schemaValueToJson(*v, &session.client().schemaGraph())));
            }
        }
        screen_->pushOverlay(
            std::make_unique<LinesOverlay>(e.name, std::move(lines)));
    }

    void pushMetadataOverlay(const ClientCatalogEntry& e) {
        auto m = session.client().getMetadata(e.id);
        std::vector<std::string> lines;
        if (!m) {
            lines.push_back("error: " + m.error().message);
        } else {
            for (const auto& kv : *m)
                lines.push_back(std::format("{} = {}", kv.key, kv.value));
            if (lines.empty()) lines.push_back("(no metadata)");
        }
        screen_->pushOverlay(
            std::make_unique<LinesOverlay>("meta " + e.name, std::move(lines)));
    }

    void openPrompt(std::string initial, Ctx* ctx) {
        screen_->pushOverlay(std::make_unique<PromptOverlay>(
            std::move(initial), ctx,
            [this](const std::string& result) {
                logBlock(result);
                flash("done");
            }));
    }

    void toggleWatch(const ClientCatalogEntry& e) {
        auto it = std::find(watchList_.begin(), watchList_.end(), &e);
        if (it != watchList_.end()) {
            watchList_.erase(it);
            flash("unwatch " + e.name);
        } else {
            watchList_.push_back(&e);
            flash("watch " + e.name);
        }
    }

    void toggleStreamEntry(const ClientCatalogEntry& e) {
        auto it = std::find(streamIds_.begin(), streamIds_.end(), e.id);
        if (it != streamIds_.end()) {
            streamIds_.erase(it);
            flash(std::format("-stream {} ({})", e.name, streamIds_.size()));
        } else {
            streamIds_.push_back(e.id);
            flash(std::format("+stream {} ({})", e.name, streamIds_.size()));
        }
    }

    void startStream() {
        if (streamIds_.empty()) { flash("no stream entries — toggle with s"); return; }
        auto r = session.client().configureStream(streamIds_, 20000, 1, 0, 0, 0);
        if (!r) { flash("stream: " + r.error().message); return; }
        session.client().setStreamDataCallback(
            [this](const ClientStreamRow& row) { onStreamRow(row); });
        auto st = session.client().startStream();
        if (!st) { flash("startStream: " + st.error().message); return; }
        streamLayout_ = r->layout;
        streaming_ = true;
        streamRows_.clear();
        streamCount_ = 0;
        flash(std::format("streaming {} entries", streamIds_.size()));
    }

    void stopStream() {
        session.client().stopStream();
        session.client().setStreamDataCallback(nullptr);
        streaming_ = false;
        flash(std::format("stopped — {} rows", streamCount_));
    }

    void onStreamRow(const ClientStreamRow& row) {
        ++streamCount_;
        std::vector<std::string> cells;
        cells.push_back(std::to_string(row.timestampUs));
        for (size_t i = 0; i < row.values.size() && i < streamLayout_.size(); ++i) {
            auto v = session.client().decodeValue(
                streamLayout_[i].schemaSlot, row.values[i].data(),
                row.values[i].size());
            cells.push_back(v ? displayValue(*v) : "<err>");
        }
        std::lock_guard<std::mutex> lk(streamMutex_);
        streamRows_.push_back(std::move(cells));
        if (streamRows_.size() > kMaxStreamRows) streamRows_.pop_front();
    }

    void rebuildStreamModel() {
        std::lock_guard<std::mutex> lk(streamMutex_);
        std::vector<UI::Row> rows;
        for (auto it = streamRows_.rbegin(); it != streamRows_.rend(); ++it) {
            rows.push_back({*it, {}, (int)rowCount_++});
        }
        stream_.setRows(std::move(rows));
    }

    void refreshWatch() {
        std::vector<UI::Row> rows;
        for (const auto* e : watchList_) {
            rows.push_back({{e->name, session.typeNameOf(*e), readEntryText(*e)},
                            {}, (int)e->id});
        }
        watch_.setRows(std::move(rows));
    }

    // ---------- key handlers ----------

    bool catalogKey(int key, Ctx* ctx) {
        switch (key) {
            case '\n': case KEY_ENTER: {
                const auto* r = catalog_.selected();
                if (r) if (const auto* e = catalogEntry(r->tag); e)
                    pushValueOverlay(*e);
                return true;
            }
            case '/': {
                openPrompt("", ctx);   // reuse prompt for search… instead:
                return true;
            }
            case 'e': {
                const auto* r = catalog_.selected();
                if (r) if (const auto* e = catalogEntry(r->tag); e)
                    openPrompt("set " + e->name + " ", ctx);
                return true;
            }
            case 'm': {
                const auto* r = catalog_.selected();
                if (r) if (const auto* e = catalogEntry(r->tag); e)
                    pushMetadataOverlay(*e);
                return true;
            }
            case 'r': {
                if (auto res = session.refreshCatalog(); !res) {
                    flash("catalog: " + res.error());
                } else {
                    rebuildCatalog();
                    rebuildFuncs();
                    flash(std::format("catalog: {}p/{}s/{}f",
                                      session.params().size(),
                                      session.signals().size(),
                                      session.functions().size()));
                }
                return true;
            }
            case 'w': {
                const auto* r = catalog_.selected();
                if (r) if (const auto* e = catalogEntry(r->tag); e)
                    toggleWatch(*e);
                return true;
            }
            case 's': {
                const auto* r = catalog_.selected();
                if (r) if (const auto* e = catalogEntry(r->tag); e)
                    toggleStreamEntry(*e);
                return true;
            }
        }
        return false;
    }

    bool funcsKey(int key, Ctx* ctx) {
        if (key == '\n' || key == KEY_ENTER) {
            const auto* r = funcs_.selected();
            if (!r) return true;
            const ClientFunctionEntry* fn = nullptr;
            for (const auto& f : session.functions())
                if ((int)f.id == r->tag) { fn = &f; break; }
            if (fn) openPrompt("call " + fn->name + " ", ctx);
            return true;
        }
        return false;
    }
};

} // namespace

int tether::io::client::runTui(IoSession& session) {
    App app(session);
    std::ostringstream sink;
    Ctx ctx{session, false, false, sink, sink};

    if (auto r = session.refreshCatalog(); !r) {
        std::fprintf(stderr, "catalog: %s\n", r.error().c_str());
        return 1;
    }
    app.rebuildCatalog();
    app.rebuildFuncs();
    app.catalog_.setColumns({"ID", "NAME", "KIND", "TYPE", "DESCRIPTION"});
    app.funcs_.setColumns({"ID", "NAME", "SIGNATURE"});
    app.watch_.setColumns({"NAME", "TYPE", "VALUE"});
    app.stream_.setColumns({"T_US", "DATA…"});

    TUI::PanelScreenHooks hooks;
    hooks.showLogView = true;    // built-in scrollback for dispatch output
    hooks.keyHints = " :cmd  ?:keys  1-4:tabs";

    TUI::PanelView catalog;
    catalog.name = "Catalog";
    catalog.renderBody = [&](int top, int bottom) {
        TUI::TablePanel::render(app.catalog_, top, bottom, 0,
                                app.screen_->session().cols());
    };
    catalog.keyHints = " ↑↓:nav  Enter:value  e:set  m:meta  w:watch  s:stream  r:refresh";
    catalog.onKey = [&](int k) {
        switch (k) {
            case KEY_UP:    app.catalog_.move(-1); return true;
            case KEY_DOWN:  app.catalog_.move(1);  return true;
            case KEY_PPAGE: app.catalog_.pageMove(-1, 20); return true;
            case KEY_NPAGE: app.catalog_.pageMove(1, 20);  return true;
            case KEY_HOME:  app.catalog_.selectFirst(); return true;
            case KEY_END:   app.catalog_.selectLast();  return true;
        }
        return app.catalogKey(k, &ctx);
    };

    TUI::PanelView watch;
    watch.name = "Watch";
    watch.renderBody = [&](int top, int bottom) {
        TUI::TablePanel::render(app.watch_, top, bottom, 0,
                                app.screen_->session().cols());
    };
    watch.keyHints = " w:toggle(catalog)  500ms poll";

    TUI::PanelView stream;
    stream.name = "Stream";
    stream.renderBody = [&](int top, int bottom) {
        TUI::TablePanel::render(app.stream_, top, bottom, 0,
                                app.screen_->session().cols());
    };
    stream.keyHints = " S:start/stop  s:toggle(catalog)";
    stream.onKey = [&](int k) {
        if (k == 'S') {
            if (app.streaming_) app.stopStream(); else app.startStream();
            return true;
        }
        return false;
    };

    TUI::PanelView funcs;
    funcs.name = "Functions";
    funcs.renderBody = [&](int top, int bottom) {
        TUI::TablePanel::render(app.funcs_, top, bottom, 0,
                                app.screen_->session().cols());
    };
    funcs.keyHints = " ↑↓:nav  Enter:call…";
    funcs.onKey = [&](int k) {
        switch (k) {
            case KEY_UP:    app.funcs_.move(-1); return true;
            case KEY_DOWN:  app.funcs_.move(1);  return true;
            case KEY_PPAGE: app.funcs_.pageMove(-1, 20); return true;
            case KEY_NPAGE: app.funcs_.pageMove(1, 20);  return true;
        }
        return app.funcsKey(k, &ctx);
    };

    hooks.views = {catalog, watch, stream, funcs};

    hooks.onTick = [&] {
        // watch polling (~500 ms)
        static int tick = 0;
        if (++tick % 10 == 0 && !app.watchList_.empty()) app.refreshWatch();
        if (app.streaming_) app.rebuildStreamModel();
        if (!session.client().isConnected() && app.screen_)
            app.screen_->flash("DISCONNECTED");
    };

    hooks.onKey = [&](int k) {
        if (k == ':') {
            app.openPrompt("", &ctx);
            return true;
        }
        return false;
    };

    TUI::PanelScreen screen(
        std::format("tether-io  epoch={}", session.client().schemaEpoch()),
        std::move(hooks));
    app.screen_ = &screen;

    app.log(std::format("catalog: {} params, {} signals, {} functions",
                        session.params().size(), session.signals().size(),
                        session.functions().size()));

    // P3 pump thread: StreamData/LogData dispatch in the background so
    // the stream tab and log pane update without blocking the UI.
    session.client().setLogDataCallback(
        [&app](const ClientLogRecord& r) {
            static const char* sev[] = {"", "ERR", "WRN", "INF", "DBG", "VB"};
            app.log(std::format("[{}] {}: {}", sev[std::min<int>(r.severity, 5)],
                                r.component, r.message));
        });
    auto sub = session.client().subscribeLog(/*minSeverity*/1 /*Info*/);
    if (!sub) app.log("subscribeLog: " + sub.error().message);
    session.startPump(20);

    std::atomic<bool> cancel{false};
    screen.run(cancel);

    session.stopPump();
    if (sub) session.client().unsubscribeLog(sub->subscriptionId);
    session.client().setLogDataCallback(nullptr);
    return 0;
}
