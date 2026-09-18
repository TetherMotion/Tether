#pragma once

/**
 * @file Panels.hpp
 * @brief Standard panel *models* — reusable building blocks for system
 *        inspection TUIs (and later a web interface).
 *
 * Everything here is UI-toolkit-agnostic: the models produce TableModel
 * rows; the terminal_ui widgets (EntityBrowserPanel) or a future HTTP
 * renderer consume them unchanged.
 *
 * Provided models:
 *  - SlaveEntity / ISlaveEntityProvider — "what entities exist and in
 *    which state" (EtherCAT slaves, but any entity list works).
 *  - NamedRegister / IRegisterReader — a named CoE register list plus an
 *    injected read transport (SDO, register read, simulation, ...).
 *  - EntityBrowser — master/detail model: entity table on top, named
 *    registers of the selected entity below.
 */

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tether/ui/TableModel.hpp"

namespace Tether {
namespace UI {

// ---------------------------------------------------------------------------
// Entity list
// ---------------------------------------------------------------------------

struct SlaveEntity {
    uint16_t index = 0;          ///< position on the bus / app index
    std::string name;            ///< human-readable name (may be empty)
    std::string state;           ///< e.g. "OP", "SAFE-OP", "DATA"
    uint16_t status_code = 0;    ///< AL status / error code, 0 = none
    bool error = false;          ///< error flag set
    bool suspended = false;      ///< supervisor-suspended / offline
    std::string detail;          ///< free-form extra info column
};

/// Application-supplied snapshot source for the entity table.
struct ISlaveEntityProvider {
    virtual ~ISlaveEntityProvider() = default;
    virtual std::vector<SlaveEntity> entities() = 0;
};

// ---------------------------------------------------------------------------
// Named register list + reader
// ---------------------------------------------------------------------------

struct NamedRegister {
    uint16_t index = 0;
    uint8_t subindex = 0;
    std::string name;
    uint8_t bits = 0;            ///< size hint; 0 = unknown (read raw)
};

struct RegisterResult {
    bool ok = false;
    std::string value;           ///< formatted value, e.g. "0x00000008"
    std::string error;           ///< failure text when !ok
};

/// Application-injected register read transport (SDO under a mutex,
/// ESC register read, cache lookup — the model doesn't care).
struct IRegisterReader {
    virtual ~IRegisterReader() = default;
    virtual RegisterResult read(uint16_t entity_index,
                                const NamedRegister& reg) = 0;
};

/// Which register list applies to which entity.  Default: fixed list.
using RegisterSelector =
    std::function<std::vector<NamedRegister>(const SlaveEntity&)>;

/// Common CoE identity/communication registers — a sensible default for
/// any EtherCAT slave; apps typically append device-specific entries.
inline std::vector<NamedRegister> standardRegisters() {
    return {
        {0x1000, 0, "Device type",        32},
        {0x1008, 0, "Device name",         0},
        {0x1009, 0, "Hardware version",    0},
        {0x100A, 0, "Software version",    0},
        {0x1018, 1, "Vendor ID",          32},
        {0x1018, 2, "Product code",       32},
        {0x1018, 3, "Revision",           32},
        {0x1018, 4, "Serial number",      32},
        {0x1C00, 1, "SM0 type (mailbox)",  8},
        {0x1C00, 2, "SM1 type (mailbox)",  8},
        {0x1C00, 3, "SM2 type (process)",  8},
        {0x1C00, 4, "SM3 type (process)",  8},
    };
}

// ---------------------------------------------------------------------------
// EntityBrowser — master/detail model
// ---------------------------------------------------------------------------

class EntityBrowser {
public:
    enum class Focus { Entities, Registers };

    void setProvider(ISlaveEntityProvider* p) { provider_ = p; }
    void setRegisterReader(IRegisterReader* r) { reader_ = r; }
    void setRegisters(std::vector<NamedRegister> regs) {
        regs_ = std::move(regs);
        selector_ = {};
    }
    void setRegisterSelector(RegisterSelector sel) { selector_ = std::move(sel); }

    TableModel& entities() { return entities_; }
    TableModel& registers() { return registers_; }
    Focus focus() const { return focus_; }
    void setFocus(Focus f) { focus_ = f; }

    bool autoRead() const { return auto_read_; }
    void setAutoRead(bool on) { auto_read_ = on; }

    const SlaveEntity* selectedEntity() const {
        const int tag = const_cast<TableModel&>(entities_).selectedTag();
        if (tag < 0) return nullptr;
        for (const auto& e : entities_list_)
            if (e.index == static_cast<uint16_t>(tag)) return &e;
        return nullptr;
    }

    /// Re-poll the provider into the entity table.  Call from the render
    /// tick — the provider decides how expensive a poll is.
    void refresh() {
        if (!provider_) return;
        entities_list_ = provider_->entities();
        std::vector<Row> rows;
        rows.reserve(entities_list_.size());
        for (const auto& e : entities_list_) {
            rows.push_back({
                .cells = {
                    std::to_string(e.index),
                    e.name,
                    e.state,
                    e.status_code ? hex(e.status_code) : "-",
                    e.error ? "ERR" : "-",
                    e.suspended ? "susp" : "-",
                    e.detail,
                },
                .fields = {{
                    {"idx", std::to_string(e.index)},
                    {"name", e.name},
                    {"state", e.state},
                    {"code", std::to_string(e.status_code)},
                    {"err", e.error ? "1" : "0"},
                    {"suspended", e.suspended ? "1" : "0"},
                }},
                .tag = static_cast<int>(e.index),
                .color = e.error ? RowErr
                       : e.suspended ? RowMuted
                       : e.state == "OP" || e.state == "DATA" ? RowOk
                       : RowWarn,
            });
        }
        entities_.setRows(std::move(rows));
    }

    /// Re-read the named registers of the currently selected entity via
    /// the injected reader.  Only call on user request or a slow poll —
    /// the transport may be a live SDO channel.
    void refreshRegisters() {
        registers_.setRows({});
        reg_status_.clear();
        const SlaveEntity* e = selectedEntity();
        if (!e) { reg_status_ = "no entity selected"; return; }
        if (!reader_) { reg_status_ = "no register reader"; return; }
        reg_slave_ = e->index;
        const auto regs = selector_ ? selector_(*e) : regs_;
        std::vector<Row> rows;
        rows.reserve(regs.size());
        for (const auto& nr : regs) {
            const RegisterResult r = reader_->read(e->index, nr);
            rows.push_back({
                .cells = {
                    "0x" + hex4(nr.index) + ":" + std::to_string(nr.subindex),
                    nr.name,
                    r.ok ? r.value : "-",
                    r.ok ? "ok" : r.error,
                },
                .fields = {{
                    {"index", "0x" + hex4(nr.index)},
                    {"sub", std::to_string(nr.subindex)},
                    {"name", nr.name},
                    {"value", r.value},
                    {"ok", r.ok ? "1" : "0"},
                }},
                .color = r.ok ? RowNone : RowErr,
            });
        }
        registers_.setRows(std::move(rows));
        reg_status_ = std::to_string(regs.size()) + " register(s) @ entity " +
                      std::to_string(e->index);
    }

    /// True when the selected entity changed since the last register read.
    bool registersStale() const {
        const SlaveEntity* e = selectedEntity();
        return e && reg_slave_ >= 0 &&
               e->index != static_cast<uint16_t>(reg_slave_);
    }

    /// True when registers were never read for the selected entity.
    bool registersUnread() const {
        const SlaveEntity* e = selectedEntity();
        return e && reg_slave_ != static_cast<int>(e->index);
    }

    const std::string& registerStatus() const { return reg_status_; }
    int registerSlave() const { return reg_slave_; }

private:
    static std::string hex(uint16_t v) {
        char b[8];
        snprintf(b, sizeof(b), "0x%04X", v);
        return b;
    }
    static std::string hex4(uint16_t v) {
        char b[8];
        snprintf(b, sizeof(b), "%04X", v);
        return b;
    }

    ISlaveEntityProvider* provider_ = nullptr;
    IRegisterReader* reader_ = nullptr;
    std::vector<NamedRegister> regs_;
    RegisterSelector selector_;
    std::vector<SlaveEntity> entities_list_;
    TableModel entities_;
    TableModel registers_;
    Focus focus_ = Focus::Entities;
    bool auto_read_ = false;
    int reg_slave_ = -1;
    std::string reg_status_;
};

} // namespace UI
} // namespace Tether
