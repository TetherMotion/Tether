// pdo_mapping_probe — generic EtherCAT PDO-mapping probe.
//
// Connects to one slave and tests how its PDO mapping objects (0x1600-0x1A0F,
// 0x1C12/0x1C13) behave under arbitrary configurations.  Each probe case:
//
//   1. forces the slave to INIT, configures the mailbox, enters PRE-OP
//   2. writes a set of PDO mapping objects (Rx/Tx, arbitrary entries)
//   3. assigns them to SM2/SM3 and configures the sync managers
//   4. requests SAFE-OP with a short timeout and records the outcome:
//        - reached SAFE-OP                     → PASS
//        - AL status code 0x0024               → InvalidInputMapping
//        - any other AL status code / timeout  → reported verbatim
//
// A-B-A integrity check: after every FAILED case the probe re-runs the last
// known-good case and requires it to still reach SAFE-OP.  If the
// known-good case also fails, the device is wedged (or the fault is
// unrelated to the mapping) and the probe stops, so a single probe result
// is only ever trusted when the baseline still passes.
//
// Built-in case generators (systematic sweep):
//   --sweep-count  <pdo>:<obj>:<sub>:<bits>[:<max>]   entry-count capacity
//   --sweep-size   <pdo>:<obj>:<sub>:<bits>[:<list>]  per-entry bit sizes
//   --sweep-total  <pdo>:<obj>:<sub>:<bits>:<max>     total-size ceiling
//   --bad          run the built-in negative battery
//
// Usage:
//   runec ./pdo_mapping_probe -i enp3s0 -s 0 \
//       --case one=1601:rx:7010:0:32,7020:0:32
//       --case two=1A01:tx:6010:0:32,6020:0:32
//       --sweep-count 1A02:tx:4101:0:32:40
//       --bad

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <vector>

#include <argparse/argparse.hpp>

#include "common/EtherCATHostSetup.hpp"
#include "common/ExampleHelpers.hpp"
#include "logging/Logger.hpp"
#include "tether/ethercat/ALResetController.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/profiles/cia402/CiA402StateUtils.hpp"
#include "tether/utils/SignalHandler.hpp"

namespace probe {

// ---------------------------------------------------------------------------
// Case model
// ---------------------------------------------------------------------------

/// One PDO mapping entry: object index : subindex : bit length.
struct EntrySpec {
    uint16_t index = 0;
    uint8_t sub = 0;
    uint16_t bits = 0;
};

/// One PDO mapping object (e.g. 0x1601) with its entries.
struct PdoSpec {
    uint16_t pdo_index = 0;
    EtherCAT::PDO::PDODirection dir = EtherCAT::PDO::PDODirection::RxPDO;
    std::vector<EntrySpec> entries;
};

/// A complete probe case: the set of PDOs written before SAFE-OP.
struct CaseSpec {
    std::string name;
    std::vector<PdoSpec> pdos;

    std::string describe() const {
        std::string out = name + " [";
        for (size_t i = 0; i < pdos.size(); ++i) {
            if (i) out += " | ";
            out += "0x" + std::format("{:04X}", pdos[i].pdo_index);
            out += (pdos[i].dir == EtherCAT::PDO::PDODirection::RxPDO) ? ":rx" : ":tx";
            out += "(" + std::to_string(pdos[i].entries.size()) + " entries)";
        }
        out += "]";
        return out;
    }
};

// ---------------------------------------------------------------------------
// Outcome model
// ---------------------------------------------------------------------------

enum class Outcome {
    SafeOpOk,             ///< slave reached SAFE-OP
    InvalidInputMapping,  ///< AL status code 0x0024
    OtherAlError,         ///< any other AL status code
    Timeout,              ///< no confirmation within the probe timeout
    SetupError,           ///< INIT/mailbox/PRE-OP/write failed before SAFE-OP
    HostRejected,         ///< Tether refused the mapping client-side
};

struct CaseResult {
    std::string name;
    Outcome outcome = Outcome::SetupError;
    uint16_t al_status = 0;
    uint16_t al_status_code = 0;
    bool baseline_confirmed = true;  ///< A-B-A: last-good case still passed
    std::string note;
};

const char* outcomeName(Outcome o) {
    switch (o) {
        case Outcome::SafeOpOk:           return "PASS";
        case Outcome::InvalidInputMapping: return "FAIL 0x0024";
        case Outcome::OtherAlError:       return "FAIL AL";
        case Outcome::Timeout:            return "FAIL TIMEOUT";
        case Outcome::SetupError:         return "FAIL SETUP";
        case Outcome::HostRejected:       return "FAIL HOST";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Probe
// ---------------------------------------------------------------------------

struct ProbeConfig {
    std::string iface;
    int slave_idx = 0;
    EtherCAT::MailboxSyncManagerConfig mbox_out{0x1200, 512};
    EtherCAT::MailboxSyncManagerConfig mbox_in{0x1000, 512};
    uint16_t mailbox_protocols = 0x0004;
    std::chrono::milliseconds safeop_timeout{400};
    std::chrono::milliseconds poll_interval{10};
};

class Probe {
public:
    explicit Probe(ProbeConfig cfg) : cfg_(std::move(cfg)) {}

    /// Bring up the host Ethernet, master, discovery and select the slave.
    bool connect(const char* tag);

    /// Set the known-good case used for the A-B-A integrity re-check.
    void setBaseline(CaseSpec c) { last_good_ = std::move(c); }

    /// Run one case with the A-B-A re-check.  Updates last_good_ on PASS.
    CaseResult runCase(const CaseSpec& c);

    void shutdown();

    EtherCAT::Slave& slave() { return *slave_; }
    EtherCAT::Master& master() { return *master_; }

private:
    /// Force INIT (two-step AL reset), configure mailbox, enter PRE-OP.
    bool prepSlave();

    /// Write the case's PDO mappings, apply custom PDOs, configure SMs.
    /// Returns Outcome::SetupError on any write failure.
    Outcome writePdos(const CaseSpec& c, std::string& note);

    /// Request SAFE-OP and poll; fills AL status + code on failure.
    Outcome probeSafeOp(uint16_t& al_status, uint16_t& al_code);

    /// Translate an ObjectDictionaryEntry for configureCustomTxPDO,
    /// keeping the synthesized object alive for the whole probe session.
    const EtherCAT::ObjectDictionary::ObjectDictionaryEntry* synthEntry(
        uint16_t index, uint8_t sub);

    ProbeConfig cfg_;
    Tether::Examples::HostEtherNetSession session_;
    std::unique_ptr<EtherCAT::Master> master_;
    EtherCAT::Slave* slave_ = nullptr;
    std::deque<EtherCAT::ObjectDictionary::ObjectDictionaryEntry> od_store_;
    std::optional<CaseSpec> last_good_;
};

bool Probe::connect(const char* tag) {
    if (!Tether::Examples::initHostEthernet(session_, cfg_.iface, tag)) {
        return false;
    }
    master_ = std::make_unique<EtherCAT::Master>();
    if (!Tether::Examples::setupEncapsulation(session_, *master_,
                                                  Tether::Examples::EncapsulationConfig{},
                                                  tag)) {
        Tether::Examples::shutdownHostEthernet(session_);
        return false;
    }
    Tether::Examples::startHostPollThread(session_, tag);
    if (!Tether::Examples::startHostMaster(session_, *master_, tag)) {
        Tether::Examples::shutdownHostEthernet(session_);
        return false;
    }

    auto slaves = master_->discovery().discover(EtherCAT::DiscoveryOption::All);
    if (slaves.empty()) {
        TETHER_LOGE(tag, "No slaves discovered");
        shutdown();
        return false;
    }
    if (cfg_.slave_idx < 0 ||
        static_cast<size_t>(cfg_.slave_idx) >= master_->getDiscoveredSlaveCount()) {
        TETHER_LOGE(tag, "Slave index {} out of range (0..{})",
                    cfg_.slave_idx, master_->getDiscoveredSlaveCount() - 1);
        shutdown();
        return false;
    }
    slave_ = &master_->slave(static_cast<uint16_t>(cfg_.slave_idx));
    const auto& info = slaves[static_cast<size_t>(cfg_.slave_idx)];
    TETHER_LOGI(tag, "Probing slave {} (vendor=0x{:08X} product=0x{:08X})",
                cfg_.slave_idx,
                info.vendor_id ? *info.vendor_id : 0,
                info.product_code ? *info.product_code : 0);
    return true;
}

void Probe::shutdown() {
    if (master_) master_->stop();
    Tether::Examples::shutdownHostEthernet(session_);
}

const EtherCAT::ObjectDictionary::ObjectDictionaryEntry* Probe::synthEntry(
    uint16_t index, uint8_t sub) {
    od_store_.push_back({
        .index = index,
        .subindex = sub,
        .name = "probe",
        .data_type = EtherCAT::ObjectDictionary::ObjectDictionaryDataType::OctetString,
        .default_value = 0,
        .unit = EtherCAT::ObjectDictionary::Unit_None,
        .options_enum = nullptr,
        .min_value = 0,
        .max_value = 0,
        .modification_mode = EtherCAT::ObjectDictionary::ModificationMode::ReadOnly,
        .effective_time = EtherCAT::ObjectDictionary::EffectiveTime::Immediately,
        .comment = nullptr,
    });
    return &od_store_.back();
}

bool Probe::prepSlave() {
    // Two-step AL reset to INIT; clears any previous error state.
    EtherCAT::ALResetController reset_ctrl(*master_);
    auto r = reset_ctrl.resetSlave(static_cast<uint16_t>(cfg_.slave_idx), 0x01, 50, 50);
    if (!r.success) {
        TETHER_LOGW("pdo_probe", "AL reset to INIT failed (code=0x{:04X}) — continuing",
                    r.final_al_status_code);
    }

    auto mb = slave_->configureMailbox(cfg_.mbox_out, cfg_.mbox_in,
                                       cfg_.mailbox_protocols);
    if (mb != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE("pdo_probe", "Mailbox config failed: {}",
                    EtherCAT::slaveErrorToString(mb));
        return false;
    }
    auto pre = slave_->transitionToPreOp();
    if (pre != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE("pdo_probe", "PRE-OP transition failed: {}",
                    EtherCAT::slaveErrorToString(pre));
        return false;
    }
    slave_->clearCustomPDOs();
    return true;
}

Outcome Probe::writePdos(const CaseSpec& c, std::string& note) {
    for (const auto& pdo : c.pdos) {
        std::vector<EtherCAT::CustomPDOMappingEntry> entries;
        entries.reserve(pdo.entries.size());
        for (const auto& e : pdo.entries) {
            if (e.bits == 0 || e.bits % 8 != 0) {
                note = "entry 0x" + std::format("{:04X}", e.index) + ":" +
                       std::to_string(e.sub) + ":" + std::to_string(e.bits) +
                       " not byte-aligned/nonzero — rejected host-side";
                return Outcome::HostRejected;
            }
            entries.emplace_back(synthEntry(e.index, e.sub),
                                 static_cast<uint8_t>(e.bits / 8));
        }
        auto err = slave_->configureCustomTxPDO(pdo.pdo_index, entries, pdo.dir);
        if (err != EtherCAT::SlaveError::Ok) {
            note = "write 0x" + std::format("{:04X}", pdo.pdo_index) + " failed: " +
                   EtherCAT::slaveErrorToString(err);
            return Outcome::SetupError;
        }
    }

    if (auto err = slave_->applyCustomPDOs(); err != EtherCAT::SlaveError::Ok) {
        note = std::string("applyCustomPDOs failed: ") + EtherCAT::slaveErrorToString(err);
        return Outcome::SetupError;
    }
    if (auto err = slave_->configurePDOSyncManagers(); err != EtherCAT::SlaveError::Ok) {
        note = std::string("configurePDOSyncManagers failed: ") + EtherCAT::slaveErrorToString(err);
        return Outcome::SetupError;
    }
    return Outcome::SafeOpOk;
}

Outcome Probe::probeSafeOp(uint16_t& al_status, uint16_t& al_code) {
    if (!master_->requestSlaveApplicationLayerState(
            EtherCAT::SlaveAddress(static_cast<uint16_t>(cfg_.slave_idx)),
            static_cast<uint8_t>(EtherCAT::ECState::SafeOp))) {
        return Outcome::Timeout;
    }

    const auto deadline = std::chrono::steady_clock::now() + cfg_.safeop_timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(cfg_.poll_interval);
        uint8_t state = 0;
        if (master_->readSlaveApplicationLayerState(
                EtherCAT::SlaveAddress(static_cast<uint16_t>(cfg_.slave_idx)),
                state)) {
            if (state == static_cast<uint8_t>(EtherCAT::ECState::SafeOp)) {
                return Outcome::SafeOpOk;
            }
            if ((state & 0x10) != 0) {  // error flag set
                break;
            }
        }
    }

    // Read AL_STATUS and AL_STATUS_CODE for the report.
    master_->readRegister(
        EtherCAT::SlaveAddress(static_cast<uint16_t>(cfg_.slave_idx)),
        EtherCAT::reg::AL_STATUS, al_status);
    master_->readRegister(
        EtherCAT::SlaveAddress(static_cast<uint16_t>(cfg_.slave_idx)),
        EtherCAT::reg::AL_STATUS_CODE, al_code);
    if (al_code == 0x0024) return Outcome::InvalidInputMapping;
    if (al_code != 0) return Outcome::OtherAlError;
    return Outcome::Timeout;
}

CaseResult Probe::runCase(const CaseSpec& c) {
    CaseResult result;
    result.name = c.name;

    if (!prepSlave()) {
        result.outcome = Outcome::SetupError;
        result.note = "INIT/mailbox/PRE-OP failed";
        return result;
    }

    result.outcome = writePdos(c, result.note);
    if (result.outcome == Outcome::SetupError ||
        result.outcome == Outcome::HostRejected) {
        return result;  // never reached SAFE-OP; nothing to A-B-A against
    }

    result.outcome = probeSafeOp(result.al_status, result.al_status_code);

    // A-B-A integrity check: a failed case is only trustworthy if the last
    // known-good case still reaches SAFE-OP afterwards.
    if (result.outcome != Outcome::SafeOpOk && last_good_.has_value()) {
        TETHER_LOGW("pdo_probe", "A-B-A: re-checking known-good case '{}' after failure of '{}'",
                    last_good_->name, c.name);
        if (!prepSlave()) {
            result.baseline_confirmed = false;
            result.note = "A-B-A: baseline PRE-OP failed — device state suspect";
            return result;
        }
        std::string note;
        auto w = writePdos(*last_good_, note);
        if (w != Outcome::SafeOpOk) {
            result.baseline_confirmed = false;
            result.note = "A-B-A: baseline write failed (" + note + ")";
            return result;
        }
        uint16_t bs = 0, bc = 0;
        auto bo = probeSafeOp(bs, bc);
        if (bo != Outcome::SafeOpOk) {
            result.baseline_confirmed = false;
            result.note = "A-B-A: baseline no longer reaches SAFE-OP (AL=0x" +
                          std::format("{:04X}", bc) + ") — device state suspect";
        }
    }

    if (result.outcome == Outcome::SafeOpOk) {
        last_good_ = c;
    }
    return result;
}

// ---------------------------------------------------------------------------
// CLI parsing helpers
// ---------------------------------------------------------------------------

/// Parse "1601:rx:7010:0:32,7020:0:32" into a PdoSpec.
std::optional<PdoSpec> parsePdoSpec(const std::string& s) {
    auto parts = [&](const std::string& str, char sep) {
        std::vector<std::string> out;
        std::string cur;
        for (char ch : str) {
            if (ch == sep) { out.push_back(cur); cur.clear(); }
            else cur += ch;
        }
        out.push_back(cur);
        return out;
    };

    // Format: <pdo_index>:<rx|tx>:<entry>,<entry>,...
    // Entries carry their own colons (idx:sub:bits), so only the first
    // two fields are split off; the remainder is the entry list.
    const auto c1 = s.find(':');
    if (c1 == std::string::npos) return std::nullopt;
    const auto c2 = s.find(':', c1 + 1);
    if (c2 == std::string::npos) return std::nullopt;

    PdoSpec pdo;
    try {
        pdo.pdo_index = static_cast<uint16_t>(std::stoul(s.substr(0, c1), nullptr, 16));
    } catch (...) { return std::nullopt; }
    const std::string dir_str = s.substr(c1 + 1, c2 - c1 - 1);
    pdo.dir = (dir_str == "tx") ? EtherCAT::PDO::PDODirection::TxPDO
                                : EtherCAT::PDO::PDODirection::RxPDO;

    // Entries: comma-separated "index:sub:bits"
    for (const auto& es : parts(s.substr(c2 + 1), ',')) {
        auto f = parts(es, ':');
        if (f.size() != 3) return std::nullopt;
        EntrySpec e;
        try {
            e.index = static_cast<uint16_t>(std::stoul(f[0], nullptr, 16));
            e.sub = static_cast<uint8_t>(std::stoul(f[1], nullptr, 10));
            e.bits = static_cast<uint16_t>(std::stoul(f[2], nullptr, 10));
        } catch (...) { return std::nullopt; }
        pdo.entries.push_back(e);
    }
    return pdo;
}

/// Parse "name=1601:rx:...;1A01:tx:..." into a CaseSpec.
std::optional<CaseSpec> parseCaseSpec(const std::string& s) {
    CaseSpec c;
    auto eq = s.find('=');
    if (eq == std::string::npos) return std::nullopt;
    c.name = s.substr(0, eq);

    std::string body = s.substr(eq + 1);
    std::string cur;
    for (char ch : body) {
        if (ch == ';') {
            auto p = parsePdoSpec(cur);
            if (!p) return std::nullopt;
            c.pdos.push_back(*p);
            cur.clear();
        } else cur += ch;
    }
    if (!cur.empty()) {
        auto p = parsePdoSpec(cur);
        if (!p) return std::nullopt;
        c.pdos.push_back(*p);
    }
    return c;
}

/// Build a CaseSpec from one PDO whose entries are generated by `gen`.
CaseSpec generatedCase(const std::string& name, PdoSpec base,
                       const std::vector<EntrySpec>& entries) {
    base.entries = entries;
    CaseSpec c;
    c.name = name;
    c.pdos.push_back(std::move(base));
    return c;
}

} // namespace probe

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

namespace {
constexpr const char* kTag = "pdo_mapping_probe";
std::atomic<bool> g_cancel{false};
} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("pdo_mapping_probe", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addSlaveArg(program, 0);
    Tether::Examples::addMailboxSizeArg(program, 512);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addDebugArg(program);

    program.add_argument("--case")
        .help("Explicit case: name=PDO;PDO... with PDO=index:rx|tx:idx:sub:bits,... "
              "(indices hex, sub/bits decimal). Repeatable.")
        .append();
    program.add_argument("--case-file")
        .help("Read --case lines from a file (one case per line, no shell "
              "quoting needed). Repeatable.")
        .append();
    program.add_argument("--baseline")
        .help("Known-good case used for A-B-A re-checks (same syntax as --case).")
        .default_value(std::string{});
    program.add_argument("--sweep-count")
        .help("Entry-count capacity: pdo:obj:sub:bits[:max] (e.g. 1A02:tx:4101:0:32:40).")
        .default_value(std::string{});
    program.add_argument("--sweep-size")
        .help("Per-entry bit sizes: pdo:obj:sub:bits[:list] (e.g. 1A01:tx:6010:0:32:8,16,32,64).")
        .default_value(std::string{});
    program.add_argument("--sweep-total")
        .help("Total-size ceiling: pdo:obj:sub:bits:max (e.g. 1A01:tx:6010:0:32:64).")
        .default_value(std::string{});
    program.add_argument("--bad")
        .help("Run the built-in negative battery (non-existent index, bad "
              "subindex, 0-bit entry, oversized total).")
        .default_value(false).implicit_value(true);
    program.add_argument("--timeout-ms")
        .help("SAFE-OP probe timeout in ms (default 400).")
        .default_value(400).scan<'i', int>();
    program.add_argument("--only")
        .help("Only run cases whose name matches this substring.")
        .default_value(std::string{});

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    if (Tether::Examples::printDebugHelpIfRequested(
            program.get<std::string>("--debug"))) {
        return 0;
    }

    std::string iface =
        Tether::Examples::resolveInterface(program.get<std::string>("--interface"), kTag);
    if (iface.empty()) return 1;

    // ---- Build the case list --------------------------------------------

    std::vector<probe::CaseSpec> cases;
    for (const auto& cs : program.get<std::vector<std::string>>("--case")) {
        auto c = probe::parseCaseSpec(cs);
        if (!c) {
            TETHER_LOGE(kTag, "Bad --case spec: '{}'", cs);
            return 1;
        }
        cases.push_back(*c);
    }
    for (const auto& path : program.get<std::vector<std::string>>("--case-file")) {
        std::ifstream in(path);
        if (!in) {
            TETHER_LOGE(kTag, "Cannot open --case-file '{}'", path);
            return 1;
        }
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            auto c = probe::parseCaseSpec(line);
            if (!c) {
                TETHER_LOGE(kTag, "Bad case line in '{}': '{}'", path, line);
                return 1;
            }
            cases.push_back(*c);
        }
    }

    // --sweep-count: capacity of a PDO mapping object.
    // Format: <pdo>:<rx|tx>:<obj>:<sub>:<bits>[:max]  (max default 32).
    if (auto sc = program.get<std::string>("--sweep-count"); !sc.empty()) {
        auto seg = [&](const std::string& s, char sep) {
            std::vector<std::string> out; std::string cur;
            for (char ch : s) { if (ch == sep) { out.push_back(cur); cur.clear(); } else cur += ch; }
            out.push_back(cur);
            return out;
        };
        auto f = seg(sc, ':');
        if (f.size() < 5) { TETHER_LOGE(kTag, "Bad --sweep-count: '{}'", sc); return 1; }
        probe::PdoSpec base;
        base.pdo_index = static_cast<uint16_t>(std::stoul(f[0], nullptr, 16));
        base.dir = (f[1] == "tx") ? EtherCAT::PDO::PDODirection::TxPDO
                                  : EtherCAT::PDO::PDODirection::RxPDO;
        probe::EntrySpec e;
        e.index = static_cast<uint16_t>(std::stoul(f[2], nullptr, 16));
        e.sub = static_cast<uint8_t>(std::stoul(f[3], nullptr, 10));
        e.bits = static_cast<uint16_t>(std::stoul(f[4], nullptr, 10));
        int max = f.size() > 5 ? std::stoi(f[5]) : 32;
        for (int n = 1; n <= max; ++n) {
            std::vector<probe::EntrySpec> entries(static_cast<size_t>(n), e);
            cases.push_back(probe::generatedCase(
                "count-" + std::to_string(n) + "x" + f[2] + ":" + f[3] + ":" + f[4],
                base, entries));
        }
    }

    // --sweep-size: which bit sizes a single entry accepts.
    if (auto ss = program.get<std::string>("--sweep-size"); !ss.empty()) {
        std::vector<std::string> seg; std::string cur;
        for (char ch : ss) { if (ch == ':') { seg.push_back(cur); cur.clear(); } else cur += ch; }
        seg.push_back(cur);
        if (seg.size() < 4) { TETHER_LOGE(kTag, "Bad --sweep-size: '{}'", ss); return 1; }
        probe::PdoSpec base;
        base.pdo_index = static_cast<uint16_t>(std::stoul(seg[0], nullptr, 16));
        base.dir = (seg[1] == "tx") ? EtherCAT::PDO::PDODirection::TxPDO
                                    : EtherCAT::PDO::PDODirection::RxPDO;
        probe::EntrySpec e;
        e.index = static_cast<uint16_t>(std::stoul(seg[2], nullptr, 16));
        e.sub = static_cast<uint8_t>(std::stoul(seg[3], nullptr, 10));
        std::vector<uint16_t> sizes;
        if (seg.size() > 5) {
            std::string list = seg[5]; std::string item;
            for (char ch : list) { if (ch == ',') { sizes.push_back(std::stoul(item)); item.clear(); } else item += ch; }
            if (!item.empty()) sizes.push_back(std::stoul(item));
        } else {
            for (uint16_t b : {8u, 16u, 24u, 32u, 48u, 64u}) sizes.push_back(b);
        }
        for (uint16_t b : sizes) {
            e.bits = b;
            cases.push_back(probe::generatedCase(
                "size-0x" + std::format("{:04X}", e.index) + ":" + std::to_string(e.sub) +
                "=" + std::to_string(b) + "b", base, {e}));
        }
    }

    // --sweep-total: grow total PDO size until the device refuses.
    if (auto st = program.get<std::string>("--sweep-total"); !st.empty()) {
        std::vector<std::string> seg; std::string cur;
        for (char ch : st) { if (ch == ':') { seg.push_back(cur); cur.clear(); } else cur += ch; }
        seg.push_back(cur);
        if (seg.size() < 5) { TETHER_LOGE(kTag, "Bad --sweep-total: '{}'", st); return 1; }
        probe::PdoSpec base;
        base.pdo_index = static_cast<uint16_t>(std::stoul(seg[0], nullptr, 16));
        base.dir = (seg[1] == "tx") ? EtherCAT::PDO::PDODirection::TxPDO
                                    : EtherCAT::PDO::PDODirection::RxPDO;
        probe::EntrySpec e;
        e.index = static_cast<uint16_t>(std::stoul(seg[2], nullptr, 16));
        e.sub = static_cast<uint8_t>(std::stoul(seg[3], nullptr, 10));
        e.bits = static_cast<uint16_t>(std::stoul(seg[4], nullptr, 10));
        int max = std::stoi(seg[5]);
        for (int n = 1; n <= max; ++n) {
            std::vector<probe::EntrySpec> entries(static_cast<size_t>(n), e);
            cases.push_back(probe::generatedCase(
                "total-" + std::to_string(n * e.bits / 8) + "B", base, entries));
        }
    }

    // --bad: negative battery.
    if (program.get<bool>("--bad")) {
        // A valid entry as a base, then break it in each dimension.
        probe::PdoSpec base;
        base.pdo_index = 0x1A01;
        base.dir = EtherCAT::PDO::PDODirection::TxPDO;

        { // non-existent object index
            probe::EntrySpec e{0xFFFF, 0, 32};
            cases.push_back(probe::generatedCase("bad-nonexistent-index", base, {e}));
        }
        { // non-PDO-mappable object (Device Type 0x1000)
            probe::EntrySpec e{0x1000, 0, 32};
            cases.push_back(probe::generatedCase("bad-non-mappable-0x1000", base, {e}));
        }
        { // bad subindex (beyond record)
            probe::EntrySpec e{0x6010, 0xFF, 32};
            cases.push_back(probe::generatedCase("bad-subindex-0xFF", base, {e}));
        }
        { // 0-bit entry (host-side rejection expected)
            probe::EntrySpec e{0x6010, 0, 0};
            cases.push_back(probe::generatedCase("bad-zero-bits", base, {e}));
        }
        { // oversized total (128 x UDINT = 512 B in one PDO)
            probe::EntrySpec e{0x6010, 0, 32};
            std::vector<probe::EntrySpec> entries(128, e);
            cases.push_back(probe::generatedCase("bad-oversized-total", base, entries));
        }
    }

    if (cases.empty()) {
        TETHER_LOGE(kTag, "No cases given — use --case, --sweep-*, or --bad.");
        return 1;
    }

    const std::string only = program.get<std::string>("--only");
    std::vector<probe::CaseSpec> filtered;
    for (auto& c : cases) {
        if (only.empty() || c.name.find(only) != std::string::npos) {
            filtered.push_back(std::move(c));
        }
    }
    cases = std::move(filtered);
    if (cases.empty()) {
        TETHER_LOGE(kTag, "No cases match --only '{}'", only);
        return 1;
    }

    // ---- Run -------------------------------------------------------------

    Tether::Platform::ensureRealtimeKernelOrExit();
    Tether::Utils::SignalHandler sig_handler(g_cancel);

    probe::ProbeConfig pcfg;
    pcfg.iface = iface;
    pcfg.slave_idx = program.get<int>("--slave");
    // Mailbox args
    {
        Tether::Examples::MailboxSizeConfig mb_size;
        Tether::Examples::MailboxAddressConfig mb_addr;
        if (!Tether::Examples::parseMailboxSize(
                program.get<std::string>("--mailbox-size"), mb_size) ||
            !Tether::Examples::parseMailboxAddress(
                program.get<std::string>("--mailbox-address"), mb_addr)) {
            return 1;
        }
        pcfg.mbox_in = {mb_addr.inAddress, mb_size.inSize};
        pcfg.mbox_out = {mb_addr.outAddress, mb_size.outSize};
    }
    pcfg.safeop_timeout = std::chrono::milliseconds(program.get<int>("--timeout-ms"));

    probe::Probe probe(pcfg);
    if (!probe.connect(kTag)) return 2;

    // Baseline: explicit --baseline, else the first case.
    if (auto bs = program.get<std::string>("--baseline"); !bs.empty()) {
        auto c = probe::parseCaseSpec(bs);
        if (!c) { TETHER_LOGE(kTag, "Bad --baseline spec: '{}'", bs); probe.shutdown(); return 1; }
        probe.setBaseline(*c);
    } else {
        probe.setBaseline(cases.front());
    }

    TETHER_LOGI(kTag, "Running {} case(s)", cases.size());

    // Summary table columns.
    std::print("\n{:<28} {:<12} {:<10} {:<6} {}\n",
               "CASE", "RESULT", "AL_CODE", "A-B-A", "NOTE");
    std::print("{}\n", std::string(80, '-'));

    int pass = 0, fail = 0;
    bool aborted = false;
    for (const auto& c : cases) {
        if (g_cancel.load()) break;
        auto r = probe.runCase(c);

        if (!r.baseline_confirmed) {
            aborted = true;
            std::print("{:<28} {:<12} {:<10} {:<6} {}\n",
                       r.name, "ABORTED", "0x" + std::format("{:04X}", r.al_status_code),
                       "FAIL", "baseline no longer passes — device state suspect");
            TETHER_LOGE(kTag, "A-B-A baseline check failed after case '{}' — stopping.",
                        r.name);
            break;
        }

        const bool ok = (r.outcome == probe::Outcome::SafeOpOk);
        ok ? ++pass : ++fail;
        std::print("{:<28} {:<12} {:<10} {:<6} {}\n",
                   r.name, probe::outcomeName(r.outcome),
                   r.al_status_code ? "0x" + std::format("{:04X}", r.al_status_code) : "-",
                   ok ? "ok" : "ok",
                   r.note);
        std::fflush(stdout);
    }

    std::print("{}\n", std::string(80, '-'));
    std::print("Summary: {} pass, {} fail{}\n", pass, fail,
               aborted ? " (ABORTED: baseline integrity check failed)" : "");

    probe.shutdown();
    return aborted ? 3 : (fail == 0 ? 0 : 1);
}
