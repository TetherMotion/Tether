#include "tether/modbus/config/Config.hpp"

#include <charconv>
#include <fstream>

#if !defined(ESP_PLATFORM)
#include "tether/modbus/link/SerialPortLink.hpp"
#include "tether/modbus/link/TcpLink.hpp"
#endif

namespace tether::modbus {
namespace {
std::string_view trim(std::string_view s) {
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string_view::npos) return {};
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}
Expected<int> toInt(const IniSection& s, const std::string& k, int def) {
    auto it = s.find(k);
    if (it == s.end()) return def;
    int v = 0;
    auto [p, ec] = std::from_chars(it->second.data(),
                                   it->second.data() + it->second.size(), v);
    if (ec != std::errc{})
        return err<int>(ModbusError::config("bad int: " + k));
    return v;
}
Expected<bool> toBool(const IniSection& s, const std::string& k, bool def) {
    auto it = s.find(k);
    if (it == s.end()) return def;
    const auto& v = it->second;
    if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
    if (v == "0" || v == "false" || v == "no" || v == "off") return false;
    return err<bool>(ModbusError::config("bad bool: " + k));
}
} // namespace

Expected<IniFile> parseIni(std::string_view text) {
    IniFile ini;
    IniSection* cur = nullptr;
    size_t pos = 0;
    while (pos <= text.size()) {
        const auto nl = text.find('\n', pos);
        const auto line = trim(text.substr(
            pos, nl == std::string_view::npos ? nl : nl - pos));
        pos = nl == std::string_view::npos ? text.size() + 1 : nl + 1;
        if (line.empty() || line.front() == '#' || line.front() == ';')
            continue;
        if (line.front() == '[' && line.back() == ']') {
            cur = &ini[std::string(trim(line.substr(1, line.size() - 2)))];
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string_view::npos)
            return err<IniFile>(ModbusError::config(
                "bad line: " + std::string(line)));
        if (!cur)
            return err<IniFile>(ModbusError::config("key outside section"));
        (*cur)[std::string(trim(line.substr(0, eq)))] =
            std::string(trim(line.substr(eq + 1)));
    }
    return ini;
}

Expected<IniFile> loadIniFile(const std::string& path) {
    std::ifstream f(path);
    if (!f)
        return err<IniFile>(ModbusError::config("cannot open: " + path));
    std::string text((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    return parseIni(text);
}

Expected<BusConfig>
ModbusConfig::busFromSection(const IniSection& s, const std::string& name) {
    BusConfig c;
    c.name = name;
    auto it = s.find("mode");
    if (it != s.end()) c.mode = it->second;
    it = s.find("device");
    if (it != s.end()) {
        auto sc = SerialConfig::parse(it->second);
        if (!sc) return err<BusConfig>(sc.error());
        c.serial = *sc;
    }
    it = s.find("host");  if (it != s.end()) c.host = it->second;
    if (auto p = toInt(s, "port", 502); !p) return err<BusConfig>(p.error());
    else c.port = uint16_t(*p);
    if (auto u = toInt(s, "unit", 1); !u) return err<BusConfig>(u.error());
    else c.unit = uint8_t(*u);
    if (auto t = toInt(s, "timeout_ms", 1000); !t)
        return err<BusConfig>(t.error());
    else c.invoke.timeoutMs = *t;
    if (auto r = toInt(s, "retries", 0); !r)
        return err<BusConfig>(r.error());
    else c.invoke.retries = *r;
    if (auto d = toInt(s, "retry_delay_ms", 0); !d)
        return err<BusConfig>(d.error());
    else c.invoke.retryDelayMs = *d;
    if (auto d = toInt(s, "inter_frame_us", 0); !d)
        return err<BusConfig>(d.error());
    else c.channel.interFrameDelayUs = *d;
    if (auto e = toBool(s, "expect_echo", false); !e)
        return err<BusConfig>(e.error());
    else c.channel.expectEcho = *e;
    it = s.find("profile"); if (it != s.end()) c.profileId = it->second;
    it = s.find("poll");
    if (it != s.end()) {
        // "name:ms, name:ms"
        size_t p = 0;
        const std::string& v = it->second;
        while (p <= v.size()) {
            const auto comma = v.find(',', p);
            const auto entry = trim(std::string_view(v).substr(
                p, comma == std::string::npos ? comma : comma - p));
            p = comma == std::string::npos ? v.size() + 1 : comma + 1;
            if (entry.empty()) continue;
            const auto colon = entry.find(':');
            if (colon == std::string_view::npos)
                return err<BusConfig>(ModbusError::config(
                    "bad poll entry: " + std::string(entry)));
            int ms = 0;
            std::from_chars(entry.data() + colon + 1,
                            entry.data() + entry.size(), ms);
            c.polls.emplace_back(std::string(entry.substr(0, colon)), ms);
        }
    }
    return c;
}

Expected<ModbusConfig> ModbusConfig::fromIni(const IniFile& ini) {
    ModbusConfig cfg;
    for (const auto& [sec, kv] : ini) {
        if (sec.rfind("modbus.", 0) != 0) continue;
        auto bus = busFromSection(kv, sec.substr(7));
        if (!bus) return err<ModbusConfig>(bus.error());
        cfg.buses.push_back(std::move(*bus));
    }
    return cfg;
}

ModbusManager::~ModbusManager() { stop(); }

Expected<void> ModbusManager::configure(const ModbusConfig& cfg) {
    for (const auto& bc : cfg.buses) {
        auto b = std::make_unique<Bus>();
        b->cfg = bc;
        if (bc.mode == "rtu") {
#if !defined(ESP_PLATFORM)
            auto link = makeSerialLink(bc.serial, bc.open);
            if (!link) return err<void>(link.error());
            b->channel = std::make_unique<RtuChannel>(
                std::move(*link), RtuTiming::forSerial(bc.serial),
                bc.channel);
#else
            return err<void>(ModbusError::unsupported("rtu on ESP"));
#endif
        } else if (bc.mode == "tcp") {
#if !defined(ESP_PLATFORM)
            auto link = makeTcpLink(bc.host, bc.port, bc.connectMs, bc.open);
            if (!link) return err<void>(link.error());
            b->channel = std::make_unique<MbapChannel>(std::move(*link),
                                                       bc.channel);
#else
            return err<void>(ModbusError::unsupported("tcp on ESP"));
#endif
        } else {
            return err<void>(ModbusError::config("bad mode: " + bc.mode));
        }
        b->executor = std::make_unique<BusExecutor>(*b->channel);
        b->client = std::make_unique<ModbusClient>(*b->executor, bc.unit,
                                                 bc.invoke);
        if (!bc.profileId.empty()) {
            const auto* p = reg_.find(bc.profileId);
            if (!p)
                return err<void>(ModbusError::config(
                    "unknown profile: " + bc.profileId));
            b->device = std::make_unique<BoundDevice>(*b->client, *p);
        }
        if (!bc.polls.empty())
            b->poller = std::make_unique<Poller>(*b->client);
        buses_[bc.name] = std::move(b);
    }
    return {};
}

Expected<void> ModbusManager::start() {
    for (auto& [_, b] : buses_) {
        if (auto e = b->channel->open(); !e) return e;
        if (b->poller) {
            if (b->device) {
                for (const auto& [name, ms] : b->cfg.polls) {
                    const auto* d = b->device->profile().find(name);
                    if (!d)
                        return err<void>(ModbusError::config(
                            "poll unknown register: " + name));
                    PollItem it;
                    it.fc = isBitTable(d->addr.table)
                        ? (d->addr.table == TableId::Coil
                               ? FunctionCode::ReadCoils
                               : FunctionCode::ReadDiscreteInputs)
                        : (d->addr.table == TableId::InputRegister
                               ? FunctionCode::ReadInputRegisters
                               : FunctionCode::ReadHoldingRegisters);
                    it.addr = d->addr.offset;
                    it.qty = uint16_t(valueSize(d->spec.type));
                    it.intervalMs = ms;
                    it.name = name;
                    b->poller->add(std::move(it));
                }
            }
            if (auto e = b->poller->start(); !e) return e;
        }
    }
    return {};
}

void ModbusManager::stop() {
    for (auto& [_, b] : buses_) {
        if (b->poller) b->poller->stop();
        if (b->channel) b->channel->close();
    }
}

ModbusClient* ModbusManager::client(const std::string& bus) {
    auto it = buses_.find(bus);
    return it == buses_.end() ? nullptr : it->second->client.get();
}
BoundDevice* ModbusManager::device(const std::string& bus) {
    auto it = buses_.find(bus);
    return it == buses_.end() ? nullptr : it->second->device.get();
}
Poller* ModbusManager::poller(const std::string& bus) {
    auto it = buses_.find(bus);
    return it == buses_.end() ? nullptr : it->second->poller.get();
}
BusConfig* ModbusManager::busConfig(const std::string& bus) {
    auto it = buses_.find(bus);
    return it == buses_.end() ? nullptr : &it->second->cfg;
}

} // namespace tether::modbus
