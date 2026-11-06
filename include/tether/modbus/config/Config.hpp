/**
 * @file Config.hpp
 * @brief INI config → BusConfig/ModbusConfig → ModbusManager (plan §13).
 *
 * File format (Klipper-style sections):
 *   [modbus.bus1]
 *   mode = rtu | tcp | ascii | gateway
 *   device = /dev/ttyUSB0:19200-8E1      (rtu)
 *   host = 192.168.1.10                  (tcp client)
 *   port = 502
 *   unit = 1
 *   timeout_ms = 1000
 *   retries = 2                          (alias for max_retries)
 *   error_strategy = retry | retry_backoff | fail_fast |
 *                    drop_and_continue | circuit_breaker
 *   max_retries = 8
 *   backoff_base_ms = 2                  (retry_backoff)
 *   backoff_max_ms = 500
 *   circuit_threshold = 5                (circuit_breaker)
 *   circuit_reset_ms = 30000
 *   parity_check = passthrough | drop | mark   (rtu serial)
 *   profile = generic/vfd
 *   poll = freq_act:100, current:200     (name:ms pairs)
 */
#pragma once

#include <map>
#include <optional>
#include <vector>

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/client/ModbusClient.hpp"
#include "tether/modbus/client/Poller.hpp"
#include "tether/modbus/profile/Profile.hpp"
#include "tether/modbus/server/Server.hpp"

namespace tether::modbus {

/// One [section] of an INI file: key → value (trimmed, '#' comments).
using IniSection = std::map<std::string, std::string>;
using IniFile = std::map<std::string, IniSection>;

/// Parse an INI text → sections keyed by section name.
Expected<IniFile> parseIni(std::string_view text);
Expected<IniFile> loadIniFile(const std::string& path);

struct BusConfig {
    std::string  name;
    std::string  mode = "rtu";          ///< rtu | tcp
    SerialConfig serial;
    std::string  host = "127.0.0.1";
    uint16_t     port = 502;
    uint8_t      unit = 1;
    int          connectMs = 5000;
    TransactionOptions tx;     //!< connection default options
    ErrorPolicy  errors;            ///< bus-wide error strategy (code-first)
    ChannelOpts  channel;
    OpenOpts     open;
    std::string  profileId;
    std::vector<std::pair<std::string, int>> polls;  ///< name → ms
};

struct ModbusConfig {
    std::vector<BusConfig> buses;
    /// Parse a whole INI: every [modbus.<name>] section → BusConfig.
    static Expected<ModbusConfig> fromIni(const IniFile& ini);
    /// Parse one [modbus.x] section.
    static Expected<BusConfig> busFromSection(const IniSection& s,
                                              const std::string& name);
};

/**
 * @brief Owns configured buses: channel + executor + client + poller.
 */
class ModbusManager {
public:
    explicit ModbusManager(ProfileRegistry& reg) : reg_(reg) {}
    ~ModbusManager();

    Expected<void> configure(const ModbusConfig& cfg);
    Expected<void> start();
    void           stop();

    ModbusClient* client(const std::string& bus);
    BoundDevice*  device(const std::string& bus);
    Poller*       poller(const std::string& bus);
    BusConfig*    busConfig(const std::string& bus);

private:
    struct Bus {
        BusConfig cfg;
        std::unique_ptr<IChannel> channel;
        std::unique_ptr<BusExecutor> executor;
        std::unique_ptr<ModbusClient> client;
        std::unique_ptr<BoundDevice> device;
        std::unique_ptr<Poller> poller;
    };
    ProfileRegistry& reg_;
    std::map<std::string, std::unique_ptr<Bus>> buses_;
};

} // namespace tether::modbus
