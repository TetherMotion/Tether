#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <optional>

namespace EtherCAT {
namespace ESI {

struct SyncManagerEntry {
    uint16_t startAddress{0};
    uint16_t defaultSize{0};
    uint8_t control{0};
    uint8_t enable{0};
    std::string name; // e.g., "MBoxOut" or "MBoxIn" or "ProcessIn"
};

struct MailboxInfo {
    std::optional<uint16_t> startAddress;
    std::optional<uint16_t> defaultSize;
    std::optional<uint16_t> protocols;
};

struct PDOEntry {
    uint16_t index{0};
    uint8_t subindex{0};
    uint16_t bitLen{0};
    std::string name;
    std::string dataType;
};

struct PDO {
    uint16_t index{0};
    std::string name;
    bool fixed{false};
    int sm{ -1 };
    std::vector<uint16_t> excludes;
    std::vector<PDOEntry> entries;
};

struct DeviceInfo {
    std::string name;
    std::string comment;
    std::string type;
    uint32_t productCode{0};
    uint32_t revision{0};
    uint32_t vendorId{0};

    // Simple mailbox/timeouts
    std::optional<uint32_t> mailbox_request_timeout_ms;
    std::optional<uint32_t> mailbox_response_timeout_ms;
    MailboxInfo mailbox;

    std::vector<SyncManagerEntry> syncManagers;
    std::vector<std::string> fmmus; // simple FMMU names like Outputs/Inputs
    std::vector<PDO> rxPdos;
    std::vector<PDO> txPdos;
};

// Parse the ESI XML file and return a list of DeviceInfo entries found.
// On failure returns false and leaves "devices" empty.
bool parseESIFile(const std::string& path, std::vector<DeviceInfo>& devices, std::string& errMsg);

// Render human-readable text for a device. If onlyMailboxes is true, only include mailbox info.
std::string formatDeviceHumanReadable(const DeviceInfo& dev, bool onlyMailboxes=false);

// Render JSON string for programmatic consumption
std::string formatDeviceJSON(const DeviceInfo& dev);

} // namespace ESI
} // namespace EtherCAT
