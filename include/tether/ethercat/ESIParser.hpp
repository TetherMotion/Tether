#pragma once

/**
 * @file ESIParser.hpp
 * @brief ESI (EtherCAT Slave Information) XML parser
 *
 * The actual XML parsing depends on tinyxml2, so this header and its
 * implementation live in the separate tether_esi library. The data
 * structures themselves are in ESITypes.hpp (no tinyxml2 dependency).
 */

#include <string>
#include <vector>
#include <cstdint>
#include <optional>

#include "tether/ethercat/ESITypes.hpp"

namespace EtherCAT {
namespace ESI {

// Parse the ESI XML file and return a list of DeviceInfo entries found.
// On failure returns false and leaves "devices" empty.
bool parseESIFile(const std::string& path, std::vector<DeviceInfo>& devices, std::string& errMsg);

// Render human-readable text for a device. If onlyMailboxes is true, only include mailbox info.
std::string formatDeviceHumanReadable(const DeviceInfo& dev, bool onlyMailboxes=false);

// Render JSON string for programmatic consumption
std::string formatDeviceJSON(const DeviceInfo& dev);

} // namespace ESI
} // namespace EtherCAT
