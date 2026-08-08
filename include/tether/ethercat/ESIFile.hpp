#pragma once

/**
 * @file ESIFile.hpp
 * @brief ESIFile — lightweight wrapper around parsed ESI (EtherCAT Slave
 *        Information) XML data
 *
 * ESIFile holds the parsed DeviceInfo entries from an ESI XML file. It can
 * be constructed from a file path (requires the tether_esi library to be
 * linked, which pulls in tinyxml2) or from pre-parsed DeviceInfo entries.
 *
 * The data structures are defined in ESITypes.hpp and have no XML parser
 * dependency, so ESIFile can be passed by const reference to Master/Slave
 * methods without requiring tether_esi at the call site. Only the
 * file-path constructor requires tether_esi.
 *
 * ## Usage
 * @code
 *   // Parse from file (requires tether_esi linked):
 *   EtherCAT::ESIFile esi("RP20_ECT_1.1.0.7.xml");
 *   if (esi.empty()) { handle error... }
 *
 *   // Use with master/slave methods:
 *   master.slave(0).configureMailbox(esi);
 *   master.slave(0).transitionToPreOp();
 *   master.slave(0).configurePDOSyncManagers(esi);
 * @endcode
 */

#include <string>
#include <vector>
#include <optional>

#include "tether/ethercat/ESITypes.hpp"
#include "tether/ethercat/TetherConfig.hpp"

namespace EtherCAT {

class ESIFile {
public:
    /// Construct from pre-parsed device info (no tether_esi needed).
    explicit ESIFile(std::vector<ESI::DeviceInfo> devices)
        : devices_(std::move(devices)) {}

    /// Parse an ESI XML file.
    /// Requires the tether_esi library to be linked (TETHER_HAVE_ESI=1).
    /// On parse failure, empty() returns true and errorMessage() has details.
    /// If ESI support is not compiled in, logs a critical error and leaves
    /// the object empty.
    explicit ESIFile(const std::string& path);

    /// True if no devices were parsed.
    bool empty() const { return devices_.empty(); }

    /// Parse error message (empty if no error).
    const std::string& errorMessage() const { return error_; }

    /// The parsed device entries.
    const std::vector<ESI::DeviceInfo>& devices() const { return devices_; }

    /// Find the device matching the given vendor/product.
    /// A value of 0 for vendorId or productCode acts as a wildcard.
    /// Falls back to devices[0] if no exact match is found.
    /// Returns nullptr if empty().
    const ESI::DeviceInfo* findDevice(uint32_t vendorId, uint32_t productCode) const {
        if (devices_.empty()) return nullptr;
        for (const auto& d : devices_) {
            bool vid_ok = (vendorId == 0) || (d.vendorId == vendorId);
            bool pid_ok = (productCode == 0) || (d.productCode == productCode);
            if (vid_ok && pid_ok) return &d;
        }
        return &devices_[0];
    }

private:
    std::vector<ESI::DeviceInfo> devices_;
    std::string error_;
};

} // namespace EtherCAT
