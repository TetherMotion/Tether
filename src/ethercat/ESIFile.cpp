/**
 * @file ESIFile.cpp
 * @brief ESIFile path-constructor implementation
 *
 * This file lives in the tether_esi library (not the master library) because
 * it calls ESI::parseESIFile() which depends on tinyxml2.
 */

#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESIParser.hpp"

#if TETHER_HAVE_ESI

#include "logging/Logger.hpp"

namespace EtherCAT {

ESIFile::ESIFile(const std::string& path) {
    std::string err;
    std::vector<ESI::DeviceInfo> devices;
    if (ESI::parseESIFile(path, devices, err)) {
        devices_ = std::move(devices);
        if (devices_.empty()) {
            error_ = "ESI XML parsed but no <Device> entries found: " + path;
            TETHER_LOGW("ESIFile", "%s", error_.c_str());
        } else {
            TETHER_LOGI("ESIFile", "Parsed ESI XML '%s': %zu device(s)", path.c_str(), devices_.size());
        }
    } else {
        error_ = "Failed to parse ESI XML '" + path + "': " + err;
        TETHER_LOGE("ESIFile", "%s", error_.c_str());
    }
}

} // namespace EtherCAT

#else // !TETHER_HAVE_ESI

#include <cstdio>
#include <cstdlib>

namespace EtherCAT {

ESIFile::ESIFile(const std::string& path) {
    error_ = "ESI support is not compiled in (TETHER_HAVE_ESI=0). "
             "Cannot parse ESI XML file: " + path;

    // Critical error — the user explicitly asked to parse an ESI file but
    // the build does not include ESI support. Print to stderr and abort so
    // the problem is immediately visible rather than silently ignored.
    std::fprintf(stderr,
                 "\n*** CRITICAL ERROR: ESI support is not compiled in ***\n"
                 "    TETHER_HAVE_ESI is not defined (0).\n"
                 "    Cannot parse ESI XML file: %s\n"
                 "    Enable ESI support by building with TETHER_BUILD_EXTRACT_ESI=ON\n"
                 "    (which creates the tether_esi library and defines TETHER_HAVE_ESI=1).\n\n",
                 path.c_str());
    std::fflush(stderr);
    std::abort();
}

} // namespace EtherCAT

#endif // TETHER_HAVE_ESI
