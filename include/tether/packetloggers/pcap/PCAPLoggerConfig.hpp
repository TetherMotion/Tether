/**
 * @file PCAPLoggerConfig.hpp
 * @brief Configuration for the PCAP packet logger implementation
 */

#pragma once

#include <cstddef>
#include <string>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

/**
 * @brief Configuration for PCAP packet logging
 */
struct PCAPLoggerConfig {
    std::string filename;             ///< Output file path
    std::string interfaceName;        ///< Interface description
    std::string interfaceDescription; ///< Additional description
    bool logTx = true;                ///< Log transmitted frames
    bool logRx = true;                ///< Log received frames
    bool appendMode = false;          ///< Append to existing file
    size_t maxFileSize = 0;           ///< Maximum file size (0 = unlimited)
    bool rotateFiles = false;         ///< Rotate files when max size reached
    int maxRotatedFiles = 5;          ///< Maximum number of rotated files
};

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
