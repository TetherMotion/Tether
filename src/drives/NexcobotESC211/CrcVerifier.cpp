#include "tether/drives/NexcobotESC211/CrcVerifier.hpp"

#include <string>
#include <vector>

#include "logging/Logger.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

CrcVerifier::CrcVerifier(EtherCAT::Slave& slave,
                         std::chrono::milliseconds poll_interval,
                         const char* tag)
    : slave_(slave), poll_interval_(poll_interval), tag_(tag) {}

bool CrcVerifier::waitUntilMatch(std::span<const CrcExpectation> expectations,
                                 std::chrono::milliseconds timeout) {
    // Map to the generic expectation model; the name gets a " CRC" suffix
    // so log lines keep the pre-Tether "FNI CRC: device=0x..." shape.
    std::vector<std::string> names;
    std::vector<EtherCAT::SdoExpectation> sdoExpectations;
    names.reserve(expectations.size());
    sdoExpectations.reserve(expectations.size());
    for (const auto& exp : expectations) {
        names.push_back(std::string(exp.name) + " CRC");
        sdoExpectations.push_back(EtherCAT::SdoExpectation{
            .index    = exp.crcIndex,
            .subindex = 0x00,
            .expected = exp.expectedCrc,
            .name     = names.back().c_str(),
        });
    }
    return EtherCAT::waitForSdoValues(slave_, sdoExpectations,
                                      poll_interval_, timeout,
                                      stop_token_, tag_);
}

uint32_t CrcVerifier::readCrc(uint16_t crcIndex) {
    uint32_t crc = 0xFFFFFFFF;
    auto err = slave_.sdoReadU32(crcIndex, 0x00, crc);
    if (err != EtherCAT::SlaveError::Ok) {
        TETHER_LOGE(tag_, "Failed to read CRC (0x{:04X}): {}",
                    crcIndex, EtherCAT::slaveErrorToString(err));
        return 0xFFFFFFFF;
    }
    return crc;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
