#include "tether/drives/NexcobotESC211/CrcVerifier.hpp"

#include <algorithm>
#include <format>
#include <string>
#include <thread>
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
    // Expectations carrying an alternate value need a manual poll — the
    // generic engine tests a single expected value per object.
    const bool hasAlternate = std::any_of(
        expectations.begin(), expectations.end(),
        [](const CrcExpectation& e) {
            return e.alternateCrc != 0 && e.alternateCrc != e.expectedCrc;
        });
    if (hasAlternate) {
        const auto deadline =
            std::chrono::steady_clock::now() + timeout;
        while (!stop_token_.stop_requested()) {
            bool allMatch = true;
            for (const auto& exp : expectations) {
                const uint32_t v = readCrc(exp.crcIndex);
                const bool match = v == exp.expectedCrc ||
                                   (exp.alternateCrc != 0 &&
                                    v == exp.alternateCrc);
                TETHER_LOGI(tag_, "{} CRC: device=0x{:08X} "
                            "expected=0x{:08X}{}{}", exp.name, v,
                            exp.expectedCrc,
                            exp.alternateCrc
                                ? std::format(" alt=0x{:08X}",
                                              exp.alternateCrc)
                                : std::string{},
                            match ? " (match)" : "");
                allMatch &= match;
            }
            if (allMatch) return true;
            if (std::chrono::steady_clock::now() >= deadline) break;
            std::this_thread::sleep_for(poll_interval_);
        }
        return false;
    }

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
