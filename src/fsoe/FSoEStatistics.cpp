// SPDX-License-Identifier: MIT

#include "fsoe/FSoEStatistics.hpp"

namespace FSoE {

void FSoEStatistics::logDiagnostic(uint16_t errorCode, const char* message) {
    if (!enableDiagnostics_) {
        return;
    }

    FSoEDiagnosticEntry entry;
    entry.timestamp = currentTimestamp_;
    entry.errorCode = errorCode;
    entry.state = currentState_;
    entry.sequenceNumber = currentSequence_;
    entry.connectionId = currentConnectionId_;
    strncpy(entry.message, message, sizeof(entry.message) - 1);
    entry.message[sizeof(entry.message) - 1] = '\0';

    if (diagnostics_.size() >= maxEntries_) {
        diagnostics_.erase(diagnostics_.begin());
    }

    diagnostics_.push_back(entry);
}

} // namespace FSoE
