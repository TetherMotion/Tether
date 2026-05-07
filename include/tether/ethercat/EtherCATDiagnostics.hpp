#pragma once

#include "tether/ethercat/EtherCATMaster.hpp"

namespace EtherCAT::Diagnostics {

struct PreOperationalMailboxDiagnosticsOptions {
    Tether::Platform::LogLevel auto_configure_log_level = Tether::Platform::LogLevel::Info;
    bool attempt_auto_configure = true;
    bool log_sdo_mailbox_config = true;
    bool log_sii_mailbox = true;
    bool log_sync_manager_registers = true;
};

void logPreOperationalMailboxDiagnostics(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag = "EtherCAT",
    const PreOperationalMailboxDiagnosticsOptions& options = {});

bool logParsedSlaveSII(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag = "EtherCAT");

void logSlaveApplicationLayerDiagnostics(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag = "EtherCAT");

} // namespace EtherCAT::Diagnostics