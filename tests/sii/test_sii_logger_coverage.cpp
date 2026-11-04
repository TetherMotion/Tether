#include <gtest/gtest.h>

#include "tether/sii/SIILogger.hpp"
#include "tether/sii/SIIParser.hpp"

using namespace EtherCAT::SII;

// Coverage tests for src/sii/SIILogger.cpp — drive every log function with
// populated, empty, and partial SII data so all format arms execute.

namespace {

SIIData makeRichData() {
    SIIData d;
    d.pdi_control = 0x0001;
    d.pdi_config = 0x0010;
    d.sync_impulse_len = 0x0064;
    d.alias_address = 0x0007;
    d.checksum = 0xA5;
    d.checksum_ok = true;

    d.identity.vendor_id = 0x00000002;
    d.identity.product_code = 0x044C2C52;
    d.identity.revision_number = 0x00110002;
    d.identity.serial_number = 0x12345678;

    // All mailbox protocol bits so every name arm prints
    d.mailbox.protocols = 0x001F;
    d.mailbox.std_rx_offset = 0x1000;
    d.mailbox.std_rx_size = 0x0080;
    d.mailbox.std_tx_offset = 0x1080;
    d.mailbox.std_tx_size = 0x0080;
    d.mailbox.bootstrap_rx_offset = 0x1000;
    d.mailbox.bootstrap_rx_size = 0x0040;
    d.mailbox.bootstrap_tx_offset = 0x1040;
    d.mailbox.bootstrap_tx_size = 0x0040;

    d.has_general = true;
    d.general.coe_details = 0x01;
    d.general.foe_details = 0x01;
    d.general.eoe_details = 0x01;
    d.general.soe_channels = 0x01;
    d.general.ds402_channels = 0x01;

    // Sync managers: cover every type name
    const uint8_t types[] = {0, 1, 2, 3, 4, 5, 6, 7};
    for (size_t i = 0; i < 8; ++i) {
        auto& sm = d.sync_managers[i];
        sm.phys_start_address = static_cast<uint16_t>(0x1000 + i * 0x100);
        sm.length = 0x0080;
        sm.sm_type = types[i];
    }
    d.sm_count = 8;

    // TxPDO with two entries (one named, one anonymous)
    SIIPDO tx;
    tx.pdo_index = 0x1A00;
    tx.sync_manager = 3;
    tx.dc_sync = 1;
    tx.n_entries = 2;
    SIIPDOEntry e1;
    e1.index = 0x6000;
    e1.subindex = 1;
    e1.data_type = 0x0004;
    e1.bit_length = 16;
    SIIPDOEntry e2;
    e2.index = 0x6010;
    e2.subindex = 2;
    e2.data_type = 0x0005;
    e2.bit_length = 32;
    tx.entries = {e1, e2};
    d.tx_pdos.push_back(tx);

    // RxPDO
    SIIPDO rx;
    rx.pdo_index = 0x1600;
    rx.sync_manager = 2;
    rx.n_entries = 1;
    SIIPDOEntry r1;
    r1.index = 0x7000;
    r1.subindex = 1;
    r1.data_type = 0x0004;
    r1.bit_length = 16;
    rx.entries = {r1};
    d.rx_pdos.push_back(rx);

    // FMMUs: every type name plus the unknown fallback
    const uint8_t fmmu_types[] = {0, 1, 2, 3, 99};
    for (size_t i = 0; i < 5; ++i) {
        d.fmmus[i].fmmu_type = fmmu_types[i];
    }
    d.fmmu_count = 5;

    // DC configs: sync0-only, sync1-only, both off
    SIIDCConfig dc0;
    dc0.cycle_time_0 = 1000000;
    dc0.shift_time_0 = 500;
    dc0.assign_activate = 0x0100;  // sync0 on
    SIIDCConfig dc1;
    dc1.cycle_time_0 = 2000000;
    dc1.assign_activate = 0x0200;  // sync1 on
    SIIDCConfig dc2;               // neither
    d.dc_configs = {dc0, dc1, dc2};

    return d;
}

}  // namespace

TEST(SIILoggerCoverage, ProtocolNamesAllBits) {
    // One call per bit exercises every protocol-name arm
    for (uint16_t bit = 0; bit < 6; ++bit) {
        EXPECT_NE(EtherCAT::SII::getMailboxProtocolName(static_cast<uint16_t>(1u << bit)),
                  nullptr);
    }
    EXPECT_NE(EtherCAT::SII::getMailboxProtocolName(0x0000), nullptr);
    EXPECT_NE(EtherCAT::SII::getMailboxProtocolName(0xFFFF), nullptr);
}

TEST(SIILoggerCoverage, LogRichData) {
    SIIData d = makeRichData();
    logSIIData(d, "SII");
    logSIIIdentity(d.identity, "SII");
    logSIIMailbox(d.mailbox, "SII");
    logSIISyncManagers(d, "SII");
    logSIIPDOs(d, "SII");
    logSIISummary(d, "slave0", "SII");
}

TEST(SIILoggerCoverage, LogEmptyData) {
    SIIData empty;
    logSIIData(empty, "SII");
    logSIIMailbox(empty.mailbox, "SII");   // no standard mailbox configured
    logSIISyncManagers(empty, "SII");      // zero sync managers
    logSIIPDOs(empty, "SII");              // no PDOs
    logSIISummary(empty, "slave1", "SII");
}

TEST(SIILoggerCoverage, LogPartialMailboxAndGeneral) {
    SIIData d;
    d.identity.vendor_id = 0x1;
    // only a standard mailbox, no bootstrap
    d.mailbox.std_rx_offset = 0x2000;
    d.mailbox.std_rx_size = 0x0020;
    d.mailbox.std_tx_offset = 0x2020;
    d.mailbox.std_tx_size = 0x0020;
    d.mailbox.protocols = 0x0001;
    d.has_general = false;   // skips the general-info block
    logSIIData(d, "SII");
    logSIIMailbox(d.mailbox, "SII");

    // and the mirror case: bootstrap only
    SIIData b;
    b.mailbox.bootstrap_rx_offset = 0x1000;
    b.mailbox.bootstrap_rx_size = 0x0040;
    b.mailbox.bootstrap_tx_offset = 0x1040;
    b.mailbox.bootstrap_tx_size = 0x0040;
    logSIIMailbox(b.mailbox, "SII");
}

TEST(SIILoggerCoverage, LogSyncManagerTypesIndividually) {
    for (uint8_t t = 0; t < 8; ++t) {
        SIIData d;
        d.sync_managers[0].sm_type = t;
        d.sync_managers[0].phys_start_address = 0x1000;
        d.sync_managers[0].length = 4;
        d.sm_count = 1;
        logSIISyncManagers(d, "SII");
    }
}

TEST(SIILoggerCoverage, LogMultiplePdosWithDcSync) {
    SIIData d;
    for (int i = 0; i < 3; ++i) {
        SIIPDO p;
        p.pdo_index = static_cast<uint16_t>(0x1A00 + i);
        p.sync_manager = 3;
        p.dc_sync = static_cast<uint8_t>(i);  // 0 free-run, 1 sync0, 2 sync1
        p.n_entries = 1;
        SIIPDOEntry e;
        e.index = static_cast<uint16_t>(0x6000 + i);
        e.subindex = 1;
        e.bit_length = 8;
        p.entries = {e};
        d.tx_pdos.push_back(p);
    }
    logSIIPDOs(d, "SII");
    logSIISummary(d, "slave2", "SII");
}
