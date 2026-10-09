/**
 * @file Slave_mailbox.cpp
 * @brief Slave — mailbox configuration and drain.
 *
 * TU split out of Slave.cpp.
 */

#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "SlaveESIHelpers.hpp"
#include "raw/RawConstants.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/ethercat/SyncManager.hpp"
#include "tether/sii/SIIReader.hpp"

#include <cstring>
#include <bit>
#include <format>

namespace EtherCAT {

static const char* TAG = "Slave";

// -- Mailbox configuration ---------------------------------------------------

SlaveError Slave::configureMailbox(Tether::Platform::LogLevel log_level) {
    if (no_mailbox_) {
        TETHER_LOGE( TAG,
            "{}: slave was declared mailbox-less via markNoMailbox() — "
            "refusing configureMailbox()", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    if (!master_->autoConfigureMailbox(index_, log_level)) {
        TETHER_LOGE( TAG,
            "{}: Failed to auto-configure mailbox from SII", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Mailbox configured from SII", logPrefix().c_str());
    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}

SlaveError Slave::configureMailbox(
    const MailboxSyncManagerConfig& mbox_out,
    const MailboxSyncManagerConfig& mbox_in,
    uint16_t protocols)
{
    if (no_mailbox_) {
        TETHER_LOGE( TAG,
            "{}: slave was declared mailbox-less via markNoMailbox() — "
            "refusing configureMailbox()", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }
    master_->setMailboxOverride(index_,
                               mbox_in.address, mbox_in.length,
                               mbox_out.address, mbox_out.length,
                               protocols);
    // Configure SDO manager with these mailbox params
    master_->sdoManager(index_).configureMailbox(
        mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length);

    // Write mailbox SM registers to slave ESC (same as autoConfigureMailbox)
    auto& pdo = master_->pdoForSlave(index_);
    auto* slave_configs = pdo.slaveConfigs();
    if (index_ < PDO::kMaxPDOSlaves) {
        slave_configs[index_].sm[0] = PDO::SyncManagerConfig::mailbox_write(
            mbox_in.address, mbox_in.length);
        slave_configs[index_].sm[1] = PDO::SyncManagerConfig::mailbox_read(
            mbox_out.address, mbox_out.length);
        if (!pdo.configureSlavesSMs(index_)) {
            TETHER_LOGE(TAG, "{}: Failed to write mailbox SM registers", logPrefix().c_str());
            return SlaveError::MailboxConfigFailed;
        }

        // SM1 may contain stale/junk data left over from slave firmware boot.
        // Drain it now so the slave has a free outbound mailbox before the
        // first SDO exchange.
        (void)master_->drainSlaveMailbox(index_);
    }

    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Mailbox configured (wr=0x{:04X}/{}, rd=0x{:04X}/{}, proto=0x{:04X})",
        logPrefix().c_str(), mbox_in.address, mbox_in.length,
        mbox_out.address, mbox_out.length, protocols);
    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
    return SlaveError::Ok;
}


SlaveError Slave::configureMailbox(
    const ESIFile& esi,
    Tether::Platform::LogLevel log_level)
{
    if (esi.empty()) {
        TETHER_LOGE(TAG, "{}: ESI file is empty — cannot configure mailbox from ESI", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }

    // Match device by SII identity
    auto id = readIdentityForESIMatch(*master_, index_);
    const ESI::DeviceInfo* dev = esi.findDevice(id.vendorId, id.productCode);
    if (!dev) {
        TETHER_LOGE(TAG, "{}: ESI file has no devices", logPrefix().c_str());
        return SlaveError::MailboxConfigFailed;
    }

    if (log_level >= Tether::Platform::LogLevel::Debug) {
        TETHER_LOGD(TAG, "{}: ESI device '{}' matched (vendor=0x{:08X} product=0x{:08X})",
                    logPrefix().c_str(), dev->name.c_str(), dev->vendorId, dev->productCode);
    }

    // Find MBoxOut (master→slave write, SM0) and MBoxIn (slave→master read, SM1)
    const ESI::SyncManagerEntry* mbxOut = findSmByName(*dev, "MBoxOut");
    const ESI::SyncManagerEntry* mbxIn  = findSmByName(*dev, "MBoxIn");

    MailboxSyncManagerConfig mbox_out{};
    MailboxSyncManagerConfig mbox_in{};

    if (mbxOut) {
        mbox_out.address = mbxOut->startAddress;
        mbox_out.length  = mbxOut->defaultSize;
    } else {
        TETHER_LOGW(TAG, "{}: ESI has no MBoxOut sync manager — using defaults", logPrefix().c_str());
        mbox_out.address = 0x1000;
        mbox_out.length  = 256;
    }

    if (mbxIn) {
        mbox_in.address = mbxIn->startAddress;
        mbox_in.length  = mbxIn->defaultSize;
    } else {
        TETHER_LOGW(TAG, "{}: ESI has no MBoxIn sync manager — using defaults", logPrefix().c_str());
        mbox_in.address = 0x1200;
        mbox_in.length  = 256;
    }

    // Protocol flags
    uint16_t protocols = 0x0004; // default: CoE
    if (dev->mailbox.protocols.has_value()) {
        protocols = *dev->mailbox.protocols;
    }

    TETHER_LOGI(TAG, "{}: Configuring mailbox from ESI (out=0x{:04X}/{}, in=0x{:04X}/{}, proto=0x{:04X})",
                logPrefix().c_str(), mbox_out.address, mbox_out.length,
                mbox_in.address, mbox_in.length, protocols);

    return configureMailbox(mbox_out, mbox_in, protocols);
}

void Slave::assumeMailboxAlreadyConfigured() {
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Assuming mailbox already configured", logPrefix().c_str());

    // Best-effort drain: the slave firmware may have left stale data in SM1
    // from boot or a previous session.  If the CoE subsystem doesn't have
    // mailbox address info (common when the firmware truly pre-configured
    // everything), the drain simply logs a warning and continues.
    if (!drainMailbox()) {
        TETHER_LOGW(TAG,
            "{}: Mailbox drain after assumeMailboxAlreadyConfigured() "
            "did not complete — stale responses may occur on first SDO exchange",
            logPrefix().c_str());
    }

    // Debug gate checkpoint: mailbox configured
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
}

void Slave::markNoMailbox() {
    no_mailbox_ = true;
    // The PRE_OP prerequisite is vacuously satisfied — there is no
    // mailbox to configure or drain.
    mailbox_configured_ = true;
    TETHER_LOGI( TAG,
        "{}: Declared mailbox-less — skipping all mailbox handling",
        logPrefix().c_str());
    master_->debugGate().notifyCheckpoint("mailbox-configured", index_);
}

bool Slave::drainMailbox(unsigned int max_drain) {
    if (no_mailbox_) return true;   // nothing to drain
    return master_->drainSlaveMailbox(index_, max_drain);
}

} // namespace EtherCAT

