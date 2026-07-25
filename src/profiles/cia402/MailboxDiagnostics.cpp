/**
 * @file MailboxDiagnostics.cpp
 * @brief Implementation of EtherCAT::Mailbox::Utils::dumpSlaveSyncAndMailboxInfo.
 *
 * This function is declared in tether/ethercat/Mailbox/Utils.hpp (with a
 * forward-declared CiA402Drive) but implemented here in tether_cia_profiles to
 * avoid a circular link dependency between the EtherCAT master core and the
 * CiA profile library.
 */

#include "tether/ethercat/Mailbox/Utils.hpp"

#include "logging/Logger.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/SyncManager/Utils.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"

namespace EtherCAT {
namespace Mailbox {
namespace Utils {

void dumpSlaveSyncAndMailboxInfo(const CiA402Drive& drive,
                                 const char* tag)
{
    if (!drive.master()) {
        TETHER_LOGW(tag, "dumpSlaveSyncAndMailboxInfo: drive has no master");
        return;
    }

    auto* master = drive.master();
    uint16_t slave_idx = drive.slaveIndex();

    // first dump mailbox header / status registers
    dumpHeaderAndStatus(*master, slave_idx, tag);

    // dump SM0/SM1 register values (same as sync-manager utils do internally)
    for (uint8_t sm = 0; sm < 2; ++sm) {
        master->slave(slave_idx).sm(sm).dump(tag);
    }

    // read and log AL status + code
    uint8_t ast[2] = {0};
    uint8_t acd[2] = {0};
    (void)master->readRegister(SlaveAddress(slave_idx), static_cast<uint16_t>(0x0130), ast, sizeof(ast), 200);
    (void)master->readRegister(SlaveAddress(slave_idx), static_cast<uint16_t>(0x0134), acd, sizeof(acd), 200);
    const uint16_t al_status = static_cast<uint16_t>(ast[0] | (ast[1] << 8));
    const uint16_t al_code   = static_cast<uint16_t>(acd[0] | (acd[1] << 8));
    TETHER_LOGI(tag, "AL_STATUS=0x%04X (%s)%s | AL status code: %s (0x%04X)",
             al_status, al_status_get_state_name(al_status),
             (al_status_has_error(al_status) ? ", ERROR" : ""),
             getALStatusCodeName(static_cast<ALStatusCode>(al_code)), al_code);
}

} // namespace Utils
} // namespace Mailbox
} // namespace EtherCAT
