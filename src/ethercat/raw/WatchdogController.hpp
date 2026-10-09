/**
 * @file WatchdogController.hpp
 * @brief Watchdog register access for EtherCAT slaves.
 *
 * @internal Internal header — not installed, not part of the public API.
 *
 * Owns the WD_DIV / WD_TIME_PDI / WD_TIME_PDATA configuration frame and the
 * contiguous watchdog status + PDI/PDATA counter block.  Stateless beyond the
 * Master reference it borrows for register access; Master keeps one instance
 * and forwards its public watchdog API, and Slave forwards through Master.
 */

#pragma once

#include <cstdint>

#include "tether/ethercat/Master.hpp"

namespace EtherCAT {

class WatchdogController {
public:
    explicit WatchdogController(Master& master) : master_(master) {}

    /// Arm both watchdogs in one frame (three APWR datagrams — the registers
    /// are not contiguous).  A zero timeout disables that watchdog.
    bool configure(SlaveAddress slave_address,
                   uint16_t pdi_timeout_100us,
                   uint16_t pdata_timeout_100us);

    /// Disarm both watchdogs.
    bool disable(SlaveAddress slave_address) {
        return configure(slave_address, 0, 0);
    }

    /// Read WD_STATUS (0x0440) plus the PDI/PDATA counters (0x0442/0x0443)
    /// in a single contiguous read.
    bool readStatus(SlaveAddress slave_address,
                    uint8_t& wd_status,
                    uint8_t& pdi_cnt,
                    uint8_t& pdata_cnt);

private:
    Master& master_;
};

} // namespace EtherCAT
