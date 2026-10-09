/**
 * @file WatchdogController.cpp
 * @brief Watchdog register access for EtherCAT slaves.
 */

#include "raw/WatchdogController.hpp"
#include "raw/internal.hpp"

#include <vector>

namespace EtherCAT {

bool WatchdogController::configure(SlaveAddress slave_address,
                                   uint16_t pdi_timeout_100us,
                                   uint16_t pdata_timeout_100us)
{
    // One frame, three APWR datagrams (the three registers are not
    // contiguous so a single block write won't work).
    const uint16_t wd_div = Raw::host_to_le16(0x09C2);
    const uint16_t pdi    = Raw::host_to_le16(pdi_timeout_100us);
    const uint16_t pdata  = Raw::host_to_le16(pdata_timeout_100us);
    const SlaveAddress addrs[3] = {slave_address, slave_address, slave_address};
    const uint16_t regs[3] = {Raw::EC_REG_WD_DIV, Raw::EC_REG_WD_TIME_PDI,
                              Raw::EC_REG_WD_TIME_PDATA};
    const void* const data[3] = {&wd_div, &pdi, &pdata};
    const uint16_t lens[3] = {2, 2, 2};

    auto batch = master_.writeRegistersBatch(addrs, regs, data, lens, 3);
    std::vector<BatchReadResult> results;
    if (batch.count() != 3 || !batch.waitAll(200, results)) return false;
    for (size_t i = 0; i < 3; ++i) {
        if (!results[i].success) return false;
    }
    return true;
}

bool WatchdogController::readStatus(SlaveAddress slave_address,
                                    uint8_t& wd_status,
                                    uint8_t& pdi_cnt,
                                    uint8_t& pdata_cnt)
{
    // 0x0440 (status) .. 0x0443 (PD counter) are contiguous — one read.
    uint8_t buf[4] = {};
    if (!master_.readRegister(slave_address, Raw::EC_REG_WD_STATUS, buf,
                              sizeof(buf), 200)) return false;
    wd_status = buf[0];
    pdi_cnt   = buf[2];
    pdata_cnt = buf[3];
    return true;
}

} // namespace EtherCAT
