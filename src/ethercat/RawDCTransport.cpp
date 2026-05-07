/**
 * @file RawDCTransport.cpp
 * @brief IDCTransport implementation using Raw:: EtherCAT frame functions
 */

#include "tether/ethercat/RawDCTransport.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/platform/EspCompat.hpp"

// Declaration for the weak time-source symbol (defined in dc_time_source.cpp)
extern "C" uint64_t ecdc_get_master_time_ns();

namespace EtherCAT {

RawDCTransport::RawDCTransport(EtherCATMaster& master)
    : master_(master)
{}

bool RawDCTransport::readRegister(uint16_t slave_index, uint16_t reg_addr,
                                   void* data, uint16_t size,
                                   unsigned int timeout_ms)
{
    const uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
    return master_.readRegister(adp, reg_addr, data, size, timeout_ms);
}

bool RawDCTransport::writeRegister(uint16_t slave_index, uint16_t reg_addr,
                                    const void* data, uint16_t size,
                                    unsigned int timeout_ms)
{
    const uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
    return master_.writeRegister(adp, reg_addr, data, size, timeout_ms);
}

bool RawDCTransport::sendSyncDatagram(uint16_t slave_index, uint16_t reg_addr,
                                       const void* data, uint16_t size)
{
    const uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
    // Use kFireAndForgetIdx so the response is handled as fire-and-forget
    // and doesn't flood the packet router with "unrouted" warnings.
    return master_.sendSingleDatagram(
        Command::ARMW,
        EtherCATMaster::kFireAndForgetIdx,
        adp,
        reg_addr,
        data,
        size,
        false
    );
}

uint64_t RawDCTransport::getMasterTimeNs()
{
    return ecdc_get_master_time_ns();
}

void RawDCTransport::delayMs(uint32_t ms)
{
    Tether::Platform::Clock::instance().delayMilliseconds(ms);
}

} // namespace EtherCAT
