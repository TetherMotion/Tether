/**
 * @file RawDCTransport.cpp
 * @brief IDCTransport implementation using Raw:: EtherCAT frame functions
 */

#include "tether/ethercat/RawDCTransport.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/platform/EspCompat.hpp"

// Declaration for the weak time-source symbol (defined in dc_time_source.cpp)
extern "C" uint64_t ecdc_get_master_time_ns();

namespace EtherCAT {

RawDCTransport::RawDCTransport(Master& master)
    : master_(master)
{}

bool RawDCTransport::readRegister(uint16_t slave_index, uint16_t reg_addr,
                                   void* data, uint16_t size,
                                   unsigned int timeout_ms)
{
    return master_.readRegister(EtherCAT::SlaveAddress(slave_index), reg_addr, data, size, timeout_ms);
}

bool RawDCTransport::writeRegister(uint16_t slave_index, uint16_t reg_addr,
                                    const void* data, uint16_t size,
                                    unsigned int timeout_ms)
{
    return master_.writeRegister(EtherCAT::SlaveAddress(slave_index), reg_addr, data, size, timeout_ms);
}

bool RawDCTransport::sendSyncDatagram(uint16_t slave_index, uint16_t reg_addr,
                                       const void* data, uint16_t size)
{
    // Broadcast Write reaches every slave in a single frame. Each DC
    // slave between the master and the end of the chain evaluates the
    // transmitted value against its local clock, compensated by its own
    // programmed transmission delay (0x0928). The slave_index argument
    // is kept for interface compatibility only.
    (void)slave_index;
    // Use kFireAndForgetIdx so the response is handled as fire-and-forget
    // and doesn't flood the packet router with "unrouted" warnings.
    return master_.sendSingleDatagram(
        Command::BWR,
        Master::kFireAndForgetIdx,
        0,
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
