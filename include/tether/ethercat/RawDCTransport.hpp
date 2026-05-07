#pragma once

/**
 * @file RawDCTransport.hpp
 * @brief IDCTransport implementation backed by Raw EtherCAT frame API
 *
 * Adapts the Raw::ec_aprd / ec_apwr / send_single_datagram functions
 * into the IDCTransport interface so EtherCATDC can be used with the
 * existing low-level frame transport.
 */

#include "tether/ethercat/IDCTransport.hpp"
#include <cstdint>

namespace EtherCAT {

class EtherCATMaster;

/**
 * @brief IDCTransport backed by EtherCATMaster transport primitives
 *
 * Translates slave_index to auto-increment address via
 * EtherCATMaster::adpForSlaveIndex().
 */
class RawDCTransport final : public IDCTransport {
public:
    /**
     * @brief Construct a RawDCTransport
     *
     * @param master  EtherCATMaster instance (must outlive this)
     */
    explicit RawDCTransport(EtherCATMaster& master);

    ~RawDCTransport() override = default;

    // IDCTransport interface
    bool readRegister(uint16_t slave_index, uint16_t reg_addr,
                      void* data, uint16_t size,
                      unsigned int timeout_ms = 200) override;

    bool writeRegister(uint16_t slave_index, uint16_t reg_addr,
                       const void* data, uint16_t size,
                       unsigned int timeout_ms = 200) override;

    bool sendSyncDatagram(uint16_t slave_index, uint16_t reg_addr,
                          const void* data, uint16_t size) override;

    uint64_t getMasterTimeNs() override;

    void delayMs(uint32_t ms) override;

private:
    EtherCATMaster& master_;
};

} // namespace EtherCAT
