#pragma once

/**
 * @file DCManager.hpp
 * @brief Instance-based Distributed Clock manager for EtherCAT master
 */

#include <memory>
#include <functional>
#include "tether/ethercat/DC.hpp"        // For DC:: types
#include "tether/ethercat/DCClass.hpp"   // For EtherCATDC class
#include "tether/ethercat/RawDCTransport.hpp"    // For RawDCTransport

namespace EtherCAT {

// Forward declaration
class Master;

/**
 * @brief Instance-based Distributed Clock manager for Master.
 * 
 * DCManager wraps an optional EtherCATDC instance owned by the master.
 * After calling init(), all DC operations are forwarded to the underlying
 * class-based DC instance managed by this object.
 */
class DCManager {
public:
    explicit DCManager(Master& master);
    ~DCManager();

    // init needs eth_handle + src_mac → out-of-line
    bool init(const DC::DCConfig& config, uint16_t slave_count = 1);

    bool start();
    bool start(std::function<bool()> pdo_exchange_fn);
    void stop();

    DC::DCState getState() const;
    DC::DCLoopStats getStats() const;

    void forceSync();
    void setPDOEnabled(bool en);
    bool reconfigureSync(uint16_t slave_index);

    /* Convenience passthroughs to the underlying EtherCATDC instance. */
    bool readRegister(uint16_t slave_index, DCRegisters reg, void* data, uint16_t size,
                      unsigned int timeout_ms = 200) { return get()->readRegister(slave_index, reg, data, size, timeout_ms); }
    bool writeRegister(uint16_t slave_index, DCRegisters reg, const void* data, uint16_t size,
                       unsigned int timeout_ms = 200) { return get()->writeRegister(slave_index, reg, data, size, timeout_ms); }

    /** @return true if DC has been initialized */
    bool isInitialized() const { return dc_instance_ != nullptr; }

    /**
     * @brief Always-returning pointer to a DC implementation.
     *
     * When DC is not configured this returns an instance of
     * `NoDistributedClockConfigured` which logs a CRITICAL error on use.
     */
    EtherCATDC* get() { return dc_instance_ ? dc_instance_.get() : sentinel_.get(); }
    const EtherCATDC* get() const { return dc_instance_ ? dc_instance_.get() : sentinel_.get(); }

    Master& master() { return master_; }

private:
    Master& master_;
    std::unique_ptr<RawDCTransport> transport_;
    std::unique_ptr<EtherCATDC> dc_instance_;
    std::unique_ptr<EtherCATDC> sentinel_;
};

} // namespace EtherCAT
