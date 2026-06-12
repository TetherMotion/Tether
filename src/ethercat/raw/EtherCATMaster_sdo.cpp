/**
 * @file EtherCATMaster_sdo.cpp
 * @brief EtherCATMaster — CoE/SDO transport and test hooks
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATSlave.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
#include "tether/ethercat/EtherCATPDO.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/ethercat/EtherCATFoE.hpp"
#include "tether/ethercat/EtherCATVoE.hpp"
#include "tether/ethercat/EtherCATEoE.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/EtherCATRealtimeLoop.hpp"
#include "tether/ethercat/SyncManagerValidation.hpp"
#include "tether/sii/SIIParser.hpp"
#include "tether/fmmu/FMMUConfiguration.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "sii/SIIReader.hpp"
#include <inttypes.h>

namespace EtherCAT {

static const char* TAG = "ethercat";

// Global debug flag for ethercat-statemachine (shared with EtherCATSlave)
extern bool g_debug_statemachine;

// Global debug flags for tx/rx packet logging (shared with EtherCATSlave)
extern bool g_debug_tx_packets;
extern bool g_debug_rx_packets;

// Global debug flags for PDO logging (shared with PDOManager)
extern bool g_debug_rx_pdo;
extern bool g_debug_tx_pdo;

// ============================================================================
// MasterSDOTransport — adapts EtherCATMaster to ISDOTransport  
// ============================================================================

class EtherCATMaster::MasterSDOTransport : public ::EtherCAT::SDO::ISDOTransport {
public:
    explicit MasterSDOTransport(EtherCATMaster& master, ::EtherCAT::SDO::SDOManager* mgr = nullptr)
        : master_(master), mgr_(mgr) {}

    void setManager(::EtherCAT::SDO::SDOManager* mgr) { mgr_ = mgr; }

    bool sdoUpload(uint16_t slave_index, uint8_t* mbx_counter,
                   uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                   uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                   uint16_t index, uint8_t sub,
                   uint8_t* out, size_t out_cap, size_t* out_len) override
    {
        uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
        bool diag = (mgr_ && mgr_->isDiagEnabled());
        return master_.coeSdoUpload(adp, mbx_counter,
                                    mbx_wr_addr, mbx_wr_len,
                                    mbx_rd_addr, mbx_rd_len,
                                    index, sub, out, out_cap, out_len, diag);
    }

    bool sdoDownload(uint16_t slave_index, uint8_t* mbx_counter,
                     uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                     uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                     uint16_t index, uint8_t sub,
                     const uint8_t* data, size_t data_len) override
    {
        uint16_t adp = EtherCATMaster::adpForSlaveIndex(slave_index);
        bool diag = (mgr_ && mgr_->isDiagEnabled());
        return master_.coeSdoDownload(adp, mbx_counter,
                                      mbx_wr_addr, mbx_wr_len,
                                      mbx_rd_addr, mbx_rd_len,
                                      index, sub, data, data_len, diag);
    }

    uint64_t getMicroseconds() override {
        return static_cast<uint64_t>(Tether::Platform::Clock::instance().getMicroseconds());
    }

private:
    EtherCATMaster& master_;
    ::EtherCAT::SDO::SDOManager* mgr_;
};

} // namespace EtherCAT
