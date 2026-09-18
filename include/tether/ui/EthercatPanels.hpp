#pragma once

/**
 * @file EthercatPanels.hpp
 * @brief Standard adapters from a live EtherCAT::Master to the
 *        entity-browser panel models.
 *
 *  - MasterEntityProvider — polls every discovered slave's AL state
 *    (from the SlaveStatusPoller cache when it runs, else via the slave's
 *    own readState), AL status code, error flag, supervisor suspension
 *    and an optional app-provided detail string.
 *  - SdoRegisterReader — reads a NamedRegister via CoE SDO.  Pass a
 *    mutex when SDO access must be serialized against other users
 *    (e.g. the blackchannel's sdo_mutex).  SDO reads are only issued
 *    when the panel calls read() — i.e. on user request — never in the
 *    cyclic path.
 *
 * Header-only so applications and a future web backend can share it.
 */

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/ui/Panels.hpp"

namespace Tether {
namespace UI {

class MasterEntityProvider : public ISlaveEntityProvider {
public:
    /// `mutex` optionally guards every master access (SDO/register path
    /// shared with other threads).  `detail_fn(i)` supplies the trailing
    /// free-form column (e.g. CiA-402 status, FSoE channel state).
    explicit MasterEntityProvider(EtherCAT::Master& master,
                                  std::mutex* mutex = nullptr)
        : master_(master), mutex_(mutex) {}

    /// Optional per-entity detail column callback.
    std::function<std::string(uint16_t)> detail_fn;

    std::vector<SlaveEntity> entities() override {
        std::vector<SlaveEntity> out;
        std::unique_lock<std::mutex> lk;
        if (mutex_) lk = std::unique_lock{*mutex_};
        const uint16_t n = master_.getDiscoveredSlaveCount();
        out.reserve(n);
        auto& poller = master_.statusPoller();
        auto& supervisor = master_.slaveSupervisor();
        for (uint16_t i = 0; i < n; ++i) {
            auto& s = master_.slave(i);
            SlaveEntity e;
            e.index = i;
            e.name = std::string{s.name()};
            std::optional<EtherCAT::SlaveState> st;
            if (poller.isRunning()) {
                st = poller.getSlaveState(i);
            } else {
                st = s.ALState();
            }
            e.state = st ? EtherCAT::slaveStateToString(*st) : "?";
            uint16_t code = 0;
            if (s.readALStatusCode(code) == EtherCAT::SlaveError::Ok)
                e.status_code = code;
            e.error = e.status_code != 0;
            e.suspended = supervisor.isSlaveSuspended(i);
            if (detail_fn) e.detail = detail_fn(i);
            out.push_back(std::move(e));
        }
        return out;
    }

private:
    EtherCAT::Master& master_;
    std::mutex* mutex_;
};

class SdoRegisterReader : public IRegisterReader {
public:
    explicit SdoRegisterReader(EtherCAT::Master& master,
                               std::mutex* mutex = nullptr)
        : master_(master), mutex_(mutex) {}

    RegisterResult read(uint16_t entity_index,
                        const NamedRegister& reg) override {
        std::unique_lock<std::mutex> lk;
        if (mutex_) lk = std::unique_lock{*mutex_};
        auto& s = master_.slave(entity_index);
        EtherCAT::SlaveError err;
        RegisterResult r;
        if (reg.bits == 8) {
            uint8_t v = 0;
            err = s.sdoReadU8(reg.index, reg.subindex, v);
            if (ok(err)) r.value = fmt("0x%02X (%u)", v, v);
        } else if (reg.bits == 16) {
            uint16_t v = 0;
            err = s.sdoReadU16(reg.index, reg.subindex, v);
            if (ok(err)) r.value = fmt("0x%04X (%u)", v, v);
        } else if (reg.bits == 32) {
            uint32_t v = 0;
            err = s.sdoReadU32(reg.index, reg.subindex, v);
            if (ok(err)) r.value = fmt("0x%08X (%u)", v, v);
        } else {
            // Unknown size — raw upload up to 64 B; show hex + ASCII.
            uint8_t buf[64];
            size_t size = sizeof(buf);
            err = s.sdoRead(reg.index, reg.subindex, buf, size);
            if (ok(err)) r.value = rawString(buf, size);
        }
        if (ok(err)) {
            r.ok = true;
        } else {
            r.error = EtherCAT::slaveErrorToString(err);
            const uint32_t abort = s.lastSdoAbortCode();
            if (abort) {
                char b[16];
                snprintf(b, sizeof(b), " (0x%08X)", abort);
                r.error += b;
            }
        }
        return r;
    }

private:
    static bool ok(EtherCAT::SlaveError e) {
        return e == EtherCAT::SlaveError::Ok;
    }
    template <typename... A>
    static std::string fmt(const char* f, A... a) {
        char b[64];
        snprintf(b, sizeof(b), f, a...);
        return b;
    }
    static std::string rawString(const uint8_t* d, size_t n) {
        std::string out;
        bool printable = n > 0;
        for (size_t i = 0; i < n; ++i)
            if (d[i] < 0x20 || d[i] > 0x7E) printable = false;
        if (printable) return "\"" + std::string(d, d + n) + "\"";
        out = "0x";
        for (size_t i = 0; i < n; ++i) {
            char b[4];
            snprintf(b, sizeof(b), "%02X", d[i]);
            out += b;
        }
        return out;
    }

    EtherCAT::Master& master_;
    std::mutex* mutex_;
};

} // namespace UI
} // namespace Tether
