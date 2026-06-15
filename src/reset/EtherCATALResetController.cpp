/**
 * @file EtherCATALResetController.cpp
 * @brief Implementation of EtherCATALResetController
 */

#include "tether/ethercat/EtherCATALResetController.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "raw/internal.hpp"
#include "tether/platform/EspCompat.hpp"

#include <chrono>
#include <thread>

namespace EtherCAT {

// ============================================================================
// Construction
// ============================================================================

EtherCATALResetController::EtherCATALResetController(EtherCATMaster& master)
    : master_(master), progress_cb_(nullptr) {}

// ============================================================================
// Callback management
// ============================================================================

void EtherCATALResetController::setProgressCallback(ALResetProgressCallback cb) {
    progress_cb_ = std::move(cb);
}

void EtherCATALResetController::clearProgressCallback() {
    progress_cb_ = nullptr;
}

// ============================================================================
// Core reset logic
// ============================================================================

ALResetResult EtherCATALResetController::resetSlave(uint16_t slave_index,
                                                       uint8_t target_state,
                                                       int max_iterations,
                                                       int sleep_ms) {
    ALResetResult result{};
    result.slave_index = slave_index;
    result.target_state = target_state;
    result.max_iterations = max_iterations;

    if (max_iterations < 1) {
        result.message = "max_iterations must be >= 1";
        return result;
    }
    if (sleep_ms < 0) {
        result.message = "sleep_ms must be >= 0";
        return result;
    }

    bool reached = false;
    uint16_t last_al = 0;
    uint16_t last_code = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Read AL_STATUS
        uint16_t al_le = 0;
        bool read_ok = master_.readRegister(SlaveAddress(slave_index),
                                              Raw::EC_REG_AL_STATUS,
                                              al_le, 200);

        // Read AL_STATUS_CODE
        uint16_t code_le = 0;
        bool code_ok = master_.readRegister(SlaveAddress(slave_index),
                                            Raw::EC_REG_AL_STATUS_CODE,
                                            code_le, 200);

        if (read_ok) {
            const uint16_t al = Raw::le16_to_host(al_le);
            last_al = al;
            last_code = code_ok ? Raw::le16_to_host(code_le) : 0;

            const uint8_t state = al & 0x0Fu;
            const bool has_err = (al & 0x10u) != 0;

            // Notify progress callback
            if (progress_cb_) {
                progress_cb_(slave_index, iter + 1, max_iterations,
                             last_al, last_code,
                             (state == target_state && !has_err));
            }

            if (state == target_state && !has_err) {
                reached = true;
                result.iterations_used = iter + 1;
                break;
            }
        } else {
            // Even on read failure, notify callback with zeros
            if (progress_cb_) {
                progress_cb_(slave_index, iter + 1, max_iterations,
                             0, 0, false);
            }
        }

        // Step 1: write target state with error ack = 0
        const uint16_t req_clear = target_state;
        master_.writeRegister(SlaveAddress(slave_index),
                              Raw::EC_REG_AL_CONTROL, req_clear);

        // Step 2: write target state with error ack = 1
        const uint16_t req_ack = static_cast<uint16_t>(target_state | 0x10u);
        master_.writeRegister(SlaveAddress(slave_index),
                              Raw::EC_REG_AL_CONTROL, req_ack);

        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    if (!reached) {
        result.iterations_used = max_iterations;
    }

    result.success = reached;
    result.final_al_status = last_al;
    result.final_al_status_code = last_code;

    if (reached) {
        result.message = "Target state reached after " +
                         std::to_string(result.iterations_used) + " iteration(s)";
    } else {
        result.message = "Failed to reach target state after " +
                         std::to_string(max_iterations) + " iteration(s)";
    }

    return result;
}

} // namespace EtherCAT
