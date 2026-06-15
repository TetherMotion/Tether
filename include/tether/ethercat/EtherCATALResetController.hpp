#pragma once

/**
 * @file EtherCATALResetController.hpp
 * @brief Application-Layer (AL) reset controller for EtherCAT slaves
 *
 * Encapsulates the two-step AL reset loop: write target state with error-ack
 * clear, then write target state with error-ack set, until the slave reports
 * the target state with no error bit, or the iteration limit is reached.
 */

#include <cstdint>
#include <functional>
#include <string>

namespace EtherCAT {

class EtherCATMaster;

// ============================================================================
// AL Reset Result
// ============================================================================

/**
 * @brief Per-slave result of an AL reset operation
 */
struct ALResetResult {
    bool success = false;          ///< True if target state reached without error
    uint16_t slave_index = 0;      ///< Slave index that was reset
    uint8_t target_state = 0;      ///< Requested target ESM state
    int iterations_used = 0;       ///< Number of iterations actually performed
    int max_iterations = 0;        ///< Max iterations that were allowed
    uint16_t final_al_status = 0;  ///< Final AL_STATUS value (host byte order)
    uint16_t final_al_status_code = 0; ///< Final AL_STATUS_CODE (host byte order)
    std::string message;           ///< Human-readable summary
};

// ============================================================================
// Progress Callback
// ============================================================================

/**
 * @brief Callback invoked once per iteration (after the read, before the writes)
 *
 * @param slave_index   Slave being reset
 * @param iteration     Current iteration number (1-based)
 * @param max_iterations Max iterations configured
 * @param al_status     Current AL_STATUS value (host byte order)
 * @param al_status_code Current AL_STATUS_CODE (host byte order)
 * @param state_reached True if the target state has been reached this iteration
 */
using ALResetProgressCallback = std::function<void(
    uint16_t slave_index,
    int iteration,
    int max_iterations,
    uint16_t al_status,
    uint16_t al_status_code,
    bool state_reached)>;

// ============================================================================
// EtherCATALResetController
// ============================================================================

/**
 * @brief AL reset controller for EtherCAT slaves
 *
 * Performs a bounded two-step AL reset on a single slave until the requested
 * target ESM state is reached and the error indicator (bit 4) is clear.
 *
 * The two-step write sequence per iteration:
 *   1. AL_CONTROL = target_state              (error ack = 0)
 *   2. AL_CONTROL = target_state | 0x10       (error ack = 1)
 *
 * @code
 *   EtherCAT::EtherCATALResetController ctrl(master);
 *   ctrl.setProgressCallback([](...){ ... });
 *   auto result = ctrl.resetSlave(0, 0x01, 50, 50); // slave 0 -> INIT
 * @endcode
 */
class EtherCATALResetController {
public:
    /**
     * @brief Construct controller bound to an EtherCATMaster instance
     * @param master Reference to the active EtherCATMaster
     */
    explicit EtherCATALResetController(EtherCATMaster& master);

    /**
     * @brief Reset a single slave to the target ESM state
     *
     * @param slave_index    Zero-based slave index on the bus
     * @param target_state   Target ESM state (e.g. 0x01 = INIT)
     * @param max_iterations Maximum reset attempts (must be >= 1)
     * @param sleep_ms       Delay between iterations in ms (may be 0)
     * @return ALResetResult with full outcome details
     */
    ALResetResult resetSlave(uint16_t slave_index,
                              uint8_t target_state,
                              int max_iterations = 50,
                              int sleep_ms = 50);

    /**
     * @brief Set a progress callback invoked each iteration
     *
     * The callback is called after reading AL_STATUS/AL_STATUS_CODE and
     * before writing AL_CONTROL. It receives the current values and
     * whether the target has already been reached (in which case no
     * further writes will occur).
     */
    void setProgressCallback(ALResetProgressCallback cb);

    /**
     * @brief Remove the progress callback
     */
    void clearProgressCallback();

private:
    EtherCATMaster& master_;
    ALResetProgressCallback progress_cb_;
};

} // namespace EtherCAT
