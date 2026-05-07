#pragma once
#include <cstdint>

namespace HAL {

/**
 * @brief Abstract interface for a periodic hardware timer
 */
class IPeriodicTimer {
public:
    virtual ~IPeriodicTimer() = default;

    /**
     * @brief Initialize the timer
     * @param frequencyHz Frequency in Hz (e.g. 1000 for 1ms)
     * @return true on success
     */
    virtual bool init(uint32_t frequencyHz) = 0;

    /**
     * @brief Start the timer
     */
    virtual void start() = 0;

    /**
     * @brief Stop the timer
     */
    virtual void stop() = 0;

    /**
     * @brief Wait for the next cycle
     * This method blocks the calling task until the next timer tick occurs.
     */
    virtual void waitForCycle() = 0;
};

}
