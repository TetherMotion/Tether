/**
 * @file LinuxClock.cpp
 * @brief Linux clock and timer HAL implementation
 */

#ifdef __linux__

#include "hal/IClock.hpp"
#include "hal/IThreading.hpp"
#include "hal/HALTypes.hpp"

#include <time.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <atomic>
#include <cstring>
#include <cerrno>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Linux Clock Implementation
// ============================================================================

class LinuxClock : public IClock {
public:
    Timestamp nowMicros() override {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<Timestamp>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    }

    Timestamp nowNanos() override {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<Timestamp>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
    }

    Timestamp systemTimeMillis() override {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return static_cast<Timestamp>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
    }

    Nanoseconds resolution() override {
        struct timespec res;
        clock_getres(CLOCK_MONOTONIC, &res);
        return static_cast<Nanoseconds>(res.tv_sec) * 1000000000 + res.tv_nsec;
    }

    void delayMicros(Microseconds us) override {
        if (us <= 0) return;

        // For short delays, use busy-wait
        if (us < 100) {
            Timestamp start = nowNanos();
            Nanoseconds target = us * 1000;
            while ((nowNanos() - start) < static_cast<Timestamp>(target)) {
                // Busy wait
            }
            return;
        }

        // For longer delays, use nanosleep
        struct timespec ts;
        ts.tv_sec = us / 1000000;
        ts.tv_nsec = (us % 1000000) * 1000;
        while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
            // Retry if interrupted
        }
    }

    void delayMillis(Milliseconds ms) override {
        if (ms <= 0) return;
        usleep(ms * 1000);
    }
};

// ============================================================================
// Linux Periodic Timer Implementation (using timerfd)
// ============================================================================

class LinuxPeriodicTimer : public IPeriodicTimer {
public:
    LinuxPeriodicTimer() = default;

    ~LinuxPeriodicTimer() override {
        stop();
        if (m_timerFd >= 0) {
            close(m_timerFd);
        }
    }

    bool init(uint32_t frequencyHz) override {
        if (frequencyHz == 0) return false;

        m_periodUs = 1000000 / frequencyHz;
        
        m_timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (m_timerFd < 0) {
            return false;
        }

        return true;
    }

    void start() override {
        if (m_timerFd < 0 || m_running) return;

        struct itimerspec its;
        its.it_value.tv_sec = m_periodUs / 1000000;
        its.it_value.tv_nsec = (m_periodUs % 1000000) * 1000;
        its.it_interval = its.it_value;

        if (timerfd_settime(m_timerFd, 0, &its, nullptr) == 0) {
            m_running = true;
            m_stats.tickCount = 0;
        }
    }

    void stop() override {
        if (!m_running) return;

        struct itimerspec its;
        memset(&its, 0, sizeof(its));
        timerfd_settime(m_timerFd, 0, &its, nullptr);
        m_running = false;
    }

    bool isRunning() const override {
        return m_running;
    }

    void waitForCycle() override {
        if (!m_running || m_timerFd < 0) return;

        uint64_t expirations;
        ssize_t ret = read(m_timerFd, &expirations, sizeof(expirations));
        
        if (ret == sizeof(expirations)) {
            Timestamp now = LinuxClock().nowMicros();
            
            if (m_lastTick != 0) {
                Microseconds jitter = std::abs(static_cast<int64_t>(now - m_lastTick) - m_periodUs);
                if (jitter > m_stats.maxJitter) {
                    m_stats.maxJitter = jitter;
                }
                // Simple moving average for avg jitter
                m_stats.avgJitter = (m_stats.avgJitter * m_stats.tickCount + jitter) / 
                                    (m_stats.tickCount + 1);
            }
            
            m_lastTick = now;
            m_stats.tickCount += expirations;
            
            if (expirations > 1) {
                m_stats.missedTicks += expirations - 1;
            }

            if (m_callback) {
                m_callback();
            }
        }
    }

    void setCallback(TimerCallback callback) override {
        m_callback = callback;
    }

    Microseconds getPeriodMicros() const override {
        return m_periodUs;
    }

    Stats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = {};
        m_lastTick = 0;
    }

private:
    int m_timerFd = -1;
    Microseconds m_periodUs = 1000;
    std::atomic<bool> m_running{false};
    TimerCallback m_callback;
    Stats m_stats;
    Timestamp m_lastTick = 0;
};

// ============================================================================
// Linux One-Shot Timer Implementation
// ============================================================================

class LinuxOneShotTimer : public IOneShotTimer {
public:
    LinuxOneShotTimer() {
        m_timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    }

    ~LinuxOneShotTimer() override {
        cancel();
        if (m_timerFd >= 0) {
            close(m_timerFd);
        }
    }

    bool start(Microseconds delayUs, TimerCallback callback) override {
        if (m_timerFd < 0 || delayUs <= 0) return false;

        m_callback = callback;
        m_expiryTime = LinuxClock().nowMicros() + delayUs;

        struct itimerspec its;
        its.it_value.tv_sec = delayUs / 1000000;
        its.it_value.tv_nsec = (delayUs % 1000000) * 1000;
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = 0;

        if (timerfd_settime(m_timerFd, 0, &its, nullptr) != 0) {
            return false;
        }

        m_pending = true;
        return true;
    }

    bool cancel() override {
        if (!m_pending) return false;

        struct itimerspec its;
        memset(&its, 0, sizeof(its));
        timerfd_settime(m_timerFd, 0, &its, nullptr);

        m_pending = false;
        return true;
    }

    bool isPending() const override {
        return m_pending;
    }

    Microseconds remaining() const override {
        if (!m_pending) return 0;

        Timestamp now = LinuxClock().nowMicros();
        if (now >= m_expiryTime) return 0;
        return m_expiryTime - now;
    }

private:
    int m_timerFd = -1;
    TimerCallback m_callback;
    Timestamp m_expiryTime = 0;
    std::atomic<bool> m_pending{false};
};

// ============================================================================
// Linux Clock Factory
// ============================================================================

class LinuxClockFactory : public IClockFactory {
public:
    IClock& getSystemClock() override {
        static LinuxClock clock;
        return clock;
    }

    std::unique_ptr<IPeriodicTimer> createPeriodicTimer() override {
        return std::make_unique<LinuxPeriodicTimer>();
    }

    std::unique_ptr<IOneShotTimer> createOneShotTimer() override {
        return std::make_unique<LinuxOneShotTimer>();
    }
};

// Clock factory singleton with explicit lifecycle.
// Use resetClockFactory() in tests to ensure clean state.
static std::unique_ptr<IClockFactory> g_clockFactory;

IClockFactory& getClockFactory() {
    if (!g_clockFactory) {
        g_clockFactory = std::make_unique<LinuxClockFactory>();
    }
    return *g_clockFactory;
}

void setClockFactory(std::unique_ptr<IClockFactory> factory) {
    g_clockFactory = std::move(factory);
}

void resetClockFactory() {
    g_clockFactory.reset();
}

} // namespace HAL
} // namespace EtherCAT

#endif // __linux__
