#pragma once

/**
 * @file CpuIsolation.hpp
 * @brief Opt-in runtime CPU isolation / claim allocator.
 *
 * What "isolation" means at runtime, honestly:
 *
 *   - True kernel isolation (no other tasks scheduled on the CPU) is only
 *     possible via the `isolcpus=` boot parameter or cgroup cpuset
 *     partitions (root).  This component DETECTS kernel-isolated CPUs via
 *     /sys/devices/system/cpu/isolated and prefers them.
 *   - Without kernel isolation, a "claim" is a process-wide reservation
 *     registry: claimed CPUs are handed to realtime threads as affinity
 *     targets, tracked so multiple Tether loops/threads never collide and
 *     never consume the whole machine, and released on exit.  On a
 *     non-isolated CPU other system tasks may still preempt — the
 *     allocator logs this explicitly so the degradation is visible.
 *
 * Guarantees:
 *   - Opt-in only (CyclicLoopConfig::cpu_isolation.enabled).
 *   - Every claim is logged; failures log a warning, never abort.
 *   - Claims are released on release()/releaseAll()/process exit
 *     (atexit hook) — CPUs are never left "locked".
 *   - The allocator never hands out the last available CPU: at least one
 *     online CPU always stays unclaimed for the rest of the system.
 *   - Kernel-isolated CPUs and CPUs already claimed (by us) are tracked
 *     separately; auto-pick prefers isolated-then-unclaimed.
 */

#include <cstdint>
#include <mutex>
#include <vector>

namespace Tether {
namespace Platform {

class CpuIsolation {
public:
    static CpuIsolation& instance();

    /// What kind of CPU a claim wants.
    struct Spec {
        int  requested_cpu   = -1;    ///< >=0: exactly this CPU
        bool prefer_isolated = true;  ///< prefer /sys-isolated CPUs
        bool avoid_cpu0      = true;  ///< CPU0 takes most default IRQs
    };

    /// A granted claim.
    struct Claim {
        int  cpu = -1;                 ///< -1 = no CPU granted
        bool kernel_isolated = false;  ///< isolcpus-managed (true isolation)
        bool valid() const { return cpu >= 0; }
    };

    /**
     * @brief Claim a CPU for a realtime thread's affinity.
     *
     * Order of preference when requested_cpu < 0:
     *   kernel-isolated & unclaimed → unclaimed non-CPU0 → any unclaimed.
     * Never grants a claim that would leave zero unclaimed online CPUs.
     *
     * @return Claim (cpu=-1 on failure — reason is logged).
     */
    Claim claim(const Spec& spec);

    /// Release one claim (idempotent — releasing a foreign/free CPU is a
    /// no-op with a debug log).
    void  release(int cpu);

    /// Release every claim this process holds.  Also registered with
    /// atexit() so CPUs are never left claimed after exit.
    void  releaseAll();

    // ---- introspection ---------------------------------------------------

    int  onlineCount()   const;   ///< CPUs in /sys/.../online (or sysconf)
    bool isKernelIsolated(int cpu) const;
    std::vector<int> isolatedCpus() const;
    std::vector<int> claimedCpus()  const;
    /// CPUs the rest of the process/system can still use.
    int  freeCount() const;

    CpuIsolation(const CpuIsolation&) = delete;
    CpuIsolation& operator=(const CpuIsolation&) = delete;

private:
    CpuIsolation();
    ~CpuIsolation() = default;

    static std::vector<int> parseCpuList(const char* path);
    static bool             listContains(const std::vector<int>& v, int cpu);

    mutable std::mutex mu_;
    std::vector<int>   online_;
    std::vector<int>   isolated_;
    uint64_t           claimed_[4] = {};   ///< bitmap, up to 256 CPUs
    bool               atexit_registered_ = false;
};

} // namespace Platform
} // namespace Tether
