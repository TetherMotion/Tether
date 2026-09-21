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
#include <string>
#include <utility>
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
        /// Q8: running as root → create a cgroup2 cpuset partition
        /// covering the claimed CPU (real isolation without isolcpus=).
        /// Best-effort: failure logs a warning and still grants the claim.
        bool create_cpuset   = false;
        /// Q9: running as root → write /proc/irq/*/smp_affinity_list so
        /// IRQs avoid every claimed CPU.  Original masks are restored on
        /// releaseAll().  Best-effort per IRQ.
        bool steer_irqs      = false;
    };

    /// A granted claim.
    struct Claim {
        int  cpu = -1;                 ///< -1 = no CPU granted
        bool kernel_isolated = false;  ///< isolcpus-managed (true isolation)
        bool cpuset_isolated = false;  ///< cgroup2 partition created (Q8)
        bool irqs_steered    = false;  ///< IRQ affinities moved off (Q9)
        bool valid() const { return cpu >= 0; }
        /// True when the CPU is hard-isolated by either mechanism.
        bool hardIsolated() const { return kernel_isolated || cpuset_isolated; }
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

    // ---- Q8/Q9: root opt-in hardening -------------------------------------
    /// Apply spec hardening to a granted claim (cpuset partition, IRQ
    /// steering).  Caller holds mu_.
    Claim finishClaim(Claim cl, const Spec& spec);
    /// Create/extend the cgroup2 cpuset partition to cover every claimed
    /// CPU.  Returns true when the partition isolates them.
    bool createCpusetPartition(int cpu);
    /// Write (all online CPUs \ claimed) into every IRQ affinity file.
    void steerIrqsOffClaims();
    /// Tear down all created partitions / restore IRQ masks.  Called from
    /// releaseAll().
    void teardownPartitions();
    void restoreIrqMasks();
    static bool writeFile(const char* path, const std::string& val);
    static bool readFile(const char* path, std::string& out);

    std::vector<int> cpuset_cpus_;          ///< CPUs with created partitions
    bool           cgroup_v2_probed_ = false;
    bool           cgroup_v2_ok_     = false;
    /// IRQ number → original smp_affinity_list text (restore on release).
    std::vector<std::pair<int, std::string>> irq_orig_masks_;
    bool           irq_steering_active_ = false;
};

} // namespace Platform
} // namespace Tether
