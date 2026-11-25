#pragma once

/**
 * @file CpuIsolation.hpp
 * @brief Opt-in runtime CPU isolation / claim allocator.
 *
 * What "isolation" means at runtime, honestly:
 *
 *   - True kernel isolation (no unaffined system work scheduled on the CPU)
 *     is possible via the `isolcpus=` boot parameter or a dedicated cpuset
 *     cgroup.  This component DETECTS kernel-isolated CPUs via
 *     /sys/devices/system/cpu/isolated and prefers them.
 *   - cgroup v2 isolated partitions are the preferred runtime mechanism.
 *     Tether creates a per-process threaded housekeeping cgroup plus a
 *     threaded isolated partition, moves the process into the housekeeping
 *     cgroup, and migrates individual RT threads into the partition with
 *     cgroup.threads.  This requires root on normal systems and temporarily
 *     moves the process out of its original (often systemd) cgroup.
 *   - cgroup v1 cpusets are retained as the legacy fallback for hybrid
 *     systems.  v1 can migrate individual threads through the tasks file,
 *     but cpu_exclusive does not by itself remove CPUs from the parent/root
 *     cpuset; a complete v1 shield also needs housekeeping-task migration.
 *   - Without kernel or cgroup isolation, a "claim" is a process-wide
 *     reservation registry: claimed CPUs are handed to realtime threads as
 *     affinity targets, tracked so multiple Tether loops/threads never
 *     collide and never consume the whole machine, and released on exit.
 *     On a non-isolated CPU other system tasks may still preempt — the
 *     allocator logs this explicitly so the degradation is visible.
 *
 * Guarantees:
 *   - Opt-in only (CyclicLoopConfig::cpu_isolation.enabled).
 *   - Every claim is logged; failures log a warning, never abort.
 *   - Claims are released on release()/releaseAll()/process exit
 *     (atexit hook) — CPUs are never left "locked" by the allocator.
 *   - The allocator never hands out the last available CPU: at least one
 *     online CPU always stays unclaimed for the rest of the system.
 *   - Kernel-isolated CPUs and CPUs already claimed (by us) are tracked
 *     separately; auto-pick prefers isolated-then-unclaimed.
 */

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Tether {
namespace Platform {

class CpuIsolation {
public:
    /** cpuset implementation requested for create_cpuset. */
    enum class CpusetBackend {
        /** Prefer cgroup v2 isolated partitions; fall back to legacy v1. */
        Auto = 0,
        /** cgroup v2 threaded isolated partition (preferred). */
        CgroupV2,
        /** legacy cgroup v1 cpu_exclusive cpuset. */
        CgroupV1,
    };

    static CpuIsolation& instance();

    /**What kind of CPU a claim wants. */
    struct Spec {
        int  requested_cpu   = -1; /**< >=0: exactly this CPU */
        bool prefer_isolated = true; /**< prefer /sys-isolated CPUs */
        bool avoid_cpu0      = true; /**< CPU0 takes most default IRQs */
        /**
         * Running with sufficient cgroup privileges → create dedicated
         * cpuset cgroups covering the claimed CPUs, then join RT threads
         * via adoptCurrentThread().
         *
         * Backend selection is controlled by cpuset_backend.  Auto (the
         * default) tries cgroup v2 first and falls back to legacy cgroup
         * v1.  The v2 backend uses two per-process threaded cgroups:
         * a housekeeping cgroup for ordinary threads and an isolated
         * partition for RT threads.  The v1 backend uses the historical
         * /sys/fs/cgroup/cpuset/tether_rt cpu_exclusive cpuset.
         */
        bool create_cpuset   = false;
        /**Requested cpuset implementation when create_cpuset is true. */
        CpusetBackend cpuset_backend = CpusetBackend::Auto;
        /**
         * Q9: automatically attempt to write /proc/irq/N/smp_affinity_list
         * so IRQs avoid every claimed CPU.  This is best-effort: insufficient
         * privileges or per-IRQ restrictions only produce warnings/degraded
         * coverage, never a denied claim.  Original masks are restored on
         * final claim release/normal exit; SIGKILL cannot run that restoration.
         * Set false to opt out.
         */
        bool steer_irqs      = true;
    };

    /**A granted claim. */
    struct Claim {
        int  cpu = -1; /**< -1 = no CPU granted */
        bool kernel_isolated = false; /**< isolcpus-managed (true isolation) */
        bool cpuset_isolated = false; /**< cpuset cgroup made/extended */
        /**
         * Active cpuset backend when cpuset_isolated is true; empty when
         * the claim is affinity-only or kernel-isolated without a cgroup.
         */
        std::optional<CpusetBackend> cpuset_backend;
        bool irqs_steered    = false; /**< IRQ affinities moved off (Q9) */
        bool valid() const { return cpu >= 0; }
        /**
         * @brief True when the CPU is hard-isolated by either mechanism.
         * @return true for kernel or cpuset isolation.
         */
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

    /**
     * Release one claim (idempotent — releasing a foreign/free CPU is a
     * no-op with a debug log).  When the last claim goes away the cpuset
     * partition is torn down and IRQ masks are restored.
     */
    void  release(int cpu);

    /**
     * Release every claim this process holds.  Also registered with
     * atexit() so CPUs are never left claimed after exit.
     */
    void  releaseAll();

    /**
     * @brief Move the CALLING thread into the cpuset created by
     *        create_cpuset, when @p cpu is covered by it.
     *
     * cgroup v2 writes the thread ID to the threaded partition's
     * cgroup.threads file; legacy cgroup v1 writes it to the tasks file.
     * The write is validated both by the kernel write result and by reading
     * back the calling thread's cgroup membership.  Call once at thread
     * start, then re-apply the affinity pin.
     *
     * @return true when the thread was moved into the cpuset;
     *         false when there is nothing to join (no cpuset, or
     *         @p cpu is not covered) or the move failed (logged).
     */
    bool adoptCurrentThread(int cpu);

    // ---- introspection ---------------------------------------------------

    int  onlineCount()   const; /**< CPUs in /sys/.../online (or sysconf) */
    bool isKernelIsolated(int cpu) const;
    std::vector<int> isolatedCpus() const;
    std::vector<int> claimedCpus()  const;
    /**CPUs the rest of the process/system can still use. */
    int  freeCount() const;
    /**Active cpuset backend, or empty when no cpuset partition exists. */
    std::optional<CpusetBackend> activeCpusetBackend() const;
    /**Stable human-readable backend name for logging/tests. */
    static const char* backendName(CpusetBackend backend);

    CpuIsolation(const CpuIsolation&) = delete;
    CpuIsolation& operator=(const CpuIsolation&) = delete;

private:
    CpuIsolation();
    ~CpuIsolation() = default;

    static std::vector<int> parseCpuList(const char* path);
    static std::vector<int> parseCpuListText(const std::string& text);
    static std::string      formatCpuList(std::vector<int> cpus);
    static bool             listContains(const std::vector<int>& v, int cpu);

    mutable std::mutex mu_;
    std::vector<int>   online_;
    std::vector<int>   isolated_;
    uint64_t           claimed_[4] = {}; /**< bitmap, up to 256 CPUs */
    bool               atexit_registered_ = false;

    // ---- root opt-in hardening -------------------------------------------
    /**
     * Apply spec hardening to a granted claim (cpuset cgroup, IRQ
     * steering).  Caller holds mu_.
     */
    Claim finishClaim(Claim cl, const Spec& spec);
    /**Create/extend the selected cpuset backend. Caller holds mu_. */
    bool createCpusetPartition(int cpu, const Spec& spec);
    /**cgroup v2 threaded housekeeping + isolated partition. */
    bool createCpusetV2(int cpu);
    /** legacy cgroup v1 cpu_exclusive cpuset. */
    bool createCpusetV1(int cpu);
    /**Update masks after claim/release. Caller holds mu_. */
    bool updateActiveCpusetMasks();
    bool updateCpusetV2Masks();
    bool updateCpusetV1Masks();
    /**Move this process into the v2 housekeeping cgroup once. */
    bool migrateProcessToV2Housekeeping();
    /**Write (all online CPUs \ claimed) into every IRQ affinity file. */
    void steerIrqsOffClaims();
    /**Tear down the created cpuset / restore process membership. */
    void teardownPartitions();
    void teardownCpusetV1();
    void teardownCpusetV2();
    /**Remove empty per-process v2 cgroup directories whose owner pid is gone. */
    void cleanupStaleV2Cgroups();
    void restoreIrqMasks();

    static bool writeFile(const char* path, const std::string& val,
                          int* error_out = nullptr);
    static bool readFile(const char* path, std::string& out);
    static bool readFileAll(const char* path, std::string& out);

    std::vector<int> cpuset_cpus_; /**< CPUs covered by the active cpuset */
    /**Affinity-only claims excluded after a failed cpuset extension. */
    uint64_t cpuset_excluded_[4] = {};
    std::optional<CpusetBackend> active_cpuset_backend_;

    // legacy cgroup v1 state
    bool           cpuset_v1_probed_ = false;
    bool           cpuset_v1_ok_     = false;

    // cgroup v2 state (per-process cgroups; direct children of the v2 root)
    int            owner_pid_ = -1;
    std::string    v2_name_suffix_; /**< pid + creation-time unique token */
    bool           v2_dirs_created_ = false;
    bool           v2_process_migrated_ = false;
    std::string    v2_original_cgroup_;

    /**IRQ number → original smp_affinity_list text (restore on release). */
    std::vector<std::pair<int, std::string>> irq_orig_masks_;
    bool           irq_steering_active_ = false;
    bool           irq_privilege_warning_emitted_ = false;
};

} // namespace Platform
} // namespace Tether
