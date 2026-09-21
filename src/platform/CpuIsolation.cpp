/**
 * @file CpuIsolation.cpp
 * @brief Runtime CPU claim allocator — see header for the honesty notes.
 */

#include "tether/platform/CpuIsolation.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#ifdef __linux__
#include <dirent.h>
#include <sched.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "logging/Logger.hpp"

namespace Tether {
namespace Platform {

static const char* TAG = "cpu_iso";

// ============================================================================
// Construction / discovery
// ============================================================================

CpuIsolation::CpuIsolation() {
    online_   = parseCpuList("/sys/devices/system/cpu/online");
    if (online_.empty()) {
#ifdef __linux__
        const long n = sysconf(_SC_NPROCESSORS_ONLN);
        for (long i = 0; i < n && i < 256; ++i)
            online_.push_back(static_cast<int>(i));
#else
        online_.push_back(0);
#endif
    }
    isolated_ = parseCpuList("/sys/devices/system/cpu/isolated");
    if (!isolated_.empty()) {
        TETHER_LOGI(TAG, "kernel-isolated CPUs: {} CPU(s)",
                    isolated_.size());
    }
}

CpuIsolation& CpuIsolation::instance() {
    static CpuIsolation inst;
    return inst;
}

// ============================================================================
// /sys cpu-list parsing ("0-3,5,8-9")
// ============================================================================

std::vector<int> CpuIsolation::parseCpuList(const char* path) {
    std::vector<int> out;
    std::ifstream f(path);
    if (!f) return out;
    std::string line;
    std::getline(f, line);
    std::istringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        const auto dash = tok.find('-');
        try {
            if (dash == std::string::npos) {
                out.push_back(std::stoi(tok));
            } else {
                const int a = std::stoi(tok.substr(0, dash));
                const int b = std::stoi(tok.substr(dash + 1));
                for (int i = a; i <= b && i < 256; ++i) out.push_back(i);
            }
        } catch (...) { /* malformed entry — skip */ }
    }
    return out;
}

bool CpuIsolation::listContains(const std::vector<int>& v, int cpu) {
    return std::find(v.begin(), v.end(), cpu) != v.end();
}

// ============================================================================
// Introspection
// ============================================================================

int  CpuIsolation::onlineCount() const { return (int)online_.size(); }

bool CpuIsolation::isKernelIsolated(int cpu) const {
    return listContains(isolated_, cpu);
}

std::vector<int> CpuIsolation::isolatedCpus() const {
    std::lock_guard<std::mutex> lk(mu_);
    return isolated_;
}

std::vector<int> CpuIsolation::claimedCpus() const {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<int> out;
    for (int c : online_)
        if (claimed_[c >> 6] & (1ULL << (c & 63))) out.push_back(c);
    return out;
}

int CpuIsolation::freeCount() const {
    std::lock_guard<std::mutex> lk(mu_);
    int free = 0;
    for (int c : online_)
        if (!(claimed_[c >> 6] & (1ULL << (c & 63)))) ++free;
    return free;
}

// ============================================================================
// claim / release
// ============================================================================

CpuIsolation::Claim CpuIsolation::claim(const Spec& spec) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!atexit_registered_) {
        std::atexit([]() { CpuIsolation::instance().releaseAll(); });
        atexit_registered_ = true;
    }

    auto claimed = [&](int c) -> bool {
        return (claimed_[c >> 6] & (1ULL << (c & 63))) != 0;
    };
    auto do_claim = [&](int c) -> Claim {
        claimed_[c >> 6] |= (1ULL << (c & 63));
        Claim cl;
        cl.cpu = c;
        cl.kernel_isolated = listContains(isolated_, c);
        if (cl.kernel_isolated) {
            TETHER_LOGI(TAG, "claimed CPU {} (kernel-isolated — true "
                             "isolation)", c);
        } else {
            TETHER_LOGI(TAG, "claimed CPU {} (affinity pin only — NOT "
                             "kernel-isolated; other tasks may still "
                             "preempt.  Use isolcpus= or a cpuset partition "
                             "for hard isolation)", c);
        }
        return cl;
    };
    // Count still-free CPUs; never grant a claim that would take the last
    // one — the rest of the system (kernel threads, IRQ handling, the
    // async poll thread) needs somewhere to run.
    int free_cnt = 0;
    for (int c : online_) if (!claimed(c)) ++free_cnt;
    auto room = [&]() { return free_cnt > 1; };

    // 1) Explicit request
    if (spec.requested_cpu >= 0) {
        const int c = spec.requested_cpu;
        if (!listContains(online_, c)) {
            TETHER_LOGW(TAG, "requested CPU {} is not online — claim denied",
                        c);
            return {};
        }
        if (claimed(c)) {
            TETHER_LOGW(TAG, "requested CPU {} already claimed — denied", c);
            return {};
        }
        if (!room()) {
            TETHER_LOGW(TAG, "claiming CPU {} would consume the last free "
                             "CPU — denied (leaving one for the system)", c);
            return {};
        }
        --free_cnt;
        return finishClaim(do_claim(c), spec);
    }

    // 2) Auto-pick: kernel-isolated & unclaimed first
    if (spec.prefer_isolated) {
        for (int c : isolated_) {
            if (!claimed(c) && listContains(online_, c) && room()) {
                --free_cnt;
                return finishClaim(do_claim(c), spec);
            }
        }
    }
    // 3) Unclaimed, avoiding CPU0 (default IRQ sink)
    for (int c : online_) {
        if (spec.avoid_cpu0 && c == 0) continue;
        if (!claimed(c) && room()) {
            --free_cnt;
            return finishClaim(do_claim(c), spec);
        }
    }
    // 4) Last resort: CPU0 itself
    for (int c : online_) {
        if (!claimed(c) && room()) {
            --free_cnt;
            return finishClaim(do_claim(c), spec);
        }
    }

    TETHER_LOGW(TAG, "no free CPU to claim ({} online, {} already claimed) "
                     "— thread runs unpinned", online_.size(),
                online_.size() - free_cnt);
    return {};
}

// ============================================================================
// Q8/Q9 — root opt-in hardening: cgroup2 cpuset partitions + IRQ steering
// ============================================================================

bool CpuIsolation::writeFile(const char* path, const std::string& val) {
    std::ofstream f(path, std::ios::trunc);
    if (!f) return false;
    f << val;
    return f.good();
}

bool CpuIsolation::readFile(const char* path, std::string& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::getline(f, out);
    return true;
}

bool CpuIsolation::createCpusetPartition(int cpu) {
#ifdef __linux__
    if (geteuid() != 0) return false;    // cpuset ops need CAP_SYS_ADMIN
    if (!cgroup_v2_probed_) {
        cgroup_v2_probed_ = true;
        struct stat st{};
        cgroup_v2_ok_ =
            ::stat("/sys/fs/cgroup/cgroup.controllers", &st) == 0;
        if (!cgroup_v2_ok_)
            TETHER_LOGW(TAG, "cgroup v2 not detected — cpuset partitions "
                             "unavailable");
    }
    if (!cgroup_v2_ok_) return false;

    // Enable cpuset delegation on the root (idempotent; EBUSY-ish failures
    // when already enabled still report write success on most kernels).
    writeFile("/sys/fs/cgroup/cgroup.subtree_control", "+cpuset");

    const std::string dir = "/sys/fs/cgroup/tether_rt";
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        TETHER_LOGW(TAG, "cpuset: mkdir {} failed ({})", dir,
                    strerror(errno));
        return false;
    }
    // The partition covers EVERY claimed CPU — a second claim extends the
    // group's cpuset rather than competing for the same name.
    std::string cpus;
    for (int c : online_) {
        if (!(claimed_[c >> 6] & (1ULL << (c & 63)))) continue;
        if (!cpus.empty()) cpus += ',';
        cpus += std::to_string(c);
    }
    if (!writeFile((dir + "/cpuset.cpus").c_str(), cpus)) {
        TETHER_LOGW(TAG, "cpuset: cannot assign CPUs [{}] to {}", cpus, dir);
        return false;
    }
    writeFile((dir + "/cpuset.mems").c_str(), "0");
    if (!writeFile((dir + "/cpuset.cpus.partition").c_str(), "isolated")) {
        TETHER_LOGW(TAG, "cpuset: partition=isolated rejected for CPU {} "
                         "(CPU may still be root-domain)", cpu);
        return false;
    }
    if (!listContains(cpuset_cpus_, cpu)) cpuset_cpus_.push_back(cpu);
    TETHER_LOGI(TAG, "cgroup2 cpuset partition 'tether_rt' isolates [{}]",
                cpus);
    return true;
#else
    (void)cpu;
    return false;
#endif
}

/// Post-grant hardening — the claim is already recorded; failures degrade
/// to the affinity-only claim with a warning (never deny the claim).
CpuIsolation::Claim CpuIsolation::finishClaim(Claim cl, const Spec& spec) {
    if (!cl.valid()) return cl;
    if (spec.create_cpuset) {
        cl.cpuset_isolated = createCpusetPartition(cl.cpu);
        if (cl.cpuset_isolated) {
            TETHER_LOGI(TAG, "CPU {} hard-isolated via cgroup2 partition",
                        cl.cpu);
        } else {
            TETHER_LOGW(TAG, "CPU {}: cpuset partition unavailable — "
                             "affinity pin only", cl.cpu);
        }
    }
    if (spec.steer_irqs) {
        steerIrqsOffClaims();
        cl.irqs_steered = irq_steering_active_;
    }
    return cl;
}

void CpuIsolation::steerIrqsOffClaims() {
#ifdef __linux__
    if (geteuid() != 0) {
        TETHER_LOGW(TAG, "IRQ steering needs root — skipped");
        return;
    }
    // Complement mask: every online CPU that is NOT claimed.
    std::string mask;
    bool first = true;
    for (int c : online_) {
        if (claimed_[c >> 6] & (1ULL << (c & 63))) continue;
        if (!first) mask += ',';
        mask += std::to_string(c);
        first = false;
    }
    if (mask.empty()) return;

    DIR* d = ::opendir("/proc/irq");
    if (!d) return;
    struct dirent* de;
    int steered = 0, failed = 0;
    while ((de = ::readdir(d))) {
        if (de->d_name[0] < '0' || de->d_name[0] > '9') continue;
        const int irq = std::atoi(de->d_name);
        const std::string path =
            std::string("/proc/irq/") + de->d_name + "/smp_affinity_list";
        std::string orig;
        if (!readFile(path.c_str(), orig)) continue;
        // Save the original mask once per IRQ so releaseAll() can restore.
        bool known = false;
        for (const auto& p : irq_orig_masks_)
            if (p.first == irq) { known = true; break; }
        if (!known) irq_orig_masks_.emplace_back(irq, orig);
        if (writeFile(path.c_str(), mask)) ++steered;
        else ++failed;   // per-IRQ restrictions — best-effort
    }
    ::closedir(d);
    irq_steering_active_ = steered > 0;
    TETHER_LOGI(TAG, "IRQ steering: {} IRQ(s) pinned to [{}], {} rejected",
                steered, mask, failed);
#endif
}

void CpuIsolation::teardownPartitions() {
#ifdef __linux__
    for (int cpu : cpuset_cpus_) {
        const std::string dir = "/sys/fs/cgroup/tether_rt";
        writeFile((dir + "/cpuset.cpus.partition").c_str(), "member");
        if (::rmdir(dir.c_str()) != 0)
            TETHER_LOGW(TAG, "cpuset: cleanup of {} for CPU {} failed ({})",
                        dir, cpu, strerror(errno));
        else
            TETHER_LOGI(TAG, "cpuset partition for CPU {} removed", cpu);
    }
    cpuset_cpus_.clear();
#endif
}

void CpuIsolation::restoreIrqMasks() {
#ifdef __linux__
    for (const auto& [irq, mask] : irq_orig_masks_) {
        const std::string path =
            "/proc/irq/" + std::to_string(irq) + "/smp_affinity_list";
        writeFile(path.c_str(), mask);
    }
    if (!irq_orig_masks_.empty())
        TETHER_LOGI(TAG, "restored {} IRQ affinity mask(s)",
                    irq_orig_masks_.size());
    irq_orig_masks_.clear();
    irq_steering_active_ = false;
#endif
}

void CpuIsolation::release(int cpu) {
    std::lock_guard<std::mutex> lk(mu_);
    if (cpu < 0 || cpu >= 256) return;
    if (claimed_[cpu >> 6] & (1ULL << (cpu & 63))) {
        claimed_[cpu >> 6] &= ~(1ULL << (cpu & 63));
        TETHER_LOGI(TAG, "released CPU {}", cpu);
    }
}

void CpuIsolation::releaseAll() {
    std::lock_guard<std::mutex> lk(mu_);
    int released = 0;
    for (int c : online_) {
        if (claimed_[c >> 6] & (1ULL << (c & 63))) {
            claimed_[c >> 6] &= ~(1ULL << (c & 63));
            ++released;
        }
    }
    if (released > 0)
        TETHER_LOGI(TAG, "released {} CPU claim(s) on exit", released);
    teardownPartitions();   // cgroup2 partition is pointless with no claims
    restoreIrqMasks();      // IRQs go back to their original affinities
}

} // namespace Platform
} // namespace Tether
