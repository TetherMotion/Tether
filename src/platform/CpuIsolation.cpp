/**
 * @file CpuIsolation.cpp
 * @brief Runtime CPU claim allocator — see header for the honesty notes.
 */

#include "tether/platform/CpuIsolation.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#ifdef __linux__
#include <sched.h>
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
        return do_claim(c);
    }

    // 2) Auto-pick: kernel-isolated & unclaimed first
    if (spec.prefer_isolated) {
        for (int c : isolated_) {
            if (!claimed(c) && listContains(online_, c) && room()) {
                --free_cnt;
                return do_claim(c);
            }
        }
    }
    // 3) Unclaimed, avoiding CPU0 (default IRQ sink)
    for (int c : online_) {
        if (spec.avoid_cpu0 && c == 0) continue;
        if (!claimed(c) && room()) {
            --free_cnt;
            return do_claim(c);
        }
    }
    // 4) Last resort: CPU0 itself
    for (int c : online_) {
        if (!claimed(c) && room()) {
            --free_cnt;
            return do_claim(c);
        }
    }

    TETHER_LOGW(TAG, "no free CPU to claim ({} online, {} already claimed) "
                     "— thread runs unpinned", online_.size(),
                online_.size() - free_cnt);
    return {};
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
}

} // namespace Platform
} // namespace Tether
