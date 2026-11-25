/**
 * @file CpuIsolation.cpp
 * @brief Runtime CPU claim allocator — see header for the honesty notes.
 */

#include "tether/platform/CpuIsolation.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#ifdef __linux__
#include <dirent.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "logging/Logger.hpp"

namespace Tether {
namespace Platform {

static const char* TAG = "cpu_iso";

namespace {

std::string trim(const std::string& in) {
    const auto first = in.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = in.find_last_not_of(" \t\r\n");
    return in.substr(first, last - first + 1);
}

bool tokenListContains(const std::string& text, const std::string& token) {
    std::istringstream in(text);
    std::string t;
    while (in >> t) if (t == token) return true;
    return false;
}

#ifdef __linux__
bool pathExists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0;
}

std::string baseName(const std::string& path) {
    const auto pos = path.find_last_of('/');
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string ownCgroupV2Path() {
    std::ifstream f("/proc/thread-self/cgroup");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("0::", 0) == 0) return line.substr(3);
    }
    return {};
}

bool cgroupPopulated(const std::string& dir) {
    std::ifstream f(dir + "/cgroup.events");
    if (!f) return true;   // unknown → never remove
    std::string key;
    long long value = 0;
    while (f >> key >> value) {
        if (key == "populated") return value != 0;
    }
    return true;
}
#endif

} // namespace

// ============================================================================
// Construction / discovery
// ============================================================================

CpuIsolation::CpuIsolation() {
#ifdef __linux__
    owner_pid_ = static_cast<int>(::getpid());
    timespec ts{};
    ::clock_gettime(CLOCK_REALTIME, &ts);
    v2_name_suffix_ = std::to_string(owner_pid_) + "_" +
                      std::to_string(static_cast<long long>(ts.tv_sec)) + "_" +
                      std::to_string(static_cast<long long>(ts.tv_nsec));
#endif
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
// cpu-list parsing/formatting ("0-3,5,8-9")
// ============================================================================

std::vector<int> CpuIsolation::parseCpuListText(const std::string& text) {
    std::vector<int> out;
    std::istringstream ss(trim(text));
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok = trim(tok);
        if (tok.empty()) continue;
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

std::vector<int> CpuIsolation::parseCpuList(const char* path) {
    std::ifstream f(path);
    if (!f) return {};
    std::string line;
    std::getline(f, line);
    return parseCpuListText(line);
}

std::string CpuIsolation::formatCpuList(std::vector<int> cpus) {
    std::sort(cpus.begin(), cpus.end());
    cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
    std::string out;
    for (size_t i = 0; i < cpus.size();) {
        size_t j = i;
        while (j + 1 < cpus.size() && cpus[j + 1] == cpus[j] + 1) ++j;
        if (!out.empty()) out += ',';
        out += std::to_string(cpus[i]);
        if (j != i) {
            out += '-';
            out += std::to_string(cpus[j]);
        }
        i = j + 1;
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

std::optional<CpuIsolation::CpusetBackend>
CpuIsolation::activeCpusetBackend() const {
    std::lock_guard<std::mutex> lk(mu_);
    return active_cpuset_backend_;
}

const char* CpuIsolation::backendName(CpusetBackend backend) {
    switch (backend) {
    case CpusetBackend::Auto:     return "auto";
    case CpusetBackend::CgroupV2: return "cgroup-v2";
    case CpusetBackend::CgroupV1: return "cgroup-v1";
    }
    return "unknown";
}

// ============================================================================
// claim / release
// ============================================================================

CpuIsolation::Claim CpuIsolation::claim(const Spec& spec) {
#ifdef __linux__
    if (owner_pid_ >= 0 && owner_pid_ != static_cast<int>(::getpid())) {
        TETHER_LOGW(TAG, "CPU claim denied in forked child — claims belong "
                         "to the creating process");
        return {};
    }
#endif
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
                             "scheduler-domain isolation)", c);
        } else {
            TETHER_LOGI(TAG, "claimed CPU {} (affinity pin only unless "
                             "cpuset creation succeeds)", c);
        }
        return cl;
    };
    // Count still-free CPUs; never grant a claim that would take the last
    // one — the rest of the system needs somewhere to run.
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
// Small file I/O.  cgroup control writes commonly fail at write/flush time,
// so use one direct write(2) and check close(2); std::ofstream can report
// success before its buffered error becomes visible.
// ============================================================================

bool CpuIsolation::writeFile(const char* path, const std::string& val,
                             int* error_out) {
#ifdef __linux__
    if (error_out) *error_out = 0;
    const int fd = ::open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        if (error_out) *error_out = errno;
        return false;
    }
    const ssize_t n = ::write(fd, val.data(), val.size());
    const int write_errno = errno;
    const int close_rc = ::close(fd);
    const int close_errno = errno;
    if (n != static_cast<ssize_t>(val.size())) {
        if (error_out) *error_out = n < 0 ? write_errno : EIO;
        return false;
    }
    if (close_rc != 0) {
        if (error_out) *error_out = close_errno;
        return false;
    }
    return true;
#else
    (void)path; (void)val;
    if (error_out) *error_out = ENOSYS;
    return false;
#endif
}

bool CpuIsolation::readFile(const char* path, std::string& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::getline(f, out);
    return true;
}

bool CpuIsolation::readFileAll(const char* path, std::string& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

// ============================================================================
// cpuset backend selection
// ============================================================================

bool CpuIsolation::createCpusetPartition(int cpu, const Spec& spec) {
#ifdef __linux__
    if (active_cpuset_backend_) {
        // The cgroup is shared by every claim in this process.  Once a
        // backend exists, later claims extend it irrespective of their own
        // requested backend — mixing backends for one process would make
        // thread membership incoherent.
        if (spec.cpuset_backend != CpusetBackend::Auto &&
            spec.cpuset_backend != *active_cpuset_backend_) {
            TETHER_LOGW(TAG, "requested {} cpuset but process already uses {}",
                        backendName(spec.cpuset_backend),
                        backendName(*active_cpuset_backend_));
        }
        cpuset_excluded_[cpu >> 6] &= ~(1ULL << (cpu & 63));
        if (!updateActiveCpusetMasks()) {
            // A failed extension must not destroy isolation for the claims
            // that were already working (for example when another process
            // exclusively owns the newly requested CPU), and later partial
            // releases must not silently retry the failed CPU.  Mark it as
            // affinity-only for cpuset-mask purposes, then roll back to the
            // pre-extension set.
            const auto before = cpuset_cpus_;
            cpuset_excluded_[cpu >> 6] |= (1ULL << (cpu & 63));
            const bool rolled_back = updateActiveCpusetMasks();
            if (!rolled_back) cpuset_cpus_ = before;
            TETHER_LOGW(TAG, "failed to extend active {} cpuset to CPU {} "
                             "(rollback {})",
                        backendName(*active_cpuset_backend_), cpu,
                        rolled_back ? "succeeded" : "failed");
            return false;
        }
        return listContains(cpuset_cpus_, cpu);
    }

    const bool try_v2 = spec.cpuset_backend == CpusetBackend::Auto ||
                        spec.cpuset_backend == CpusetBackend::CgroupV2;
    const bool try_v1 = spec.cpuset_backend == CpusetBackend::Auto ||
                        spec.cpuset_backend == CpusetBackend::CgroupV1;

    if (try_v2) {
        if (createCpusetV2(cpu)) return true;
        if (spec.cpuset_backend == CpusetBackend::CgroupV2) return false;
        TETHER_LOGW(TAG, "cgroup v2 cpuset unavailable — trying legacy v1");
    }
    if (try_v1) {
        if (createCpusetV1(cpu)) return true;
    }
#else
    (void)cpu; (void)spec;
#endif
    return false;
}

CpuIsolation::Claim CpuIsolation::finishClaim(Claim cl, const Spec& spec) {
    if (!cl.valid()) return cl;
    if (spec.create_cpuset) {
        cl.cpuset_isolated = createCpusetPartition(cl.cpu, spec);
        if (cl.cpuset_isolated) {
            cl.cpuset_backend = active_cpuset_backend_;
            TETHER_LOGI(TAG, "CPU {} hard-isolated via {} cpuset",
                        cl.cpu, backendName(*active_cpuset_backend_));
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

// ============================================================================
// cgroup v2 backend
// ============================================================================

#ifdef __linux__
namespace {

std::string v2RtDirForSuffix(const std::string& suffix) {
    return "/sys/fs/cgroup/tether_v2_rt_" + suffix;
}
std::string v2HkDirForSuffix(const std::string& suffix) {
    return "/sys/fs/cgroup/tether_v2_hk_" + suffix;
}

bool makeEmptyCgroupDir(const std::string& dir) {
    if (::mkdir(dir.c_str(), 0755) == 0) return true;
    if (errno != EEXIST) return false;
    // Reuse only a provably empty stale directory (normally left by a
    // crashed earlier process).  Never evict a populated cgroup.
    if (!cgroupPopulated(dir) && ::rmdir(dir.c_str()) == 0)
        return ::mkdir(dir.c_str(), 0755) == 0;
    return false;
}

} // namespace
#endif

void CpuIsolation::cleanupStaleV2Cgroups() {
#ifdef __linux__
    DIR* d = ::opendir("/sys/fs/cgroup");
    if (!d) return;

    // Scan housekeeping/RT pairs.  An active owner always leaves at least
    // one of the pair populated after setup; the short age grace covers the
    // transient mkdir→migration setup window (including pid namespaces where
    // the numeric owner pid is not visible here).
    constexpr time_t kSetupGraceSeconds = 5;
    const time_t now = ::time(nullptr);
    struct dirent* de;
    while ((de = ::readdir(d))) {
        const std::string hk_name = de->d_name;
        const std::string hk_prefix = "tether_v2_hk_";
        if (hk_name.rfind(hk_prefix, 0) != 0) continue;
        const std::string suffix = hk_name.substr(hk_prefix.size());
        const auto sep = suffix.find('_');
        if (sep == std::string::npos) continue;
        const long pid = std::strtol(suffix.substr(0, sep).c_str(), nullptr, 10);
        if (pid > 0 &&
            ::access(("/proc/" + std::to_string(pid)).c_str(), F_OK) == 0)
            continue;

        const std::string hk = "/sys/fs/cgroup/" + hk_name;
        const std::string rt = "/sys/fs/cgroup/tether_v2_rt_" + suffix;
        struct stat st{};
        if (::stat(hk.c_str(), &st) != 0) continue;
        if (now - st.st_mtim.tv_sec < kSetupGraceSeconds) continue;
        if (cgroupPopulated(hk)) continue;
        if (pathExists(rt) && cgroupPopulated(rt)) continue;

        if (pathExists(rt) && ::rmdir(rt.c_str()) != 0) continue;
        if (::rmdir(hk.c_str()) == 0)
            TETHER_LOGI(TAG, "removed stale empty cpuset cgroup pair {}", suffix);
    }
    ::closedir(d);

    // Normally RT and housekeeping cgroups are removed as a pair.  If an RT
    // directory nevertheless outlives its housekeeping sibling (for example
    // a partially failed teardown), reap it independently when it is empty.
    d = ::opendir("/sys/fs/cgroup");
    if (!d) return;
    while ((de = ::readdir(d))) {
        const std::string rt_name = de->d_name;
        const std::string rt_prefix = "tether_v2_rt_";
        if (rt_name.rfind(rt_prefix, 0) != 0) continue;
        const std::string suffix = rt_name.substr(rt_prefix.size());
        const std::string hk = "/sys/fs/cgroup/tether_v2_hk_" + suffix;
        if (pathExists(hk)) continue;
        const std::string rt = "/sys/fs/cgroup/" + rt_name;
        struct stat st{};
        if (::stat(rt.c_str(), &st) != 0) continue;
        if (now - st.st_mtim.tv_sec < kSetupGraceSeconds) continue;
        if (!cgroupPopulated(rt) && ::rmdir(rt.c_str()) == 0)
            TETHER_LOGI(TAG, "removed stale empty isolated cpuset cgroup {}", rt_name);
    }
    ::closedir(d);
#endif
}

bool CpuIsolation::updateCpusetV2Masks() {
#ifdef __linux__
    const std::string rt = v2RtDirForSuffix(v2_name_suffix_);
    const std::string hk = v2HkDirForSuffix(v2_name_suffix_);

    std::vector<int> claimed;
    std::vector<int> housekeeping;
    for (int c : online_) {
        const uint64_t bit = 1ULL << (c & 63);
        if ((claimed_[c >> 6] & bit) && !(cpuset_excluded_[c >> 6] & bit))
            claimed.push_back(c);
        else
            housekeeping.push_back(c);
    }
    if (claimed.empty() || housekeeping.empty()) return false;
    const std::string claimed_s = formatCpuList(claimed);
    const std::string hk_s = formatCpuList(housekeeping);

    int err = 0;
    if (!writeFile((hk + "/cpuset.cpus").c_str(), hk_s, &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot assign housekeeping CPUs [{}] ({})",
                    hk_s, strerror(err));
        return false;
    }
    if (!writeFile((rt + "/cpuset.cpus").c_str(), claimed_s, &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot assign isolated CPUs [{}] ({})",
                    claimed_s, strerror(err));
        return false;
    }
    if (pathExists(rt + "/cpuset.cpus.exclusive") &&
        !writeFile((rt + "/cpuset.cpus.exclusive").c_str(), claimed_s, &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot make CPUs [{}] exclusive ({})",
                    claimed_s, strerror(err));
        return false;
    }
    if (!writeFile((rt + "/cpuset.cpus.partition").c_str(), "isolated", &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot create isolated partition ({})",
                    strerror(err));
        return false;
    }

    // Read back the effective state — cgroup writes can be accepted while
    // leaving the partition invalid or granting only a subset of CPUs.
    std::string s;
    if (!readFile((rt + "/cpuset.cpus.partition").c_str(), s) ||
        trim(s) != "isolated") {
        TETHER_LOGW(TAG, "v2 cpuset: partition did not become isolated (state '{}')",
                    trim(s));
        return false;
    }
    if (!readFile((rt + "/cpuset.cpus.effective").c_str(), s)) return false;
    const auto effective = parseCpuListText(s);
    for (int c : claimed) {
        if (!listContains(effective, c)) {
            TETHER_LOGW(TAG, "v2 cpuset: CPU {} missing from effective set [{}]",
                        c, trim(s));
            return false;
        }
    }
    if (pathExists(rt + "/cpuset.cpus.exclusive.effective") &&
        readFile((rt + "/cpuset.cpus.exclusive.effective").c_str(), s)) {
        const auto xeffective = parseCpuListText(s);
        for (int c : claimed) {
            if (!listContains(xeffective, c)) {
                TETHER_LOGW(TAG, "v2 cpuset: CPU {} missing from exclusive set [{}]",
                            c, trim(s));
                return false;
            }
        }
    }
    if (readFile((hk + "/cpuset.cpus.effective").c_str(), s)) {
        const auto hk_eff = parseCpuListText(s);
        for (int c : claimed) {
            if (listContains(hk_eff, c)) {
                TETHER_LOGW(TAG, "v2 cpuset: housekeeping cgroup still includes CPU {}",
                            c);
                return false;
            }
        }
    }
    cpuset_cpus_ = claimed;
    return true;
#else
    return false;
#endif
}

bool CpuIsolation::migrateProcessToV2Housekeeping() {
#ifdef __linux__
    if (v2_process_migrated_) return true;
    const std::string hk = v2HkDirForSuffix(v2_name_suffix_);
    if (v2_original_cgroup_.empty()) {
        v2_original_cgroup_ = ownCgroupV2Path();
        if (v2_original_cgroup_.empty()) v2_original_cgroup_ = "/";
    }

    int err = 0;
    const std::string pid = std::to_string(owner_pid_);
    if (!writeFile((hk + "/cgroup.procs").c_str(), pid, &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot move process {} into housekeeping "
                         "cgroup ({})", pid, strerror(err));
        return false;
    }

    // Validate thread-visible membership rather than trusting the write.
    const std::string now = ownCgroupV2Path();
    const std::string expect_suffix = "/" + baseName(hk);
    if (!endsWith(now, expect_suffix)) {
        TETHER_LOGW(TAG, "v2 cpuset: process membership readback mismatch "
                         "(expected suffix '{}', got '{}')",
                    expect_suffix, now);
        return false;
    }
    v2_process_migrated_ = true;
    TETHER_LOGI(TAG, "v2 cpuset: process moved {} → {} (RT threads migrate "
                     "individually into the isolated partition)",
                v2_original_cgroup_, now);
    return true;
#else
    return false;
#endif
}

bool CpuIsolation::createCpusetV2(int cpu) {
#ifdef __linux__
    (void)cpu;
    if (geteuid() != 0) {
        TETHER_LOGW(TAG, "cgroup v2 cpuset needs root on this system — "
                         "claim degrades to affinity-only");
        return false;
    }

    std::string controllers;
    if (!readFile("/sys/fs/cgroup/cgroup.controllers", controllers) ||
        !tokenListContains(controllers, "cpuset")) {
        TETHER_LOGW(TAG, "cgroup v2 cpuset controller unavailable");
        return false;
    }
    std::string subtree;
    if (!readFile("/sys/fs/cgroup/cgroup.subtree_control", subtree) ||
        !tokenListContains(subtree, "cpuset")) {
        int err = 0;
        if (!writeFile("/sys/fs/cgroup/cgroup.subtree_control", "+cpuset", &err)) {
            TETHER_LOGW(TAG, "cgroup v2: cannot enable cpuset in root "
                             "subtree_control ({})", strerror(err));
            return false;
        }
    }

    cleanupStaleV2Cgroups();
    const std::string rt = v2RtDirForSuffix(v2_name_suffix_);
    const std::string hk = v2HkDirForSuffix(v2_name_suffix_);
    const bool hk_ok = makeEmptyCgroupDir(hk);
    const bool rt_ok = hk_ok && makeEmptyCgroupDir(rt);
    v2_dirs_created_ = hk_ok;
    if (!hk_ok || !rt_ok) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot create {} / {}", hk, rt);
        teardownCpusetV2();
        return false;
    }

    int err = 0;
    if (!writeFile((hk + "/cgroup.type").c_str(), "threaded", &err) ||
        !writeFile((rt + "/cgroup.type").c_str(), "threaded", &err)) {
        TETHER_LOGW(TAG, "v2 cpuset: cannot switch cgroups to threaded mode ({})",
                    strerror(err));
        teardownCpusetV2();
        return false;
    }
    if (!updateCpusetV2Masks()) {
        teardownCpusetV2();
        return false;
    }
    if (!migrateProcessToV2Housekeeping()) {
        teardownCpusetV2();
        return false;
    }

    active_cpuset_backend_ = CpusetBackend::CgroupV2;
    TETHER_LOGI(TAG, "v2 cpuset: threaded isolated partition {} covers [{}]; "
                     "housekeeping threads confined to complement",
                baseName(rt), formatCpuList(cpuset_cpus_));
    return true;
#else
    return false;
#endif
}

void CpuIsolation::teardownCpusetV2() {
#ifdef __linux__
    if (!v2_dirs_created_) return;
    const std::string rt = v2RtDirForSuffix(v2_name_suffix_);
    const std::string hk = v2HkDirForSuffix(v2_name_suffix_);

    // Move still-member RT threads back to housekeeping before dropping the
    // partition.  Normally RT threads have already exited; this is the net.
    std::string members;
    if (readFileAll((rt + "/cgroup.threads").c_str(), members)) {
        std::istringstream in(members);
        std::string tid;
        while (in >> tid)
            writeFile((hk + "/cgroup.threads").c_str(), tid);
    }

    // Even if directory removal fails, do not leave CPUs isolated forever.
    writeFile((rt + "/cpuset.cpus.partition").c_str(), "member");

    // If process restoration fails, at least stop confining its threads.
    writeFile((hk + "/cpuset.cpus").c_str(), formatCpuList(online_));

    if (v2_process_migrated_) {
        std::string target = v2_original_cgroup_.empty() ? "/" : v2_original_cgroup_;
        std::string target_dir = target == "/" ? "/sys/fs/cgroup"
                                               : "/sys/fs/cgroup" + target;
        const std::string pid = std::to_string(owner_pid_);
        int err = 0;
        if (!writeFile((target_dir + "/cgroup.procs").c_str(), pid, &err)) {
            TETHER_LOGW(TAG, "v2 cpuset: cannot restore process to {} ({}); trying root",
                        target_dir, strerror(err));
            writeFile("/sys/fs/cgroup/cgroup.procs", pid);
        }
        v2_process_migrated_ = false;
    }

    if (::rmdir(rt.c_str()) != 0 && errno != ENOENT)
        TETHER_LOGW(TAG, "v2 cpuset: cleanup of {} failed ({})", rt, strerror(errno));
    if (::rmdir(hk.c_str()) != 0 && errno != ENOENT)
        TETHER_LOGW(TAG, "v2 cpuset: cleanup of {} failed ({})", hk, strerror(errno));
    v2_dirs_created_ = false;
    v2_original_cgroup_.clear();
    if (active_cpuset_backend_ == CpusetBackend::CgroupV2)
        active_cpuset_backend_.reset();
    cpuset_cpus_.clear();
    TETHER_LOGI(TAG, "v2 cpuset partition torn down");
#endif
}

// ============================================================================
// legacy cgroup v1 backend
// ============================================================================

bool CpuIsolation::updateCpusetV1Masks() {
#ifdef __linux__
    const std::string dir = "/sys/fs/cgroup/cpuset/tether_rt";
    std::vector<int> claimed;
    for (int c : online_) {
        const uint64_t bit = 1ULL << (c & 63);
        if ((claimed_[c >> 6] & bit) && !(cpuset_excluded_[c >> 6] & bit))
            claimed.push_back(c);
    }
    if (claimed.empty()) return false;
    const std::string cpus = formatCpuList(claimed);

    int err = 0;
    std::string mems;
    if (!readFile("/sys/fs/cgroup/cpuset/cpuset.mems", mems) ||
        trim(mems).empty()) {
        mems = "0";
    }
    if (!writeFile((dir + "/cpuset.mems").c_str(), trim(mems), &err)) {
        TETHER_LOGW(TAG, "v1 cpuset: cannot assign mems [{}] ({})", trim(mems),
                    strerror(err));
        return false;
    }
    if (!writeFile((dir + "/cpuset.cpus").c_str(), cpus, &err)) {
        TETHER_LOGW(TAG, "v1 cpuset: cannot assign CPUs [{}] ({})", cpus,
                    strerror(err));
        return false;
    }
    if (!writeFile((dir + "/cpuset.cpu_exclusive").c_str(), "1", &err)) {
        TETHER_LOGW(TAG, "v1 cpuset: cannot enable cpu_exclusive ({})",
                    strerror(err));
        return false;
    }
    cpuset_cpus_ = claimed;
    return true;
#else
    return false;
#endif
}

bool CpuIsolation::createCpusetV1(int cpu) {
#ifdef __linux__
    (void)cpu;
    if (geteuid() != 0) return false;    // cpuset ops need root/CAP_SYS_ADMIN
    if (!cpuset_v1_probed_) {
        cpuset_v1_probed_ = true;
        struct stat st{};
        cpuset_v1_ok_ =
            ::stat("/sys/fs/cgroup/cpuset/cpuset.cpus", &st) == 0;
        if (!cpuset_v1_ok_)
            TETHER_LOGW(TAG, "cgroup v1 cpuset not mounted — legacy backend "
                             "unavailable");
    }
    if (!cpuset_v1_ok_) return false;

    const std::string dir = "/sys/fs/cgroup/cpuset/tether_rt";
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        TETHER_LOGW(TAG, "v1 cpuset: mkdir {} failed ({})", dir,
                    strerror(errno));
        return false;
    }
    if (!updateCpusetV1Masks()) return false;

    // Keep OUR non-RT threads off the claimed CPUs too.  This is only a
    // process-local mitigation: v1 cpu_exclusive does not remove CPUs from
    // the parent/root cpuset, so it is not a complete system shield.
    cpu_set_t keep;
    CPU_ZERO(&keep);
    bool any = false;
    for (int c : online_) {
        if (claimed_[c >> 6] & (1ULL << (c & 63))) continue;
        CPU_SET(c, &keep);
        any = true;
    }
    if (any)
        sched_setaffinity(0, sizeof(keep), &keep);

    active_cpuset_backend_ = CpusetBackend::CgroupV1;
    TETHER_LOGI(TAG, "v1 cpuset cgroup 'tether_rt' exclusively covers [{}] "
                     "(legacy; root-cpuset tasks are not fully shielded)",
                formatCpuList(cpuset_cpus_));
    return true;
#else
    return false;
#endif
}

void CpuIsolation::teardownCpusetV1() {
#ifdef __linux__
    const std::string dir = "/sys/fs/cgroup/cpuset/tether_rt";
    if (!pathExists(dir)) {
        if (active_cpuset_backend_ == CpusetBackend::CgroupV1)
            active_cpuset_backend_.reset();
        return;
    }

    // Move member threads back to the root cpuset — rmdir fails EBUSY while
    // tasks remain.  Read the full multi-line tasks file.
    std::string members;
    if (readFileAll((dir + "/tasks").c_str(), members)) {
        std::istringstream in(members);
        std::string tid;
        while (in >> tid)
            writeFile("/sys/fs/cgroup/cpuset/tasks", tid);
    }

    // Restore the calling thread's affinity.
    cpu_set_t all;
    CPU_ZERO(&all);
    for (int c : online_) CPU_SET(c, &all);
    sched_setaffinity(0, sizeof(all), &all);

    if (::rmdir(dir.c_str()) != 0 && errno != ENOENT)
        TETHER_LOGW(TAG, "v1 cpuset: cleanup of {} failed ({})", dir,
                    strerror(errno));
    else
        TETHER_LOGI(TAG, "v1 cpuset cgroup 'tether_rt' removed");
    if (active_cpuset_backend_ == CpusetBackend::CgroupV1)
        active_cpuset_backend_.reset();
    cpuset_cpus_.clear();
#endif
}

bool CpuIsolation::updateActiveCpusetMasks() {
    if (!active_cpuset_backend_) return false;
    if (*active_cpuset_backend_ == CpusetBackend::CgroupV2)
        return updateCpusetV2Masks();
    if (*active_cpuset_backend_ == CpusetBackend::CgroupV1)
        return updateCpusetV1Masks();
    return false;
}

// ============================================================================
// IRQ steering
// ============================================================================

void CpuIsolation::steerIrqsOffClaims() {
#ifdef __linux__
    if (geteuid() != 0) {
        if (!irq_privilege_warning_emitted_) {
            irq_privilege_warning_emitted_ = true;
            TETHER_LOGW(TAG, "IRQ steering needs root — skipped");
        }
        return;
    }
    // Keep IRQs off foreign isolated partitions as well as our own claims.
    // Otherwise two independent Tether processes isolating different CPUs
    // would repeatedly steer interrupts onto each other's RT cores.  Older
    // kernels may not expose cpuset.cpus.isolated; then this reduces to the
    // process-local claim set exactly as before.
    const auto kernel_isolated =
        parseCpuList("/sys/fs/cgroup/cpuset.cpus.isolated");
    std::vector<int> housekeeping;
    for (int c : online_) {
        if (claimed_[c >> 6] & (1ULL << (c & 63))) continue;
        if (listContains(kernel_isolated, c)) continue;
        housekeeping.push_back(c);
    }
    const std::string mask = formatCpuList(housekeeping);
    if (mask.empty()) {
        TETHER_LOGW(TAG, "IRQ steering: no non-isolated housekeeping CPU available");
        return;
    }

    DIR* d = ::opendir("/proc/irq");
    if (!d) {
        TETHER_LOGW(TAG, "IRQ steering: cannot open /proc/irq — skipped");
        return;
    }
    struct dirent* de;
    int steered = 0, failed = 0, seen = 0;
    while ((de = ::readdir(d))) {
        if (de->d_name[0] < '0' || de->d_name[0] > '9') continue;
        ++seen;
        const int irq = std::atoi(de->d_name);
        const std::string path =
            std::string("/proc/irq/") + de->d_name + "/smp_affinity_list";
        std::string orig;
        if (!readFile(path.c_str(), orig)) continue;
        bool known = false;
        for (const auto& p : irq_orig_masks_)
            if (p.first == irq) { known = true; break; }
        if (!known) irq_orig_masks_.emplace_back(irq, orig);
        if (writeFile(path.c_str(), mask)) ++steered;
        else ++failed;   // per-IRQ restrictions — best-effort
    }
    ::closedir(d);
    irq_steering_active_ = steered > 0;
    if (steered == 0) {
        if (failed > 0) {
            TETHER_LOGW(TAG, "IRQ steering: unable to steer any IRQ ({} rejected)",
                        failed);
        } else {
            TETHER_LOGW(TAG, "IRQ steering: no steerable IRQ affinity entries found "
                            "({} entries scanned)", seen);
        }
    } else if (failed > 0) {
        TETHER_LOGW(TAG, "IRQ steering: partial coverage — {} IRQ(s) pinned to [{}], {} rejected",
                    steered, mask, failed);
    } else {
        TETHER_LOGI(TAG, "IRQ steering: {} IRQ(s) pinned to [{}]",
                    steered, mask);
    }
#endif
}

// ============================================================================
// Thread adoption
// ============================================================================

bool CpuIsolation::adoptCurrentThread(int cpu) {
#ifdef __linux__
    if (owner_pid_ >= 0 && owner_pid_ != static_cast<int>(::getpid()))
        return false;
    std::lock_guard<std::mutex> lk(mu_);
    if (cpu < 0 || !listContains(cpuset_cpus_, cpu) ||
        !active_cpuset_backend_) return false;

    std::string path;
    std::string expected;
    if (*active_cpuset_backend_ == CpusetBackend::CgroupV2) {
        const std::string rt = v2RtDirForSuffix(v2_name_suffix_);
        path = rt + "/cgroup.threads";
        expected = "/" + baseName(rt);
    } else {
        path = "/sys/fs/cgroup/cpuset/tether_rt/tasks";
        expected = "tether_rt";
    }

    const long tid = static_cast<long>(::syscall(SYS_gettid));
    int err = 0;
    if (!writeFile(path.c_str(), std::to_string(tid), &err)) {
        TETHER_LOGW(TAG, "cpuset: cannot move tid {} into {} ({})",
                    tid, path, strerror(err));
        return false;
    }

    // Validate the thread-visible membership.  A successful cgroup write is
    // necessary but not sufficient evidence in delegated/namespaced setups.
    std::string cgroup_text;
    readFileAll("/proc/thread-self/cgroup", cgroup_text);
    const std::string v2_path = ownCgroupV2Path();
    const bool ok = *active_cpuset_backend_ == CpusetBackend::CgroupV2
        ? endsWith(v2_path, expected)
        : cgroup_text.find(expected) != std::string::npos;
    if (!ok) {
        TETHER_LOGW(TAG, "cpuset: tid {} write succeeded but membership is '{}'",
                    tid, trim(cgroup_text));
        return false;
    }
    TETHER_LOGI(TAG, "thread {} joined {} cpuset", tid,
                backendName(*active_cpuset_backend_));
    return true;
#else
    (void)cpu;
    return false;
#endif
}

// ============================================================================
// release / cleanup
// ============================================================================

void CpuIsolation::teardownPartitions() {
    // Never evict a foreign legacy tether_rt cgroup that this process did
    // not actually use; the fixed v1 name is visible system-wide.
    if (v2_dirs_created_ || active_cpuset_backend_ == CpusetBackend::CgroupV2)
        teardownCpusetV2();
    if (active_cpuset_backend_ == CpusetBackend::CgroupV1)
        teardownCpusetV1();
    cpuset_cpus_.clear();
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
#ifdef __linux__
    if (owner_pid_ >= 0 && owner_pid_ != static_cast<int>(::getpid())) return;
#endif
    std::lock_guard<std::mutex> lk(mu_);
    if (cpu < 0 || cpu >= 256) return;
    if (!(claimed_[cpu >> 6] & (1ULL << (cpu & 63)))) return;
    claimed_[cpu >> 6] &= ~(1ULL << (cpu & 63));
    cpuset_excluded_[cpu >> 6] &= ~(1ULL << (cpu & 63));
    TETHER_LOGI(TAG, "released CPU {}", cpu);

    bool any = false;
    for (int c : online_)
        if (claimed_[c >> 6] & (1ULL << (c & 63))) { any = true; break; }
    if (any) {
        if (active_cpuset_backend_) {
            bool cpuset_any = false;
            for (int c : online_) {
                const uint64_t bit = 1ULL << (c & 63);
                if ((claimed_[c >> 6] & bit) &&
                    !(cpuset_excluded_[c >> 6] & bit)) {
                    cpuset_any = true;
                    break;
                }
            }
            if (cpuset_any) {
                if (!updateActiveCpusetMasks())
                    TETHER_LOGW(TAG, "failed to shrink active cpuset after CPU {} release",
                                cpu);
            } else {
                // Only affinity-only claims remain (for example after a
                // failed extension); keeping the old partition would isolate
                // a CPU whose RT claim has already been released.
                teardownPartitions();
            }
        }
        if (irq_steering_active_) steerIrqsOffClaims();
    } else {
        teardownPartitions();
        restoreIrqMasks();
    }
}

void CpuIsolation::releaseAll() {
#ifdef __linux__
    // atexit handlers are inherited across fork().  Claims and the v2
    // cgroup names belong to the process that created them; a forked child
    // must never tear down or migrate its parent's cgroup state.
    if (owner_pid_ >= 0 && owner_pid_ != static_cast<int>(::getpid())) return;
#endif
    std::lock_guard<std::mutex> lk(mu_);
    int released = 0;
    for (int c : online_) {
        if (claimed_[c >> 6] & (1ULL << (c & 63))) {
            claimed_[c >> 6] &= ~(1ULL << (c & 63));
            ++released;
        }
    }
    for (auto& word : cpuset_excluded_) word = 0;
    if (released > 0)
        TETHER_LOGI(TAG, "released {} CPU claim(s) on exit", released);
    teardownPartitions();
    restoreIrqMasks();
}

} // namespace Platform
} // namespace Tether
