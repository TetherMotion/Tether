/**
 * @file cpu_isolation_test.cpp
 * @brief Empirical scheduler-isolation test for Tether::Platform::CpuIsolation.
 *
 * The test deliberately uses an adversarial EXTERNAL process.  It pins that
 * process at the claimed CPU and measures (a) whether the process can actually
 * execute there and (b) how much throughput the realtime worker loses.  The
 * noise process is forked before the cpuset is created so it remains outside
 * Tether's cgroup hierarchy.
 */

#include "tether/platform/CpuIsolation.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <sched.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using Tether::Platform::CpuIsolation;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kMaxCpus = 256;

struct Options {
    int cpu = -1;
    int baseline_ms = 700;
    int contend_ms = 1200;
    int cooldown_ms = 500;
    bool create_cpuset = true;
    bool steer_irqs = true;  // default: automatic best-effort steering
    bool report_only = false;
    CpuIsolation::CpusetBackend backend = CpuIsolation::CpusetBackend::Auto;
};

struct NoiseResult {
    int pin_rc = -1;
    int pin_errno = 0;
    unsigned long long iterations = 0;
    unsigned long long samples[kMaxCpus] = {};
    char cgroup[160] = {};
};

struct Burner {
    std::atomic<unsigned long long> iterations{0};
    std::atomic<unsigned long long> samples[kMaxCpus];
    std::string cgroup;
    std::atomic<bool> sample_enabled{true};

    Burner() {
        for (int i = 0; i < kMaxCpus; ++i) samples[i].store(0);
    }

    void run() {
        unsigned long long local = 0;
        unsigned long long x = 0x9e3779b97f4a7c15ULL;
        while (!stop.load(std::memory_order_relaxed)) {
            for (int i = 0; i < 1024; ++i) {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                ++local;
            }
            iterations.store(local, std::memory_order_relaxed);
            if (sample_enabled.load(std::memory_order_relaxed)) {
                const int c = sched_getcpu();
                if (c >= 0 && c < kMaxCpus)
                    samples[c].fetch_add(1, std::memory_order_relaxed);
            }
            if (x == 42) std::cerr << "";   // keep the integer loop alive
        }
        iterations.store(local, std::memory_order_relaxed);
        cgroup = readFirstLine("/proc/thread-self/cgroup");
    }

    std::atomic<bool> stop{false};

    static std::string readFirstLine(const char* path) {
        std::ifstream f(path);
        std::string s;
        std::getline(f, s);
        return s;
    }
};

std::string firstLine(const std::string& path) {
    std::ifstream f(path);
    std::string s;
    std::getline(f, s);
    return s;
}

bool exists(const std::string& path) {
    return access(path.c_str(), F_OK) == 0;
}

bool cgroupWithPrefixExists(const std::string& prefix) {
    DIR* d = opendir("/sys/fs/cgroup");
    if (!d) return false;
    bool found = false;
    while (dirent* de = readdir(d)) {
        if (prefix == de->d_name ||
            std::string(de->d_name).rfind(prefix, 0) == 0) {
            found = true;
            break;
        }
    }
    closedir(d);
    return found;
}

std::string affinityString() {
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) != 0) return "<error>";
    std::string out;
    for (int i = 0; i < CPU_SETSIZE && i < kMaxCpus; ++i) {
        if (!CPU_ISSET(i, &set)) continue;
        if (!out.empty()) out += ',';
        out += std::to_string(i);
    }
    return out.empty() ? "<empty>" : out;
}

std::string joinMask(const std::atomic<unsigned long long>* samples) {
    std::string out;
    for (int i = 0; i < kMaxCpus; ++i) {
        const auto n = samples[i].load();
        if (!n) continue;
        if (!out.empty()) out += ' ';
        out += "cpu" + std::to_string(i) + "=" + std::to_string(n);
    }
    return out.empty() ? "<none>" : out;
}

std::string joinMask(const unsigned long long* samples) {
    std::string out;
    for (int i = 0; i < kMaxCpus; ++i) {
        if (!samples[i]) continue;
        if (!out.empty()) out += ' ';
        out += "cpu" + std::to_string(i) + "=" + std::to_string(samples[i]);
    }
    return out.empty() ? "<none>" : out;
}

bool writeAll(int fd, const void* data, size_t size) {
    const char* p = static_cast<const char*>(data);
    while (size) {
        const ssize_t n = write(fd, p, size);
        if (n <= 0) return false;
        p += n;
        size -= static_cast<size_t>(n);
    }
    return true;
}

bool readAll(int fd, void* data, size_t size) {
    char* p = static_cast<char*>(data);
    while (size) {
        const ssize_t n = read(fd, p, size);
        if (n <= 0) return false;
        p += n;
        size -= static_cast<size_t>(n);
    }
    return true;
}

double measureRate(Burner& burner, int ms) {
    const auto t0 = Clock::now();
    const auto c0 = burner.iterations.load(std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    const auto t1 = Clock::now();
    const auto c1 = burner.iterations.load(std::memory_order_relaxed);
    const double sec = std::chrono::duration<double>(t1 - t0).count();
    return sec > 0.0 ? static_cast<double>(c1 - c0) / sec : 0.0;
}

void burnNoise(int cpu, int duration_ms, int result_fd) {
    NoiseResult r;
    cpu_set_t one;
    CPU_ZERO(&one);
    CPU_SET(cpu, &one);
    r.pin_rc = sched_setaffinity(0, sizeof(one), &one);
    r.pin_errno = r.pin_rc == 0 ? 0 : errno;

    unsigned long long x = 0x243f6a8885a308d3ULL;
    const auto deadline = Clock::now() + std::chrono::milliseconds(duration_ms);
    while (Clock::now() < deadline) {
        for (int i = 0; i < 1024; ++i) {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            ++r.iterations;
        }
        const int c = sched_getcpu();
        if (c >= 0 && c < kMaxCpus) ++r.samples[c];
        if (x == 42) _exit(90);   // impossible; keeps the loop observable
    }

    std::string cg = Burner::readFirstLine("/proc/thread-self/cgroup");
    strncpy(r.cgroup, cg.c_str(), sizeof(r.cgroup) - 1);
    writeAll(result_fd, &r, sizeof(r));
    _exit(0);
}

void usage(const char* argv0) {
    std::cout <<
        "usage: " << argv0 << " [options]\n"
        "  --cpu N              exact CPU to claim (default: auto-pick)\n"
        "  --backend auto|v2|v1 cpuset backend (default: auto = v2, then v1)\n"
        "  --baseline-ms N      uncontended measurement window (700)\n"
        "  --contend-ms N       adversarial overlap window (1200)\n"
        "  --cooldown-ms N      post-noise measurement window (500)\n"
        "  --no-cpuset          disable cpuset creation (affinity-only control)\n"
        "  --steer-irqs         explicitly enable IRQ steering (default: auto/best-effort)\n"
        "  --no-steer-irqs      disable IRQ steering/restoration\n"
        "  --report-only        always exit 0 after printing the verdict\n"
        "  -h, --help           this help\n"
        "\nExit 0 means external noise executed zero times on the claimed CPU.\n"
        "Exit 2 means interference was observed or no hard isolation existed.\n";
}

bool parseBackend(const std::string& s, CpuIsolation::CpusetBackend& out) {
    using B = CpuIsolation::CpusetBackend;
    if (s == "auto") { out = B::Auto; return true; }
    if (s == "v2" || s == "cgroup-v2") { out = B::CgroupV2; return true; }
    if (s == "v1" || s == "cgroup-v1") { out = B::CgroupV1; return true; }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << "\n";
                exit(64);
            }
            return argv[++i];
        };
        if (a == "--cpu") opt.cpu = std::atoi(need_value("--cpu"));
        else if (a == "--backend") {
            const std::string b = need_value("--backend");
            if (!parseBackend(b, opt.backend)) {
                std::cerr << "unknown backend '" << b << "'\n";
                return 64;
            }
        }
        else if (a == "--baseline-ms") opt.baseline_ms = std::atoi(need_value("--baseline-ms"));
        else if (a == "--contend-ms") opt.contend_ms = std::atoi(need_value("--contend-ms"));
        else if (a == "--cooldown-ms") opt.cooldown_ms = std::atoi(need_value("--cooldown-ms"));
        else if (a == "--no-cpuset") opt.create_cpuset = false;
        else if (a == "--steer-irqs") opt.steer_irqs = true;
        else if (a == "--no-steer-irqs") opt.steer_irqs = false;
        else if (a == "--report-only") opt.report_only = true;
        else if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else {
            std::cerr << "unknown option: " << a << "\n";
            usage(argv[0]);
            return 64;
        }
    }
    if (opt.cpu >= kMaxCpus || opt.baseline_ms <= 0 ||
        opt.contend_ms <= 0 || opt.cooldown_ms < 0) {
        std::cerr << "invalid numeric option\n";
        return 64;
    }

    std::cout << "== cpu_isolation_test ==\n";
    std::cout << "host: pid=" << getpid() << " euid=" << geteuid()
              << " online=" << firstLine("/sys/devices/system/cpu/online")
              << " kernel_isolated=[" << firstLine("/sys/devices/system/cpu/isolated") << "]"
              << " affinity=" << affinityString() << "\n";
    std::cout << "cgroups: v1_cpuset=" << (exists("/sys/fs/cgroup/cpuset") ? "present" : "absent")
              << " v2_controllers=[" << firstLine("/sys/fs/cgroup/cgroup.controllers") << "]"
              << " process=" << Burner::readFirstLine("/proc/thread-self/cgroup") << "\n";

    // Create pipes and fork the adversary BEFORE claiming.  Consequently the
    // noise process is not part of this process's cgroup migration and is a
    // genuine external task.
    int start_pipe[2] = {-1, -1};
    int result_pipe[2] = {-1, -1};
    if (pipe(start_pipe) != 0 || pipe(result_pipe) != 0) {
        std::cerr << "pipe failed: " << strerror(errno) << "\n";
        return 1;
    }
    const pid_t noise_pid = fork();
    if (noise_pid < 0) {
        std::cerr << "fork failed: " << strerror(errno) << "\n";
        return 1;
    }
    if (noise_pid == 0) {
        close(start_pipe[1]);
        close(result_pipe[0]);
        // Protocol: "<cpu>\n<duration-ms>\nX".  The CPU is known only after
        // the parent's claim, so it cannot be supplied before fork.
        std::string proto;
        char ch = 0;
        while (read(start_pipe[0], &ch, 1) == 1) {
            if (ch == 'X') break;
            proto += ch;
            if (proto.size() > 64) _exit(4);
        }
        close(start_pipe[0]);
        const auto nl = proto.find('\n');
        if (nl == std::string::npos) _exit(3);
        burnNoise(std::atoi(proto.substr(0, nl).c_str()),
                  std::atoi(proto.substr(nl + 1).c_str()), result_pipe[1]);
    }

    close(start_pipe[0]);
    close(result_pipe[1]);

    CpuIsolation& iso = CpuIsolation::instance();
    CpuIsolation::Spec spec;
    spec.requested_cpu = opt.cpu;
    spec.create_cpuset = opt.create_cpuset;
    spec.cpuset_backend = opt.backend;
    spec.steer_irqs = opt.steer_irqs;

    // Auto-picking happens in claim().  The already-forked noise child waits
    // for the actual CPU and duration through the start pipe, so auto mode is
    // exact as well.
    auto claim = iso.claim(spec);
    if (!claim.valid()) {
        std::cerr << "ERROR: no CPU claim granted\n";
        const char zero = '0';
        (void)!write(start_pipe[1], &zero, 1);
        kill(noise_pid, SIGKILL);
        waitpid(noise_pid, nullptr, 0);
        return 1;
    }

    std::cout << "claim: cpu=" << claim.cpu
              << " kernel_isolated=" << claim.kernel_isolated
              << " cpuset_isolated=" << claim.cpuset_isolated
              << " backend=" << (claim.cpuset_backend
                                  ? CpuIsolation::backendName(*claim.cpuset_backend)
                                  : "none")
              << " hard_isolated=" << claim.hardIsolated()
              << " irqs_steered=" << claim.irqs_steered << "\n";

    Burner burner;
    std::thread worker([&] {
        cpu_set_t one;
        CPU_ZERO(&one);
        CPU_SET(claim.cpu, &one);
        const bool adopted = iso.adoptCurrentThread(claim.cpu);
        const int pin_rc = sched_setaffinity(0, sizeof(one), &one);
        std::cout << "worker: adopted=" << adopted << " pin_rc=" << pin_rc
                  << " affinity=" << affinityString() << "\n";
        burner.run();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const double baseline = measureRate(burner, opt.baseline_ms);

    // The child was forked before the CPU was known.  It expects CPU and
    // duration as decimal lines terminated by X; write the start command now.
    const std::string noise_spec = std::to_string(claim.cpu) + "\n" +
                                   std::to_string(opt.contend_ms) + "\nX";
    writeAll(start_pipe[1], noise_spec.data(), noise_spec.size());
    close(start_pipe[1]);

    const double contended = measureRate(burner, opt.contend_ms);

    NoiseResult noise;
    int status = 0;
    waitpid(noise_pid, &status, 0);
    const bool noise_read = readAll(result_pipe[0], &noise, sizeof(noise));
    close(result_pipe[0]);

    double cooldown = 0.0;
    if (opt.cooldown_ms > 0) cooldown = measureRate(burner, opt.cooldown_ms);

    burner.stop.store(true);
    worker.join();

    unsigned long long noise_total = 0;
    for (int i = 0; i < kMaxCpus; ++i) noise_total += noise.samples[i];
    const unsigned long long noise_target = noise.samples[claim.cpu];
    const double target_frac = noise_total ?
        static_cast<double>(noise_target) / static_cast<double>(noise_total) : 0.0;
    const double retention = baseline > 0.0 ? contended / baseline : 0.0;
    const double recovery = baseline > 0.0 && cooldown > 0.0 ? cooldown / baseline : 0.0;

    std::cout << "worker: final_cgroup=" << (burner.cgroup.empty() ? "<unread>" : burner.cgroup)
              << " cpu_samples: " << joinMask(burner.samples) << "\n";
    if (!noise_read || !WIFEXITED(status)) {
        std::cerr << "ERROR: noise process did not report cleanly\n";
        iso.release(claim.cpu);
        return 1;
    }
    std::cout << "noise: pin_rc=" << noise.pin_rc
              << (noise.pin_rc ? std::string(" errno=") + strerror(noise.pin_errno) : "")
              << " cgroup=" << noise.cgroup
              << " iterations=" << noise.iterations
              << " cpu_samples: " << joinMask(noise.samples) << "\n";
    std::cout << "throughput: baseline=" << baseline / 1e6
              << "M/s contended=" << contended / 1e6
              << "M/s retention=" << retention * 100.0 << "%";
    if (cooldown > 0.0)
        std::cout << " cooldown=" << cooldown / 1e6 << "M/s recovery=" << recovery * 100.0 << "%";
    std::cout << "\n";

    bool pass = claim.hardIsolated() && noise_target == 0;
    std::cout << "verdict: ";
    if (!claim.hardIsolated()) {
        std::cout << "NOT HARD-ISOLATED";
    } else if (noise_target == 0) {
        std::cout << "SCHEDULER-ISOLATED";
    } else {
        std::cout << "INTERFERENCE";
    }
    std::cout << " (noise_on_target=" << noise_target
              << "/" << noise_total << " fraction=" << target_frac * 100.0 << "%)\n";
    if (pass && retention < 0.90) {
        std::cout << "warning: no on-CPU scheduling interference, but throughput retention "
                  << retention * 100.0 << "% suggests SMT sibling, IRQ, frequency, or "
                     "cache/memory contention\n";
    }

    iso.release(claim.cpu);
    const std::string suffix = std::to_string(getpid()) + "_";
    const bool rt_left = cgroupWithPrefixExists("tether_v2_rt_" + suffix);
    const bool hk_left = cgroupWithPrefixExists("tether_v2_hk_" + suffix);
    std::cout << "cleanup: v2_rt_left=" << rt_left << " v2_hk_left=" << hk_left
              << " process_cgroup=" << Burner::readFirstLine("/proc/thread-self/cgroup")
              << " affinity=" << affinityString()
              << " root_effective=" << firstLine("/sys/fs/cgroup/cpuset.cpus.effective")
              << " root_isolated=" << firstLine("/sys/fs/cgroup/cpuset.cpus.isolated")
              << " active_backend=" << (iso.activeCpusetBackend()
                                         ? CpuIsolation::backendName(*iso.activeCpusetBackend())
                                         : "none")
              << "\n";
    return (pass || opt.report_only) ? 0 : 2;
}
