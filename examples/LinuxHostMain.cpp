/**
 * @file LinuxHostMain.cpp
 * @brief Linux host program for EtherCAT master with FIFO scheduling
 *
 * This program demonstrates using the HAL on Linux with real-time scheduling.
 * It requires CAP_SYS_NICE capability or root privileges to set SCHED_FIFO.
 *
 * Build:
 *   cd host_tests/linux_host && mkdir build && cd build
 *   cmake .. && make
 *
 * Run (as root or with capabilities):
 *   sudo ./ethercat_host eth0
 *   # or with capabilities:
 *   sudo setcap cap_net_raw,cap_sys_nice+ep ./ethercat_host
 *   ./ethercat_host eth0
 */

#ifdef __linux__

#include "hal/HAL.hpp"
#include "packetloggers/pcap/PCAPLogger.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/utils/SignalHandler.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <memory>
#include <magic_enum/magic_enum.hpp>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <getopt.h>
#include <chrono>
#include <thread>

using namespace EtherCAT::HAL;

// ============================================================================
// Global State
// ============================================================================

static std::atomic<bool> g_running{true};
static std::atomic<uint64_t> g_cycleCount{0};
static std::atomic<uint64_t> g_txCount{0};
static std::atomic<uint64_t> g_rxCount{0};
static std::atomic<uint64_t> g_maxLatencyUs{0};

// ============================================================================
// Real-Time Setup
// ============================================================================

struct RTConfig {
    int schedulerPolicy = SCHED_FIFO;
    int priority = 80;  // High but not max
    bool lockMemory = true;
    size_t preAllocStackKB = 512;
    int cpuAffinity = -1;  // -1 = no affinity
};

static bool setupRealtime(const RTConfig& config) {
    // Set scheduler policy and priority
    struct sched_param param;
    param.sched_priority = config.priority;
    
    if (sched_setscheduler(0, config.schedulerPolicy, &param) != 0) {
        std::perror("Warning: Failed to set SCHED_FIFO (run as root or with CAP_SYS_NICE)");
        // Continue anyway - not fatal
    } else {
        std::printf("Set scheduler to SCHED_FIFO with priority %d\n", config.priority);
    }
    
    // Lock memory to prevent paging
    if (config.lockMemory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            std::perror("Warning: Failed to lock memory");
            // Continue anyway
        } else {
            std::printf("Memory locked\n");
        }
    }
    
    // Set CPU affinity if specified
    if (config.cpuAffinity >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(config.cpuAffinity, &cpuset);
        
        if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0) {
            std::perror("Warning: Failed to set CPU affinity");
        } else {
            std::printf("CPU affinity set to core %d\n", config.cpuAffinity);
        }
    }
    
    return true;
}

// ============================================================================
// EtherCAT Frame Building
// ============================================================================

static constexpr uint16_t ETHERCAT_ETHERTYPE = 0x88A4;

// Build a simple BRD (Broadcast Read) datagram
static size_t buildBRDFrame(uint8_t* frame, const MacAddress& srcMac) {
    // Ethernet header (14 bytes)
    // Destination: broadcast
    frame[0] = 0xFF; frame[1] = 0xFF; frame[2] = 0xFF;
    frame[3] = 0xFF; frame[4] = 0xFF; frame[5] = 0xFF;
    // Source
    std::memcpy(frame + 6, srcMac.bytes, 6);
    // EtherType
    frame[12] = ETHERCAT_ETHERTYPE >> 8;
    frame[13] = ETHERCAT_ETHERTYPE & 0xFF;
    
    // EtherCAT header (2 bytes)
    uint16_t ecatHeader = (12 << 0) | (0x01 << 11);
    frame[14] = ecatHeader & 0xFF;
    frame[15] = ecatHeader >> 8;
    
    // EtherCAT datagram
    frame[16] = 0x07;  // BRD command
    frame[17] = 0x00;  // Index
    frame[18] = 0x00; frame[19] = 0x00;  // ADP
    frame[20] = 0x00; frame[21] = 0x00;  // ADO
    uint16_t lenField = 2 | (1 << 14);
    frame[22] = lenField & 0xFF;
    frame[23] = lenField >> 8;
    frame[24] = 0x00; frame[25] = 0x00;  // IRQ
    frame[26] = 0x00; frame[27] = 0x00;  // Data
    frame[28] = 0x00; frame[29] = 0x00;  // WKC
    
    return 30;
}

// ============================================================================
// Cyclic Task
// ============================================================================

struct CyclicTaskContext {
    HALInstance* hal;
    uint8_t txFrame[1518];
    size_t txFrameLen;
    uint8_t rxBuffer[1518];
};

static void runCyclicTask(CyclicTaskContext* ctx) {
    auto startUs = ctx->hal->nowMicros();
    
    // Send frame
    Error err = ctx->hal->ethernet().transmit(ctx->txFrame, ctx->txFrameLen);
    if (err == Error::OK) {
        g_txCount++;
    }
    
    // Poll for response
    int rxCount = ctx->hal->ethernet().poll(1);  // 1ms timeout
    if (rxCount > 0) {
        g_rxCount += rxCount;
    }
    
    auto endUs = ctx->hal->nowMicros();
    auto latencyUs = endUs - startUs;
    
    // Update max latency
    uint64_t maxLat = g_maxLatencyUs.load();
    while (latencyUs > maxLat && !g_maxLatencyUs.compare_exchange_weak(maxLat, latencyUs)) {
        // Retry
    }
    
    g_cycleCount++;
}

// ============================================================================
// Statistics
// ============================================================================

static void printStats(Timestamp startTimeUs) {
    auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    double elapsed = static_cast<double>(nowUs - startTimeUs) / 1e6;
    
    uint64_t cycles = g_cycleCount.load();
    uint64_t tx = g_txCount.load();
    uint64_t rx = g_rxCount.load();
    uint64_t maxLat = g_maxLatencyUs.load();
    
    double cyclesPerSec = cycles / elapsed;
    double lossRate = tx > 0 ? 100.0 * (tx - rx) / tx : 0.0;
    
    std::printf("\r[%.1fs] Cycles: %lu (%.1f/s) TX: %lu RX: %lu Loss: %.2f%% MaxLat: %lu us   ",
                elapsed, cycles, cyclesPerSec, tx, rx, lossRate, maxLat);
    std::fflush(stdout);
}

// ============================================================================
// Command Line Options
// ============================================================================

struct Options {
    const char* interface = nullptr;
    uint32_t cycleFrequencyHz = 1000;  // 1 kHz default
    bool enablePcap = false;
    const char* pcapFile = "ethercat.pcapng";
    bool enableVlan = false;
    uint16_t vlanId = 0;
    int rtPriority = 80;
    int cpuAffinity = -1;
    bool verbose = false;
};

static void printUsage(const char* progName) {
    std::printf("Usage: %s [options] <interface>\n", progName);
    std::printf("\n");
    std::printf("Options:\n");
    std::printf("  -i, --interface <if>  Network interface name (overrides positional <interface>)\n");
    std::printf("  -f, --freq <Hz>       Cycle frequency in Hz (default: 1000)\n");
    std::printf("  -p, --pcap <file>     Enable PcapNG logging to file\n");
    std::printf("  -v, --vlan <id>       Use VLAN with specified ID\n");
    std::printf("  -r, --rtprio <prio>   Real-time priority (default: 80)\n");
    std::printf("  -a, --affinity <cpu>  CPU affinity (default: none)\n");
    std::printf("  -V, --verbose         Verbose output\n");
    std::printf("  -h, --help            Show this help\n");
    std::printf("\n");
    std::printf("Example:\n");
    std::printf("  sudo %s -f 500 -p trace.pcapng -i eth0\n", progName);
}

static bool parseOptions(int argc, char* argv[], Options& opts) {
    static struct option longOpts[] = {
        {"interface", required_argument, nullptr, 'i'},
        {"freq",     required_argument, nullptr, 'f'},
        {"pcap",     required_argument, nullptr, 'p'},
        {"vlan",     required_argument, nullptr, 'v'},
        {"rtprio",   required_argument, nullptr, 'r'},
        {"affinity", required_argument, nullptr, 'a'},
        {"verbose",  no_argument,       nullptr, 'V'},
        {"help",     no_argument,       nullptr, 'h'},
        {nullptr,    0,                 nullptr, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "i:f:p:v:r:a:Vh", longOpts, nullptr)) != -1) {
        switch (opt) {
            case 'i':
                opts.interface = optarg;
                break;
            case 'f':
                opts.cycleFrequencyHz = std::atoi(optarg);
                break;
            case 'p':
                opts.enablePcap = true;
                opts.pcapFile = optarg;
                break;
            case 'v':
                opts.enableVlan = true;
                opts.vlanId = std::atoi(optarg);
                break;
            case 'r':
                opts.rtPriority = std::atoi(optarg);
                break;
            case 'a':
                opts.cpuAffinity = std::atoi(optarg);
                break;
            case 'V':
                opts.verbose = true;
                break;
            case 'h':
                printUsage(argv[0]);
                return false;
            default:
                return false;
        }
    }
    
    if (optind >= argc) {
        std::fprintf(stderr, "Error: Interface name required\n");
        printUsage(argv[0]);
        return false;
    }
    
    opts.interface = argv[optind];
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    Options opts;
    if (!parseOptions(argc, argv, opts)) {
        return 1;
    }
    
    std::printf("EtherCAT Linux Host (HAL-based)\n");
    std::printf("Interface: %s\n", opts.interface);
    std::printf("Cycle frequency: %u Hz\n", opts.cycleFrequencyHz);
    
    // Setup signal handlers
    Tether::Utils::SignalHandler sig_handler(g_running, false);

    // Ensure kernel supports realtime (warns if low-latency desktop, exits if below)
    Tether::Platform::ensureRealtimeKernelOrExit();

    // Setup real-time
    RTConfig rtConfig;
    rtConfig.priority = opts.rtPriority;
    rtConfig.cpuAffinity = opts.cpuAffinity;
    setupRealtime(rtConfig);
    
    // Configure HAL
    HALConfig halConfig;
    halConfig.ethernet.interfaceName = opts.interface;
    halConfig.ethernet.promiscuous = true;
    halConfig.ethernet.ethertypeFilter = ETHERCAT_ETHERTYPE;
    halConfig.enablePacketLogging = opts.enablePcap;
    if (opts.enablePcap) {
        halConfig.pcapConfig.filename = opts.pcapFile;
        halConfig.createPacketLogger = [](const Tether::PacketLoggers::PCAP::PCAPLoggerConfig&) {
            return Tether::PacketLoggers::PCAP::createPCAPLogger();
        };
    }
    halConfig.enableVlan = opts.enableVlan;
    halConfig.vlanId = opts.vlanId;
    halConfig.useRealtimeScheduling = true;
    halConfig.realtimePriority = opts.rtPriority;
    
    // Initialize HAL
    Error err = initHAL(halConfig);
    if (err != Error::OK) {
        std::fprintf(stderr, "Error: Failed to initialize HAL: %s\n", magic_enum::enum_name(err).data());
        return 1;
    }
    
    HALInstance& hal = getHAL();
    
    // Wait for link
    std::printf("Waiting for link...\n");
    for (int i = 0; i < 50; i++) {
        LinkStatus status = hal.ethernet().getLinkStatus();
        if (status.up) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    LinkStatus linkStatus = hal.ethernet().getLinkStatus();
    if (!linkStatus.up) {
        std::fprintf(stderr, "Error: Link not up\n");
        shutdownHAL();
        return 1;
    }
    
    MacAddress mac;
    hal.ethernet().getMacAddress(mac);
    std::printf("Link up at %u Mbps, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", 
                linkStatus.speedMbps,
                mac.bytes[0], mac.bytes[1], mac.bytes[2],
                mac.bytes[3], mac.bytes[4], mac.bytes[5]);
    
    // Setup cyclic task context
    CyclicTaskContext ctx;
    ctx.hal = &hal;
    ctx.txFrameLen = buildBRDFrame(ctx.txFrame, mac);
    
    // Create periodic timer
    auto timer = hal.createPeriodicTimer();
    if (!timer->init(opts.cycleFrequencyHz)) {
        std::fprintf(stderr, "Error: Failed to initialize timer\n");
        shutdownHAL();
        return 1;
    }
    
    auto startTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    auto lastStatsPrint = startTimeUs;
    
    // RX callback to count received frames
    hal.ethernet().setRxCallback([](const uint8_t*, size_t, const RxFrameInfo&, void*) {
        // Counting done in poll()
    }, nullptr);
    
    // Start timer
    timer->start();
    
    std::printf("Running at %u Hz (Ctrl+C to stop)...\n", opts.cycleFrequencyHz);
    
    // Main loop
    while (g_running) {
        timer->waitForCycle();
        runCyclicTask(&ctx);
        
        auto nowUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (nowUs - lastStatsPrint >= 500000) {  // Every 500ms
            printStats(startTimeUs);
            lastStatsPrint = nowUs;
        }
    }
    
    std::printf("\n\nStopping...\n");
    
    // Stop timer
    timer->stop();
    
    // Final stats
    printStats(startTimeUs);
    std::printf("\n");
    
    // Shutdown HAL
    shutdownHAL();
    
    std::printf("Done.\n");
    return 0;
}

#else
// Non-Linux stub
#include <cstdio>
int main() {
    std::fprintf(stderr, "This program only runs on Linux\n");
    return 1;
}
#endif // __linux__
