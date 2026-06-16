/**
 * @file dual_instance_linux.cpp
 * @brief Demonstrates two independent Master instances on Linux
 *
 * This example creates two separate Master objects, each with its own
 * NetworkInterface, queues, and CiA 402 drive.  The two masters run
 * independently on separate threads, proving that the class-based API
 * supports true multi-instance usage.
 *
 * Because real Ethernet hardware is not available in a typical host test
 * environment, the example uses loopback mock adapters. Each adapter
 * simulates a single EtherCAT slave that echoes broadcast reads with
 * wkc = 1 and responds to APRD / APWR.
 *
 * Build (standalone):
 *   g++ -std=c++17 -I ../include -I ../include/tether \
 *       -I ../include/tether/ethercat dual_instance_linux.cpp -lpthread -o dual_instance
 */

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <queue>

// EtherCAT class-based API
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Types.hpp"
#include "tether/platform/EspCompat.hpp"

static const char* TAG = "DualInstance";

// =============================================================================
// Mock Loopback Adapter
// =============================================================================

/**
 * @brief A trivial loopback "Ethernet" adapter.
 *
 * Captures frames sent by the master, synthesises a minimal response
 * (BRD with wkc=1 for slave discovery), and feeds it back via
 * master.handleRxFrame().
 */
class LoopbackAdapter {
public:
    explicit LoopbackAdapter(const char* name) : name_(name) {}

    /// Wire up to a master instance
    EtherCAT::NetworkInterface makeInterface() {
        EtherCAT::NetworkInterface iface{};
        iface.send = [this](const uint8_t* buf, size_t len) -> bool {
            return this->onSend(buf, len);
        };
        return iface;
    }

    /// Set the master that will receive responses
    void setMaster(EtherCAT::Master* m) { master_ = m; }

    /// Number of frames sent through this adapter
    uint32_t txCount() const { return tx_count_.load(); }

private:
    bool onSend(const uint8_t* buf, size_t len) {
        tx_count_.fetch_add(1);

        if (len < 14 + 2 + 10 + 2) return true;  // too short to be useful

        // Build a minimal echo: flip dst/src MAC, set wkc = 1
        uint8_t reply[1518];
        std::memcpy(reply, buf, len);
        // Swap MACs
        std::memcpy(reply, buf + 6, 6);   // dst ← original src
        std::memcpy(reply + 6, buf, 6);   // src ← original dst

        // Set WKC = 1 (last 2 bytes of EtherCAT payload)
        // EtherCAT frame header is at offset 14, payload starts at 16
        // datagram header is 10 bytes, data follows, then WKC (2 bytes)
        const size_t ec_header_offset = 14;
        const size_t dg_offset = ec_header_offset + 2;  // after frame header
        if (len >= dg_offset + 10 + 2) {
            // datagram data length from lenFlags at offset dg+6..7
            uint16_t lenFlags;
            std::memcpy(&lenFlags, reply + dg_offset + 6, 2);
            uint16_t datalen = lenFlags & 0x07FF;
            size_t wkc_off = dg_offset + 10 + datalen;
            if (wkc_off + 2 <= len) {
                uint16_t wkc = 1;
                std::memcpy(reply + wkc_off, &wkc, 2);
            }
        }

        // Feed back to master
        if (master_) {
            master_->handleRxFrame(reply, len);
        }
        return true;
    }

    const char* name_;
    EtherCAT::Master* master_{nullptr};
    std::atomic<uint32_t> tx_count_{0};
};

// =============================================================================
// Per-instance worker
// =============================================================================

struct InstanceContext {
    const char* name;
    LoopbackAdapter* adapter;
    EtherCAT::Master* master;
    std::atomic<bool> done{false};
};

static void instanceWorker(InstanceContext* ctx) {
    TETHER_LOGI(TAG, "[%s] Starting master instance", ctx->name);

    const uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00,
                            static_cast<uint8_t>(ctx->name[0])};
    auto iface = ctx->adapter->makeInterface();
    ctx->adapter->setMaster(ctx->master);

    ctx->master->start(iface, mac);

    // Let it run for a short while
    for (int i = 0; i < 20 && ctx->master->isRunning(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    uint16_t slaves = ctx->master->getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "[%s] Discovered %u slave(s), adapter TX count: %u",
             ctx->name, slaves, ctx->adapter->txCount());

    ctx->master->stop();
    TETHER_LOGI(TAG, "[%s] Master stopped", ctx->name);
    ctx->done.store(true);
}

// =============================================================================
// Main
// =============================================================================

int main() {
    printf("=== Dual EtherCAT Master Instance Demo (Linux) ===\n\n");

    // Create two completely independent masters
    EtherCAT::Master::Config cfg;
    cfg.rx_queue_depth   = 8;
    cfg.txpdo_queue_depth = 4;

    EtherCAT::Master master_a(cfg);
    EtherCAT::Master master_b(cfg);

    LoopbackAdapter adapter_a("MasterA");
    LoopbackAdapter adapter_b("MasterB");

    InstanceContext ctx_a{"MasterA", &adapter_a, &master_a};
    InstanceContext ctx_b{"MasterB", &adapter_b, &master_b};

    // Run each master on its own thread
    std::thread t_a(instanceWorker, &ctx_a);
    std::thread t_b(instanceWorker, &ctx_b);

    t_a.join();
    t_b.join();

    printf("\n=== Both masters completed independently ===\n");
    printf("  MasterA TX frames: %u\n", adapter_a.txCount());
    printf("  MasterB TX frames: %u\n", adapter_b.txCount());

    return 0;
}
