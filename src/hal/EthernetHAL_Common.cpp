/**
 * @file EthernetHAL_Common.cpp
 * @brief Common utility functions for EthernetHAL
 */

#include "hal/EthernetHAL.hpp"
#include "hal/IEthernet.hpp"
#include <cstring>

namespace EtherCAT {
namespace HAL {

void buildEthernetHeader(uint8_t* buffer,
                          const uint8_t dst_mac[6],
                          const uint8_t src_mac[6],
                          uint16_t ethertype) {
    // Destination MAC (bytes 0-5)
    std::memcpy(buffer, dst_mac, 6);

    // Source MAC (bytes 6-11)
    std::memcpy(buffer + 6, src_mac, 6);

    // EtherType (bytes 12-13) - big-endian
    buffer[12] = static_cast<uint8_t>((ethertype >> 8) & 0xFF);
    buffer[13] = static_cast<uint8_t>(ethertype & 0xFF);
}

uint16_t getEtherType(const uint8_t* frame) {
    // EtherType is at bytes 12-13 in big-endian
    return (static_cast<uint16_t>(frame[12]) << 8) | frame[13];
}

// ============================================================================
// Mock HAL Implementation (for testing)
// ============================================================================

class MockEthernetHAL : public IEthernetHAL {
public:
    MockEthernetHAL() = default;
    ~MockEthernetHAL() override { shutdown(); }

    bool init(const char* interface_name) override {
        (void)interface_name;
        m_initialized = true;
        // Default MAC
        m_mac[0] = 0x02;  // Locally administered
        m_mac[1] = 0x00;
        m_mac[2] = 0x00;
        m_mac[3] = 0x00;
        m_mac[4] = 0x00;
        m_mac[5] = 0x01;
        return true;
    }

    void shutdown() override {
        m_initialized = false;
        m_rx_callback = nullptr;
    }

    bool isInitialized() const override {
        return m_initialized;
    }

    bool getMacAddress(uint8_t mac[6]) const override {
        if (!m_initialized) return false;
        std::memcpy(mac, m_mac, 6);
        return true;
    }

    bool setMacAddress(const uint8_t mac[6]) override {
        std::memcpy(m_mac, mac, 6);
        return true;
    }

    bool transmit(const uint8_t* frame, size_t length) override {
        if (!m_initialized) return false;
        if (length < kMinFrameSize || length > kMaxFrameSize) return false;

        m_stats.tx_frames++;
        m_stats.tx_bytes += length;

        // Store for loopback testing
        if (length <= sizeof(m_last_tx_frame)) {
            std::memcpy(m_last_tx_frame, frame, length);
            m_last_tx_length = length;
        }

        // If loopback enabled, send to RX callback
        if (m_loopback && m_rx_callback) {
            if (m_ethertype_filter == 0 || getEtherType(frame) == m_ethertype_filter) {
                m_rx_callback(frame, length, m_user_data);
                m_stats.rx_frames++;
                m_stats.rx_bytes += length;
            } else {
                m_stats.rx_filtered++;
            }
        }

        return true;
    }

    void setRxCallback(EthernetHALRxCallback callback, void* user_data) override {
        m_rx_callback = callback;
        m_user_data = user_data;
    }

    void setEtherTypeFilter(uint16_t ethertype) override {
        m_ethertype_filter = ethertype;
    }

    bool setPromiscuous(bool enable) override {
        m_promiscuous = enable;
        return true;
    }

    int poll(uint32_t timeout_ms) override {
        (void)timeout_ms;
        return 0;  // Mock doesn't need polling
    }

    Stats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = {};
    }

    // Mock-specific methods
    void setLoopback(bool enable) { m_loopback = enable; }
    bool isLoopback() const { return m_loopback; }

    // Inject a frame as if received
    void injectRxFrame(const uint8_t* frame, size_t length) {
        if (m_rx_callback && m_initialized) {
            if (m_ethertype_filter == 0 || getEtherType(frame) == m_ethertype_filter) {
                m_rx_callback(frame, length, m_user_data);
                m_stats.rx_frames++;
                m_stats.rx_bytes += length;
            } else {
                m_stats.rx_filtered++;
            }
        }
    }

    // Get last transmitted frame
    const uint8_t* getLastTxFrame() const { return m_last_tx_frame; }
    size_t getLastTxLength() const { return m_last_tx_length; }

private:
    bool m_initialized = false;
    bool m_promiscuous = false;
    bool m_loopback = false;
    uint8_t m_mac[6] = {0};
    uint16_t m_ethertype_filter = 0;
    EthernetHALRxCallback m_rx_callback = nullptr;
    void* m_user_data = nullptr;
    Stats m_stats = {};

    uint8_t m_last_tx_frame[kMaxFrameSize] = {0};
    size_t m_last_tx_length = 0;
};

std::unique_ptr<IEthernetHAL> createMockHAL() {
    return std::make_unique<MockEthernetHAL>();
}

// ============================================================================
// IEthernet to IEthernetHAL Adapter
// ============================================================================

class EthernetHALAdapter : public IEthernetHAL {
public:
    explicit EthernetHALAdapter(std::unique_ptr<IEthernet> eth)
        : m_eth(std::move(eth)) {}

    bool init(const char* interface_name) override {
        EthernetConfig config;
        config.interfaceName = interface_name;
        return m_eth->init(config) == Error::OK;
    }

    void shutdown() override { m_eth->shutdown(); }

    bool isInitialized() const override { return m_eth->isInitialized(); }

    bool getMacAddress(uint8_t mac[6]) const override {
        MacAddress addr;
        if (m_eth->getMacAddress(addr) != Error::OK) return false;
        std::memcpy(mac, addr.bytes, 6);
        return true;
    }

    bool setMacAddress(const uint8_t mac[6]) override {
        return m_eth->setMacAddress(MacAddress(mac)) == Error::OK;
    }

    bool transmit(const uint8_t* frame, size_t length) override {
        return m_eth->transmit(frame, length) == Error::OK;
    }

    void setRxCallback(EthernetHALRxCallback callback, void* user_data) override {
        m_user_callback = callback;
        m_user_data = user_data;
        if (callback) {
            m_eth->setRxCallback(
                [](const uint8_t* frame, size_t length, const RxFrameInfo&, void* userData) {
                    auto* self = static_cast<EthernetHALAdapter*>(userData);
                    if (self->m_user_callback) {
                        self->m_user_callback(frame, length, self->m_user_data);
                    }
                },
                this);
        } else {
            m_eth->setRxCallback(nullptr, nullptr);
        }
    }

    void setEtherTypeFilter(uint16_t ethertype) override {
        m_eth->setEthertypeFilter(ethertype);
    }

    bool setPromiscuous(bool enable) override {
        return m_eth->setPromiscuous(enable) == Error::OK;
    }

    int poll(uint32_t timeout_ms) override {
        return m_eth->poll(static_cast<Milliseconds>(timeout_ms));
    }

    Stats getStats() const override {
        auto es = m_eth->getStats();
        Stats s;
        s.tx_frames = es.txFrames;
        s.tx_bytes = es.txBytes;
        s.tx_errors = es.txErrors;
        s.rx_frames = es.rxFrames;
        s.rx_bytes = es.rxBytes;
        s.rx_errors = es.rxErrors;
        s.rx_filtered = es.rxFiltered;
        return s;
    }

    void resetStats() override { m_eth->resetStats(); }

private:
    std::unique_ptr<IEthernet> m_eth;
    EthernetHALRxCallback m_user_callback = nullptr;
    void* m_user_data = nullptr;
};


// ============================================================================

#if defined(__linux__)
std::unique_ptr<IEthernetHAL> createLinuxRawSocketHAL() {
    return std::make_unique<EthernetHALAdapter>(createLinuxRawSocketEthernet());
}
#endif

#ifdef ESP_PLATFORM
std::unique_ptr<IEthernetHAL> createESP32HAL() {
    return std::make_unique<EthernetHALAdapter>(createESP32Ethernet());
}
#endif

// Default HAL Creation
// ============================================================================

std::unique_ptr<IEthernetHAL> createDefaultHAL() {
#if defined(__linux__)
    // On Linux host, use raw socket HAL
    return createLinuxRawSocketHAL();
#else
    // On ESP32, use ESP32 HAL
    return createESP32HAL();
#endif
}

} // namespace HAL
} // namespace EtherCAT
