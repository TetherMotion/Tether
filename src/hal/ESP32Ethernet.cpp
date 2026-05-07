/**
 * @file ESP32Ethernet.cpp
 * @brief ESP32 Ethernet HAL implementation using ESP-IDF driver
 */

#if defined(ESP_PLATFORM) || defined(ESP32)

#include "hal/IEthernet.hpp"
#include "hal/HALTypes.hpp"

#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <cstring>
#include <atomic>

static const char* TAG = "ESP32Eth";

namespace EtherCAT {
namespace HAL {

/**
 * @brief ESP32 Ethernet HAL implementation
 */
class ESP32Ethernet : public IEthernet {
public:
    ESP32Ethernet() = default;
    ~ESP32Ethernet() override { shutdown(); }

    Error init(const EthernetConfig& config) override {
        if (m_initialized) {
            return Error::AlreadyInitialized;
        }

        // Store configuration
        m_config = config;

        // Initialize MAC configuration
        eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
        eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
        
        // Configure EMAC clock
        esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
        esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_APPL_CLK_OUT_GPIO;
        
        // Disable SMI for simulated PHY
        esp32_emac_config.smi_gpio.mdc_num = -1;
        esp32_emac_config.smi_gpio.mdio_num = -1;

        // Create MAC instance
        esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
        if (!mac) {
            TETHER_LOGE(TAG, "Failed to create ESP32 MAC");
            return Error::InternalError;
        }

        // Use simulated PHY for now
        m_ethHandle = nullptr;
        esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, &m_simulatedPhy);
        
        esp_err_t err = esp_eth_driver_install(&eth_config, &m_ethHandle);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "Failed to install Ethernet driver: %s", esp_err_to_name(err));
            return Error::InternalError;
        }

        // Register event handlers
        err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, 
                                         &ESP32Ethernet::ethEventHandler, this);
        if (err != ESP_OK) {
            TETHER_LOGW(TAG, "Failed to register ETH event handler: %s", esp_err_to_name(err));
        }

        // Set RX callback
        err = esp_eth_update_input_path(m_ethHandle, &ESP32Ethernet::rxCallback, this);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "Failed to set RX callback: %s", esp_err_to_name(err));
            esp_eth_driver_uninstall(m_ethHandle);
            return Error::InternalError;
        }

        // Start Ethernet
        err = esp_eth_start(m_ethHandle);
        if (err != ESP_OK) {
            TETHER_LOGE(TAG, "Failed to start Ethernet: %s", esp_err_to_name(err));
            esp_eth_driver_uninstall(m_ethHandle);
            return Error::InternalError;
        }

        // Enable promiscuous and multicast if needed
        if (config.promiscuous) {
            setPromiscuous(true);
        }
        setAllMulticast(true);

        // Get MAC address
        esp_eth_ioctl(m_ethHandle, ETH_CMD_G_MAC_ADDR, m_mac.bytes);
        TETHER_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 m_mac.bytes[0], m_mac.bytes[1], m_mac.bytes[2],
                 m_mac.bytes[3], m_mac.bytes[4], m_mac.bytes[5]);

        m_ethertypeFilter = config.ethertypeFilter;
        m_initialized = true;

        TETHER_LOGI(TAG, "ESP32 Ethernet initialized");
        return Error::OK;
    }

    void shutdown() override {
        if (!m_initialized) return;

        if (m_ethHandle) {
            esp_eth_stop(m_ethHandle);
            esp_eth_driver_uninstall(m_ethHandle);
            m_ethHandle = nullptr;
        }

        esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, 
                                     &ESP32Ethernet::ethEventHandler);

        m_initialized = false;
        TETHER_LOGI(TAG, "ESP32 Ethernet shutdown");
    }

    bool isInitialized() const override {
        return m_initialized;
    }

    Error getMacAddress(MacAddress& mac) const override {
        if (!m_initialized) return Error::NotInitialized;
        mac = m_mac;
        return Error::OK;
    }

    Error setMacAddress(const MacAddress& mac) override {
        if (!m_initialized) return Error::NotInitialized;
        
        esp_err_t err = esp_eth_ioctl(m_ethHandle, ETH_CMD_S_MAC_ADDR, 
                                       const_cast<uint8_t*>(mac.bytes));
        if (err != ESP_OK) {
            return Error::InternalError;
        }
        m_mac = mac;
        return Error::OK;
    }

    Error transmit(const uint8_t* frame, size_t length) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length > kMaxFrameSize) return Error::BufferTooSmall;

        esp_err_t err = esp_eth_transmit(m_ethHandle, const_cast<uint8_t*>(frame), length);
        
        if (err == ESP_OK) {
            m_stats.txFrames++;
            m_stats.txBytes += length;
            return Error::OK;
        }
        
        m_stats.txErrors++;
        if (err == ESP_ERR_NO_MEM) {
            m_stats.txDropped++;
            return Error::WouldBlock;
        }
        
        return Error::TransmitFailed;
    }

    Error transmitVlan(const uint8_t* frame, size_t length,
                       uint16_t vlanId, uint8_t priority) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length + kVlanTagSize > kMaxFrameSizeVlan) return Error::BufferTooSmall;

        // Build frame with VLAN tag
        uint8_t vlanFrame[kMaxFrameSizeVlan];
        
        memcpy(vlanFrame, frame, 12);  // MACs
        vlanFrame[12] = 0x81;
        vlanFrame[13] = 0x00;
        uint16_t tci = ((priority & 0x07) << 13) | (vlanId & 0x0FFF);
        vlanFrame[14] = (tci >> 8) & 0xFF;
        vlanFrame[15] = tci & 0xFF;
        memcpy(vlanFrame + 16, frame + 12, length - 12);

        return transmit(vlanFrame, length + kVlanTagSize);
    }

    Error transmitGather(const BufferDesc* iov, size_t count) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!iov || count == 0) return Error::InvalidArgument;

        size_t totalLen = 0;
        for (size_t i = 0; i < count; i++) {
            totalLen += iov[i].length;
        }

        if (totalLen < kMinFrameSize || totalLen > kMaxFrameSize) {
            return Error::InvalidArgument;
        }

        uint8_t frame[kMaxFrameSize];
        size_t offset = 0;
        for (size_t i = 0; i < count; i++) {
            memcpy(frame + offset, iov[i].data, iov[i].length);
            offset += iov[i].length;
        }

        return transmit(frame, totalLen);
    }

    void setRxCallback(RxCallback callback, void* userData) override {
        m_userRxCallback = callback;
        m_userData = userData;
    }

    int poll(Milliseconds timeoutMs) override {
        // ESP32 Ethernet is interrupt-driven, no polling needed
        (void)timeoutMs;
        return 0;
    }

    void setEthertypeFilter(uint16_t ethertype) override {
        m_ethertypeFilter = ethertype;
    }

    Error setPromiscuous(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;
        
        esp_err_t err = esp_eth_ioctl(m_ethHandle, ETH_CMD_S_PROMISCUOUS, &enable);
        if (err != ESP_OK) {
            return Error::InternalError;
        }
        m_promiscuous = enable;
        return Error::OK;
    }

    Error addMulticastAddress(const MacAddress& mac) override {
        // ESP32 doesn't have fine-grained multicast filtering
        // Use all-multicast mode instead
        (void)mac;
        return setAllMulticast(true);
    }

    Error removeMulticastAddress(const MacAddress& mac) override {
        (void)mac;
        return Error::OK;
    }

    Error setAllMulticast(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;
        
        esp_err_t err = esp_eth_ioctl(m_ethHandle, ETH_CMD_S_ALL_MULTICAST, &enable);
        if (err != ESP_OK) {
            TETHER_LOGW(TAG, "ETH_CMD_S_ALL_MULTICAST failed: %s", esp_err_to_name(err));
            return Error::NotSupported;
        }
        return Error::OK;
    }

    LinkStatus getLinkStatus() const override {
        LinkStatus status;
        if (!m_initialized) return status;

        eth_link_t link;
        esp_err_t err = esp_eth_ioctl(m_ethHandle, ETH_CMD_G_LINK, &link);
        if (err == ESP_OK) {
            status.up = (link == ETH_LINK_UP);
        }

        eth_speed_t speed;
        err = esp_eth_ioctl(m_ethHandle, ETH_CMD_G_SPEED, &speed);
        if (err == ESP_OK) {
            status.speedMbps = (speed == ETH_SPEED_100M) ? 100 : 10;
        }

        eth_duplex_t duplex;
        err = esp_eth_ioctl(m_ethHandle, ETH_CMD_G_DUPLEX_MODE, &duplex);
        if (err == ESP_OK) {
            status.fullDuplex = (duplex == ETH_DUPLEX_FULL);
        }

        return status;
    }

    void setLinkCallback(LinkCallback callback, void* userData) override {
        m_linkCallback = callback;
        m_linkUserData = userData;
    }

    Error waitForLinkUp(Milliseconds timeoutMs) override {
        TickType_t start = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(timeoutMs);
        
        while ((xTaskGetTickCount() - start) < timeout) {
            if (getLinkStatus().up) {
                return Error::OK;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        return Error::Timeout;
    }

    EthernetStats getStats() const override {
        return m_stats;
    }

    void resetStats() override {
        m_stats = {};
    }

    void* nativeHandle() override {
        return m_ethHandle;
    }

    const char* getInterfaceName() const override {
        return "eth0";
    }

    // Simulated link control (for testing without PHY)
    void setSimulatedLink(bool up) {
        if (m_mediator) {
            m_mediator->on_state_changed(m_mediator, ETH_STATE_SPEED, (void*)ETH_SPEED_100M);
            m_mediator->on_state_changed(m_mediator, ETH_STATE_DUPLEX, (void*)ETH_DUPLEX_FULL);
            m_mediator->on_state_changed(m_mediator, ETH_STATE_PAUSE, (void*)0);
            m_mediator->on_state_changed(m_mediator, ETH_STATE_LINK, 
                                         (void*)(up ? ETH_LINK_UP : ETH_LINK_DOWN));
        }
    }

private:
    bool m_initialized = false;
    EthernetConfig m_config;
    esp_eth_handle_t m_ethHandle = nullptr;
    MacAddress m_mac;
    bool m_promiscuous = false;
    uint16_t m_ethertypeFilter = 0;

    RxCallback m_userRxCallback = nullptr;
    void* m_userData = nullptr;
    LinkCallback m_linkCallback = nullptr;
    void* m_linkUserData = nullptr;

    EthernetStats m_stats;

    esp_eth_mediator_t* m_mediator = nullptr;

    // Simulated PHY for testing without external PHY chip
    esp_eth_phy_t m_simulatedPhy = {
        .set_mediator = [](esp_eth_phy_t* phy, esp_eth_mediator_t* mediator) -> esp_err_t {
            auto* self = reinterpret_cast<ESP32Ethernet*>(
                reinterpret_cast<char*>(phy) - offsetof(ESP32Ethernet, m_simulatedPhy));
            self->m_mediator = mediator;
            return ESP_OK;
        },
        .reset = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
        .reset_hw = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
        .init = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
        .deinit = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
        .autonego_ctrl = [](esp_eth_phy_t* phy, eth_phy_autoneg_cmd_t cmd, bool* en) -> esp_err_t { 
            return ESP_OK; 
        },
        .get_link = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
        .set_link = [](esp_eth_phy_t* phy, eth_link_t link) -> esp_err_t { return ESP_OK; },
        .pwrctl = [](esp_eth_phy_t* phy, bool enable) -> esp_err_t { return ESP_OK; },
        .set_addr = [](esp_eth_phy_t* phy, uint32_t addr) -> esp_err_t { return ESP_OK; },
        .get_addr = [](esp_eth_phy_t* phy, uint32_t* addr) -> esp_err_t {
            *addr = 0;
            return ESP_OK;
        },
        .advertise_pause_ability = [](esp_eth_phy_t* phy, uint32_t ability) -> esp_err_t {
            return ESP_OK;
        },
        .loopback = [](esp_eth_phy_t* phy, bool enable) -> esp_err_t { return ESP_OK; },
        .del = [](esp_eth_phy_t* phy) -> esp_err_t { return ESP_OK; },
    };

    static esp_err_t rxCallback(esp_eth_handle_t eth_handle, uint8_t* buffer, 
                                uint32_t length, void* priv) {
        auto* self = static_cast<ESP32Ethernet*>(priv);
        
        if (!buffer || length == 0) {
            return ESP_OK;
        }

        // Apply EtherType filter
        if (length >= 14 && self->m_ethertypeFilter != 0) {
            uint16_t ethertype = (buffer[12] << 8) | buffer[13];
            
            // Handle VLAN
            if (ethertype == kEtherType8021Q && length >= 18) {
                ethertype = (buffer[16] << 8) | buffer[17];
            }
            
            if (ethertype != self->m_ethertypeFilter) {
                self->m_stats.rxFiltered++;
                free(buffer);
                return ESP_OK;
            }
        }

        self->m_stats.rxFrames++;
        self->m_stats.rxBytes += length;

        if (self->m_userRxCallback) {
            RxFrameInfo info;
            info.timestamp = esp_timer_get_time();
            
            // Check for VLAN tag
            if (length >= 18) {
                uint16_t ethertype = (buffer[12] << 8) | buffer[13];
                if (ethertype == kEtherType8021Q) {
                    info.vlanTagPresent = true;
                    uint16_t tci = (buffer[14] << 8) | buffer[15];
                    info.vlanId = tci & 0x0FFF;
                    info.vlanPriority = (tci >> 13) & 0x07;
                }
            }
            
            self->m_userRxCallback(buffer, length, info, self->m_userData);
        }

        free(buffer);
        return ESP_OK;
    }

    static void ethEventHandler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data) {
        auto* self = static_cast<ESP32Ethernet*>(arg);
        
        switch (event_id) {
            case ETHERNET_EVENT_CONNECTED:
                TETHER_LOGI(TAG, "Ethernet Link Up");
                if (self->m_linkCallback) {
                    LinkStatus status = self->getLinkStatus();
                    self->m_linkCallback(status, self->m_linkUserData);
                }
                break;
            case ETHERNET_EVENT_DISCONNECTED:
                TETHER_LOGI(TAG, "Ethernet Link Down");
                if (self->m_linkCallback) {
                    LinkStatus status;
                    status.up = false;
                    self->m_linkCallback(status, self->m_linkUserData);
                }
                break;
            case ETHERNET_EVENT_START:
                TETHER_LOGI(TAG, "Ethernet Started");
                break;
            case ETHERNET_EVENT_STOP:
                TETHER_LOGI(TAG, "Ethernet Stopped");
                break;
            default:
                break;
        }
    }
};

std::unique_ptr<IEthernet> createESP32Ethernet() {
    return std::make_unique<ESP32Ethernet>();
}

} // namespace HAL
} // namespace EtherCAT

#endif // ESP_PLATFORM || ESP32
