/**
 * @file STM32Ethernet.cpp
 * @brief STM32 Ethernet HAL implementation using STM32 HAL driver
 *
 * NOTE: This is a best-effort implementation without testing.
 * The project structure for STM32 doesn't exist yet.
 */

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32_HAL)

#include "hal/IEthernet.hpp"
#include "hal/HALTypes.hpp"

// STM32 HAL includes (adjust paths as needed for your STM32 family)
#ifdef STM32F4
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_eth.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_eth.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_eth.h"
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <cstring>
#include <atomic>

namespace EtherCAT {
namespace HAL {

/**
 * @brief STM32 Ethernet HAL implementation
 *
 * Uses the STM32 HAL Ethernet driver with DMA for high-performance frame I/O.
 */
class STM32Ethernet : public IEthernet {
public:
    STM32Ethernet() = default;
    ~STM32Ethernet() override { shutdown(); }

    Error init(const EthernetConfig& config) override {
        if (m_initialized) {
            return Error::AlreadyInitialized;
        }

        m_config = config;

        // Initialize Ethernet handle
        m_heth.Instance = ETH;
        m_heth.Init.MACAddr = const_cast<uint8_t*>(config.macAddress.bytes);
        m_heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
        m_heth.Init.RxBuffLen = kMaxFrameSize;

        // Initialize Ethernet with HAL
        if (HAL_ETH_Init(&m_heth) != HAL_OK) {
            return Error::InternalError;
        }

        // Allocate DMA buffers
        m_txBuffer = new uint8_t[kMaxFrameSize];
        m_rxBuffer = new uint8_t[kMaxFrameSize];

        if (!m_txBuffer || !m_rxBuffer) {
            shutdown();
            return Error::NoMemory;
        }

        // Configure DMA descriptors
        memset(&m_txConfig, 0, sizeof(m_txConfig));
        m_txConfig.TxBuffer = m_txBuffer;
        m_txConfig.Length = 0;

        // Get MAC address from hardware
        HAL_ETH_GetMACAddress(&m_heth, ETH_MAC_ADDRESS0, m_mac.bytes);

        // Set promiscuous mode if requested
        if (config.promiscuous) {
            setPromiscuous(true);
        }

        m_ethertypeFilter = config.ethertypeFilter;

        // Start Ethernet
        if (HAL_ETH_Start(&m_heth) != HAL_OK) {
            shutdown();
            return Error::InternalError;
        }

        // Create RX semaphore
        m_rxSemaphore = xSemaphoreCreateBinary();

        m_initialized = true;
        return Error::OK;
    }

    void shutdown() override {
        if (!m_initialized) return;

        HAL_ETH_Stop(&m_heth);
        HAL_ETH_DeInit(&m_heth);

        delete[] m_txBuffer;
        delete[] m_rxBuffer;
        m_txBuffer = nullptr;
        m_rxBuffer = nullptr;

        if (m_rxSemaphore) {
            vSemaphoreDelete(m_rxSemaphore);
            m_rxSemaphore = nullptr;
        }

        m_initialized = false;
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

        HAL_ETH_SetMACAddress(&m_heth, ETH_MAC_ADDRESS0, 
                              const_cast<uint8_t*>(mac.bytes));
        m_mac = mac;
        return Error::OK;
    }

    Error transmit(const uint8_t* frame, size_t length) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length > kMaxFrameSize) return Error::BufferTooSmall;

        // Copy frame to DMA-aligned buffer
        memcpy(m_txBuffer, frame, length);

        // Configure TX descriptor
        m_txConfig.Length = length;
        m_txConfig.TxBuffer = m_txBuffer;

        // Transmit
        if (HAL_ETH_Transmit(&m_heth, &m_txConfig, HAL_MAX_DELAY) != HAL_OK) {
            m_stats.txErrors++;
            return Error::TransmitFailed;
        }

        m_stats.txFrames++;
        m_stats.txBytes += length;
        return Error::OK;
    }

    Error transmitVlan(const uint8_t* frame, size_t length,
                       uint16_t vlanId, uint8_t priority) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!frame || length < kMinFrameSize) return Error::InvalidArgument;
        if (length + kVlanTagSize > kMaxFrameSizeVlan) return Error::BufferTooSmall;

        // Build VLAN-tagged frame
        uint8_t* vlanFrame = m_txBuffer;
        
        memcpy(vlanFrame, frame, 12);  // MACs
        vlanFrame[12] = 0x81;
        vlanFrame[13] = 0x00;
        uint16_t tci = ((priority & 0x07) << 13) | (vlanId & 0x0FFF);
        vlanFrame[14] = (tci >> 8) & 0xFF;
        vlanFrame[15] = tci & 0xFF;
        memcpy(vlanFrame + 16, frame + 12, length - 12);

        m_txConfig.Length = length + kVlanTagSize;
        m_txConfig.TxBuffer = vlanFrame;

        if (HAL_ETH_Transmit(&m_heth, &m_txConfig, HAL_MAX_DELAY) != HAL_OK) {
            m_stats.txErrors++;
            return Error::TransmitFailed;
        }

        m_stats.txFrames++;
        m_stats.txBytes += length + kVlanTagSize;
        return Error::OK;
    }

    Error transmitGather(const BufferDesc* iov, size_t count) override {
        if (!m_initialized) return Error::NotInitialized;
        if (!iov || count == 0) return Error::InvalidArgument;

        // Calculate total length and copy to contiguous buffer
        size_t totalLen = 0;
        for (size_t i = 0; i < count; i++) {
            totalLen += iov[i].length;
        }

        if (totalLen < kMinFrameSize || totalLen > kMaxFrameSize) {
            return Error::InvalidArgument;
        }

        size_t offset = 0;
        for (size_t i = 0; i < count; i++) {
            memcpy(m_txBuffer + offset, iov[i].data, iov[i].length);
            offset += iov[i].length;
        }

        m_txConfig.Length = totalLen;
        m_txConfig.TxBuffer = m_txBuffer;

        if (HAL_ETH_Transmit(&m_heth, &m_txConfig, HAL_MAX_DELAY) != HAL_OK) {
            m_stats.txErrors++;
            return Error::TransmitFailed;
        }

        m_stats.txFrames++;
        m_stats.txBytes += totalLen;
        return Error::OK;
    }

    void setRxCallback(RxCallback callback, void* userData) override {
        m_rxCallback = callback;
        m_userData = userData;
    }

    int poll(Milliseconds timeoutMs) override {
        if (!m_initialized) return 0;

        int count = 0;
        ETH_BufferTypeDef rxBuffer;

        // Check for received frames
        while (HAL_ETH_ReadData(&m_heth, &rxBuffer) == HAL_OK) {
            if (rxBuffer.buffer && rxBuffer.len > 0) {
                // Apply EtherType filter
                if (rxBuffer.len >= 14 && m_ethertypeFilter != 0) {
                    uint16_t ethertype = (rxBuffer.buffer[12] << 8) | rxBuffer.buffer[13];
                    
                    if (ethertype == kEtherType8021Q && rxBuffer.len >= 18) {
                        ethertype = (rxBuffer.buffer[16] << 8) | rxBuffer.buffer[17];
                    }
                    
                    if (ethertype != m_ethertypeFilter) {
                        m_stats.rxFiltered++;
                        continue;
                    }
                }

                m_stats.rxFrames++;
                m_stats.rxBytes += rxBuffer.len;

                if (m_rxCallback) {
                    RxFrameInfo info;
                    info.timestamp = getCurrentTimestamp();
                    
                    // Check VLAN tag
                    if (rxBuffer.len >= 18) {
                        uint16_t ethertype = (rxBuffer.buffer[12] << 8) | rxBuffer.buffer[13];
                        if (ethertype == kEtherType8021Q) {
                            info.vlanTagPresent = true;
                            uint16_t tci = (rxBuffer.buffer[14] << 8) | rxBuffer.buffer[15];
                            info.vlanId = tci & 0x0FFF;
                            info.vlanPriority = (tci >> 13) & 0x07;
                        }
                    }

                    m_rxCallback(rxBuffer.buffer, rxBuffer.len, info, m_userData);
                }

                count++;
            }
        }

        return count;
    }

    void setEthertypeFilter(uint16_t ethertype) override {
        m_ethertypeFilter = ethertype;
    }

    Error setPromiscuous(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;

        ETH_MACFilterConfigTypeDef filterConfig;
        HAL_ETH_GetMACFilterConfig(&m_heth, &filterConfig);
        
        filterConfig.PromiscuousMode = enable ? ENABLE : DISABLE;
        
        if (HAL_ETH_SetMACFilterConfig(&m_heth, &filterConfig) != HAL_OK) {
            return Error::InternalError;
        }

        m_promiscuous = enable;
        return Error::OK;
    }

    Error addMulticastAddress(const MacAddress& mac) override {
        if (!m_initialized) return Error::NotInitialized;
        
        // STM32 uses hash-based multicast filtering
        // Add to multicast hash table
        HAL_ETH_SetMACAddress(&m_heth, ETH_MAC_ADDRESS1, 
                              const_cast<uint8_t*>(mac.bytes));
        return Error::OK;
    }

    Error removeMulticastAddress(const MacAddress& mac) override {
        (void)mac;
        return Error::OK;
    }

    Error setAllMulticast(bool enable) override {
        if (!m_initialized) return Error::NotInitialized;

        ETH_MACFilterConfigTypeDef filterConfig;
        HAL_ETH_GetMACFilterConfig(&m_heth, &filterConfig);
        
        filterConfig.PassAllMulticast = enable ? ENABLE : DISABLE;
        
        if (HAL_ETH_SetMACFilterConfig(&m_heth, &filterConfig) != HAL_OK) {
            return Error::InternalError;
        }

        return Error::OK;
    }

    LinkStatus getLinkStatus() const override {
        LinkStatus status;
        if (!m_initialized) return status;

        // Read PHY link status
        uint32_t linkState;
        if (HAL_ETH_ReadPHYRegister(&m_heth, 0, 1, &linkState) == HAL_OK) {
            status.up = (linkState & 0x04) != 0;  // Link status bit
        }

        // Read speed/duplex from PHY
        uint32_t phyStatus;
        if (HAL_ETH_ReadPHYRegister(&m_heth, 0, 0x10, &phyStatus) == HAL_OK) {
            status.speedMbps = (phyStatus & 0x02) ? 100 : 10;
            status.fullDuplex = (phyStatus & 0x04) != 0;
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
        return &m_heth;
    }

    const char* getInterfaceName() const override {
        return "eth0";
    }

    // Interrupt handlers - call from HAL_ETH_RxCpltCallback
    void onRxComplete() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(m_rxSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

private:
    bool m_initialized = false;
    EthernetConfig m_config;
    ETH_HandleTypeDef m_heth;
    ETH_TxPacketConfig m_txConfig;
    MacAddress m_mac;
    bool m_promiscuous = false;
    uint16_t m_ethertypeFilter = 0;

    uint8_t* m_txBuffer = nullptr;
    uint8_t* m_rxBuffer = nullptr;
    SemaphoreHandle_t m_rxSemaphore = nullptr;

    RxCallback m_rxCallback = nullptr;
    void* m_userData = nullptr;
    LinkCallback m_linkCallback = nullptr;
    void* m_linkUserData = nullptr;

    EthernetStats m_stats;

    static Timestamp getCurrentTimestamp() {
        return xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
    }
};

std::unique_ptr<IEthernet> createSTM32Ethernet() {
    return std::make_unique<STM32Ethernet>();
}

} // namespace HAL
} // namespace EtherCAT

#endif // STM32F4 || STM32F7 || STM32H7 || STM32_HAL
