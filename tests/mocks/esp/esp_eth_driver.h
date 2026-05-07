#pragma once

// Mock esp_eth_driver.h for host testing

#include <stdint.h>
#include "esp_err.h"

// Ethernet commands
typedef enum {
    ETH_CMD_G_MAC_ADDR,
    ETH_CMD_S_MAC_ADDR,
    ETH_CMD_G_PHY_ADDR,
    ETH_CMD_S_PHY_ADDR,
    ETH_CMD_G_SPEED,
    ETH_CMD_S_SPEED,
    ETH_CMD_S_PROMISCUOUS,
    ETH_CMD_S_FLOW_CTRL,
    ETH_CMD_G_DUPLEX_MODE,
    ETH_CMD_S_DUPLEX_MODE,
    ETH_CMD_S_PHY_LOOPBACK,
} esp_eth_io_cmd_t;

// Mock function
inline esp_err_t esp_eth_ioctl(void* hdl, esp_eth_io_cmd_t cmd, void *data) {
    return ESP_OK;
}

inline esp_err_t esp_eth_transmit(void* hdl, void *buf, size_t length) {
    return ESP_OK;
}
