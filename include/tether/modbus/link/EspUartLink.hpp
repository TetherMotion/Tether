/**
 * @file EspUartLink.hpp
 * @brief ESP-IDF UART backend for Modbus RTU on ESP32 targets.
 *
 * Header-only ILink impl over the ESP-IDF uart driver; compiled only
 * under ESP_PLATFORM (host builds never see it). RS-485 direction is
 * handled by uart_set_mode(UART_MODE_RS485_HALF_DUPLEX) — the driver
 * toggles the RTS pin itself.
 */
#pragma once

#if defined(ESP_PLATFORM) && defined(TETHER_MODBUS_ESP_UART)

#include "driver/uart.h"

#include "tether/modbus/link/ILink.hpp"
#include "tether/modbus/link/SerialConfig.hpp"

namespace tether::modbus {

class EspUartLink final : public ILink {
public:
    struct Opts {
        uart_port_t port = UART_NUM_1;
        int txPin = 17, rxPin = 16, rtsPin = -1;  // rtsPin≥0 → RS-485
        int rxBuf = 1024, txBuf = 1024;
        int eventQueueLen = 8;  // 0 → no UART event queue (no err counts)
    };

    /// `cfg` supplies baud/dataBits/parity/stopBits (see SerialConfig).
    EspUartLink(const Opts& o, const SerialConfig& cfg)
        : o_(o), cfg_(cfg) {}
    ~EspUartLink() override { close(); }

    Expected<void> open() override {
        if (open_) return {};
        uart_config_t uc{};
        uc.baud_rate = int(cfg_.baud);
        uc.data_bits = uart_word_length_t(cfg_.dataBits - 5);
        uc.parity = cfg_.parity == Parity::Even ? UART_PARITY_EVEN
                  : cfg_.parity == Parity::Odd  ? UART_PARITY_ODD
                                                : UART_PARITY_DISABLE;
        uc.stop_bits = cfg_.stopBits == StopBits::Two ? UART_STOP_BITS_2
                                                      : UART_STOP_BITS_1;
        uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        if (uart_param_config(o_.port, &uc) != ESP_OK)
            return err(ModbusError::transport("uart_param_config"));
        if (uart_set_pin(o_.port, o_.txPin, o_.rxPin,
                         o_.rtsPin, UART_PIN_NO_CHANGE) != ESP_OK)
            return err(ModbusError::transport("uart_set_pin"));
        if (uart_driver_install(o_.port, o_.rxBuf, o_.txBuf,
                                o_.eventQueueLen,
                                o_.eventQueueLen > 0 ? &eventQueue_
                                                     : nullptr,
                                0) != ESP_OK)
            return err(ModbusError::transport("uart_driver_install"));
        if (o_.rtsPin >= 0)
            uart_set_mode(o_.port, UART_MODE_RS485_HALF_DUPLEX);
        open_ = true;
        return {};
    }
    void close() override {
        if (open_) { uart_driver_delete(o_.port); open_ = false; }
    }
    bool isOpen() const override { return open_; }

    Expected<void> write(std::span<const uint8_t> b,
                         int timeoutMs) override {
        if (!open_) return err(ModbusError::transport("esp-uart closed"));
        drainEvents();
        const int n = uart_write_bytes(o_.port, b.data(), b.size());
        if (n != int(b.size()))
            return err(ModbusError::transport("esp-uart short tx"));
        if (uart_wait_tx_done(o_.port, pdMS_TO_TICKS(timeoutMs)) != ESP_OK)
            return err(ModbusError::timeout("esp-uart drain"));
        return {};
    }
    Expected<size_t> read(std::span<uint8_t> b, int timeoutMs) override {
        if (!open_) return err(ModbusError::transport("esp-uart closed"));
        drainEvents();
        const int n = uart_read_bytes(o_.port, b.data(), b.size(),
                                      pdMS_TO_TICKS(timeoutMs));
        if (n < 0) return err(ModbusError::transport("esp-uart rx"));
        return size_t(n);
    }
    Expected<void> flush(Flush f) override {
        if (f == Flush::Rx || f == Flush::Both)
            uart_flush_input(o_.port);
        if (f == Flush::Tx || f == Flush::Both)
            uart_wait_tx_done(o_.port, pdMS_TO_TICKS(100));
        return {};
    }
    LinkInfo info() const override {
        return {"esp-uart", "UART" + std::to_string(int(o_.port)),
                LinkKind::Serial, false, true};
    }
    Expected<size_t> drain() override {
        size_t n = 0;
        uart_get_buffered_data_len(o_.port, &n);
        if (n) uart_flush_input(o_.port);
        return n;
    }

    /**
     * @brief Hardware error counters drained from the UART event queue
     * (parity/framing/fifo/buffer/break). All zero when the link was
     * opened with Opts::eventQueueLen == 0 — the driver then delivers
     * no events. Errored bytes are dropped by the IDF UART driver, so a
     * corrupted byte always shortens/garbles the frame → CRC reject.
     */
    struct ErrorCounters { uint64_t parity = 0, frame = 0, fifoOvf = 0,
                                  bufferFull = 0, brk = 0; };
    const ErrorCounters& errors() const { return errs_; }
    void resetErrors() { errs_ = ErrorCounters{}; }

private:
    /// Drain pending UART driver events into errs_ (never blocks).
    void drainEvents() {
        if (!eventQueue_) return;
        uart_event_t ev{};
        while (xQueueReceive(eventQueue_, &ev, 0) == pdTRUE) {
            switch (ev.type) {
                case UART_PARITY_ERR:  ++errs_.parity; break;
                case UART_FRAME_ERR:   ++errs_.frame; break;
                case UART_FIFO_OVF:    ++errs_.fifoOvf; break;
                case UART_BUFFER_FULL: ++errs_.bufferFull; break;
                case UART_BREAK:       ++errs_.brk; break;
                default: break;
            }
        }
    }

    Opts o_;
    SerialConfig cfg_;
    QueueHandle_t eventQueue_ = nullptr;
    ErrorCounters errs_{};
    bool open_ = false;
};

} // namespace tether::modbus

#endif // ESP_PLATFORM && TETHER_MODBUS_ESP_UART
