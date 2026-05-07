/**
 * @file SLIPStreamDecoder.hpp
 * @author Uli Köhler <github@techoverflow.net>
 * @version 1.0
 * @date 2025-08-19
 * 
 * @copyright Copyright (C) 2022..2025 Uli Köhler
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>
#include "SLIPStream/SLIP.hpp"

namespace SLIPStream {

enum class LogType: uint8_t {
    Unknown = 0,
    RXBufferOverflow = 1
};

// Stateful decoder for SLIP messages
class Decoder {
public:
    Decoder(uint8_t* rxbuf, size_t rxbufSize, std::function<void(uint8_t*, size_t)> messageCallback, std::function<void(LogType, const char*)> logCallback);

    void consume(const uint8_t* data, size_t size);
    void consume(uint8_t byte);

    // Clear RX buf etc
    void reset();
    
private:
    bool lastCharIsEsc;
    uint8_t* rxbuf;
    size_t rxbufPos;
    size_t rxbufSize;
    std::function<void(uint8_t*, size_t)> messageCallback;
    std::function<void(LogType, const char*)> logCallback;

    // The following are for logging only
    const char* logTag; // Tag for logging, like "ZMCU-SLIP"
};

} // namespace SLIPStream
