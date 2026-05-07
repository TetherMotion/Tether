#include "SLIPStream/SLIP.hpp"
#include "SLIPStream/Decoder.hpp"

namespace SLIPStream {

Decoder::Decoder(uint8_t* rxbuf, size_t rxbufSize, std::function<void(uint8_t*, size_t)> messageCallback, std::function<void(LogType, const char*)> logCallback)
    : lastCharIsEsc(false), rxbuf(rxbuf), rxbufPos(0), rxbufSize(rxbufSize), messageCallback(messageCallback), logCallback(logCallback) {
}


void Decoder::consume(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; i++)
    {
        consume(data[i]);
    }
    
}

void Decoder::consume(uint8_t c) {
    if(rxbufSize - rxbufPos < 2) {
        logCallback(LogType::RXBufferOverflow, "RX buffer overflow");
		reset();
    }
    // Adapted from https://techoverflow.net/2022/07/19/a-python-slip-decoder-using-serial_asyncio/
    if(lastCharIsEsc) {
        // This character must be either
        // SLIP_ESCEND or SLIP_ESCESC
        if(c == ESCEND) { // Literal END character
            rxbuf[rxbufPos++] = END;
        } else if(c == ESCESC) { // Literal ESC character
            rxbuf[rxbufPos++] = ESC;
        } else {
            //print(red("Encountered invalid SLIP escape sequence. Ignoring..."))
            // Ignore bad part of message
            reset();
        }
        lastCharIsEsc = false; // Reset state
    } else { // last char was NOT ESC
        if(c == END) { // END of message
            // Emit message
            messageCallback(rxbuf, rxbufPos);
            // Remove current message from buffer
            reset();
        } else if(c == ESC) {
            // Handle escaped character next 
            lastCharIsEsc = true;
        } else { // Any other character
            rxbuf[rxbufPos++] = c;
        }
    }
}

void Decoder::reset() {
    rxbufPos = 0;
	lastCharIsEsc = false;
}

} // namespace SLIPStream
