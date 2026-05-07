#include "SLIPStream/Encoder.hpp"

namespace SLIPStream {

Encoder::Encoder(OutputFn outputFn, size_t txBufferSize, size_t maxSendChunk)
    : outputFn(std::move(outputFn)), txBuf(txBufferSize), txHead(0), txTail(0), txSize(0), maxSendChunk(maxSendChunk), endPending(false) {}

bool Encoder::queueByte(uint8_t b) {
    if (txSize >= txBuf.size()) return false; // full
    txBuf[txTail] = b;
    txTail = (txTail + 1) % txBuf.size();
    txSize++;
    return true;
}

bool Encoder::dequeueByte(uint8_t& b) {
    if (txSize == 0) return false;
    b = txBuf[txHead];
    txHead = (txHead + 1) % txBuf.size();
    txSize--;
    return true;
}

WriteStatus Encoder::flush() {
    size_t sent = 0;
    while (txSize > 0 && sent < maxSendChunk) {
        uint8_t b;
        // Peek without removing
        b = txBuf[txHead];
        WriteStatus st = outputFn(b);
        if (st == WriteStatus::Ok) {
            // Actually remove
            (void)dequeueByte(b);
            sent++;
            continue;
        } else if (st == WriteStatus::RetryLater) {
            return WriteStatus::RetryLater;
        } else {
            return WriteStatus::Error;
        }
    }
    return WriteStatus::Ok;
}

WriteStatus Encoder::ensureFree(size_t n) {
    if (txBuf.size() - txSize >= n) return WriteStatus::Ok;
    // Try to flush some bytes
    WriteStatus st = flush();
    if (st != WriteStatus::Ok) return st;
    return (txBuf.size() - txSize >= n) ? WriteStatus::Ok : WriteStatus::RetryLater;
}

WriteStatus Encoder::encodeOne(const uint8_t* data, size_t size, size_t& consumed) {
    consumed = 0;
    if (size == 0) return WriteStatus::Ok;
    uint8_t c = *data;
    if (c == END) {
        // Need two bytes: ESC, ESCEND
        WriteStatus st = ensureFree(2);
        if (st != WriteStatus::Ok) return st;
        if (!queueByte(ESC)) return WriteStatus::RetryLater;
        if (!queueByte(ESCEND)) return WriteStatus::RetryLater;
    } else if (c == ESC) {
        // Need two bytes: ESC, ESCESC
        WriteStatus st = ensureFree(2);
        if (st != WriteStatus::Ok) return st;
        if (!queueByte(ESC)) return WriteStatus::RetryLater;
        if (!queueByte(ESCESC)) return WriteStatus::RetryLater;
    } else {
        WriteStatus st = ensureFree(1);
        if (st != WriteStatus::Ok) return st;
        if (!queueByte(c)) return WriteStatus::RetryLater;
    }
    consumed = 1;
    return WriteStatus::Ok;
}

std::pair<WriteStatus, size_t> Encoder::pushPacket(const uint8_t* data, size_t size) {
    size_t consumed = 0;
    // First, try to send any already queued bytes for fairness
    WriteStatus st = flush();
    if (st == WriteStatus::Error) return {st, consumed};
    if (st == WriteStatus::RetryLater) return {st, consumed};

    // If we were in the middle of final END from previous call, attempt it
    if (endPending) {
        // Ensure space then queue END
        st = ensureFree(1);
        if (st != WriteStatus::Ok) return {st, consumed};
        queueByte(END);
        endPending = false;
        // Try to send a bit immediately to reduce latency
        st = flush();
        if (st != WriteStatus::Ok) return {st, consumed};
    }

    // Encode payload bytes
    while (consumed < size) {
        size_t c = 0;
        st = encodeOne(data + consumed, size - consumed, c);
        if (st != WriteStatus::Ok) return {st, consumed};
        consumed += c;
        // Opportunistic small flush to respect maxSendChunk pacing
        st = flush();
        if (st == WriteStatus::Error) return {st, consumed};
        if (st == WriteStatus::RetryLater) return {st, consumed};
    }

    // Append END terminator for the packet
    st = ensureFree(1);
    if (st != WriteStatus::Ok) {
        if (st == WriteStatus::RetryLater) {
            endPending = true; // Remember to append END on next call
        }
        return {st, consumed};
    }
    queueByte(END);

    // Final flush attempt
    st = flush();
    if (st != WriteStatus::Ok) return {st, consumed};
    return {WriteStatus::Ok, consumed};
}

} // namespace SLIPStream