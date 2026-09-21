/**
 * @file Master_batch.cpp
 * @brief Batch register transactions + idx allocator reset.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/Slave.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <vector>

namespace EtherCAT {

static const char* TAG = "ethercat";

// ============================================================================
// BatchTransaction implementation
// ============================================================================

Master::BatchTransaction::BatchTransaction(TransactionRouter* router,
                                             std::vector<uint8_t> idxs,
                                             std::vector<size_t> slots,
                                             std::vector<RxDatagram> responses)
    : router_(router)
    , idxs_(std::move(idxs))
    , slots_(std::move(slots))
    , responses_(std::move(responses))
{
}

Master::BatchTransaction::~BatchTransaction()
{
    // Clean up any unclaimed slots
    if (router_) {
        for (size_t i = 0; i < slots_.size(); i++) {
            if (slots_[i] < TransactionRouter::kNumSlots) {
                router_->cancelPreRegistered(slots_[i]);
            }
        }
    }
}

BatchReadResult Master::BatchTransaction::getResult(size_t i, uint32_t timeout_ms)
{
    BatchReadResult result;
    if (i >= idxs_.size() || !router_) return result;

    if (slots_[i] >= TransactionRouter::kNumSlots) return result;

    // If already completed (response was routed before we called), return it
    if (responses_[i].wkc != 0 || responses_[i].datalen > 0) {
        result.success = true;
        result.wkc = responses_[i].wkc;
        result.datalen = responses_[i].datalen;
        result.data = responses_[i].data;
        return result;
    }

    // Wait for the response
    WaitResult wr = router_->waitForPreRegistered(slots_[i], timeout_ms);
    if (wr.success) {
        result.success = true;
        result.wkc = wr.wkc;
        result.datalen = wr.data_length;
        result.data = responses_[i].data;
    }

    return result;
}

bool Master::BatchTransaction::waitAll(uint32_t timeout_ms,
                                        std::vector<BatchReadResult>& out)
{
    out.resize(idxs_.size());
    bool all_ok = true;
    for (size_t i = 0; i < idxs_.size(); i++) {
        out[i] = getResult(i, timeout_ms);
        if (!out[i].success) all_ok = false;
    }
    return all_ok;
}

void Master::BatchTransaction::cancel()
{
    cancelled_ = true;
    if (router_) {
        for (size_t i = 0; i < slots_.size(); i++) {
            if (slots_[i] < TransactionRouter::kNumSlots) {
                router_->cancelPreRegistered(slots_[i]);
            }
        }
    }
}

// ============================================================================
// Batch read/write APIs
// ============================================================================

Master::BatchTransaction Master::readRegistersBatch(
    const SlaveAddress* slave_addresses,
    const uint16_t* register_addresses,
    const uint16_t* lengths,
    size_t count)
{
    if (count == 0 || !slave_addresses || !register_addresses || !lengths)
        return BatchTransaction();

    std::vector<MultiDatagramSpec> specs(count);
    std::vector<uint8_t> idxs(count);
    std::vector<size_t> slots(count);
    std::vector<RxDatagram> responses(count);

    for (size_t i = 0; i < count; i++) {
        idxs[i] = allocIdx();
        slots[i] = idxs[i]; // slot index == idx in TransactionRouter

        Command cmd = slave_addresses[i].isPhysical() ? Command::APRD : Command::FPRD;
        specs[i] = MultiDatagramSpec{
            cmd,
            idxs[i],
            slave_addresses[i].raw(),
            register_addresses[i],
            nullptr,        // no data for reads
            lengths[i],
            true            // roundtrip
        };

        // Pre-register the waiter slot
        PacketFilter filter = PacketFilter::byIndex(idxs[i]);
        slots[i] = packet_router_.preRegisterWaiter(
            filter, responses[i].data, sizeof(responses[i].data));
    }

    // Send all datagrams in one frame (auto-splits if needed)
    size_t sent = sendMultiDatagram(specs.data(), count);
    if (sent == 0) {
        // Send failed — cancel all pre-registered slots
        for (size_t i = 0; i < count; i++) {
            if (slots[i] < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slots[i]);
        }
        return BatchTransaction();
    }

    return BatchTransaction(&packet_router_, std::move(idxs), std::move(slots),
                            std::move(responses));
}

Master::BatchTransaction Master::writeRegistersBatch(
    const SlaveAddress* slave_addresses,
    const uint16_t* register_addresses,
    const void* const* data,
    const uint16_t* lengths,
    size_t count)
{
    if (count == 0 || !slave_addresses || !register_addresses || !data || !lengths)
        return BatchTransaction();

    std::vector<MultiDatagramSpec> specs(count);
    std::vector<uint8_t> idxs(count);
    std::vector<size_t> slots(count);
    std::vector<RxDatagram> responses(count);

    for (size_t i = 0; i < count; i++) {
        idxs[i] = allocIdx();
        slots[i] = idxs[i];

        Command cmd = slave_addresses[i].isPhysical() ? Command::APWR : Command::FPWR;
        specs[i] = MultiDatagramSpec{
            cmd,
            idxs[i],
            slave_addresses[i].raw(),
            register_addresses[i],
            data[i],
            lengths[i],
            true            // roundtrip
        };

        PacketFilter filter = PacketFilter::byIndex(idxs[i]);
        slots[i] = packet_router_.preRegisterWaiter(
            filter, responses[i].data, sizeof(responses[i].data));
    }

    size_t sent = sendMultiDatagram(specs.data(), count);
    if (sent == 0) {
        for (size_t i = 0; i < count; i++) {
            if (slots[i] < TransactionRouter::kNumSlots)
                packet_router_.cancelPreRegistered(slots[i]);
        }
        return BatchTransaction();
    }

    return BatchTransaction(&packet_router_, std::move(idxs), std::move(slots),
                            std::move(responses));
}


void Master::resetIdx()
{
    next_idx_.store(0, std::memory_order_relaxed);
}

} // namespace EtherCAT
