/**
 * @file Master_cyclic.cpp
 * @brief Master — forwarding shims for the cyclic datapath.
 *
 * The public/test surface of the cyclic fast path is unchanged; the state and
 * the work live on CyclicDatapath (CyclicDatapath.{hpp,cpp}).  TU split out of
 * CyclicDatapath.cpp so that file holds only the datapath class.
 */

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/CyclicChannel.hpp"
#include "tether/ethercat/ProcessImage.hpp"
#include "raw/CyclicDatapath.hpp"

#include <string>

namespace EtherCAT {

// Master forwarders — the public/test surface is unchanged; the state and
// the work live on CyclicDatapath.
// ============================================================================

uint64_t Master::cyclicSlotToken(uint8_t slot) const
{
    return datapath_->slotToken(slot);
}

uint8_t Master::cyclicSlotGen(uint8_t slot) const
{
    return datapath_->slotGen(slot);
}

uint64_t Master::sliceSlotToken(uint8_t slice) const
{
    return datapath_->sliceSlotToken(slice);
}

uint8_t Master::sliceSlotGen(uint8_t slice) const
{
    return datapath_->sliceSlotGen(slice);
}

bool Master::sendSliceDatagram(Command cmd, uint8_t slice_slot,
                               uint16_t adp, uint16_t ado,
                               const void* data, uint16_t datalen,
                               bool roundtrip)
{
    return datapath_->sendSliceDatagram(cmd, slice_slot, adp, ado,
                                      data, datalen, roundtrip);
}

bool Master::waitSliceSlotView(uint8_t slice, uint64_t token,
                               uint32_t timeout_ns, CyclicSlotView& out)
{
    return datapath_->waitSliceView(slice, token, timeout_ns, out);
}

uint32_t Master::waitSliceSlotMask(uint32_t slice_mask,
                                   const uint64_t* tokens,
                                   uint32_t timeout_ns,
                                   CyclicSlotView* views)
{
    return datapath_->waitSliceMask(slice_mask, tokens, timeout_ns, views);
}

uint32_t Master::cyclicPayloadOffset() const
{
    return datapath_->payloadOffset();
}

bool Master::sendCyclicDatagram(Command cmd, uint8_t slot,
                                uint16_t adp, uint16_t ado,
                                const void* data, uint16_t datalen,
                                bool roundtrip)
{
    return datapath_->sendDatagram(cmd, slot, adp, ado,
                                   data, datalen, roundtrip);
}

bool Master::sendCyclicPoolFrame(const CyclicDgramSpec* dgs, size_t count)
{
    return datapath_->sendPoolFrame(dgs, count);
}

uint8_t Master::waitCyclicPool(const uint8_t* positions,
                               const uint64_t* tokens,
                               uint8_t count, uint32_t timeout_ns,
                               CyclicSlotView* views, bool* arrived)
{
    return datapath_->waitPool(positions, tokens, count, timeout_ns,
                               views, arrived);
}

uint8_t* Master::acquireCyclicTxFrame() { return datapath_->acquireTxFrame(); }

void Master::composeCyclicHeader(uint8_t* frame, Command cmd, uint8_t slot,
                                 uint16_t adp, uint16_t ado, uint16_t datalen,
                                 bool roundtrip)
{
    datapath_->composeHeader(frame, cmd, slot, adp, ado, datalen, roundtrip);
}

bool Master::sendCyclicFrame(uint32_t frame_len)
{
    return datapath_->sendFrame(frame_len);
}

void Master::dispatchChannelFrame(const CyclicFrameView& v)
{
    datapath_->dispatchFrame(v);
}

bool Master::waitCyclicSlotView(uint8_t slot, uint64_t token,
                                uint32_t timeout_ns, CyclicSlotView& out)
{
    return datapath_->waitView(slot, token, timeout_ns, out);
}

uint32_t Master::waitCyclicSlotMask(uint32_t slot_mask,
                                    const uint64_t* tokens,
                                    uint32_t timeout_ns,
                                    CyclicSlotView* views)
{
    return datapath_->waitMask(slot_mask, tokens, timeout_ns, views);
}

bool Master::waitCyclicSlot(uint8_t slot, uint64_t token,
                            uint32_t timeout_ns, RxDatagram& out)
{
    return datapath_->wait(slot, token, timeout_ns, out);
}

void Master::depositCyclicSlot(uint8_t idx, Command cmd,
                               uint16_t adp, uint16_t ado,
                               const uint8_t* payload, uint16_t datalen,
                               uint16_t wkc, uint8_t gen)
{
    datapath_->deposit(idx, cmd, adp, ado, payload, datalen, wkc, gen);
}

void Master::publishCyclicSlotView(uint8_t idx, Command cmd,
                                   uint16_t adp, uint16_t ado,
                                   const uint8_t* payload, uint16_t datalen,
                                   uint16_t wkc, uint32_t cookie,
                                   uint64_t stamp_ns, uint8_t gen)
{
    datapath_->publishView(idx, cmd, adp, ado, payload, datalen,
                           wkc, cookie, stamp_ns, gen);
}

void Master::setupCyclicDatapath(CyclicWireMode wire_mode,
                                 ImageMode image_mode,
                                 const std::string& shm_image_name,
                                 uint32_t rx_spin_ns,
                                 uint32_t slot_spin_ns,
                                 CyclicLoopConfig::SlotWaitFallback slot_fallback,
                                 bool strict_wkc,
                                 const MemoryLockConfig& memlock)
{
    datapath_->setup(wire_mode, image_mode, shm_image_name, rx_spin_ns,
                     slot_spin_ns, slot_fallback, strict_wkc, memlock);
}

void Master::teardownCyclicDatapath()
{
    datapath_->teardown();
}

ICyclicChannel* Master::cyclicChannel() const
{
    return datapath_->channel_.get();
}

ProcessImage& Master::processImage() { return datapath_->image_; }
const ProcessImage& Master::processImage() const { return datapath_->image_; }

bool Master::cyclicExchangeSuspended() const
{
    return datapath_->suspended();
}

} // namespace EtherCAT

