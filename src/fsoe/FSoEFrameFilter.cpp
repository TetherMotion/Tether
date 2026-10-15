#include "tether/fsoe/FSoEFrameFilter.hpp"

#include <algorithm>
#include <cstring>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"

namespace FSoE {

namespace SynPDO = EtherCAT::Drives::SynapticonPDO;

// ============================================================================
// NoFSoEFilter — verbatim passthrough
// ============================================================================

size_t NoFSoEFilter::filterSlaveToMaster(const uint8_t* in, size_t in_len,
                                         uint8_t* out, size_t out_cap) {
    const size_t n = std::min(in_len, out_cap);
    if (n) std::memcpy(out, in, n);
    return n;
}

size_t NoFSoEFilter::filterMasterToSlave(const uint8_t* in, size_t in_len,
                                         uint8_t* out, size_t out_cap) {
    const size_t n = std::min(in_len, out_cap);
    if (n) std::memcpy(out, in, n);
    return n;
}

// ============================================================================
// LW1ToLW2FSoEFilter — 31-byte LW1 response → 35-byte LW2 frame
// ============================================================================
//
// Wire layouts (SynapticonPDO.hpp):
//   LW1: [CMD][w0][CRC0]…[w6][CRC6][ConnID]                  31 bytes
//   LW2: [CMD][w0][CRC0]…[w6][CRC6][w7][CRC7][ConnID]        35 bytes
// The appended word w7 is the safe torque actual value (0x6616:00) —
// zeroed (an LW1 drive produces no safe torque data) — with its own
// segment CRC_7.  Because each CRC_i also folds in the header (chain
// base + ConnID + SeqNo + Cmd), the whole frame is re-serialized with a
// chain the master accepts — the embedded SlaveFrameRewriter tracks it
// from the master's outgoing frames (noteMasterFrame).

size_t LW1ToLW2FSoEFilter::filterSlaveToMaster(const uint8_t* in,
                                               size_t in_len,
                                               uint8_t* out,
                                               size_t out_cap) {
    uint8_t  cmd = 0;
    uint16_t conn_id = 0;
    size_t   data_len = 0;
    uint8_t  data[CRC::MAX_PARSE_DATA_SIZE];
    if (!CRC::extractFSoEFrame(in, in_len, cmd, data, data_len,
                               conn_id) ||
        !isFSoECommand(cmd)) {
        return 0;
    }
    // Already LW2-sized or larger — nothing to expand; let the
    // rewriter still re-chain it for the master.
    if (data_len >= SynPDO::fsoeSafeDataSize(SynPDO::FSoEFrameVariant::LW2)) {
        return rewriter_.rewrite(in, in_len, out, out_cap);
    }
    // Pad the tail with zeroed safe-data words up to the LW2 size
    // (word 7 = safe torque actual value 0x6616, absent on LW1).
    const size_t target =
        SynPDO::fsoeSafeDataSize(SynPDO::FSoEFrameVariant::LW2);
    std::memset(data + data_len, 0, target - data_len);
    data_len = target;

    // Re-serialize at the LW2 length (layout only — CRC chain
    // placeholders are rebuilt by the rewriter below).
    uint8_t tmp[CRC::MAX_PARSE_DATA_SIZE * 3];
    const size_t tmp_len = CRC::buildFSoEFrame(
        tmp, cmd, data, data_len, conn_id, /*start_crc=*/0,
        kInitialSeqNo);
    if (tmp_len == 0 || tmp_len > out_cap) return 0;
    return rewriter_.rewrite(tmp, tmp_len, out, out_cap);
}

size_t LW1ToLW2FSoEFilter::filterMasterToSlave(const uint8_t* in,
                                               size_t in_len,
                                               uint8_t* out,
                                               size_t out_cap) {
    // Master→slave command frames are layout-identical for LW1/LW2.
    const size_t n = std::min(in_len, out_cap);
    if (n) std::memcpy(out, in, n);
    return n;
}

} // namespace FSoE
