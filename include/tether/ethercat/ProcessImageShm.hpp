/**
 * @file ProcessImageShm.hpp
 * @brief Shared-memory layout descriptors for the process image export.
 *
 * Split out of ProcessImage.hpp.
 */

#pragma once

#include <cstdint>

namespace EtherCAT {

struct ShmEntryDesc {
    uint32_t index;          ///< mapping-entry index (entryHandle().index)
    uint32_t slave_index;
    uint32_t pdo_index;      ///< e.g. 0x1600
    uint32_t direction;      ///< PDODirection as u32
    int32_t  offset;         ///< byte offset in its region (-1 = buffered)
    uint32_t size;           ///< bytes
    char     label[16];      ///< human tag, e.g. "rx:s2:0x1600" (may be empty)
};

struct ShmImageHeader {
    uint32_t magic;           ///< kShmMagic
    uint32_t version;         ///< kShmVersion
    uint32_t rx_bytes;        ///< output region bytes
    uint32_t tx_bytes;        ///< input region bytes
    uint32_t header_bytes;    ///< = sizeof(ShmImageHeader) — fwd compat
    uint32_t epoch;
    std::atomic<uint32_t> in_seq;      ///< input publish counter (futex word)
    std::atomic<uint32_t> in_waiters;  ///< registered input waiters
    std::atomic<uint32_t> out_seq;     ///< output commit counter
    std::atomic<uint32_t> out_waiters; ///< registered output waiters
    std::atomic<uint32_t> send_seq;    ///< async-loop send-request counter
    std::atomic<uint32_t> send_waiters;///< registered send waiters
    // ---- handshake (Q20): exporter fills all fields + checksum, then
    // ---- release-publishes ready=1 last.  Attachers must check ready and
    // ---- validate checksum before trusting any offset/size field.
    std::atomic<uint32_t> ready;       ///< 1 = export complete & consistent
    std::atomic<uint32_t> checksum;    ///< FNV-1a over fixed sizing fields
    // ---- async-loop additions ----
    std::atomic<uint64_t> send_stamp_ns;///< mono stamp of last triggerSend
    std::atomic<uint32_t> send_epoch;   ///< waiters generation (Q25 reclaim)
    std::atomic<uint32_t> flags;        ///< kShmFlag* feature bits
    // ---- optional seqlock words (Q5), one per region ----
    std::atomic<uint32_t> rx_lock_seq;  ///< output region: odd = write active
    std::atomic<uint32_t> tx_lock_seq;  ///< input region:  odd = write active
    // ---- entry-table export (Q4) ----
    std::atomic<uint32_t> entry_count;  ///< rows published (post-ready)
    uint32_t entry_table_off;           ///< byte offset of ShmEntryDesc[]
    // ---- per-producer trigger accounting (Q26) ----
    std::atomic<uint32_t> producer_triggers[8]; ///< bump per triggerSend(id)
    std::atomic<uint32_t> producer_claims;      ///< slot bitmap (claimProducerSlot)
};
inline constexpr uint32_t kShmMagic   = 0x54494D47;  ///< 'TIMG'
inline constexpr uint32_t kShmVersion = 2;
/// shm header flag bits.
inline constexpr uint32_t kShmFlagSeqlock    = 1u << 0;  ///< region seqlocks on
inline constexpr uint32_t kShmFlagEntryTable = 1u << 1;  ///< entry table present
/// Max producer slots in producer_triggers[] (registerProducer).
inline constexpr uint32_t kMaxShmProducers = 8;
/// Payload layout: [ShmImageHeader][output rx_bytes][input tx_bytes][entries],
/// regions 64-byte aligned after the header.
struct ShmImageLayout {
    static constexpr uint32_t kHeaderPadded = 256;
    static uint32_t outputOff()                { return kHeaderPadded; }
    static uint32_t inputOff(uint32_t rx)      { return kHeaderPadded + rx; }
    /// Byte offset where the exported entry table begins (when present).
    static uint32_t entryTableOff(uint32_t rx, uint32_t tx) {
        return kHeaderPadded + rx + tx;
    }
    static uint32_t totalBytes(uint32_t rx, uint32_t tx,
                               uint32_t entry_rows = 0) {
        return kHeaderPadded + rx + tx + entry_rows * sizeof(ShmEntryDesc);
    }
};

} // namespace EtherCAT
