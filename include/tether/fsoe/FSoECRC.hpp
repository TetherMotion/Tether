#pragma once

// ============================================================================
// FSoE CRC-16 — Correct implementation per ETG.5100
// ============================================================================
//
// This implementation follows the FSoE (Functional Safety over EtherCAT)
// CRC-16 algorithm as specified in ETG.5100 S (D) V1.2.0, section 8.1.3.2.
//
// The FSoE CRC is NOT a standard CRC-16.  It uses a custom per-byte update
// step that gives each input byte an extra factor of x^24 compared to a
// standard byte-wise CRC.  This is achieved by using TWO lookup tables:
//
//   T0[i] = (i * x^16) mod P    — the standard byte-wise table (k=0)
//   T3[i] = (i * x^40) mod P    — the slice-by-4 table (k=3)
//
// The per-byte update step is:
//
//   crc_new = (crc_lo << 8) ^ T0[crc_hi] ^ T3[input]
//
// Compare with a standard CRC-16 update:
//
//   crc_std = (crc_lo << 8) ^ T0[crc_hi ^ input]
//           = (crc_lo << 8) ^ T0[crc_hi] ^ T0[input]
//
// The only difference is T3[input] vs T0[input] — an extra factor of x^24
// per byte.  Equivalently, the FSoE CRC is the CRC of the message as if
// each byte were followed by 3 zero bytes, but with the initial value
// shifted only by the actual byte count (not the padded count).
//
// References (TechOverflow):
//   - How to compute the CRC checksum for FSoE PDUs:
//     https://techoverflow.net/2026/08/09/how-to-compute-the-crc-checksum-for-fsoe-pdus/
//   - FSoE CRC: Which polynomial does it use?
//     https://techoverflow.net/2026/06/25/fsoe-crc-which-polynomial-does-it-use/
//   - How are the FSoE CRC tables constructed?
//     https://techoverflow.net/2026/08/09/how-are-the-fsoe-crc-tables-constructed/
//   - FSoE: How does CRC inheritance work?
//     https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
//
// SPDX-License-Identifier: CC0-1.0 (algorithm implementation);
// see TechOverflow posts for original source.
// ============================================================================

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>

namespace FSoE::CRC {

// ---------------------------------------------------------------------------
// CRC Error Detail
// ---------------------------------------------------------------------------

/// Diagnostic detail populated by parseFSoEFrame when CRC verification fails.
///
/// FSoE frames carry multiple CRC-16 segments (one per 2 data bytes).  This
/// struct identifies which segment failed and what the expected/received
/// values were, so callers can produce verbose error messages instead of a
/// bare "CRCError".
struct CrcErrorDetail {
    bool valid = false;             ///< Set to true when the struct is populated
    int segment_index = -1;         ///< 0-based index of the failing CRC segment
    uint16_t expected_crc = 0;      ///< CRC value computed by the receiver
    uint16_t received_crc = 0;      ///< CRC value stored in the frame
    size_t frame_offset = 0;        ///< Byte offset of the failing CRC in the frame
};

// ---------------------------------------------------------------------------
// Polynomial
// ---------------------------------------------------------------------------
//
// FSoE uses the 17-bit polynomial:
//
//   P(x) = x^16 + x^13 + x^12 + x^11 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
//
// Packed as a hexadecimal value WITH the implicit x^16 term: 0x139B7.
// The 16-bit representation used inside the shift register (WITHOUT the
// x^16 term): 0x39B7.
//
// The CRC is processed MSB-first (the "normal" or non-reflected direction):
// each message bit is taken from the most-significant end of the working
// register, and the polynomial is XORed in when the shifted-out bit is 1.
//
// Source: ETG.5100 S (D) V1.2.0, section 8.1.3.2: CRC polynomial selection.
// See: https://techoverflow.net/2026/06/25/fsoe-crc-which-polynomial-does-it-use/
inline constexpr uint16_t kPolynomial     = 0x39B7;  // 16-bit form (x^16 implicit)
inline constexpr uint32_t kPolynomialFull = 0x139B7; // 17-bit form (for reference)

// ---------------------------------------------------------------------------
// Compile-time table choice
// ---------------------------------------------------------------------------
//
// By default, the table-based implementation is used (two 256-entry, 16-bit
// lookup tables, ~1 KB total).  Define TETHER_FSOE_CRC_NO_TABLE=1 before
// including this header to use the on-the-fly implementation instead, which
// computes each table entry on demand using bit-by-bit polynomial long
// division.  Both variants produce identical results.
#ifdef TETHER_FSOE_CRC_NO_TABLE
inline constexpr bool kUseTable = false;
#else
inline constexpr bool kUseTable = true;
#endif

// ---------------------------------------------------------------------------
// constexpr table generation
// ---------------------------------------------------------------------------
//
// The two FSoE tables are derived purely from the polynomial — no hand-typed
// constants.  Each entry is computed by loading the byte value into the high
// byte of a 16-bit register and shifting left, XORing the polynomial in
// whenever bit 15 is set.
//
//   T0[i] = (i * x^16) mod P   — 8 left shifts of i << 8  (k=0)
//   T3[i] = (i * x^40) mod P   — 32 left shifts of i << 8 (k=3)
//
// T3 is equivalently the result of applying T0 four times starting from
// byte i (i.e., processing byte i followed by three zero bytes).
//
// See: https://techoverflow.net/2026/08/09/how-are-the-fsoe-crc-tables-constructed/

/// Compute (byte * x^(8+shifts)) mod P by bit-by-bit polynomial long division.
/// Loads byte into the high byte of a 16-bit register and shifts left
/// 'shifts' times, XORing POLY in whenever bit 15 is set.
constexpr uint16_t polyMod(uint8_t byte, int shifts) {
    uint16_t r = static_cast<uint16_t>(byte) << 8;
    for (int i = 0; i < shifts; i++)
        r = (r & 0x8000U) ? static_cast<uint16_t>((r << 1) ^ kPolynomial)
                          : static_cast<uint16_t>(r << 1);
    return r;
}

/// Build T0: the ordinary byte-wise table (k=0, 8 shifts).
/// T0[i] = (i * x^16) mod P
constexpr std::array<uint16_t, 256> makeTable0() {
    std::array<uint16_t, 256> t{};
    for (int i = 0; i < 256; i++)
        t[i] = polyMod(static_cast<uint8_t>(i), 8);
    return t;
}

/// Build T3: the slice-by-4 table (k=3, 32 shifts).
/// T3[i] = (i * x^40) mod P
constexpr std::array<uint16_t, 256> makeTable3() {
    std::array<uint16_t, 256> t{};
    for (int i = 0; i < 256; i++)
        t[i] = polyMod(static_cast<uint8_t>(i), 32);
    return t;
}

/// Statically precomputed lookup tables (constexpr-generated at compile time).
/// See: https://techoverflow.net/2026/08/09/how-are-the-fsoe-crc-tables-constructed/
inline constexpr auto crcTable0 = makeTable0();  // T0[i] = (i * x^16) mod P
inline constexpr auto crcTable3 = makeTable3();  // T3[i] = (i * x^40) mod P

// ---------------------------------------------------------------------------
// Low-level CRC update
// ---------------------------------------------------------------------------

/// FSoE per-byte CRC update step:
///
///   new_crc = (crc_lo << 8) ^ T0[crc_hi] ^ T3[input]
///
/// The term (crc_lo << 8) ^ T0[crc_hi] is exactly the standard MSB-first
/// CRC register shift by one byte (equivalent to processing a zero byte).
/// Then T3[input] is XORed in, giving each input byte an extra factor of
/// x^24 compared to a standard CRC.
///
/// When TETHER_FSOE_CRC_NO_TABLE is defined, T0 and T3 are computed on the
/// fly using polyMod() — slower (8+32 shifts per byte) but self-contained.
///
/// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
inline uint16_t updateCrc(uint16_t crc, uint8_t input) {
    if constexpr (kUseTable) {
        return static_cast<uint16_t>(
            ((crc & 0xFF) << 8) ^ crcTable0[crc >> 8] ^ crcTable3[input]);
    } else {
        uint16_t t0 = polyMod(static_cast<uint8_t>(crc >> 8), 8);
        uint16_t t3 = polyMod(input, 32);
        return static_cast<uint16_t>(((crc & 0xFF) << 8) ^ t0 ^ t3);
    }
}

/// Process a 16-bit value in little-endian byte order (lo first, then hi).
inline uint16_t updateCrc16(uint16_t crc, uint16_t word) {
    crc = updateCrc(crc, static_cast<uint8_t>(word & 0xFF));
    crc = updateCrc(crc, static_cast<uint8_t>((word >> 8) & 0xFF));
    return crc;
}

// ---------------------------------------------------------------------------
// High-level FSoE CRC functions
// ---------------------------------------------------------------------------
//
// The CRC is reset to 0 at the start of each computation, then bytes are
// processed in this order:
//
//   oldCRC-Lo, oldCRC-Hi, ConnID-Lo, ConnID-Hi, SeqNo-Lo, SeqNo-Hi,
//   Command, Data[0], [Data[1], ...]
//
// For PDUs larger than 10 bytes, multiple CRCs are computed — each segment
// CRC restarts from a shared crc_common base (the state after processing
// the first 7 header bytes) and adds an index byte pair and 2 data bytes.
//
// CRC inheritance — two levels:
//
// 1. Cross-frame: The first two bytes folded into every CRC calculation are
//    the PREVIOUS frame's CRC0 (startCrc).  This creates a chain across
//    frames — an attacker cannot replay an old frame because the CRC chain
//    would break.
//
// 2. Intra-frame: Within a single PDU with multiple CRCs, each segment CRC
//    restarts from crc_common (the shared base after processing oldCRC,
//    ConnID, SeqNo, and Command).  CRC0 continues from crc_common with
//    Data[0..1]; CRCi (i >= 1) restarts from crc_common with Index(i) and
//    Data[2i, 2i+1].  The index byte ensures that even if two segments
//    contain identical data, their CRCs differ.
//
// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/

/// Compute CRC0: the first (and possibly only) CRC for an FSoE PDU.
///
/// Processes: startCrc(2), ConnID(2), SeqNo(2), Command(1), Data[0..dataLen-1]
/// dataLen is 1 for PDU size <= 6, 2 for PDU size > 6.
///
/// @param startCrc  The previous frame's CRC0 (CRC inheritance).  0 for the
///                  very first frame.
/// @param connId    The connection ID (read from the end of the PDU).
/// @param seqNo     The sequence number (not transmitted, but shared between
///                  master and slave).
/// @param command   The command byte.
/// @param data      Pointer to the safe data bytes.
/// @param dataLen   Number of data bytes to include (1 or 2).
/// @return The CRC0 value.  This becomes startCrc for the next frame.
inline uint16_t computeCrc0(uint16_t startCrc, uint16_t connId,
                             uint16_t seqNo, uint8_t command,
                             const uint8_t* data, int dataLen) {
    uint16_t crc = 0;
    crc = updateCrc16(crc, startCrc);   // oldCRC-Lo, oldCRC-Hi
    crc = updateCrc16(crc, connId);     // ConnID-Lo, ConnID-Hi
    crc = updateCrc16(crc, seqNo);      // SeqNo-Lo, SeqNo-Hi
    crc = updateCrc(crc, command);      // Command
    for (int i = 0; i < dataLen; i++)
        crc = updateCrc(crc, data[i]);  // Data bytes
    return crc;
}

/// Compute crc_common: the shared base for multi-CRC segments.
///
/// Processes: startCrc(2), ConnID(2), SeqNo(2), Command(1) — the 7 header
/// bytes.  This base encapsulates all the "header" fields: the inherited
/// old CRC, the Connection ID, the Sequence Number, and the Command byte.
/// It is saved and reused as the starting point for each segment CRC.
///
/// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
inline uint16_t computeCrcCommon(uint16_t startCrc, uint16_t connId,
                                  uint16_t seqNo, uint8_t command) {
    uint16_t crc = 0;
    crc = updateCrc16(crc, startCrc);
    crc = updateCrc16(crc, connId);
    crc = updateCrc16(crc, seqNo);
    crc = updateCrc(crc, command);
    return crc;
}

/// Compute CRCi (i >= 1): a subsequent segment CRC from crc_common.
///
/// Restarts from crc_common, then processes: Index(2), Data[2i], Data[2i+1].
/// The index byte pair (1-based, little-endian) ensures that even if two
/// segments contain identical data bytes, their CRCs will differ because
/// the index differs.  Without the index, swapping two data segments would
/// go undetected.
///
/// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/
inline uint16_t computeCrcI(uint16_t crcCommon, uint16_t index,
                             uint8_t data0, uint8_t data1) {
    uint16_t crc = crcCommon;           // restart from shared base
    crc = updateCrc16(crc, index);      // Index-Lo, Index-Hi
    crc = updateCrc(crc, data0);        // Data[2i]
    crc = updateCrc(crc, data1);        // Data[2i+1]
    return crc;
}

/// Compute all CRCs for an FSoE PDU.
///
/// @param startCrc  Previous frame's CRC0 (0 for the first frame).
/// @param connId    Connection ID.
/// @param seqNo     Sequence number (shared between master and slave).
/// @param command   Command byte.
/// @param data      Safe data bytes.
/// @param numData   Number of data bytes in the PDU.
/// @param pduSize   Total PDU size in bytes (CMD + data + CRCs + ConnID).
/// @return Vector of CRCs: [CRC0, CRC1, CRC2, ...].  The first element
///         becomes startCrc for the next frame (CRC inheritance).
inline std::array<uint16_t, 16> computeAllCrcs(uint16_t startCrc,
                                               uint16_t connId,
                                               uint16_t seqNo,
                                               uint8_t command,
                                               const uint8_t* data,
                                               int numData,
                                               int pduSize) {
    std::array<uint16_t, 16> crcs{};
    int numCrcs = 0;

    // First 1-2 data bytes (before CRC0).
    // PDU size <= 6: 1 data byte; PDU size > 6: 2 data bytes.
    int firstData = (pduSize > 6) ? 2 : 1;

    // CRC0: oldCRC + ConnID + SeqNo + Cmd + Data[0..firstData-1]
    crcs[numCrcs++] = computeCrc0(startCrc, connId, seqNo, command,
                                   data, firstData);

    // Additional segment CRCs (if there are more data bytes beyond the
    // first segment).  Each additional segment covers 2 data bytes (or
    // 1 data byte + 1 zero-padding byte for odd lengths).
    // Segment layout: 2 data bytes + 2 CRC bytes = 4 bytes (even),
    // or 1 data byte + 2 CRC bytes = 3 bytes (odd, last segment).
    if (numData > firstData) {
        uint16_t crcCommon = computeCrcCommon(startCrc, connId, seqNo,
                                               command);
        int numExtra = (numData - firstData + 1) / 2;
        int dataPos = firstData;

        for (int i = 1; i <= numExtra; i++) {
            uint8_t d0 = data[dataPos];
            uint8_t d1 = (dataPos + 1 < numData) ? data[dataPos + 1] : 0;
            crcs[numCrcs++] = computeCrcI(crcCommon,
                                           static_cast<uint16_t>(i),
                                           d0, d1);
            dataPos += 2;
        }
    }

    return crcs;
}

// ---------------------------------------------------------------------------
// Frame layout constants and helpers (unchanged from original)
// ---------------------------------------------------------------------------
//
// PDU byte layout (reconstructed from CalcCrc field accesses):
//
//   size = 6 (1 data byte, 1 CRC):
//     [Command, Data0, CRC0Lo, CRC0Hi, ConnIDLo, ConnIDHi]
//
//   size = 7..10 (2 data bytes, 1 CRC):
//     [Command, Data0, Data1, CRC0Lo, CRC0Hi, ConnIDLo, ConnIDHi]
//
//   size > 10 (multi-CRC, 2+2k data bytes):
//     [Command, Data0, Data1, CRC0(2), Data2, Data3, CRC1(2),
//      Data4, Data5, CRC2(2), ..., ConnIDLo, ConnIDHi]
//
// ConnID is always at the last 2 bytes.
// CRC0 is at offset 2 (size <= 6) or offset 3 (size > 6).
// Each additional segment is 4 bytes: 2 data + 2 CRC.
//
// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/

inline constexpr size_t MIN_FSOE_FRAME_SIZE = 3;

// Maximum payload bytes that parseFSoEFrame will write to out_data.
// Accommodates 16 bytes of safe data + 2 bytes error code (fail-safe response).
inline constexpr size_t MAX_PARSE_DATA_SIZE = 18;

/// Compute the total frame size (including CMD, CRCs, and ConnID) for a
/// given data length.
inline constexpr size_t fsoeFrameSize(size_t data_len) {
    if (data_len == 0) return MIN_FSOE_FRAME_SIZE;  // CMD(1) + ConnID(2)
    size_t full_chunks = data_len / 2;
    size_t has_odd = data_len % 2;
    return 1 + full_chunks * 4 + (has_odd ? 3 : 0) + 2;
}

/// Compute the data length from the frame size.
inline constexpr size_t fsoeDataLen(size_t frame_size) {
    if (frame_size < MIN_FSOE_FRAME_SIZE) return 0;
    if (frame_size == MIN_FSOE_FRAME_SIZE) return 0;  // CMD + ConnID only
    size_t remaining = frame_size - 1 - 2;  // subtract CMD and ConnID
    size_t full_chunks = remaining / 4;
    size_t remainder = remaining % 4;
    // remainder 0 = all full chunks (even data)
    // remainder 3 = last chunk is 1 data byte + 2 CRC (odd data)
    // remainder 1 or 2 = invalid frame
    if (remainder != 0 && remainder != 3) return 0;
    return full_chunks * 2 + (remainder == 3 ? 1 : 0);
}

/// Compute the PDU size from the data length.
/// PDU size = frame size (CMD + data + CRCs + ConnID).
inline constexpr size_t fsoePduSize(size_t data_len) {
    return fsoeFrameSize(data_len);
}

/// Extract data fields from an FSoE frame WITHOUT CRC verification.
///
/// This is used when CRC has already been verified (e.g., by validateFrame)
/// and the caller only needs to extract the command, data, and ConnID.
/// It does NOT update any CRC state and does NOT verify CRCs.
///
/// @param frame      Input frame bytes.
/// @param frame_len  Frame length in bytes.
/// @param out_cmd    Receives the command byte.
/// @param out_data   Receives the data bytes (empty span to skip extraction).
/// @param out_data_len  Receives the data length.
/// @param out_conn_id   Receives the connection ID.
/// @return true if the frame format is valid (CRCs are NOT checked).
inline bool extractFSoEFrame(const uint8_t* frame, size_t frame_len,
                              uint8_t& out_cmd,
                              std::span<uint8_t> out_data, size_t& out_data_len,
                              uint16_t& out_conn_id) {
    if (frame_len < MIN_FSOE_FRAME_SIZE) return false;

    out_cmd = frame[0];

    if (frame_len == MIN_FSOE_FRAME_SIZE) {
        out_data_len = 0;
        out_conn_id = static_cast<uint16_t>(frame[1]) |
                      (static_cast<uint16_t>(frame[2]) << 8);
        return true;
    }

    size_t data_len = fsoeDataLen(frame_len);
    if (data_len == 0) return false;
    if (data_len > MAX_PARSE_DATA_SIZE) return false;

    int pduSize = static_cast<int>(frame_len);

    if (!out_data.empty()) {
        // Bounds check: refuse to write more than the caller's buffer can hold.
        if (data_len > out_data.size()) return false;
        if (pduSize <= 6) {
            out_data[0] = frame[1];
        } else {
            size_t offset = 1;
            out_data[0] = frame[offset];
            out_data[1] = frame[offset + 1];
            offset += 4;
            int numExtra = (static_cast<int>(data_len) - 2 + 1) / 2;
            for (int i = 1; i <= numExtra; i++) {
                size_t dataPos = 2 + (i - 1) * 2;
                out_data[dataPos] = frame[offset];
                if (dataPos + 1 < data_len) {
                    out_data[dataPos + 1] = frame[offset + 1];
                    offset += 4;
                } else {
                    offset += 3;
                }
            }
        }
    }
    out_data_len = data_len;

    out_conn_id = static_cast<uint16_t>(frame[frame_len - 2]) |
                  (static_cast<uint16_t>(frame[frame_len - 1]) << 8);

    return true;
}

// ---------------------------------------------------------------------------
// Frame building
// ---------------------------------------------------------------------------

/// Build an FSoE frame with correct CRCs.
///
/// The frame layout is:
///   [CMD] [Data0] [Data1] [CRC0(2)] [Data2] [Data3] [CRC1(2)] ... [ConnID(2)]
///
/// For 0 data bytes: [CMD] [ConnID(2)]  (no CRC — Reset frame)
/// For 1 data byte:  [CMD] [Data0] [CRC0(2)] [ConnID(2)]
/// For 2+ data bytes: [CMD] [Data0] [Data1] [CRC0(2)] ... [ConnID(2)]
///
/// @param out        Output buffer (must be at least fsoeFrameSize(data_len) bytes)
/// @param cmd        Command byte
/// @param data       Safe data bytes (may be nullptr if data_len == 0)
/// @param data_len   Number of data bytes
/// @param conn_id    Connection ID
/// @param start_crc  Previous frame's CRC0 (CRC inheritance).  Use 0 for the
///                   very first frame.
/// @param seq_no     Sequence number (shared between master and slave).
/// @param out_crc0   If non-null, receives the CRC0 value (for CRC inheritance:
///                   pass as start_crc to the next frame).
/// @return Frame size in bytes, or 0 on error.
inline size_t buildFSoEFrame(uint8_t* out, uint8_t cmd,
                              const uint8_t* data, size_t data_len,
                              uint16_t conn_id,
                              uint16_t start_crc = 0,
                              uint16_t seq_no = 0,
                              uint16_t* out_crc0 = nullptr) {
    if (data_len == 0) {
        // Reset frame: CMD(1) + ConnID(2) = 3 bytes, no CRC
        if (out_crc0) *out_crc0 = 0;
        out[0] = cmd;
        out[1] = conn_id & 0xFF;
        out[2] = (conn_id >> 8) & 0xFF;
        return MIN_FSOE_FRAME_SIZE;
    }

    size_t frame_size = fsoeFrameSize(data_len);
    int pduSize = static_cast<int>(frame_size);

    // Compute all CRCs for this PDU
    auto crcs = computeAllCrcs(start_crc, conn_id, seq_no, cmd,
                               data, static_cast<int>(data_len), pduSize);

    if (out_crc0) *out_crc0 = crcs[0];

    // Build the frame
    out[0] = cmd;

    if (pduSize <= 6) {
        // 1 data byte: [CMD] [Data0] [CRC0(2)] [ConnID(2)]
        out[1] = data[0];
        out[2] = crcs[0] & 0xFF;
        out[3] = (crcs[0] >> 8) & 0xFF;
        out[4] = conn_id & 0xFF;
        out[5] = (conn_id >> 8) & 0xFF;
    } else {
        // 2+ data bytes: [CMD] [Data0] [Data1] [CRC0(2)] [Data2] [Data3] [CRC1(2)] ... [ConnID(2)]
        size_t offset = 1;
        // First segment: 2 data bytes + CRC0
        out[offset]     = data[0];
        out[offset + 1] = data[1];
        out[offset + 2] = crcs[0] & 0xFF;
        out[offset + 3] = (crcs[0] >> 8) & 0xFF;
        offset += 4;

        // Additional segments: 2 data bytes + CRCi (or 1 data byte + CRCi for odd)
        int numExtra = (static_cast<int>(data_len) - 2 + 1) / 2;
        for (int i = 1; i <= numExtra; i++) {
            size_t dataPos = 2 + (i - 1) * 2;
            if (dataPos + 1 < data_len) {
                // Even segment: 2 data bytes + 2 CRC bytes
                out[offset]     = data[dataPos];
                out[offset + 1] = data[dataPos + 1];
                out[offset + 2] = crcs[i] & 0xFF;
                out[offset + 3] = (crcs[i] >> 8) & 0xFF;
                offset += 4;
            } else {
                // Odd segment: 1 data byte + 2 CRC bytes = 3 bytes
                out[offset]     = data[dataPos];
                out[offset + 1] = crcs[i] & 0xFF;
                out[offset + 2] = (crcs[i] >> 8) & 0xFF;
                offset += 3;
            }
        }

        // ConnID at the end
        out[offset]     = conn_id & 0xFF;
        out[offset + 1] = (conn_id >> 8) & 0xFF;
    }

    return frame_size;
}

// ---------------------------------------------------------------------------
// Frame parsing and CRC verification
// ---------------------------------------------------------------------------

/// Parse an FSoE frame and verify all CRCs.
///
/// @param frame      Input frame bytes.
/// @param frame_len  Frame length in bytes.
/// @param out_cmd    Receives the command byte.
/// @param out_data   Receives the data bytes (empty span to skip extraction).
/// @param out_data_len  Receives the data length.
/// @param out_conn_id   Receives the connection ID.
/// @param start_crc  Previous frame's CRC0 (for CRC inheritance).  Use 0 for
///                   the very first frame.
/// @param seq_no     Expected sequence number (shared between master and slave).
/// @param out_crc0   If non-null, receives the frame's CRC0 (for CRC inheritance:
///                   pass as start_crc when parsing the next frame).
/// @param out_crc_error  If non-null, receives diagnostic detail (expected vs
///                       received CRC, failing segment index) when CRC
///                       verification fails.  Not populated for non-CRC
///                       failures (e.g. too-short frame).
/// @return true if all CRCs verify, false otherwise.
inline bool parseFSoEFrame(const uint8_t* frame, size_t frame_len,
                            uint8_t& out_cmd,
                            std::span<uint8_t> out_data, size_t& out_data_len,
                            uint16_t& out_conn_id,
                            uint16_t start_crc = 0,
                            uint16_t seq_no = 0,
                            uint16_t* out_crc0 = nullptr,
                            CrcErrorDetail* out_crc_error = nullptr) {
    if (frame_len < MIN_FSOE_FRAME_SIZE) return false;

    out_cmd = frame[0];

    // Reset frame: CMD(1) + ConnID(2) = 3 bytes, no CRC to verify
    if (frame_len == MIN_FSOE_FRAME_SIZE) {
        out_data_len = 0;
        out_conn_id = static_cast<uint16_t>(frame[1]) |
                      (static_cast<uint16_t>(frame[2]) << 8);
        if (out_crc0) *out_crc0 = 0;
        return true;
    }

    size_t data_len = fsoeDataLen(frame_len);
    if (data_len == 0 && frame_len != MIN_FSOE_FRAME_SIZE) return false;

    // Safety check: reject frames with payload exceeding the maximum
    // supported size.
    if (data_len > MAX_PARSE_DATA_SIZE) return false;

    int pduSize = static_cast<int>(frame_len);

    // Extract data bytes from the frame
    if (!out_data.empty()) {
        // Bounds check: refuse to write more than the caller's buffer can hold.
        if (data_len > out_data.size()) return false;
        if (pduSize <= 6) {
            // 1 data byte: [CMD] [Data0] [CRC0(2)] [ConnID(2)]
            out_data[0] = frame[1];
        } else {
            // 2+ data bytes
            size_t offset = 1;
            // First segment: 2 data bytes
            out_data[0] = frame[offset];
            out_data[1] = frame[offset + 1];
            offset += 4;  // skip CRC0

            // Additional segments: 2 data bytes (even) or 1 (odd, last)
            int numExtra = (static_cast<int>(data_len) - 2 + 1) / 2;
            for (int i = 1; i <= numExtra; i++) {
                size_t dataPos = 2 + (i - 1) * 2;
                out_data[dataPos] = frame[offset];
                if (dataPos + 1 < data_len) {
                    out_data[dataPos + 1] = frame[offset + 1];
                    offset += 4;  // 2 data + 2 CRC
                } else {
                    // Odd last segment: 1 data + 2 CRC = 3 bytes
                    offset += 3;
                }
            }
        }
    }
    out_data_len = data_len;

    // Extract ConnID from the last 2 bytes
    out_conn_id = static_cast<uint16_t>(frame[frame_len - 2]) |
                  (static_cast<uint16_t>(frame[frame_len - 1]) << 8);

    // Compute expected CRCs
    const uint8_t* data_ptr = out_data.data();
    // If out_data is empty, we need a temporary buffer to extract data for CRC
    uint8_t temp_data[MAX_PARSE_DATA_SIZE] = {0};
    if (out_data.empty()) {
        if (pduSize <= 6) {
            temp_data[0] = frame[1];
        } else {
            size_t offset = 1;
            temp_data[0] = frame[offset];
            temp_data[1] = frame[offset + 1];
            offset += 4;
            int numExtra = (static_cast<int>(data_len) - 2 + 1) / 2;
            for (int i = 1; i <= numExtra; i++) {
                size_t dataPos = 2 + (i - 1) * 2;
                temp_data[dataPos] = frame[offset];
                if (dataPos + 1 < data_len) {
                    temp_data[dataPos + 1] = frame[offset + 1];
                    offset += 4;
                } else {
                    offset += 3;
                }
            }
        }
        data_ptr = temp_data;
    }

    auto expected_crcs = computeAllCrcs(start_crc, out_conn_id, seq_no,
                                        out_cmd, data_ptr,
                                        static_cast<int>(data_len), pduSize);

    if (out_crc0) *out_crc0 = expected_crcs[0];

    // Verify each CRC
    if (pduSize <= 6) {
        // 1 data byte: [CMD] [Data0] [CRC0(2)] [ConnID(2)]
        uint16_t stored_crc0 = static_cast<uint16_t>(frame[2]) |
                               (static_cast<uint16_t>(frame[3]) << 8);
        if (stored_crc0 != expected_crcs[0]) {
            if (out_crc_error) {
                out_crc_error->valid = true;
                out_crc_error->segment_index = 0;
                out_crc_error->expected_crc = expected_crcs[0];
                out_crc_error->received_crc = stored_crc0;
                out_crc_error->frame_offset = 2;
            }
            return false;
        }
    } else {
        // 2+ data bytes: verify CRC0 and all subsequent CRCi.
        // Frame layout: [CMD] [D0] [D1] [CRC0(2)] [D2] [D3] [CRC1(2)] ... [ConnID(2)]
        // For odd data:  [CMD] [D0] [D1] [CRC0(2)] [D2] [CRC1(2)] [ConnID(2)]
        // CRC0 is at offset 3 (after CMD + 2 data bytes).
        // After CRC0, each segment is: [data(1-2)] [CRCi(2)]
        size_t offset = 3;  // CRC0 starts here
        int numCrcs = 1;
        int numExtra = (static_cast<int>(data_len) - 2 + 1) / 2;
        numCrcs += numExtra;

        for (int i = 0; i < numCrcs; i++) {
            uint16_t stored_crc = static_cast<uint16_t>(frame[offset]) |
                                  (static_cast<uint16_t>(frame[offset + 1]) << 8);
            if (stored_crc != expected_crcs[i]) {
                if (out_crc_error) {
                    out_crc_error->valid = true;
                    out_crc_error->segment_index = i;
                    out_crc_error->expected_crc = expected_crcs[i];
                    out_crc_error->received_crc = stored_crc;
                    out_crc_error->frame_offset = offset;
                }
                return false;
            }
            offset += 2;  // skip this CRC's 2 bytes
            // Skip the next data segment (if not the last CRC)
            if (i < numCrcs - 1) {
                size_t dataPos = 2 + i * 2;
                if (dataPos + 1 < data_len) {
                    offset += 2;  // 2 data bytes (even segment)
                } else {
                    offset += 1;  // 1 data byte (odd segment)
                }
            }
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// FSoEFrameScanner — offline frame detection in arbitrary binary data
// ---------------------------------------------------------------------------
//
// ============================================================================
// SAFETY WARNING — READ BEFORE USE
// ============================================================================
//
// The functions in this class perform an EXHAUSTIVE, UNAUTHENTICATED search:
// they try many offsets/lengths and accept whatever happens to pass CRC
// verification.  A random or adversarial blob of data has a non-negligible
// chance of containing a byte sequence that verifies (each CRC check has a
// ~2^-16 false-positive rate, multiplied by every candidate tried).
//
// Therefore, to preserve the safety guarantees of FSoE:
//
//   *** These functions MUST NOT be used dynamically on received data. ***
//   *** They may only be used ONCE per buffer (e.g. a one-time scan of   ***
//   *** a captured dump or commissioning snapshot), never in the cyclic ***
//   *** RX path and never to "resynchronize" a live connection.          ***
//
// Accepting a spuriously-verifying "frame" found inside otherwise likely
// data would break the FSoE integrity guarantees — the protocol relies on
// each frame being validated exactly once at its expected position.  Using
// this scanner on live RX data turns a probabilistic search into a frame
// acceptance mechanism, which is precisely what the CRC chain is designed
// to prevent.  The cyclic receive path must always use parseFSoEFrame()
// with the configured, fixed frame length.
// ============================================================================

/// Result of a frame scan: location and length of a verified FSoE frame
/// inside a larger buffer.
struct FSoEFrameMatch {
    size_t offset = 0;     ///< Byte offset of the frame's CMD byte
    size_t length = 0;     ///< Verified frame length in bytes
    uint16_t conn_id = 0;  ///< Connection ID parsed from the frame
    uint8_t command = 0;   ///< Command byte of the frame
    bool found = false;    ///< true if a frame was found
};

/// Offline scanner that locates CRC-verified FSoE frames in binary data.
///
/// The FSoE PDU carries no length field — its size is implicitly agreed via
/// the configured safe-data length.  This class recovers the position and
/// length of a frame when they are not known a priori.
///
/// OFFLINE / ONE-SHOT USE ONLY — see the safety warning above.  Do NOT use
/// on live received data.
///
/// Usage:
///   FSoEFrameScanner scanner({.expected_conn_id = 0x1234});
///   auto m = scanner.findFrame(dump, dump_len);          // first match
///   auto n = scanner.findFrame(dump, dump_len, m.offset + 1);  // next
class FSoEFrameScanner {
public:
    /// Scanner configuration (CRC-inheritance inputs and match filters).
    struct Config {
        uint16_t start_crc = 0;  ///< Previous frame's CRC0.  0 if unknown /
                                 ///< first frame — frames chained from a
                                 ///< predecessor only verify with the
                                 ///< correct start_crc.
        uint16_t seq_no = 0;     ///< Expected sequence number.
        int expected_conn_id = -1;  ///< If >= 0, only frames whose ConnID
                                    ///< equals this value are accepted
                                    ///< (strongly recommended — it is the
                                    ///< main disambiguator against spurious
                                    ///< CRC hits).
        /// Only accept verified frames at least this long.  Defaults to
        /// MIN_FSOE_FRAME_SIZE+1 so the CRC-less 3-byte Reset frame — which
        /// matches vacuously at every offset — is excluded.  Set to
        /// MIN_FSOE_FRAME_SIZE (with expected_conn_id) to also detect
        /// Reset frames.
        size_t min_frame_len = MIN_FSOE_FRAME_SIZE + 1;
        /// If non-zero, only this exact frame length is tried at each
        /// offset (fast path when the frame size is known).
        size_t expected_len = 0;
    };

    constexpr FSoEFrameScanner() = default;
    explicit constexpr FSoEFrameScanner(Config cfg) : cfg_(cfg) {}

    /// Find the length for which an FSoE frame's CRCs verify.
    ///
    /// Scans candidate frame sizes and returns the longest one whose CRCs
    /// verify.  Candidates are tried from max_len downwards; only
    /// structurally valid sizes are tested (see fsoeDataLen: 3-byte Reset
    /// frame, then sizes of the form 4k+2 / 4k+3).  Returning the LONGEST
    /// match is deliberate: a 3-byte Reset frame has no CRC and therefore
    /// "verifies" vacuously for any buffer, so it is only returned when no
    /// longer candidate verifies.
    ///
    /// @param frame    Input buffer containing an FSoE frame as a prefix
    ///                 (may have trailing bytes beyond it).
    /// @param max_len  Number of valid bytes in the buffer.  Capped at the
    ///                 largest frame parseFSoEFrame supports
    ///                 (fsoeFrameSize(MAX_PARSE_DATA_SIZE)).
    /// @return The verified frame length in bytes, or 0 if no candidate
    ///         length passes CRC verification (and the configured filters).
    size_t findFrameLength(const uint8_t* frame, size_t max_len) const {
        if (!frame || max_len < MIN_FSOE_FRAME_SIZE) return 0;

        // parseFSoEFrame rejects data_len > MAX_PARSE_DATA_SIZE, so larger
        // candidates can never verify — cap the scan.
        const size_t hard_max = fsoeFrameSize(MAX_PARSE_DATA_SIZE);
        if (max_len > hard_max) max_len = hard_max;

        uint8_t cmd = 0;
        size_t data_len = 0;
        uint16_t conn_id = 0;

        // Try CRC-bearing candidates longest-first.
        for (size_t len = max_len; len > MIN_FSOE_FRAME_SIZE; --len) {
            if (fsoeDataLen(len) == 0) continue;  // structurally invalid
            if (!parseFSoEFrame(frame, len, cmd, {}, data_len, conn_id,
                                cfg_.start_crc, cfg_.seq_no))
                continue;
            if (!connIdMatches(conn_id)) continue;
            return len;
        }

        // Last resort: the 3-byte Reset frame (CMD + ConnID, no CRC).
        if (parseFSoEFrame(frame, MIN_FSOE_FRAME_SIZE, cmd, {}, data_len,
                           conn_id, cfg_.start_crc, cfg_.seq_no) &&
            connIdMatches(conn_id)) {
            return MIN_FSOE_FRAME_SIZE;
        }
        return 0;
    }

    /// Scan a binary buffer for the first FSoE frame whose CRCs verify.
    ///
    /// Slides a window over the buffer and applies findFrameLength() at
    /// every offset.
    ///
    /// @param data          Buffer to scan.
    /// @param data_len      Number of valid bytes in the buffer.
    /// @param start_offset  Byte offset to begin scanning at (default 0).
    ///                      To find subsequent frames, call again with
    ///                      match.offset + 1.
    /// @return FSoEFrameMatch with found=true on success.
    FSoEFrameMatch findFrame(const uint8_t* data, size_t data_len,
                              size_t start_offset = 0) const {
        FSoEFrameMatch match{};
        if (!data || data_len < MIN_FSOE_FRAME_SIZE) return match;
        size_t min_len = cfg_.min_frame_len;
        if (min_len < MIN_FSOE_FRAME_SIZE) min_len = MIN_FSOE_FRAME_SIZE;

        for (size_t off = start_offset;
             off + MIN_FSOE_FRAME_SIZE <= data_len; ++off) {
            const uint8_t* p = data + off;
            const size_t remaining = data_len - off;

            if (cfg_.expected_len != 0) {
                // Fast path: try only the expected length at this offset.
                if (cfg_.expected_len < min_len ||
                    cfg_.expected_len > remaining)
                    continue;
                uint8_t cmd = 0;
                size_t dlen = 0;
                uint16_t conn = 0;
                if (!parseFSoEFrame(p, cfg_.expected_len, cmd, {}, dlen,
                                    conn, cfg_.start_crc, cfg_.seq_no))
                    continue;
                if (!connIdMatches(conn)) continue;
                match.offset = off;
                match.length = cfg_.expected_len;
                match.conn_id = conn;
                match.command = cmd;
                match.found = true;
                return match;
            }

            // General path: find the longest verifying length here.
            size_t len = findFrameLength(p, remaining);
            if (len < min_len) continue;
            // Re-extract cmd/conn for the match record.
            uint8_t cmd = 0;
            size_t dlen = 0;
            uint16_t conn = 0;
            parseFSoEFrame(p, len, cmd, {}, dlen, conn,
                           cfg_.start_crc, cfg_.seq_no);
            match.offset = off;
            match.length = len;
            match.conn_id = conn;
            match.command = cmd;
            match.found = true;
            return match;
        }
        return match;
    }

private:
    bool connIdMatches(uint16_t conn_id) const {
        return cfg_.expected_conn_id < 0 ||
               conn_id == static_cast<uint16_t>(cfg_.expected_conn_id);
    }

    Config cfg_;
};

// ---------------------------------------------------------------------------
// Backward-compatible standard CRC-16 (for parameter CRC computation)
// ---------------------------------------------------------------------------
//
// This is a standard MSB-first CRC-16 with polynomial 0x39B7, init=0, no
// final XOR.  It is NOT the FSoE frame CRC — it is used only for computing
// the parameter CRC (a checksum over the parameter data to verify that
// master and slave have the same parameters).
//
// Uses only T0 (the standard byte-wise table).
inline uint16_t calculate(const uint8_t* data, size_t len,
                           uint16_t init_crc = 0) {
    if (!data || len == 0) return init_crc;
    uint16_t crc = init_crc;
    for (size_t i = 0; i < len; i++) {
        // Standard CRC update: crc = (crc << 8) ^ T0[(crc >> 8) ^ data[i]]
        if constexpr (kUseTable) {
            crc = static_cast<uint16_t>(
                (crc << 8) ^ crcTable0[(crc >> 8) ^ data[i]]);
        } else {
            crc = static_cast<uint16_t>(
                (crc << 8) ^ polyMod(static_cast<uint8_t>((crc >> 8) ^ data[i]), 8));
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// ETG.5100 §8.1.3.4 — Sequence number and CRC collision avoidance
// ---------------------------------------------------------------------------
//
// The FSoE standard requires a virtual 16-bit sequence number that is folded
// into the CRC computation but NOT transmitted in the frame.  Key rules:
//
//   1. The sequence number range is 1..65535.  The value 0 is NEVER used.
//      After 65535 the counter wraps back to 1.
//
//   2. The master maintains a master sequence number (incremented with each
//      new master PDU); the slave maintains a slave sequence number
//      (incremented with each new slave PDU).  They are independent counters.
//
//   3. CRC collision avoidance (do…while loop):
//      - Master side: if CRC0 of the new master PDU equals CRC0 of the
//        previous master PDU, increment the master seq and recompute until
//        they differ.
//      - Slave side: if CRC0 of the new slave PDU equals CRC0 of the
//        previous slave PDU, increment the slave seq and recompute until
//        they differ.
//
//   4. The checker (slave checking master PDUs, master checking slave PDUs)
//      must replicate the same algorithm: try the expected seq, and if the
//      CRC doesn't verify, increment the seq and retry.
//
// See: https://techoverflow.net/2026/08/09/fsoe-how-does-crc-inheritance-work/

/// Increment a sequence number with ETG.5100 wrap-around (1..65535, skip 0).
///
///   0     → 1   (initial value)
///   1     → 2
///   65534 → 65535
///   65535 → 1   (wrap, skip 0)
inline uint16_t incrementSeqNo(uint16_t seq) {
    return (seq == 65535) ? 1 : static_cast<uint16_t>(seq + 1);
}

/// Decrement a sequence number with ETG.5100 wrap-around (1..65535, skip 0).
///
///   0     → 65535 (wrap)
///   1     → 65535 (wrap, skip 0)
///   2     → 1
///   65535 → 65534
inline uint16_t decrementSeqNo(uint16_t seq) {
    return (seq <= 1) ? 65535 : static_cast<uint16_t>(seq - 1);
}

/// Build an FSoE frame with ETG.5100 §8.1.3.4 CRC collision avoidance.
///
/// This is a wrapper around buildFSoEFrame that implements the do…while loop:
/// if the computed CRC0 equals the previous frame's CRC0 (start_crc), the
/// sequence number is incremented (wrapping 65535 → 1, skipping 0) and the
/// CRC is recomputed until they differ.
///
/// @param out            Output buffer.
/// @param cmd            Command byte.
/// @param data           Safe data bytes (may be nullptr if data_len == 0).
/// @param data_len       Number of data bytes.
/// @param conn_id        Connection ID.
/// @param start_crc      Previous frame's CRC0 (CRC inheritance).  Use 0 for
///                       the very first frame.  This is also the value used
///                       for collision detection.
/// @param initial_seq    Initial sequence number to try (must be 1..65535).
/// @param out_crc0       If non-null, receives the frame's CRC0.
/// @param out_seq_used   If non-null, receives the sequence number that was
///                       actually used (after collision avoidance).  The
///                       caller should update its counter to this value.
/// @return Frame size in bytes, or 0 on error.
inline size_t buildFSoEFrameWithCollisionAvoidance(
    uint8_t* out, uint8_t cmd,
    const uint8_t* data, size_t data_len,
    uint16_t conn_id,
    uint16_t start_crc,
    uint16_t initial_seq,
    uint16_t* out_crc0 = nullptr,
    uint16_t* out_seq_used = nullptr)
{
    // Reset frames (data_len == 0) have no CRC, so collision avoidance
    // doesn't apply.  Just build the frame with the given seq.
    if (data_len == 0) {
        if (out_seq_used) *out_seq_used = initial_seq;
        return buildFSoEFrame(out, cmd, data, data_len, conn_id,
                              start_crc, initial_seq, out_crc0);
    }

    uint16_t seq = initial_seq;
    uint16_t crc0 = 0;
    size_t frame_size = 0;

    // do…while loop per ETG.5100 §8.1.3.4:
    //   Compute CRC0 with the current seq.
    //   If CRC0 == start_crc (previous frame's CRC0), increment seq and retry.
    //   Continue until CRC0 != start_crc.
    //
    // The loop is guaranteed to terminate: with 65535 possible seq values
    // and only 65536 possible CRC0 values, at most 65535 iterations are
    // needed (and in practice, collisions are extremely rare).
    do {
        frame_size = buildFSoEFrame(out, cmd, data, data_len, conn_id,
                                    start_crc, seq, &crc0);
        if (frame_size == 0) {
            // Build failed — shouldn't happen for valid inputs
            if (out_seq_used) *out_seq_used = seq;
            return 0;
        }
        if (crc0 != start_crc) {
            // No collision — accept this frame
            break;
        }
        // CRC collision: increment seq and retry
        seq = incrementSeqNo(seq);
    } while (true);

    if (out_crc0) *out_crc0 = crc0;
    if (out_seq_used) *out_seq_used = seq;
    return frame_size;
}

/// Parse an FSoE frame with ETG.5100 §8.1.3.4 CRC collision avoidance.
///
/// This is a wrapper around parseFSoEFrame that replicates the collision
/// avoidance algorithm on the checking side.  The algorithm is:
///
///   1. Try the expected seq → compute CRC.
///   2. If CRC matches:
///      a. If CRC0 != start_crc (no collision) → accept.
///      b. If CRC0 == start_crc (collision) → increment seq, go to 1.
///   3. If CRC doesn't match → FAIL (genuine CRC error).
///
/// The key insight: the checker only tries incrementing the seq when the
/// CRC *matches* but there's a collision.  If the CRC doesn't match at all,
/// it's a genuine error — the frame is corrupted or the wrong seq was used.
/// This prevents false positives where a corrupted frame happens to match
/// a different seq value by chance.
///
/// @param frame          Input frame bytes.
/// @param frame_len      Frame length in bytes.
/// @param out_cmd        Receives the command byte.
/// @param out_data       Receives the data bytes (empty span to skip).
/// @param out_data_len   Receives the data length.
/// @param out_conn_id    Receives the connection ID.
/// @param start_crc      Previous frame's CRC0 (for CRC inheritance).
/// @param initial_seq    Expected sequence number (must be 1..65535).
/// @param out_crc0       If non-null, receives the frame's CRC0.
/// @param out_seq_used   If non-null, receives the seq that matched.
/// @param out_crc_error  If non-null, receives diagnostic detail on failure.
/// @param max_attempts   Maximum number of collision retries (default 65535).
/// @return true if all CRCs verify and no collision, false otherwise.
inline bool parseFSoEFrameWithCollisionAvoidance(
    const uint8_t* frame, size_t frame_len,
    uint8_t& out_cmd,
    std::span<uint8_t> out_data, size_t& out_data_len,
    uint16_t& out_conn_id,
    uint16_t start_crc,
    uint16_t initial_seq,
    uint16_t* out_crc0 = nullptr,
    uint16_t* out_seq_used = nullptr,
    CrcErrorDetail* out_crc_error = nullptr,
    uint32_t max_attempts = 65535)
{
    // Reset frames (data_len == 0) have no CRC, so collision avoidance
    // doesn't apply.  Just parse the frame with the given seq.
    if (frame_len == MIN_FSOE_FRAME_SIZE) {
        if (out_seq_used) *out_seq_used = initial_seq;
        return parseFSoEFrame(frame, frame_len, out_cmd,
                              out_data, out_data_len, out_conn_id,
                              start_crc, initial_seq,
                              out_crc0, out_crc_error);
    }

    uint16_t seq = initial_seq;
    CrcErrorDetail crc_error{};

    for (uint32_t attempt = 0; attempt < max_attempts; ++attempt) {
        uint16_t crc0 = 0;
        CrcErrorDetail err{};
        if (parseFSoEFrame(frame, frame_len, out_cmd,
                           out_data, out_data_len, out_conn_id,
                           start_crc, seq, &crc0, &err)) {
            // CRC verified.  Now check for collision: the generator would
            // have incremented seq if CRC0 == start_crc.  If we see a frame
            // where CRC0 == start_crc, the generator should have used a
            // different seq — replicate the collision avoidance.
            if (crc0 == start_crc) {
                // Collision — increment seq and retry
                seq = incrementSeqNo(seq);
                continue;
            }
            // Success: CRC verified and no collision
            if (out_crc0) *out_crc0 = crc0;
            if (out_seq_used) *out_seq_used = seq;
            return true;
        }
        // CRC didn't match with this seq.  Per the standard, the checker
        // only increments seq on collision (CRC matches but equals previous).
        // A CRC mismatch is a genuine error — don't try other seq values.
        if (err.valid && !crc_error.valid) {
            crc_error = err;
        }
        break;
    }

    // CRC verification failed — report the error
    if (out_crc_error) *out_crc_error = crc_error;
    if (out_seq_used) *out_seq_used = initial_seq;  // unchanged on failure
    return false;
}

// ---------------------------------------------------------------------------
// Native CRC-chain resynchronization
// ---------------------------------------------------------------------------

/// Solve a 16-bit affine CRC system: find x such that f(x) == target,
/// where f(x) = eval(x) is affine over GF(2) (f(x) = M·x ⊕ f(0)).
/// Returns true and sets out_x when a (unique or minimum-norm) solution
/// exists; false if the system is inconsistent.
namespace detail {
inline bool solveCrcAffine16(auto&& eval, uint16_t target, uint16_t& out_x) {
    // rows[e] = 16-bit coefficient mask for output bit e; rhs[e] = RHS bit.
    uint16_t rows[16] = {};
    uint8_t  rhs[16]  = {};
    const uint16_t base = eval(0);
    const uint16_t r = static_cast<uint16_t>(target ^ base);
    for (int b = 0; b < 16; ++b) {
        rhs[b] = static_cast<uint8_t>((r >> b) & 1);
        uint16_t coeff = 0;
        for (int j = 0; j < 16; ++j) {
            if ((static_cast<uint16_t>(eval(static_cast<uint16_t>(1u << j))
                     ^ base) >> b) & 1)
                coeff |= static_cast<uint16_t>(1u << j);
        }
        rows[b] = coeff;
    }
    int pivot_col[16];
    for (int v = 0; v < 16; ++v) pivot_col[v] = -1;
    int r_sel = 0;
    for (int v = 0; v < 16 && r_sel < 16; ++v) {
        int p = -1;
        for (int e = r_sel; e < 16; ++e) {
            if ((rows[e] >> v) & 1) { p = e; break; }
        }
        if (p < 0) continue;
        std::swap(rows[p], rows[r_sel]);
        std::swap(rhs[p], rhs[r_sel]);
        for (int e = 0; e < 16; ++e) {
            if (e != r_sel && ((rows[e] >> v) & 1)) {
                rows[e] ^= rows[r_sel];
                rhs[e] ^= rhs[r_sel];
            }
        }
        pivot_col[v] = r_sel;
        ++r_sel;
    }
    for (int e = 0; e < 16; ++e) {
        if (rows[e] == 0 && rhs[e] != 0) return false;
    }
    uint16_t x = 0;
    for (int v = 0; v < 16; ++v) {
        if (pivot_col[v] >= 0 && rhs[pivot_col[v]])
            x |= static_cast<uint16_t>(1u << v);
    }
    out_x = x;
    return true;
}
} // namespace detail

/// Recover the CRC-chain seed a frame was generated with, directly from
/// the frame's own stored CRCs — "native" resynchronization.
///
/// Every stored CRC is an affine function of the shared 16-bit
/// `crc_common` state (CRC16 over startCrc ‖ connId ‖ seqNo ‖ cmd).  The
/// individual (startCrc, seqNo) pair is *not* identifiable — all seeds
/// producing the same crc_common generate identical frames — so the solve
/// works in two steps:
///
///   1. Invert the trailing data-byte updates on stored CRC0 to recover
///      crc_common exactly (16 unknowns, GF(2) affine solve — always
///      solvable because byte-append steps are invertible).
///   2. Verify every additional stored segment CRC against
///      computeCrcI(crc_common, …).  A frame with N stored CRCs yields
///      N−1 independent 16-bit verifications — genuine corruption
///      detection for N ≥ 2 (data_len ≥ 3).  For N = 1 (data_len ≤ 2)
///      there is no independent check and any consistent frame is
///      accepted — structural validity only.
///   3. Recover an *equivalent* sequence number: the seqNo such that
///      computeCrcCommon(0, connId, seqNo, cmd) == crc_common (another
///      invertible affine solve, startCrc fixed to 0).  The returned
///      (0, seqNo) seed is functionally identical to the master's real
///      seed for this frame, but is a representative — it is not
///      guaranteed to equal the master's actual seqNo.  Callers should
///      therefore re-resync each frame while the mode is active rather
///      than assuming the recovered seq tracks the master's counter.
///
/// Typical use: a slave that joins a running connection mid-stream (or
/// missed a state-transition frame and lost the inherited CRC chain) can
/// verify-and-accept the next valid frame and continue.
///
/// @param frame         Input frame bytes (must contain ≥ 1 CRC segment).
/// @param frame_len     Frame length in bytes.
/// @param out_start_crc Receives the equivalent seed's startCrc (always 0).
/// @param out_seq_no    Receives the equivalent sequence number.
/// @return true if the frame verifies under the recovered crc_common.
inline bool resyncSolveSeed(const uint8_t* frame, size_t frame_len,
                            uint16_t& out_start_crc, uint16_t& out_seq_no) {
    uint8_t cmd = 0;
    uint16_t conn_id = 0;
    size_t data_len = 0;
    uint8_t data[MAX_PARSE_DATA_SIZE] = {};
    if (!extractFSoEFrame(frame, frame_len, cmd,
                          std::span<uint8_t>(data, sizeof(data)),
                          data_len, conn_id)) {
        return false;
    }
    if (data_len == 0) {
        // Reset frame: no CRCs to solve — nothing to resync.
        return false;
    }

    const int pduSize = static_cast<int>(frame_len);
    const int firstData = (pduSize > 6) ? 2 : 1;
    const int numExtra =
        (static_cast<int>(data_len) - firstData + 1) / 2;

    // Step 1 — recover crc_common by inverting the data-byte updates
    // appended to it for CRC0:  crc0 = updateCrc*(crc_common, data[0..]).
    const uint16_t stored_crc0 = (pduSize <= 6)
        ? static_cast<uint16_t>(frame[2]) |
              (static_cast<uint16_t>(frame[3]) << 8)
        : static_cast<uint16_t>(frame[3]) |
              (static_cast<uint16_t>(frame[4]) << 8);
    auto crc0_of_common = [&](uint16_t common) {
        uint16_t crc = common;
        for (int i = 0; i < firstData; ++i)
            crc = updateCrc(crc, data[i]);
        return crc;
    };
    uint16_t crc_common = 0;
    if (!detail::solveCrcAffine16(crc0_of_common, stored_crc0, crc_common))
        return false;

    // Step 2 — verify all additional segment CRCs against the recovered
    // crc_common.  Each is an independent 16-bit check on the frame.
    size_t crc_off = (pduSize <= 6) ? 0 : 7;  // first segment CRC offset
    for (int i = 1; i <= numExtra; ++i) {
        const uint16_t stored_i =
            static_cast<uint16_t>(frame[crc_off]) |
            (static_cast<uint16_t>(frame[crc_off + 1]) << 8);
        const int pos = firstData + (i - 1) * 2;
        const uint8_t d0 = data[pos];
        const uint8_t d1 =
            (pos + 1 < static_cast<int>(data_len)) ? data[pos + 1] : 0;
        if (stored_i != computeCrcI(crc_common,
                                    static_cast<uint16_t>(i), d0, d1))
            return false;
        crc_off += (pos + 1 < static_cast<int>(data_len)) ? 4 : 3;
    }

    // Step 3 — recover an equivalent seqNo: solve
    //   computeCrcCommon(0, conn_id, seq, cmd) == crc_common
    // for seq (startCrc = 0).  Invertible because seq enters the CRC as a
    // full 16-bit word (multiplication by x^k mod P — a bijection).
    auto common_of_seq = [&](uint16_t seq) {
        return computeCrcCommon(0, conn_id, seq, cmd);
    };
    uint16_t seq = 0;
    if (!detail::solveCrcAffine16(common_of_seq, crc_common, seq))
        return false;

    out_start_crc = 0;
    out_seq_no = seq;
    return true;
}

/// Parse an FSoE frame in CRC-resync mode.
///
/// Recovers the frame's crc_common from its stored CRCs and verifies all
/// additional segment CRCs against it (resyncSolveSeed), then runs the
/// full parseFSoEFrame() verification with the equivalent (0, seqNo)
/// seed.  Succeeds only when every stored CRC is reproduced — frames
/// with ≥ 2 CRC segments (data_len ≥ 3) get genuine independent
/// verification; shorter frames are accepted on structural validity
/// alone (documented in resyncSolveSeed).
///
/// @param out_start_crc  If non-null, receives the equivalent startCrc
///                       (always 0 — see resyncSolveSeed).
/// @param out_seq_used   If non-null, receives the equivalent seqNo.
/// @return true if the seed was solved and the frame verifies with it.
inline bool parseFSoEFrameResync(
    const uint8_t* frame, size_t frame_len,
    uint8_t& out_cmd,
    std::span<uint8_t> out_data, size_t& out_data_len,
    uint16_t& out_conn_id,
    uint16_t* out_crc0 = nullptr,
    uint16_t* out_start_crc = nullptr,
    uint16_t* out_seq_used = nullptr,
    CrcErrorDetail* out_crc_error = nullptr) {
    uint16_t start_crc = 0, seq_no = 0;
    if (!resyncSolveSeed(frame, frame_len, start_crc, seq_no)) {
        if (out_crc_error) out_crc_error->valid = false;
        return false;
    }
    if (!parseFSoEFrame(frame, frame_len, out_cmd, out_data, out_data_len,
                        out_conn_id, start_crc, seq_no,
                        out_crc0, out_crc_error)) {
        return false;
    }
    if (out_start_crc) *out_start_crc = start_crc;
    if (out_seq_used) *out_seq_used = seq_no;
    return true;
}

} // namespace FSoE::CRC
