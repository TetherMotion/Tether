# FSoE native CRC-chain resynchronization

`FSoESlave` can optionally recover from a lost CRC chain instead of rejecting
every subsequent frame. The mode is **opt-in** and intended for emulators and
test setups — not for fail-safe production slaves.

## When it is needed

FSoE CRCs are chained: each frame's CRCs are computed over
`startCrc ‖ connId ‖ seqNo ‖ cmd ‖ data`, where `startCrc` is the previous
(opposite-direction) frame's CRC0 and `seqNo` is a shared counter that is never
transmitted. If the slave joins a running connection mid-stream, or misses the
state-transition frame on which the master reset the chain (the ESC211 does
this), its tracked `(startCrc, seqNo)` no longer matches the master's and every
frame fails CRC verification permanently.

## How it works

`FSoE::CRC::resyncSolveSeed` (`include/tether/fsoe/FSoECRC.hpp`) recovers the
chain state **from the frame itself**:

- Every stored CRC is an affine (GF(2)-linear) function of the shared 16-bit
  `crc_common` state — `CRC16` over `startCrc ‖ connId ‖ seqNo ‖ cmd`.
- Step 1 inverts the data-byte updates appended for CRC0, recovering
  `crc_common` exactly via a 16×16 GF(2) linear solve
  (`detail::solveCrcAffine16`). Byte-append CRC steps are invertible, so this
  always succeeds.
- Step 2 verifies every *additional* stored segment CRC against
  `computeCrcI(crc_common, …)`. A frame with N stored CRCs therefore yields
  N−1 independent 16-bit verifications — real corruption detection for N ≥ 2
  (data_len ≥ 3). For N = 1 (data_len ≤ 2) there is no independent check and a
  structurally valid frame is accepted unconditionally.
- Step 3 recovers an *equivalent* sequence number: the `seqNo` such that
  `computeCrcCommon(0, connId, seqNo, cmd) == crc_common`, another invertible
  affine solve with `startCrc` fixed to 0.

`FSoE::CRC::parseFSoEFrameResync` wraps this with the full `parseFSoEFrame`
verification (command byte, structure, data extraction, conn_id) using the
recovered seed.

### Important caveats

- The `(startCrc, seqNo)` pair is **not identifiable**: all seeds producing the
  same `crc_common` generate identical frames. The returned `(0, seqNo)` is an
  equivalent representative — **not** the master's actual sequence number.
  Callers must therefore treat resync as a per-frame operation while the mode
  is active, not a one-time synchronization.
- For frames with a single CRC segment (data_len ≤ 2) the resynced frame is
  accepted on structural validity alone — there is no independent equation to
  check against.
- Enabling this mode weakens the protocol's desync detection: a master that
  does *not* reset its chain could have a genuinely desynced stream silently
  "repaired". Keep it off unless the master requires it.

## Enabling

Generic slave:

```cpp
FSoE::FSoESlaveConfig cfg;
cfg.crcResyncEnabled = true;   // default false
```

SafeMotion servo emulator:

```cpp
Synapticon::ServoEmulatorConfig cfg;
cfg.crc_resync = true;         // default false
```

`esc211_fsoe_monitor` enables it (together with
`accept_any_connection_id` and `reset_crc_on_state_transition`) because the
emulated channels join the ESC211's broadcast handshake after it has already
started, and the PDO window only carries the latest frame.

## Failure order in `FSoESlave::validateFrame`

1. Normal parse with the inherited chain and expected sequence
   (collision-avoidance variant).
2. If `resetCrcOnStateTransition`: retry with a fresh chain
   (`start_crc = 0`, `seq = initialSeqNo`).
3. If `crcResyncEnabled`: solve the seed from the frame and verify.

A frame is accepted only when one of these fully verifies; otherwise the
existing CRC-error path runs unchanged.

## Tests

`tests/fsoe/test_fsoe_crc_regression.cpp`, suite `FSoECRCResync`:

- equivalent seed solves and the frame verifies under it,
- resync parse extracts cmd/data/conn_id correctly,
- multi-CRC frames: a corrupted trailing segment CRC is rejected,
- structurally invalid frames (bad command) are rejected.
