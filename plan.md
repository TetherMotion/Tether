# Tether Controls Framework — Security & Safety Audit

Audit of the Tether controls framework for memory safety, thread safety, and
interfacing issues. Three parallel subagents surveyed (1) EtherCAT master/slave,
(2) motion/control/replanner, (3) HAL/platform/IO/bindings. Each subagent claim
was verified directly against the source; ~60% of the reported "Critical"
findings were false positives (the guards they claimed were missing actually
exist). This document addresses only the verified real issues.

All 14 verified findings have been fixed and verified (full build + 10697/10697
tests passing).

## Rejected Subagent Findings (do not fix)

The following "Critical" findings reported by the subagents were verified as
false positives and do **not** need fixing:

- **SDO upload segmented "buffer overflow"** (`src/ethercat/raw/SDOUpload.cpp:545-548`)
  — guarded by `if (out && outCap > produced)` on the preceding line; the
  `outCap - produced` subtraction cannot underflow.
- **SDO download segmented "buffer overflow"** (`src/ethercat/raw/SDODownload.cpp:783,800-801`)
  — `seg_bytes` is clamped to `<= 7` by the ternary `(remaining < 7) ? remaining : 7`;
  `seg_req.data` is 7 bytes; no overflow.
- **`SlaveEmulator` register `addr + len` integer overflow**
  (`src/ethercat/SlaveEmulator.cpp:424,528`) — `addr` and `len` are `uint16_t`,
  promoted to `int` for the addition; no wraparound at `UINT16_MAX`.
- **G-code parser `snprintf` truncation** (`src/gcode/GCodeParser.cpp:207`)
  — `std::snprintf` always null-terminates within the given size; no
  unterminated string possible.
- **`MotionReplanner` acos domain error / NaN before clamp**
  (`src/replanner/MotionReplanner.cpp:303-307`) — magnitude guard at line 303
  returns early before the division; no race on local stack vectors.
- **`RollingStatistics` first-sample divide by zero**
  (`src/replanner/MotionReplanner.cpp:22-31`) — `count_` is incremented to 1
  *before* the Welford division, so `delta / count_` divides by 1, not 0.
- **`LinuxEthernet` partial `sendto`/`recvfrom`** — raw EtherCAT frames are
  atomic datagrams on `AF_PACKET`; partial sends do not occur.
- **`LinuxEthernet` `SO_BINDTODEVICE` validation** — `strncpy` into
  `m_ifname` is bounded; interface validity is checked via `ioctl(SIOCGIFINDEX)`.
