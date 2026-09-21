# Design decisions — FastLoop / 4 kHz realtime work

All previously open questions have been resolved and implemented.  This
file intentionally no longer lists them; the decisions live in the code
and in git history (see `docs/CyclicRealtimeTransport.md` for the
transport design and `git log` for the implementation commits).

Last two items closed:

* **Q12 — Windows channel**: `futexWaitOn`/`futexWakeAllOn` in
  `ProcessImage.cpp` now map to `WaitOnAddress`/`WakeByAddressAll` under
  `_WIN32` (with a QPC-based `monoNowNs`), and
  `src/hal/WindowsEthernet.cpp` provides an Npcap/wpcap-backed
  `IEthernet` implementation (wired into `cmake/components/hal.cmake`
  for Windows builds).
* **Q16 — per-frame bandwidth**: `Master::Config::max_frame_size`
  (default 1514, clamped [1514, 9014]) enables jumbo-frame links;
  `EtherCATTransport::setMaxFrameSize` raises the per-frame payload
  ceiling, `kMaxDatagramDataSize` was raised to the 11-bit protocol
  limit (2047), the cyclic channel ring slots take
  `CyclicChannelConfig::frame_size`, and the HAL accepts
  `EthernetConfig::maxFrameSize`.  Note: the EtherCAT frame header's
  11-bit length field caps total EtherCAT payload at 2047 B per
  Ethernet frame — going beyond needs multi-EtherCAT-frame packing.
