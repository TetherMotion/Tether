# Fix mailbox read polling and counter bugs in esc211 SDO path

**Status: APPROVED by user — proceeding with implementation.**

Fix the `coe_sdo.cpp` poll loops so they actually wait for the slave mailbox response instead of aborting on the first empty read, and correct the mailbox counter range to match the EtherCAT spec (1-7).

## Plan

1. **Remove premature `wkc==0` aborts in poll loops** (`coe_sdo.cpp`)
   - `coe_sdo_upload` init-response loop (~line 261)
   - `coe_sdo_upload` segment-response loop (~line 453)
   - `coe_sdo_download` response loop (~line 635)
   In all three places, `!master.readRegister(...)` followed by `master.lastWkc() == 0` causes an immediate `return false`. Because `readRegister` returns `false` both on timeout *and* on `wkc==0` (empty mailbox), the very first empty poll aborts the transaction. Remove the `lastWkc()==0` early-return branches so the loops fall through to the `sleep/continue` retry path.

2. **Fix mailbox counter range** (`coe_sdo.cpp` + `raw/internal.hpp`)
   - The counter currently wraps `0-15` (`& 0x0Fu`). Per ETG.1000.6 it must be a 3-bit value cycling `1-7`.
   - Update the three `(mbx_cnt + 1u) & 0x0Fu` increments to wrap `1-7`.
   - Update `mbx_type_with_cnt` to mask with `0x07` instead of `0x0F` so bit 7 stays reserved.
