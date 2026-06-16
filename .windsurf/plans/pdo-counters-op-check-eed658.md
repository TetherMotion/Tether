# Add Per-Slave PDO Request/Reply Counters and Enforce Them for OP Transition

Add per-slave request/reply counters to `PDOManager`, increment them on every successful PDO exchange, and make `Slave::transitionToOp()` fail fast if a slave has mapped PDOs but has not yet seen both a request and a reply.

## Problem

The OP transition can fail silently when no process data has been exchanged. By tracking per-slave request (RxPDO sent) and reply (TxPDO received) counters, the state machine can reject an OP transition early with a clear diagnostic instead of waiting 5 s and timing out.

## Plan

### 1. Extend `PDO::SlaveConfig` (`EtherCATPDO.hpp`)
Add two counter fields:
```cpp
uint32_t pdo_request_count = 0;   // successful RxPDO sends (master → slave)
uint32_t pdo_reply_count   = 0;   // successful TxPDO receives (slave → master)
```

### 2. Add PDOManager accessors (`EtherCATPDO.hpp` + `PDOManager.cpp`)
```cpp
bool     hasSlavePDOEntries(uint16_t slave_index) const;
uint32_t getSlavePDORequestCount(uint16_t slave_index) const;
uint32_t getSlavePDOReplyCount(uint16_t slave_index) const;
```
- `hasSlavePDOEntries` iterates the mapping and returns `true` if any enabled entry belongs to the given slave.
- The getters read the corresponding `slave_configs_` slot (returning `0` for out-of-range indices).

### 3. Increment counters in `PDOManager::sendRxPDO()`
After a successful send, increment `slave_configs_[entry->slave_index].pdo_request_count`.

### 4. Increment counters in `PDOManager::receiveTxPDO()`
After a successful receive, increment `slave_configs_[entry->slave_index].pdo_reply_count`.

### 5. Increment counters in `PDOManager::exchangeAll()` (LRW path)
When `logical_addr_mgr_->exchangeAllLRW(mapping_)` succeeds, iterate all mapping entries and increment the request/reply counters for each slave that has an enabled RxPDO/TxPDO entry.

### 6. Enforce the check in `Slave::transitionToOp()` (`EtherCATSlave.cpp`)
At the start of `transitionToOp()`, before requesting OP:
```cpp
auto& pdo_mgr = master_.pdo();
if (pdo_mgr.hasSlavePDOEntries(index_)) {
    if (pdo_mgr.getSlavePDORequestCount(index_) == 0 ||
        pdo_mgr.getSlavePDOReplyCount(index_) == 0) {
        TETHER_LOGE(TAG, "Slave %u: OP transition rejected — "
                    "no PDO exchange (req=%u reply=%u)", index_, ...);
        return SlaveError::TransportError;
    }
}
```
If the slave has no mapped PDOs, the check is skipped (mailbox-only slaves are unaffected).

### 7. Reset counters in `PDOManager::init()`
Zero the counters alongside the rest of the `slave_configs_` array during initialisation.

## Files to modify
- `include/tether/ethercat/EtherCATPDO.hpp` — extend `SlaveConfig`, add PDOManager methods.
- `src/ethercat/raw/PDOManager.cpp` — implement accessors, increment counters, reset on init.
- `src/ethercat/EtherCATSlave.cpp` — add OP-transition gate.
