/**
 * @file MasterBatch.hpp
 * @brief Master batch (multi-datagram) transaction handle.
 *
 * Split out of Master.hpp.  Master re-exports it as Master::BatchTransaction.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

class TransactionRouter;

class BatchTransaction {
public:
    BatchTransaction() = default;
    BatchTransaction(TransactionRouter* router,
                     std::vector<uint8_t> idxs,
                     std::vector<size_t> slots,
                     std::vector<RxDatagram> responses);

    BatchTransaction(BatchTransaction&&) = default;
    BatchTransaction& operator=(BatchTransaction&&) = default;
    BatchTransaction(const BatchTransaction&) = delete;
    BatchTransaction& operator=(const BatchTransaction&) = delete;

    ~BatchTransaction();

    /// Get result for datagram at index i (blocks up to timeout_ms)
    BatchReadResult getResult(size_t i, uint32_t timeout_ms);

    /// Wait for all results (blocks up to timeout_ms per datagram)
    bool waitAll(uint32_t timeout_ms, std::vector<BatchReadResult>& out);

    /// Cancel any pending waits
    void cancel();

    /// Number of datagrams in this transaction
    size_t count() const { return idxs_.size(); }

private:
    TransactionRouter* router_{nullptr};
    std::vector<uint8_t> idxs_;
    std::vector<size_t> slots_;
    std::vector<RxDatagram> responses_;
    bool cancelled_{false};
};

} // namespace EtherCAT
