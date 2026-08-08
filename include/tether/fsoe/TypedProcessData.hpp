#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>

#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"

namespace FSoE {

template<typename MainToSlavePayload, typename SlaveToMainPayload, typename Codec>
class TypedMainProcessDataView {
public:
    explicit TypedMainProcessDataView(FSoEMasterConnection& connection)
        : connection_(connection)
    {
    }

    bool write(const MainToSlavePayload& payload)
    {
        std::array<uint8_t, Codec::kMainToSlaveSize> bytes{};
        Codec::encodeMainToSlave(payload, bytes);
        return connection_.writeOutputProcessData(bytes);
    }

    std::optional<SlaveToMainPayload> read() const
    {
        std::array<uint8_t, Codec::kSlaveToMainSize> bytes{};
        if (connection_.readInputProcessData(bytes) != bytes.size()) {
            return std::nullopt;
        }

        return Codec::decodeSlaveToMain(bytes);
    }

    bool exchangeWith(FSoESlave& slave, uint64_t current_time_ms)
    {
        return connection_.exchangeWith(slave, current_time_ms);
    }

    /// Typed FSoE exchange with a slave emulator, with profile-specific hooks.
    ///
    /// Encodes @p payload, runs the raw master↔slave frame exchange.  @p mid_exchange
    /// is invoked after the master→slave frame has been delivered to the slave but
    /// before the slave→master frame is read (e.g. for the SafeMotionServoEmulator's
    /// command/status synchronization).  @p on_success is invoked only when the
    /// raw frame exchange succeeds (e.g. to clear edge-triggered pulse bits).
    ///
    /// The decoded slave→main payload is not returned here because it may be
    /// unavailable (std::nullopt) during the handshake phase even when the raw
    /// exchange succeeds.  Call read() separately to obtain it.
    ///
    /// @return true if the raw frame exchange succeeded.
    bool exchangeWith(
        FSoESlave& slave,
        const MainToSlavePayload& payload,
        uint64_t current_time_ms,
        const std::function<void()>& mid_exchange = {},
        const std::function<void()>& on_success = {})
    {
        if (!write(payload)) {
            return false;
        }

        // The raw exchange drives both FSMs and the frame round-trip.
        // FSoEMasterConnection::exchangeWith does not expose a mid-exchange
        // hook, so we replicate the round-trip here to insert @p mid_exchange
        // between the master→slave and slave→master halves.
        connection_.update(current_time_ms);
        slave.update(current_time_ms);

        std::array<uint8_t, 64> tx{};
        std::array<uint8_t, 64> rx{};

        const size_t tx_len = connection_.prepareTxFrame(tx.data(), tx.size());
        if (tx_len == 0) {
            return false;
        }
        if (!slave.processRxFrame(tx.data(), tx_len)) {
            return false;
        }

        if (mid_exchange) {
            mid_exchange();
        }

        const size_t rx_len = slave.prepareTxFrame(rx.data(), rx.size());
        if (rx_len == 0) {
            return false;
        }
        if (!connection_.processRxFrame(rx.data(), rx_len)) {
            return false;
        }

        if (on_success) {
            on_success();
        }
        return true;
    }

    /// Typed FSoE-over-PDO exchange for real drive communication.
    ///
    /// Encodes @p payload into the connection's safe outputs, runs the FSoE
    /// state machine, builds the master→slave frame into @p rx_pdo_out
    /// (the RxPDO buffer), and processes the slave→master frame from
    /// @p tx_pdo_in (the TxPDO buffer).  @p on_success is invoked only when
    /// the raw exchange succeeds (e.g. to clear edge-triggered pulse bits).
    ///
    /// The decoded slave→main payload is not returned here because it may be
    /// unavailable (std::nullopt) during the handshake phase even when the raw
    /// exchange succeeds.  Call read() separately to obtain it.
    ///
    /// @return true if the raw frame exchange succeeded.
    bool exchangeViaPDO(
        uint8_t* rx_pdo_out, size_t rx_pdo_max,
        const uint8_t* tx_pdo_in, size_t tx_pdo_len,
        const MainToSlavePayload& payload,
        uint64_t current_time_ms,
        const std::function<void()>& on_success = {})
    {
        if (!write(payload)) {
            return false;
        }

        if (!connection_.exchangeViaPDO(rx_pdo_out, rx_pdo_max,
                                        tx_pdo_in, tx_pdo_len,
                                        current_time_ms)) {
            return false;
        }

        if (on_success) {
            on_success();
        }
        return true;
    }

    FSoEMasterConnection& rawConnection() { return connection_; }
    const FSoEMasterConnection& rawConnection() const { return connection_; }

private:
    FSoEMasterConnection& connection_;
};

template<typename MainToSlavePayload, typename SlaveToMainPayload, typename Codec>
class TypedSlaveProcessDataView {
public:
    explicit TypedSlaveProcessDataView(FSoESlave& slave)
        : slave_(slave)
    {
    }

    bool publish(const SlaveToMainPayload& payload)
    {
        std::array<uint8_t, Codec::kSlaveToMainSize> bytes{};
        Codec::encodeSlaveToMain(payload, bytes);
        return slave_.writeInputProcessData(bytes);
    }

    std::optional<MainToSlavePayload> consume() const
    {
        std::array<uint8_t, Codec::kMainToSlaveSize> bytes{};
        if (slave_.readOutputProcessData(bytes) != bytes.size()) {
            return std::nullopt;
        }

        return Codec::decodeMainToSlave(bytes);
    }

    FSoESlave& rawSlave() { return slave_; }
    const FSoESlave& rawSlave() const { return slave_; }

private:
    FSoESlave& slave_;
};

} // namespace FSoE