#pragma once

#include <array>
#include <cstddef>
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