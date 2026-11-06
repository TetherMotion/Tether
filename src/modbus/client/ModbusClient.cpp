#include "tether/modbus/client/ModbusClient.hpp"

#include "tether/modbus/client/BusExecutor.hpp"
#include "tether/modbus/link/SerialPortLink.hpp"
#include "tether/modbus/link/TcpLink.hpp"

namespace tether::modbus {

Expected<RtuMaster> makeRtuMaster(const SerialConfig& cfg, uint8_t unit,
                                  const ChannelOpts& copt,
                                  const OpenOpts& oopt,
                                  const TransactionOptions& iopt) {
#if defined(ESP_PLATFORM)
    return err<RtuMaster>(ModbusError::unsupported(
        "serial link not built (ESP)"));
#else
    auto link = makeSerialLink(cfg, oopt);
    if (!link) return err<RtuMaster>(link.error());
    RtuMaster m;
    m.channel = std::make_unique<RtuChannel>(
        std::move(*link), RtuTiming::forSerial(cfg), copt);
    if (auto e = m.channel->open(); !e) return err<RtuMaster>(e.error());
    m.executor = std::make_unique<BusExecutor>(*m.channel);
    m.executor->setDefaultOptions(iopt);
    m.client = std::make_unique<ModbusClient>(*m.executor, unit);
    return m;
#endif
}

Expected<TcpMaster> makeTcpMaster(std::string_view host, uint16_t port,
                                  uint8_t unit, int connectMs,
                                  const TransactionOptions& iopt) {
#if defined(ESP_PLATFORM)
    return err<TcpMaster>(ModbusError::unsupported("tcp link not built"));
#else
    auto link = makeTcpLink(host, port, connectMs);
    if (!link) return err<TcpMaster>(link.error());
    TcpMaster m;
    m.channel = std::make_unique<MbapChannel>(std::move(*link));
    if (auto e = m.channel->open(); !e) return err<TcpMaster>(e.error());
    m.executor = std::make_unique<BusExecutor>(*m.channel);
    m.executor->setDefaultOptions(iopt);
    m.client = std::make_unique<ModbusClient>(*m.executor, unit);
    return m;
#endif
}

Expected<TcpMaster> makeTcpMasterPipelined(
    std::string_view host, uint16_t port, uint8_t unit, int maxInflight,
    int connectMs, const TransactionOptions& iopt) {
#if defined(ESP_PLATFORM)
    return err<TcpMaster>(ModbusError::unsupported("tcp link not built"));
#else
    auto link = makeTcpLink(host, port, connectMs);
    if (!link) return err<TcpMaster>(link.error());
    TcpMaster m;
    m.channel = std::make_unique<MbapChannel>(std::move(*link));
    m.txn = std::make_unique<TransactionManager>(*m.channel,
                                                 maxInflight);
    m.txn->setDefaultOptions(iopt);
    if (auto e = m.txn->start(); !e) return err<TcpMaster>(e.error());
    m.client = std::make_unique<ModbusClient>(*m.txn, unit);
    return m;
#endif
}

} // namespace tether::modbus
