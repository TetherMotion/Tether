// SPDX-License-Identifier: Apache-2.0 OR MPL-2.0
/**
 * @file tether_io_ec_server.cpp
 * @brief tether_io server exposing the EtherCAT/CoE (CANopen) surface.
 *
 * Two modes:
 *
 *   --simulate (default)  in-memory CiA 402 fleet + SimulatedSdoAccess —
 *                         machine.sdo.*, machine.pdo.map,
 *                         machine.supervisor.*, machine.diagnostics.*
 *                         all live, no hardware required.
 *
 *   --master-iface NAME     real EtherCAT master on kernel interface NAME
 *   --master-udp HOST:PORT  real master over the UDP link tunnel
 *                         (privilege-free; pair with beckhoff_slave_chain
 *                         or any UDP-link slave).  The CANopen/CoE
 *                         wrappers — machine.sdo.read/write/list over
 *                         EtherCatSdoAccess, machine.pdo.map via
 *                         EtherCatPdoAccess, machine.supervisor.* via
 *                         EtherCatSupervisorAccess, and CoE 0x10F3
 *                         diagnostics via EtherCatDiagnosticsAdapter —
 *                         run against the live master.  The generic
 *                         machine.command/state surface stays simulated
 *                         (DS402 axis bring-up is application-specific).
 *
 * The PLC reaches this server through tplc-canopen (TPLC_CANOPEN_IO)
 * which calls machine.sdo.* — CiA 301 object-dictionary access, the
 * same protocol CANopen uses on CAN, carried here over EtherCAT CoE.
 */
#include "tether/io/CiA402MachineProfile.hpp"
#include "tether/io/EtherCatDiagnosticsAdapter.hpp"
#include "tether/io/EtherCatSdoAdapter.hpp"
#include "tether/io/EtherCatSupervisionAdapter.hpp"
#include "tether/io/MachineControl.hpp"
#include "tether/io/MachineService.hpp"
#include "tether/io/ProducerHelpers.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"
#include "tether/io/SimulatedMachineService.hpp"
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TcpTransport.hpp"

#include <argparse/argparse.hpp>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "common/EthercatFleetMaster.hpp"

namespace io = tether::io;
namespace cia402 = tether::io::cia402;
namespace mach = tether::io::machine;

namespace {

struct Options {
    std::string bind = "127.0.0.1";
    uint16_t port = 42791;
    std::string masterIface;
    std::string masterUdp;
    std::string sessionRole;
    std::string sessionActor = "tether-io-tcp";
};

mach::Role parseRole(const std::string& name) {
    if (name == "observer") return mach::Role::Observer;
    if (name == "operator") return mach::Role::Operator;
    if (name == "technician") return mach::Role::Technician;
    if (name == "engineer") return mach::Role::ControlsEngineer;
    if (name == "admin") return mach::Role::Administrator;
    throw std::runtime_error("unknown --session-role: " + name);
}

Options parseArgs(int argc, char** argv) {
    argparse::ArgumentParser p("tether_io_ec_server");
    p.add_argument("--bind").default_value(std::string("127.0.0.1"));
    p.add_argument("--port").default_value(std::string("42791"));
    p.add_argument("--master-iface").default_value(std::string());
    p.add_argument("--master-udp").default_value(std::string());
    p.add_argument("--session-role")
        .help("assign every TCP session this authenticated role "
              "(observer/operator/technician/engineer/admin); needed "
              "for machine.sdo.* which requires an authenticated "
              "caller — plain TCP carries no credential handshake")
        .default_value(std::string());
    p.add_argument("--session-actor")
        .help("actor name recorded for --session-role identities")
        .default_value(std::string("tether-io-tcp"));
    p.parse_args(argc, argv);
    return {p.get<std::string>("--bind"),
            uint16_t(std::stoul(p.get<std::string>("--port"))),
            p.get<std::string>("--master-iface"),
            p.get<std::string>("--master-udp"),
            p.get<std::string>("--session-role"),
            p.get<std::string>("--session-actor")};
}

struct Running {
    io::SchemaGraph graph = cia402::machineProfileSchemaGraph();
    io::SchemaCatalog catalog;
    io::Registry registry;

    bool init() {
        return cia402::SimulatedCiA402Fleet::installSchemas(graph, catalog);
    }
};

io::ServerConfig makeConfig(io::SchemaCatalog& catalog,
                            const Options& opt) {
    io::ServerConfig sc;
    sc.timestampFn = io::steadyClockUs;
    sc.schemaCatalog = &catalog;
    if (!opt.sessionRole.empty()) {
        const mach::Role role = parseRole(opt.sessionRole);
        const std::string actor = opt.sessionActor;
        sc.sessionInitFn = [role, actor](io::Session& session) {
            mach::SessionIdentity id;
            id.actor = actor;
            id.source = "tether-io-tcp-session-role";
            id.role = role;
            id.authenticated = true;
            session.setIdentity(std::move(id));
        };
    }
    return sc;
}

int runSimulated(const Options& opt) {
    Running run;
    if (!run.init()) { std::fprintf(stderr, "schema install failed\n"); return 1; }

    cia402::SimulatedCiA402Fleet fleet;
    if (!fleet.registerSignals(run.registry, run.catalog)) {
        std::fprintf(stderr, "fleet registration failed\n"); return 1;
    }
    mach::SimulatedMachineStack stack;
    if (!stack.install(run.registry, run.catalog,
                       /*enableSdo*/ true, /*enableDiagnostics*/ true,
                       /*enableExtension*/ false)) {
        std::fprintf(stderr, "machine stack install failed\n"); return 1;
    }

    io::Server server(run.registry,
                      std::make_unique<io::TcpTransportServer>(
                          opt.port, opt.bind.c_str()),
                      makeConfig(run.catalog, opt));
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }
    std::printf("tether_io_ec_server (simulated) on %s:%u\n",
                opt.bind.c_str(), opt.port);
    std::fflush(stdout);
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(3600));
    return 0;
}

/** Real-master mode: live CoE/SDO/PDO/supervision wrappers; the
 *  machine.command surface stays simulated (per-axis DS402 dispatch is
 *  application configuration, out of scope for the generic server). */
int runMaster(const Options& opt) {
    tether::examples::FleetMaster fleet;
    bool ok = false;
    if (!opt.masterUdp.empty()) {
        std::string host = opt.masterUdp;
        uint16_t port = 0;
        const auto colon = host.rfind(':');
        if (colon != std::string::npos) {
            port = uint16_t(std::stoi(host.substr(colon + 1)));
            host = host.substr(0, colon);
        }
        ok = port != 0 && fleet.open(host, port);
    } else {
        ok = fleet.openIface(opt.masterIface);
    }
    if (!ok) { std::fprintf(stderr, "master open failed\n"); return 1; }
    auto& master = fleet.master();

    Running run;
    if (!run.init()) { std::fprintf(stderr, "schema install failed\n"); return 1; }

    /* simulated state/command surface */
    cia402::SimulatedCiA402Fleet simFleet;
    mach::SimulatedStateSource stateSource(simFleet);
    mach::AlarmService alarms;
    mach::SimulatedDispatcher dispatcher(simFleet, stateSource, alarms);
    mach::OperationTracker operations;
    mach::ControlAuthority authority(
        std::chrono::seconds(60), [](const mach::AuthorityEvent&) {});
    io::EventJournal journal;
    mach::MachineCommandGate gate(
        authority,
        [&stateSource](const mach::CommandRequest& r) {
            return stateSource.commandEnvironment(r);
        },
        [&dispatcher, &operations](const mach::CommandRequest& r,
                                   const mach::SessionIdentity& id) {
            return dispatcher.dispatch(r, id, operations);
        },
        [](const mach::AuditRecord&) { return std::string("audit"); });
    mach::MachineService service(
        stateSource, dispatcher, authority, gate, alarms, operations,
        &journal);

    /* live CoE wrappers — the CANopen surface */
    mach::EtherCatSdoAccess sdoAccess(master);
    mach::MachineSdoService sdoService(sdoAccess, &journal);
    service.setSdoService(&sdoService);
    mach::EtherCatPdoAccess pdoAccess(master);
    service.setPdoSource(&pdoAccess);
    mach::EtherCatSupervisorAccess supervisorAccess(master, &journal);
    mach::MachineSupervisorService supervisorService(supervisorAccess,
                                                     &journal);
    service.setSupervisor(&supervisorService);
    mach::EtherCatDiagnosticsAdapter diagnostics(master);
    service.setDiagnosticsSource(&diagnostics);

    if (!simFleet.registerSignals(run.registry, run.catalog)) {
        std::fprintf(stderr, "fleet registration failed\n"); return 1;
    }
    if (!service.install(run.registry, run.catalog)) {
        std::fprintf(stderr, "machine service install failed\n"); return 1;
    }

    io::Server server(run.registry,
                      std::make_unique<io::TcpTransportServer>(
                          opt.port, opt.bind.c_str()),
                      makeConfig(run.catalog, opt));
    if (!server.start()) { std::fprintf(stderr, "server start failed\n"); return 1; }
    std::printf("tether_io_ec_server (master) on %s:%u\n",
                opt.bind.c_str(), opt.port);
    std::fflush(stdout);
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(3600));
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    const Options opt = parseArgs(argc, argv);
    if (!opt.masterIface.empty() || !opt.masterUdp.empty())
        return runMaster(opt);
    return runSimulated(opt);
}
