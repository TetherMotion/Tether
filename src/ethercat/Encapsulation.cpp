#include "tether/ethercat/Encapsulation.hpp"

#include <iostream>
#include <sstream>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/NetworkAbstraction.hpp"
#include "tether/ethercat/VlanProbe.hpp"
#include "tether/hal/IEthernet.hpp"

namespace EtherCAT {

namespace {

bool parseVid(const std::string& s, uint16_t& out) {
    try {
        int v = std::stoi(s);
        if (v < 1 || v > 4095) return false;
        out = static_cast<uint16_t>(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseVidRange(const std::string& s,
                   VLANRouter::VLANRange& out) {
    const size_t dash = s.find('-');
    try {
        if (dash == std::string::npos) {
            uint16_t v;
            if (!parseVid(s, v)) return false;
            out = VLANRouter::VLANRange{v, v};
            return true;
        }
        uint16_t lo, hi;
        if (!parseVid(s.substr(0, dash), lo) ||
            !parseVid(s.substr(dash + 1), hi) || lo > hi) return false;
        out = VLANRouter::VLANRange{lo, hi};
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

bool parseEncapsulationSpec(const std::string& spec,
                            EncapsulationConfig& out,
                            const char* /*tag*/) {
    out = EncapsulationConfig{};
    if (spec.empty() || spec == "raw" || spec == "none")
        return true;   // plain EtherCAT

    std::optional<uint16_t> txVlanToken;   // explicit vlantx:
    bool txVlanSeen  = false;
    bool txVlanOff   = false;
    bool vlanSeen    = false;
    bool udpSeen     = false;

    std::stringstream ss(spec);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok.erase(0, tok.find_first_not_of(" \t"));
        tok.erase(tok.find_last_not_of(" \t") + 1);
        if (tok.empty()) continue;

        if (tok == "raw" || tok == "none") {
            if (spec.find(',') != std::string::npos) {
                std::cerr << "--encapsulation: 'raw'/'none' cannot be combined "
                             "with other tokens\n";
                return false;
            }
            continue;
        }

        if (tok.rfind("vlan:", 0) == 0) {
            if (vlanSeen) {
                std::cerr << "--encapsulation: duplicate 'vlan:' token\n";
                return false;
            }
            vlanSeen = true;
            const std::string arg = tok.substr(5);
            if (arg == "any") {
                out.rxAny = true;
            } else {
                VLANRouter::VLANRange r{0, 0};
                if (!parseVidRange(arg, r)) {
                    std::cerr << "--encapsulation: invalid vlan value '" << arg
                              << "' (expected vid 1-4095, lo-hi range, or 'any')\n";
                    return false;
                }
                out.rxRange = r;
                out.txVlan  = r.start;   // TX tag defaults to first RX VID
            }
            continue;
        }

        if (tok.rfind("vlantx:", 0) == 0) {
            if (txVlanSeen) {
                std::cerr << "--encapsulation: duplicate 'vlantx:' token\n";
                return false;
            }
            txVlanSeen = true;
            const std::string arg = tok.substr(7);
            if (arg == "off" || arg == "none") {
                txVlanOff = true;
            } else {
                uint16_t v;
                if (!parseVid(arg, v)) {
                    std::cerr << "--encapsulation: invalid vlantx value '"
                              << arg << "' (expected vid 1-4095 or 'off')\n";
                    return false;
                }
                txVlanToken = v;
            }
            continue;
        }

        if (tok == "udp" || tok.rfind("udp:", 0) == 0) {
            if (udpSeen) {
                std::cerr << "--encapsulation: duplicate 'udp' token\n";
                return false;
            }
            udpSeen = true;
            out.udp = true;
            if (tok.size() > 4) {
                try {
                    int p = std::stoi(tok.substr(4));
                    if (p < 1 || p > 65535) throw std::out_of_range("port");
                    out.udpPort = static_cast<uint16_t>(p);
                } catch (...) {
                    std::cerr << "--encapsulation: invalid udp port '"
                              << tok.substr(4) << "' (expected 1-65535)\n";
                    return false;
                }
            }
            continue;
        }

        std::cerr << "--encapsulation: unknown token '" << tok
                  << "' (expected vlan:<vid|lo-hi|any>, vlantx:<vid|off>, "
                     "udp[:<port>], raw)\n";
        return false;
    }

    // An explicit vlantx: overrides the VID implied by vlan:<vid>.
    if (txVlanOff) out.txVlan = std::nullopt;
    else if (txVlanToken) out.txVlan = *txVlanToken;

    if (txVlanSeen && !vlanSeen && !txVlanOff && !txVlanToken) {
        // unreachable, but keep the invariant obvious
        return false;
    }
    return true;
}

void logEncapsulationConfig(const EncapsulationConfig& config,
                            const char* tag) {
    if (!config.enabled()) return;

    if (config.rxAny) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX=any (undefined target), TX={}",
                    config.txVlan ? std::to_string(*config.txVlan).c_str()
                                  : "untagged");
    } else if (config.rxRange) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX={}-{}, TX={}",
                    config.rxRange->start, config.rxRange->end,
                    config.txVlan ? std::to_string(*config.txVlan).c_str()
                                  : "untagged");
    } else if (config.txVlan) {
        TETHER_LOGI(tag, "Encapsulation: VLAN RX=untagged, TX={}",
                    *config.txVlan);
    }
    if (config.udp) {
        TETHER_LOGI(tag, "Encapsulation: EtherCAT-over-UDP dst port {}",
                    config.udpPort);
    }
}

std::vector<CBPFInsn> buildEncapsulationBpfProgram(
    const EncapsulationConfig& config, VlanDeliveryHint delivery) {
    CBPFSpec s;
    s.udp_port      = config.udpPort;
    s.vlan_delivery = delivery;

    if (config.rxRange) {
        // VLAN mode: reject everything except VID∈range + inner EtherCAT
        // (and inner UDP-encapsulated EtherCAT when udp is on).
        s.untagged_ethercat = false;
        s.untagged_udp      = false;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
        s.vlan_range        = CBPFVlanRange{config.rxRange->start,
                                            config.rxRange->end};
    } else if (config.rxAny) {
        // Catch-all tagged traffic: no untagged, any-VID tagged EtherCAT.
        s.untagged_ethercat = false;
        s.untagged_udp      = false;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
    } else {
        // Default / TX-tag-only: untagged + any-VID tagged EtherCAT (+UDP).
        s.untagged_ethercat = true;
        s.untagged_udp      = config.udp;
        s.tagged_ethercat   = true;
        s.tagged_udp        = config.udp;
    }
    return CBPFProgramFactory::build(s);
}

void attachEncapsulationBpfFilter(
    HAL::IEthernet& eth,
    const EncapsulationConfig& config,
    const char* tag,
    VlanDeliveryHint delivery) {
    const int fd = static_cast<int>(
        reinterpret_cast<intptr_t>(eth.nativeHandle()));
    if (fd < 0) return;   // backend without a socket fd — nothing to attach

    const auto prog = buildEncapsulationBpfProgram(config, delivery);
    if (prog.empty()) {
        TETHER_LOGW(tag, "Encapsulation produced an empty BPF program — "
                         "no filter attached");
        return;
    }
    if (!CBPFProgramFactory::attach(fd, prog)) {
        // Soft failure: userspace filtering keeps correctness; only the
        // kernel-side wakeup savings are lost.
        TETHER_LOGW(tag, "BPF socket filter attach failed — continuing "
                         "without kernel filtering");
    } else {
        TETHER_LOGI(tag, "Kernel cBPF filter attached ({} insns)", prog.size());
    }
}

NetworkInterface* setupEncapsulation(
    HAL::IEthernet& eth,
    NetworkInterface& backend,
    Master& master,
    std::unique_ptr<VLANRouter>& routerStorage,
    const EncapsulationConfig& encapsulation,
    const char* tag) {
    // 0. Wire probe — before any filter is attached and before the poll
    //    thread is running.  A bogus-logical-address LRW is forwarded
    //    around the whole segment unchanged, so the reply measures the
    //    real round-trip — startCyclicLoop() sizes the collect deadline
    //    from it.  Under VLAN encapsulation it additionally detects how
    //    the kernel delivers the tag, which prunes the dead leg out of
    //    every generated filter; a missing reply means the bus is not a
    //    closed loop or the VID never reaches the slaves — fail startup
    //    instead of spinning silently.  Untagged links only warn: the
    //    probe may be refused on bare segments and RTT autosizing is a
    //    convenience, not a correctness gate there.
    VlanDeliveryHint delivery = VlanDeliveryHint::Auto;
    int64_t probe_rtt_us = -1;
    {
        const uint16_t probe_vid = encapsulation.vlanActive()
            ? encapsulation.txVlan.value_or(
                  encapsulation.rxRange ? encapsulation.rxRange->start : 0)
            : 0;
        const auto probe = probeVlanTagDelivery(eth, probe_vid);
        const bool answered =
            probe && probe->delivery != VlanTagDelivery::NoReply;
        if (answered) {
            probe_rtt_us = probe->rtt_us;
            TETHER_LOGI(tag, "wire probe: reply in {} us (vid={}, "
                             "tag delivery = {}, tpid=0x{:04X})",
                        probe->rtt_us, probe->vid,
                        toString(probe->delivery), probe->tpid);
        }
        if (encapsulation.vlanActive()) {
            if (!answered) {
                TETHER_LOGE(tag, "VLAN probe: no reply to bogus-LRW within "
                                 "10 ms (vid={}) — segment not a closed "
                                 "loop or VID never reaches the bus",
                            probe_vid);
                return nullptr;
            }
            if (encapsulation.rxRange &&
                !encapsulation.rxRange->contains(probe->vid)) {
                TETHER_LOGW(tag, "VLAN probe: replies carry VID {} outside "
                                 "the configured RX range {}-{}",
                            probe->vid, encapsulation.rxRange->start,
                            encapsulation.rxRange->end);
            }
            delivery = vlanDeliveryHint(probe->delivery);
            if (delivery != VlanDeliveryHint::Auto)
                TETHER_LOGI(tag, "Generating filters for delivery mode '{}'",
                            toString(probe->delivery));
        } else if (!answered) {
            TETHER_LOGW(tag, "wire probe: no reply to bogus-LRW — cyclic "
                             "RX budget falls back to the cycle-period "
                             "default (segment not a closed loop?)");
        }

        // Feed encap + hint + measured RTT into the master's WireEncap:
        // the cyclic datapath composes its socket-pair demux programs
        // with the same knowledge, bakes the TX tag into cyclic header
        // templates, and the loop sizes rx_budget_ns from the RTT.
        WireEncap we;
        if (encapsulation.vlanActive()) {
            we.tx_vlan     = encapsulation.txVlan.value_or(0);
            we.rx_vlan_any = encapsulation.rxAny;
            if (encapsulation.rxRange) {
                we.rx_vlan_lo = encapsulation.rxRange->start;
                we.rx_vlan_hi = encapsulation.rxRange->end;
            }
        }
        we.delivery_hint = delivery;
        if (probe_rtt_us > 0)
            we.wire_rtt_ns = static_cast<uint32_t>(probe_rtt_us) * 1000u;
        master.setWireEncap(we);
    }

    // 1. Kernel-side ingress filter.
    attachEncapsulationBpfFilter(eth, encapsulation, tag, delivery);

    // 2. EtherCAT-over-UDP.
    if (encapsulation.udp) {
        UdpEncapsulationConfig uc;
        uc.enabled          = true;
        uc.destination_port = encapsulation.udpPort;
        master.setUdpEncapsulation(uc);
        if (!master.isUdpEncapsulationEnabled()) {
            TETHER_LOGE(tag, "--encapsulation udp requires a build with "
                             "TETHER_ENABLE_UDP_ENCAPSULATION=ON");
            return nullptr;
        }
        // The HAL's software EtherType filter (0x88A4) would drop IPv4/UDP
        // frames — disable it; the attached cBPF filter does the real work
        // and the master parser rejects anything stray regardless.
        eth.setEthertypeFilter(0);
    }

    // 3. VLAN router or direct RX callback.
    if (encapsulation.vlanActive()) {
        // VLAN-routed masters must not bypass the VLANRouter: drop the
        // direct-receive fast path so the cyclic executive waits on the
        // eventfd/slot while the poll thread keeps demuxing by VLAN tag.
        backend.receive       = nullptr;
        backend.native_handle = nullptr;

        routerStorage = std::make_unique<VLANRouter>();
        routerStorage->setBackend(&backend);
        // Alias shared_ptr: caller owns the master for the session lifetime.
        auto master_sp =
            std::shared_ptr<Master>(&master, [](auto*) {});
        if (encapsulation.rxAny) {
            routerStorage->setUndefinedTarget(master_sp,
                                              encapsulation.txVlan, true);
        } else if (encapsulation.rxRange) {
            routerStorage->addMaster(master_sp, *encapsulation.rxRange,
                                     encapsulation.txVlan);
        } else {
            routerStorage->addMaster(master_sp, std::nullopt,
                                     encapsulation.txVlan);
        }

        auto* router = routerStorage.get();
        eth.setRxCallback(
            [router](const uint8_t* frame, size_t len,
                     const HAL::RxFrameInfo& info, void*) {
                // When the kernel stripped the tag (rx-vlan-offload), the
                // VID arrives only via metadata — pass it out-of-band so
                // the router needs no reinserted tag bytes.
                std::optional<uint16_t> aux_vlan;
                if (info.vlanTagPresent && len >= 14 &&
                    !(frame[12] == 0x81 && frame[13] == 0x00))
                    aux_vlan = info.vlanId;
                router->processRxFrame(frame, len, aux_vlan);
            },
            nullptr);

        NetworkInterface* masterIface = encapsulation.rxAny
            ? routerStorage->undefinedNetworkInterface()
            : routerStorage->networkInterfaceFor(&master);
        if (!masterIface) {
            TETHER_LOGE(tag, "Failed to obtain per-master NetworkInterface "
                             "from VLAN router");
            return nullptr;
        }
        return masterIface;
    }

    eth.setRxCallback(
        [&master](const uint8_t* frame, size_t len,
                  const HAL::RxFrameInfo&, void*) {
            master.handleRxFrame(frame, len);
        },
        nullptr);
    return &backend;
}

} // namespace EtherCAT
