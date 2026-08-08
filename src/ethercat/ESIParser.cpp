#include "tether/ethercat/ESIParser.hpp"
#include <tinyxml2.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <bit>
#include "tether/ethercat/PDOManager.hpp" // for SM flag constants
#include "tether/sii/SIIParser.hpp" // for mailbox protocol constants

using namespace tinyxml2;

namespace EtherCAT {
namespace ESI {

static uint32_t parseHexAttribute(const char* s) {
    if (!s) return 0;
    std::string v(s);
    // Attributes are often given as "#x1000". Accept forms with optional # and 0x.
    if (v.rfind("#x", 0) == 0 || v.rfind("#X", 0) == 0) v = v.substr(2);
    if (v.rfind("0x", 0) == 0 || v.rfind("0X", 0) == 0) v = v.substr(2);
    uint32_t val = 0;
    std::stringstream ss;
    ss << std::hex << v;
    ss >> val;
    return val;
}

bool parseESIFile(const std::string& path, std::vector<DeviceInfo>& devices, std::string& errMsg) {
    devices.clear();
    XMLDocument doc;
    XMLError r = doc.LoadFile(path.c_str());
    if (r != XML_SUCCESS) {
        errMsg = std::string("Failed to load XML: ") + doc.ErrorStr();
        return false;
    }

    XMLElement* root = doc.RootElement();
    if (!root) {
        errMsg = "No root element in XML";
        return false;
    }

    // Walk Devices/Device
    XMLElement* devicesEl = root->FirstChildElement("Descriptions");
    if (!devicesEl) devicesEl = root; // fallback
    if (devicesEl) devicesEl = devicesEl->FirstChildElement("Devices");

    if (!devicesEl) {
        errMsg = "No <Devices> element found";
        return false;
    }

    for (XMLElement* dev = devicesEl->FirstChildElement("Device"); dev; dev = dev->NextSiblingElement("Device")) {
        DeviceInfo info;
        XMLElement* typeEl = dev->FirstChildElement("Type");
        if (typeEl) {
            const char* t = typeEl->GetText();
            if (t) info.type = t;
            const char* prod = typeEl->Attribute("ProductCode");
            if (prod) info.productCode = parseHexAttribute(prod);
            const char* rev = typeEl->Attribute("RevisionNo");
            if (rev) info.revision = parseHexAttribute(rev);
        }
        XMLElement* nameEl = dev->FirstChildElement("Name");
        if (nameEl && nameEl->GetText()) info.name = nameEl->GetText();
        XMLElement* commentEl = dev->FirstChildElement("Comment");
        if (commentEl && commentEl->GetText()) info.comment = commentEl->GetText();

        // Find Vendor id under top-level <Vendor>
        XMLElement* vendorEl = root->FirstChildElement("Vendor");
        if (vendorEl) {
            XMLElement* idEl = vendorEl->FirstChildElement("Id");
            if (idEl && idEl->GetText()) {
                const char* vid = idEl->GetText();
                info.vendorId = parseHexAttribute(vid);
            }
        }

        // Mailbox timeouts: Descriptions/Devices/Device/Info/Mailbox/Timeout
        XMLElement* infoEl = dev->FirstChildElement("Info");
        if (infoEl) {
            XMLElement* mboxEl = infoEl->FirstChildElement("Mailbox");
            if (mboxEl) {
                XMLElement* tEl = mboxEl->FirstChildElement("Timeout");
                if (tEl) {
                    XMLElement* req = tEl->FirstChildElement("RequestTimeout");
                    XMLElement* resp = tEl->FirstChildElement("ResponseTimeout");
                    if (req && req->GetText()) info.mailbox_request_timeout_ms = std::stoul(req->GetText());
                    if (resp && resp->GetText()) info.mailbox_response_timeout_ms = std::stoul(resp->GetText());
                }
            }
        }

        // Scan for <Sm ...> nodes (Sync Managers) — visit each <Sm> once
        for (XMLElement* node = dev->FirstChildElement("Sm"); node; node = node->NextSiblingElement("Sm")) {
            SyncManagerEntry sment;
            const char* sa = node->Attribute("StartAddress");
            const char* dsz = node->Attribute("DefaultSize");
            const char* cb = node->Attribute("ControlByte");
            const char* en = node->Attribute("Enable");
            if (sa) sment.startAddress = static_cast<uint16_t>(parseHexAttribute(sa));
            if (dsz) sment.defaultSize = static_cast<uint16_t>(std::stoi(dsz));
            if (cb) sment.control = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(static_cast<uint8_t>(parseHexAttribute(cb)));
            if (en) sment.enable = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(static_cast<uint8_t>(std::stoi(en)));
            if (node->GetText()) sment.name = node->GetText();
            info.syncManagers.push_back(sment);
        }

        // Also attempt to find explicit Mailbox entries in <Sm> elements (MBoxOut/MBoxIn)
        // (Some ESI files put mailbox info as Sm elements)
        for (const auto& sm : info.syncManagers) {
            if (sm.name.find("MBox") != std::string::npos) {
                // Map to mailbox fields if present
                if (sm.name.find("MBoxOut") != std::string::npos) {
                    // master write -> device mailbox write address
                    // Save primary values
                    // startAddress and defaultSize used
                    // store only if present
                    // prefer the first seen
                    if (!info.mailbox.startAddress && sm.startAddress) info.mailbox.startAddress = sm.startAddress;
                    if (!info.mailbox.defaultSize && sm.defaultSize) info.mailbox.defaultSize = sm.defaultSize;
                }
            }
        }

        // Mailbox protocols - try to find <Mailbox> category entries with <Protocols>
        // In many ESI files protocols are under SIIDescription or in other places; search for "Protocols" child
        // naive search under Device.
        //
        // If no explicit <Protocols> text is found, derive protocol flags from
        // child elements: <CoE>, <AoE>, <EoE>, <FoE>, <SoE>, <VoE>.
        // This handles ESI files (e.g. Kinco RP20) that use child elements
        // rather than a numeric <Protocols> value.
        uint16_t derived_protocols = 0;
        bool found_protocols_text = false;
        for (XMLElement* c = dev->FirstChildElement(); c; c = c->NextSiblingElement()) {
            if (std::string(c->Name()) == "Mailbox") {
                XMLElement* protocols = c->FirstChildElement("Protocols");
                if (protocols && protocols->GetText()) {
                    info.mailbox.protocols = static_cast<uint16_t>(parseHexAttribute(protocols->GetText()));
                    found_protocols_text = true;
                }
                // Derive from child elements if no <Protocols> text
                if (!found_protocols_text) {
                    if (c->FirstChildElement("CoE")) derived_protocols |= SII::MBX_PROTO_COE;
                    if (c->FirstChildElement("AoE")) derived_protocols |= SII::MBX_PROTO_AOE;
                    if (c->FirstChildElement("EoE")) derived_protocols |= SII::MBX_PROTO_EOE;
                    if (c->FirstChildElement("FoE")) derived_protocols |= SII::MBX_PROTO_FOE;
                    if (c->FirstChildElement("SoE")) derived_protocols |= SII::MBX_PROTO_SOE;
                    if (c->FirstChildElement("VoE")) derived_protocols |= SII::MBX_PROTO_VOE;
                }
            }
            // Also check deeper
            XMLElement* child = c->FirstChildElement("Mailbox");
            if (child) {
                XMLElement* protocols = child->FirstChildElement("Protocols");
                if (protocols && protocols->GetText()) {
                    info.mailbox.protocols = static_cast<uint16_t>(parseHexAttribute(protocols->GetText()));
                    found_protocols_text = true;
                }
                if (!found_protocols_text) {
                    if (child->FirstChildElement("CoE")) derived_protocols |= SII::MBX_PROTO_COE;
                    if (child->FirstChildElement("AoE")) derived_protocols |= SII::MBX_PROTO_AOE;
                    if (child->FirstChildElement("EoE")) derived_protocols |= SII::MBX_PROTO_EOE;
                    if (child->FirstChildElement("FoE")) derived_protocols |= SII::MBX_PROTO_FOE;
                    if (child->FirstChildElement("SoE")) derived_protocols |= SII::MBX_PROTO_SOE;
                    if (child->FirstChildElement("VoE")) derived_protocols |= SII::MBX_PROTO_VOE;
                }
            }
        }
        // If no explicit <Protocols> text was found but we derived flags from
        // child elements, use the derived value.
        if (!found_protocols_text && derived_protocols != 0 && !info.mailbox.protocols) {
            info.mailbox.protocols = derived_protocols;
        }

        // FMMU list (simple name entries)
        for (XMLElement* f = dev->FirstChildElement("Fmmu"); f; f = f->NextSiblingElement("Fmmu")) {
            if (f->GetText()) info.fmmus.push_back(f->GetText());
        }

        // Parse RxPdo / TxPdo blocks
        auto parsePdoBlock = [&](const char* tag, std::vector<PDO>& outVec) {
            for (XMLElement* pdo = dev->FirstChildElement(tag); pdo; pdo = pdo->NextSiblingElement(tag)) {
                PDO obj;
                const char* idx = pdo->FirstChildElement("Index") ? pdo->FirstChildElement("Index")->GetText() : nullptr;
                if (idx) obj.index = static_cast<uint16_t>(parseHexAttribute(idx));
                const char* nm = pdo->FirstChildElement("Name") ? pdo->FirstChildElement("Name")->GetText() : nullptr;
                if (nm) obj.name = nm;
                const char* fixedAttr = pdo->Attribute("Fixed");
                if (fixedAttr) obj.fixed = (std::string(fixedAttr) != "0");
                const char* smAttr = pdo->Attribute("Sm");
                if (smAttr) obj.sm = std::stoi(smAttr);

                // Exclude entries
                for (XMLElement* ex = pdo->FirstChildElement("Exclude"); ex; ex = ex->NextSiblingElement("Exclude")) {
                    if (ex->GetText()) obj.excludes.push_back(static_cast<uint16_t>(parseHexAttribute(ex->GetText())));
                }

                // Entries
                for (XMLElement* e = pdo->FirstChildElement("Entry"); e; e = e->NextSiblingElement("Entry")) {
                    PDOEntry ent;
                    XMLElement* iEl = e->FirstChildElement("Index");
                    XMLElement* sEl = e->FirstChildElement("SubIndex");
                    XMLElement* bEl = e->FirstChildElement("BitLen");
                    XMLElement* nEl = e->FirstChildElement("Name");
                    XMLElement* dtEl = e->FirstChildElement("DataType");
                    if (iEl && iEl->GetText()) ent.index = static_cast<uint16_t>(parseHexAttribute(iEl->GetText()));
                    if (sEl && sEl->GetText()) ent.subindex = static_cast<uint8_t>(std::stoi(sEl->GetText()));
                    if (bEl && bEl->GetText()) ent.bitLen = static_cast<uint16_t>(std::stoi(bEl->GetText()));
                    if (nEl && nEl->GetText()) ent.name = nEl->GetText();
                    if (dtEl && dtEl->GetText()) ent.dataType = dtEl->GetText();
                    obj.entries.push_back(ent);
                }

                outVec.push_back(obj);
            }
        };

        parsePdoBlock("RxPdo", info.rxPdos);
        parsePdoBlock("TxPdo", info.txPdos);

        devices.push_back(info);
    }

    return true;
}

// Format helpers
static std::string formatHex(uint32_t v, int width=4) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(width) << v;
    return ss.str();
}

static std::string formatSMControl(const EtherCAT::SyncManager::SMControlReg& ctrl) {
    using namespace EtherCAT::PDO;
    std::string out;
    uint8_t modeBits = ctrl.mode;
    if (modeBits == SM_CTRL_MODE_BUFFERED) out += "BUFFERED";
    else if (modeBits == SM_CTRL_MODE_MAILBOX) out += "MAILBOX";
    else {
        out += "MODE_" + std::to_string(modeBits);
    }

    out += " (";
    bool first = true;
    if (ctrl.direction) { if (!first) out += " | "; out += "MASTER->SLAVE"; first=false; }
    else { if (!first) out += " | "; out += "SLAVE->MASTER"; first=false; }
    if (ctrl.ecat_irq) { if (!first) out += " | "; out += "IRQ_ECAT"; first=false; }
    if (ctrl.pdi_irq)  { if (!first) out += " | "; out += "IRQ_PDI"; first=false; }
    if (ctrl.watchdog) { if (!first) out += " | "; out += "WATCHDOG"; first=false; }
    if (ctrl.repeat_req) { if (!first) out += " | "; out += "REPEAT_REQ"; }
    out += ")";
    return out;
}

static std::string formatMailboxProtocols(uint16_t proto) {
    using namespace EtherCAT::SII;
    if (proto == 0) return "-";
    std::string out;
    bool first=false;
    if (proto & MBX_PROTO_AOE) { if (first) out += " | "; out += "AoE"; first=true; }
    if (proto & MBX_PROTO_EOE) { if (first) out += " | "; out += "EoE"; first=true; }
    if (proto & MBX_PROTO_COE) { if (first) out += " | "; out += "CoE"; first=true; }
    if (proto & MBX_PROTO_FOE) { if (first) out += " | "; out += "FoE"; first=true; }
    if (proto & MBX_PROTO_SOE) { if (first) out += " | "; out += "SoE"; first=true; }
    if (proto & MBX_PROTO_VOE) { if (first) out += " | "; out += "VoE"; }
    return out;
}

std::string formatDeviceHumanReadable(const DeviceInfo& dev, bool onlyMailboxes) {
    std::stringstream ss;
    ss << "Device: " << (dev.name.empty() ? dev.type : dev.name) << "\n";
    if (!dev.comment.empty()) ss << "  Comment: " << dev.comment << "\n";
    ss << "  VendorId: " << formatHex(dev.vendorId, 8) << "  ProductCode: " << formatHex(dev.productCode, 8) << " Revision: " << formatHex(dev.revision, 4) << "\n";

    if (!onlyMailboxes) {
        // Sync Managers
        if (!dev.syncManagers.empty()) {
            ss << "  Sync Managers:\n";
            for (size_t i=0;i<dev.syncManagers.size();++i) {
                const auto& s = dev.syncManagers[i];
                ss << "    SM" << i << ": start=" << formatHex(s.startAddress,4) << " len=" << s.defaultSize
                   << " ctrl=0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (unsigned)std::bit_cast<uint8_t>(s.control) << std::dec
                   << " " << formatSMControl(s.control) << " enable=" << (unsigned)std::bit_cast<uint8_t>(s.enable) << " name=" << s.name << "\n";
            }
        }

        // FMMUs
        if (!dev.fmmus.empty()) {
            ss << "  FMMUs:\n";
            for (const auto& f : dev.fmmus) ss << "    " << f << "\n";
        }

        // Mailbox timeouts
        if (dev.mailbox_request_timeout_ms || dev.mailbox_response_timeout_ms) {
            ss << "  Mailbox Timeouts:";
            if (dev.mailbox_request_timeout_ms) ss << " Request=" << *dev.mailbox_request_timeout_ms << "ms";
            if (dev.mailbox_response_timeout_ms) ss << " Response=" << *dev.mailbox_response_timeout_ms << "ms";
            ss << "\n";
        }
    }

    // Mailbox details
    if (dev.mailbox.startAddress || dev.mailbox.defaultSize || dev.mailbox.protocols) {
        ss << "  Mailbox:\n";
        if (dev.mailbox.startAddress) ss << "    StartAddress=" << formatHex(*dev.mailbox.startAddress,4) << "\n";
        if (dev.mailbox.defaultSize)  ss << "    DefaultSize=" << *dev.mailbox.defaultSize << "\n";
        if (dev.mailbox.protocols)    ss << "    Protocols=" << formatHex(*dev.mailbox.protocols,4) << " (" << formatMailboxProtocols(*dev.mailbox.protocols) << ")\n";
    }

    // PDOs
    if (!onlyMailboxes) {
        if (!dev.rxPdos.empty()) {
            ss << "  RxPDOs:\n";
            for (const auto& p : dev.rxPdos) {
                ss << "    PDO 0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << p.index << std::dec
                   << " name=" << p.name << " fixed=" << (p.fixed?"true":"false") << " sm=" << p.sm << " entries=" << p.entries.size() << "\n";
                for (const auto& e : p.entries) {
                    ss << "      - 0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << e.index << std::dec
                       << ", sub=" << (int)e.subindex << ", bits=" << e.bitLen << ", type=" << e.dataType << ", name=" << e.name << "\n";
                }
            }
        }
        if (!dev.txPdos.empty()) {
            ss << "  TxPDOs:\n";
            for (const auto& p : dev.txPdos) {
                ss << "    PDO 0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << p.index << std::dec
                   << " name=" << p.name << " fixed=" << (p.fixed?"true":"false") << " sm=" << p.sm << " entries=" << p.entries.size() << "\n";
                for (const auto& e : p.entries) {
                    ss << "      - 0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << e.index << std::dec
                       << ", sub=" << (int)e.subindex << ", bits=" << e.bitLen << ", type=" << e.dataType << ", name=" << e.name << "\n";
                }
            }
        }
    }

    return ss.str();
}

std::string formatDeviceJSON(const DeviceInfo& dev) {
    std::stringstream ss;
    ss << "{";
    ss << "\"name\":\"" << (dev.name.empty() ? dev.type : dev.name) << "\",";
    ss << "\"comment\":\"" << dev.comment << "\",";
    ss << "\"vendorId\":\"" << formatHex(dev.vendorId,8) << "\",";
    ss << "\"productCode\":\"" << formatHex(dev.productCode,8) << "\",";
    ss << "\"revision\":\"" << formatHex(dev.revision,4) << "\",";

    // Mailbox
    ss << "\"mailbox\":{";
    if (dev.mailbox.startAddress) ss << "\"start\":\"" << formatHex(*dev.mailbox.startAddress,4) << "\",";
    if (dev.mailbox.defaultSize) ss << "\"defaultSize\":" << *dev.mailbox.defaultSize << ",";
    if (dev.mailbox.protocols) ss << "\"protocols\":\"" << formatHex(*dev.mailbox.protocols,4) << "\"";
    ss << "},";

    // Sync managers
    ss << "\"syncManagers\": [";
    for (size_t i=0;i<dev.syncManagers.size();++i) {
        const auto& s = dev.syncManagers[i];
        ss << "{";
        ss << "\"start\":\"" << formatHex(s.startAddress,4) << "\",";
        ss << "\"length\":" << s.defaultSize << ",";
        ss << "\"control\":\"0x" << std::hex << std::uppercase << (unsigned)std::bit_cast<uint8_t>(s.control) << std::dec << "\",";
        ss << "\"enable\":" << (s.enable.enable ?"true":"false") << ",";
        ss << "\"name\":\"" << s.name << "\"";
        ss << "}";
        if (i+1<dev.syncManagers.size()) ss << ",";
    }
    ss << "],";

    // FMMUs
    ss << "\"fmmus\": [";
    for (size_t i=0;i<dev.fmmus.size();++i) {
        ss << "\"" << dev.fmmus[i] << "\"";
        if (i+1<dev.fmmus.size()) ss << ",";
    }
    ss << "],";

    // PDOs helper
    auto writePdos = [&](const char* key, const std::vector<PDO>& pdos) {
        ss << "\"" << key << "\": [";
        for (size_t i=0;i<pdos.size();++i) {
            const auto& p = pdos[i];
            ss << "{";
            ss << "\"index\":\"0x" << std::hex << std::uppercase << p.index << "\"" << std::dec << ",";
            ss << "\"name\":\"" << p.name << "\",";
            ss << "\"fixed\":" << (p.fixed?"true":"false") << ",";
            ss << "\"sm\":" << p.sm << ",";
            ss << "\"entries\": [";
            for (size_t j=0;j<p.entries.size();++j) {
                const auto& e = p.entries[j];
                ss << "{";
                ss << "\"index\":\"0x" << std::hex << std::uppercase << e.index << "\"" << std::dec << ",";
                ss << "\"sub\":" << (int)e.subindex << ",";
                ss << "\"bits\":" << e.bitLen << ",";
                ss << "\"type\":\"" << e.dataType << "\",";
                ss << "\"name\":\"" << e.name << "\"";
                ss << "}";
                if (j+1<p.entries.size()) ss << ",";
            }
            ss << "]}";
            if (i+1<pdos.size()) ss << ",";
        }
        ss << "],";
    };

    writePdos("rxPdos", dev.rxPdos);
    writePdos("txPdos", dev.txPdos);

    // Remove trailing comma if present and close object
    std::string out = ss.str();
    if (!out.empty() && out.back() == ',') out.pop_back();
    out += "}";
    return out;
}

} // namespace ESI
} // namespace EtherCAT
