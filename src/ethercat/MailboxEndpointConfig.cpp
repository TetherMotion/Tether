#include "tether/ethercat/MailboxEndpointConfig.hpp"

#include <iostream>

#include "logging/Logger.hpp"

namespace EtherCAT {

namespace {

bool parseHexOrDec(const std::string& s, int& out) {
    try {
        if (s.size() > 2 && (s.substr(0, 2) == "0x" || s.substr(0, 2) == "0X")) {
            out = std::stoi(s.substr(2), nullptr, 16);
        } else {
            out = std::stoi(s);
        }
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

bool parseMailboxSize(const std::string& str, MailboxSizeConfig& out) {
    // Short form: plain number (e.g. "256")
    if (str.find(':') == std::string::npos) {
        int val = 0;
        if (!parseHexOrDec(str, val) || val < 1 || val > 65535) {
            std::cerr << "Invalid --mailbox-size value: " << str << "\n";
            std::cerr << "Expected a number 1-65535 or in:<size>,out:<size>\n";
            return false;
        }
        out.inSize = static_cast<uint16_t>(val);
        out.outSize = static_cast<uint16_t>(val);
        return true;
    }

    // Long form: in:<size>,out:<size>
    size_t inPos = str.find("in:");
    size_t outPos = str.find("out:");
    if (inPos == std::string::npos || outPos == std::string::npos) {
        std::cerr << "Invalid --mailbox-size format: " << str << "\n";
        std::cerr << "Expected in:<size>,out:<size> or a single number\n";
        return false;
    }

    size_t inStart = inPos + 3;
    size_t inEnd = str.find(',', inStart);
    std::string inStr = str.substr(inStart, inEnd - inStart);

    size_t outStart = outPos + 4;
    std::string outStr = str.substr(outStart);

    int inVal = 0, outVal = 0;
    if (!parseHexOrDec(inStr, inVal) || inVal < 1 || inVal > 65535) {
        std::cerr << "Invalid --mailbox-size in-value: " << inStr << "\n";
        return false;
    }
    if (!parseHexOrDec(outStr, outVal) || outVal < 1 || outVal > 65535) {
        std::cerr << "Invalid --mailbox-size out-value: " << outStr << "\n";
        return false;
    }

    out.inSize = static_cast<uint16_t>(inVal);
    out.outSize = static_cast<uint16_t>(outVal);
    return true;
}

bool parseMailboxAddress(const std::string& str, MailboxAddressConfig& out) {
    size_t inPos = str.find("in:");
    size_t outPos = str.find("out:");
    if (inPos == std::string::npos || outPos == std::string::npos) {
        std::cerr << "Invalid --mailbox-address format: " << str << "\n";
        std::cerr << "Expected in:<hex>,out:<hex> (e.g. in:0x1000,out:0x1200)\n";
        return false;
    }

    size_t inStart = inPos + 3;
    size_t inEnd = str.find(',', inStart);
    std::string inStr = str.substr(inStart, inEnd - inStart);

    size_t outStart = outPos + 4;
    std::string outStr = str.substr(outStart);

    int inVal = 0, outVal = 0;
    if (!parseHexOrDec(inStr, inVal) || inVal < 0 || inVal > 65535) {
        std::cerr << "Invalid --mailbox-address in-value: " << inStr << "\n";
        return false;
    }
    if (!parseHexOrDec(outStr, outVal) || outVal < 0 || outVal > 65535) {
        std::cerr << "Invalid --mailbox-address out-value: " << outStr << "\n";
        return false;
    }

    if (inVal == outVal) {
        std::cerr << "--mailbox-address error: in-address (0x" << std::hex << inVal << ") must differ from out-address (0x" << outVal << ")" << std::dec << "\n";
        return false;
    }

    out.inAddress = static_cast<uint16_t>(inVal);
    out.outAddress = static_cast<uint16_t>(outVal);
    return true;
}

void logMailboxConfig(const MailboxSizeConfig& size,
                      const MailboxAddressConfig& addr,
                      const char* tag) {
    TETHER_LOGI(tag, "Mailbox config: MbxOut addr=0x{:04X} len={}, MbxIn addr=0x{:04X} len={}",
                addr.outAddress, size.outSize,
                addr.inAddress, size.inSize);
}

} // namespace EtherCAT
