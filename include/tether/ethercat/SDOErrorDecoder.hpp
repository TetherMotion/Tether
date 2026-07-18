#pragma once

#include <cstdint>

namespace EtherCAT {
namespace Raw {

class SDOErrorDecoder {
public:
    virtual ~SDOErrorDecoder() = default;

    virtual const char* sdoAbortCodeStr(uint32_t code) const;
    virtual const char* mbxErrorCodeStr(uint16_t code) const;
    virtual const char* mbxErrorDetailStr(uint16_t errCode, uint16_t detail) const;
};

} // namespace Raw
} // namespace EtherCAT
