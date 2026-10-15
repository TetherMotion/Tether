#include "tether/ethercat/SdoAbortReport.hpp"

#include <cstdio>

#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {

uint32_t reportSdoAbort(const Slave& slave, const char* tag) {
    const uint32_t abort_code = slave.lastSdoAbortCode();
    if (abort_code == 0) return 0;

    Raw::SDOErrorDecoder decoder;
    const char* meaning = decoder.sdoAbortCodeStr(abort_code);

    const bool was_download = slave.lastSdoWasDownload();
    const size_t attempted_len = slave.lastSdoAttemptedLength();
    const char* op_str = was_download ? "download (write)" : "upload (read)";
    const char* len_str = was_download
                              ? "payload length sent to slave"
                              : "read buffer capacity offered";

    TETHER_LOGE(tag, "Slave rejected the SDO request: CoE abort code 0x{:08X} ({}). "
                     "Operation: {}, attempted {}: {} bytes.",
                abort_code, meaning, op_str, len_str, attempted_len);

    // Always echo to stderr so the user sees it on the console even when log
    // output is redirected or silenced.
    std::fprintf(stderr,
                 "ERROR: Slave rejected the SDO request.\n"
                 "  CoE SDO abort code: 0x%08X (%s)\n"
                 "  Operation:          %s\n"
                 "  Attempted %s: %zu bytes\n",
                 abort_code, meaning,
                 op_str, len_str, attempted_len);

    // The length-mismatch family  -  the signature of the original
    // "slave rejects the write because the payload is the wrong size"
    // failure. Point the user at the object dictionary / ESI file.
    if (abort_code == 0x06070010 || abort_code == 0x06070012 ||
        abort_code == 0x06070013) {
        const char* which = (abort_code == 0x06070012)
                                ? "too high"
                                : (abort_code == 0x06070013) ? "too low"
                                                             : "mismatch";
        std::fprintf(stderr,
                     "  Cause: the payload size you sent (%zu bytes) does not match the size\n"
                     "         the slave expects for this object (length %s).\n"
                     "  Fix:   check the object dictionary / ESI (XML) file for the\n"
                     "         target index:subindex to find the correct data type\n"
                     "         and byte length, then send exactly that many bytes.\n",
                     attempted_len, which);
    }

    std::fflush(stderr);
    return abort_code;
}

} // namespace EtherCAT
