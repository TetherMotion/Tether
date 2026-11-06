/**
 * @file test_core_utils_coverage.cpp
 * @brief Coverage for small EtherCAT core helpers: describeControlword(),
 *        sdoAbortCodeStr() (all codes + uint32 overload), and the weak
 *        ecdc_* time-source defaults compiled into the master library.
 */

#include <gtest/gtest.h>

#include <string>

#include "tether/ethercat/Controlword.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/SDOAbortCodes.hpp"

using namespace EtherCAT;

// ============================================================================
// describeControlword — CiA-402 command decoding
// ============================================================================

TEST(ControlwordDescribe, CanonicalCommands) {
    EXPECT_NE(describeControlword(0x0080).find("FaultReset"), std::string::npos);
    EXPECT_NE(describeControlword(0x000F).find("EnableOperation"), std::string::npos);
    EXPECT_NE(describeControlword(0x0007).find("SwitchOn"), std::string::npos);
    EXPECT_NE(describeControlword(0x0006).find("Shutdown"), std::string::npos);
    EXPECT_NE(describeControlword(0x0002).find("QuickStop"), std::string::npos);
    EXPECT_NE(describeControlword(0x0000).find("DisableVoltage"), std::string::npos);
}

TEST(ControlwordDescribe, QuickStopRequiresBit1Only) {
    // bit1 set + bit2 clear -> QuickStop; both set is not quick-stop.
    EXPECT_NE(describeControlword(0x0002).find("QuickStop"), std::string::npos);
    EXPECT_EQ(describeControlword(0x0006).find("QuickStop"), std::string::npos);
}

TEST(ControlwordDescribe, UnknownCommand) {
    // 0x0004 matches no canonical command (EnOp bit alone).
    EXPECT_NE(describeControlword(0x0004).find("?"), std::string::npos);
}

TEST(ControlwordDescribe, BitAnnotations) {
    const std::string s = describeControlword(0x018F);
    EXPECT_NE(s.find("SwitchOn"), std::string::npos);   // bit0
    EXPECT_NE(s.find("EnVolt"), std::string::npos);     // bit1
    EXPECT_NE(s.find("!QStop"), std::string::npos);     // bit2 set
    EXPECT_NE(s.find("EnOp"), std::string::npos);       // bit3
    EXPECT_NE(s.find("FaultRst"), std::string::npos);   // bit7
    EXPECT_NE(s.find("HALT"), std::string::npos);       // bit8
    EXPECT_NE(s.find("0x018F"), std::string::npos);     // raw value rendered
}

TEST(ControlwordDescribe, ClearedBitsAnnotatedNegative) {
    const std::string s = describeControlword(0x0000);
    EXPECT_NE(s.find("!SwitchOn"), std::string::npos);
    EXPECT_NE(s.find("!EnVolt"), std::string::npos);
    EXPECT_NE(s.find("QSTOP!"), std::string::npos);     // bit2 clear
    EXPECT_NE(s.find("!EnOp"), std::string::npos);
    EXPECT_EQ(s.find("FaultRst"), std::string::npos);
    EXPECT_EQ(s.find("HALT"), std::string::npos);
}

// ============================================================================
// sdoAbortCodeStr — every enumerator + fallback + uint32 overload
// ============================================================================

TEST(SdoAbortCodes, EveryCodeHasDistinctText) {
    struct { SDOAbortCode code; const char* text; } cases[] = {
        {SDOAbortCode::Success,               "Success"},
        {SDOAbortCode::ToggleBitNotChanged,   "Toggle bit not alternated"},
        {SDOAbortCode::Timeout,               "SDO protocol timeout"},
        {SDOAbortCode::InvalidCommand,        "Invalid command"},
        {SDOAbortCode::InvalidBlockSize,      "Invalid block size"},
        {SDOAbortCode::InvalidSequenceNumber, "Invalid sequence number"},
        {SDOAbortCode::CrcError,              "CRC error"},
        {SDOAbortCode::OutOfMemory,           "Out of memory"},
        {SDOAbortCode::UnsupportedAccess,     "Unsupported access"},
        {SDOAbortCode::ReadOnlyObject,        "Write to read-only object"},
        {SDOAbortCode::WriteOnlyObject,       "Read from write-only object"},
        {SDOAbortCode::ObjectNotFound,        "Object does not exist"},
        {SDOAbortCode::PdoMappingError,       "Object cannot be mapped to PDO"},
        {SDOAbortCode::PdoLengthExceeded,     "PDO length exceeded"},
        {SDOAbortCode::ParameterIncompatible, "Parameter incompatibility"},
        {SDOAbortCode::InternalError,         "General internal incompatibility"},
        {SDOAbortCode::HardwareError,         "Hardware error"},
        {SDOAbortCode::DataTypeMismatch,      "Data type mismatch, length mismatch"},
        {SDOAbortCode::DataTypeTooLong,       "Data type mismatch, length too high"},
        {SDOAbortCode::DataTypeTooShort,      "Data type mismatch, length too low"},
        {SDOAbortCode::SubindexNotFound,      "Subindex does not exist"},
        {SDOAbortCode::InvalidValue,          "Invalid value for parameter"},
        {SDOAbortCode::ValueTooHigh,          "Value too high"},
        {SDOAbortCode::ValueTooLow,           "Value too low"},
        {SDOAbortCode::MaxLessThanMin,        "Maximum less than minimum"},
        {SDOAbortCode::ResourceNotAvailable,  "Resource not available"},
        {SDOAbortCode::GeneralError,          "General error"},
        {SDOAbortCode::TransferAborted,       "Data transfer aborted"},
        {SDOAbortCode::LocalControlError,     "Local control error"},
        {SDOAbortCode::DeviceStateError,      "Wrong device state"},
        {SDOAbortCode::DictionaryNotPresent,  "Object dictionary not present"},
        {SDOAbortCode::NoDataAvailable,       "No data available"},
    };
    for (const auto& c : cases) {
        EXPECT_STREQ(sdoAbortCodeStr(c.code), c.text)
            << "code 0x" << std::hex << static_cast<uint32_t>(c.code);
    }
}

TEST(SdoAbortCodes, UnknownCodeFallsBack) {
    EXPECT_STREQ(sdoAbortCodeStr(static_cast<SDOAbortCode>(0x0BADF00D)),
                 "Unknown abort code");
}

TEST(SdoAbortCodes, RawUint32Overload) {
    EXPECT_STREQ(sdoAbortCodeStr(0x06020000u), "Object does not exist");
    EXPECT_STREQ(sdoAbortCodeStr(0xDEADBEEFu), "Unknown abort code");
}

// ============================================================================
// ecdc_* weak time-source defaults (host implementation)
// ============================================================================

TEST(DcTimeSource, InitGetDeinit) {
    EXPECT_TRUE(DC::ecdc_init_time_source());
    const uint64_t t1 = DC::ecdc_get_master_time_ns();
    const uint64_t t2 = DC::ecdc_get_master_time_ns();
    EXPECT_GT(t1, 0u);
    EXPECT_GE(t2, t1);                  // CLOCK_MONOTONIC never goes backwards
    DC::ecdc_deinit_time_source();
}
