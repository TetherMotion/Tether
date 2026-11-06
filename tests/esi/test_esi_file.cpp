/**
 * @file test_esi_file.cpp
 * @brief Coverage for EtherCAT::ESIFile — path constructor (success,
 *        parse failure, empty-device XML), findDevice matching rules.
 */

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tether/ethercat/ESIFile.hpp"

using namespace EtherCAT;

namespace {

std::string tmpXml(const std::string& name, const std::string& body) {
    std::string path = "/tmp/tether_esifile_" + name + "_" +
                       std::to_string(::getpid()) + ".xml";
    std::ofstream f(path);
    f << body;
    return path;
}

const char* kDeviceXml = R"XML(<?xml version="1.0"?>
<EtherCATInfo>
  <Vendor>
    <Id>#x0000009E</Id>
    <Name>TestVendor</Name>
  </Vendor>
  <Descriptions>
    <Devices>
      <Device>
        <Type ProductCode="#x1234" RevisionNo="#x0002">TestDrive</Type>
        <Name>A6 Servo</Name>
        <Comment>CiA402 drive</Comment>
        <Sm StartAddress="#x1000" ControlByte="#x26" Enable="1" DefaultSize="128">MBoxOut</Sm>
        <Sm StartAddress="#x1080" ControlByte="#x22" Enable="1" DefaultSize="128">MBoxIn</Sm>
        <Sm StartAddress="#x1100" ControlByte="#x44" Enable="1" DefaultSize="12">Outputs</Sm>
        <Sm StartAddress="#x1180" ControlByte="#x40" Enable="1" DefaultSize="12">Inputs</Sm>
        <Fmmu>Outputs</Fmmu>
        <Fmmu>Inputs</Fmmu>
      </Device>
    </Devices>
  </Descriptions>
</EtherCATInfo>
)XML";

const char* kNoDevicesXml = R"XML(<?xml version="1.0"?>
<EtherCATInfo>
  <Vendor>
    <Id>#x0000009E</Id>
    <Name>TestVendor</Name>
  </Vendor>
  <Descriptions>
    <Devices>
    </Devices>
  </Descriptions>
</EtherCATInfo>
)XML";

// ============================================================================
// Pre-parsed vector constructor + findDevice
// ============================================================================

TEST(ESIFileCov, VectorConstructor) {
    std::vector<ESI::DeviceInfo> devs(2);
    devs[0].vendorId = 0x9E;
    devs[0].productCode = 0x1234;
    devs[1].vendorId = 0xAA;
    devs[1].productCode = 0x5678;
    ESIFile f(devs);
    EXPECT_FALSE(f.empty());
    EXPECT_TRUE(f.errorMessage().empty());
    EXPECT_EQ(f.devices().size(), 2u);
}

TEST(ESIFileCov, EmptyVectorFindDeviceReturnsNull) {
    ESIFile f(std::vector<ESI::DeviceInfo>{});
    EXPECT_TRUE(f.empty());
    EXPECT_EQ(f.findDevice(0x9E, 0x1234), nullptr);
    EXPECT_EQ(f.findDevice(0, 0), nullptr);
}

TEST(ESIFileCov, FindDeviceMatching) {
    std::vector<ESI::DeviceInfo> devs(2);
    devs[0].vendorId = 0x9E;
    devs[0].productCode = 0x1234;
    devs[1].vendorId = 0xAA;
    devs[1].productCode = 0x5678;
    ESIFile f(devs);

    // Exact match → that device
    EXPECT_EQ(f.findDevice(0x9E, 0x1234), &f.devices()[0]);
    EXPECT_EQ(f.findDevice(0xAA, 0x5678), &f.devices()[1]);
    // Wildcard vendor (0) + exact product
    EXPECT_EQ(f.findDevice(0, 0x5678), &f.devices()[1]);
    // Exact vendor + wildcard product
    EXPECT_EQ(f.findDevice(0xAA, 0), &f.devices()[1]);
    // No match → falls back to devices[0]
    EXPECT_EQ(f.findDevice(0xFF, 0xFFFF), &f.devices()[0]);
}

// ============================================================================
// Path constructor (requires tether_esi / TETHER_HAVE_ESI)
// ============================================================================

TEST(ESIFileCov, PathCtorParsesDevices) {
    std::string p = tmpXml("dev", kDeviceXml);
    ESIFile f(p);
    ::unlink(p.c_str());
    EXPECT_TRUE(f.errorMessage().empty());
    EXPECT_FALSE(f.empty());
    EXPECT_GE(f.devices().size(), 1u);
    // The parsed device is findable by its product code
    EXPECT_NE(f.findDevice(0, 0x1234), nullptr);
}

TEST(ESIFileCov, PathCtorMissingFileSetsError) {
    ESIFile f("/tmp/definitely_no_such_esi_file_zzz.xml");
    EXPECT_TRUE(f.empty());
    EXPECT_FALSE(f.errorMessage().empty());
    EXPECT_NE(f.errorMessage().find("Failed to parse"), std::string::npos);
}

TEST(ESIFileCov, PathCtorNoDevicesSetsError) {
    std::string p = tmpXml("nodev", kNoDevicesXml);
    ESIFile f(p);
    ::unlink(p.c_str());
    EXPECT_TRUE(f.empty());
    EXPECT_FALSE(f.errorMessage().empty());
    EXPECT_NE(f.errorMessage().find("no <Device>"), std::string::npos);
}

}  // namespace
