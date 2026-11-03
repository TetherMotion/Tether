/**
 * @file test_esi_parser_synthetic.cpp
 * @brief Coverage for the ESI XML parser (src/ethercat/ESIParser.cpp)
 *        using synthetic XML documents written to /tmp — vendor block,
 *        device/Type attributes, mailbox timeouts, SM entries, FMMUs,
 *        RxPDO/TxPDO entries, plus error paths and both formatters.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include "tether/ethercat/ESIParser.hpp"

using namespace EtherCAT;

namespace {

std::string tmpXml(const std::string& name, const std::string& body) {
    std::string path = "/tmp/tether_esi_" + name + "_" +
                       std::to_string(getpid()) + ".xml";
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
        <Info>
          <Mailbox>
            <Timeout>
              <RequestTimeout>500</RequestTimeout>
              <ResponseTimeout>5000</ResponseTimeout>
            </Timeout>
          </Mailbox>
        </Info>
        <Sm StartAddress="#x1000" ControlByte="#x26" Enable="1" DefaultSize="128">MBoxOut</Sm>
        <Sm StartAddress="#x1080" ControlByte="#x22" Enable="1" DefaultSize="128">MBoxIn</Sm>
        <Sm StartAddress="#x1100" ControlByte="#x44" Enable="1" DefaultSize="12">Outputs</Sm>
        <Sm StartAddress="#x1180" ControlByte="#x40" Enable="1" DefaultSize="12">Inputs</Sm>
        <Fmmu>Outputs</Fmmu>
        <Fmmu>Inputs</Fmmu>
        <RxPdo Sm="2" Fixed="1">
          <Index>#x1600</Index>
          <Name>RxPDO1</Name>
          <Entry>
            <Index>#x6040</Index>
            <SubIndex>0</SubIndex>
            <BitLen>16</BitLen>
            <Name>Controlword</Name>
            <DataType>UINT</DataType>
          </Entry>
          <Entry>
            <Index>#x607A</Index>
            <SubIndex>0</SubIndex>
            <BitLen>32</BitLen>
            <Name>Target position</Name>
            <DataType>DINT</DataType>
          </Entry>
        </RxPdo>
        <TxPdo Sm="3" Fixed="1">
          <Index>#x1A00</Index>
          <Name>TxPDO1</Name>
          <Entry>
            <Index>#x6041</Index>
            <SubIndex>0</SubIndex>
            <BitLen>16</BitLen>
            <Name>Statusword</Name>
            <DataType>UINT</DataType>
          </Entry>
        </TxPdo>
      </Device>
    </Devices>
  </Descriptions>
</EtherCATInfo>
)XML";

TEST(ESIParserSynthetic, ParsesCompleteDevice) {
    auto path = tmpXml("full", kDeviceXml);
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    ASSERT_TRUE(ESI::parseESIFile(path, devices, err)) << err;
    std::remove(path.c_str());

    ASSERT_EQ(devices.size(), 1u);
    const auto& d = devices[0];
    EXPECT_EQ(d.name, "A6 Servo");
    EXPECT_EQ(d.type, "TestDrive");
    EXPECT_EQ(d.comment, "CiA402 drive");
    EXPECT_EQ(d.productCode, 0x1234u);
    EXPECT_EQ(d.revision, 0x0002u);
    EXPECT_EQ(d.vendorId, 0x9Eu);

    ASSERT_TRUE(d.mailbox_request_timeout_ms.has_value());
    EXPECT_EQ(*d.mailbox_request_timeout_ms, 500u);
    ASSERT_TRUE(d.mailbox_response_timeout_ms.has_value());
    EXPECT_EQ(*d.mailbox_response_timeout_ms, 5000u);

    ASSERT_EQ(d.syncManagers.size(), 4u);
    EXPECT_EQ(d.syncManagers[0].startAddress, 0x1000);
    EXPECT_EQ(d.syncManagers[0].defaultSize, 128);
    EXPECT_EQ(d.syncManagers[2].name, "Outputs");

    ASSERT_EQ(d.fmmus.size(), 2u);
    EXPECT_EQ(d.fmmus[0], "Outputs");

    ASSERT_EQ(d.rxPdos.size(), 1u);
    EXPECT_EQ(d.rxPdos[0].index, 0x1600);
    EXPECT_TRUE(d.rxPdos[0].fixed);
    EXPECT_EQ(d.rxPdos[0].sm, 2);
    ASSERT_EQ(d.rxPdos[0].entries.size(), 2u);
    EXPECT_EQ(d.rxPdos[0].entries[0].index, 0x6040);
    EXPECT_EQ(d.rxPdos[0].entries[1].bitLen, 32);

    ASSERT_EQ(d.txPdos.size(), 1u);
    EXPECT_EQ(d.txPdos[0].index, 0x1A00);
    EXPECT_EQ(d.txPdos[0].entries[0].index, 0x6041);
}

TEST(ESIParserSynthetic, MultipleDevicesParsed) {
    auto path = tmpXml("multi", R"XML(<?xml version="1.0"?>
<EtherCATInfo>
  <Descriptions><Devices>
    <Device><Type ProductCode="#x1">A</Type><Name>DevA</Name></Device>
    <Device><Type ProductCode="#x2">B</Type><Name>DevB</Name></Device>
  </Devices></Descriptions>
</EtherCATInfo>)XML");
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    ASSERT_TRUE(ESI::parseESIFile(path, devices, err)) << err;
    std::remove(path.c_str());
    ASSERT_EQ(devices.size(), 2u);
    EXPECT_EQ(devices[0].name, "DevA");
    EXPECT_EQ(devices[1].productCode, 0x2u);
}

TEST(ESIParserSynthetic, MissingFileFails) {
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    EXPECT_FALSE(ESI::parseESIFile("/tmp/definitely_missing_esi.xml",
                                   devices, err));
    EXPECT_FALSE(err.empty());
}

TEST(ESIParserSynthetic, MalformedXmlFails) {
    auto path = tmpXml("bad", "this is <not> well-formed <<< xml");
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    EXPECT_FALSE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());
    EXPECT_FALSE(err.empty());
}

TEST(ESIParserSynthetic, MissingDevicesElementFails) {
    auto path = tmpXml("nodev", R"XML(<?xml version="1.0"?>
<EtherCATInfo><Vendor><Id>#x1</Id></Vendor></EtherCATInfo>)XML");
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    EXPECT_FALSE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());
    EXPECT_NE(err.find("Devices"), std::string::npos);
}

TEST(ESIParserSynthetic, EmptyDeviceListSucceeds) {
    auto path = tmpXml("empty", R"XML(<?xml version="1.0"?>
<EtherCATInfo><Descriptions><Devices/></Descriptions></EtherCATInfo>)XML");
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    EXPECT_TRUE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());
    EXPECT_TRUE(devices.empty());
}

TEST(ESIParserSynthetic, FormatDeviceJSONEmitsAllSections) {
    auto path = tmpXml("json", kDeviceXml);
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    ASSERT_TRUE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());

    const std::string j = ESI::formatDeviceJSON(devices[0]);
    EXPECT_NE(j.find("\"rxPdos\""), std::string::npos);
    EXPECT_NE(j.find("\"txPdos\""), std::string::npos);
    EXPECT_NE(j.find("syncManagers"), std::string::npos);
    EXPECT_NE(j.find("0x1600"), std::string::npos);
}

TEST(ESIParserSynthetic, FormatDeviceHumanReadableEmitsText) {
    auto path = tmpXml("hr", kDeviceXml);
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    ASSERT_TRUE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());

    const std::string full = ESI::formatDeviceHumanReadable(devices[0], false);
    EXPECT_NE(full.find("A6 Servo"), std::string::npos);
    const std::string mbox = ESI::formatDeviceHumanReadable(devices[0], true);
    EXPECT_FALSE(mbox.empty());
}

TEST(ESIParserSynthetic, MissingOptionalFieldsTolerated) {
    auto path = tmpXml("sparse", R"XML(<?xml version="1.0"?>
<EtherCATInfo>
  <Descriptions><Devices>
    <Device>
      <Type>Minimal</Type>
      <Name>MinDev</Name>
    </Device>
  </Devices></Descriptions>
</EtherCATInfo>)XML");
    std::vector<ESI::DeviceInfo> devices;
    std::string err;
    ASSERT_TRUE(ESI::parseESIFile(path, devices, err));
    std::remove(path.c_str());
    ASSERT_EQ(devices.size(), 1u);
    EXPECT_EQ(devices[0].name, "MinDev");
    EXPECT_EQ(devices[0].productCode, 0u);
    EXPECT_FALSE(devices[0].mailbox_request_timeout_ms.has_value());
    EXPECT_TRUE(devices[0].syncManagers.empty());
    EXPECT_TRUE(devices[0].rxPdos.empty());
}

} // namespace
