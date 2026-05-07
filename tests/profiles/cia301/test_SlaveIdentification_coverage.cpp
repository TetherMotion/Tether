/**
 * @file test_SlaveIdentification_coverage.cpp
 * @brief Deep coverage tests for SlaveIdentity, SlaveIdentifier structs and utilities
 */

#include "tether/profiles/cia301/SlaveIdentification.hpp"
#include <gtest/gtest.h>
#include <cstring>

using namespace EtherCAT;

// ============================================================================
// IdentityRecord Tests
// ============================================================================

TEST(IdentityRecordCovTest, DefaultValues) {
    IdentityRecord r;
    EXPECT_EQ(r.vendor_id, 0u);
    EXPECT_EQ(r.product_code, 0u);
    EXPECT_EQ(r.revision_number, 0u);
    EXPECT_EQ(r.serial_number, 0u);
}

TEST(IdentityRecordCovTest, IsValid) {
    IdentityRecord r;
    EXPECT_FALSE(r.isValid());
    r.vendor_id = 0x00000002; // Beckhoff
    EXPECT_TRUE(r.isValid());
}

TEST(IdentityRecordCovTest, RevisionMajorMinor) {
    IdentityRecord r;
    r.revision_number = 0x00050003; // Major=5, Minor=3
    EXPECT_EQ(r.revisionMajor(), 5);
    EXPECT_EQ(r.revisionMinor(), 3);
}

TEST(IdentityRecordCovTest, RevisionMajorMinor_ZeroPacked) {
    IdentityRecord r;
    r.revision_number = 0x00000000;
    EXPECT_EQ(r.revisionMajor(), 0);
    EXPECT_EQ(r.revisionMinor(), 0);
}

TEST(IdentityRecordCovTest, RevisionMajorMinor_MaxValues) {
    IdentityRecord r;
    r.revision_number = 0xFFFFFFFF;
    EXPECT_EQ(r.revisionMajor(), 0xFFFF);
    EXPECT_EQ(r.revisionMinor(), 0xFFFF);
}

// ============================================================================
// DeviceTypeInfo Tests
// ============================================================================

TEST(DeviceTypeInfoCovTest, DefaultValues) {
    DeviceTypeInfo d;
    EXPECT_EQ(d.raw_value, 0u);
}

TEST(DeviceTypeInfoCovTest, ProfileNumber) {
    DeviceTypeInfo d;
    d.raw_value = 402; // CiA 402
    EXPECT_EQ(d.profileNumber(), 402);
    EXPECT_TRUE(d.isCiA402Drive());
}

TEST(DeviceTypeInfoCovTest, NotCiA402) {
    DeviceTypeInfo d;
    d.raw_value = 401;
    EXPECT_EQ(d.profileNumber(), 401);
    EXPECT_FALSE(d.isCiA402Drive());
}

TEST(DeviceTypeInfoCovTest, AdditionalInfo) {
    DeviceTypeInfo d;
    d.raw_value = 0x00010192; // additionalInfo=1, profileNumber=0x0192=402
    EXPECT_EQ(d.profileNumber(), 402);
    EXPECT_EQ(d.additionalInfo(), 1);
}

TEST(DeviceTypeInfoCovTest, HighBits) {
    DeviceTypeInfo d;
    d.raw_value = 0xABCD0000;
    EXPECT_EQ(d.profileNumber(), 0);
    EXPECT_EQ(d.additionalInfo(), 0xABCD);
}

// ============================================================================
// SlaveIdentity Tests
// ============================================================================

TEST(SlaveIdentityCovTest, DefaultState) {
    SlaveIdentity id;
    EXPECT_EQ(id.vendorId(), 0u);
    EXPECT_EQ(id.productCode(), 0u);
    EXPECT_EQ(id.revisionNumber(), 0u);
    EXPECT_EQ(id.serialNumber(), 0u);
    EXPECT_FALSE(id.isValid());
    EXPECT_FALSE(id.isCiA402Drive());
    EXPECT_EQ(id.slaveIndex(), 0);
    EXPECT_EQ(id.errorRegister(), 0);
    EXPECT_EQ(id.supportedDriveModes(), 0u);
}

TEST(SlaveIdentityCovTest, SetGetIdentityRecord) {
    SlaveIdentity id;
    IdentityRecord r;
    r.vendor_id = 0x00000002; // Beckhoff
    r.product_code = 0x04440000;
    r.revision_number = 0x00010002;
    r.serial_number = 0x12345678;
    id.setIdentityRecord(r);
    
    EXPECT_EQ(id.vendorId(), 0x00000002u);
    EXPECT_EQ(id.productCode(), 0x04440000u);
    EXPECT_EQ(id.revisionNumber(), 0x00010002u);
    EXPECT_EQ(id.serialNumber(), 0x12345678u);
    
    auto& rec = id.identityRecord();
    EXPECT_EQ(rec.vendor_id, 0x00000002u);
}

TEST(SlaveIdentityCovTest, SetGetDeviceType) {
    SlaveIdentity id;
    DeviceTypeInfo dt;
    dt.raw_value = 402;
    id.setDeviceType(dt);
    
    EXPECT_TRUE(id.isCiA402Drive());
    EXPECT_EQ(id.deviceType().profileNumber(), 402);
}

TEST(SlaveIdentityCovTest, StringHandling) {
    SlaveIdentity id;
    const char* name = "Test Device";
    const char* ptr = id.addString(name, strlen(name));
    EXPECT_NE(ptr, nullptr);
    id.setDeviceNamePtr(ptr);
    EXPECT_TRUE(id.hasDeviceName());
    EXPECT_STREQ(id.deviceName(), "Test Device");
}

TEST(SlaveIdentityCovTest, HardwareVersion) {
    SlaveIdentity id;
    const char* hw = "HW 1.0";
    const char* ptr = id.addString(hw, strlen(hw));
    id.setHardwareVersionPtr(ptr);
    EXPECT_TRUE(id.hasHardwareVersion());
    EXPECT_STREQ(id.hardwareVersion(), "HW 1.0");
}

TEST(SlaveIdentityCovTest, SoftwareVersion) {
    SlaveIdentity id;
    const char* sw = "FW 2.3.0";
    const char* ptr = id.addString(sw, strlen(sw));
    id.setSoftwareVersionPtr(ptr);
    EXPECT_TRUE(id.hasSoftwareVersion());
    EXPECT_STREQ(id.softwareVersion(), "FW 2.3.0");
}

TEST(SlaveIdentityCovTest, OrderCode) {
    SlaveIdentity id;
    const char* oc = "ORD-12345";
    const char* ptr = id.addString(oc, strlen(oc));
    id.setOrderCodePtr(ptr);
    EXPECT_STREQ(id.orderCode(), "ORD-12345");
}

TEST(SlaveIdentityCovTest, NoStrings) {
    SlaveIdentity id;
    EXPECT_FALSE(id.hasDeviceName());
    EXPECT_FALSE(id.hasHardwareVersion());
    EXPECT_FALSE(id.hasSoftwareVersion());
    // deviceName() etc. should return empty or nullptr
    EXPECT_TRUE(id.deviceName() == nullptr || strlen(id.deviceName()) == 0);
}

TEST(SlaveIdentityCovTest, SetSlaveIndex) {
    SlaveIdentity id;
    id.setSlaveIndex(7);
    EXPECT_EQ(id.slaveIndex(), 7);
}

TEST(SlaveIdentityCovTest, SetErrorRegister) {
    SlaveIdentity id;
    id.setErrorRegister(0x42);
    EXPECT_EQ(id.errorRegister(), 0x42);
}

TEST(SlaveIdentityCovTest, SetSupportedDriveModes) {
    SlaveIdentity id;
    id.setSupportedDriveModes(0x000001FF);
    EXPECT_EQ(id.supportedDriveModes(), 0x000001FFu);
}

TEST(SlaveIdentityCovTest, SetValid) {
    SlaveIdentity id;
    EXPECT_FALSE(id.isValid());
    // isValid() requires BOTH m_valid AND vendor_id != 0
    IdentityRecord r;
    r.vendor_id = 0x00000002;
    id.setIdentityRecord(r);
    id.setValid(true);
    EXPECT_TRUE(id.isValid());
    id.setValid(false);
    EXPECT_FALSE(id.isValid());
}

TEST(SlaveIdentityCovTest, Clear) {
    SlaveIdentity id;
    IdentityRecord r;
    r.vendor_id = 0x00000002;
    id.setIdentityRecord(r);
    id.setValid(true);
    id.setSlaveIndex(5);
    
    id.clear();
    EXPECT_EQ(id.vendorId(), 0u);
    EXPECT_FALSE(id.isValid());
    EXPECT_EQ(id.slaveIndex(), 0);
}

TEST(SlaveIdentityCovTest, RemainingBufferSpace) {
    SlaveIdentity id;
    size_t initial = id.remainingBufferSpace();
    EXPECT_GT(initial, 0u);
    
    // Add a string
    const char* str = "Hello";
    id.addString(str, strlen(str));
    EXPECT_LT(id.remainingBufferSpace(), initial);
}

TEST(SlaveIdentityCovTest, AddMultipleStrings) {
    SlaveIdentity id;
    const char* s1 = "Device Name";
    const char* s2 = "HW Rev 1.0";
    const char* s3 = "FW Rev 2.0";
    
    const char* p1 = id.addString(s1, strlen(s1));
    const char* p2 = id.addString(s2, strlen(s2));
    const char* p3 = id.addString(s3, strlen(s3));
    
    EXPECT_NE(p1, nullptr);
    EXPECT_NE(p2, nullptr);
    EXPECT_NE(p3, nullptr);
    EXPECT_STREQ(p1, s1);
    EXPECT_STREQ(p2, s2);
    EXPECT_STREQ(p3, s3);
}

// ============================================================================
// SlaveIdentity Copy/Move
// ============================================================================

TEST(SlaveIdentityCovTest, CopyConstruction) {
    SlaveIdentity orig;
    IdentityRecord r;
    r.vendor_id = 0x00000002;
    r.product_code = 0x04440000;
    orig.setIdentityRecord(r);
    orig.setValid(true);
    orig.setSlaveIndex(3);
    
    const char* name = "TestDrive";
    const char* ptr = orig.addString(name, strlen(name));
    orig.setDeviceNamePtr(ptr);
    
    // Copy
    SlaveIdentity copy(orig);
    EXPECT_EQ(copy.vendorId(), 0x00000002u);
    EXPECT_EQ(copy.productCode(), 0x04440000u);
    EXPECT_TRUE(copy.isValid());
    EXPECT_EQ(copy.slaveIndex(), 3);
    EXPECT_TRUE(copy.hasDeviceName());
    EXPECT_STREQ(copy.deviceName(), "TestDrive");
}

TEST(SlaveIdentityCovTest, CopyAssignment) {
    SlaveIdentity orig;
    IdentityRecord r;
    r.vendor_id = 0x00000539; // Yaskawa
    orig.setIdentityRecord(r);
    const char* hw = "HW-1.0";
    orig.setHardwareVersionPtr(orig.addString(hw, strlen(hw)));
    orig.setValid(true);
    
    SlaveIdentity copy;
    copy = orig;
    EXPECT_EQ(copy.vendorId(), 0x00000539u);
    EXPECT_TRUE(copy.isValid());
    EXPECT_TRUE(copy.hasHardwareVersion());
}

TEST(SlaveIdentityCovTest, MoveConstruction) {
    SlaveIdentity orig;
    IdentityRecord r;
    r.vendor_id = 0x00000002;
    orig.setIdentityRecord(r);
    const char* sw = "SW-2.0";
    orig.setSoftwareVersionPtr(orig.addString(sw, strlen(sw)));
    orig.setValid(true);
    
    SlaveIdentity moved(std::move(orig));
    EXPECT_EQ(moved.vendorId(), 0x00000002u);
    EXPECT_TRUE(moved.isValid());
    EXPECT_TRUE(moved.hasSoftwareVersion());
}

TEST(SlaveIdentityCovTest, MoveAssignment) {
    SlaveIdentity orig;
    IdentityRecord r;
    r.vendor_id = 0x00000006; // ABB
    orig.setIdentityRecord(r);
    orig.setValid(true);
    
    SlaveIdentity moved;
    moved = std::move(orig);
    EXPECT_EQ(moved.vendorId(), 0x00000006u);
    EXPECT_TRUE(moved.isValid());
}

// ============================================================================
// VendorID constants
// ============================================================================

TEST(VendorIDCovTest, AllConstants) {
    EXPECT_EQ(VendorID::Beckhoff, 0x00000002u);
    EXPECT_EQ(VendorID::Lenze, 0x0000003Bu);
    EXPECT_EQ(VendorID::Bosch, 0x00000048u);
    EXPECT_EQ(VendorID::Kollmorgen, 0x0000006Au);
    EXPECT_EQ(VendorID::Delta, 0x000001DDu);
    EXPECT_EQ(VendorID::Omron, 0x00000083u);
    EXPECT_EQ(VendorID::Panasonic, 0x0000006Du);
    EXPECT_EQ(VendorID::Yaskawa, 0x00000539u);
    EXPECT_EQ(VendorID::Mitsubishi, 0x00000070u);
    EXPECT_EQ(VendorID::Siemens, 0x0000004Du);
    EXPECT_EQ(VendorID::ABB, 0x00000006u);
    EXPECT_EQ(VendorID::Schneider, 0x0000005Au);
    EXPECT_EQ(VendorID::Festo, 0x0000001Fu);
    EXPECT_EQ(VendorID::Copley, 0x000000ABu);
    EXPECT_EQ(VendorID::ElmoMotion, 0x0000009Au);
    EXPECT_EQ(VendorID::TechnosoftMotion, 0x000000C4u);
    EXPECT_EQ(VendorID::Nanotec, 0x0000026Cu);
    EXPECT_EQ(VendorID::Trinamic, 0x00000286u);
    EXPECT_EQ(VendorID::INOVANCE, 0x00100000u);
    EXPECT_EQ(VendorID::Leadshine, 0x000004D8u);
}

// ============================================================================
// getVendorName
// ============================================================================

TEST(VendorNameCovTest, KnownVendors) {
    EXPECT_NE(std::string(getVendorName(VendorID::Beckhoff)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Yaskawa)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Siemens)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Lenze)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Bosch)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Kollmorgen)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Delta)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Omron)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Panasonic)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Mitsubishi)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::ABB)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Schneider)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Festo)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Copley)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::ElmoMotion)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::TechnosoftMotion)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Nanotec)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Trinamic)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::INOVANCE)), std::string("Unknown"));
    EXPECT_NE(std::string(getVendorName(VendorID::Leadshine)), std::string("Unknown"));
}

TEST(VendorNameCovTest, UnknownVendor) {
    const char* name = getVendorName(0xDEADBEEF);
    EXPECT_NE(name, nullptr);
    // Unknown vendors should still return a non-null string
}

// ============================================================================
// getProductName
// ============================================================================

TEST(ProductNameCovTest, UnknownProduct) {
    const char* name = getProductName(0x00000000);
    EXPECT_NE(name, nullptr);
}

TEST(ProductNameCovTest, SomeProduct) {
    const char* name = getProductName(0x04440000);
    EXPECT_NE(name, nullptr);
}

// ============================================================================
// logSlaveIdentity (smoke test - just ensure no crash)
// ============================================================================

TEST(LogIdentityCovTest, LogSingle) {
    SlaveIdentity id;
    IdentityRecord r;
    r.vendor_id = VendorID::Beckhoff;
    r.product_code = 0x04440000;
    r.revision_number = 0x00010002;
    r.serial_number = 0x12345678;
    id.setIdentityRecord(r);
    id.setValid(true);
    id.setSlaveIndex(1);
    
    DeviceTypeInfo dt;
    dt.raw_value = 402;
    id.setDeviceType(dt);
    
    const char* name = "EL7031";
    id.setDeviceNamePtr(id.addString(name, strlen(name)));
    
    logSlaveIdentity(id, "TEST");
}

TEST(LogIdentityCovTest, LogSingle_NoStrings) {
    SlaveIdentity id;
    logSlaveIdentity(id, "EMPTY");
}

TEST(LogIdentityCovTest, LogTable) {
    SlaveIdentity ids[3];
    for (int i = 0; i < 3; i++) {
        IdentityRecord r;
        r.vendor_id = VendorID::Beckhoff;
        r.product_code = static_cast<uint32_t>(i + 1);
        ids[i].setIdentityRecord(r);
        ids[i].setValid(true);
        ids[i].setSlaveIndex(static_cast<uint16_t>(i));
    }
    logSlaveIdentityTable(ids, 3, "TABLE");
}

TEST(LogIdentityCovTest, LogTable_Empty) {
    logSlaveIdentityTable(nullptr, 0, "EMPTY_TABLE");
}

// ============================================================================
// Full SlaveIdentity with all fields set
// ============================================================================

TEST(SlaveIdentityCovTest, FullyPopulated) {
    SlaveIdentity id;
    
    // Identity record
    IdentityRecord r;
    r.vendor_id = VendorID::Yaskawa;
    r.product_code = 0x00001234;
    r.revision_number = 0x00020003;
    r.serial_number = 0xABCDEF01;
    id.setIdentityRecord(r);
    
    // Device type
    DeviceTypeInfo dt;
    dt.raw_value = 402;
    id.setDeviceType(dt);
    
    // Strings
    id.setDeviceNamePtr(id.addString("Sigma-7", 7));
    id.setHardwareVersionPtr(id.addString("HW 1.5", 6));
    id.setSoftwareVersionPtr(id.addString("FW 3.0.1", 8));
    id.setOrderCodePtr(id.addString("SGD7S-5R5A00A002", 16));
    
    // Other fields
    id.setSlaveIndex(2);
    id.setErrorRegister(0x10);
    id.setSupportedDriveModes(0x000003FF);
    id.setValid(true);
    
    // Verify everything
    EXPECT_EQ(id.vendorId(), VendorID::Yaskawa);
    EXPECT_EQ(id.productCode(), 0x00001234u);
    EXPECT_EQ(id.revisionNumber(), 0x00020003u);
    EXPECT_EQ(id.serialNumber(), 0xABCDEF01u);
    EXPECT_TRUE(id.isCiA402Drive());
    EXPECT_TRUE(id.hasDeviceName());
    EXPECT_TRUE(id.hasHardwareVersion());
    EXPECT_TRUE(id.hasSoftwareVersion());
    EXPECT_STREQ(id.deviceName(), "Sigma-7");
    EXPECT_EQ(id.slaveIndex(), 2);
    EXPECT_EQ(id.errorRegister(), 0x10);
    EXPECT_EQ(id.supportedDriveModes(), 0x000003FFu);
    EXPECT_TRUE(id.isValid());
    EXPECT_GT(id.remainingBufferSpace(), 0u);
    
    // Log it
    logSlaveIdentity(id, "FULL_TEST");
}
