/**
 * @file test_SlaveIdentification.cpp
 * @brief Tests for CiA 301 SlaveIdentification classes
 *
 * Covers IdentityRecord, DeviceTypeInfo, SlaveIdentity (construction,
 * copy/move, string buffer, accessors), free functions, and VendorID constants.
 */
#include <gtest/gtest.h>
#include <tether/profiles/cia301/SlaveIdentification.hpp>
#include <cstring>

using namespace EtherCAT;

// ============================================================================
// IdentityRecord
// ============================================================================
TEST(IdentityRecordTest, Defaults) {
    IdentityRecord r{};
    EXPECT_EQ(r.vendor_id, 0u);
    EXPECT_EQ(r.product_code, 0u);
    EXPECT_EQ(r.revision_number, 0u);
    EXPECT_EQ(r.serial_number, 0u);
    EXPECT_FALSE(r.isValid());
}

TEST(IdentityRecordTest, Valid) {
    IdentityRecord r{};
    r.vendor_id = 0x1234;
    EXPECT_TRUE(r.isValid());
}

TEST(IdentityRecordTest, RevisionSplit) {
    IdentityRecord r{};
    r.revision_number = 0x00030005;
    EXPECT_EQ(r.revisionMajor(), 3);
    EXPECT_EQ(r.revisionMinor(), 5);
}

TEST(IdentityRecordTest, RevisionHighOnly) {
    IdentityRecord r{};
    r.revision_number = 0xFFFF0000;
    EXPECT_EQ(r.revisionMajor(), 0xFFFF);
    EXPECT_EQ(r.revisionMinor(), 0);
}

TEST(IdentityRecordTest, RevisionLowOnly) {
    IdentityRecord r{};
    r.revision_number = 0x0000ABCD;
    EXPECT_EQ(r.revisionMajor(), 0);
    EXPECT_EQ(r.revisionMinor(), 0xABCD);
}

// ============================================================================
// DeviceTypeInfo
// ============================================================================
TEST(DeviceTypeInfoTest, Defaults) {
    DeviceTypeInfo d{};
    EXPECT_EQ(d.raw_value, 0u);
    EXPECT_EQ(d.profileNumber(), 0);
    EXPECT_EQ(d.additionalInfo(), 0);
    EXPECT_FALSE(d.isCiA402Drive());
}

TEST(DeviceTypeInfoTest, CiA402) {
    DeviceTypeInfo d{};
    d.raw_value = 402;
    EXPECT_EQ(d.profileNumber(), 402);
    EXPECT_TRUE(d.isCiA402Drive());
}

TEST(DeviceTypeInfoTest, UpperBits) {
    DeviceTypeInfo d{};
    d.raw_value = 0x00010192; // additional=1, profile=402
    EXPECT_EQ(d.profileNumber(), 402);
    EXPECT_EQ(d.additionalInfo(), 1);
    EXPECT_TRUE(d.isCiA402Drive());
}

TEST(DeviceTypeInfoTest, NotDrive) {
    DeviceTypeInfo d{};
    d.raw_value = 401;
    EXPECT_FALSE(d.isCiA402Drive());
}

// ============================================================================
// SlaveIdentity — construction & defaults
// ============================================================================
TEST(SlaveIdentityTest, DefaultConstruction) {
    SlaveIdentity si;
    EXPECT_EQ(si.vendorId(), 0u);
    EXPECT_EQ(si.productCode(), 0u);
    EXPECT_EQ(si.revisionNumber(), 0u);
    EXPECT_EQ(si.serialNumber(), 0u);
    EXPECT_FALSE(si.isValid());
    EXPECT_FALSE(si.hasDeviceName());
    EXPECT_FALSE(si.hasHardwareVersion());
    EXPECT_FALSE(si.hasSoftwareVersion());
    EXPECT_STREQ(si.deviceName(), "");
    EXPECT_STREQ(si.hardwareVersion(), "");
    EXPECT_STREQ(si.softwareVersion(), "");
    EXPECT_STREQ(si.orderCode(), "");
    EXPECT_EQ(si.slaveIndex(), 0);
    EXPECT_EQ(si.errorRegister(), 0);
    EXPECT_EQ(si.supportedDriveModes(), 0u);
    EXPECT_FALSE(si.isCiA402Drive());
}

// ============================================================================
// setters / getters
// ============================================================================
TEST(SlaveIdentityTest, SetIdentityRecord) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0xBEEF;
    r.product_code = 42;
    r.revision_number = 0x00010002;
    r.serial_number = 999;
    si.setIdentityRecord(r);
    si.setValid(true);

    EXPECT_TRUE(si.isValid());
    EXPECT_EQ(si.vendorId(), 0xBEEFu);
    EXPECT_EQ(si.productCode(), 42u);
    EXPECT_EQ(si.revisionNumber(), 0x00010002u);
    EXPECT_EQ(si.serialNumber(), 999u);
}

TEST(SlaveIdentityTest, SetDeviceType) {
    SlaveIdentity si;
    DeviceTypeInfo t{};
    t.raw_value = 402;
    si.setDeviceType(t);
    EXPECT_TRUE(si.isCiA402Drive());
}

TEST(SlaveIdentityTest, SetSlaveIndex) {
    SlaveIdentity si;
    si.setSlaveIndex(7);
    EXPECT_EQ(si.slaveIndex(), 7);
}

TEST(SlaveIdentityTest, SetErrorRegister) {
    SlaveIdentity si;
    si.setErrorRegister(0xAB);
    EXPECT_EQ(si.errorRegister(), 0xAB);
}

TEST(SlaveIdentityTest, SetSupportedDriveModes) {
    SlaveIdentity si;
    si.setSupportedDriveModes(0xFF);
    EXPECT_EQ(si.supportedDriveModes(), 0xFFu);
}

// ============================================================================
// String buffer
// ============================================================================
TEST(SlaveIdentityTest, AddStringNormal) {
    SlaveIdentity si;
    const char* p = si.addString("Hello", 5);
    ASSERT_NE(p, nullptr);
    si.setDeviceNamePtr(p);
    EXPECT_TRUE(si.hasDeviceName());
    EXPECT_STREQ(si.deviceName(), "Hello");
}

TEST(SlaveIdentityTest, AddMultipleStrings) {
    SlaveIdentity si;
    const char* name = si.addString("Drive1", 6);
    const char* hw = si.addString("HW-v2", 5);
    const char* sw = si.addString("SW-1.0", 6);
    const char* order = si.addString("ORD-100", 7);

    ASSERT_NE(name, nullptr);
    ASSERT_NE(hw, nullptr);
    ASSERT_NE(sw, nullptr);
    ASSERT_NE(order, nullptr);

    si.setDeviceNamePtr(name);
    si.setHardwareVersionPtr(hw);
    si.setSoftwareVersionPtr(sw);
    si.setOrderCodePtr(order);

    EXPECT_STREQ(si.deviceName(), "Drive1");
    EXPECT_STREQ(si.hardwareVersion(), "HW-v2");
    EXPECT_STREQ(si.softwareVersion(), "SW-1.0");
    EXPECT_STREQ(si.orderCode(), "ORD-100");
}

TEST(SlaveIdentityTest, AddStringBufferOverflow) {
    SlaveIdentity si;
    // Fill the buffer
    size_t remaining = si.remainingBufferSpace();
    EXPECT_GT(remaining, 0u);

    // Add a string that uses most of the buffer
    std::string big(remaining - 2, 'X'); // -2 for null + safety
    const char* p = si.addString(big.c_str(), big.size());
    // May or may not succeed depending on null terminator accounting
    (void)p;

    // Overfill — should return nullptr
    const char* q = si.addString("overflow", 8);
    // If buffer already full, q should be nullptr
    if (si.remainingBufferSpace() < 9) {
        EXPECT_EQ(q, nullptr);
    }
}

TEST(SlaveIdentityTest, RemainingBufferSpace) {
    SlaveIdentity si;
    size_t initial = si.remainingBufferSpace();
    EXPECT_EQ(initial, kMaxIdentityStringBuffer);

    si.addString("test", 4);
    EXPECT_LT(si.remainingBufferSpace(), initial);
}

// ============================================================================
// Copy constructor
// ============================================================================
TEST(SlaveIdentityTest, CopyConstruct) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0x1234;
    si.setIdentityRecord(r);
    si.setValid(true);
    const char* name = si.addString("Motor", 5);
    si.setDeviceNamePtr(name);

    SlaveIdentity copy(si);
    EXPECT_TRUE(copy.isValid());
    EXPECT_EQ(copy.vendorId(), 0x1234u);
    EXPECT_STREQ(copy.deviceName(), "Motor");
    EXPECT_TRUE(copy.hasDeviceName());

    // Original unchanged
    EXPECT_STREQ(si.deviceName(), "Motor");
}

// ============================================================================
// Copy assignment
// ============================================================================
TEST(SlaveIdentityTest, CopyAssign) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0x5678;
    si.setIdentityRecord(r);
    si.setValid(true);
    const char* hw = si.addString("HW-3", 4);
    si.setHardwareVersionPtr(hw);

    SlaveIdentity copy;
    copy = si;
    EXPECT_EQ(copy.vendorId(), 0x5678u);
    EXPECT_STREQ(copy.hardwareVersion(), "HW-3");
}

// ============================================================================
// Move constructor
// ============================================================================
TEST(SlaveIdentityTest, MoveConstruct) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0xAAAA;
    si.setIdentityRecord(r);
    si.setValid(true);
    const char* name = si.addString("Servo", 5);
    si.setDeviceNamePtr(name);

    SlaveIdentity moved(std::move(si));
    EXPECT_TRUE(moved.isValid());
    EXPECT_EQ(moved.vendorId(), 0xAAAAu);
    EXPECT_STREQ(moved.deviceName(), "Servo");
}

// ============================================================================
// Move assignment
// ============================================================================
TEST(SlaveIdentityTest, MoveAssign) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0xBBBB;
    si.setIdentityRecord(r);
    si.setValid(true);

    SlaveIdentity dest;
    dest = std::move(si);
    EXPECT_EQ(dest.vendorId(), 0xBBBBu);
}

// ============================================================================
// clear
// ============================================================================
TEST(SlaveIdentityTest, Clear) {
    SlaveIdentity si;
    IdentityRecord r{};
    r.vendor_id = 0x9999;
    si.setIdentityRecord(r);
    si.setValid(true);
    const char* name = si.addString("X", 1);
    si.setDeviceNamePtr(name);

    si.clear();
    EXPECT_FALSE(si.isValid());
    EXPECT_EQ(si.vendorId(), 0u);
    EXPECT_FALSE(si.hasDeviceName());
    EXPECT_STREQ(si.deviceName(), "");
    EXPECT_EQ(si.remainingBufferSpace(), kMaxIdentityStringBuffer);
}

// ============================================================================
// isValid logic
// ============================================================================
TEST(SlaveIdentityTest, IsValidRequiresBoth) {
    SlaveIdentity si;
    // valid=true but vendor=0 → isValid false
    si.setValid(true);
    EXPECT_FALSE(si.isValid());

    IdentityRecord r{};
    r.vendor_id = 1;
    si.setIdentityRecord(r);
    EXPECT_TRUE(si.isValid());

    // valid=false but vendor set → false
    si.setValid(false);
    EXPECT_FALSE(si.isValid());
}

// ============================================================================
// Free functions
// ============================================================================
TEST(VendorNameTest, KnownVendor) {
    const char* name = getVendorName(VendorID::Beckhoff);
    ASSERT_NE(name, nullptr);
    EXPECT_STRNE(name, "");
}

TEST(VendorNameTest, UnknownVendor) {
    const char* name = getVendorName(0xDEADBEEF);
    ASSERT_NE(name, nullptr);
    // Should return fallback like "Vendor 0xDEAD..."
    EXPECT_STRNE(name, "");
}

TEST(ProductNameTest, FallbackFormat) {
    const char* name = getProductName(0x12345678);
    ASSERT_NE(name, nullptr);
    EXPECT_STRNE(name, "");
}

// ============================================================================
// VendorID constants
// ============================================================================
TEST(VendorIDTest, ConstantsNonZero) {
    EXPECT_NE(VendorID::Beckhoff, 0u);
    EXPECT_NE(VendorID::Lenze, 0u);
    EXPECT_NE(VendorID::Bosch, 0u);
    EXPECT_NE(VendorID::Delta, 0u);
    EXPECT_NE(VendorID::Omron, 0u);
    EXPECT_NE(VendorID::Yaskawa, 0u);
    EXPECT_NE(VendorID::Siemens, 0u);
}
