#include "unity.h"
#include "TestRegistry.hpp"
#include "profiles/cia301/SlaveIdentification.hpp"
#include "MockSDOHelper.hpp"
#include <cstring>
#include <iostream>

using namespace EtherCAT;

// Test reading a normal string
static void test_read_normal_string(void) {
    MockSDO::clear();
    MockSDO::setString(CiA301::ManufacturerDeviceName, "MyDevice");

    // TODO: Provide a real or mock SDOManager instance for tests
    // SlaveIdentifier identifier(sdo);
    // char buffer[64];
    // size_t len = 0;
    // bool ok = identifier.readString(0, CiA301::ManufacturerDeviceName, buffer, sizeof(buffer), &len);
    (void)0; // placeholder
    
    TEST_ASSERT_TRUE_MESSAGE(ok, "Failed to read normal string");
    TEST_ASSERT_EQUAL_STRING("MyDevice", buffer);
    TEST_ASSERT_EQUAL_UINT32(8, len);
}

// Test reading a zero-length string
// This represents the bug reported by the user
static void test_read_zero_length_string(void) {
    MockSDO::clear();
    // Simulate empty string
    MockSDO::setString(CiA301::ManufacturerDeviceName, "");

    // TODO: Provide a real or mock SDOManager instance for tests
    // SlaveIdentifier identifier(sdo);
    // char buffer[64];
    // std::memset(buffer, 0xAA, sizeof(buffer));
    // size_t len = 0;
    // bool ok = identifier.readString(0, CiA301::ManufacturerDeviceName, buffer, sizeof(buffer), &len);
    (void)0; // placeholder
    
    // Current buggy behavior: returns false for empty string
    // Desired behavior: returns true, length 0
    
    TEST_ASSERT_TRUE_MESSAGE(ok, "Should succeed for zero-length visible string");
    TEST_ASSERT_EQUAL_UINT32(0, len);
    TEST_ASSERT_EQUAL_UINT8('\0', buffer[0]);
}

// Public registration function
static void __attribute__((constructor)) register_repro_tests(void) {
    TestRegistry::registerTest("test_read_normal_string", test_read_normal_string);
    TestRegistry::registerTest("test_read_zero_length_string", test_read_zero_length_string);
}
