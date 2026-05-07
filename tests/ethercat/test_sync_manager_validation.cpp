#include <gtest/gtest.h>
#include "tether/ethercat/SyncManagerValidation.hpp"

using namespace EtherCAT;
using namespace EtherCAT::PDO;

class SyncManagerValidationTest : public ::testing::Test {
protected:
    std::vector<SyncManagerConfig> getValidConfigs() {
        std::vector<SyncManagerConfig> configs(4);
        
        // SM0: start=0x1000 len=128 ctrl=0x26 (MbxIn: ECAT writes, master→slave)
        configs[0].phys_start_addr = 0x1000;
        configs[0].length = 128;
        configs[0].control = 0x26;
        configs[0].enable = true;
        
        // SM1: start=0x1080 len=128 ctrl=0x22 (MbxOut: ECAT reads, slave→master)
        configs[1].phys_start_addr = 0x1080;
        configs[1].length = 128;
        configs[1].control = 0x22;
        configs[1].enable = true;
        
        // SM2: start=0x1100 len=8 ctrl=0x60 (Buffered, ECAT reads = TxPDO/inputs)
        configs[2].phys_start_addr = 0x1100;
        configs[2].length = 8;
        configs[2].control = 0x60;
        configs[2].enable = true;
        
        // SM3: start=0x1400 len=32 ctrl=0x24 (Buffered, ECAT writes = RxPDO/outputs)
        configs[3].phys_start_addr = 0x1400;
        configs[3].length = 32;
        configs[3].control = 0x24;
        configs[3].enable = true;
        
        return configs;
    }
};

TEST_F(SyncManagerValidationTest, ValidConfigPasses) {
    auto configs = getValidConfigs();
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
    EXPECT_EQ(result.error_message, "");
}

TEST_F(SyncManagerValidationTest, DetectsOverlapSM0_SM1) {
    auto configs = getValidConfigs();
    // SM0 ends at 0x1080. SM1 starts at 0x1080. No overlap.
    // Extend SM0 to overlap
    configs[0].length = 129; 
    
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result.valid);
    EXPECT_NE(result.error_message.find("Overlap"), std::string::npos) << "Msg: " << result.error_message;
}

TEST_F(SyncManagerValidationTest, DetectsOverlapDisjoint) {
    auto configs = getValidConfigs();
    // Move SM1 into SM0 range
    configs[1].phys_start_addr = 0x1005; // Inside 0x1000 - 0x1080
    
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result.valid);
    EXPECT_NE(result.error_message.find("Overlap"), std::string::npos);
}

TEST_F(SyncManagerValidationTest, DetectsOverlapReversedOrder) {
    auto configs = getValidConfigs();
    // Move SM0 to be AFTER SM1 but overlapping
    // SM1: 0x1080 len=128 -> end 0x1100
    // SM0: 0x10F0
    configs[0].phys_start_addr = 0x10F0;
    
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result.valid);
    EXPECT_NE(result.error_message.find("Overlap"), std::string::npos);
}

TEST_F(SyncManagerValidationTest, ValidatesControlBytes) {
    auto configs = getValidConfigs();
    
    // Invalid SM0
    configs[0].control = 0x00;
    auto result0 = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result0.valid);
    EXPECT_NE(result0.error_message.find("SM0 invalid control"), std::string::npos);
    
    configs = getValidConfigs(); // Reset
    configs[1].control = 0xFF; // Invalid SM1
    auto result1 = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result1.valid);
    EXPECT_NE(result1.error_message.find("SM1 invalid control"), std::string::npos);

    configs = getValidConfigs();
    configs[2].control = 0x22; // Wrong: MAILBOX mode instead of BUFFERED for SM2
    auto result2 = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result2.valid);
    EXPECT_NE(result2.error_message.find("SM2 invalid mode"), std::string::npos);

    configs = getValidConfigs();
    configs[3].control = 0x26; // Wrong: MAILBOX mode instead of BUFFERED for SM3
    auto result3 = SyncManagerValidation::validate(configs);
    EXPECT_FALSE(result3.valid);
    EXPECT_NE(result3.error_message.find("SM3 invalid mode"), std::string::npos);
}

TEST_F(SyncManagerValidationTest, AcceptsDifferentAddressesIfNonOverlapping) {
    auto configs = getValidConfigs();
    // Shift all addresses up by 0x2000
    for(auto& cfg : configs) {
        cfg.phys_start_addr += 0x2000;
    }
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_TRUE(result.valid) << "Shifting addresses should be valid if types match";
}

TEST_F(SyncManagerValidationTest, AcceptsDifferentLengths) {
    auto configs = getValidConfigs();
    configs[0].length = 256;
    configs[1].phys_start_addr = 0x2000; // Move SM1 out of way
    // Validate
    auto result = SyncManagerValidation::validate(configs);
    EXPECT_TRUE(result.valid);
}

TEST_F(SyncManagerValidationTest, DisablesAreIgnored) {
    auto configs = getValidConfigs();
    // Make SM1 invalid but disabled
    configs[1].control = 0xFF;
    configs[1].enable = false;
    
    auto result = SyncManagerValidation::validate(configs);
    // Should pass if we only check enabled SMs? 
    // Implementation checks enabled SMs for control bytes.
    EXPECT_TRUE(result.valid);

    // Make SM1 overlap SM0 but disabled
    configs[1].control = 0x22; // restore valid control
    configs[1].phys_start_addr = 0x1000; // Overlap SM0
    configs[1].enable = false;
    
    auto result2 = SyncManagerValidation::validate(configs);
    EXPECT_TRUE(result2.valid) << "Disabled SM shouldn't cause overlap error";
}
