/**
 * @file MockDriveBackend.hpp
 * @brief GMock-based mock and functional fake for DriveBackend
 */
#pragma once

#include <gmock/gmock.h>
#include <tether/profiles/cia402/DriveBackend.hpp>

namespace CiA402 {
namespace mock {

/**
 * @brief Full GMock mock of DriveBackend – use with NiceMock to silence unused stubs
 */
class MockDriveBackend : public DriveBackend {
public:
    // Lifecycle
    MOCK_METHOD(bool, initialize, (), (override));
    MOCK_METHOD(void, deinitialize, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const, override));
    MOCK_METHOD(std::string, getName, (), (const, override));

    // PDO
    MOCK_METHOD(bool, updateInputs, (), (override));
    MOCK_METHOD(bool, updateOutputs, (), (override));
    MOCK_METHOD(DriveState, getState, (), (const, override));

    // Control/Status
    MOCK_METHOD(uint16_t, readStatusWord, (), (override));
    MOCK_METHOD(void, writeControlWord, (uint16_t), (override));
    MOCK_METHOD(uint16_t, readControlWord, (), (const, override));

    // Operating mode
    MOCK_METHOD(bool, setOperatingMode, (OperatingMode), (override));
    MOCK_METHOD(OperatingMode, getOperatingMode, (), (const, override));
    MOCK_METHOD(OperatingMode, getDisplayedMode, (), (const, override));

    // Position
    MOCK_METHOD(void, setTargetPosition, (int32_t), (override));
    MOCK_METHOD(int32_t, getActualPosition, (), (const, override));
    MOCK_METHOD(int32_t, getPositionDemand, (), (const, override));
    MOCK_METHOD(int32_t, getFollowingError, (), (const, override));
    MOCK_METHOD(void, setPositionOffset, (int32_t), (override));

    // Velocity
    MOCK_METHOD(void, setTargetVelocity, (int32_t), (override));
    MOCK_METHOD(int32_t, getActualVelocity, (), (const, override));
    MOCK_METHOD(int32_t, getVelocityDemand, (), (const, override));
    MOCK_METHOD(void, setVelocityOffset, (int32_t), (override));

    // Torque
    MOCK_METHOD(void, setTargetTorque, (int16_t), (override));
    MOCK_METHOD(int16_t, getActualTorque, (), (const, override));
    MOCK_METHOD(void, setTorqueOffset, (int16_t), (override));

    // Profile parameters
    MOCK_METHOD(void, setProfileVelocity, (uint32_t), (override));
    MOCK_METHOD(void, setProfileAcceleration, (uint32_t), (override));
    MOCK_METHOD(void, setProfileDeceleration, (uint32_t), (override));
    MOCK_METHOD(void, setMotionProfileType, (int16_t), (override));

    // Homing
    MOCK_METHOD(bool, configureHoming, (const HomingParams&), (override));
    MOCK_METHOD(HomingParams, getHomingParams, (), (const, override));

    // Interpolation
    MOCK_METHOD(bool, configureInterpolation, (const InterpolationParams&), (override));
    MOCK_METHOD(bool, addInterpolationPoint, (int32_t), (override));
    MOCK_METHOD(void, clearInterpolationBuffer, (), (override));

    // SDO
    MOCK_METHOD(SDOResult, readSDO, (uint16_t, uint8_t, void*, size_t), (override));
    MOCK_METHOD(SDOResult, writeSDO, (uint16_t, uint8_t, const void*, size_t), (override));

    // Configuration
    MOCK_METHOD(bool, configure, (const DriveConfig&), (override));
    MOCK_METHOD(DriveConfig, getConfiguration, (), (const, override));
    MOCK_METHOD(bool, storeParameters, (), (override));
    MOCK_METHOD(bool, restoreParameters, (), (override));

    // Error handling
    MOCK_METHOD(uint16_t, getErrorCode, (), (const, override));
    MOCK_METHOD(uint8_t, getErrorRegister, (), (const, override));
    MOCK_METHOD(std::vector<uint16_t>, getErrorHistory, (), (const, override));
    MOCK_METHOD(bool, clearErrorHistory, (), (override));

    // Callbacks
    MOCK_METHOD(void, setStateChangeCallback, (StateChangeCallback), (override));
    MOCK_METHOD(void, setErrorCallback, (ErrorCallback), (override));
    MOCK_METHOD(void, setWarningCallback, (WarningCallback), (override));
    MOCK_METHOD(void, setSyncCallback, (SyncCallback), (override));

    // Timing
    MOCK_METHOD(uint32_t, getCycleTimeUs, (), (const, override));
    MOCK_METHOD(bool, setCycleTimeUs, (uint32_t), (override));
    MOCK_METHOD(uint64_t, getLastUpdateTimestamp, (), (const, override));
};

/**
 * @brief Functional fake with controllable position/velocity/torque
 *
 * All methods return sensible defaults. Use setActualPosition() etc. to
 * control what getActualPosition() returns, and check lastTargetPosition()
 * to see what was commanded.
 */
class FakeDriveBackend : public DriveBackend {
public:
    // -- Controllable state --------------------------------------------------
    void setActualPosition(int32_t p) { m_actualPos = p; }
    void setActualVelocity(int32_t v) { m_actualVel = v; }
    void setActualTorque(int16_t t)   { m_actualTorque = t; }

    int32_t lastTargetPosition() const { return m_targetPos; }
    int32_t lastVelocityOffset() const { return m_velOffset; }
    int32_t lastTargetVelocity() const { return m_targetVel; }

    // -- Lifecycle -----------------------------------------------------------
    bool initialize() override { return true; }
    void deinitialize() override {}
    bool isConnected() const override { return true; }
    std::string getName() const override { return "FakeDrive"; }

    // -- PDO -----------------------------------------------------------------
    bool updateInputs() override { return true; }
    bool updateOutputs() override { return true; }
    DriveState getState() const override { return {}; }

    // -- Control/Status ------------------------------------------------------
    uint16_t readStatusWord() override { return 0; }
    void writeControlWord(uint16_t) override {}
    uint16_t readControlWord() const override { return 0; }

    // -- Operating mode ------------------------------------------------------
    bool setOperatingMode(OperatingMode) override { return true; }
    OperatingMode getOperatingMode() const override { return OperatingMode::NoMode; }
    OperatingMode getDisplayedMode() const override { return OperatingMode::NoMode; }

    // -- Position ------------------------------------------------------------
    void setTargetPosition(int32_t p) override { m_targetPos = p; }
    int32_t getActualPosition() const override { return m_actualPos; }
    int32_t getPositionDemand() const override { return 0; }
    int32_t getFollowingError() const override { return 0; }
    void setPositionOffset(int32_t) override {}

    // -- Velocity ------------------------------------------------------------
    void setTargetVelocity(int32_t v) override { m_targetVel = v; }
    int32_t getActualVelocity() const override { return m_actualVel; }
    int32_t getVelocityDemand() const override { return 0; }
    void setVelocityOffset(int32_t o) override { m_velOffset = o; }

    // -- Torque --------------------------------------------------------------
    void setTargetTorque(int16_t) override {}
    int16_t getActualTorque() const override { return m_actualTorque; }
    void setTorqueOffset(int16_t) override {}

    // -- Profile parameters --------------------------------------------------
    void setProfileVelocity(uint32_t) override {}
    void setProfileAcceleration(uint32_t) override {}
    void setProfileDeceleration(uint32_t) override {}
    void setMotionProfileType(int16_t) override {}

    // -- Homing --------------------------------------------------------------
    bool configureHoming(const HomingParams&) override { return true; }
    HomingParams getHomingParams() const override { return {}; }

    // -- Interpolation -------------------------------------------------------
    bool configureInterpolation(const InterpolationParams&) override { return true; }
    bool addInterpolationPoint(int32_t) override { return true; }
    void clearInterpolationBuffer() override {}

    // -- SDO -----------------------------------------------------------------
    SDOResult readSDO(uint16_t, uint8_t, void*, size_t) override { return {}; }
    SDOResult writeSDO(uint16_t, uint8_t, const void*, size_t) override { return {}; }

    // -- Config --------------------------------------------------------------
    bool configure(const DriveConfig&) override { return true; }
    DriveConfig getConfiguration() const override { return {}; }
    bool storeParameters() override { return true; }
    bool restoreParameters() override { return true; }

    // -- Errors --------------------------------------------------------------
    uint16_t getErrorCode() const override { return 0; }
    uint8_t getErrorRegister() const override { return 0; }
    std::vector<uint16_t> getErrorHistory() const override { return {}; }
    bool clearErrorHistory() override { return true; }

    // -- Callbacks -----------------------------------------------------------
    void setStateChangeCallback(StateChangeCallback) override {}
    void setErrorCallback(ErrorCallback) override {}
    void setWarningCallback(WarningCallback) override {}
    void setSyncCallback(SyncCallback) override {}

    // -- Timing --------------------------------------------------------------
    uint32_t getCycleTimeUs() const override { return 1000; }
    bool setCycleTimeUs(uint32_t) override { return true; }
    uint64_t getLastUpdateTimestamp() const override { return 0; }

private:
    int32_t m_actualPos{0};
    int32_t m_actualVel{0};
    int16_t m_actualTorque{0};
    int32_t m_targetPos{0};
    int32_t m_targetVel{0};
    int32_t m_velOffset{0};
};

} // namespace mock
} // namespace CiA402
