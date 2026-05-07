/**
 * @file CiA401Slave.hpp
 * @brief CiA 401 Generic I/O Module Slave Implementation
 *
 * @details
 * Implements a complete CiA 401 compliant I/O slave with:
 * - Digital inputs (8/16/32 bit grouped)
 * - Digital outputs (8/16/32 bit grouped)
 * - Analog inputs (16-bit values, configurable channels)
 * - Analog outputs (16-bit values, configurable channels)
 * - Interrupt handling
 * - Error modes for outputs
 *
 * ## Object Dictionary Structure
 *
 * | Index Range | Description |
 * |-------------|-------------|
 * | 0x1000-0x1FFF | Communication profile (CiA 301) |
 * | 0x6000-0x607F | Digital inputs |
 * | 0x6100-0x617F | Digital input parameters |
 * | 0x6200-0x627F | Digital outputs |
 * | 0x6300-0x637F | Digital output parameters |
 * | 0x6400-0x647F | Analog inputs |
 * | 0x6410-0x641F | Analog input parameters |
 * | 0x6500-0x657F | Analog outputs |
 * | 0x6510-0x651F | Analog output parameters |
 *
 * ## Usage Example
 *
 * @code
 * CiA401SlaveConfig config;
 * config.identity.vendorId = 0x1234;
 * config.digitalInputs8 = 2;   // 16 digital inputs
 * config.digitalOutputs8 = 2;  // 16 digital outputs
 * config.analogInputs = 4;     // 4 analog inputs
 *
 * auto io = std::make_unique<CiA401Slave>(config);
 * io->setDigitalInput(0, 0xFF);  // Set inputs for simulation
 * io->start();
 * @endcode
 */

#pragma once

#include "slave/profiles/ProfileSlave.hpp"
#include "profiles/cia401/CiA401Defs.hpp"

#include <array>
#include <atomic>
#include <functional>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CiA 401 Slave Configuration
// ============================================================================

/**
 * @brief Configuration for CiA 401 I/O slave
 */
struct CiA401SlaveConfig {
    // Identity
    SlaveIdentity identity = {
        .vendorId = 0x00000000,
        .productCode = 0x00000191,  // Device type for I/O module
        .revisionNumber = 0x00010000,
        .serialNumber = 0x00000001,
        .deviceName = "CiA 401 I/O Module",
    };
    
    // Digital I/O configuration (number of 8-bit groups)
    uint8_t digitalInputs8 = 1;   ///< Number of 8-bit digital input groups (max 32)
    uint8_t digitalOutputs8 = 1;  ///< Number of 8-bit digital output groups (max 32)
    
    // Analog I/O configuration (number of channels)
    uint8_t analogInputs = 0;     ///< Number of analog input channels (max 8)
    uint8_t analogOutputs = 0;    ///< Number of analog output channels (max 8)
    
    // Analog parameters
    int16_t analogInputMin = -32768;   ///< Minimum analog input value
    int16_t analogInputMax = 32767;    ///< Maximum analog input value
    int16_t analogOutputMin = -32768;  ///< Minimum analog output value
    int16_t analogOutputMax = 32767;   ///< Maximum analog output value
    
    // Error behavior
    bool outputErrorModeEnabled = true;  ///< Use error values on communication loss
    
    // DC support
    bool supportsDC = true;
    uint32_t defaultCycleTime = 1000000;  // 1ms
    
    // Logging
    SlaveLogConfig logConfig;
};

// ============================================================================
// CiA 401 Slave Class
// ============================================================================

/**
 * @brief CiA 401 Generic I/O Module Slave
 *
 * Provides a complete implementation of a CiA 401 I/O module including:
 * - Configurable digital I/O (8/16/32-bit access)
 * - Configurable analog I/O (16-bit resolution)
 * - Standard object dictionary
 * - PDO mappings for common configurations
 */
class CiA401Slave : public ProfileSlave {
public:
    /**
     * @brief Constructor
     * @param config I/O slave configuration
     */
    explicit CiA401Slave(const CiA401SlaveConfig& config);
    
    /**
     * @brief Destructor
     */
    ~CiA401Slave() override;
    
    // ========================================================================
    // ProfileSlave Interface
    // ========================================================================
    
    const char* getProfileName() const override { return "CiA 401"; }
    uint32_t getDeviceType() const override { return 0x00000191; }
    
    void updateTxPDO() override;
    void processRxPDO() override;
    void simulate(uint64_t deltaNs) override;
    
    // ========================================================================
    // Digital Input Access
    // ========================================================================
    
    /**
     * @brief Get number of digital input bytes
     */
    size_t getDigitalInputCount() const { return ioConfig_.digitalInputs8; }
    
    /**
     * @brief Set digital input value (8-bit group)
     *
     * For simulation: set the input values that the slave reports.
     *
     * @param group 8-bit group index (0-31)
     * @param value 8-bit value
     */
    void setDigitalInput8(size_t group, uint8_t value);
    
    /**
     * @brief Get digital input value (8-bit group)
     */
    uint8_t getDigitalInput8(size_t group) const;
    
    /**
     * @brief Set digital input value (16-bit group)
     */
    void setDigitalInput16(size_t group, uint16_t value);
    
    /**
     * @brief Get digital input value (16-bit group)
     */
    uint16_t getDigitalInput16(size_t group) const;
    
    /**
     * @brief Set single digital input bit
     */
    void setDigitalInputBit(size_t bit, bool value);
    
    /**
     * @brief Get single digital input bit
     */
    bool getDigitalInputBit(size_t bit) const;
    
    // ========================================================================
    // Digital Output Access
    // ========================================================================
    
    /**
     * @brief Get number of digital output bytes
     */
    size_t getDigitalOutputCount() const { return ioConfig_.digitalOutputs8; }
    
    /**
     * @brief Get digital output value (8-bit group)
     *
     * Returns the value written by the master.
     */
    uint8_t getDigitalOutput8(size_t group) const;
    
    /**
     * @brief Get digital output value (16-bit group)
     */
    uint16_t getDigitalOutput16(size_t group) const;
    
    /**
     * @brief Get single digital output bit
     */
    bool getDigitalOutputBit(size_t bit) const;
    
    /**
     * @brief Set output callback
     *
     * Called when digital outputs are updated by master.
     */
    using DigitalOutputCallback = std::function<void(const uint8_t* data, size_t len)>;
    void setDigitalOutputCallback(DigitalOutputCallback callback);
    
    // ========================================================================
    // Analog Input Access
    // ========================================================================
    
    /**
     * @brief Get number of analog inputs
     */
    size_t getAnalogInputCount() const { return ioConfig_.analogInputs; }
    
    /**
     * @brief Set analog input value
     *
     * For simulation: set the input values that the slave reports.
     *
     * @param channel Channel index (0-7)
     * @param value 16-bit value
     */
    void setAnalogInput(size_t channel, int16_t value);
    
    /**
     * @brief Get analog input value
     */
    int16_t getAnalogInput(size_t channel) const;
    
    /**
     * @brief Set analog input scaling
     *
     * @param channel Channel index
     * @param offset Offset value
     * @param gain Gain (1.0 = unity)
     */
    void setAnalogInputScaling(size_t channel, int16_t offset, float gain);
    
    // ========================================================================
    // Analog Output Access
    // ========================================================================
    
    /**
     * @brief Get number of analog outputs
     */
    size_t getAnalogOutputCount() const { return ioConfig_.analogOutputs; }
    
    /**
     * @brief Get analog output value
     *
     * Returns the value written by the master.
     */
    int16_t getAnalogOutput(size_t channel) const;
    
    /**
     * @brief Set analog output callback
     */
    using AnalogOutputCallback = std::function<void(size_t channel, int16_t value)>;
    void setAnalogOutputCallback(AnalogOutputCallback callback);
    
    // ========================================================================
    // Interrupt/Event Handling
    // ========================================================================
    
    /**
     * @brief Configure digital input interrupt
     *
     * @param group 8-bit group index
     * @param mask Interrupt mask (1 = enabled)
     * @param mode 0=any change, 1=low-to-high, 2=high-to-low
     */
    void configureDigitalInterrupt(size_t group, uint8_t mask, uint8_t mode);
    
    /**
     * @brief Set interrupt callback
     */
    using InterruptCallback = std::function<void(size_t group, uint8_t changedBits)>;
    void setInterruptCallback(InterruptCallback callback);
    
    // ========================================================================
    // Error Handling
    // ========================================================================
    
    /**
     * @brief Set error value for digital outputs
     *
     * These values are used when communication is lost.
     */
    void setDigitalOutputErrorValue(size_t group, uint8_t value);
    
    /**
     * @brief Set error mode for digital outputs
     *
     * @param group 8-bit group index
     * @param mode 0=use error value, 1=keep last value
     */
    void setDigitalOutputErrorMode(size_t group, uint8_t mode);
    
    /**
     * @brief Trigger communication error (for testing)
     */
    void triggerCommunicationError();
    
    /**
     * @brief Clear communication error
     */
    void clearCommunicationError();
    
protected:
    void initObjectDictionary() override;
    void initPDOMappings() override;
    void onStateChange(SlaveState oldState, SlaveState newState) override;
    
private:
    // Configuration
    CiA401SlaveConfig ioConfig_;
    
    // Digital I/O storage (max 32 bytes each = 256 bits)
    std::array<uint8_t, 32> digitalInputs_;
    std::array<uint8_t, 32> digitalOutputs_;
    std::array<uint8_t, 32> digitalInputPolarity_;
    std::array<uint8_t, 32> digitalOutputPolarity_;
    std::array<uint8_t, 32> digitalOutputErrorMode_;
    std::array<uint8_t, 32> digitalOutputErrorValue_;
    std::array<uint8_t, 32> digitalInterruptMask_;
    std::array<uint8_t, 32> previousDigitalInputs_;  // For interrupt detection
    
    // Analog I/O storage (max 8 channels each)
    std::array<int16_t, 8> analogInputs_;
    std::array<int16_t, 8> analogOutputs_;
    std::array<int16_t, 8> analogInputOffset_;
    std::array<float, 8> analogInputGain_;
    std::array<int16_t, 8> analogOutputErrorValue_;
    
    // Interrupt configuration
    bool interruptGlobalEnable_ = false;
    
    // Callbacks
    DigitalOutputCallback digitalOutputCallback_;
    AnalogOutputCallback analogOutputCallback_;
    InterruptCallback interruptCallback_;
    
    // Communication status
    bool communicationError_ = false;
    
    // PDO layout
    struct PDOLayout {
        size_t digitalInputOffset = 0;
        size_t digitalInputSize = 0;
        size_t analogInputOffset = 0;
        size_t analogInputSize = 0;
        size_t digitalOutputOffset = 0;
        size_t digitalOutputSize = 0;
        size_t analogOutputOffset = 0;
        size_t analogOutputSize = 0;
    } pdoLayout_;
    
    // SDO handlers
    SDOAbortCode readDigitalInput(uint16_t index, uint8_t subindex,
                                   uint8_t* data, size_t& len);
    SDOAbortCode writeDigitalOutput(uint16_t index, uint8_t subindex,
                                     const uint8_t* data, size_t len);
    SDOAbortCode readAnalogInput(uint16_t index, uint8_t subindex,
                                  uint8_t* data, size_t& len);
    SDOAbortCode writeAnalogOutput(uint16_t index, uint8_t subindex,
                                    const uint8_t* data, size_t len);
    
    // Helper methods
    void checkInterrupts();
    void applyErrorValues();
};

// ============================================================================
// Factory Function
// ============================================================================

/**
 * @brief Create a CiA 401 I/O slave with specified configuration
 */
std::unique_ptr<CiA401Slave> createCiA401Slave(const CiA401SlaveConfig& config);

/**
 * @brief Create a simple digital I/O slave
 *
 * @param inputs Number of digital input bits (rounded up to 8)
 * @param outputs Number of digital output bits (rounded up to 8)
 */
std::unique_ptr<CiA401Slave> createDigitalIOSlave(size_t inputs, size_t outputs);

/**
 * @brief Create a simple analog I/O slave
 *
 * @param inputs Number of analog input channels
 * @param outputs Number of analog output channels
 */
std::unique_ptr<CiA401Slave> createAnalogIOSlave(size_t inputs, size_t outputs);

}  // namespace slave
}  // namespace EtherCAT
