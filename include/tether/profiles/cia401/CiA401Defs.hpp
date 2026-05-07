/**
 * @file CiA401Defs.hpp
 * @brief CiA 401 Device Profile Definitions for I/O Modules
 * 
 * @details
 * This file contains comprehensive definitions for CiA 401 compliant I/O modules:
 * - Object dictionary indices and subindices for digital/analog I/O
 * - Module types and classifications
 * - Interrupt and event handling parameters
 * - PDO mapping definitions
 * - Error codes and diagnostic objects
 * 
 * Reference: CiA 401 v3.0 - CANopen device profile for I/O modules
 * 
 * @note All object indices follow the CiA 401 specification
 */

#pragma once

#include <cstdint>

namespace CiA401 {

// ============================================================================
// Device Type and Profile Information
// ============================================================================

/** Device type for CiA 401 I/O modules (returned in object 0x1000) */
constexpr uint32_t DEVICE_TYPE = 0x00000191; // Profile 401

/** Profile number */
constexpr uint16_t PROFILE_NUMBER = 401;

// ============================================================================
// Digital Input Objects (0x6000-0x60FF)
// ============================================================================

/** @name Digital Input Objects */
/** @{ */

/** Read digital input 8-bit blocks (0x6000) */
constexpr uint16_t DigitalInput8 = 0x6000;

/** Number of digital input 8-bit blocks (subindex 0) */
constexpr uint8_t DigitalInput8_NumberOfElements = 0x00;

/** Digital input polarity 8-bit (0x6002) */
constexpr uint16_t DigitalInputPolarity8 = 0x6002;

/** Digital input filter constant 8-bit (0x6003) */
constexpr uint16_t DigitalInputFilter8 = 0x6003;

/** Global interrupt enable digital (0x6005) */
constexpr uint16_t DigitalInterruptEnable = 0x6005;

/** Interrupt mask any change 8-bit (0x6006) */
constexpr uint16_t DigitalInterruptMaskAnyChange8 = 0x6006;

/** Interrupt mask low-to-high 8-bit (0x6007) */
constexpr uint16_t DigitalInterruptMaskLowToHigh8 = 0x6007;

/** Interrupt mask high-to-low 8-bit (0x6008) */
constexpr uint16_t DigitalInterruptMaskHighToLow8 = 0x6008;

/** Read digital input 16-bit blocks (0x6020) */
constexpr uint16_t DigitalInput16 = 0x6020;

/** Digital input polarity 16-bit (0x6022) */
constexpr uint16_t DigitalInputPolarity16 = 0x6022;

/** Digital input filter constant 16-bit (0x6023) */
constexpr uint16_t DigitalInputFilter16 = 0x6023;

/** Interrupt mask any change 16-bit (0x6026) */
constexpr uint16_t DigitalInterruptMaskAnyChange16 = 0x6026;

/** Interrupt mask low-to-high 16-bit (0x6027) */
constexpr uint16_t DigitalInterruptMaskLowToHigh16 = 0x6027;

/** Interrupt mask high-to-low 16-bit (0x6028) */
constexpr uint16_t DigitalInterruptMaskHighToLow16 = 0x6028;

/** Read digital input 32-bit blocks (0x6040) */
constexpr uint16_t DigitalInput32 = 0x6040;

/** Digital input polarity 32-bit (0x6042) */
constexpr uint16_t DigitalInputPolarity32 = 0x6042;

/** Digital input filter constant 32-bit (0x6043) */
constexpr uint16_t DigitalInputFilter32 = 0x6043;

/** Interrupt mask any change 32-bit (0x6046) */
constexpr uint16_t DigitalInterruptMaskAnyChange32 = 0x6046;

/** Interrupt mask low-to-high 32-bit (0x6047) */
constexpr uint16_t DigitalInterruptMaskLowToHigh32 = 0x6047;

/** Interrupt mask high-to-low 32-bit (0x6048) */
constexpr uint16_t DigitalInterruptMaskHighToLow32 = 0x6048;

/** @} */

// ============================================================================
// Digital Output Objects (0x6200-0x62FF)
// ============================================================================

/** @name Digital Output Objects */
/** @{ */

/** Write digital output 8-bit blocks (0x6200) */
constexpr uint16_t DigitalOutput8 = 0x6200;

/** Digital output polarity 8-bit (0x6202) */
constexpr uint16_t DigitalOutputPolarity8 = 0x6202;

/** Digital output error mode 8-bit (0x6206) */
constexpr uint16_t DigitalOutputErrorMode8 = 0x6206;

/** Digital output error value 8-bit (0x6207) */
constexpr uint16_t DigitalOutputErrorValue8 = 0x6207;

/** Digital output filter/PWM 8-bit (0x6208) */
constexpr uint16_t DigitalOutputFilter8 = 0x6208;

/** Write digital output 16-bit blocks (0x6220) */
constexpr uint16_t DigitalOutput16 = 0x6220;

/** Digital output polarity 16-bit (0x6222) */
constexpr uint16_t DigitalOutputPolarity16 = 0x6222;

/** Digital output error mode 16-bit (0x6226) */
constexpr uint16_t DigitalOutputErrorMode16 = 0x6226;

/** Digital output error value 16-bit (0x6227) */
constexpr uint16_t DigitalOutputErrorValue16 = 0x6227;

/** Digital output filter/PWM 16-bit (0x6228) */
constexpr uint16_t DigitalOutputFilter16 = 0x6228;

/** Write digital output 32-bit blocks (0x6240) */
constexpr uint16_t DigitalOutput32 = 0x6240;

/** Digital output polarity 32-bit (0x6242) */
constexpr uint16_t DigitalOutputPolarity32 = 0x6242;

/** Digital output error mode 32-bit (0x6246) */
constexpr uint16_t DigitalOutputErrorMode32 = 0x6246;

/** Digital output error value 32-bit (0x6247) */
constexpr uint16_t DigitalOutputErrorValue32 = 0x6247;

/** Digital output filter/PWM 32-bit (0x6248) */
constexpr uint16_t DigitalOutputFilter32 = 0x6248;

/** @} */

// ============================================================================
// Analog Input Objects (0x6400-0x64FF)
// ============================================================================

/** @name Analog Input Objects */
/** @{ */

/** Read analog input 16-bit (0x6401) */
constexpr uint16_t AnalogInput16 = 0x6401;

/** Read analog input 32-bit (0x6402) - for high resolution */
constexpr uint16_t AnalogInput32 = 0x6402;

/** Analog input interrupt trigger selection (0x6421) */
constexpr uint16_t AnalogInputInterruptTrigger = 0x6421;

/** Analog input interrupt source (0x6422) */
constexpr uint16_t AnalogInputInterruptSource = 0x6422;

/** Global interrupt enable analog input (0x6423) */
constexpr uint16_t AnalogInputInterruptEnable = 0x6423;

/** Analog input upper limit interrupt (0x6424) */
constexpr uint16_t AnalogInputUpperLimit = 0x6424;

/** Analog input lower limit interrupt (0x6425) */
constexpr uint16_t AnalogInputLowerLimit = 0x6425;

/** Analog input delta interrupt (0x6426) */
constexpr uint16_t AnalogInputDelta = 0x6426;

/** Analog input negative delta interrupt (0x6427) */
constexpr uint16_t AnalogInputNegativeDelta = 0x6427;

/** Analog input positive delta interrupt (0x6428) */
constexpr uint16_t AnalogInputPositiveDelta = 0x6428;

/** Analog input scaling offset (0x6431) */
constexpr uint16_t AnalogInputOffset = 0x6431;

/** Analog input scaling factor (0x6432) */
constexpr uint16_t AnalogInputScaling = 0x6432;

/** Analog input SI unit (0x6440) */
constexpr uint16_t AnalogInputSIUnit = 0x6440;

/** @} */

// ============================================================================
// Analog Output Objects (0x6410-0x641F for configuration, 0x6411 for value)
// ============================================================================

/** @name Analog Output Objects */
/** @{ */

/** Write analog output 16-bit (0x6411) */
constexpr uint16_t AnalogOutput16 = 0x6411;

/** Write analog output 32-bit (0x6412) - for high resolution */
constexpr uint16_t AnalogOutput32 = 0x6412;

/** Analog output error mode (0x6441) */
constexpr uint16_t AnalogOutputErrorMode = 0x6441;

/** Analog output error value (0x6442) */
constexpr uint16_t AnalogOutputErrorValue = 0x6442;

/** Analog output scaling offset (0x6451) */
constexpr uint16_t AnalogOutputOffset = 0x6451;

/** Analog output scaling factor (0x6452) */
constexpr uint16_t AnalogOutputScaling = 0x6452;

/** Analog output SI unit (0x6460) */
constexpr uint16_t AnalogOutputSIUnit = 0x6460;

/** @} */

// ============================================================================
// Counter Objects (0x6500-0x65FF) - Optional
// ============================================================================

/** @name Counter Objects */
/** @{ */

/** Counter value 32-bit (0x6500) */
constexpr uint16_t CounterValue = 0x6500;

/** Counter preset value (0x6501) */
constexpr uint16_t CounterPreset = 0x6501;

/** Counter control (0x6502) */
constexpr uint16_t CounterControl = 0x6502;

/** Counter status (0x6503) */
constexpr uint16_t CounterStatus = 0x6503;

/** @} */

// ============================================================================
// Frequency/PWM Input Objects (0x6510-0x651F) - Optional
// ============================================================================

/** @name Frequency Input Objects */
/** @{ */

/** Frequency input value (0x6510) */
constexpr uint16_t FrequencyInput = 0x6510;

/** Period input value (0x6511) */
constexpr uint16_t PeriodInput = 0x6511;

/** Duty cycle input value (0x6512) */
constexpr uint16_t DutyCycleInput = 0x6512;

/** @} */

// ============================================================================
// PWM Output Objects (0x6520-0x652F) - Optional
// ============================================================================

/** @name PWM Output Objects */
/** @{ */

/** PWM output duty cycle (0x6520) */
constexpr uint16_t PWMDutyCycle = 0x6520;

/** PWM output frequency (0x6521) */
constexpr uint16_t PWMFrequency = 0x6521;

/** @} */

// ============================================================================
// Error Mode Values
// ============================================================================

namespace ErrorMode {
    /** Output maintains last value on communication error */
    constexpr uint8_t MaintainLastValue = 0x00;
    
    /** Output goes to error value on communication error */
    constexpr uint8_t UseErrorValue = 0x01;
}

// ============================================================================
// Interrupt Trigger Selection Values
// ============================================================================

namespace InterruptTrigger {
    /** Trigger on upper limit exceeded */
    constexpr uint8_t UpperLimit = 0x01;
    
    /** Trigger on lower limit undercut */
    constexpr uint8_t LowerLimit = 0x02;
    
    /** Trigger on delta change (absolute) */
    constexpr uint8_t Delta = 0x04;
    
    /** Trigger on negative delta */
    constexpr uint8_t NegativeDelta = 0x08;
    
    /** Trigger on positive delta */
    constexpr uint8_t PositiveDelta = 0x10;
}

// ============================================================================
// Filter Constant Values (for digital inputs)
// ============================================================================

namespace FilterConstant {
    /** No filtering */
    constexpr uint8_t NoFilter = 0x00;
    
    /** 3ms filter time */
    constexpr uint8_t Filter3ms = 0x01;
    
    /** 10ms filter time */
    constexpr uint8_t Filter10ms = 0x02;
    
    /** 20ms filter time */
    constexpr uint8_t Filter20ms = 0x03;
    
    /** 100ms filter time */
    constexpr uint8_t Filter100ms = 0x04;
}

// ============================================================================
// Counter Control Bits
// ============================================================================

namespace CounterControlBits {
    /** Enable counter */
    constexpr uint8_t Enable = 0x01;
    
    /** Set counter to preset value */
    constexpr uint8_t Preset = 0x02;
    
    /** Count up (1) or down (0) */
    constexpr uint8_t CountUp = 0x04;
    
    /** Latch on trigger */
    constexpr uint8_t Latch = 0x08;
}

// ============================================================================
// Counter Status Bits
// ============================================================================

namespace CounterStatusBits {
    /** Counter is running */
    constexpr uint8_t Running = 0x01;
    
    /** Overflow occurred */
    constexpr uint8_t Overflow = 0x02;
    
    /** Underflow occurred */
    constexpr uint8_t Underflow = 0x04;
    
    /** Counter at preset value */
    constexpr uint8_t AtPreset = 0x08;
}

// ============================================================================
// SI Unit Prefix Codes (for analog I/O)
// ============================================================================

namespace SIUnitPrefix {
    constexpr uint8_t Pico = 0xF4;   // 10^-12
    constexpr uint8_t Nano = 0xF7;   // 10^-9
    constexpr uint8_t Micro = 0xFA;  // 10^-6
    constexpr uint8_t Milli = 0xFD;  // 10^-3
    constexpr uint8_t None = 0x00;   // 10^0
    constexpr uint8_t Kilo = 0x03;   // 10^3
    constexpr uint8_t Mega = 0x06;   // 10^6
    constexpr uint8_t Giga = 0x09;   // 10^9
}

// ============================================================================
// SI Unit Type Codes
// ============================================================================

namespace SIUnitType {
    constexpr uint8_t Dimensionless = 0x00;
    constexpr uint8_t Length_Meter = 0x01;
    constexpr uint8_t Mass_Kilogram = 0x02;
    constexpr uint8_t Time_Second = 0x03;
    constexpr uint8_t Current_Ampere = 0x04;
    constexpr uint8_t Temperature_Kelvin = 0x05;
    constexpr uint8_t Voltage_Volt = 0x06;
    constexpr uint8_t Force_Newton = 0x07;
    constexpr uint8_t Pressure_Pascal = 0x08;
    constexpr uint8_t Angle_Radian = 0x0A;
    constexpr uint8_t Frequency_Hertz = 0x10;
    constexpr uint8_t Power_Watt = 0x11;
    constexpr uint8_t Resistance_Ohm = 0x12;
    constexpr uint8_t Capacitance_Farad = 0x13;
    constexpr uint8_t Temperature_Celsius = 0x2D;
}

// ============================================================================
// Default PDO Mappings
// ============================================================================

namespace DefaultPDOMapping {
    // TxPDO 1 - Digital Inputs 8-bit
    constexpr uint32_t TxPDO1_Entry1 = 0x60000108; // 8 bits of digital input block 1
    
    // RxPDO 1 - Digital Outputs 8-bit
    constexpr uint32_t RxPDO1_Entry1 = 0x62000108; // 8 bits of digital output block 1
    
    // TxPDO 2 - Analog Inputs 16-bit
    constexpr uint32_t TxPDO2_Entry1 = 0x64010110; // Analog input 1 (16 bits)
    constexpr uint32_t TxPDO2_Entry2 = 0x64010210; // Analog input 2 (16 bits)
    
    // RxPDO 2 - Analog Outputs 16-bit
    constexpr uint32_t RxPDO2_Entry1 = 0x64110110; // Analog output 1 (16 bits)
    constexpr uint32_t RxPDO2_Entry2 = 0x64110210; // Analog output 2 (16 bits)
}

// ============================================================================
// Module Type Classification
// ============================================================================

/**
 * @brief Module type classifications for CiA 401 devices
 */
enum class ModuleType : uint8_t {
    DigitalInputOnly = 0x01,
    DigitalOutputOnly = 0x02,
    DigitalIO = 0x03,
    AnalogInputOnly = 0x10,
    AnalogOutputOnly = 0x20,
    AnalogIO = 0x30,
    MixedDigitalAnalog = 0x33,
    Counter = 0x40,
    FrequencyInput = 0x50,
    PWMOutput = 0x60,
    FullFeatured = 0xFF
};

/**
 * @brief Get human-readable name for module type
 */
inline const char* getModuleTypeName(ModuleType type) {
    switch (type) {
        case ModuleType::DigitalInputOnly: return "Digital Input";
        case ModuleType::DigitalOutputOnly: return "Digital Output";
        case ModuleType::DigitalIO: return "Digital I/O";
        case ModuleType::AnalogInputOnly: return "Analog Input";
        case ModuleType::AnalogOutputOnly: return "Analog Output";
        case ModuleType::AnalogIO: return "Analog I/O";
        case ModuleType::MixedDigitalAnalog: return "Mixed Digital/Analog I/O";
        case ModuleType::Counter: return "Counter Module";
        case ModuleType::FrequencyInput: return "Frequency Input";
        case ModuleType::PWMOutput: return "PWM Output";
        case ModuleType::FullFeatured: return "Full Featured I/O";
        default: return "Unknown";
    }
}

// ============================================================================
// Digital I/O Structures
// ============================================================================

/**
 * @brief Configuration for a digital input channel block
 */
struct DigitalInputConfig {
    uint8_t polarity;      ///< Polarity inversion mask (1 = inverted)
    uint8_t filter;        ///< Filter constant (see FilterConstant namespace)
    uint8_t int_any;       ///< Interrupt mask for any change
    uint8_t int_l2h;       ///< Interrupt mask for low-to-high
    uint8_t int_h2l;       ///< Interrupt mask for high-to-low
    
    DigitalInputConfig() : polarity(0), filter(0), int_any(0), int_l2h(0), int_h2l(0) {}
};

/**
 * @brief Configuration for a digital output channel block
 */
struct DigitalOutputConfig {
    uint8_t polarity;      ///< Polarity inversion mask (1 = inverted)
    uint8_t error_mode;    ///< Error mode (see ErrorMode namespace)
    uint8_t error_value;   ///< Value to output on error
    uint8_t filter_pwm;    ///< Filter or PWM setting
    
    DigitalOutputConfig() : polarity(0), error_mode(0), error_value(0), filter_pwm(0) {}
};

// ============================================================================
// Analog I/O Structures
// ============================================================================

/**
 * @brief Configuration for an analog input channel
 */
struct AnalogInputConfig {
    int32_t offset;        ///< Offset value for scaling
    int32_t scaling;       ///< Scaling factor (fixed point)
    uint32_t si_unit;      ///< SI unit code
    uint8_t trigger;       ///< Interrupt trigger selection
    int16_t upper_limit;   ///< Upper limit for interrupt
    int16_t lower_limit;   ///< Lower limit for interrupt
    int16_t delta;         ///< Delta for interrupt
    
    AnalogInputConfig() 
        : offset(0), scaling(0x10000), si_unit(0), trigger(0),
          upper_limit(0x7FFF), lower_limit(0), delta(0) {}
};

/**
 * @brief Configuration for an analog output channel
 */
struct AnalogOutputConfig {
    int32_t offset;        ///< Offset value for scaling
    int32_t scaling;       ///< Scaling factor (fixed point)
    uint32_t si_unit;      ///< SI unit code
    uint8_t error_mode;    ///< Error mode
    int16_t error_value;   ///< Value on communication error
    
    AnalogOutputConfig()
        : offset(0), scaling(0x10000), si_unit(0), error_mode(0), error_value(0) {}
};

// ============================================================================
// PDO Structures for Real-Time Data
// ============================================================================

/**
 * @brief TxPDO for basic 8-bit digital inputs (up to 4 blocks = 32 inputs)
 */
struct __attribute__((packed)) TxPDO_DigitalInputs8 {
    uint8_t inputs[4];     ///< Up to 4 blocks of 8-bit inputs
};

/**
 * @brief TxPDO for 16-bit digital inputs (up to 2 blocks = 32 inputs)
 */
struct __attribute__((packed)) TxPDO_DigitalInputs16 {
    uint16_t inputs[2];    ///< Up to 2 blocks of 16-bit inputs
};

/**
 * @brief TxPDO for 32-bit digital inputs (1 block = 32 inputs)
 */
struct __attribute__((packed)) TxPDO_DigitalInputs32 {
    uint32_t inputs;       ///< 1 block of 32-bit inputs
};

/**
 * @brief RxPDO for basic 8-bit digital outputs (up to 4 blocks = 32 outputs)
 */
struct __attribute__((packed)) RxPDO_DigitalOutputs8 {
    uint8_t outputs[4];    ///< Up to 4 blocks of 8-bit outputs
};

/**
 * @brief TxPDO for analog inputs (up to 4 channels, 16-bit each)
 */
struct __attribute__((packed)) TxPDO_AnalogInputs16 {
    int16_t inputs[4];     ///< Up to 4 analog inputs
};

/**
 * @brief RxPDO for analog outputs (up to 4 channels, 16-bit each)
 */
struct __attribute__((packed)) RxPDO_AnalogOutputs16 {
    int16_t outputs[4];    ///< Up to 4 analog outputs
};

/**
 * @brief Combined TxPDO with digital and analog inputs
 */
struct __attribute__((packed)) TxPDO_Combined {
    uint8_t digital_inputs[2];   ///< 16 digital inputs
    int16_t analog_inputs[2];    ///< 2 analog inputs
};

/**
 * @brief Combined RxPDO with digital and analog outputs
 */
struct __attribute__((packed)) RxPDO_Combined {
    uint8_t digital_outputs[2];  ///< 16 digital outputs
    int16_t analog_outputs[2];   ///< 2 analog outputs
};

} // namespace CiA401
