/* ============================================================================================================================ *//**
 * @file       controlword.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 7:07:23 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures used to parse and construct 'Controlword' object
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_CONTROLWORD_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_CONTROLWORD_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// Private includes
#include "ethercat/utilities/named_bitset.hpp"
#include "ethercat/devices/elmo/registers.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* ========================================================== Controlword ========================================================= */

/**
 * @brief Parser structure for the "Controlword" object
 */
struct Controlword : public utilities::named_bitset<registers::CONTROLWORD.type.get_bitsize()> {

public: /* ---------------------------------------------------- Public types ----------------------------------------------------- */

    /// Value type of the controlword numeric variable
    using TypeRepresentation = common::types::traits::TypeRepresentation<registers::CONTROLWORD.type.get_id()>;

public: /* -------------------------------------------------- Public constants --------------------------------------------------- */
   
    /// Size of the controlword numeric variable
    static constexpr std::size_t Bitsize = registers::CONTROLWORD.type.get_bitsize();

    /**
     * @brief State-bit-index of the "Controlword" object
     */
    enum class State : std::size_t {
        SwitchOn        = 0,
        EnableVoltage   = 1,
        QuickStop       = 2,
        EnableOperation = 3,
        FaultReset      = 7,
        Halt            = 8,
    };

    /**
     * @brief Mode-specific-bit-index of the "Controlword" object
     */
    enum class ModeSpecific : std::size_t {
        Bit1 = 4,
        Bit2 = 5,
        Bit3 = 6,
    };

    /**
     * @brief Velocity-mode-specific-bit-index of the "Controlword" object
     */
    enum class VelocityMode : std::size_t {
        RfgEnable                = common::utilities::to_underlying(ModeSpecific::Bit1),
        ReferenceGeneratorEnable = common::utilities::to_underlying(ModeSpecific::Bit1),
        RfgUnlock                = common::utilities::to_underlying(ModeSpecific::Bit2),
        ReferenceGeneratorUnlock = common::utilities::to_underlying(ModeSpecific::Bit2),
        RfgUseRef                = common::utilities::to_underlying(ModeSpecific::Bit3),
        ReferenceGeneratorUseRef = common::utilities::to_underlying(ModeSpecific::Bit3),
    };

    /**
     * @brief Profiled-position-mode-specific-bit-index of the "Controlword" object
     */
    enum class ProfiledPositionMode : std::size_t {
        NewSetpoint            = common::utilities::to_underlying(ModeSpecific::Bit1),
        ChangeSetImmediately   = common::utilities::to_underlying(ModeSpecific::Bit2),
        AbsoluteRelativeSwitch = common::utilities::to_underlying(ModeSpecific::Bit3),
    };

    /**
     * @brief Homing-mode-specific-bit-index of the "Controlword" object
     */
    enum class HomingMode : std::size_t {
        Start = common::utilities::to_underlying(ModeSpecific::Bit1),
    };

    /**
     * @brief Interpolated-position-mode-specific-bit-index of the "Controlword" object
     */
    enum class InterpolatedPositionMode : std::size_t {
        Enable = common::utilities::to_underlying(ModeSpecific::Bit1),
    };
    
public: /* ----------------------------------------------------- Public ctors ---------------------------------------------------- */

    /// Forward constructors
    using utilities::named_bitset<Bitsize>::named_bitset;
    /// Wrap copy constructor
    constexpr Controlword(const utilities::named_bitset<Bitsize> &set) : utilities::named_bitset<Bitsize>{ set } {  }

public: /* --------------------------------------------------- Public operators -------------------------------------------------- */

    /// Forward assignment operator
    using utilities::named_bitset<Bitsize>::operator=;

    /// Wrap member operator of the bitset
    ETHERCAT_UTILITIES_WRAP_BITSET_MEMBER_OPERATORS(utilities::named_bitset<Bitsize>, Controlword);

public: /* --------------------------------------------------- Public operators -------------------------------------------------- */

    /**
     * @brief Converts Controlword to the integral constant representing the 
     *    "Controlword" object
     * 
     * @returns 
     *    controlword converted to integral constant
     */
    constexpr TypeRepresentation to_value() const {
        return utilities::named_bitset<Bitsize>::to_value<TypeRepresentation>();
    } 
    
};

/// Wrap binary operators of the bitset
ETHERCAT_UTILITIES_WRAP_BITSET_BINARY_OPERATORS(utilities::named_bitset<Controlword::Bitsize>, Controlword);

/* ======================================================= Controlword masks ====================================================== */

namespace controlword {

    /// Use custom OR operator for enumerations
    using utilities::operator|;

    /**
     * @brief Namespace grouping state-bits masks
     */
    namespace State {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::State;

        /// Mask of "State" fields in the "Controlword"
        constexpr Controlword Mask { 
            Bit::SwitchOn        |
            Bit::EnableVoltage   |
            Bit::QuickStop       |
            Bit::EnableOperation |
            Bit::FaultReset     
        };
        
        /// Mask of the "Shutdown" state in the "Controlword"
        constexpr Controlword Shutdown { Bit::QuickStop | Bit::EnableVoltage };
        /// Mask of the "Switch ON" state in the "Controlword"
        constexpr Controlword SwitchOn { Bit::QuickStop | Bit::EnableVoltage | Bit::SwitchOn };
        /// Mask of the "Disable Voltage" state in the "Controlword"
        constexpr Controlword DisableVoltage {  };
        /// Mask of the "Quick Stop" state in the "Controlword"
        constexpr Controlword QuickStop { Bit::EnableVoltage  };
        /// Mask of the "Disable Operation" state in the "Controlword"
        constexpr Controlword DisableOperation { Bit::QuickStop | Bit::EnableVoltage | Bit::SwitchOn };
        /// Mask of the "Enable Operation" state in the "Controlword"
        constexpr Controlword EnableOperation { Bit::EnableOperation | Bit::QuickStop | Bit::EnableVoltage | Bit::SwitchOn };

        /// Mask of the "Controlword" before sending "Fault Reset" command
        constexpr Controlword BeforeFaultReset {  };
        /// Mask of the "Controlword" after sending "Fault Reset" command
        constexpr Controlword AfterFaultReset { Bit::FaultReset };

        /// Mask of the "Halt" bit in the "Controlword"
        constexpr Controlword Halt { Bit::Halt };
        
    };

    /**
     * @brief Namespace grouping mode-specific-bit-masks 
     *    of the "Controlword" object
     */
    namespace ModeSpecific {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::ModeSpecific;

        /// Mask of "mode-specific" fields in the "Controlword"
        constexpr Controlword Mask { Bit::Bit1 | Bit::Bit2 | Bit::Bit3 };
        
    };

    /**
     * @brief Namespace grouping velocity-mode-specific-bit-masks 
     *    of the "Controlword" object
     */
    namespace VelocityMode {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::VelocityMode;

        /// Mask of "velocity-mode-specific" fields in the "Controlword"
        constexpr Controlword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Controlword" after sending "Rfg Enable" command
        constexpr Controlword RfgEnable { Bit::RfgEnable };
        /// Mask of the "Controlword" after sending "Rfg Enable" command
        constexpr Controlword ReferenceGeneratorEnable { Bit::ReferenceGeneratorEnable };
        /// Mask of the "Controlword" after sending "Rfg Unlock" command
        constexpr Controlword RfgUnlock { Bit::RfgUnlock };
        /// Mask of the "Controlword" after sending "Rfg Unlock" command
        constexpr Controlword ReferenceGeneratorUnlock { Bit::ReferenceGeneratorUnlock };
        /// Mask of the "Controlword" after sending "Use Rfg reference" command
        constexpr Controlword RfgUseRef { Bit::RfgUseRef };
        /// Mask of the "Controlword" after sending "Use Rfg reference" command
        constexpr Controlword ReferenceGeneratorUseRef { Bit::ReferenceGeneratorUseRef };

    };

    /**
     * @brief Namespace grouping profiled-position-mode-specific-bit-masks 
     *    of the "Controlword" object
     */
    namespace ProfiledPositionMode {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::ProfiledPositionMode;

        /// Mask of "profiled-position-mode-specific" fields in the "Controlword"
        constexpr Controlword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Controlword" after sending "New setpoint" command
        constexpr Controlword NewSetpoint { Bit::NewSetpoint };
        /// Mask of the "Controlword" after sending "Change set immediately" command
        constexpr Controlword ChangeSetImmediately { Bit::ChangeSetImmediately };
        /// Mask of the "Controlword" after sending "Absolute relative switch" command
        constexpr Controlword AbsoluteRelativeSwitch { Bit::AbsoluteRelativeSwitch };

    };

    /**
     * @brief Namespace grouping homing-mode-specific-bit-masks 
     *    of the "Controlword" object
     */
    namespace HomingMode {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::HomingMode;

        /// Mask of "homing-mode-specific" fields in the "Controlword"
        constexpr Controlword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Controlword" after sending "Start" command
        constexpr Controlword Start { Bit::Start };

    };

    /**
     * @brief Namespace grouping interpolated-position-mode-specific-bit-masks 
     *    of the "Controlword" object
     */
    namespace InterpolatedPositionMode {

        /// Alias for bit-indexing enumeration
        using Bit = Controlword::InterpolatedPositionMode;

        /// Mask of "interpolated-position-mode-specific" fields in the "Controlword"
        constexpr Controlword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Controlword" after sending "Enable" command
        constexpr Controlword Enable { Bit::Enable };

    };

} // End namespace controlword

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
