/* ============================================================================================================================ *//**
 * @file       statusword.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 6:17:40 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures used to parse and construct 'Statusword' object
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_STATUSWORD_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_DEVICE_CONTROL_STATUSWORD_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// Private includes
#include "ethercat/utilities/named_bitset.hpp"
#include "ethercat/devices/elmo/registers.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* =========================================================== Statusword ========================================================= */

/**
 * @brief Parser structure for the "Statusword" object
 */
struct Statusword : public utilities::named_bitset<registers::STATUSWORD.type.get_bitsize()> {

public: /* ---------------------------------------------------- Public types ----------------------------------------------------- */

    /// Representation type of the statusword numeric variable
    using RepresentationType = common::types::traits::TypeRepresentation<registers::STATUSWORD.type.get_id()>;

public: /* -------------------------------------------------- Public constants --------------------------------------------------- */
   
    /// Size of the statusword numeric variable
    static constexpr std::size_t Bitsize = registers::STATUSWORD.type.get_bitsize();

    /**
     * @brief State-bit-index of the "Statusword" object
     */
    enum class State : std::size_t {
        ReadyToSwitchOn     = 0,
        SwitchedOn          = 1,
        OperationEnabled    = 2,
        Fault               = 3,
        VoltageEnabled      = 4,
        QuickStop           = 5,
        SwitchOnDisabled    = 6,
        Warning             = 7,
        Remote              = 9,
        TargetReached       = 10,
        InternalLimitActive = 11,
    };

    /**
     * @brief Mode-specific-bit-index of the "Statusword" object
     */
    enum class ModeSpecific : std::size_t {
        Bit1 = 12,
        Bit2 = 13,
    };

    /**
     * @brief Profiled-position-mode-specific-bit-index of the "Statusword" object
     */
    enum class ProfiledPositionMode : std::size_t {
        SetpointAck    = common::utilities::to_underlying(ModeSpecific::Bit1),
        FollowingError = common::utilities::to_underlying(ModeSpecific::Bit2),
    };

    /**
     * @brief Profiled-velocity-mode-specific-bit-index of the "Statusword" object
     */
    enum class ProfiledVelocityMode : std::size_t {
        Speed            = common::utilities::to_underlying(ModeSpecific::Bit1),
        MaxSlippageError = common::utilities::to_underlying(ModeSpecific::Bit2),
    };

    /**
     * @brief Homing-mode-specific-bit-index of the "Statusword" object
     */
    enum class HomingMode : std::size_t {
        HomingAttained = common::utilities::to_underlying(ModeSpecific::Bit1),
        HomingError    = common::utilities::to_underlying(ModeSpecific::Bit2),
    };

    /**
     * @brief Interpolated-position-mode-specific-bit-index of the "Statusword" object
     */
    enum class InterpolatedPositionMode : std::size_t {
        Active = common::utilities::to_underlying(ModeSpecific::Bit1),
    };
        
public: /* ----------------------------------------------------- Public ctors ---------------------------------------------------- */

    /// Forward constructors
    using utilities::named_bitset<Bitsize>::named_bitset;
    /// Wrap copy constructor
    constexpr Statusword(const utilities::named_bitset<Bitsize> &set) : utilities::named_bitset<Bitsize>{ set } {  }

public: /* --------------------------------------------------- Public operators -------------------------------------------------- */

    /// Forward assignment operator
    using utilities::named_bitset<Bitsize>::operator=;

    /// Wrap member operator of the bitset
    ETHERCAT_UTILITIES_WRAP_BITSET_MEMBER_OPERATORS(utilities::named_bitset<Bitsize>, Statusword);

public: /* --------------------------------------------------- Public operators -------------------------------------------------- */

    /**
     * @brief Converts Statusword to the integral constant representing the 
     *    "Statusword" object
     * 
     * @returns 
     *    statusword converted to integral constant
     */
    constexpr RepresentationType to_value() const {
        return utilities::named_bitset<Bitsize>::to_value<RepresentationType>();
    } 

};

/// Wrap binary operators of the bitset
ETHERCAT_UTILITIES_WRAP_BITSET_BINARY_OPERATORS(utilities::named_bitset<Statusword::Bitsize>, Statusword);

/* ======================================================= Statusword masks ====================================================== */

namespace statusword {

    /// Use custom OR operator for enumerations
    using utilities::operator|;

    /**
     * @brief Namespace grouping state-bits masks
     */
    namespace State {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::State;

        /// Mask of "State" fields in the "Statusword"
        constexpr Statusword Mask { 
            Bit::ReadyToSwitchOn    |
            Bit::SwitchedOn         |
            Bit::OperationEnabled   |
            Bit::Fault              |
            Bit::SwitchOnDisabled
        };

        /// Extended mask of "State" fields in the "Statusword"
        constexpr Statusword ExtendedMask { 
            Bit::ReadyToSwitchOn    |
            Bit::SwitchedOn         |
            Bit::OperationEnabled   |
            Bit::Fault              |
            Bit::QuickStop          |
            Bit::SwitchOnDisabled
        };
        
        /// Mask of bits that should be taken into account for "Not ready to switch on" state in the "Statusword"
        constexpr Statusword NotReadyToSwitchOnMask { Mask };
        /// Mask of the "Not ready to switch on" state in the "Statusword"
        constexpr Statusword NotReadyToSwitchOn {  };

        /// Mask of bits that should be taken into account for "Switched on disabled" state in the "Statusword"
        constexpr Statusword SwitchedOnDisabledMask { Mask };
        /// Mask of the "Switched on disabled" state in the "Statusword"
        constexpr Statusword SwitchedOnDisabled { Bit::SwitchOnDisabled };

        /// Mask of bits that should be taken into account for "Ready to switch on" state in the "Statusword"
        constexpr Statusword ReadyToSwitchOnMask { ExtendedMask };
        /// Mask of the "Ready to switch on" state in the "Statusword"
        constexpr Statusword ReadyToSwitchOn { Bit::ReadyToSwitchOn | Bit::QuickStop };

        /// Mask of bits that should be taken into account for "Switch on" state in the "Statusword"
        constexpr Statusword SwitchOnMask { ExtendedMask };
        /// Mask of the "Switch on" state in the "Statusword"
        constexpr Statusword SwitchOn { Bit::ReadyToSwitchOn | Bit::SwitchedOn | Bit::QuickStop };

        /// Mask of bits that should be taken into account for "Operation enabled" state in the "Statusword"
        constexpr Statusword OperationEnabledMask { ExtendedMask };
        /// Mask of the "Operation enabled" state in the "Statusword"
        constexpr Statusword OperationEnabled { Bit::ReadyToSwitchOn | Bit::SwitchedOn | Bit::OperationEnabled | Bit::QuickStop };

        /// Mask of bits that should be taken into account for "Quick stop active" state in the "Statusword"
        constexpr Statusword QuickStopActiveMask { ExtendedMask };
        /// Mask of the "Quick stop active" state in the "Statusword"
        constexpr Statusword QuickStopActive { Bit::ReadyToSwitchOn | Bit::SwitchedOn | Bit::OperationEnabled };

        /// Mask of bits that should be taken into account for "Fault reaction acitve" state in the "Statusword"
        constexpr Statusword FaultReactionAcitveMask { Mask };
        /// Mask of the "Fault reaction acitve" state in the "Statusword"
        constexpr Statusword FaultReactionAcitve { Bit::ReadyToSwitchOn | Bit::SwitchedOn | Bit::OperationEnabled | Bit::Fault };

        /// Mask of bits that should be taken into account for "Fault" state in the "Statusword"
        constexpr Statusword FaultMask { Mask };
        /// Mask of the "Fault" state in the "Statusword"
        constexpr Statusword Fault { Bit::Fault };
        
    };

    /**
     * @brief Namespace grouping mode-specific-bit-masks 
     *    of the "Statusword" object
     */
    namespace ModeSpecific {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::ModeSpecific;

        /// Mask of "mode-specific" fields in the "Statusword"
        constexpr Statusword Mask { Bit::Bit1 | Bit::Bit2 };
        
    };

    /**
     * @brief Namespace grouping profiled-position-mode-specific-bit-masks 
     *    of the "Statusword" object
     */
    namespace ProfiledPositionMode {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::ProfiledPositionMode;

        /// Mask of "profiled-position-mode-specific" fields in the "Statusword"
        constexpr Statusword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Statusword" after sending "Setpoint acknowledge" status
        constexpr Statusword SetpointAck { Bit::SetpointAck };
        /// Mask of the "Statusword" after sending "Following error" status
        constexpr Statusword FollowingError { Bit::FollowingError };
        
    };

    /**
     * @brief Namespace grouping profiled-velocity-mode-specific-bit-masks 
     *    of the "Statusword" object
     */
    namespace ProfiledVelocityMode {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::ProfiledVelocityMode;

        /// Mask of "profiled-velocity-mode-specific" fields in the "Statusword"
        constexpr Statusword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Statusword" after sending "Speed" status
        constexpr Statusword Speed { Bit::Speed };
        /// Mask of the "Statusword" after sending "Max slippage error" status
        constexpr Statusword MaxSlippageError { Bit::MaxSlippageError };

    };

    /**
     * @brief Namespace grouping homing-mode-specific-bit-masks 
     *    of the "Statusword" object
     */
    namespace HomingMode {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::HomingMode;

        /// Mask of "homing-mode-specific" fields in the "Statusword"
        constexpr Statusword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Statusword" after sending "Homing attained" status
        constexpr Statusword HomingAttained { Bit::HomingAttained };
        /// Mask of the "Statusword" after sending "Homing error" status
        constexpr Statusword HomingError { Bit::HomingError };

    };

    /**
     * @brief Namespace grouping interpolated-position-mode-specific-bit-masks 
     *    of the "Statusword" object
     */
    namespace InterpolatedPositionMode {

        /// Alias for bit-indexing enumeration
        using Bit = Statusword::InterpolatedPositionMode;

        /// Mask of "interpolated-position-mode-specific" fields in the "Statusword"
        constexpr Statusword Mask { ModeSpecific::Mask };
        
        /// Mask of the "Statusword" after sending "Active" status
        constexpr Statusword Active { Bit::Active };

    };

} // End namespace statusword

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
