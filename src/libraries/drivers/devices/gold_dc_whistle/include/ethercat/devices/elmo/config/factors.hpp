/* ============================================================================================================================ *//**
 * @file       factors.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Friday, 1st July 2022 2:48:17 pm
 * @project    engineering-thesis
 * @brief      Definitions of configuration constants & structures related to 'Factors' of the Elmo driver's Objects 
 *             Dictionary (OD)
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_FACTORS_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_FACTORS_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "ethercat/utilities/named_bitset.hpp"
#include "ethercat/devices/elmo/registers.hpp"
#include "ethercat/devices/elmo/config/common.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {
    
/* ========================================================== Definitions ========================================================= */

/**
 * @brief Possible options of the "Polarity" object
 */
enum class Polarity : TypeRepresentation<registers::POLARITY.type.get_id()> {
    Forward  = 0,
    Reversed = 1,
};

/**
 * @brief Structure describing polarity configruation of the driver
 */
struct PolarityConfig {
    
    /// Polarity of the motor's position
    Polarity position_polarity;
    /// Polarity of the motor's velocity
    Polarity velocity_polarity;
    
};

/**
 * @brief Parser structure for the "Digital inputes" object
 */
struct PolarityVector : public utilities::named_bitset<registers::POLARITY.type.get_bitsize()> {

    /**
     * @brief Bits-indexer type for the "Polarity" bit-vector object
     */
    enum Type : std::size_t {
        Position = 7,
        Velocity = 6
    };
    
    // Forward assignment operator
    using utilities::named_bitset<registers::POLARITY.type.get_bitsize()>::operator=;
    
};

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
