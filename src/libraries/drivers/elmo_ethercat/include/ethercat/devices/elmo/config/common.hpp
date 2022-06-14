/* ============================================================================================================================ *//**
 * @file       common.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 2nd June 2022 12:53:22 pm
 * @modified   Monday, 13th June 2022 5:41:05 pm
 * @project    engineering-thesis
 * @brief      Definitions of common entities ofr configuration module
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_DEVICES_ELMO_CONFIG_COMMON_H__
#define __ETHERCAT_DEVICES_ELMO_CONFIG_COMMON_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <cstdint>
// Private includes
#include "ethercat.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace ethercat::devices::elmo::config {

/* ============================================================= Types ============================================================ */

/// Alias for representation type of the given EtherCAt type
template<auto type_id> using TypeRepresentation = common::types::traits::TypeRepresentation<type_id>;

/* ================================================================================================================================ */

} // End namespace ethercat::devices::elmo::config

/* ================================================================================================================================ */

#endif
