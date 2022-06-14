/* ============================================================================================================================ *//**
 * @file       types.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 9th May 2022 5:37:44 pm
 * @modified   Saturday, 11th June 2022 5:11:36 pm
 * @project    engineering-thesis
 * @brief      Declarations of common utilities related to EtherCAT (CoE) data types
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 * @note This header file should be included entirely by downstream files. In particular no files from ethercat/common/types/ 
 *    subdirectory should be included individually. It is required to provide proper includsion order
 */// ============================================================================================================================= */

#ifndef __ETHERCAT_COMMON_TYPES_H__
#define __ETHERCAT_COMMON_TYPES_H__

/* =========================================================== Includes =========================================================== */

#include "ethercat/common/types/common.hpp"
#include "ethercat/common/types/builtin.hpp"
#include "ethercat/common/types/structural.hpp"
#include "ethercat/common/types/type.hpp"
#include "ethercat/common/types/traits.hpp"

/* ==================================================== Implementation includes =================================================== */

#include "ethercat/common/types/builtin/builtin.hpp"
#include "ethercat/common/types/structural/structural.hpp"
#include "ethercat/common/types/type/type.hpp"

/* ================================================================================================================================ */

#endif
