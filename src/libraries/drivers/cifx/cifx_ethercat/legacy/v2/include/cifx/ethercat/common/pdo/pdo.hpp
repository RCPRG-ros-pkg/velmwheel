/* ============================================================================================================================ *//**
 * @file       pdo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 27th April 2022 1:02:55 pm
 * @modified   Wednesday, 27th April 2022 4:37:10 pm
 * @project    engineering-thesis
 * @brief      Definition of function templates of defined in ifx/ethercat/common/pdo.hpp header
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_COMMON_PDO_PDO_H__
#define __CIFX_ETHERCAT_COMMON_PDO_PDO_H__

/* =========================================================== Includes =========================================================== */

#include "cifx/ethercat/common/pdo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace pdo {

/* ========================================================== Definitions ========================================================= */

template<DataType type>
Value make_value(data_type_to_value_type<type> value) {
    return Value{ std::in_place_index<data_type_to_index<type>>, value };
}


template<DataType type>
void set_value(Value &pdo_value, data_type_to_value_type<type> value) {
    pdo_value = make_value<type>(value);
}

/* ================================================================================================================================ */

} // End namespace pdo
} // End namespace ethercat
} // End namespace cifx

#endif
