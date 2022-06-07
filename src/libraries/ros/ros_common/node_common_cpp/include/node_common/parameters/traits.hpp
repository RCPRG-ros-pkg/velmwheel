/* ============================================================================================================================ *//**
 * @file       traits.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Wednesday, 25th May 2022 1:30:02 am
 * @project    engineering-thesis
 * @brief      Definitions of private traits API required by the `parameters` module
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_PARAMETERS_TRAITS_H__
#define __NODE_COMMON_PARAMETERS_TRAITS_H__

/* =========================================================== Includes =========================================================== */

#include <array>
#include <vector>
#include <cstdint>
#include <string>
#include <type_traits>

/* =========================================================== Namespace ========================================================== */

namespace node_common {
namespace parameters {
namespace details {

/* ============================================================ Traits ============================================================ */

namespace details {

    /**
     * @brief Base type of the trait
     * @param type 
     *    type to specialize traits for
     * @param constexpr_type 
     *    underlying type used to represent the parameter in constexpr context
     */
    template<typename Type, std::size_t N = 0>
    struct param_type_traits {
        static constexpr bool is_param_type = false;
    };

    /**
     * @brief Helper macro defining specialization of the @ref node_common::parameters::details::details::param_type_traits 
     *    traits
     * @param type 
     *    type to specialize traits for
     * @param constexpr_friendly_type 
     *    underlying type used to represent the parameter in constexpr context
     */
    #define define_param_traits_specialization(type, constexpr_friendly_type) \
        template<>                                                            \
        struct param_type_traits<type> {                                      \
            static constexpr bool is_param_type = true;                       \
            using constexpr_type = constexpr_friendly_type;                   \
        }

    /**
     * @brief Helper macro defining specialization of the @ref node_common::parameters::details::details::param_type_traits 
     *    traits for array-typed parameters
     * @param type 
     *    type to specialize traits for
     * @param constexpr_friendly_type 
     *    underlying type used to represent the parameter in constexpr context
     */
    #define define_param_traits_array_specialization(type, constexpr_friendly_type) \
        template<std::size_t N>                                                     \
        struct param_type_traits<std::vector<type>, N> {                            \
            static constexpr bool is_param_type = true;                             \
            using constexpr_type = std::array<constexpr_friendly_type, N>;          \
        }

    // Specializations for non-array parameter types
    define_param_traits_specialization( bool,        bool         );
    define_param_traits_specialization( int,         int          );
    define_param_traits_specialization( double,      double       );
    define_param_traits_specialization( std::string, const char * );
    
    // Specializations for array parameter types
    define_param_traits_array_specialization( uint8_t,     uint8_t      );
    define_param_traits_array_specialization( bool,        bool         );
    define_param_traits_array_specialization( int,         int          );
    define_param_traits_array_specialization( double,      double       );
    define_param_traits_array_specialization( std::string, const char * );

    #undef define_param_traits_specialization
    #undef define_param_traits_array_specialization

}

/**
 * @brief Base trait type excluding types not supported by ROS2 parameters mechanism
 *    from resolution set of templates defined by the module and defining 
 *    constexpr-friendly version of the type representing the parameter
 * 
 * @tparam Type 
 *     C++ representation of the type of the parameters
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * 
 * @note Trait shall not be used when ROS2 supports (at least unoficially) C++20 
 *    which introduces compile-time-context memory allocation model and thhus constexpr
 *    version of @ref std::string and @ref std::vector
 */
template<typename Type, std::size_t N = 0>
struct param_type_traits : details::param_type_traits<Type, N> {};

/* ================================================================================================================================ */

} // End namespace details
} // End namespace parameters
} // End namespace node_common

#endif
