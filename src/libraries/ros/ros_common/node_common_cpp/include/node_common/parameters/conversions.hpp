/* ============================================================================================================================ *//**
 * @file       conversions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Wednesday, 25th May 2022 12:38:50 am
 * @project    engineering-thesis
 * @brief      Definitions of private conversions API required by the `parameters` module
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_PARAMETERS_CONVERSIONS_H__
#define __NODE_COMMON_PARAMETERS_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

#include "node_common/parameters/traits.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {
namespace parameters {
namespace details {

/* ======================================================== Helper traits ========================================================= */

namespace details 
{
    template <typename T, template <typename...> typename Template>
    struct is_specialization_of : std::false_type {};
    
    template <template <typename...> typename Template, typename... Args>
    struct is_specialization_of<Template<Args...>, Template> : std::true_type {};
}

/**
 * @brief Trait checking whether the type is a specialization a class template
 * @tparam T 
 *    class to be verified
 * @tparam Template 
 *    template to be compared against
 */
template <typename T, template <typename...> typename Template>
struct is_specialization_of : 
    details::is_specialization_of<T, Template> {};

/// @brief Alias for @ref is_specialization_of
template <typename T, template <typename...> typename Template>
inline constexpr bool is_specialization_of_v = 
    is_specialization_of<T, Template>::value;
    
/* ======================================================= Helper functions ======================================================= */

/**
 * @brief Converts constexpr-friendly default value of the parameter to the runtime representation
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 * @param default_value 
 * @return auto 
 */
template<typename T, std::size_t N>
static inline auto default_value_to_runtime(const typename param_type_traits<T, N>::constexpr_type &default_value) {
    

    // For array-type parameters fill the array element-by-element
    if constexpr (is_specialization_of_v<T, std::vector>) {
        
        T runtime_representation;

        // Prepare memory for the elements
        runtime_representation.reserve(default_value.size());
        // Fill the vector
        for(auto &elem : default_value)
            runtime_representation.push_back(elem);

        return runtime_representation;

    // For non-array-type parameters just return the default value
    } else 
        return default_value;
}

/**
 * @brief Converts C++ type to the ROS2 parameter type constant
 * 
 * @tparam T 
 *    C++ type of the parameter
 * @returns 
 *    corresponding constant from the @ref rclcpp::ParameterType enumeration
 */
template<typename T>
constexpr rclcpp::ParameterType type_to_param_type() {
         if constexpr( std::is_same_v<T, bool>                     ) return rclcpp::ParameterType::PARAMETER_BOOL;
    else if constexpr( std::is_same_v<T, int>                      ) return rclcpp::ParameterType::PARAMETER_INTEGER;
    else if constexpr( std::is_same_v<T, double>                   ) return rclcpp::ParameterType::PARAMETER_DOUBLE;
    else if constexpr( std::is_same_v<T, std::string>              ) return rclcpp::ParameterType::PARAMETER_STRING;
    else if constexpr( std::is_same_v<T, std::vector<uint8_t>>     ) return rclcpp::ParameterType::PARAMETER_BYTE_ARRAY;
    else if constexpr( std::is_same_v<T, std::vector<bool>>        ) return rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
    else if constexpr( std::is_same_v<T, std::vector<int>>         ) return rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    else if constexpr( std::is_same_v<T, std::vector<double>>      ) return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
    else if constexpr( std::is_same_v<T, std::vector<std::string>> ) return rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
    else                                                             return rclcpp::ParameterType::PARAMETER_NOT_SET;
}

/* ================================================================================================================================ */

} // End namespace details
} // End namespace parameters
} // End namespace node_common

#endif
