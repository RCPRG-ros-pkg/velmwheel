/* ============================================================================================================================ *//**
 * @file       parameters
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Tuesday, 14th June 2022 4:04:53 pm
 * @project    engineering-thesis
 * @brief      Definitions of API implementing common routines related to ROS2 parameters system
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= *//

#ifndef __NODE_COMMON_PARAMETERS_PARAMETERS_H__
#define __NODE_COMMON_PARAMETERS_PARAMETERS_H__

/* =========================================================== Includes =========================================================== */

// Standard includes
#include <sstream>
// Private includes
#include "node_common/parameters.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {
namespace parameters {

/* ============================================================ Helpers =========================================================== */

namespace details {

    /**
     * @brief Helper function computing fully qualified name of the parameter
     * 
     * @param ns 
     *    parameter'a namespace
     * @param name 
     *    parameter'a name
     * @returns 
     *    fully qualified name of the parameter
     */
    static inline auto declare_parameter(std::string_view ns, std::string_view name) {

        std::stringstream fqn_name;

        // Append namespace to the string
        if(ns.size() > 0)
            fqn_name << ns << ".";

        // Append name to the string
        fqn_name << name;

        return fqn_name.str();
    }

}


/* ========================================================== Definitions ========================================================= */

template<typename T, std::size_t N>
auto declare_parameter(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor) {

    rcl_interfaces::msg::ParameterDescriptor param_description;

    // Prepare parameter's description
    param_description.description            = descriptor.description;
    param_description.additional_constraints = descriptor.additional_constraints;
    param_description.read_only              = descriptor.read_only;
    param_description.dynamic_typing         = descriptor.dynamic_typing;
    // Prepare parameter's range
    if(descriptor.range.has_value()) {

        // Fill floatingpoint range constraint
        if(std::holds_alternative<FloatingPointRange>(*descriptor.range)) {

            rcl_interfaces::msg::FloatingPointRange ros_range;
        
            // Get the range
            auto range = std::get<FloatingPointRange>(*descriptor.range);
            // Parse the range
            ros_range.from_value = range.min;
            ros_range.to_value   = range.max;
            ros_range.step       = range.step;
            // Write down the range
            param_description.floating_point_range.push_back(ros_range);

        } else if(std::holds_alternative<IntegerRange>(*descriptor.range)) {

            rcl_interfaces::msg::IntegerRange ros_range;
        
            // Get the range
            auto range = std::get<IntegerRange>(*descriptor.range);
            // Parse the range
            ros_range.from_value = range.min;
            ros_range.to_value   = range.max;
            ros_range.step       = range.step;
            // Write down the range
            param_description.integer_range.push_back(ros_range);
            
        }

    }
    
    // Make std::string from parameter's name to meet ROS API requirements
    std::string name = std::string{ descriptor.name };

    // Register the parameter without default value
    if(not descriptor.default_value.has_value()) {
        return node.declare_parameter(
            name,
            details::type_to_param_type<T>(),
            param_description
        );
    // Register the parameter with default value
    } else {
        return node.declare_parameter(
            name,
            rclcpp::ParameterValue{ details::default_value_to_runtime<T, N>(*descriptor.default_value) },
            param_description
        );
    }

}


template<typename T, std::size_t N>
auto declare_parameter(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor) {

    // Compute parameter's name
    auto fqn_name = details::declare_parameter(ns, descriptor.name);

    // Copy original descriptor
    ParamDescriptor<T, N> tmp_descriptor { descriptor };
    // Update parameter's name
    tmp_descriptor.name = fqn_name.c_str();

    return declare_parameter<T, N>(node, tmp_descriptor);
}


template<typename T>
std::optional<T> get_param(rclcpp::Node &node, std::string_view name) {

    // Assert that the supported type has been given for specialization
    static_assert(parameters::details::param_type_traits<T>::is_param_type, 
        "[node_common::parameters::get_param] Given T type is not compatible with supported ROS2 parameters types");

    rclcpp::Parameter param;
    

    // Make std::string from parameter's name to meet ROS API requirements
    std::string name_str = std::string{ name };

    /**
     * @brief Get the parameter (if parameter has not be initialized, treat it as unset)
     * @note The @ref rclcpp::exceptions::ParameterUninitializedException exception that
     *    is actually thrown by get_parameter() is not described as part of the throw set
     *    of the @ref get_param() method . For this reason catch any exception to be safe.
     */
    try {
        param = node.get_parameter(name_str);
    } catch (...) {
        return std::optional<T>{};
    }
    
    // Check if parameters has been set
    bool is_parameter_set = 
        (param.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET);

    std::optional<T> param_value;

    // Fill the value depending on the type, if the parameter is set
    if(is_parameter_set) {
             if constexpr( std::is_same_v<T, bool>                     ) param_value = param.as_bool();
        else if constexpr( std::is_same_v<T, int>                      ) param_value = param.as_int();
        else if constexpr( std::is_same_v<T, double>                   ) param_value = param.as_double();
        else if constexpr( std::is_same_v<T, std::string>              ) param_value = param.as_string();
        else if constexpr( std::is_same_v<T, std::vector<uint8_t>>     ) param_value = param.as_byte_array();
        else if constexpr( std::is_same_v<T, std::vector<bool>>        ) param_value = param.as_bool_array();
        else if constexpr( std::is_same_v<T, std::vector<int>>         ) param_value = param.as_integer_array();
        else if constexpr( std::is_same_v<T, std::vector<double>>      ) param_value = param.as_double_array();
        else if constexpr( std::is_same_v<T, std::vector<std::string>> ) param_value = param.as_string_array();
    }

    return param_value;
}


template<typename T>
std::optional<T> get_param(rclcpp::Node &node, std::string_view ns, std::string_view name) {

    // Compute parameter's name
    auto fqn_name = details::declare_parameter(ns, name);

    return get_param<T>(node, fqn_name);
}


template<typename T, std::size_t N>
std::optional<T> get_param(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor) {
    return get_param<T>(node, descriptor.name);
}


template<typename T, std::size_t N>
std::optional<T> get_param(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor) {

    // Compute parameter's name
    auto fqn_name = details::declare_parameter(ns, descriptor.name);

    // Copy original descriptor
    ParamDescriptor<T, N> tmp_descriptor { descriptor };
    // Update parameter's name
    tmp_descriptor.name = fqn_name.c_str();

    return get_param<T, N>(node, tmp_descriptor);
}


template<typename T, std::size_t N>
std::optional<T> declare_parameter_and_get(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor) {

    // Declare the parameter
    declare_parameter(node, descriptor);
    // Return value depending on whether parameter is set
    return get_param(node, descriptor);
}


template<typename T, std::size_t N>
std::optional<T> declare_parameter_and_get(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor) {

    // Compute parameter's name
    auto fqn_name = details::declare_parameter(ns, descriptor.name);

    // Copy original descriptor
    ParamDescriptor<T, N> tmp_descriptor { descriptor };
    // Update parameter's name
    tmp_descriptor.name = fqn_name.c_str();

    return declare_parameter_and_get<T, N>(node, tmp_descriptor);
}

/* ================================================================================================================================ */

} // End namespace parameters
} // End namespace node_common

#endif
