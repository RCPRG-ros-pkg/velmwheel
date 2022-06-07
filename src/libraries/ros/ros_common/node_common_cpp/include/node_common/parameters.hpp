/* ============================================================================================================================ *//**
 * @file       communication.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 16th March 2022 4:37:20 pm
 * @modified   Tuesday, 31st May 2022 8:35:00 am
 * @project    engineering-thesis
 * @brief      Declarations of API implementing common routines related to ROS2 parameters system
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __NODE_COMMON_PARAMETERS_H__
#define __NODE_COMMON_PARAMETERS_H__

/* =========================================================== Includes =========================================================== */

// Standrd includes
#include <optional>
#include <variant>
#include <string_view>
// Private includes
#include "rclcpp/rclcpp.hpp"
#include "node_common/parameters/traits.hpp"
#include "node_common/parameters/conversions.hpp"

/* =========================================================== Namespace ========================================================== */

namespace node_common {

/**
 * @brief Set of wrappers related to ROS parameters
 */
namespace parameters {

/* ============================================================= Types ============================================================ */

/**
 * Helper constexpr-friendly representation of the valid range of the parameter
 */
template<typename T>
struct Range {
    
    /// Minimal acceptable value (inclusive)
    T min { };
    /// Maximal acceptable value (inclusive)
    T max { };
    /// Step size determining acceptable values between minimum and maximum
    T step { };
    
};

/// Representation of the integer-type range
using IntegerRange = Range<int>;
/// Representation of the floating-point-type range
using FloatingPointRange = Range<double>;

/**
 * @brief Compact, constexpr-friendly representation of the ROS2 node's parameter
 * 
 * @tparam T 
 *    type of the parameter (it's validity is not checked by the API)
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * 
 * @note The structure is designed to be used with @ref rclcpp::Node::declare_parameter
 *    mehtod. As so it assumes that @a name and @a type fields of the 
 *    @ref rcl_interfaces::msg::ParameterDescriptor are not used by the ROS2 API (according 
 *    to [1]).
 * @note N argument shall not be used when ROS2 supports (at least unoficially) C++20 
 *    which introduces compile-time-context memory allocation model and thhus constexpr
 *    version of std::string and std::vector
 * 
 * @see [1] https://answers.ros.org/question/378427/how-is-the-name-field-of-parameterdescriptor-used-by-declare_parameter/
 * 
 * @todo Switch to concepts when ROS2 supports C++20 (at least it compiles with)
 */
template<typename T, std::size_t N = 0>
struct ParamDescriptor {

    // Assert that the supported type has been given for specialization
    static_assert(parameters::details::param_type_traits<T, N>::is_param_type, 
        "[node_common::parameters::ParamDescriptor] Given T type is not compatible with supported ROS2 parameters types");

public:

    /// Helper alias naming type used to represent parameters' constraints
    using Constraints = std::optional<std::variant<IntegerRange, FloatingPointRange>>;

public:
    
    /// Name of the parameter
    std::string_view name {  };
    
    /// Access right of the parameter
    bool read_only { false };
    /// If true, the parameter is allowed to change type.
    bool dynamic_typing { false };

    /// Default value of the parameter
    std::optional<typename details::param_type_traits<T, N>::constexpr_type> default_value { };

    /// Range constraints for the parameter
    Constraints range { };

    /// Description of the parameter
    std::string_view description {  };
    
    /**
     * Additional constraints of the parameter; plain English description of additional constraints which
     * cannot be expressed with the available constraints, e.g. "only prime numbers". By convention, this
     * should only be used to clarify constraints which cannot be completely expressed with the parameter
     * constraints.
     */
    std::string_view additional_constraints {  };

};


/**
 * @brief Declares parameter described by the @p descriptor in the @p node
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param node 
 *    reference to the node for which the parameter has to be registered for
 * @param descriptor 
 *    description of the parameter
 */
template<typename T, std::size_t N = 0>
inline auto declare_parameter(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor);

/**
 * @brief Declares parameter described by the @p descriptor in the @p node
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param ns 
 *    parameter's namespace
 * @param node 
 *    reference to the node for which the parameter has to be registered for
 * @param descriptor 
 *    description of the parameter
 */
template<typename T, std::size_t N = 0>
inline auto declare_parameter(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor);

/**
 * @brief Reads value of the parameter declared in the @p node with the given @p descriptor
 * 
 * @tparam T 
 *    type of the parameter
 * @param node 
 *    node that the parameter is registered for
 * @param name 
 *    name of the parameter
 * 
 * @retval value
 *    current value of the parameter if it was set
 * @retval empty
 *    empty optional if parameter has not been set
 * 
 * @throws rclcpp::exceptions::ParameterNotDeclaredException
 *    if the parameter has not been declared and undeclared parameters are not allowed
 */
template<typename T>
inline std::optional<T> get_param(rclcpp::Node &node, std::string_view name);

/**
 * @brief Reads value of the parameter declared in the @p node with the given @p descriptor
 * 
 * @tparam T 
 *    type of the parameter
 * @param node 
 *    node that the parameter is registered for
 * @param ns 
 *    parameter's namespace
 * @param name 
 *    name of the parameter
 * 
 * @retval value
 *    current value of the parameter if it was set
 * @retval empty
 *    empty optional if parameter has not been set
 * 
 * @throws rclcpp::exceptions::ParameterNotDeclaredException
 *    if the parameter has not been declared and undeclared parameters are not allowed
 */
template<typename T>
inline std::optional<T> get_param(rclcpp::Node &node, std::string_view ns, std::string_view name);

/**
 * @brief Reads value of the parameter declared in the @p node with the given @p descriptor
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param node 
 *    node that the parameter is registered for
 * @param descriptor 
 *    descriptor used to register the parameter
 * 
 * @retval value
 *    current value of the parameter if it was set
 * @retval empty
 *    empty optional if parameter has not been set
 * 
 * @throws rclcpp::exceptions::ParameterNotDeclaredException
 *    if the parameter has not been declared and undeclared parameters are not allowed
 */
template<typename T, std::size_t N = 0>
inline std::optional<T> get_param(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor);

/**
 * @brief Reads value of the parameter declared in the @p node with the given @p descriptor
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param node 
 *    node that the parameter is registered for
 * @param ns 
 *    parameter's namespace
 * @param descriptor 
 *    descriptor used to register the parameter
 * 
 * @retval value
 *    current value of the parameter if it was set
 * @retval empty
 *    empty optional if parameter has not been set
 * 
 * @throws rclcpp::exceptions::ParameterNotDeclaredException
 *    if the parameter has not been declared and undeclared parameters are not allowed
 */
template<typename T, std::size_t N = 0>
inline std::optional<T> get_param(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor);

/**
 * @brief Declares parameter described by the @p descriptor in the @p node and reads it from the 
 *    parameters server
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param node 
 *    reference to the node for which the parameter has to be registered for
 * @param descriptor 
 *    description of the parameter
 */
template<typename T, std::size_t N = 0>
inline std::optional<T> declare_parameter_and_get(rclcpp::Node &node, const ParamDescriptor<T, N>& descriptor);

/**
 * @brief Declares parameter described by the @p descriptor in the @p node and reads it from the 
 *    parameters server
 * 
 * @tparam T 
 *    type of the parameter
 * @tparam N 
 *    size of the constexpr default value of the array-type parameters (unused for 
 *    non-array-type parameters)
 * @param node 
 *    reference to the node for which the parameter has to be registered for
 * @param ns 
 *    parameter's namespace
 * @param descriptor 
 *    description of the parameter
 */
template<typename T, std::size_t N = 0>
inline std::optional<T> declare_parameter_and_get(rclcpp::Node &node, std::string_view ns, const ParamDescriptor<T, N>& descriptor);

/* ================================================================================================================================ */

} // End namespace parameters
} // End namespace node_common
    
/* ==================================================== Implementation includes =================================================== */

#include "node_common/parameters/parameters.hpp"

/* ================================================================================================================================ */

#endif
