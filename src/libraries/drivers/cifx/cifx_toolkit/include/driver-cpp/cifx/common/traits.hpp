/* ============================================================================================================================ *//**
 * @file       traits.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 4th May 2022 6:18:45 pm
 * @modified   Friday, 27th May 2022 5:46:46 pm
 * @project    engineering-thesis
 * @brief      Set of common types traits
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_COMMON_TRAITS_H__
#define __CIFX_COMMON_TRAITS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <functional>
#include <type_traits>

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace common {
namespace traits {

/* ======================================================== Function traits ======================================================= */

namespace details {

    struct void_t { using type = void; };

    template<typename T> 
    struct function_traits;  

    template<typename R, typename ...Args> 
    struct function_traits<std::function<R(Args...)>>
    {
        static const size_t nargs = sizeof...(Args);

        typedef R result_type;

        template <size_t i>
        struct arg
        {
            typedef typename std::tuple_element<i, std::tuple<Args...>>::type type;
        };

        template <size_t i>
        struct arg_or_void
        {
            typedef typename std::conditional<
                i < sizeof...(Args),
                std::tuple_element<i, std::tuple<Args...>>,
                void_t
            >::type::type type;
        };

    };

} // End namespace details

/**
 * @brief Helper traits used to obtain types related to the specialization of
 *    the @ref std::function template
 */
template<typename T> 
struct function_traits : public details::function_traits<T> { };

/* =========================================================== Removals =========================================================== */

namespace details {

    template<class T>
    struct remove_cvref {
        typedef std::remove_cv_t<std::remove_reference_t<T>> type;
    };

} // End namespace details

/**
 * @brief Helper trait removing cv and reference attribtues from the type @tparam T
 * 
 * @tparam T 
 *   type to be modified
 */
template<class T>
struct remove_cvref : public details::remove_cvref<T> { };

/// Helper alias for remove_cvref<T>::type
template<class T>
using remove_cvref_t = typename remove_cvref<T>::type;

/* ================================================================================================================================ */

} // End namespace traits
} // End namespace common
} // End namespace cifx

/* ================================================================================================================================ */

#endif
