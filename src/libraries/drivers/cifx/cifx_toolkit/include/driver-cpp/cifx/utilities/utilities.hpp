/* ============================================================================================================================ *//**
 * @file       utilities.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Monday, 14th February 2022 2:33:20 pm
 * @modified   Friday, 27th May 2022 3:54:03 pm
 * @project    engineering-thesis
 * @brief      Set of handy utilities for playing with CIFX driver library
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_UTILITIES_UTILITIES_H__
#define __CIFX_UTILITIES_UTILITIES_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/utilities.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* =========================================================== Functions ========================================================== */

void set_thread_params(const ThreadConfig &config) {
    set_thread_params(pthread_self(), config);
}

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
