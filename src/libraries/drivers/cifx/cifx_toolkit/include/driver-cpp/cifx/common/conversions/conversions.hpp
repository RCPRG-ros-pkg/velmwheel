/* ============================================================================================================================ *//**
 * @file       conversions.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 22nd April 2022 3:26:59 am
 * @modified   Friday, 27th May 2022 3:42:17 pm
 * @project    engineering-thesis
 * @brief      Definitions of private c-cpp conversions functions
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_COMMON_CONVERSIONS_CONVERSIONS_H__
#define __CIFX_COMMON_CONVERSIONS_CONVERSIONS_H__

/* =========================================================== Includes =========================================================== */

// Private includes
#include "cifx/common/conversions.hpp"
#include "cifx/common/enum.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace conversions {

/* ========================================================== Conversions ========================================================= */

unsigned long to_c(LogLevel log_level) {
    return static_cast<unsigned long>(log_level);
}


int to_c(ThreadSchedPolicy thread_policy) {
    return static_cast<int>(thread_policy);
}


CIFX_THREAD_INFO to_c(ThreadConfig thread_info) {
    return CIFX_THREAD_INFO{
        .affinity          = thread_info.affinity,
        .sched_policy      = common::to_underlying(thread_info.sched_policy),
        .sched_priority    = thread_info.sched_priority,
        .sched_inheritance = PTHREAD_EXPLICIT_SCHED
    };
}

/* ================================================================================================================================ */

} // End namespace conversions
} // End namespace cifx

/* ================================================================================================================================ */

#endif
