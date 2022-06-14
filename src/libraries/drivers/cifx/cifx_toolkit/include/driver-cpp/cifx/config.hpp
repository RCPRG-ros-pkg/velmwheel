/* ============================================================================================================================ *//**
 * @file       config.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 22nd April 2022 3:26:59 am
 * @modified   Friday, 27th May 2022 3:50:44 pm
 * @project    engineering-thesis
 * @brief      Declarations of cpp-adapted configuration structures defined in the cifxDriver.h
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_CONFIG_H__
#define __CIFX_CONFIG_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <sys/mman.h>
#include <pthread.h>
// System includes
#include <string_view>
#include <optional>
#include <chrono>
// CIFX includes
#include "cifxDriver.h"
#include "cifXHWFunctions.h"

/* ========================================================== Namespaces ========================================================== */

namespace cifx {

/* ========================================================= Enumerations ========================================================= */

/**
 * @brief Log levels of the CIFX toolkit
 */
enum class LogLevel : unsigned long {
    Debug   = TRACE_LEVEL_DEBUG,
    Info    = TRACE_LEVEL_INFO,
    Warning = TRACE_LEVEL_WARNING,
    Error   = TRACE_LEVEL_ERROR
};

/**
 * @brief Scheduling policy for the thread
 */
enum class ThreadSchedPolicy : int {
    Other      = SCHED_OTHER,
    Fifo       = SCHED_FIFO,
    RoundRobin = SCHED_RR
};

/**
 * @brief Configuration of the driver thread (mirror of cifx::ThreadConfig defined to 
 *    decouple ROS interface from toolkit definitions) 
 */
struct ThreadConfig {

    /// CPU affinity (bitmask) of the IRQs-handling thread
    unsigned affinity { 0xFUL };
	/// Scheduling policy of the IRQs-handling thread
    ThreadSchedPolicy sched_policy { ThreadSchedPolicy::Other };
    /// Scheduling priority of the IRQs-handling thread
    int sched_priority { 0 };

};

/// Platform-specific thread ID type
using thread_id = pthread_t;

/* ================================================================================================================================ */

} // End namespace cifx

/* ================================================================================================================================ */

#endif
