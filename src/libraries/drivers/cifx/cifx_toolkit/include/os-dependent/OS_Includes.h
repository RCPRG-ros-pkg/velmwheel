/* ============================================================================================================================ *//**
 * @file       OS_Includes.h
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 15th June 2022 1:34:29 pm
 * @project    engineering-thesis
 * @brief      Declarations of common types and routines utilized by the implementation of OS-specific elements of the 
 *             CIFX/netX C Toolkit
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_OS_INCLUDES_H__
#define __CIFX_DRIVER_OS_INCLUDES_H__

/* =========================================================== Includes =========================================================== */

#include <pthread.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include "cifXHWFunctions.h"

/* ============================================================ Macros ============================================================ */

// Compiler-specific handler for unused function's parameters
#define UNREFERENCED_PARAMETER(a) (a=a)

// Logging colours
#define LOG_DEBUG_COL   "\033[0;37m"
#define LOG_INFO_COL    "\033[0;32m"
#define LOG_WARNING_COL "\033[0;33m"
#define LOG_ERROR_COL   "\033[0;31m"
#define LOG_END_COL     "\033[0m"

// Size of the trace's snrptin's buffer
#define TRACE_BUF_SIZE 256

/* ========================================================= Declarations ========================================================= */

/**
 * @brief Implementation of the tracing function used e.g. by USER_Trace()
 * 
 * @param ulTraceLevel 
 *    trace level
 * @param format 
 *    string format (printf-like)
 * @param args
 *    format-dependent arguments
 */
void xTraceVa(uint32_t trace_level, const char* format, va_list args);

/**
 * @brief Implementation of the tracing function used e.g. by USER_Trace()
 * 
 * @param ulTraceLevel 
 *    trace level
 * @param format 
 *    string format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTrace(uint32_t trace_level, const char* format, ...);

/**
 * @brief Prints debug message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceDebugVa(const char *context, const char *format, va_list arg);

/**
 * @brief Prints debug message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceDebug(const char *context, const char *format, ...);

/**
 * @brief Prints information message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceInfoVa(const char *context, const char *format, va_list arg);

/**
 * @brief Prints information message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceInfo(const char *context, const char *format, ...);

/**
 * @brief Prints warning message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceWarnVa(const char *context, const char *format, va_list arg);

/**
 * @brief Prints warning message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceWarn(const char *context, const char *format, ...);

/**
 * @brief Prints error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceErrorVa(const char *context, const char *format, va_list arg);

/**
 * @brief Prints error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceError(const char *context, const char *format, ...);

/**
 * @brief Prints toolkit error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param ec 
 *   toolkit error code
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceToolkitErrorVa(const char *context, int32_t ec, const char *format, va_list arg);

/**
 * @brief Prints toolkit error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param ec 
 *   toolkit error code
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceToolkitError(const char *context, int32_t ec, const char *format, ...);

/**
 * @brief Prints system error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param ec 
 *   system error code
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceSystemErrorVa(const char *context, int ec, const char *format, va_list arg);

/**
 * @brief Prints system error message to the stdout
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param ec 
 *   system error code
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceSystemError(const char *context, int ec, const char *format, ...);

/**
 * @brief Prints global system error message to the stdout (errno-based)
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param arg
 *    format-dependent arguments
 */
void xTraceGlobalSystemErrorVa(const char *context, const char *format, va_list arg);

/**
 * @brief Prints global system error message to the stdout (errno-based)
 * 
 * @param context 
 *    message's context; not used when NULL
 * @param format 
 *    message's format (printf-like)
 * @param ...
 *    format-dependent arguments
 */
void xTraceGlobalSystemError(const char *context, const char *format, ...);

/* ========================================================= Declarations ========================================================= */

// Libuio device's structure
struct uio_info_t;

// CIFX Toolkit's device structure
struct DEVICEINSTANCEtag;
typedef struct DEVICEINSTANCEtag DEVICEINSTANCE;

/* ======================================================== Data structures ======================================================= */

/**
 * @brief OS-specific data associated with the CIFX device
 */
typedef struct {

	/// Identificator of the thread reading IRQs from the CIFX device
    pthread_t irq_thread; 

    /// Set to non-zero value by the main thread when IRQs-handling thread should be terminated
    int irq_terminate;
    /// Scheduling policy of the IRQs-handling thread
    int irq_sched_policy;
    /// Scheduling priority of the IRQs-handling thread
    int irq_sched_priority;
    /// Scheduling parameters' inheritance mode of the IRQs-handling thread
	int irq_sched_inheritance;
    /// CPU affinity (bitmask) of the IRQs-handling thread
    unsigned irq_affinity;

	///  Pointer to the CIFX device structure (toolkit-specific)
    DEVICEINSTANCE* dev_instance;

	/// UIO structure (libuio-specific) associated with the device
    struct uio_info_t* uio_device;

} CIFX_OSDEPENDENT;

/* ================================================================================================================================ */

#endif
