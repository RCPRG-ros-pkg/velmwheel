/* ============================================================================================================================ *//**
 * @file       OS_Common.h
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:46:06 pm
 * @project    engineering-thesis
 * @brief      Common functions used in the implementation of OS-dependent part of the CIFX/netX Toolkit [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_COMMON_H__
#define __CIFX_DRIVER_COMMON_H__

/* =========================================================== Includes =========================================================== */

#include <stdint.h>
#include <time.h>

/* ======================================================= Public functions ======================================================= */

/**
 * @returns 
 *    Linux-specific time structure filled with the current time posessed from the RT-clock
 */
struct timespec get_clock_time() ;

/**
 * @brief Adds requested number of [ms] to the time structure
 * 
 * @param ts
 *    time structure to be modified
 * @param msec
 *    number of [ms] to be added
 * @returns
 *    modified copy of the @p ts
 */
struct timespec timespec_add_ms(const struct timespec* ts, uint32_t msec);

/**
 * @brief Set scheduling policy to the pthread's attributes
 * 
 * @param attr
 *    attributes to be modified
 * @param sched_policy
 *    scheduling policy to be set
 * @param context
 *    string representing context of the calling function
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
int set_thread_sched_policy(pthread_attr_t* attr, int sched_policy, const char* context);

/**
 * @brief Set scheduling priority to the pthread's attributes
 * 
 * @param attr
 *    attributes to be modified
 * @param sched_prior
 *    scheduling policy to be set
 * @param context
 *    string representing context of the calling function
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
int set_thread_sched_priority(pthread_attr_t* attr, int sched_prior, const char* context);

/**
 * @brief Set scheduling parameter's inheritance to the pthread's attributes 
 * 
 * @param attr
 *    attributes to be modified
 * @param inheritsched
 *    scheduling properties' inheritance
 * @param context
 *    string representing context of the calling function
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 * 
 * @see pthread_attr_setinheritsched
 */
int set_thread_sched_inheritance(pthread_attr_t* attr, int inheritsched, const char* context);

/**
 * @brief Set affinity (assignement of the thread to the subset of CPUs present in system) 
 *    to the pthread's attributes
 * 
 * @param attr
 *    attributes to be modified
 * @param affinity
 *    bit-mask representing CPUs subset
 * @param context
 *    string representing context of the calling function
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 * 
 * @see pthread_setaffinity_np
 */
int set_thread_affinity(pthread_attr_t* attr, unsigned affinity, const char* context);

/**
 * @brief: Destroy pthread's attributes
 * 
 * @param attr
 *    attributes to be destroyed
 */
void destroy_thread_attr(pthread_attr_t* attr);

/* ================================================================================================================================ */

#endif
