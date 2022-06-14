/* ============================================================================================================================ *//**
 * @file       OS_CosPolling.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es COS (Change of State) polling functions
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_COS_POLLING_H__
#define __CIFX_DRIVER_COS_POLLING_H__

/* ========================================================= Declarations ========================================================= */

/**
 * @brief Configures and runs a new COS thread with the given parameters
 * 
 * @param interval_ms
 *    polling interval in ms
 * @param thread_params
 *    polling thread's parameters
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
int xDriverCosPollingStart(unsigned long interval_ms, const CIFX_THREAD_INFO* thread_params);

/**
 * @brief Terminates the COS thread and joins it with the calling one
 */
void xDriverCosPollingStop();

/* ================================================================================================================================ */

#endif
