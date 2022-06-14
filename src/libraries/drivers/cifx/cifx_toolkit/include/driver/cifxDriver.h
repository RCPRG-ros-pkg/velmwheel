/* ============================================================================================================================ *//**
 * @file       cifxDriver.h
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 12th May 2022 7:10:11 pm
 * @modified   Friday, 27th May 2022 3:42:15 pm
 * @project    engineering-thesis
 * @brief      Main header of the CIFX driver
 * @note       At the moment CIFX/netX Toolkit's implementation supports only a single CIFX card with a PCI interface. Only
 *             Communication Channel 0 is supported. Driver assumes that the Toolkit is compiled in the DMA-mode (i.e. with
 *             CIFX_TOOLKIT_DMA macro defined)
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_DRIVER_H__
#define __CIFX_DRIVER_H__

/* =========================================================== Includes =========================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "netXAPI.h"
#include "cifXUser.h"
#include "cifXErrors.h"
#include "cifXToolkit.h"

/* ================================================================================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================ Macros ============================================================ */

/*
 * Additional error code compliant with 'cifxErrors.h'
 */

/// Error code returned when user requests repeted initializaiton of the toolkit
#define CIFX_TKT_ALREADY_INITIALIZED     ( (int32_t)0xC0001001L )
/// Error code returned due to toolkit initialization error
#define CIFX_TKT_INIT_ERROR              ( (int32_t)0xC0001002L )
/// Error code returned when user requests repeted initializaiton of the CIFX device
#define CIFX_DEV_ALREADY_INITIALIZED     ( (int32_t)0xC0001003L )
/// Error code returned due to toolkit being uninitialized
#define CIFX_DEV_TKT_NOT_INITIALIZED     ( (int32_t)0xC0001004L )
/// Error code returned due to CIFX device initialization error
#define CIFX_DEV_INIT_ERROR              ( (int32_t)0xC0001005L )

/* ======================================================== Data Structures ======================================================= */

typedef struct {

    /// CPU affinity (bitmask) of the IRQs-handling thread
    unsigned affinity;
	/// Scheduling policy of the IRQs-handling thread
    int sched_policy;
    /// Scheduling priority of the IRQs-handling thread
    int sched_priority;
	/// Scheduling parameters' inheritance mode of the IRQs-handling thread
	int sched_inheritance;
	
} CIFX_THREAD_INFO;

/**
 * @brief Driver-specific structure describing initial parameters of the CIFX device
 */
typedef struct {
	
	/// Index of the uioX device representing the CIFX card
	int uio_num;

	/// Device's name inside toolkit
	const char *name;
	
	/// IRQ-handling thread's parameters
	CIFX_THREAD_INFO irq_thread_params;

	/// Path to the bootloader file (default path used if NULL)
	const char *bootloader_file;
	/// Path to the firmware file (default path used if NULL)
	const char *firmware_file;
	/// Path to the card's configuration (e.g. ENI) file (default path used if NULL, no configuration file if "none")
	const char *config_file;
	
} CIFX_DEVICE_INIT;

/**
 * @brief Initialization structure for CIFX proprietary driver
 */
typedef struct {
	
	/// COS polling interval in milliseconds (-1 to disable)
	int cos_polling_interval_ms;
	/// COS-polling thread's parameters (taken into account if polling enabled)
	CIFX_THREAD_INFO cos_polling_thread_params;
	
	/// Tracing logs threshold of driver
	unsigned long trace_level;

} CIFX_LINUX_INIT;

/* ================================================== Public functions (Toolkit) ================================================== */

/**
 * @brief Initializes CIFX/netX Toolkit with given parameters
 * 
 * @param init
 * 	  toolkit's parameters
 * 
 * @retval CIFX_NO_ERROR
 *    on success
 * @retval "error code"
 *    on error
 * 
 * @see cifxErrors.h
 */
int xToolkitInit(const CIFX_LINUX_INIT* init);

/**
 * @retval true 
 *    if toolkit is initialized
 * @retval false 
 *    otherwise
 */
bool xToolkitIsInit();

/**
 * @brief Deinitializes CIFX/netX Toolkit
 */
void xToolkitDeinit();

/* ================================================ Public functions (CIFX Device) ================================================ */

/**
 * @brief Initializes CIFX device with given parameters
 * 
 * @param init
 * 	  toolkit's parameters
 * 
 * @retval CIFX_NO_ERROR 
 *    on success
 * @retval "error code" 
 *    on error
 * 
 * @see cifxErrors.h
 */
int xDeviceInit(const CIFX_DEVICE_INIT *init);

/**
 * @retval pointer 
 *    to the device registerred to the toolkit if it was already initialized
 * @retval NULL 
 *    otherwise
 */
PDEVICEINSTANCE xDeviceGet();

/**
 * @retval true 
 *    if device is initialized
 * @retval false 
 *    otherwise
 */
bool xDeviceIsInit();

/**
 * @brief Deinitializes CIFX device
 */
void xDeviceDeinit();

/* ================================================== Public function (Auxiliary) ================================================= */

/**
 * @brief Implementation of the @ref xDriverGetErrorDescription() for custom error codes
 * 
 * @param lError 
 *    Error to look up
 * @param szBuffer 
 *    Pointer to return data
 * @param ulBufferLen 
 *    Length of return buffer
 * 
 * @retval CIFX_NO_ERROR 
 *    on success
 * @retval code
 *    error code on error
 */
int32_t xDriverGetCustomErrorDescription(int32_t lError, char* szBuffer, uint32_t ulBufferLen);

/* ================================================================================================================================ */

#ifdef __cplusplus
}
#endif

/* ================================================================================================================================ */

#endif
