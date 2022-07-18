/* ============================================================================================================================ *//**
 * @file       cifxDevice.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 13th April 2021 8:03:28 am
 * @modified   Monday, 27th June 2022 5:12:00 pm
 * @project    engineering-thesis
 * @brief      (De)Initialization functions of the CIFX driver [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "cifXErrors.h"
#include "cifXToolkit.h"
#include "cifxDriver.h"
#include "OS_CosPolling.h"
#include "OS_Includes.h"

/* ========================================================== Static data ========================================================= */

/// True, when toolkit is initialized
static bool g_initialized = false;
/// True, when COS (Change of State) polling thread is running
static bool g_cos_polling = false;
/// File's context
static const char *context = "cifx_driver";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Initializes CIFX Toolkit
 * 
 * @param init
 *   initialization parameters of the driver
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    value on error
 */
static int init_toolkit(const CIFX_LINUX_INIT* init) {

	assert(init != NULL);
    
    // Reset trace level
    g_ulTraceLevel = 0;
	// Set the trace level
    switch(init->trace_level) {
        case TRACE_LEVEL_DEBUG:   g_ulTraceLevel = (TRACE_LEVEL_DEBUG | TRACE_LEVEL_INFO | TRACE_LEVEL_WARNING | TRACE_LEVEL_ERROR); break;
        case TRACE_LEVEL_INFO:    g_ulTraceLevel = (                    TRACE_LEVEL_INFO | TRACE_LEVEL_WARNING | TRACE_LEVEL_ERROR); break;
        case TRACE_LEVEL_WARNING: g_ulTraceLevel = (                                       TRACE_LEVEL_WARNING | TRACE_LEVEL_ERROR); break;
        case TRACE_LEVEL_ERROR:   g_ulTraceLevel = (                                                             TRACE_LEVEL_ERROR); break;
        default:
            xTraceError(context, "Invalid log level requested (%d)", init->trace_level);
            fflush(stdout);
            return -1;
    }
    
	// Display initialization message
	xTraceInfo(context, "Initializing toolkit...");
    
	// Initialize the toolkit
	const int32_t ec = cifXTKitInit();
	if(ec != CIFX_NO_ERROR)	{
		xTraceToolkitError(context, ec, "Could not initialize toolkit");
		return -1;
	}
    
	return 0;
}


/**
 * @brief Deinitializes CIFX Toolkit
 */
static void deinit_toolkit() {
	xTraceInfo(context, "Deinitializing toolkit...");
	cifXTKitDeinit();
	xTraceInfo(context, "Toolkit deinitialized...");
}


/**
 * @brief Initializes COS (Change of State) polling thread if requested
 * 
 * @param init
 *    driver's initialization parameters
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int init_cos_polling(const CIFX_LINUX_INIT* init) {

	assert(init != NULL);

	// Disable COS polling, if requested
	if(init->cos_polling_interval_ms == -1) {
		g_cos_polling = false;
		xTraceInfo(context, "NOT using COS polling");
		return 0;
	}
    
	xTraceInfo(context, "Initializing COS polling...");

	// Start COS polling thread
	if( xDriverCosPollingStart(init->cos_polling_interval_ms, &init->cos_polling_thread_params) == -1 )
		return -1;

	g_cos_polling = true;
	xTraceInfo(context, "COS polling initialized");

	return 0;
}


/**
 * @brief Stops COS (Change of State) polling thread if running
 */
static void deinit_cos_polling() {

	if(!g_cos_polling)
		return;

	xTraceInfo(context, "Deinitializing COS polling...");
	xDriverCosPollingStop();
	xTraceInfo(context, "COS polling deinitialized");
}

/* ================================================== Public functions (Toolkit) ================================================== */

int32_t xToolkitInit(const CIFX_LINUX_INIT* init) {

	// Check pointer's corectness
	if(init == NULL)
		return CIFX_INVALID_POINTER;
	
	// Check if Toolkit was already initialized
	if(g_initialized)
		return CIFX_TKT_ALREADY_INITIALIZED;
    
	// Toolkit initialization
	if(init_toolkit(init) == -1)
		return CIFX_TKT_INIT_ERROR;

	// COS polling thread initialization
	if(init_cos_polling(init) == -1) {
		deinit_toolkit();
		return CIFX_TKT_INIT_ERROR;
	}
    
	// Signal driver's initializtion finished
	g_initialized = true;

	return CIFX_NO_ERROR;
}


bool xToolkitIsInit() {
	return g_initialized;
}


void xToolkitDeinit() {
	
	if(!g_initialized)
		return;

	xTraceInfo(context, "Deinitializing...");

	// Deinitialize device
	xDeviceDeinit();

	// COS polling thread deinitialization	
	deinit_cos_polling();

	// Toolkit deinitialization
	deinit_toolkit();

	// Signal driver's deinitializtion finished
	g_initialized = false;

	xTraceInfo(context, "Deinitialized");
}

/* ========================================================== Global data ========================================================= */

/**
 * @brief Analogous of CIFX_ERROR_TO_DESCRtag for custom errors
 */
static struct CIFX_CUSTOM_ERROR_TO_DESCRtag
{
    /// Error code
    int32_t lError;
    /// Description string
    char* szErrorDescr;

} s_atCustomErrorToDescrTable[] =
#ifndef CIFX_TOOLKIT_NO_ERRORLOOKUP
{
    { CIFX_TKT_ALREADY_INITIALIZED,     "Toolkit has already been initialized"                                 },
    { CIFX_TKT_INIT_ERROR,              "Failed to initialize CIFX Toolkit"                                    },
    { CIFX_DEV_ALREADY_INITIALIZED,     "CIFX device has already been initialized"                             },
    { CIFX_DEV_TKT_NOT_INITIALIZED,     "CIFX device cannot be initialized as Toolkit has not been initialize" },
    { CIFX_DEV_INIT_ERROR,              "Failed to initialize CIFX Device"                                     }
#else
    { CIFX_NO_ERROR, "" },
#endif
};

/* ================================================= Public functions (Auxiliary) ================================================= */

#ifdef CIFX_TOOLKIT_PARAMETER_CHECK
  #define CHECK_POINTER(param) if ((void*)NULL == param) return CIFX_INVALID_POINTER;
#else
  #define CHECK_POINTER(param)
#endif

int32_t xDriverGetCustomErrorDescription(int32_t lError, char* szBuffer, uint32_t ulBufferLen) {
    
    // Assume that function will faile
    int32_t lRet = CIFX_FUNCTION_FAILED;

    int iIdx = 0;

    // Check if vali pointer has been passed
    CHECK_POINTER(szBuffer);
    
    // Iterate over lookup table
    for(iIdx = 0; iIdx < (int)(sizeof(s_atCustomErrorToDescrTable) / sizeof(s_atCustomErrorToDescrTable[0])); ++iIdx) {

        // If error code matches, return description
        if(s_atCustomErrorToDescrTable[iIdx].lError == lError) {

            // Copy description to the output buffer
            (void)OS_Strncpy(szBuffer, s_atCustomErrorToDescrTable[iIdx].szErrorDescr, ulBufferLen);
            // Set return code
            lRet = CIFX_NO_ERROR;
            // Escape loop
            break;
        }
    }

    return lRet;
}

/* ================================================================================================================================ */
