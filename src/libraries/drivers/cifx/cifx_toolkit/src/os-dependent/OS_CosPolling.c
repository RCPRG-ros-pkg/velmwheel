/** ==================================================================================================================================
 * @file       OS_CosPolling.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Friday, 20th May 2022 8:12:30 pm
 * @project    engineering-thesis
 * @brief     CIFX'es COS (Change of State) polling functions [implementation]
 *    
 *    
 * @note Separate COS polling thread is used only in device's initialization and de-initialization phase. During normal
 *    operation and interrupt mode is used
 * @copyright Krzysztof Pierczyk © 2022
 * ================================================================================================================================ */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cifxDriver.h"
#include "OS_Common.h"
#include "OS_Includes.h"
#include "OS_Dependent.h"
#include "OS_CosPolling.h"

/* ========================================================== Static data ========================================================= */

/// True if COS (Change of State) polling thread was started
static bool g_started = false;
/// True if COS (Change of State) polling thread terminated
static bool g_poll_terminate;
/// Interval of the polling
static unsigned long g_poll_interval_ms;
/// Thread structure
static pthread_t g_poll_thread;
/// Context string
static const char *context = "os_cos_polling";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Change of State (COS) polling routine
 * 
 * @param arg
 *    thread's arguments
 * 
 * @returns 
 *    @c NULL
 */
static void* do_polling(void* arg) {

	(void) (arg);

	// Poll COS inwith the configured period
	while(!g_poll_terminate) {
		OS_Sleep(g_poll_interval_ms);
		cifXTKitCyclicTimer();
	}

	return NULL;
}


/**
 * @brief Initialize pthread's attributes for COS polling thread
 * 
 * @param attr
 *    attributes to be initialized
 * @param sched_policy
 *    scheduling policy
 * @param sched_prio
 *    scheduling priority
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int init_poll_thread_attr(pthread_attr_t* attr, const CIFX_THREAD_INFO* thread_params) {
	
	assert(attr != NULL);

	// Print debug info
	xTraceInfo(context, "Initializing polling thread attributes...");

	// Initialize object
	const int ec = pthread_attr_init(attr);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not init polling thread attributes");
		return -1;
	}

	// Set thread's attributes
	if(set_thread_sched_policy(attr, thread_params->sched_policy, context)           < 0 || 
	   set_thread_sched_priority(attr, thread_params->sched_priority, context)       < 0 || 
	   set_thread_sched_inheritance(attr, thread_params->sched_inheritance, context) < 0 ||
	   set_thread_affinity(attr, thread_params->affinity, context)                   < 0
	) {
		destroy_thread_attr(attr);
		return -1;
	}

	// Print debug info
	xTraceInfo(context, "Polling thread attributes initialized");

	return 0;
}


/**
 * @brief Creates and run a COS (change of State) polling thread
 * 
 * @param thread_attr
 *    thread's attributes
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int create_poll_thread(const pthread_attr_t* thread_attr) {

	assert(thread_attr != NULL);
	
	// Print debug info
	xTraceInfo(context, "Creating polling thread...");

	// Ensure the thread is not running already
	assert(pthread_detach(g_poll_thread) != 0);

	// Create a new thread
	const int ec = pthread_create(&g_poll_thread, thread_attr, &do_polling, NULL);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not create polling thread");
		return -1;
	}

	// Print debug info
	xTraceInfo(context, "Polling thread created");

	return 0;
}


/**
 * @brief Joins the COS polling thread with the calling thread
 */
static void join_poll_thread() {

	// Print debug info
	xTraceInfo(context, "Joining polling thread...");

	void** thread_return = NULL;
	const int ec = pthread_join(g_poll_thread, thread_return);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not join polling thread");
	}

	// Print debug info
	xTraceInfo(context, "Joined polling thread");
}

/* ======================================================= Public functions ======================================================= */

int xDriverCosPollingStart(unsigned long interval_ms, const CIFX_THREAD_INFO* thread_params) {

	assert(!g_started);

	// Printinf debug info
	xTraceInfo(context, "Starting...");

	// Configure thread's attributes
	pthread_attr_t thread_attr;
	if(init_poll_thread_attr(&thread_attr, thread_params) == -1)	{
		return -1;
	}

	// Update global informations about the COS thread
	g_poll_interval_ms = interval_ms;
	g_poll_terminate = false;

	// Create and run the thread
	if(create_poll_thread(&thread_attr) == -1) {
		destroy_thread_attr(&thread_attr);
		return -1;
	}

	// Free attributes' memory
	destroy_thread_attr(&thread_attr);

	// Mark thread as started
	g_started = true;

	// Printinf debug info
	xTraceInfo(context, "Started");
	return 0;
}


void xDriverCosPollingStop() {
	
	assert(g_started);
	assert(!g_poll_terminate);
	
	// Print debug info
	xTraceInfo(context, "Stopping...");

	g_poll_terminate = true;
	join_poll_thread();

	g_started = false;
	xTraceInfo(context, "Stopped");
}

/* ================================================================================================================================ */
