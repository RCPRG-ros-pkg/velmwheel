/* ============================================================================================================================ *//**
 * @file       OS_Common.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:45:55 pm
 * @project    engineering-thesis
 * @brief      Common functions used in the implementation of OS-dependent part of the CIFX/netX Toolkit [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#define _GNU_SOURCE

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdint.h>
#include <stdarg.h>
#include <time.h>
#include <stdio.h>
#include <sched.h>
#include "OS_Includes.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_common";

/* ======================================================= Public functions ======================================================= */

struct timespec get_clock_time() {

	struct timespec ts;
	if(clock_gettime(CLOCK_REALTIME, &ts) == -1) {
		xTraceGlobalSystemError(context, "Could not get current time");
		assert(0);
	}

	return ts;
}


struct timespec timespec_add_ms(const struct timespec* ts, uint32_t msec) {
	struct timespec result = *ts;
	result.tv_sec += (msec / 1000);
	result.tv_nsec += (msec % 1000) * 1000;
	if(result.tv_nsec >= 1000000000) {
		result.tv_sec += 1;
		result.tv_nsec = result.tv_nsec % 1000000000;
	}
	return result;
}


int set_thread_sched_policy(pthread_attr_t* attr, int sched_policy, const char* context) {

	assert(attr != NULL);
	xTraceInfo(context, "Setting sched policy in IRQ thread attributes to %d...", sched_policy);

	// Set the attribute
	const int ec = pthread_attr_setschedpolicy(attr, sched_policy);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not set sched policy in IRQ thread attributes");
		return -1;
	}

	return 0;
}


int set_thread_sched_priority(pthread_attr_t* attr, int sched_prior, const char* context) {

	assert(attr != NULL);

	// Print debugging message
	xTraceInfo(context, "Setting sched priority in IRQ thread attributes to %d...", sched_prior);

	// Set the attribute
	struct sched_param schedparam;
	schedparam.sched_priority = sched_prior;
	const int ec = pthread_attr_setschedparam(attr, &schedparam);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not set sched priority in IRQ thread attributes");
		return -1;
	}

	return 0;
}


int set_thread_sched_inheritance(pthread_attr_t* attr, int inheritsched, const char* context) {
	
    assert(attr != NULL);
    xTraceInfo(context, "Setting explicitness of sched params in IRQ thread atributes...");

    const int ec = pthread_attr_setinheritsched(attr, inheritsched);
    if(ec != 0) {
		xTraceSystemError(context, ec, "Could not set sched params to be explicit in IRQ thread attributes");
        return -1;
    }

    return 0;
}


int set_thread_affinity(pthread_attr_t* attr, unsigned affinity, const char* context) {

	assert(attr != NULL);

	// Print debugging message
	xTraceInfo(context, "Settings affinity in IRQ thread attributes to 0x%x...", affinity);

	// Initialize empty cpuset
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);

	// Fill cpuset with cpus given from affinity
	const int max_bit = (sizeof(affinity) * 8);
	for(int i = 0; i < max_bit; ++i) {
		const unsigned cpu_mask = (1 << i);
		if(affinity & cpu_mask) {
			CPU_SET(i, &cpuset);
		}
	}

	// Provide calculated cpuset to thread attributes
	const int ec = pthread_attr_setaffinity_np(attr, sizeof(cpuset), &cpuset);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not set affinity in IRQ thread attributes");
		return -1;
	}

	return 0;
}


void destroy_thread_attr(pthread_attr_t* attr) {
	assert(attr != NULL);
	const int ec = pthread_attr_destroy(attr);
	assert(ec == 0);
}

/* ================================================================================================================================ */
