/* ============================================================================================================================ *//**
 * @file       OS_Lock.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Monday, 27th June 2022 7:16:24 pm
 * @project    engineering-thesis
 * @brief      CIFX'es locks-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "cifxDriver.h"
#include "OS_Includes.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_lock";

/* ======================================================= Public functions ======================================================= */

void* OS_CreateLock(void) {

	xTraceDebug(context, "Creating lock...");

	// Allocate memory for the lock
	pthread_spinlock_t* lock = (pthread_spinlock_t*) malloc(sizeof(pthread_spinlock_t));
	if(lock == NULL) {
		xTraceGlobalSystemError(context, "Could not create lock");
		return NULL;
	}

	xTraceDebug(context, "Initializing lock...");

	// Initialize the lock
	const int pshared = PTHREAD_PROCESS_SHARED;
	int ec = pthread_spin_init(lock, pshared);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not initialize lock");
		free((void*) lock);
		return NULL;
	}

	xTraceDebug(context, "Lock created successfully");
	return (void*) lock;
}


void OS_EnterLock(void* pvLock) {

	assert(pvLock != NULL);

	pthread_spinlock_t* lock = (pthread_spinlock_t*) pvLock;
	const int ec = pthread_spin_lock(lock);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not enter the lock");
		assert(0);
		return;
	}
}


void OS_LeaveLock(void* pvLock) {

	assert(pvLock != NULL);

	pthread_spinlock_t* lock = (pthread_spinlock_t*) pvLock;
	const int ec = pthread_spin_unlock(lock);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not leave the lock");
		assert(0); // This should not happen
		return;
	}
}


void OS_DeleteLock(void* pvLock) {

	assert(pvLock != NULL);
	xTraceDebug(context, "Deleting lock...");

	pthread_spinlock_t* lock = (pthread_spinlock_t*) pvLock;
	const int ec = pthread_spin_destroy(lock);
	free((void*) lock);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not delete lock");
		assert(0);
		return;
	}

	xTraceDebug(context, "Lock deleted successfully");
}

/* ================================================================================================================================ */
