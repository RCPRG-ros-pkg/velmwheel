/* ============================================================================================================================ *//**
 * @file       OS_Irq.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es interrupt-handling functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#define _GNU_SOURCE // Needed for non-posix extensions

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <sched.h>
#include <sys/time.h>
#include <pthread.h>
#include <libuio.h>
#include "cifXToolkit.h"
#include "cifXEndianess.h"
#include "cifxDriver.h"
#include "OS_Common.h"
#include "OS_Includes.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_irq";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Configures pthread's attributes. These attributes are associated with the thread 
 *    processing IRQs from the CIFX UIO device.
 * 
 * @param[out] attr 
 *    attributes to be configured
 * @param sched_policy
 *    scheduling policy of the IRQ thread
 * @param sched_priority
 *    scheduling priority of the IRQ thread
 * @param affinity
 *    affinity (bitmask) of the IRQ thread
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int init_irq_thread_attr(
	pthread_attr_t* attr,
	int sched_policy,
	int sched_prior,
	int sched_inherit,
	unsigned affinity
) {
	assert(attr != NULL);
	xTraceInfo(context, "Initializing IRQ thread attributes...");

	// Initialize attributes object
	const int ec = pthread_attr_init(attr);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not init IRQ thread attributes");
		return -1;
	}

	static const char context[] = "irq";

	// Set thread's attributes
	if(set_thread_sched_policy(attr, sched_policy, context)       < 0 || 
	   set_thread_sched_priority(attr, sched_prior, context)      < 0 || 
	   set_thread_sched_inheritance(attr, sched_inherit, context) < 0 ||
	   set_thread_affinity(attr, affinity, context)               < 0
	) {
		destroy_thread_attr(attr);
		return -1;
	}

	return 0;
}


/**
 * @brief Wrapper around uio_irqwait_timeout(). Wait for the closest UIO interrupt with 
 *    the 10ms timeout
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 * @param timeout_error [out]
 *    set to true if the timeout occured
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int wait_for_uio_interrupt(CIFX_OSDEPENDENT* osdep, bool* timeout_error) {

	assert(osdep != NULL);
	assert(timeout_error != NULL);
	xTraceDebug(context, "Waiting for and UIO interrupt...");

	// Set the hardcoded timeout (!)
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 10000;

	// Wait for IRQ
	*timeout_error = false;
	if(uio_irqwait_timeout(osdep->uio_device, &timeout) == -1) {
		
		// Unknown error
		if(errno != ETIMEDOUT) {
			xTraceGlobalSystemError(context, "Could not wait for UIO interrupt");
			return -1;
		}

		// Timeout
		xTraceDebug(context, "Timeout during waiting for UIO interrupt");
		*timeout_error = true;
		return -1;
	}

	xTraceDebug(context, "UIO interrupt occured");
	return 0;
}


/**
 * @brief CIFX IRQs handler. Wrapper around cifXTKitISRHandler() and cifXTKitDSRHandler()
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 */
static void handle_uio_interrupt(CIFX_OSDEPENDENT* osdep) {
	
	assert(osdep != NULL);
	xTraceDebug(context, "Handling UIO interrupt...");

	// Filter all shared IRQs
	const int ignore_global_irq_flag = 1;
	
	// Call the CIFX ISR handler
	const int result = cifXTKitISRHandler(osdep->dev_instance, ignore_global_irq_flag);
	assert(result != CIFX_TKIT_IRQ_OTHERDEVICE);
	// On DRS call an appropriate handler
	if(result == CIFX_TKIT_IRQ_DSR_REQUESTED)
		cifXTKitDSRHandler(osdep->dev_instance);

	xTraceDebug(context, "UIO interrupt handled");
}


/**
 * @brief Reenables CIFX device's interrupts via PCI register
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 */
static void reenable_uio_interrupts(CIFX_OSDEPENDENT* osdep) {
	
	assert(osdep != NULL);

	// Get current IRQEnable flag's value
	const uint32_t curr_val = osdep->dev_instance->ptGlobalRegisters->ulIRQEnable_0;
	// Set the flag
	const uint32_t new_val = HOST_TO_LE32(curr_val | MSK_IRQ_EN0_INT_REQ);
	// Write flag's value back to the structure
	HWIF_WRITE32(
		osdep->dev_instance,
		osdep->dev_instance->ptGlobalRegisters->ulIRQEnable_0,
		new_val
	);
}


/**
 * @brief CIFX device's IRQs handling routine (run in the independant thread)
 * 
 * @param arg
 *    thread's arguments
 * @returns 
 *    @c NULL
 */
static void* irq_handler(void* arg) {

	assert(arg != NULL);
	xTraceInfo(context, "IRQ handler started");

	// Loop until termination
	CIFX_OSDEPENDENT* osdep = (CIFX_OSDEPENDENT*) arg;
	while(osdep->irq_terminate == 0) {
		bool timeout_error;
		if(wait_for_uio_interrupt(osdep, &timeout_error) == -1) {
			if(timeout_error)
				continue;
			else
				return NULL;
		}

		// Handle the interrupt
		handle_uio_interrupt(osdep);

		// Reenable interrupts on the PCI
		reenable_uio_interrupts(osdep);
	}

	xTraceInfo(context, "IRQ handler finished");

	return NULL;
}


/**
 * @brief Creates a CIFX device's IRQs-handling task with the given attributes
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 * @param attr
 *    thread's attributes
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int create_irq_thread(CIFX_OSDEPENDENT* osdep, pthread_attr_t* attr) {

	assert(osdep != NULL);
	xTraceInfo(context, "Creating IRQ thread...");

	osdep->irq_terminate = 0;

	// Create and IRQ thread
	const int ec = pthread_create(&osdep->irq_thread, attr, &irq_handler, osdep);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not create IRQ thread");
		return -1;
	}

	xTraceInfo(context, "IRQ thread created");
	return 0;
}


/**
 * @brief Creates and runs a CIFX device's IRQs-handling thread
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int start_irq_thread(CIFX_OSDEPENDENT* osdep) {

	assert(osdep != NULL);

	// Configrue thread's attributes
	pthread_attr_t irq_thread_attr;
	if(init_irq_thread_attr(
		&irq_thread_attr,
		osdep->irq_sched_policy,
		osdep->irq_sched_priority,
		osdep->irq_sched_inheritance,
		osdep->irq_affinity
	) == -1)
		return -1;

	// Create and run the thread
	const int result = create_irq_thread(osdep, &irq_thread_attr);

	// Free attributes obejct's memory
	pthread_attr_destroy(&irq_thread_attr);

	return result;
}


/**
 * @brief: Terminates the CIFX device's IRQs-handling thread
 * 
 * @param osdep
 *    OS-dependent data associated with the CIFX driver
 * 
 * @retval 0 
 *    on success
 * @retval "< 0" 
 *    on error
 */
static int stop_irq_thread(CIFX_OSDEPENDENT* osdep) {

	assert(osdep != NULL);
	xTraceInfo(context, "Joining IRQ thread...");

	// Mark the thread to terminate (shared variable)
	osdep->irq_terminate = 1;

	// Join the terminated thread
	void** retval = NULL;
	const int ec = pthread_join(osdep->irq_thread, retval);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not stop IRQ thread");
		return -1;
	}

	xTraceInfo(context, "IRQ thread joined");
	return 0;
}

/* ======================================================= Public functions ======================================================= */

void OS_EnableInterrupts(void* pvOSDependent) {
	assert(pvOSDependent != NULL);
	CIFX_OSDEPENDENT *osdep = (CIFX_OSDEPENDENT*) pvOSDependent;
	if(start_irq_thread(osdep) == -1)
		xTraceError(context, "Could not enable interrupts");
}


void OS_DisableInterrupts(void* pvOSDependent) {
	assert(pvOSDependent != NULL);
	CIFX_OSDEPENDENT *osdep = (CIFX_OSDEPENDENT*) pvOSDependent;
	if(stop_irq_thread(osdep) == -1)
		xTraceError(context, "Could not disable interrupts");
}

/* ================================================================================================================================ */
