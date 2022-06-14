/* ============================================================================================================================ *//**
 * @file       OS_Pci.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es PCI-handling-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#define PCI_CONFIG_SIZE 256LU

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libuio.h>
#include "cifxDriver.h"
#include "OS_Includes.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_pci";

/* ======================================================= Static functions ======================================================= */

/**
 * @returns 
 *    pointer to the dynamically allocated buffer for the PCI configuration
 */
static void* create_pci_config_buffer() {

	xTraceDebug(context, "Creating PCI config buffer...");

	const size_t count = PCI_CONFIG_SIZE;
	void* config = malloc(count);
	if(config == NULL) {
		xTraceGlobalSystemError(context, "Could not create PCI config buffer");
		return NULL;
	}

	xTraceDebug(context, "PCI config buffer created");
	return config;
}


/**
 * @brief Gets a path to the PCI config file
 * 
 * @param osdep 
 *    OS-dependent data associated with the CIFX device
 * @param path [out] 
 *    buffer to write path to the CIFX device's PCI configuration to
 * @param path_length 
 *    length of the @p path buffer
 */
static void get_pci_config_path(CIFX_OSDEPENDENT* osdep, char* path, size_t path_length) {

	assert(osdep != NULL);
	xTraceDebug(context, "Getting PCI config file path...");
	
	// Get the path to the UIO device
	const char* uio_path = uio_get_path(osdep->uio_device);
	assert(uio_path != NULL);

	// Create path to the UIO's PCI config file
	const int ec = snprintf(path, path_length, "%s/device/config", uio_path);
	assert(ec != -1);

	xTraceDebug(context, "PCI config file path is: %s", path);
}


/**
 * @brief Opens a PCI config file of the CIFX device by the path
 * 
 * @param path
 *    path to the config file
 * @param open_flags
 *    file's flags
 * 
 * @retval "file's descriptor" 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int open_pci_config_file(const char* path, int open_flags) {

	assert(path != NULL);
	xTraceDebug(context, "Opening PCI config file %s in mode %d...", path, open_flags);

	const int fd = open(path, open_flags);
	if(fd == -1) {
		xTraceGlobalSystemError(context, "Could not open PCI config file");
		return -1;
	}

	xTraceDebug(context, "PCI config file opened");
	return fd;
}


/**
 * @brief Opens a PCI config file of the CIFX device basing on the OS-dependenta data
 *    associated witht he device's structure (@see DEVICEINSTANCE)
 * @param osdep
 *    OS-dependant data associated with the device
 * @param open_flags
 *    open flags for the configuration file
 * 
 * @retval "file's descriptor" 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int open_pci_config(CIFX_OSDEPENDENT* osdep, int open_flags) {
	assert(osdep != NULL);
	char path[1024];
	get_pci_config_path(osdep, path, sizeof(path));
	return open_pci_config_file(path, open_flags);
}


/**
 * @brief Writes configuration to the open PCI configfile
 * 
 * @param fd
 *    config file's descriptor
 * @param config
 *    configuration to be written
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int write_to_pci_config_file(int fd, const void* config) {

	xTraceDebug(context, "Writing to PCI config file...");

	const ssize_t written = write(fd, config, PCI_CONFIG_SIZE);
	if(written == -1) {
		xTraceGlobalSystemError(context, "Could not write to PCI config file");
		return -1;
	}

	xTraceDebug(context, "PCI config written");
	return 0;
}


/**
 * @brief Reads from the open PCI configfile 
 * 
 * @param fd
 *    config file's descriptor
 * @param config
 *    buffer to write configuration to
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int read_from_pci_config_file(int fd, void* config) {

	assert(fd != -1);
	assert(config != 0);
	xTraceDebug(context, "Reading from %lu bytes PCI config file...", PCI_CONFIG_SIZE);

	const ssize_t read_count = read(fd, config, PCI_CONFIG_SIZE);
	if(read_count == -1) {
		xTraceGlobalSystemError(context, "Could not read from PCI config file");
		return -1;
	}

	xTraceDebug(context, "Successfully read from PCI config file");
	return 0;
}


/**
 * @brief Writes PCI configuration to the CIFX driver associated with the given 
 *    OS-dependend data
 * 
 * @param osdep
 *   OS-dependent data associated with the CIFX device
 * @param config
 *   configuration to be written
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int write_pci_config(CIFX_OSDEPENDENT* osdep, const void* config) {

	assert(osdep != NULL);
	assert(config != NULL);

	const int fd = open_pci_config(osdep, O_WRONLY);
	if(fd == -1)
		return -1;

	const int ec = write_to_pci_config_file(fd, config);
	close(fd);

	return ec;
}


/**
 * @brief Reads PCI configuration from the CIFX driver associated with the given 
 *    OS-dependend data
 * 
 * @param osdep
 *   OS-dependent data associated with the CIFX device
 * @param config
 *   buffer for configuration to be read
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int read_pci_config_to_buffer(CIFX_OSDEPENDENT* osdep, void* config) {

	assert(osdep != NULL);
	assert(config != NULL);

	const int fd = open_pci_config(osdep, O_RDONLY);
	if(fd == -1)
		return -1;

	if(read_from_pci_config_file(fd, config) == -1)	{
		close(fd);
		return -1;
	}

	return 0;
}


/**
 * @brief Reads PCI configuration from the CIFX driver associated with the given 
 *    OS-dependend data and saves it to dynamically-allocated buffer. Returns pointer
 *    to the buffer on success
 * 
 * @param osdep
 *   OS-dependent data associated with the CIFX device
 * 
 * @retval "pointer" 
 *    to the buffer containing configuration on success
 * @retval "NULL"
 *    on error
 * 
 * @note Buffer should be deallocated by the calling app if the call succseeded
 */
static void* read_pci_config(CIFX_OSDEPENDENT* osdep) {

	assert(osdep != NULL);

	void* config = create_pci_config_buffer();
	if(config == NULL)
		return NULL;

	if(read_pci_config_to_buffer(osdep, config) == -1) {
		free(config);
		return NULL;
	}

	return config;
}

/* ======================================================= Public functions ======================================================= */

void OS_WritePCIConfig(void* pvOSDependent, void* pvPCIConfig) {

	assert(pvOSDependent != NULL);
	assert(pvPCIConfig != NULL);

	CIFX_OSDEPENDENT* osdep = (CIFX_OSDEPENDENT*) pvOSDependent;
	const int ec = write_pci_config(osdep, pvPCIConfig);
	(void) ec;

	/**
	 * @note The @v pvPCIConfig buffer must be freed, because it is provided
	 *    from OS_ReadPCIConfig()
	 */

	free((void*) pvPCIConfig); 
}


void* OS_ReadPCIConfig(void* pvOSDependent) {
	assert(pvOSDependent != NULL);
	CIFX_OSDEPENDENT* osdep = (CIFX_OSDEPENDENT*) pvOSDependent;
	return read_pci_config(osdep);
}

/* ================================================================================================================================ */
