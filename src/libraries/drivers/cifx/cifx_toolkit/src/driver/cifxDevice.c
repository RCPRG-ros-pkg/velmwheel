/* ============================================================================================================================ *//**
 * @file       cifxDevice.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 13th April 2021 7:51:3 am
 * @modified   Wednesday, 15th June 2022 9:54:53 pm
 * @project    engineering-thesis
 * @brief      Set of functions used to configure CIFX card [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

// Size of the buffers holding device-related files' paths
#define PATH_BUFF_SIZE 256
// Size of DMA buffer
#define DMA_BUFFER_SIZE (8*1024)

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libuio.h>
#include "cifxDriver.h"

/* =========================================================== Constants ========================================================== */

/// Name of the UIO device that is supported by the toolkit implementation
static char *supported_uio_device_name = "netx";
/// Name of the DPM memory map of the UIO device that is supported by the toolkit implementation
static char *supported_uio_device_dpm_memory_map_name = "dpm";
/// Name of the DMA memory map of the UIO device that is supported by the toolkit implementation
static char *supported_uio_device_dma_memory_map_name = "dma";

/* ========================================================== Global Data ========================================================= */

/// Path to the bootloader file
char bootloader_f[PATH_BUFF_SIZE] = {0};
/// Path to the firmware file
char firmware_f[PATH_BUFF_SIZE] = {0};
/// Path to the configuration file
char config_f[PATH_BUFF_SIZE] = {0};

/* ========================================================== Static Data ========================================================= */

/// True, when drivers are initialized
static bool g_initialized = false;
/// Pointer to the libuio device's structure
static struct uio_info_t* g_uio_device = NULL;
/// Pointer to the CIFX Tooolkit device's structure
static DEVICEINSTANCE* g_dev_instance = NULL;
/// File's context
static const char *context = "cifx_device";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Validates name of the device descibed by the @p uio_device. CIFX card's name
 *    should be 'netx'
 * 
 * @param uio_device
 *    libuio structure describing the CIFX card
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int check_uio_device_name(struct uio_info_t* uio_device){

	assert(uio_device != NULL);
	xTraceInfo(context, "Checking, if UIO device has correct name...");

	// Get name of the UIO device
	const char* uio_name = uio_get_name(uio_device);
	assert(uio_name != NULL);

	// Check if the name is valid
	if(strcmp(uio_name, supported_uio_device_name) != 0) {
		xTraceInfo(context, "Specified UIO device is invalid: name not match");
		return -1;
	}

	return 0;
}


/**
 * @brief Checks whether the CIFX cards has a DPM (Dual Port Memory) mapping named 'dpm'
 *    in the UIO device's directory
 * 
 * @param uio_device
 *    libuio structure describing the CIFX card
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int check_uio_device_dpm_map(struct uio_info_t* uio_device) {

	assert(uio_device != NULL);
	xTraceInfo(context, "Checking, if UIO device has DPM map...");

	// Check whether the mapping exists
	const int dpm_index = uio_get_map_index_by_name(uio_device, supported_uio_device_dpm_memory_map_name);
	if(dpm_index == -1)	{
		xTraceInfo(context, "Specified UIO device is invalid: DPM map not found");
		return -1;
	}

	return 0;
}


/**
 * @brief Checks whether the CIFX cards is properly configured with respect to the UIO driver
 * 
 * @param uio_device
 *    libuio structure describing the CIFX card
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int check_uio_device(struct uio_info_t* uio_device) {
	
	assert(uio_device != NULL);
	xTraceInfo(context, "Checking UIO device...");

	// Check the name correctness and DPM mapping
	if(check_uio_device_name(uio_device) == -1 || check_uio_device_dpm_map(uio_device) == -1)
		return -1;

	return 0;
}


/**
 * @brief Acquires UIO device basing on it's index in the /sys/class/uio/ directory. Preliminarily
 *    checks whether the device as a CIFX card.
 * 
 * @param uio_num
 *    index of the device in the  /sys/class/uio/ directory
 * 
 * @retval pointer 
 *     device's libuio structure's pointer on success
 * @retval NULL
 *    when the device could not be acquired or validation failed
 */
static struct uio_info_t* acquire_uio_device(int uio_num) {

	xTraceInfo(context, "Acquiring UIO device...");

	// Fins the UIO device
	struct uio_info_t* uio_device = uio_find_by_uio_num(uio_num);
	if(uio_device == NULL) {
		xTraceGlobalSystemError(context, "Could not acquire UIO device (%d)", uio_num);
		return NULL;
	}

	// Validate the UIO device
	if(check_uio_device(uio_device) == -1) {
		uio_free_info(uio_device);
		return NULL;
	}

	// Open the UIO device
	if(uio_open(uio_device) == -1) {
		xTraceGlobalSystemError(context, "Could not open UIO device (%d)", uio_num);
		uio_free_info(uio_device);
		return NULL;
	}

	xTraceInfo(context, "Successfully acquired UIO device (%d)", uio_num);
	return uio_device;
}


/**
 * @brief Releases the UIO device (i.e. CIFX card) and frees its resources
 * 
 * @param uio_device
 *    libuio structure describing the CIFX card
 */
static void release_uio_device(struct uio_info_t* uio_device) {

	assert(uio_device != NULL);
	xTraceInfo(context, "Releasing UIO device...");

	const int ec = uio_close(uio_device);
	assert(ec == 0);
	uio_free_info(uio_device);

	xTraceInfo(context, "Released UIO device");
}


/**
 * @brief Deinitializes the CIFX card in the Toolkit clearing it's OS-dependant data
 * 
 * @param dev_instance
 *    CIFX device to be deinitialized
 */
static void deinit_uio_dev_instance(DEVICEINSTANCE* dev_instance) {
	assert(dev_instance != NULL);
	assert(dev_instance->pvOSDependent);
	free(dev_instance->pvOSDependent);
	dev_instance->pvOSDependent = NULL;
}


/**
 * @brief Creates OS-dependant data structure describing a CIFX device
 * 
 * @retval pointer
 *     to the structure on success
 * @retval NULL
 *    on error
 */
static CIFX_OSDEPENDENT* create_osdependent() {
	
	xTraceInfo(context, "Creating OS dependent structures...");

	// Allocate memory for the object
	CIFX_OSDEPENDENT* osdependent = (CIFX_OSDEPENDENT*) malloc(sizeof(CIFX_OSDEPENDENT));
	if(osdependent == NULL)	{
		xTraceGlobalSystemError(context, "Could not create OS dependent structure");
		return NULL;
	}

	return osdependent;
}


/**
 * @brief Sets CIFX device's name in the Toolkit structure basing on device's libuio
 *    structure
 * 
 * @param dev_instance
 *    Toolkit CIFX'es device structure
 * @param name
 *    device's name inside the Toolkit
 */
static void set_dev_instance_name(DEVICEINSTANCE* dev_instance, const char* name) {

	assert(dev_instance != NULL);
	assert(name != NULL);

	// Compose device's name basing on the version
	strncpy(dev_instance->szName, name, sizeof(dev_instance->szName));
}


/**
 * @brief Passes pointer to the DPM mapping of the CIFX device to the Toolkit device's structure
 * 
 * @param dev_instance
 *    Toolkit device's structure
 * @param uio_device
 *    libuio device's structure
 */
static void set_uio_dev_instance_dpm_data(DEVICEINSTANCE* dev_instance, struct uio_info_t* uio_device) {

	assert(dev_instance != NULL);
	assert(uio_device != NULL);

	// Get index of the DPM mapping (ensure that it exists)
	const int dpm_index = uio_get_map_index_by_name(uio_device, supported_uio_device_dpm_memory_map_name);
	assert(dpm_index != -1);

	// Get pointer to the DPM mapping
	void* dpm_ptr = uio_get_mem_map(uio_device, dpm_index);
	assert(dpm_ptr != NULL);

	// Get size of the DPM mapping
	const size_t dpm_size = uio_get_mem_size(uio_device, dpm_index);
	assert(dpm_size != 0);

	// Map DPM memory mapping into CIFX device structure in the Toolkit
	dev_instance->pbDPM = dpm_ptr;
	dev_instance->ulDPMSize = dpm_size;
}


/**
 * @brief Configures CIFX device's DMA mappings for I/O areas of communication channels
 * 
 * @param dev_instance
 *    Toolkit device's structure
 * @param uio_device
 *    libuio device's structure
 */
static void set_uio_dev_instance_dma_data(DEVICEINSTANCE* dev_instance, struct uio_info_t* uio_device) {
	
	assert(dev_instance != NULL);
	assert(uio_device != NULL);

	// Get index of the devices' DMA memory mapping
	const int dma_index = uio_get_map_index_by_name(uio_device, supported_uio_device_dma_memory_map_name);
	if(dma_index == -1)	{
		return;
	}

	// Get DMA buffer's virtual address (cast to char * type for pointers arithmetics)
	char* dma_ptr = (char*) uio_get_mem_map(uio_device, dma_index);
	assert(dma_ptr != NULL);

	// Get DMA buffer physical address (CIFX card needs it to perform proper DMA transfers from I/O bus to memory)
	unsigned long dma_addr = uio_get_mem_addr(uio_device, dma_index);
	assert(dma_addr != 0);

	// Set number of DMA buffers (always 8 - number of card's communication channels - @see docs)
	dev_instance->ulDMABufferCount = 8;

	/**
	 * @note If DMA is used, all buffers must be allocated even if only a signle
	 *    communication channel is used
	 */

	// Initialize all DMA buffers
	for(size_t j = 0; j < dev_instance->ulDMABufferCount; ++j) {

		dev_instance->atDmaBuffers[j].ulSize = DMA_BUFFER_SIZE;
		dev_instance->atDmaBuffers[j].ulPhysicalAddress = dma_addr;
		dev_instance->atDmaBuffers[j].pvBuffer = (void*) dma_ptr;

		dma_ptr += DMA_BUFFER_SIZE;
		dma_addr += DMA_BUFFER_SIZE;
	}
}


/**
 * @brief Sets device's OS-dependant data in the CIFX Toolkit sructure basing on the
 *    device's libuio structure and driver device's initialization structure
 * 
 * @param dev_instance
 *    CIFX Toolkit device's structure
 * @param uio_device
 *    libuio device's structure
 * @param device_info
 *    driver-specific device's initialization structure
 */
static void set_dev_instance_osdependent_data(
	DEVICEINSTANCE* dev_instance, 
	struct uio_info_t* uio_device, 
	const CIFX_DEVICE_INIT* device_info
) {
	assert(dev_instance != NULL);
	assert(uio_device != NULL);
	assert(dev_instance->pvOSDependent != NULL);
	assert(device_info != NULL);

	// Get pointer to the OSDependent struct
	CIFX_OSDEPENDENT* osdependent = (CIFX_OSDEPENDENT*) dev_instance->pvOSDependent;

	// Configure the structure
	osdependent->dev_instance          = dev_instance;
	osdependent->uio_device            = uio_device;
	osdependent->irq_sched_policy      = device_info->irq_thread_params.sched_policy;
	osdependent->irq_sched_priority    = device_info->irq_thread_params.sched_priority;
	osdependent->irq_sched_inheritance = device_info->irq_thread_params.sched_inheritance;
	osdependent->irq_affinity          = device_info->irq_thread_params.affinity;
}


/**
 * @brief Initializes device's CIFX Toolkit sructure and device's libuio structure basing on
 *    driver-specific device's initialization structure
 * 
 * @param dev_instance
 *    CIFX Toolkit device's structure
 * @param uio_device
 *    libuio device's structure
 * @param device_info
 *    driver-specific device's initialization structure
 *  
 * @retval 0 
 *    on success
 * @retval -1
 *    on error
 */
static int init_uio_dev_instance(
	DEVICEINSTANCE* dev_instance, 
	struct uio_info_t* uio_device, 
	const CIFX_DEVICE_INIT* device_info
) {
	assert(dev_instance != NULL);
	assert(uio_device != NULL);
	assert(device_info != NULL);
	xTraceInfo(context, "Initializing UIO dev instance...");

	// Initialize CIFX Toolkit device's structure
	memset(dev_instance, 0, sizeof(DEVICEINSTANCE));

	// Create fresh OS-dependant config for initialized structure
	CIFX_OSDEPENDENT* osdependent = create_osdependent();
	if(osdependent == NULL)	{
		return -1;
	}

	// Map OS-dependent data to the CIFX Toolkit device's object
	dev_instance->pvOSDependent = osdependent;

	// Mark device as PCI card
	dev_instance->fPCICard = 1;

	// Configure device's toolkit parameters
	set_dev_instance_name(dev_instance, device_info->name);
	set_dev_instance_osdependent_data(dev_instance, uio_device, device_info);

	// Configure UIO
	set_uio_dev_instance_dpm_data(dev_instance, uio_device);
	set_uio_dev_instance_dma_data(dev_instance, uio_device);

	return 0;
}


/**
 * @brief Creates a new CIFX Toolkit device's structure and initializes libuio device's
 *    structure @p uio_device (must be already allocated) basing on the driver-specific
 *    device's initialization structure
 * 
 * @param uio_device
 *    libuio device's structure
 * @param device_info
 *    driver-specific device's initialization structure
 * 
 * @retval pointer 
 *    to the CIFX Toolkit device's structure on success
 * @retval NULL 
 *    on error
 */
static DEVICEINSTANCE* create_uio_dev_instance(
	struct uio_info_t* uio_device, 
	const CIFX_DEVICE_INIT* device_info
) {
	assert(uio_device != NULL);
	assert(device_info != NULL);
	
	xTraceInfo(context, "Creating UIO dev instance...");

	// Allocate memory for the CIFX device object
	DEVICEINSTANCE* dev_instance = (DEVICEINSTANCE*) malloc(sizeof(DEVICEINSTANCE));
	if(dev_instance == NULL) {
		xTraceGlobalSystemError(context, "Could not create UIO dev instance");
		return NULL;
	}

	// Initialize the CIFX device obejct
	if(init_uio_dev_instance(dev_instance, uio_device, device_info) == -1) {
		free(dev_instance);
		return NULL;
	}

	xTraceInfo(context, "Created UIO dev instance");
	return dev_instance;
}


/**
 * @brief Registers CIFX device in the toolkit
 * 
 * @param dev_instance
 *    CIFX Toolkit device's structure
 * 
 * @retval 0 
 *    on success
 * @retval "< 0"
 *    on error
 */
static int add_uio_dev_instance_to_toolkit(DEVICEINSTANCE* dev_instance) {

	assert(dev_instance != NULL);
	xTraceInfo(context, "Adding UIO dev instance to toolkit...");

	const int ec = cifXTKitAddDevice(dev_instance);
	if(ec != CIFX_NO_ERROR)	{
		xTraceSystemError(context, ec, "Could not add device to toolkit");
		return -1;
	}

	xTraceInfo(context, "UIO dev instance added to toolkit");
	return 0;
}


/**
 * @brief Destroys CIFX Toolkit and libuio device's structure
 * 
 * @param dev_instance
 *    CIFX Toolkit device's structure
 */
static void destroy_uio_dev_instance(DEVICEINSTANCE* dev_instance) {
	assert(dev_instance != NULL);
	deinit_uio_dev_instance(dev_instance);
	free(dev_instance);
}


/**
 * @brief Deregisters CIFX device from the Toolkit (should be called before destroy_uio_dev_instance())
 * 
 * @param dev_instance
 *    CIFX Toolkit device's structure
 */
static void remove_uio_dev_instance_from_toolkit(DEVICEINSTANCE* dev_instance) {

	assert(dev_instance != NULL);
	xTraceInfo(context, "Removing UIO dev instance from toolkit...");

	// Remove device from the toolkit
	char* board_name = dev_instance->szName;
	const int ec = cifXTKitRemoveDevice(board_name, true);
	if(ec != CIFX_NO_ERROR)	{
		xTraceSystemError(context, ec, "Could not remove device from toolkit: %s");
		return;
	}

	xTraceInfo(context, "UIO dev instance removed from toolkit");
}

/* =================================================== Public functions (Device) ================================================== */

int xDeviceInit(const CIFX_DEVICE_INIT* device_info) {

    // Check pointer's corectness
	if(device_info == NULL)
		return CIFX_INVALID_POINTER;
	
	// Check if device was already initialized
	if(g_initialized)
		return CIFX_DEV_ALREADY_INITIALIZED;

	// Check if toolkit is initialized
	if(!xToolkitIsInit())
		return CIFX_DEV_TKT_NOT_INITIALIZED;

	xTraceInfo(context, "Initializing...");
    
	// Check if custom paths (if given) fit into internal buffers
	if(device_info->bootloader_file != NULL && strlen(device_info->bootloader_file) > PATH_BUFF_SIZE)
		return CIFX_DEV_INIT_ERROR;
	if(device_info->firmware_file != NULL && strlen(device_info->firmware_file) > PATH_BUFF_SIZE)
		return CIFX_DEV_INIT_ERROR;
	if(device_info->config_file != NULL && strlen(device_info->config_file) > PATH_BUFF_SIZE)
		return CIFX_DEV_INIT_ERROR;

	// Save custom paths to bootloader/firmware/configuration files (if given)
	if(device_info->bootloader_file != NULL)
		strncpy(bootloader_f, device_info->bootloader_file, PATH_BUFF_SIZE);
	if(device_info->firmware_file != NULL)
		strncpy(firmware_f, device_info->firmware_file, PATH_BUFF_SIZE);
	if(device_info->config_file != NULL)
		strncpy(config_f, device_info->config_file, PATH_BUFF_SIZE);
    
    /**
     * @warning Reset 'errno' to @c 0 manually to make sure that the libuio will not fail
     *   for a false reason (yeah, it happens... .-.)
     */
    errno = 0;

	// Acquire the UIO device
	struct uio_info_t* uio_device = acquire_uio_device(device_info->uio_num);
	if(uio_device == NULL)
		return CIFX_DEV_INIT_ERROR;

	// Create CIFX device object from UIO description
	DEVICEINSTANCE* dev_instance = create_uio_dev_instance(uio_device, device_info);
	if(dev_instance == NULL) {
		release_uio_device(uio_device);
		return CIFX_DEV_INIT_ERROR;
	}

	// Register the device in the toolkit
	if(add_uio_dev_instance_to_toolkit(dev_instance) == -1) {
		destroy_uio_dev_instance(dev_instance);
		release_uio_device(uio_device);
		return CIFX_DEV_INIT_ERROR;
	}

	// Save instance of the initialized device as global symbol
	g_dev_instance = dev_instance;
	// Save UIO description of the initialized device as global symbol
	g_uio_device = uio_device;
	// Mark device as initialized
	g_initialized = true;

	xTraceInfo(context, "Initialized");
	return 0;
}


PDEVICEINSTANCE xDeviceGet() {
    return g_dev_instance;
}


bool xDeviceIsInit() {
	return g_initialized;
}


void xDeviceDeinit() {

	// Check whether device was initialized
	if(!g_initialized) 
		return;

	xTraceInfo(context, "Deinitializing...");

	// Unregister device from Toolkit 
	remove_uio_dev_instance_from_toolkit(g_dev_instance);

	// Clean custom paths to bootloader/firmware/config files
	bootloader_f[0] = '\0';
	firmware_f[0] = '\0';
	config_f[0] = '\0';

	// Destroy CIFX Toolkit device's structure
	destroy_uio_dev_instance(g_dev_instance);
	// Destroy UIO device description
	release_uio_device(g_uio_device);

	// Update global state of the library
	g_initialized = false;
	g_uio_device = NULL;
	g_dev_instance = NULL;

	xTraceInfo(context, "Deinitialized");
}

/* ================================================================================================================================ */
