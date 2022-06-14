/** ==================================================================================================================================
 * @file       user.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Friday, 20th May 2022 8:16:15 pm
 * @project    engineering-thesis
 * @brief      Definitions of the USER-dependant functions of the CIFX/netX Toolkit [implementation]
 * 
 * 
 * @note default paths to firmware and configuration files (i.e. if path not given in the driver's configuration structure)
 *    are related to EtherCAT Master configuration and are searched in /opt/cifx/devconfig/slot0/channel0 directory. Bootloader
 *    in such case is expected to reside under /opt/cifx/NETX100-BSL.bin
 * @copyright Krzysztof Pierczyk © 2022
 * ================================================================================================================================ */

#define DEFAULT_BOOTLOADER_PATH    "/opt/cifx/NETX100-BSL.bin"
#define DEFAULT_FIRMWARE_PATH      "/opt/cifx/devconfig/slot0/channel0/cifxecm.nxf"
#define DEFAULT_CONFIGURATION_PATH "/opt/cifx/devconfig/slot0/channel0/ethercat.xml"

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include "cifXToolkit.h"
#include "cifXErrors.h"

/* ========================================================= Declarations ========================================================= */

/// Path to the bootloader file
extern char bootloader_f[];
/// Path to the firmware file
extern char firmware_f[];
/// Path to the configuration file
extern char config_f[];

/* ======================================================= Public functions ======================================================= */

/**
 * @brief Returns the number of firmware files associated with the chennel of CIFX device
 * 
 * @param ptDevInfo
 *    device's reference including channel number, for which the firmware file count should be read
 * @returns 
 *    number for firmware files to download; the returned value will be used as maximum value
 *    for ulIdx in calls to USER_GetFirmwareFile 
 */
uint32_t USER_GetFirmwareFileCount(PCIFX_DEVICE_INFORMATION ptDevInfo) {

    assert(ptDevInfo != NULL);
    assert(ptDevInfo->ptDeviceInstance->ulSlotNumber == 0);

    /**
     * @note: The overlying assert indicates that only the first slot (i.e. a single
     *    device) is supported
     * @note: Only the first channel of the device is supported
     */
    
    if(ptDevInfo->ulChannel == 0)
        return 1;
    else
        return 0;
}


/**
 * @brief Returns path to the firmware file with the given @p ulIdx of the given channel of the CIFX
 *     device and Idx passed as argument
 * 
 * @param ptDevInfo
 *    device's reference including channel number, for which the firmware file should be delivered
 * @param ulIdx
 *    index of the returned firmware file (0..USER_GetFirmwareFileCount() - 1)
 * @param ptFileInfo
 *    structure containing short and full file name of the firmware
 * @returns 
 *    non-zero value on success
 */
int32_t USER_GetFirmwareFile(
    PCIFX_DEVICE_INFORMATION ptDevInfo, 
    uint32_t ulIdx, 
    PCIFX_FILE_INFORMATION ptFileInfo
) {
    assert(ptDevInfo != NULL);
    assert(ptFileInfo != NULL);
    // Only first slot supported
    assert(ptDevInfo->ptDeviceInstance->ulSlotNumber == 0);
    // Only one channel supported
    assert(ptDevInfo->ulChannel == 0);
    // Only one firmware supported
    assert(ulIdx == 0);

    // If default path used
    if(firmware_f[0] == '\0') {
        const char *short_name = basename(DEFAULT_FIRMWARE_PATH);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        const char full_name[] = DEFAULT_FIRMWARE_PATH;
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(full_name));
        strcpy(ptFileInfo->szFullFileName, full_name);
    } 
    // If custom path used
    else {
        const char *short_name = basename(firmware_f);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(firmware_f));
        strcpy(ptFileInfo->szFullFileName, firmware_f);
    }

    return 1;
}


/**
 * @brief Returns the number of configuration files associated with the communication channel
 *    of the CIFX device
 * 
 * @param ptDevInfo
 *    device's reference including channel number, for which the configuration file count should be read
 * @returns 
 *    number for confgiuration files, to download; the returned value will be used as maximum value for
 *    ulIdx in calls to USER_GetConfgirationFile
 */
uint32_t USER_GetConfigurationFileCount(PCIFX_DEVICE_INFORMATION ptDevInfo) {

    assert(ptDevInfo != NULL);
    // Only first slot supported
    assert(ptDevInfo->ptDeviceInstance->ulSlotNumber == 0);
    
    if(ptDevInfo->ulChannel == 0 && strcmp(config_f, "none") != 0)
        return 1;
    else
        return 0;
}


/**
 * @brief Returns configuration file information for the given device/channel and
 *    Idx passed as argument.
 * 
 * @param ptDevInfo
 *    device's reference including channel number, for which the configuration file should be delivered
 * @param ulIdx
 *    Index of the returned file (0..USER_GetConfigurationFileCount() - 1)
 * @param ptFileInfo
 *    structure containing short and full file name of the configuration file
 * @returns 
 *    non-zero value on success 
 */
int32_t USER_GetConfigurationFile(PCIFX_DEVICE_INFORMATION ptDevInfo, uint32_t ulIdx, PCIFX_FILE_INFORMATION ptFileInfo) {

    assert(ptDevInfo != NULL);
    assert(ptDevInfo->ptDeviceInstance != NULL);
    assert(ptFileInfo != NULL);
    // Only first slot supported
    assert(ptDevInfo->ptDeviceInstance->ulSlotNumber == 0);
    // Only one channel supported
    assert(ptDevInfo->ulChannel == 0);
    // Only one configuration file supported
    assert(ulIdx == 0);

    // If default path used
    if(config_f[0] == '\0') {
        const char *short_name = basename(DEFAULT_CONFIGURATION_PATH);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        const char *full_name = DEFAULT_CONFIGURATION_PATH;
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(full_name));
        strcpy(ptFileInfo->szFullFileName, full_name);
    } 
    // If custom path used
    else if(strcmp(config_f, "none") != 0) {
        const char *short_name = basename(config_f);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(config_f));
        strcpy(ptFileInfo->szFullFileName, config_f);
    }
    // No configuration file
    else
        return 0;

    return 1;
}


/**
 * @brief Returns OS file information for the given communication channel of the CIFX device
 * 
 * @param ptDevInfo
 *    device's reference for which the firmware file should be delivered
 * @param ptFileInfo
 *    structure containing short and full file name of the OS file
 * @returns 
 *    non-zero value on success 
 * 
 * @note Not supported, Firmware should be always loaded with the OS compiled in
 */
int32_t USER_GetOSFile(PCIFX_DEVICE_INFORMATION ptDevInfo, PCIFX_FILE_INFORMATION ptFileInfo) {
    assert(ptDevInfo != NULL);
    assert(ptFileInfo != NULL);
    return 0; 
}


/**
 * @brief Retrieve the full file name of the cifX bootloader binary image
 * 
 * @param ptDevInstance
 *    pointer to the device instance
 * @param ptFileInfo [out]
 *    structure with short and full file name of the bootloader
 */
void USER_GetBootloaderFile(PDEVICEINSTANCE ptDevInstance, PCIFX_FILE_INFORMATION ptFileInfo) {

    assert(ptDevInstance != NULL);
    assert(ptFileInfo != NULL);
    // Only one chip type supported
    assert(ptDevInstance->eChipType == eCHIP_TYPE_NETX500);

    // If default path used
    if(bootloader_f[0] == '\0') {
        const char *short_name = basename(DEFAULT_BOOTLOADER_PATH);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        const char *full_name = DEFAULT_BOOTLOADER_PATH;
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(full_name));
        strcpy(ptFileInfo->szFullFileName, full_name);
    } 
    // If custom path used
    else {
        const char *short_name = basename(bootloader_f);
        assert(sizeof(ptFileInfo->szShortFileName) > strlen(short_name));
        strcpy(ptFileInfo->szShortFileName, short_name);
        assert(sizeof(ptFileInfo->szFullFileName) > strlen(bootloader_f));
        strcpy(ptFileInfo->szFullFileName, bootloader_f);
    }
}


/**
 * @brief Retrieve the alias name of a cifX Board depending on the Device and Serialnumber passed
 *    during this call
 * 
 * @param ptDevInfo
 *    Device and Serial number of the card
 * @param ulMaxLen
 *    Maximum length of alias
 * @param szAlias
 *    Buffer to copy alias to. A string of length 0 means no alias
 */
void USER_GetAliasName(PCIFX_DEVICE_INFORMATION ptDevInfo, uint32_t ulMaxLen, char* szAlias) {

    assert(ptDevInfo != NULL);
    assert(ulMaxLen > 0);
    assert(szAlias != NULL);
    const char alias[] = "cifx0";
    assert(ulMaxLen > strlen(alias));
    strcpy(szAlias, alias);
}


/**
 * @brief Read the warmstart data from a given warmstart file
 * 
 * @param ptDevInfo
 *    Device- and Serial number of the card
 * @param ptPacket
 *    Buffer for the warmstart packet
 * @returns 
 *    non-zero value on success
 * 
 * @note: Feature not supported
 */
int32_t USER_GetWarmstartParameters(PCIFX_DEVICE_INFORMATION ptDevInfo, CIFX_PACKET* ptPacket){
    (void)(ptDevInfo);
    (void)(ptPacket);
    return 0; 
}

/**
 * @brief User-provided trace function used by the Toolkit to print log
 * 
 * @param ptDevInstance 
 *    handle to the device associated with the trace log
 * @param ulTraceLevel 
 *    requested trace levle
 * @param szFormat 
 *    string format (printf-like)
 * @param ... 
 *    format-dependent arguments
 */
void USER_Trace(PDEVICEINSTANCE ptDevInstance, uint32_t ulTraceLevel, const char* szFormat, ...) {
    
    assert(szFormat != NULL);
    (void) ptDevInstance;
    
    // Initialize variable arguments
    va_list vaList;
    va_start(vaList, szFormat);

    // Print tarce log
    xTrace(ulTraceLevel, szFormat, vaList);

    // Remove variable list
    va_end(vaList);
}

/**
 * @brief Predicates whether interrupts mode for the device should be used
 * 
 * @param ptDevInfo
 *    device infotmations
 * @returns 
 *    non-zero value if interrupts should be enables
 * 
 * @note driver always utilizes interrupts
 */
int32_t USER_GetInterruptEnable(PCIFX_DEVICE_INFORMATION ptDevInfo) {
    assert(ptDevInfo != NULL);
    return 1; 
}


/**
 * @brief Predicates whether DMA mode for the device should be used
 * 
 * @param ptDevInfo
 *    device infotmations
 * @returns 
 *    non-zero value if DMA mode used
 * 
* @note driver always utilizes DMA mode
 */
int USER_GetDMAMode(PCIFX_DEVICE_INFORMATION ptDevInfo) {
    assert(ptDevInfo != NULL);
    return 1;
}

/* ================================================================================================================================ */
