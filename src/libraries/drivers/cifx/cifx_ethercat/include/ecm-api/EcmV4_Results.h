/**************************************************************************************
Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmV4_Results.h 82353 2019-04-09 10:28:34Z Sven $:

Description:
  EcmV4_Results.h
**************************************************************************************/

#ifndef __ECMV4_RESULTS_H
#define __ECMV4_RESULTS_H

#include<stdint.h>

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 AP Task Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_AP_FIRMWARE_HAS_CRASHED
//
// MessageText:
//
// Firmware has crashed.
//
#define ERR_ECMV4_AP_FIRMWARE_HAS_CRASHED ((uint32_t)0xC0D70001L)

//
// MessageId: ERR_ECMV4_AP_CONFIGURATION_INTERFACE_NOT_AVAILABLE
//
// MessageText:
//
// Configuration interface not currently available.
//
#define ERR_ECMV4_AP_CONFIGURATION_INTERFACE_NOT_AVAILABLE ((uint32_t)0xC0D70002L)

//
// MessageId: ERR_ECMV4_AP_SET_TARGET_STATE_NOT_ALLOWED_DURING_CFG_LOADING
//
// MessageText:
//
// Set Target State is not allowed during configuration loading.
//
#define ERR_ECMV4_AP_SET_TARGET_STATE_NOT_ALLOWED_DURING_CFG_LOADING ((uint32_t)0xC0D70003L)

//
// MessageId: ERR_ECMV4_AP_INVALID_STARTUP_PARAMETER
//
// MessageText:
//
// Invalid startup parameter.
//
#define ERR_ECMV4_AP_INVALID_STARTUP_PARAMETER ((uint32_t)0xC0D70004L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 LLD Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_LLD_TIMEOUT
//
// MessageText:
//
// ESC Register access timeout.
//
#define ERR_ECMV4_LLD_TIMEOUT            ((uint32_t)0xC0CC0001L)

//
// MessageId: ERR_ECMV4_LLD_UNSUPPORTED_COMMAND
//
// MessageText:
//
// LLD: Unsupported EtherCAT telegram command.
//
#define ERR_ECMV4_LLD_UNSUPPORTED_COMMAND ((uint32_t)0xC0CC0003L)

//
// MessageId: ERR_ECMV4_LLD_DUPLICATE_FIXED_STATION_ADDRESS
//
// MessageText:
//
// LLD: Duplicate fixed station address.
//
#define ERR_ECMV4_LLD_DUPLICATE_FIXED_STATION_ADDRESS ((uint32_t)0xC0CC0004L)

//
// MessageId: ERR_ECMV4_LLD_SII_CHECKSUM_ERROR
//
// MessageText:
//
// LLD: SII Checksum Error.
//
#define ERR_ECMV4_LLD_SII_CHECKSUM_ERROR ((uint32_t)0xC0CC0005L)

//
// MessageId: ERR_ECMV4_LLD_SII_EEPROM_LOADING_ERROR
//
// MessageText:
//
// LLD: SII EEPROM Loading Error.
//
#define ERR_ECMV4_LLD_SII_EEPROM_LOADING_ERROR ((uint32_t)0xC0CC0006L)

//
// MessageId: ERR_ECMV4_LLD_SII_MISSING_ERROR_ACK
//
// MessageText:
//
// LLD: SII Missing Error Ack.
//
#define ERR_ECMV4_LLD_SII_MISSING_ERROR_ACK ((uint32_t)0xC0CC0007L)

//
// MessageId: ERR_ECMV4_LLD_STATE_CHANGE_FAILED
//
// MessageText:
//
// LLD: ESM State Change failed.
//
#define ERR_ECMV4_LLD_STATE_CHANGE_FAILED ((uint32_t)0xC0CC0008L)

//
// MessageId: ERR_ECMV4_LLD_UNEXPECTED_AL_STATUS
//
// MessageText:
//
// LLD: Unexpected ALSTATUS.
//
#define ERR_ECMV4_LLD_UNEXPECTED_AL_STATUS ((uint32_t)0xC0CC0009L)

//
// MessageId: ERR_ECMV4_LLD_UNEXPECTED_WKC
//
// MessageText:
//
// LLD: Unexpected Working Count.
//
#define ERR_ECMV4_LLD_UNEXPECTED_WKC     ((uint32_t)0xC0CC000AL)

//
// MessageId: ERR_ECMV4_LLD_MAILBOX_NOT_AVAILABLE
//
// MessageText:
//
// LLD: Mailbox not available.
//
#define ERR_ECMV4_LLD_MAILBOX_NOT_AVAILABLE ((uint32_t)0xC0CC000BL)

//
// MessageId: ERR_ECMV4_LLD_MAILBOX_MESSAGE_TOO_LARGE
//
// MessageText:
//
// LLD: Mailbox message too large.
//
#define ERR_ECMV4_LLD_MAILBOX_MESSAGE_TOO_LARGE ((uint32_t)0xC0CC000CL)

//
// MessageId: ERR_ECMV4_LLD_CONFIGURATION_IN_PROGRESS
//
// MessageText:
//
// LLD: Configuration in progress.
//
#define ERR_ECMV4_LLD_CONFIGURATION_IN_PROGRESS ((uint32_t)0xC0CC000DL)

//
// MessageId: ERR_ECMV4_LLD_TOO_MANY_CYCLIC_FRAMES
//
// MessageText:
//
// LLD: Too many cyclic frames.
//
#define ERR_ECMV4_LLD_TOO_MANY_CYCLIC_FRAMES ((uint32_t)0xC0CC000EL)

//
// MessageId: ERR_ECMV4_LLD_CYCLIC_FRAME_EXCEEDS_MTU
//
// MessageText:
//
// LLD: SII Checksum Error.
//
#define ERR_ECMV4_LLD_CYCLIC_FRAME_EXCEEDS_MTU ((uint32_t)0xC0CC000FL)

//
// MessageId: ERR_ECMV4_LLD_INVALID_CYCLIC_TELEGRAM_CONFIG
//
// MessageText:
//
// LLD: Invalid cyclic telegram config.
//
#define ERR_ECMV4_LLD_INVALID_CYCLIC_TELEGRAM_CONFIG ((uint32_t)0xC0CC0010L)

//
// MessageId: ERR_ECMV4_LLD_BUILDING_COPY_ROUTINES_FAILED
//
// MessageText:
//
// LLD: Building copy routines failed.
//
#define ERR_ECMV4_LLD_BUILDING_COPY_ROUTINES_FAILED ((uint32_t)0xC0CC0011L)

//
// MessageId: ERR_ECMV4_LLD_UNSUPPORTED_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// LLD: Unsupported slave station address.
//
#define ERR_ECMV4_LLD_UNSUPPORTED_SLAVE_STATION_ADDRESS ((uint32_t)0xC0CC0012L)

//
// MessageId: ERR_ECMV4_LLD_STATION_ADDRESS_NOT_ALLOWED
//
// MessageText:
//
// LLD: Station address not allowed.
//
#define ERR_ECMV4_LLD_STATION_ADDRESS_NOT_ALLOWED ((uint32_t)0xC0CC0013L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_STD_TX_MBX_PHYS_OFFSET
//
// MessageText:
//
// LLD: Invalid Std TxMbx PhysOffset.
//
#define ERR_ECMV4_LLD_INVALID_STD_TX_MBX_PHYS_OFFSET ((uint32_t)0xC0CC0014L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_STD_RX_MBX_PHYS_OFFSET
//
// MessageText:
//
// LLD: Invalid Std RxMbx PhysOffset.
//
#define ERR_ECMV4_LLD_INVALID_STD_RX_MBX_PHYS_OFFSET ((uint32_t)0xC0CC0015L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_BOOT_TX_MBX_PHYS_OFFSET
//
// MessageText:
//
// LLD: Invalid Boot TxMbx PhysOffset.
//
#define ERR_ECMV4_LLD_INVALID_BOOT_TX_MBX_PHYS_OFFSET ((uint32_t)0xC0CC0016L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_BOOT_RX_MBX_PHYS_OFFSET
//
// MessageText:
//
// LLD: Invalid Boot RxMbx PhysOffset.
//
#define ERR_ECMV4_LLD_INVALID_BOOT_RX_MBX_PHYS_OFFSET ((uint32_t)0xC0CC0017L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_STD_TX_MBX_SM_NO
//
// MessageText:
//
// LLD: Invalid Std TxMbx SyncManager number.
//
#define ERR_ECMV4_LLD_INVALID_STD_TX_MBX_SM_NO ((uint32_t)0xC0CC0018L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_STD_RX_MBX_SM_NO
//
// MessageText:
//
// LLD: Invalid Std RxMbx SyncManager number.
//
#define ERR_ECMV4_LLD_INVALID_STD_RX_MBX_SM_NO ((uint32_t)0xC0CC0019L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_BOOT_TX_MBX_SM_NO
//
// MessageText:
//
// LLD: Invalid Boot TxMbx SyncManager number.
//
#define ERR_ECMV4_LLD_INVALID_BOOT_TX_MBX_SM_NO ((uint32_t)0xC0CC001AL)

//
// MessageId: ERR_ECMV4_LLD_INVALID_BOOT_RX_MBX_SM_NO
//
// MessageText:
//
// LLD: Invalid Boot RxMbx SyncManager number.
//
#define ERR_ECMV4_LLD_INVALID_BOOT_RX_MBX_SM_NO ((uint32_t)0xC0CC001BL)

//
// MessageId: ERR_ECMV4_LLD_UNCONFIGURED_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// LLD: Unconfigured slave station address.
//
#define ERR_ECMV4_LLD_UNCONFIGURED_SLAVE_STATION_ADDRESS ((uint32_t)0xC0CC001CL)

//
// MessageId: ERR_ECMV4_LLD_WRONG_SLAVE_STATE
//
// MessageText:
//
// LLD: Wrong slave state.
//
#define ERR_ECMV4_LLD_WRONG_SLAVE_STATE  ((uint32_t)0xC0CC001DL)

//
// MessageId: ERR_ECMV4_LLD_CYCLE_TIME_TOO_SMALL
//
// MessageText:
//
// LLD: Cycle time too short.
//
#define ERR_ECMV4_LLD_CYCLE_TIME_TOO_SMALL ((uint32_t)0xC0CC001EL)

//
// MessageId: ERR_ECMV4_LLD_REPETITION_COUNT_NOT_SUPPORTED
//
// MessageText:
//
// LLD: Repetition count not supported.
//
#define ERR_ECMV4_LLD_REPETITION_COUNT_NOT_SUPPORTED ((uint32_t)0xC0CC001FL)

//
// MessageId: ERR_ECMV4_LLD_INVALID_CALLBACK_TYPE
//
// MessageText:
//
// LLD: Invalid callback type.
//
#define ERR_ECMV4_LLD_INVALID_CALLBACK_TYPE ((uint32_t)0xC0CC0020L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_CYCLE_MULTIPLIER
//
// MessageText:
//
// LLD: Invalid cycle multiplier.
//
#define ERR_ECMV4_LLD_INVALID_CYCLE_MULTIPLIER ((uint32_t)0xC0CC0021L)

//
// MessageId: ERR_ECMV4_LLD_UNKNOWN_ERROR
//
// MessageText:
//
// LLD: Unknown error.
//
#define ERR_ECMV4_LLD_UNKNOWN_ERROR      ((uint32_t)0xC0CC0022L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_REG_LENGTH
//
// MessageText:
//
// LLD: Invalid register length.
//
#define ERR_ECMV4_LLD_INVALID_REG_LENGTH ((uint32_t)0xC0CC0023L)

//
// MessageId: ERR_ECMV4_LLD_INVALID_PARAMETER
//
// MessageText:
//
// LLD: Invalid parameter.
//
#define ERR_ECMV4_LLD_INVALID_PARAMETER  ((uint32_t)0xC0CC0024L)

//
// MessageId: ERR_ECMV4_LLD_IRQ_NOT_AVAILABLE
//
// MessageText:
//
// LLD: IRQ not available.
//
#define ERR_ECMV4_LLD_IRQ_NOT_AVAILABLE  ((uint32_t)0xC0CC0025L)

//
// MessageId: ERR_ECMV4_LLD_IOMEM_IRQ_NOT_AVAILABLE
//
// MessageText:
//
// LLD: I/O Mem or IRQ not available.
//
#define ERR_ECMV4_LLD_IOMEM_IRQ_NOT_AVAILABLE ((uint32_t)0xC0CC0026L)

//
// MessageId: ERR_ECMV4_LLD_HW_INIT_FAILED
//
// MessageText:
//
// LLD: Hardware init failed.
//
#define ERR_ECMV4_LLD_HW_INIT_FAILED     ((uint32_t)0xC0CC0027L)

//
// MessageId: ERR_ECMV4_LLD_MUTEX_CREATION_FAILED
//
// MessageText:
//
// LLD: Mutex creation failed.
//
#define ERR_ECMV4_LLD_MUTEX_CREATION_FAILED ((uint32_t)0xC0CC0028L)

//
// MessageId: ERR_ECMV4_LLD_DC_RX_LATCH_COMMAND_REQUIRED_FOR_DC
//
// MessageText:
//
// LLD: DC Rx-Latch command required for DC.
//
#define ERR_ECMV4_LLD_DC_RX_LATCH_COMMAND_REQUIRED_FOR_DC ((uint32_t)0xC0CC0029L)

//
// MessageId: ERR_ECMV4_LLD_TX_PROCESS_IMAGE_EXCEEDED
//
// MessageText:
//
// LLD: Tx process image exceeded.
//
#define ERR_ECMV4_LLD_TX_PROCESS_IMAGE_EXCEEDED ((uint32_t)0xC0CC002AL)

//
// MessageId: ERR_ECMV4_LLD_RX_PROCESS_IMAGE_EXCEEDED
//
// MessageText:
//
// LLD: Rx process image exceeded.
//
#define ERR_ECMV4_LLD_RX_PROCESS_IMAGE_EXCEEDED ((uint32_t)0xC0CC002BL)

//
// MessageId: ERR_ECMV4_LLD_MBX_STATE_IMAGE_EXCEEDED
//
// MessageText:
//
// LLD: MbxState image exceeded.
//
#define ERR_ECMV4_LLD_MBX_STATE_IMAGE_EXCEEDED ((uint32_t)0xC0CC002CL)

//
// MessageId: ERR_ECMV4_LLD_RESULT_DUPLICATE_BWR_RX_LATCH_CMD
//
// MessageText:
//
// LLD: Duplicate BWR RX Latch Cmd.
//
#define ERR_ECMV4_LLD_RESULT_DUPLICATE_BWR_RX_LATCH_CMD ((uint32_t)0xC0CC002DL)

//
// MessageId: ERR_ECMV4_LLD_RESULT_DUPLICATE_EXT_SYSTIME_CONTROL_CMD
//
// MessageText:
//
// LLD: Duplicate ExtSync Control Cmd.
//
#define ERR_ECMV4_LLD_RESULT_DUPLICATE_EXT_SYSTIME_CONTROL_CMD ((uint32_t)0xC0CC002EL)

//
// MessageId: ERR_ECMV4_LLD_CC_PROCESS_IMAGE_EXCEEDED
//
// MessageText:
//
// LLD: Cross communication process image exceeded.
//
#define ERR_ECMV4_LLD_CC_PROCESS_IMAGE_EXCEEDED ((uint32_t)0xC0CC002FL)

//
// MessageId: ERR_ECMV4_LLD_SII_TIMEOUT
//
// MessageText:
//
// LLD: SII Timeout.
//
#define ERR_ECMV4_LLD_SII_TIMEOUT        ((uint32_t)0xC0CC0030L)

//
// MessageId: ERR_ECMV4_LLD_BUS_NOT_ENABLED
//
// MessageText:
//
// LLD: Bus not enabled.
//
#define ERR_ECMV4_LLD_BUS_NOT_ENABLED    ((uint32_t)0xC0CC0031L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 EMC Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_EMC_BUS_IS_OFF
//
// MessageText:
//
// Bus is off.
//
#define ERR_ECMV4_EMC_BUS_IS_OFF         ((uint32_t)0x40CD0017L)

//
// MessageId: ERR_ECMV4_EMC_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// Request destination problem.
//
#define ERR_ECMV4_EMC_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0CD0001L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// Invalid slave station address.
//
#define ERR_ECMV4_EMC_INVALID_SLAVE_STATION_ADDRESS ((uint32_t)0xC0CD0002L)

//
// MessageId: ERR_ECMV4_EMC_CONFIGURATION_BUFFER_IS_OPEN
//
// MessageText:
//
// Configuration Buffer is open.
//
#define ERR_ECMV4_EMC_CONFIGURATION_BUFFER_IS_OPEN ((uint32_t)0xC0CD0003L)

//
// MessageId: ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION
//
// MessageText:
//
// Wrong state for reconfiguration.
//
#define ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION ((uint32_t)0xC0CD0004L)

//
// MessageId: ERR_ECMV4_EMC_CONFIGURATION_BUFFER_IS_NOT_OPEN
//
// MessageText:
//
// Configuration Buffer is not open.
//
#define ERR_ECMV4_EMC_CONFIGURATION_BUFFER_IS_NOT_OPEN ((uint32_t)0xC0CD0005L)

//
// MessageId: ERR_ECMV4_EMC_SLAVE_STATION_ADDRESS_ALREADY_IN_CONFIG
//
// MessageText:
//
// Slave station address is already in configuration.
//
#define ERR_ECMV4_EMC_SLAVE_STATION_ADDRESS_ALREADY_IN_CONFIG ((uint32_t)0xC0CD0006L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_STD_MBX_PARAMETERS
//
// MessageText:
//
// Invalid Std Mbx parameters.
//
#define ERR_ECMV4_EMC_INVALID_STD_MBX_PARAMETERS ((uint32_t)0xC0CD0007L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_BOOT_MBX_PARAMETERS
//
// MessageText:
//
// Invalid Boot Mbx parameters.
//
#define ERR_ECMV4_EMC_INVALID_BOOT_MBX_PARAMETERS ((uint32_t)0xC0CD0008L)

//
// MessageId: ERR_ECMV4_EMC_STD_MBX_SM_ARE_OVERLAPPING
//
// MessageText:
//
// Std Mbx SyncManagers are overlapping.
//
#define ERR_ECMV4_EMC_STD_MBX_SM_ARE_OVERLAPPING ((uint32_t)0xC0CD0009L)

//
// MessageId: ERR_ECMV4_EMC_BOOT_MBX_SM_ARE_OVERLAPPING
//
// MessageText:
//
// Boot Mbx SyncManagers are overlapping.
//
#define ERR_ECMV4_EMC_BOOT_MBX_SM_ARE_OVERLAPPING ((uint32_t)0xC0CD000AL)

//
// MessageId: ERR_ECMV4_EMC_SM_PARAMS_ALREADY_ADDED
//
// MessageText:
//
// SyncManager params already added.
//
#define ERR_ECMV4_EMC_SM_PARAMS_ALREADY_ADDED ((uint32_t)0xC0CD000BL)

//
// MessageId: ERR_ECMV4_EMC_INVALID_SM_NUMBER
//
// MessageText:
//
// Invalid SyncManager number.
//
#define ERR_ECMV4_EMC_INVALID_SM_NUMBER  ((uint32_t)0xC0CD000CL)

//
// MessageId: ERR_ECMV4_EMC_FMMU_PARAMS_ALREADY_ADDED
//
// MessageText:
//
// FMMU params already added.
//
#define ERR_ECMV4_EMC_FMMU_PARAMS_ALREADY_ADDED ((uint32_t)0xC0CD000DL)

//
// MessageId: ERR_ECMV4_EMC_INVALID_FMMU_NUMBER
//
// MessageText:
//
// Invalid FMMU number.
//
#define ERR_ECMV4_EMC_INVALID_FMMU_NUMBER ((uint32_t)0xC0CD000EL)

//
// MessageId: ERR_ECMV4_EMC_INVALID_MIN_STATE
//
// MessageText:
//
// Invalid Min State.
//
#define ERR_ECMV4_EMC_INVALID_MIN_STATE  ((uint32_t)0xC0CD000FL)

//
// MessageId: ERR_ECMV4_EMC_CYCLE_FRAME_AMOUNT_EXCEEDED
//
// MessageText:
//
// Cycle frame amount exceeded.
//
#define ERR_ECMV4_EMC_CYCLE_FRAME_AMOUNT_EXCEEDED ((uint32_t)0xC0CD0010L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_CYCLIC_FRAME_IN_CONFIGURATION
//
// MessageText:
//
// Invalid cyclic frame in configuration.
//
#define ERR_ECMV4_EMC_INVALID_CYCLIC_FRAME_IN_CONFIGURATION ((uint32_t)0xC0CD0011L)

//
// MessageId: ERR_ECMV4_EMC_CYCLE_FRAME_INDEX_NOT_VALID
//
// MessageText:
//
// Cycle frame index not valid.
//
#define ERR_ECMV4_EMC_CYCLE_FRAME_INDEX_NOT_VALID ((uint32_t)0xC0CD0012L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_TELEGRAM_LENGTH
//
// MessageText:
//
// Invalid telegram length.
//
#define ERR_ECMV4_EMC_INVALID_TELEGRAM_LENGTH ((uint32_t)0xC0CD0013L)

//
// MessageId: ERR_ECMV4_EMC_CYCLE_FRAME_LENGTH_EXCEEDED
//
// MessageText:
//
// Cycle frame length exceeded.
//
#define ERR_ECMV4_EMC_CYCLE_FRAME_LENGTH_EXCEEDED ((uint32_t)0xC0CD0014L)

//
// MessageId: ERR_ECMV4_EMC_AMOUNT_OF_TELEGRAMS_IN_CYCLIC_FRAMES_EXCEEDED
//
// MessageText:
//
// Amount of telegrams in cyclic frame exceeded.
//
#define ERR_ECMV4_EMC_AMOUNT_OF_TELEGRAMS_IN_CYCLIC_FRAMES_EXCEEDED ((uint32_t)0xC0CD0015L)

//
// MessageId: ERR_ECMV4_EMC_STATE_CHANGE_IN_PROGRESS
//
// MessageText:
//
// State change in progress.
//
#define ERR_ECMV4_EMC_STATE_CHANGE_IN_PROGRESS ((uint32_t)0xC0CD0016L)

//
// MessageId: ERR_ECMV4_EMC_TOO_MANY_SLAVES_GIVEN
//
// MessageText:
//
// Too many slaves given.
//
#define ERR_ECMV4_EMC_TOO_MANY_SLAVES_GIVEN ((uint32_t)0xC0CD0018L)

//
// MessageId: ERR_ECMV4_EMC_DUPLICATE_STATION_ADDRESS_IN_LIST
//
// MessageText:
//
// Duplicate station address in list.
//
#define ERR_ECMV4_EMC_DUPLICATE_STATION_ADDRESS_IN_LIST ((uint32_t)0xC0CD0019L)

//
// MessageId: ERR_ECMV4_EMC_CONFIGURATION_DATA_INCORRECT
//
// MessageText:
//
// Configuration data incorrect.
//
#define ERR_ECMV4_EMC_CONFIGURATION_DATA_INCORRECT ((uint32_t)0xC0CD001BL)

//
// MessageId: ERR_ECMV4_EMC_VENDORID_MISMATCH
//
// MessageText:
//
// Vendor Id mismatch.
//
#define ERR_ECMV4_EMC_VENDORID_MISMATCH  ((uint32_t)0xC0CD001CL)

//
// MessageId: ERR_ECMV4_EMC_PRODUCTCODE_MISMATCH
//
// MessageText:
//
// Product Code mismatch.
//
#define ERR_ECMV4_EMC_PRODUCTCODE_MISMATCH ((uint32_t)0xC0CD001DL)

//
// MessageId: ERR_ECMV4_EMC_REVISIONNO_MISMATCH
//
// MessageText:
//
// Revision Number mismatch.
//
#define ERR_ECMV4_EMC_REVISIONNO_MISMATCH ((uint32_t)0xC0CD001EL)

//
// MessageId: ERR_ECMV4_EMC_SERIALNO_MISMATCH
//
// MessageText:
//
// Serial Number mismatch.
//
#define ERR_ECMV4_EMC_SERIALNO_MISMATCH  ((uint32_t)0xC0CD001FL)

//
// MessageId: ERR_ECMV4_EMC_LOST_CONNECTION
//
// MessageText:
//
// Lost Connection.
//
#define ERR_ECMV4_EMC_LOST_CONNECTION    ((uint32_t)0xC0CD0020L)

//
// MessageId: ERR_ECMV4_EMC_UNKNOWN_STATE_CHANGE_HAPPENED
//
// MessageText:
//
// Unknown state change happened.
//
#define ERR_ECMV4_EMC_UNKNOWN_STATE_CHANGE_HAPPENED ((uint32_t)0xC0CD0021L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_STATE_CHANGE_HAPPENED
//
// MessageText:
//
// Unexpected state change happened.
//
#define ERR_ECMV4_EMC_UNEXPECTED_STATE_CHANGE_HAPPENED ((uint32_t)0xC0CD0022L)

//
// MessageId: ERR_ECMV4_EMC_SLAVE_CHANGED_STATE
//
// MessageText:
//
// Slave changed state.
//
#define ERR_ECMV4_EMC_SLAVE_CHANGED_STATE ((uint32_t)0xC0CD0023L)

//
// MessageId: ERR_ECMV4_EMC_FILE_PROVIDER_INIT_FAILED
//
// MessageText:
//
// File Provider init failed.
//
#define ERR_ECMV4_EMC_FILE_PROVIDER_INIT_FAILED ((uint32_t)0xC0CD0024L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_TIMESTAMP_ERROR
//
// MessageText:
//
// DC Rx Timestamp Error.
//
#define ERR_ECMV4_EMC_DC_RX_TIMESTAMP_ERROR ((uint32_t)0xC0CD0026L)

//
// MessageId: ERR_ECMV4_EMC_DC_MASTER_PORT_TIMESTAMP_ERROR
//
// MessageText:
//
// DC Master Port Timestamp Error.
//
#define ERR_ECMV4_EMC_DC_MASTER_PORT_TIMESTAMP_ERROR ((uint32_t)0xC0CD0027L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_SLAVE_INDEX
//
// MessageText:
//
// Invalid slave index.
//
#define ERR_ECMV4_EMC_INVALID_SLAVE_INDEX ((uint32_t)0xC0CD0028L)

//
// MessageId: ERR_ECMV4_EMC_WRONG_MASTER_STATE
//
// MessageText:
//
// Wrong master state.
//
#define ERR_ECMV4_EMC_WRONG_MASTER_STATE ((uint32_t)0xC0CD0029L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_TRANSFER_ID
//
// MessageText:
//
// Invalid transfer id.
//
#define ERR_ECMV4_EMC_INVALID_TRANSFER_ID ((uint32_t)0xC0CD002AL)

//
// MessageId: ERR_ECMV4_EMC_INVALID_SEGMENTATION
//
// MessageText:
//
// Invalid segmentation.
//
#define ERR_ECMV4_EMC_INVALID_SEGMENTATION ((uint32_t)0xC0CD002BL)

//
// MessageId: ERR_ECMV4_EMC_EOE_IP_PARAMS_ALREADY_ADDED
//
// MessageText:
//
// EoE IP Parameter already added.
//
#define ERR_ECMV4_EMC_EOE_IP_PARAMS_ALREADY_ADDED ((uint32_t)0xC0CD002CL)

//
// MessageId: ERR_ECMV4_EMC_EOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// EoE Support not available.
//
#define ERR_ECMV4_EMC_EOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0CD002DL)

//
// MessageId: ERR_ECMV4_EMC_END_CONFIGURATION_IN_PROGRESS
//
// MessageText:
//
// End configuration in progress.
//
#define ERR_ECMV4_EMC_END_CONFIGURATION_IN_PROGRESS ((uint32_t)0xC0CD002EL)

//
// MessageId: ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_BUS_IS_ON
//
// MessageText:
//
// Wrong state for reconfiguration: Bus Is On.
//
#define ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_BUS_IS_ON ((uint32_t)0xC0CD002FL)

//
// MessageId: ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_BUS_SCAN_ACTIVE
//
// MessageText:
//
// Wrong state for reconfiguration: Bus Scan is active.
//
#define ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_BUS_SCAN_ACTIVE ((uint32_t)0xC0CD0030L)

//
// MessageId: ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_IN_PROGRESS_TO_BUSOFF
//
// MessageText:
//
// Wrong state for reconfiguration: Bus Off in progress.
//
#define ERR_ECMV4_EMC_WRONG_STATE_FOR_RECONFIGURATION_IN_PROGRESS_TO_BUSOFF ((uint32_t)0xC0CD0031L)

//
// MessageId: ERR_ECMV4_EMC_NO_DIAG_ENTRY_AVAILABLE
//
// MessageText:
//
// No diag entry available.
//
#define ERR_ECMV4_EMC_NO_DIAG_ENTRY_AVAILABLE ((uint32_t)0xC0CD0032L)

//
// MessageId: ERR_ECMV4_EMC_SLAVE_SYNC_PARAMS_NOT_POSSIBLE_WITHOUT_WORKING_DC
//
// MessageText:
//
// Slave Sync parameters are not possible without working DC.
//
#define ERR_ECMV4_EMC_SLAVE_SYNC_PARAMS_NOT_POSSIBLE_WITHOUT_WORKING_DC ((uint32_t)0xC0CD0033L)

//
// MessageId: ERR_ECMV4_EMC_MANDATORY_SLAVE_MISSING
//
// MessageText:
//
// Mandatory slave missing.
//
#define ERR_ECMV4_EMC_MANDATORY_SLAVE_MISSING ((uint32_t)0xC0CD0034L)

//
// MessageId: ERR_ECMV4_EMC_WRONG_SLAVE_AT_POSITION
//
// MessageText:
//
// Wrong slave at position.
//
#define ERR_ECMV4_EMC_WRONG_SLAVE_AT_POSITION ((uint32_t)0xC0CD0035L)

//
// MessageId: ERR_ECMV4_EMC_NO_DC_REF_CLOCK
//
// MessageText:
//
// No DC Reference clock available.
//
#define ERR_ECMV4_EMC_NO_DC_REF_CLOCK    ((uint32_t)0xC0CD0036L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_DC_REF_CLOCK
//
// MessageText:
//
// Invalid DC reference clock.
//
#define ERR_ECMV4_EMC_INVALID_DC_REF_CLOCK ((uint32_t)0xC0CD0038L)

//
// MessageId: ERR_ECMV4_EMC_COE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// CoE Support not available.
//
#define ERR_ECMV4_EMC_COE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0CD0039L)

//
// MessageId: ERR_ECMV4_EMC_SOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// SoE Support not available.
//
#define ERR_ECMV4_EMC_SOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0CD003AL)

//
// MessageId: ERR_ECMV4_EMC_FOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// FoE Support not available.
//
#define ERR_ECMV4_EMC_FOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0CD003BL)

//
// MessageId: ERR_ECMV4_EMC_AOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// AoE Support not available.
//
#define ERR_ECMV4_EMC_AOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0CD003CL)

//
// MessageId: ERR_ECMV4_EMC_RECONNECTED
//
// MessageText:
//
// Slave reconnected.
//
#define ERR_ECMV4_EMC_RECONNECTED        ((uint32_t)0x40CD003EL)

//
// MessageId: ERR_ECMV4_EMC_DC_STOPPED
//
// MessageText:
//
// DC ARMW/FRMW has been stopped.
//
#define ERR_ECMV4_EMC_DC_STOPPED         ((uint32_t)0x80CD003FL)

//
// MessageId: ERR_ECMV4_EMC_STOPPED_DUE_SYNC_ERROR
//
// MessageText:
//
// Stopped due sync error.
//
#define ERR_ECMV4_EMC_STOPPED_DUE_SYNC_ERROR ((uint32_t)0xC0CD0040L)

//
// MessageId: ERR_ECMV4_EMC_MANDATORY_SLAVE_NOT_IN_OP
//
// MessageText:
//
// Mandatory slave is not in OP.
//
#define ERR_ECMV4_EMC_MANDATORY_SLAVE_NOT_IN_OP ((uint32_t)0xC0CD0041L)

//
// MessageId: ERR_ECMV4_EMC_BUS_CYCLE_TIME_NOT_POSSIBLE
//
// MessageText:
//
// Bus cycle time not possible.
//
#define ERR_ECMV4_EMC_BUS_CYCLE_TIME_NOT_POSSIBLE ((uint32_t)0xC0CD0042L)

//
// MessageId: ERR_ECMV4_EMC_TOPOLOGY_ERROR_DETECTED
//
// MessageText:
//
// Topology error detected.
//
#define ERR_ECMV4_EMC_TOPOLOGY_ERROR_DETECTED ((uint32_t)0xC0CD0043L)

//
// MessageId: ERR_ECMV4_EMC_TOPOLOGY_MISMATCH_DETECTED
//
// MessageText:
//
// Topology mismatch detected.
//
#define ERR_ECMV4_EMC_TOPOLOGY_MISMATCH_DETECTED ((uint32_t)0xC0CD0044L)

//
// MessageId: ERR_ECMV4_EMC_NO_VALID_TOPOLOGY_CONFIGURATION_DATA
//
// MessageText:
//
// No valid topology configuration data.
//
#define ERR_ECMV4_EMC_NO_VALID_TOPOLOGY_CONFIGURATION_DATA ((uint32_t)0xC0CD0045L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0
//
// MessageText:
//
// Unexpected slave at port 0.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0 ((uint32_t)0xC0CD0046L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1
//
// MessageText:
//
// Unexpected slave at port 1.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1 ((uint32_t)0xC0CD0047L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT2
//
// MessageText:
//
// Unexpected slave at port 2.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT2 ((uint32_t)0xC0CD0048L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT3
//
// MessageText:
//
// Unexpected slave at port 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT3 ((uint32_t)0xC0CD0049L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_CONNECTED
//
// MessageText:
//
// Unexpected slave connected.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_CONNECTED ((uint32_t)0xC0CD004AL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0
//
// MessageText:
//
// Missing slave at port 0.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0 ((uint32_t)0xC0CD004BL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1
//
// MessageText:
//
// Missing slave at port 1.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1 ((uint32_t)0xC0CD004CL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT2
//
// MessageText:
//
// Missing slave at port 2.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT2 ((uint32_t)0xC0CD004DL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT3
//
// MessageText:
//
// Missing slave at port 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT3 ((uint32_t)0xC0CD004EL)

//
// MessageId: ERR_ECMV4_EMC_SLAVE_NOT_CHECKED
//
// MessageText:
//
// Slave not checked.
//
#define ERR_ECMV4_EMC_SLAVE_NOT_CHECKED  ((uint32_t)0xC0CD004FL)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1
//
// MessageText:
//
// Unexpected slaves at ports 0 and 1.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1 ((uint32_t)0xC0CD0050L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_2
//
// MessageText:
//
// Unexpected slaves at ports 0 and 2.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_2 ((uint32_t)0xC0CD0051L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_3
//
// MessageText:
//
// Unexpected slaves at ports 0 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_3 ((uint32_t)0xC0CD0052L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_2
//
// MessageText:
//
// Unexpected slaves at ports 1 and 2.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_2 ((uint32_t)0xC0CD0053L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_3
//
// MessageText:
//
// Unexpected slaves at ports 1 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_3 ((uint32_t)0xC0CD0054L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT2_3
//
// MessageText:
//
// Unexpected slaves at ports 2 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT2_3 ((uint32_t)0xC0CD0055L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1_2
//
// MessageText:
//
// Unexpected slaves at ports 0, 1 and 2.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1_2 ((uint32_t)0xC0CD0056L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1_3
//
// MessageText:
//
// Unexpected slaves at ports 0, 1 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_1_3 ((uint32_t)0xC0CD0057L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_2_3
//
// MessageText:
//
// Unexpected slaves at ports 0, 2 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT0_2_3 ((uint32_t)0xC0CD0058L)

//
// MessageId: ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_2_3
//
// MessageText:
//
// Unexpected slaves at ports 1, 2 and 3.
//
#define ERR_ECMV4_EMC_UNEXPECTED_SLAVE_AT_PORT1_2_3 ((uint32_t)0xC0CD0059L)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1
//
// MessageText:
//
// Missing slaves at ports 0 and 1.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1 ((uint32_t)0xC0CD005AL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_2
//
// MessageText:
//
// Missing slaves at ports 0 and 2.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_2 ((uint32_t)0xC0CD005BL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_3
//
// MessageText:
//
// Missing slaves at ports 0 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_3 ((uint32_t)0xC0CD005CL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_2
//
// MessageText:
//
// Missing slaves at ports 1 and 2.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_2 ((uint32_t)0xC0CD005DL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_3
//
// MessageText:
//
// Missing slaves at ports 1 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_3 ((uint32_t)0xC0CD005EL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT2_3
//
// MessageText:
//
// Missing slaves at ports 2 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT2_3 ((uint32_t)0xC0CD005FL)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1_2
//
// MessageText:
//
// Missing slaves at ports 0, 1 and 2.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1_2 ((uint32_t)0xC0CD0060L)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1_3
//
// MessageText:
//
// Missing slaves at ports 0, 1 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_1_3 ((uint32_t)0xC0CD0061L)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_2_3
//
// MessageText:
//
// Missing slaves at ports 0, 2 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT0_2_3 ((uint32_t)0xC0CD0062L)

//
// MessageId: ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_2_3
//
// MessageText:
//
// Missing slaves at ports 1, 2 and 3.
//
#define ERR_ECMV4_EMC_MISSING_SLAVE_AT_PORT1_2_3 ((uint32_t)0xC0CD0063L)

//
// MessageId: ERR_ECMV4_EMC_HC_PARTICIPANT_NOT_ALLOWED_IN_MANDATORY_SLAVE_LIST
//
// MessageText:
//
// Hot Connect participant is not allowed in mandatory slave list.
//
#define ERR_ECMV4_EMC_HC_PARTICIPANT_NOT_ALLOWED_IN_MANDATORY_SLAVE_LIST ((uint32_t)0xC0CD0065L)

//
// MessageId: ERR_ECMV4_EMC_HC_PARTICIPANT_NOT_ALLOWED_IN_MULTIPLE_HC_GROUPS
//
// MessageText:
//
// Hot Connect participant is not allowed in multiple Hot Connect groups.
//
#define ERR_ECMV4_EMC_HC_PARTICIPANT_NOT_ALLOWED_IN_MULTIPLE_HC_GROUPS ((uint32_t)0xC0CD0066L)

//
// MessageId: ERR_ECMV4_EMC_HC_GROUP_HEAD_IS_NOT_LISTED_FOR_HC_DETECTION
//
// MessageText:
//
// Hot Connect Group Head is not listed for Hot Connect detection.
//
#define ERR_ECMV4_EMC_HC_GROUP_HEAD_IS_NOT_LISTED_FOR_HC_DETECTION ((uint32_t)0xC0CD0067L)

//
// MessageId: ERR_ECMV4_EMC_DC_SETUP_CALCULATION_ERROR
//
// MessageText:
//
// DC Setup calculation error.
//
#define ERR_ECMV4_EMC_DC_SETUP_CALCULATION_ERROR ((uint32_t)0xC0CD0068L)

//
// MessageId: ERR_ECMV4_EMC_NON_DC_SLAVE_MORE_THAN_2_PORTS_IN_DC_SETUP
//
// MessageText:
//
// A Slave without DC support has more than 2 ports in DC setup.
//
#define ERR_ECMV4_EMC_NON_DC_SLAVE_MORE_THAN_2_PORTS_IN_DC_SETUP ((uint32_t)0xC0CD0069L)

//
// MessageId: ERR_ECMV4_EMC_HC_GROUP_CONTAINS_NOT_CONFIGURED_SLAVE
//
// MessageText:
//
// Hot Connect Group contains not configured slave.
//
#define ERR_ECMV4_EMC_HC_GROUP_CONTAINS_NOT_CONFIGURED_SLAVE ((uint32_t)0xC0CD006AL)

//
// MessageId: ERR_ECMV4_EMC_ALCONTROL_TIMEOUT
//
// MessageText:
//
// ALControl Timeout.
//
#define ERR_ECMV4_EMC_ALCONTROL_TIMEOUT  ((uint32_t)0xC0CD006BL)

//
// MessageId: ERR_ECMV4_EMC_DC_MEASUREMENT_ERROR
//
// MessageText:
//
// DC Measurement Error.
//
#define ERR_ECMV4_EMC_DC_MEASUREMENT_ERROR ((uint32_t)0xC0CD006CL)

//
// MessageId: ERR_ECMV4_EMC_RX_DESTINATION_EXCEEDS_RX_IMAGE_SIZE
//
// MessageText:
//
// Rx Process data destination exceeds Rx Image Size.
//
#define ERR_ECMV4_EMC_RX_DESTINATION_EXCEEDS_RX_IMAGE_SIZE ((uint32_t)0xC0CD006DL)

//
// MessageId: ERR_ECMV4_EMC_TX_SOURCE_EXCEEDS_TX_IMAGE_SIZE
//
// MessageText:
//
// Tx Process data source exceeds Tx image Size.
//
#define ERR_ECMV4_EMC_TX_SOURCE_EXCEEDS_TX_IMAGE_SIZE ((uint32_t)0xC0CD006EL)

//
// MessageId: ERR_ECMV4_EMC_WCSTATEBIT_EXCEEDS_RX_IMAGE_SIZE
//
// MessageText:
//
// WcState bit exceeds Rx Image Size.
//
#define ERR_ECMV4_EMC_WCSTATEBIT_EXCEEDS_RX_IMAGE_SIZE ((uint32_t)0xC0CD006FL)

//
// MessageId: ERR_ECMV4_EMC_WKC_MAPPING_EXCEEDS_RX_IMAGE_SIZE
//
// MessageText:
//
// WKC mapping exceeds Rx Image Size.
//
#define ERR_ECMV4_EMC_WKC_MAPPING_EXCEEDS_RX_IMAGE_SIZE ((uint32_t)0xC0CD0070L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0
//
// MessageText:
//
// DC Rx-Latch Error at Port 0.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0 ((uint32_t)0xC0CD0071L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1
//
// MessageText:
//
// DC Rx-Latch Error at Port 1.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1 ((uint32_t)0xC0CD0072L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT2
//
// MessageText:
//
// DC Rx-Latch Error at Port 2.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT2 ((uint32_t)0xC0CD0073L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT3
//
// MessageText:
//
// DC Rx-Latch Error at Port 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT3 ((uint32_t)0xC0CD0074L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0 and 1.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1 ((uint32_t)0xC0CD0075L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_2
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0 and 2.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_2 ((uint32_t)0xC0CD0076L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_3 ((uint32_t)0xC0CD0077L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_2
//
// MessageText:
//
// DC Rx-Latch Error at Ports 1 and 2.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_2 ((uint32_t)0xC0CD0078L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 1 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_3 ((uint32_t)0xC0CD0079L)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT2_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 2 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT2_3 ((uint32_t)0xC0CD007AL)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_2
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0, 1 and 2.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_2 ((uint32_t)0xC0CD007BL)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0, 1 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_3 ((uint32_t)0xC0CD007CL)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_2_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0, 2 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_2_3 ((uint32_t)0xC0CD007DL)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_2_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 1, 2 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT1_2_3 ((uint32_t)0xC0CD007EL)

//
// MessageId: ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_2_3
//
// MessageText:
//
// DC Rx-Latch Error at Ports 0, 1, 2 and 3.
//
#define ERR_ECMV4_EMC_DC_RX_LATCH_ERROR_AT_PORT0_1_2_3 ((uint32_t)0xC0CD007FL)

//
// MessageId: ERR_ECMV4_EMC_ASSIGN_PDO_IS_MISSING_PDO_MAPPING
//
// MessageText:
//
// AssignPDO is missing PDO-Mapping.
//
#define ERR_ECMV4_EMC_ASSIGN_PDO_IS_MISSING_PDO_MAPPING ((uint32_t)0xC0CD0080L)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_TO_SAME_SM
//
// MessageText:
//
// ExtSync object is not mapped to same SyncManager.
//
#define ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_TO_SAME_SM ((uint32_t)0xC0CD0081L)

//
// MessageId: ERR_ECMV4_EMC_DUPLICATE_EXT_SYNC_OBJ
//
// MessageText:
//
// Duplicate ExtSync object.
//
#define ERR_ECMV4_EMC_DUPLICATE_EXT_SYNC_OBJ ((uint32_t)0xC0CD0082L)

//
// MessageId: ERR_ECMV4_EMC_UNSUPPORTED_EXT_SYNC_OBJ_RECORD
//
// MessageText:
//
// Unsupported ExtSync object record.
//
#define ERR_ECMV4_EMC_UNSUPPORTED_EXT_SYNC_OBJ_RECORD ((uint32_t)0xC0CD0083L)

//
// MessageId: ERR_ECMV4_EMC_UNSUPPORTED_MAPPING_OF_EXT_SYNC_OBJ_RECORD
//
// MessageText:
//
// Unsupported mapping of ExtSync object.
//
#define ERR_ECMV4_EMC_UNSUPPORTED_MAPPING_OF_EXT_SYNC_OBJ_RECORD ((uint32_t)0xC0CD0084L)

//
// MessageId: ERR_ECMV4_EMC_MISSING_MAPPING_OF_EXT_SYNC_OBJ_RECORD
//
// MessageText:
//
// Missing mapping of ExtSync object.
//
#define ERR_ECMV4_EMC_MISSING_MAPPING_OF_EXT_SYNC_OBJ_RECORD ((uint32_t)0xC0CD0085L)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_TO_SAME_FMMU
//
// MessageText:
//
// ExtSync object is not mapped to same FMMU.
//
#define ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_TO_SAME_FMMU ((uint32_t)0xC0CD0086L)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_OBJ_INTERNAL_ERROR
//
// MessageText:
//
// Internal error encountered with ExtSync object.
//
#define ERR_ECMV4_EMC_EXT_SYNC_OBJ_INTERNAL_ERROR ((uint32_t)0xC0CD0087L)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_IN_ONE_CYCLIC_CMD
//
// MessageText:
//
// ExtSync object is not mapped in one cyclic command.
//
#define ERR_ECMV4_EMC_EXT_SYNC_OBJ_IS_NOT_MAPPED_IN_ONE_CYCLIC_CMD ((uint32_t)0xC0CD0088L)

//
// MessageId: ERR_ECMV4_EMC_UNSUPPORTED_FMMU_MAPPING_OF_EXT_SYNC_OBJ_RECORD
//
// MessageText:
//
// Unsupported FMMU mapping of Ext Sync object.
//
#define ERR_ECMV4_EMC_UNSUPPORTED_FMMU_MAPPING_OF_EXT_SYNC_OBJ_RECORD ((uint32_t)0xC0CD0089L)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_REQUIRES_ADJUST_EXT_SYNC_CMD
//
// MessageText:
//
// ExtSync requires Adjust Ext Sync Cmd.
//
#define ERR_ECMV4_EMC_EXT_SYNC_REQUIRES_ADJUST_EXT_SYNC_CMD ((uint32_t)0xC0CD008AL)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_CMD_DOES_NOT_MATCH_XRMW_CMD
//
// MessageText:
//
// ExtSync command does not match xRMW command.
//
#define ERR_ECMV4_EMC_EXT_SYNC_CMD_DOES_NOT_MATCH_XRMW_CMD ((uint32_t)0xC0CD008BL)

//
// MessageId: ERR_ECMV4_EMC_EXT_SYNC_REQUIRES_XRMW_CMD
//
// MessageText:
//
// ExtSync command requires xRMW command.
//
#define ERR_ECMV4_EMC_EXT_SYNC_REQUIRES_XRMW_CMD ((uint32_t)0xC0CD008CL)

//
// MessageId: ERR_ECMV4_EMC_EXPLICIT_DEV_IDENT_FAILED_ALSTATUS
//
// MessageText:
//
// Explicit Device Identification failed (ALSTATUS).
//
#define ERR_ECMV4_EMC_EXPLICIT_DEV_IDENT_FAILED_ALSTATUS ((uint32_t)0xC0CD008DL)

//
// MessageId: ERR_ECMV4_EMC_EXPLICIT_DEV_IDENT_FAILED_REG
//
// MessageText:
//
// Explicit Device Identification failed (register).
//
#define ERR_ECMV4_EMC_EXPLICIT_DEV_IDENT_FAILED_REG ((uint32_t)0xC0CD008EL)

//
// MessageId: ERR_ECMV4_EMC_COPY_INFOS_FOUND_AT_UNMAPPED_RECEIVE_DATA
//
// MessageText:
//
// CopyInfos found at unmapped receive data.
//
#define ERR_ECMV4_EMC_COPY_INFOS_FOUND_AT_UNMAPPED_RECEIVE_DATA ((uint32_t)0xC0CD008FL)

//
// MessageId: ERR_ECMV4_EMC_COPY_INFO_RECEIVE_DATA_AREA_NOT_MATCHING
//
// MessageText:
//
// CopyInfo receive data area is not matching.
//
#define ERR_ECMV4_EMC_COPY_INFO_RECEIVE_DATA_AREA_NOT_MATCHING ((uint32_t)0xC0CD0090L)

//
// MessageId: ERR_ECMV4_EMC_SDO_UPLOAD_TOO_LONG
//
// MessageText:
//
// SDO Upload data is too long.
//
#define ERR_ECMV4_EMC_SDO_UPLOAD_TOO_LONG ((uint32_t)0xC0CD0091L)

//
// MessageId: ERR_ECMV4_EMC_SDO_UPLOAD_TOO_SHORT
//
// MessageText:
//
// SDO Upload data is too short.
//
#define ERR_ECMV4_EMC_SDO_UPLOAD_TOO_SHORT ((uint32_t)0xC0CD0092L)

//
// MessageId: ERR_ECMV4_EMC_SDO_UPLOAD_COMPARE_DOES_NOT_MATCH_EXPECTATION
//
// MessageText:
//
// SDO Upload compare does not match expectation.
//
#define ERR_ECMV4_EMC_SDO_UPLOAD_COMPARE_DOES_NOT_MATCH_EXPECTATION ((uint32_t)0xC0CD0093L)

//
// MessageId: ERR_ECMV4_EMC_SOE_READ_TOO_LONG
//
// MessageText:
//
// SoE Read data too long.
//
#define ERR_ECMV4_EMC_SOE_READ_TOO_LONG  ((uint32_t)0xC0CD0094L)

//
// MessageId: ERR_ECMV4_EMC_SOE_UPLOAD_TOO_SHORT
//
// MessageText:
//
// SoE Read data too short.
//
#define ERR_ECMV4_EMC_SOE_UPLOAD_TOO_SHORT ((uint32_t)0xC0CD0095L)

//
// MessageId: ERR_ECMV4_EMC_SOE_READ_COMPARE_DOES_NOT_MATCH_EXPECTATION
//
// MessageText:
//
// SoE Read compare does not match expectation.
//
#define ERR_ECMV4_EMC_SOE_READ_COMPARE_DOES_NOT_MATCH_EXPECTATION ((uint32_t)0xC0CD0096L)

//
// MessageId: ERR_ECMV4_EMC_REG_INITCMD_COMPARE_DOES_NOT_MATCH_EXPECTATION
//
// MessageText:
//
// Register InitCmd compare does not match expectation.
//
#define ERR_ECMV4_EMC_REG_INITCMD_COMPARE_DOES_NOT_MATCH_EXPECTATION ((uint32_t)0xC0CD0097L)

//
// MessageId: ERR_ECMV4_EMC_REDUNDANCY_PORT_ONLY_POSSIBLE_ONCE
//
// MessageText:
//
// Redundancy port only possible once.
//
#define ERR_ECMV4_EMC_REDUNDANCY_PORT_ONLY_POSSIBLE_ONCE ((uint32_t)0xC0CD0098L)

//
// MessageId: ERR_ECMV4_EMC_STARTUP_SCAN_SII_FAILED
//
// MessageText:
//
// Startup scan of SII failed.
//
#define ERR_ECMV4_EMC_STARTUP_SCAN_SII_FAILED ((uint32_t)0xC0CD0099L)

//
// MessageId: ERR_ECMV4_EMC_STARTUP_VERIFY_SII_FAILED
//
// MessageText:
//
// Startup verify of SII failed.
//
#define ERR_ECMV4_EMC_STARTUP_VERIFY_SII_FAILED ((uint32_t)0xC0CD009AL)

//
// MessageId: ERR_ECMV4_EMC_MAIN_PORT_NOT_CONNECTED
//
// MessageText:
//
// Main port not connected.
//
#define ERR_ECMV4_EMC_MAIN_PORT_NOT_CONNECTED ((uint32_t)0xC0CD009BL)

//
// MessageId: ERR_ECMV4_EMC_BUS_SCAN_TOO_MANY_SLAVES
//
// MessageText:
//
// Bus Scan: Too many slaves.
//
#define ERR_ECMV4_EMC_BUS_SCAN_TOO_MANY_SLAVES ((uint32_t)0xC0CD009CL)

//
// MessageId: ERR_ECMV4_EMC_BUS_SCAN_SPLIT_RING_NOT_SUPPORTED
//
// MessageText:
//
// Bus Scan: Split ring not supported.
//
#define ERR_ECMV4_EMC_BUS_SCAN_SPLIT_RING_NOT_SUPPORTED ((uint32_t)0xC0CD009DL)

//
// MessageId: ERR_ECMV4_EMC_BUS_SHUTDOWN
//
// MessageText:
//
// Bus Shutdown.
//
#define ERR_ECMV4_EMC_BUS_SHUTDOWN       ((uint32_t)0xC0CD009EL)

//
// MessageId: ERR_ECMV4_EMC_MASTER_ADDRESS_NOT_ALLOWED_AS_STATION_ADDRESS
//
// MessageText:
//
// Master address not allowed as station address.
//
#define ERR_ECMV4_EMC_MASTER_ADDRESS_NOT_ALLOWED_AS_STATION_ADDRESS ((uint32_t)0xC0CD009FL)

//
// MessageId: ERR_ECMV4_EMC_FIRST_STATION_HAS_INVALID_PORT_0
//
// MessageText:
//
// First Station has invalid Port 0.
//
#define ERR_ECMV4_EMC_FIRST_STATION_HAS_INVALID_PORT_0 ((uint32_t)0xC0CD00A0L)

//
// MessageId: ERR_ECMV4_EMC_STATION_HAS_INVALID_PORT
//
// MessageText:
//
// Station has invalid port.
//
#define ERR_ECMV4_EMC_STATION_HAS_INVALID_PORT ((uint32_t)0xC0CD00A1L)

//
// MessageId: ERR_ECMV4_EMC_STATION_HAS_NOT_LISTED_STATION_ADDRESS_IN_PORT
//
// MessageText:
//
// Station has not listed station address in port.
//
#define ERR_ECMV4_EMC_STATION_HAS_NOT_LISTED_STATION_ADDRESS_IN_PORT ((uint32_t)0xC0CD00A2L)

//
// MessageId: ERR_ECMV4_EMC_PORT_CONNECTION_BETWEEN_STATIONS_DOES_NOT_MATCH
//
// MessageText:
//
// Port connection between stations does not match.
//
#define ERR_ECMV4_EMC_PORT_CONNECTION_BETWEEN_STATIONS_DOES_NOT_MATCH ((uint32_t)0xC0CD00A3L)

//
// MessageId: ERR_ECMV4_EMC_STATION_HAS_ALREADY_USED_STATION_ADDRESS_IN_PORT
//
// MessageText:
//
// Station has already used station address in port.
//
#define ERR_ECMV4_EMC_STATION_HAS_ALREADY_USED_STATION_ADDRESS_IN_PORT ((uint32_t)0xC0CD00A4L)

//
// MessageId: ERR_ECMV4_EMC_INVALID_SM_PHYS_START_ADDRESS
//
// MessageText:
//
// Invalid SyncMan physical start address.
//
#define ERR_ECMV4_EMC_INVALID_SM_PHYS_START_ADDRESS ((uint32_t)0xC0CD00A5L)

//
// MessageId: ERR_ECMV4_EMC_DC_TOPOLOGY_ON_REDUNDANCY_PORT_NOT_SUPPORTED
//
// MessageText:
//
// DC Topology on redundancy port not supported.
//
#define ERR_ECMV4_EMC_DC_TOPOLOGY_ON_REDUNDANCY_PORT_NOT_SUPPORTED ((uint32_t)0xC0CD00A6L)

//
// MessageId: ERR_ECMV4_EMC_SM_ASSIGN_PDO_ALREADY_ADDED
//
// MessageText:
//
// SM-AssignPDO already added.
//
#define ERR_ECMV4_EMC_SM_ASSIGN_PDO_ALREADY_ADDED ((uint32_t)0xC0CD00A7L)

//
// MessageId: ERR_ECMV4_EMC_DC_BASE_SYNC_OFFSET_PERCENTAGE_OUT_OF_RANGE
//
// MessageText:
//
// DC Base Sync Offset out of range.
//
#define ERR_ECMV4_EMC_DC_BASE_SYNC_OFFSET_PERCENTAGE_OUT_OF_RANGE ((uint32_t)0xC0CD00A8L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 CoE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_COE_INITIALIZATION_ERROR
//
// MessageText:
//
// CoE Initialization Error.
//
#define ERR_ECMV4_COE_INITIALIZATION_ERROR ((uint32_t)0xC0CF0001L)

//
// MessageId: ERR_ECMV4_COE_INVALID_TRANSFER_HANDLE
//
// MessageText:
//
// CoE Invalid transfer handle.
//
#define ERR_ECMV4_COE_INVALID_TRANSFER_HANDLE ((uint32_t)0xC0CF0002L)

//
// MessageId: ERR_ECMV4_COE_NO_MAILBOX_AVAILABLE
//
// MessageText:
//
// CoE Mailbox not available.
//
#define ERR_ECMV4_COE_NO_MAILBOX_AVAILABLE ((uint32_t)0xC0CF0003L)

//
// MessageId: ERR_ECMV4_COE_INVALID_TRANSFER_STATE
//
// MessageText:
//
// CoE Invalid transfer state.
//
#define ERR_ECMV4_COE_INVALID_TRANSFER_STATE ((uint32_t)0xC0CF0004L)

//
// MessageId: ERR_ECMV4_COE_TRANSFER_SEGMENT_TOO_LONG
//
// MessageText:
//
// CoE Transfer-Segment too long.
//
#define ERR_ECMV4_COE_TRANSFER_SEGMENT_TOO_LONG ((uint32_t)0xC0CF0005L)

//
// MessageId: ERR_ECMV4_COE_SHUTTING_DOWN
//
// MessageText:
//
// CoE Shutting Down.
//
#define ERR_ECMV4_COE_SHUTTING_DOWN      ((uint32_t)0xC0CF0006L)

//
// MessageId: ERR_ECMV4_COE_MAX_TOTAL_BYTES_SMALLER_THAN_ACTUAL_TOTAL_BYTES
//
// MessageText:
//
// CoE Max Total Bytes is smaller than Actual Total Bytes.
//
#define ERR_ECMV4_COE_MAX_TOTAL_BYTES_SMALLER_THAN_ACTUAL_TOTAL_BYTES ((uint32_t)0xC0CF0007L)

//
// MessageId: ERR_ECMV4_COE_MAILBOX_TRANSMIT_FAILED
//
// MessageText:
//
// CoE Mailbox transmit failed.
//
#define ERR_ECMV4_COE_MAILBOX_TRANSMIT_FAILED ((uint32_t)0xC0CF0008L)

//
// MessageId: ERR_ECMV4_COE_TRANSFER_ABORTED
//
// MessageText:
//
// CoE Transfer aborted.
//
#define ERR_ECMV4_COE_TRANSFER_ABORTED   ((uint32_t)0xC0CF0009L)

//
// MessageId: ERR_ECMV4_COE_SDOINFO_INITIALIZATION_ERROR
//
// MessageText:
//
// CoE SDOINFO Initialization error.
//
#define ERR_ECMV4_COE_SDOINFO_INITIALIZATION_ERROR ((uint32_t)0xC0CF000AL)

//
// MessageId: ERR_ECMV4_COE_WRONG_SLAVE_STATE
//
// MessageText:
//
// CoE: Wrong slave state.
//
#define ERR_ECMV4_COE_WRONG_SLAVE_STATE  ((uint32_t)0xC0CF000BL)

//
// MessageId: ERR_ECMV4_COE_PROTOCOL_ERROR
//
// MessageText:
//
// CoE Protocol Error.
//
#define ERR_ECMV4_COE_PROTOCOL_ERROR     ((uint32_t)0xC0CF000CL)

//
// MessageId: ERR_ECMV4_NO_AOE_AVAILABLE
//
// MessageText:
//
// CoE: No AoE available.
//
#define ERR_ECMV4_NO_AOE_AVAILABLE       ((uint32_t)0xC0CF000DL)

//
// MessageId: ERR_ECMV4_COE_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// CoE: Request destination problem.
//
#define ERR_ECMV4_COE_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0CF000EL)

//
// MessageId: ERR_ECMV4_COE_INVALID_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// CoE: Invalid slave station address.
//
#define ERR_ECMV4_COE_INVALID_SLAVE_STATION_ADDRESS ((uint32_t)0xC0CF000FL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_TOGGLE_BIT_NOT_ALTERNATED
//
// MessageText:
//
// Toggle bit was not changed.
//
#define ERR_ECMV4_COE_ABORTCODE_TOGGLE_BIT_NOT_ALTERNATED ((uint32_t)0xC0CF8000L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_COMMAND_SPECIFIER_NOT_VALID
//
// MessageText:
//
// Client/Server command specifier not valid or unknown.
//
#define ERR_ECMV4_COE_ABORTCODE_COMMAND_SPECIFIER_NOT_VALID ((uint32_t)0xC0CF8001L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_PROTOCOL_TIMEOUT
//
// MessageText:
//
// SDO protocol timeout.
//
#define ERR_ECMV4_COE_ABORTCODE_PROTOCOL_TIMEOUT ((uint32_t)0xC0CF8002L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OUT_OF_MEMORY
//
// MessageText:
//
// Out of memory.
//
#define ERR_ECMV4_COE_ABORTCODE_OUT_OF_MEMORY ((uint32_t)0xC0CF8003L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_UNSUPPORTED_ACCESS
//
// MessageText:
//
// Unsupported access to an object.
//
#define ERR_ECMV4_COE_ABORTCODE_UNSUPPORTED_ACCESS ((uint32_t)0xC0CF8004L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_IS_WRITE_ONLY
//
// MessageText:
//
// Attempt to read a write only object.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_IS_WRITE_ONLY ((uint32_t)0xC0CF8005L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_IS_READ_ONLY
//
// MessageText:
//
// Attempt to write to a read only object.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_IS_READ_ONLY ((uint32_t)0xC0CF8006L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_SUBINDEX_CANNOT_BE_WRITTEN_SI0_NZ
//
// MessageText:
//
// Subindex cannot be written, Subindex 0 must be 0 for write access.
//
#define ERR_ECMV4_COE_ABORTCODE_SUBINDEX_CANNOT_BE_WRITTEN_SI0_NZ ((uint32_t)0xC0CF8007L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_COMPLETE_ACCESS_NOT_SUPPORTED
//
// MessageText:
//
// Complete Access not supported.
//
#define ERR_ECMV4_COE_ABORTCODE_COMPLETE_ACCESS_NOT_SUPPORTED ((uint32_t)0xC0CF8008L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_LENGTH_EXCEEDS_MAILBOX_SIZE
//
// MessageText:
//
// Object length exceeds mailbox size.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_LENGTH_EXCEEDS_MAILBOX_SIZE ((uint32_t)0xC0CF8009L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_MAPPED_TO_RXPDO_NO_WRITE
//
// MessageText:
//
// Object mapped to RxPDO. SDO Download blocked.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_MAPPED_TO_RXPDO_NO_WRITE ((uint32_t)0xC0CF800AL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_DOES_NOT_EXIST
//
// MessageText:
//
// The object does not exist in the object dictionary.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_DOES_NOT_EXIST ((uint32_t)0xC0CF800BL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_OBJECT_CANNOT_BE_PDO_MAPPED
//
// MessageText:
//
// The object cannot be mapped into the PDO.
//
#define ERR_ECMV4_COE_ABORTCODE_OBJECT_CANNOT_BE_PDO_MAPPED ((uint32_t)0xC0CF800CL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_PDO_LENGTH_WOULD_EXCEED
//
// MessageText:
//
// The number and length of the objects to be mapped would exceed the PDO length.
//
#define ERR_ECMV4_COE_ABORTCODE_PDO_LENGTH_WOULD_EXCEED ((uint32_t)0xC0CF800DL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_GEN_PARAM_INCOMPATIBILITY
//
// MessageText:
//
// General parameter incompatibility reason.
//
#define ERR_ECMV4_COE_ABORTCODE_GEN_PARAM_INCOMPATIBILITY ((uint32_t)0xC0CF800EL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_ACCESS_FAILED_DUE_TO_HW_ERROR
//
// MessageText:
//
// Access failed due to a hardware error.
//
#define ERR_ECMV4_COE_ABORTCODE_ACCESS_FAILED_DUE_TO_HW_ERROR ((uint32_t)0xC0CF800FL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_DATATYPE_DOES_NOT_MATCH
//
// MessageText:
//
// Data type does not match, length of service parameter does not match.
//
#define ERR_ECMV4_COE_ABORTCODE_DATATYPE_DOES_NOT_MATCH ((uint32_t)0xC0CF8010L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_DATATYPE_LENGTH_TOO_LONG
//
// MessageText:
//
// Data type does not match, service parameter too long.
//
#define ERR_ECMV4_COE_ABORTCODE_DATATYPE_LENGTH_TOO_LONG ((uint32_t)0xC0CF8011L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_DATATYPE_LENGTH_TOO_SHORT
//
// MessageText:
//
// Data type does not match, service parameter too short.
//
#define ERR_ECMV4_COE_ABORTCODE_DATATYPE_LENGTH_TOO_SHORT ((uint32_t)0xC0CF8012L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_SUBINDEX_DOES_NOT_EXIST
//
// MessageText:
//
// Subindex does not exist.
//
#define ERR_ECMV4_COE_ABORTCODE_SUBINDEX_DOES_NOT_EXIST ((uint32_t)0xC0CF8013L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_RANGE_OF_PARAMETER_EXCEEDED
//
// MessageText:
//
// Value range of parameter exceeded (only for write access).
//
#define ERR_ECMV4_COE_ABORTCODE_RANGE_OF_PARAMETER_EXCEEDED ((uint32_t)0xC0CF8014L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_VALUE_OF_PARAM_WRITTEN_TOO_HIGH
//
// MessageText:
//
// Value of parameter written too high.
//
#define ERR_ECMV4_COE_ABORTCODE_VALUE_OF_PARAM_WRITTEN_TOO_HIGH ((uint32_t)0xC0CF8015L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_VALUE_OF_PARAM_WRITTEN_TOO_LOW
//
// MessageText:
//
// Value of parameter written too low.
//
#define ERR_ECMV4_COE_ABORTCODE_VALUE_OF_PARAM_WRITTEN_TOO_LOW ((uint32_t)0xC0CF8016L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_MIN_VALUE_IS_LESS_THAN_MAX_VALUE
//
// MessageText:
//
// Maximum value is less than minimum value.
//
#define ERR_ECMV4_COE_ABORTCODE_MIN_VALUE_IS_LESS_THAN_MAX_VALUE ((uint32_t)0xC0CF8017L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_GENERAL_ERROR
//
// MessageText:
//
// General error.
//
#define ERR_ECMV4_COE_ABORTCODE_GENERAL_ERROR ((uint32_t)0xC0CF8018L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_NO_TRANSFER_TO_APP
//
// MessageText:
//
// Data cannot be transferred to or stored in the application.
//
#define ERR_ECMV4_COE_ABORTCODE_NO_TRANSFER_TO_APP ((uint32_t)0xC0CF8019L)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_LOCAL_CONTROL
//
// MessageText:
//
// Data cannot be transferred to or stored in the application because of local control.
//
#define ERR_ECMV4_COE_ABORTCODE_LOCAL_CONTROL ((uint32_t)0xC0CF801AL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_NO_TRANSFER_DUE_TO_CURRENT_STATE
//
// MessageText:
//
// Data cannot be transferred to or stored in the application because of the present device state.
//
#define ERR_ECMV4_COE_ABORTCODE_NO_TRANSFER_DUE_TO_CURRENT_STATE ((uint32_t)0xC0CF801BL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_NO_OBJECT_DICTIONARY_PRESENT
//
// MessageText:
//
// Object dictionary dynamic generation fails or no object dictionary is present.
//
#define ERR_ECMV4_COE_ABORTCODE_NO_OBJECT_DICTIONARY_PRESENT ((uint32_t)0xC0CF801CL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_UNKNOWN_ABORT_CODE
//
// MessageText:
//
// Unknown SDO abort code.
//
#define ERR_ECMV4_COE_ABORTCODE_UNKNOWN_ABORT_CODE ((uint32_t)0xC0CF801DL)

//
// MessageId: ERR_ECMV4_COE_ABORTCODE_GEN_INTERNAL_INCOMPAT
//
// MessageText:
//
// General internal incompatibility in the device.
//
#define ERR_ECMV4_COE_ABORTCODE_GEN_INTERNAL_INCOMPAT ((uint32_t)0xC0CF801EL)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 FoE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_FOE_INITIALIZATION_ERROR
//
// MessageText:
//
// FoE: Initialization Error.
//
#define ERR_ECMV4_FOE_INITIALIZATION_ERROR ((uint32_t)0xC0D10001L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_UNKNOWN_ERROR
//
// MessageText:
//
// FoE: Unknown Error.
//
#define ERR_ECMV4_FOE_ERROR_UNKNOWN_ERROR ((uint32_t)0xC0D10002L)

//
// MessageId: ERR_ECMV4_FOE_INVALID_TRANSFER_HANDLE
//
// MessageText:
//
// FoE: Invalid transfer handle.
//
#define ERR_ECMV4_FOE_INVALID_TRANSFER_HANDLE ((uint32_t)0xC0D10003L)

//
// MessageId: ERR_ECMV4_FOE_INVALID_TRANSFER_STATE
//
// MessageText:
//
// FoE: Invalid transfer state.
//
#define ERR_ECMV4_FOE_INVALID_TRANSFER_STATE ((uint32_t)0xC0D10004L)

//
// MessageId: ERR_ECMV4_FOE_INVALID_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// FoE: Invalid slave station address.
//
#define ERR_ECMV4_FOE_INVALID_SLAVE_STATION_ADDRESS ((uint32_t)0xC0D10005L)

//
// MessageId: ERR_ECMV4_FOE_WRONG_SLAVE_STATE
//
// MessageText:
//
// FoE: Wrong slave state.
//
#define ERR_ECMV4_FOE_WRONG_SLAVE_STATE  ((uint32_t)0xC0D10006L)

//
// MessageId: ERR_ECMV4_FOE_NO_MAILBOX_AVAILABLE
//
// MessageText:
//
// FoE: No mailbox available.
//
#define ERR_ECMV4_FOE_NO_MAILBOX_AVAILABLE ((uint32_t)0xC0D10007L)

//
// MessageId: ERR_ECMV4_FOE_TRANSFER_ABORTED
//
// MessageText:
//
// FoE: Transfer aborted.
//
#define ERR_ECMV4_FOE_TRANSFER_ABORTED   ((uint32_t)0xC0D10008L)

//
// MessageId: ERR_ECMV4_FOE_PROTOCOL_TIMEOUT
//
// MessageText:
//
// FoE: Protocol Timeout.
//
#define ERR_ECMV4_FOE_PROTOCOL_TIMEOUT   ((uint32_t)0xC0D10009L)

//
// MessageId: ERR_ECMV4_FOE_TRANSFER_SEGMENT_TOO_LONG
//
// MessageText:
//
// FoE: Transfer segment too long.
//
#define ERR_ECMV4_FOE_TRANSFER_SEGMENT_TOO_LONG ((uint32_t)0xC0D1000AL)

//
// MessageId: ERR_ECMV4_FOE_MAILBOX_TRANSMIT_FAILED
//
// MessageText:
//
// FoE: Mailbox transmit failed.
//
#define ERR_ECMV4_FOE_MAILBOX_TRANSMIT_FAILED ((uint32_t)0xC0D1000BL)

//
// MessageId: ERR_ECMV4_FOE_FILENAME_TOO_LONG
//
// MessageText:
//
// FoE: Filename too long.
//
#define ERR_ECMV4_FOE_FILENAME_TOO_LONG  ((uint32_t)0xC0D1000CL)

//
// MessageId: ERR_ECMV4_FOE_BUFFER_EXCEEDED
//
// MessageText:
//
// FoE: Buffer exceeded.
//
#define ERR_ECMV4_FOE_BUFFER_EXCEEDED    ((uint32_t)0xC0D1000DL)

//
// MessageId: ERR_ECMV4_FOE_FIRST_SEGMENT_SHOULD_NOT_BE_EMPTY
//
// MessageText:
//
// FoE: First segment should not be empty.
//
#define ERR_ECMV4_FOE_FIRST_SEGMENT_SHOULD_NOT_BE_EMPTY ((uint32_t)0xC0D1000EL)

//
// MessageId: ERR_ECMV4_FOE_SEGMENT_SHOULD_BE_EMPTY
//
// MessageText:
//
// FoE: Segment should be empty.
//
#define ERR_ECMV4_FOE_SEGMENT_SHOULD_BE_EMPTY ((uint32_t)0xC0D1000FL)

//
// MessageId: ERR_ECMV4_FOE_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// FoE: Request Destination Problem.
//
#define ERR_ECMV4_FOE_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0D10010L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_NOT_DEFINED
//
// MessageText:
//
// FoE: Not Defined.
//
#define ERR_ECMV4_FOE_ERROR_NOT_DEFINED  ((uint32_t)0xC0D18000L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_NOT_FOUND
//
// MessageText:
//
// FoE: Not Found.
//
#define ERR_ECMV4_FOE_ERROR_NOT_FOUND    ((uint32_t)0xC0D18001L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_ACCESS_DENIED
//
// MessageText:
//
// FoE: Access Denied.
//
#define ERR_ECMV4_FOE_ERROR_ACCESS_DENIED ((uint32_t)0xC0D18002L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_DISK_FULL
//
// MessageText:
//
// FoE: Disk full.
//
#define ERR_ECMV4_FOE_ERROR_DISK_FULL    ((uint32_t)0xC0D18003L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_ILLEGAL
//
// MessageText:
//
// FoE: Illegal.
//
#define ERR_ECMV4_FOE_ERROR_ILLEGAL      ((uint32_t)0xC0D18004L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_PACKET_NUMBER_WRONG
//
// MessageText:
//
// FoE: Packet number wrong.
//
#define ERR_ECMV4_FOE_ERROR_PACKET_NUMBER_WRONG ((uint32_t)0xC0D18005L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_ALREADY_EXISTS
//
// MessageText:
//
// FoE: Already exists.
//
#define ERR_ECMV4_FOE_ERROR_ALREADY_EXISTS ((uint32_t)0xC0D18006L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_NO_USER
//
// MessageText:
//
// FoE: No user.
//
#define ERR_ECMV4_FOE_ERROR_NO_USER      ((uint32_t)0xC0D18007L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_BOOTSTRAP_ONLY
//
// MessageText:
//
// FoE: Bootstrap only.
//
#define ERR_ECMV4_FOE_ERROR_BOOTSTRAP_ONLY ((uint32_t)0xC0D18008L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_NOT_BOOTSTRAP
//
// MessageText:
//
// FoE: Not Bootstrap.
//
#define ERR_ECMV4_FOE_ERROR_NOT_BOOTSTRAP ((uint32_t)0xC0D18009L)

//
// MessageId: ERR_ECMV4_FOE_ERROR_NO_RIGHTS
//
// MessageText:
//
// FoE: No rights.
//
#define ERR_ECMV4_FOE_ERROR_NO_RIGHTS    ((uint32_t)0xC0D1800AL)

//
// MessageId: ERR_ECMV4_FOE_ERROR_PROGRAM_ERROR
//
// MessageText:
//
// FoE: Program Error.
//
#define ERR_ECMV4_FOE_ERROR_PROGRAM_ERROR ((uint32_t)0xC0D1800BL)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 EoE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_EOE_INVALID_MAC_ADDRESS
//
// MessageText:
//
// EoE: Invalid MAC address.
//
#define ERR_ECMV4_EOE_INVALID_MAC_ADDRESS ((uint32_t)0xC0D00001L)

//
// MessageId: ERR_ECMV4_EOE_INVALID_CALLBACK_TYPE
//
// MessageText:
//
// EoE: Invalid callback type.
//
#define ERR_ECMV4_EOE_INVALID_CALLBACK_TYPE ((uint32_t)0xC0D00002L)

//
// MessageId: ERR_ECMV4_EOE_DESTINATION_UNREACHABLE
//
// MessageText:
//
// EoE: Destination unreachable.
//
#define ERR_ECMV4_EOE_DESTINATION_UNREACHABLE ((uint32_t)0xC0D00003L)

//
// MessageId: ERR_ECMV4_EOE_INVALID_EOE_RESPONSE
//
// MessageText:
//
// EoE: Invalid EoE Response.
//
#define ERR_ECMV4_EOE_INVALID_EOE_RESPONSE ((uint32_t)0xC0D00004L)

//
// MessageId: ERR_ECMV4_EOE_UNKNOWN_ERROR
//
// MessageText:
//
// EoE: Unknown Error.
//
#define ERR_ECMV4_EOE_UNKNOWN_ERROR      ((uint32_t)0xC0D00005L)

//
// MessageId: ERR_ECMV4_EOE_UNSPECIFIED_ERROR
//
// MessageText:
//
// EoE: Unspecified error.
//
#define ERR_ECMV4_EOE_UNSPECIFIED_ERROR  ((uint32_t)0xC0D00006L)

//
// MessageId: ERR_ECMV4_EOE_UNSUPPORTED_FRAME_TYPE
//
// MessageText:
//
// EoE: Unsupported frame type.
//
#define ERR_ECMV4_EOE_UNSUPPORTED_FRAME_TYPE ((uint32_t)0xC0D00007L)

//
// MessageId: ERR_ECMV4_EOE_NO_IP_SUPPORT
//
// MessageText:
//
// EoE: No IP support.
//
#define ERR_ECMV4_EOE_NO_IP_SUPPORT      ((uint32_t)0xC0D00008L)

//
// MessageId: ERR_ECMV4_EOE_DHCP_NOT_SUPPORTED
//
// MessageText:
//
// EoE: DHCP not supported.
//
#define ERR_ECMV4_EOE_DHCP_NOT_SUPPORTED ((uint32_t)0xC0D00009L)

//
// MessageId: ERR_ECMV4_EOE_NO_FILTER_SUPPORT
//
// MessageText:
//
// EoE: No filter support.
//
#define ERR_ECMV4_EOE_NO_FILTER_SUPPORT  ((uint32_t)0xC0D0000AL)

//
// MessageId: ERR_ECMV4_EOE_TIMEOUT
//
// MessageText:
//
// EoE: Timeout.
//
#define ERR_ECMV4_EOE_TIMEOUT            ((uint32_t)0xC0D0000BL)

//
// MessageId: ERR_ECMV4_EOE_SHUTTING_DOWN
//
// MessageText:
//
// EoE: Shutting Down.
//
#define ERR_ECMV4_EOE_SHUTTING_DOWN      ((uint32_t)0xC0D0000CL)

//
// MessageId: ERR_ECMV4_EOE_MASTER_ADDRESS_NOT_ALLOWED
//
// MessageText:
//
// EoE: Master address not allowed.
//
#define ERR_ECMV4_EOE_MASTER_ADDRESS_NOT_ALLOWED ((uint32_t)0xC0D0000DL)

//
// MessageId: ERR_ECMV4_EOE_CONFIGURATION_IS_NOT_OPEN
//
// MessageText:
//
// EoE: Configuration is not open.
//
#define ERR_ECMV4_EOE_CONFIGURATION_IS_NOT_OPEN ((uint32_t)0xC0D0000EL)

//
// MessageId: ERR_ECMV4_EOE_CONFIGURATION_IS_ALREADY_OPEN
//
// MessageText:
//
// EoE: Configuration is already open.
//
#define ERR_ECMV4_EOE_CONFIGURATION_IS_ALREADY_OPEN ((uint32_t)0xC0D0000FL)

//
// MessageId: ERR_ECMV4_EOE_DUPLICATE_IP_ADDRESS
//
// MessageText:
//
// EoE: Duplicate IP address.
//
#define ERR_ECMV4_EOE_DUPLICATE_IP_ADDRESS ((uint32_t)0xC0D00010L)

//
// MessageId: ERR_ECMV4_EOE_DUPLICATE_MAC_ADDRESS_ON_MULTIPLE_PORTS
//
// MessageText:
//
// EoE: Duplicate MAC address on multiple ports.
//
#define ERR_ECMV4_EOE_DUPLICATE_MAC_ADDRESS_ON_MULTIPLE_PORTS ((uint32_t)0xC0D00011L)

//
// MessageId: ERR_ECMV4_EOE_FRAME_TOO_LARGE
//
// MessageText:
//
// EoE: Frame too large.
//
#define ERR_ECMV4_EOE_FRAME_TOO_LARGE    ((uint32_t)0xC0D00012L)

//
// MessageId: ERR_ECMV4_EOE_IF_INITIALIZATION_ERROR
//
// MessageText:
//
// EoE IF: Initialization Error.
//
#define ERR_ECMV4_EOE_IF_INITIALIZATION_ERROR ((uint32_t)0xC0D00013L)

//
// MessageId: ERR_ECMV4_EOE_IF_NO_FRAME_AVAILABLE
//
// MessageText:
//
// EoE IF: No frame available.
//
#define ERR_ECMV4_EOE_IF_NO_FRAME_AVAILABLE ((uint32_t)0xC0D00014L)

//
// MessageId: ERR_ECMV4_EOE_LINK_DOWN
//
// MessageText:
//
// EoE: Link Down.
//
#define ERR_ECMV4_EOE_LINK_DOWN          ((uint32_t)0xC0D00015L)

//
// MessageId: ERR_ECMV4_EOE_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// EoE: Request Destination Problem.
//
#define ERR_ECMV4_EOE_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0D00016L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 SoE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_SOE_UNKNOWN_SOE_ERROR
//
// MessageText:
//
// Unknown SoE error.
//
#define ERR_ECMV4_SOE_UNKNOWN_SOE_ERROR  ((uint32_t)0xC0D20001L)

//
// MessageId: ERR_ECMV4_SOE_INITIALIZATION_ERROR
//
// MessageText:
//
// SoE: Initialization Error.
//
#define ERR_ECMV4_SOE_INITIALIZATION_ERROR ((uint32_t)0xC0D20002L)

//
// MessageId: ERR_ECMV4_SOE_INVALID_TRANSFER_HANDLE
//
// MessageText:
//
// SoE: Invalid transfer handle.
//
#define ERR_ECMV4_SOE_INVALID_TRANSFER_HANDLE ((uint32_t)0xC0D20003L)

//
// MessageId: ERR_ECMV4_SOE_NO_MAILBOX_AVAILABLE
//
// MessageText:
//
// SoE: No mailbox available.
//
#define ERR_ECMV4_SOE_NO_MAILBOX_AVAILABLE ((uint32_t)0xC0D20004L)

//
// MessageId: ERR_ECMV4_SOE_INVALID_TRANSFER_STATE
//
// MessageText:
//
// SoE: Invalid transfer state.
//
#define ERR_ECMV4_SOE_INVALID_TRANSFER_STATE ((uint32_t)0xC0D20005L)

//
// MessageId: ERR_ECMV4_SOE_TRANSFER_SEGMENT_TOO_LONG
//
// MessageText:
//
// SoE: Transfer segment too long.
//
#define ERR_ECMV4_SOE_TRANSFER_SEGMENT_TOO_LONG ((uint32_t)0xC0D20006L)

//
// MessageId: ERR_ECMV4_SOE_SHUTTING_DOWN
//
// MessageText:
//
// SoE: Shutting Down.
//
#define ERR_ECMV4_SOE_SHUTTING_DOWN      ((uint32_t)0xC0D20007L)

//
// MessageId: ERR_ECMV4_SOE_MAX_TOTAL_BYTES_SMALLER_THAN_ACTUAL_TOTAL_BYTES
//
// MessageText:
//
// SoE: Max Total bytes is smaller than actual total bytes.
//
#define ERR_ECMV4_SOE_MAX_TOTAL_BYTES_SMALLER_THAN_ACTUAL_TOTAL_BYTES ((uint32_t)0xC0D20008L)

//
// MessageId: ERR_ECMV4_SOE_MAILBOX_TRANSMIT_FAILED
//
// MessageText:
//
// SoE: Mailbox transmit failed.
//
#define ERR_ECMV4_SOE_MAILBOX_TRANSMIT_FAILED ((uint32_t)0xC0D20009L)

//
// MessageId: ERR_ECMV4_SOE_INVALID_SOE_HEADER
//
// MessageText:
//
// SoE: Invalid SoE header.
//
#define ERR_ECMV4_SOE_INVALID_SOE_HEADER ((uint32_t)0xC0D2000AL)

//
// MessageId: ERR_ECMV4_SOE_PROTOCOL_TIMEOUT
//
// MessageText:
//
// SoE: Protocol Timeout.
//
#define ERR_ECMV4_SOE_PROTOCOL_TIMEOUT   ((uint32_t)0xC0D2000BL)

//
// MessageId: ERR_ECMV4_SOE_PROTOCOL_ERROR
//
// MessageText:
//
// SoE: Protocol Error.
//
#define ERR_ECMV4_SOE_PROTOCOL_ERROR     ((uint32_t)0xC0D2000CL)

//
// MessageId: ERR_ECMV4_SOE_TRANSFER_ABORTED
//
// MessageText:
//
// SoE: Transfer aborted.
//
#define ERR_ECMV4_SOE_TRANSFER_ABORTED   ((uint32_t)0xC0D2000DL)

//
// MessageId: ERR_ECMV4_SOE_WRONG_SLAVE_STATE
//
// MessageText:
//
// SoE: Wrong slave state.
//
#define ERR_ECMV4_SOE_WRONG_SLAVE_STATE  ((uint32_t)0xC0D2000EL)

//
// MessageId: ERR_ECMV4_SOE_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// SoE: Request Destination Problem.
//
#define ERR_ECMV4_SOE_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0D2000FL)

//
// MessageId: ERR_ECMV4_SOE_NO_AOE_AVAILABLE
//
// MessageText:
//
// SoE: No AoE available.
//
#define ERR_ECMV4_SOE_NO_AOE_AVAILABLE   ((uint32_t)0xC0D20010L)

//
// MessageId: ERR_ECMV4_SOE_INVALID_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// SoE: Invalid slave station address.
//
#define ERR_ECMV4_SOE_INVALID_SLAVE_STATION_ADDRESS ((uint32_t)0xC0D20011L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NO_IDN
//
// MessageText:
//
// SoE: No IDN.
//
#define ERR_ECMV4_SOE_SSC_NO_IDN         ((uint32_t)0xC0D21001L)

//
// MessageId: ERR_ECMV4_SOE_SSC_INVALID_ACCESS_TO_ELEMENT_1
//
// MessageText:
//
// SoE: Invalid access to element 1.
//
#define ERR_ECMV4_SOE_SSC_INVALID_ACCESS_TO_ELEMENT_1 ((uint32_t)0xC0D21009L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NO_NAME
//
// MessageText:
//
// SoE: No Name.
//
#define ERR_ECMV4_SOE_SSC_NO_NAME        ((uint32_t)0xC0D22001L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NAME_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Name transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_NAME_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D22002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NAME_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Name transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_NAME_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D22003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NAME_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Name cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_NAME_CANNOT_BE_CHANGED ((uint32_t)0xC0D22004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NAME_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Name is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_NAME_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D22005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_ATTRIBUTE_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Attribute transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_ATTRIBUTE_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D23002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_ATTRIBUTE_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Attribute transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_ATTRIBUTE_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D23003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_ATTRIBUTE_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Attribute cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_ATTRIBUTE_CANNOT_BE_CHANGED ((uint32_t)0xC0D23004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_ATTRIBUTE_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Attribute is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_ATTRIBUTE_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D23005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NO_UNIT
//
// MessageText:
//
// SoE: No Unit.
//
#define ERR_ECMV4_SOE_SSC_NO_UNIT        ((uint32_t)0xC0D24001L)

//
// MessageId: ERR_ECMV4_SOE_SSC_UNIT_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Unit transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_UNIT_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D24002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_UNIT_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Name transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_UNIT_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D24003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_UNIT_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Unit cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_UNIT_CANNOT_BE_CHANGED ((uint32_t)0xC0D24004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_UNIT_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Unit is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_UNIT_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D24005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NO_MINIMUM_VALUE
//
// MessageText:
//
// SoE: No minimum value.
//
#define ERR_ECMV4_SOE_SSC_NO_MINIMUM_VALUE ((uint32_t)0xC0D25001L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Minimum value transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D25002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Minimum value transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D25003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Minimum value cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_CANNOT_BE_CHANGED ((uint32_t)0xC0D25004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Minimum value is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_MINIMUM_VALUE_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D25005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_NO_MAXIMUM_VALUE
//
// MessageText:
//
// SoE: No maximum value.
//
#define ERR_ECMV4_SOE_SSC_NO_MAXIMUM_VALUE ((uint32_t)0xC0D26001L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Maximum value transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D26002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Maximum value transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D26003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Maximum value cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_CANNOT_BE_CHANGED ((uint32_t)0xC0D26004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Maximum value is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_MAXIMUM_VALUE_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D26005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_TRANSMISSION_IS_TOO_SHORT
//
// MessageText:
//
// SoE: Operation data transmission is too short.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_TRANSMISSION_IS_TOO_SHORT ((uint32_t)0xC0D27002L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_TRANSMISSION_IS_TOO_LONG
//
// MessageText:
//
// SoE: Operation data transmission is too long.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_TRANSMISSION_IS_TOO_LONG ((uint32_t)0xC0D27003L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_CANNOT_BE_CHANGED
//
// MessageText:
//
// SoE: Operation data cannot be changed.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_CANNOT_BE_CHANGED ((uint32_t)0xC0D27004L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_THIS_TIME
//
// MessageText:
//
// SoE: Operation data is write protected at this time.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_THIS_TIME ((uint32_t)0xC0D27005L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_LOWER_THAN_MINIMUM_VALUE
//
// MessageText:
//
// SoE: Operation data is lower than minimum value.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_LOWER_THAN_MINIMUM_VALUE ((uint32_t)0xC0D27006L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_HIGHER_THAN_MAXIMUM_VALUE
//
// MessageText:
//
// SoE: Operation data is higher than maximum value.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_HIGHER_THAN_MAXIMUM_VALUE ((uint32_t)0xC0D27007L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_INVALID
//
// MessageText:
//
// SoE: Operation data is invalid.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_INVALID ((uint32_t)0xC0D27008L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_BY_PASSWORD
//
// MessageText:
//
// SoE: Operation data is write protected by password.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_BY_PASSWORD ((uint32_t)0xC0D27009L)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_DUE_CYCLICALLY_CONFIGURED
//
// MessageText:
//
// SoE: Operation data is write protected due to being cyclically configured.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_DUE_CYCLICALLY_CONFIGURED ((uint32_t)0xC0D2700AL)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_INVALID_INDIRECT_ADDRESSING
//
// MessageText:
//
// SoE: Invalid indirect addressing.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_INVALID_INDIRECT_ADDRESSING ((uint32_t)0xC0D2700BL)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_DUE_OTHER_SETTINGS
//
// MessageText:
//
// SoE: Operation data is write protected due other settings.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_DUE_OTHER_SETTINGS ((uint32_t)0xC0D2700CL)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_INVALID_FLOATING_POINT_NUMBER
//
// MessageText:
//
// SoE: Invalid floating point number.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_INVALID_FLOATING_POINT_NUMBER ((uint32_t)0xC0D2700DL)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_PARAMETERIZATION_LEVEL
//
// MessageText:
//
// SoE: Operation data is write protected at parameterization level.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_PARAMETERIZATION_LEVEL ((uint32_t)0xC0D2700EL)

//
// MessageId: ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_OPERATION_LEVEL
//
// MessageText:
//
// SoE: Operation data is write protected at operation level.
//
#define ERR_ECMV4_SOE_SSC_OPDATA_IS_WRITE_PROTECTED_AT_OPERATION_LEVEL ((uint32_t)0xC0D2700FL)

//
// MessageId: ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_ALREADY_ACTIVE
//
// MessageText:
//
// SoE: Procedure command already active.
//
#define ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_ALREADY_ACTIVE ((uint32_t)0xC0D27010L)

//
// MessageId: ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_INTERRUPTIBLE
//
// MessageText:
//
// SoE: Procedure command is not interruptible.
//
#define ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_INTERRUPTIBLE ((uint32_t)0xC0D27011L)

//
// MessageId: ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_EXECUTABLE_AT_THIS_TIME
//
// MessageText:
//
// SoE: Procedure command is not executable at this time.
//
#define ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_EXECUTABLE_AT_THIS_TIME ((uint32_t)0xC0D27012L)

//
// MessageId: ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_EXECUTABLE_INVALID_PARAM
//
// MessageText:
//
// SoE: Procedure command is not executable due to invalid parameter.
//
#define ERR_ECMV4_SOE_SSC_PROCEDURE_COMMAND_NOT_EXECUTABLE_INVALID_PARAM ((uint32_t)0xC0D27013L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 ENI Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_OPENING_TAG
//
// MessageText:
//
// ENI: Closing tag does not match opening tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_OPENING_TAG ((uint32_t)0xC0D40001L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_NUMBER_FIELD
//
// MessageText:
//
// ENI: Unexpected opening tag in number field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_NUMBER_FIELD ((uint32_t)0xC0D40002L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_NUMBER_FIELD
//
// MessageText:
//
// ENI: Unexpected single tag in number field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_NUMBER_FIELD ((uint32_t)0xC0D40003L)

//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_NUMBER_FIELD_TAG
//
// MessageText:
//
// ENI: Closing tag does not match number field tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_NUMBER_FIELD_TAG ((uint32_t)0xC0D40004L)

//
// MessageId: ERR_ECMV4_ENI_NUMBER_FIELD_IS_INVALID
//
// MessageText:
//
// ENI: Number field is invalid.
//
#define ERR_ECMV4_ENI_NUMBER_FIELD_IS_INVALID ((uint32_t)0xC0D40005L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_STRING_FIELD
//
// MessageText:
//
// ENI: Unexpected opening tag in string field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_STRING_FIELD ((uint32_t)0xC0D40006L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_STRING_FIELD
//
// MessageText:
//
// ENI: Unexpected single tag in string field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_STRING_FIELD ((uint32_t)0xC0D40007L)

//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_STRING_FIELD_TAG
//
// MessageText:
//
// ENI: Closing tag does not match string field tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_STRING_FIELD_TAG ((uint32_t)0xC0D40008L)

//
// MessageId: ERR_ECMV4_ENI_DATA_FIELD_IS_NOT_A_HEX_STRING
//
// MessageText:
//
// ENI: Data field is not a hex string.
//
#define ERR_ECMV4_ENI_DATA_FIELD_IS_NOT_A_HEX_STRING ((uint32_t)0xC0D40009L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_DATA_FIELD
//
// MessageText:
//
// ENI: Unexpected opening tag in data field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_DATA_FIELD ((uint32_t)0xC0D4000AL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_DATA_FIELD
//
// MessageText:
//
// ENI: Unexpected single tag in data field.
//
#define ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_DATA_FIELD ((uint32_t)0xC0D4000BL)

//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_DATA_FIELD_TAG
//
// MessageText:
//
// ENI: Closing tag does not match data field tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_DATA_FIELD_TAG ((uint32_t)0xC0D4000CL)

//
// MessageId: ERR_ECMV4_ENI_INTERNAL_ERROR
//
// MessageText:
//
// ENI: Internal Error.
//
#define ERR_ECMV4_ENI_INTERNAL_ERROR     ((uint32_t)0xC0D4000DL)

//
// MessageId: ERR_ECMV4_ENI_PREMATURE_END_OF_FILE
//
// MessageText:
//
// ENI: Premature End of File.
//
#define ERR_ECMV4_ENI_PREMATURE_END_OF_FILE ((uint32_t)0xC0D4000EL)

//
// MessageId: ERR_ECMV4_ENI_END_OF_FILE
//
// MessageText:
//
// ENI: End of file.
//
#define ERR_ECMV4_ENI_END_OF_FILE        ((uint32_t)0xC0D4000FL)

//
// MessageId: ERR_ECMV4_ENI_INVALID_XML
//
// MessageText:
//
// ENI: Invalid XML.
//
#define ERR_ECMV4_ENI_INVALID_XML        ((uint32_t)0xC0D40010L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_ECAT_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected single tag in InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_ECAT_INITCMD_BLOCK ((uint32_t)0xC0D40011L)

//
// MessageId: ERR_ECMV4_ENI_DUPLICATE_TAG_IN_ECAT_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Duplicate tag in InitCmd block.
//
#define ERR_ECMV4_ENI_DUPLICATE_TAG_IN_ECAT_INITCMD_BLOCK ((uint32_t)0xC0D40012L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_ECAT_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected opening tag in InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_ECAT_INITCMD_BLOCK ((uint32_t)0xC0D40013L)

//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_ECAT_INITCMD_TAG
//
// MessageText:
//
// ENI: Closing tag does not match InitCmd opening tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_ECAT_INITCMD_TAG ((uint32_t)0xC0D40014L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_TRANSITION_IN_ECAT_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid transition in InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_TRANSITION_IN_ECAT_INITCMD_BLOCK ((uint32_t)0xC0D40015L)

//
// MessageId: ERR_ECMV4_ENI_ECAT_INITCMD_IS_INCOMPLETE
//
// MessageText:
//
// ENI: InitCmd block is incomplete.
//
#define ERR_ECMV4_ENI_ECAT_INITCMD_IS_INCOMPLETE ((uint32_t)0xC0D40016L)

//
// MessageId: ERR_ECMV4_ENI_ECAT_INITCMD_VALIDATE_BLOCK_IS_INVALID
//
// MessageText:
//
// ENI: InitCmd validate block is invalid.
//
#define ERR_ECMV4_ENI_ECAT_INITCMD_VALIDATE_BLOCK_IS_INVALID ((uint32_t)0xC0D40017L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK
//
// MessageText:
//
// ENI: Unexpected single tag in InitCmd validate block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_SINGLE_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK ((uint32_t)0xC0D40018L)

//
// MessageId: ERR_ECMV4_ENI_DUPLICATE_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK
//
// MessageText:
//
// ENI: Duplicate tag in InitCmd validate block.
//
#define ERR_ECMV4_ENI_DUPLICATE_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK ((uint32_t)0xC0D40019L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK
//
// MessageText:
//
// ENI: Unexpected opening tag in InitCmd validate block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_OPENING_TAG_IN_ECAT_INITCMD_VALIDATE_BLOCK ((uint32_t)0xC0D4001AL)

//
// MessageId: ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_ECAT_INITCMD_VALIDATE_TAG
//
// MessageText:
//
// ENI: Closing tag does not match InitCmd validate opening tag.
//
#define ERR_ECMV4_ENI_CLOSING_TAG_DOES_NOT_MATCH_ECAT_INITCMD_VALIDATE_TAG ((uint32_t)0xC0D4001BL)

//
// MessageId: ERR_ECMV4_ENI_XML_FILE_IS_NOT_AN_ENI_XML
//
// MessageText:
//
// ENI: XML file is not an ENI file.
//
#define ERR_ECMV4_ENI_XML_FILE_IS_NOT_AN_ENI_XML ((uint32_t)0xC0D4001CL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_ETHERCATCONFIG_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in EtherCATConfig block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_ETHERCATCONFIG_BLOCK ((uint32_t)0xC0D4001DL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CONFIG_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Config block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CONFIG_BLOCK ((uint32_t)0xC0D4001EL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Master block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_BLOCK ((uint32_t)0xC0D4001FL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Master/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_INITCMDS_BLOCK ((uint32_t)0xC0D40020L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_BLOCK ((uint32_t)0xC0D40021L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INITCMDS_BLOCK ((uint32_t)0xC0D40022L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_BLOCK ((uint32_t)0xC0D40023L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_BLOCK ((uint32_t)0xC0D40024L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_DC_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/DC block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_DC_BLOCK ((uint32_t)0xC0D40025L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_HOTCONNECT_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/HotConnect block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_HOTCONNECT_BLOCK ((uint32_t)0xC0D40026L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_PREVIOUSPORT_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/PreviousPort block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_PREVIOUSPORT_BLOCK ((uint32_t)0xC0D40027L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Cyclic block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_BLOCK ((uint32_t)0xC0D40028L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Cyclic/Frame block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_BLOCK ((uint32_t)0xC0D40029L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Cyclic/Frame/Cmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_BLOCK ((uint32_t)0xC0D4002AL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/CoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_BLOCK ((uint32_t)0xC0D4002BL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/CoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_INITCMDS_BLOCK ((uint32_t)0xC0D4002CL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/CoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_COE_INITCMD_BLOCK ((uint32_t)0xC0D4002DL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/SoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_BLOCK ((uint32_t)0xC0D4002EL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/SoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_INITCMDS_BLOCK ((uint32_t)0xC0D4002FL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/SoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SOE_INITCMD_BLOCK ((uint32_t)0xC0D40030L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/EoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_BLOCK ((uint32_t)0xC0D40031L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/EoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_INITCMDS_BLOCK ((uint32_t)0xC0D40032L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/EoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_EOE_INITCMD_BLOCK ((uint32_t)0xC0D40033L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/AoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_BLOCK ((uint32_t)0xC0D40034L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/AoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_INITCMDS_BLOCK ((uint32_t)0xC0D40035L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/AoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_AOE_INITCMD_BLOCK ((uint32_t)0xC0D40036L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/FoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_BLOCK ((uint32_t)0xC0D40037L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/FoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_INITCMDS_BLOCK ((uint32_t)0xC0D40038L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/FoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_FOE_INITCMD_BLOCK ((uint32_t)0xC0D40039L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/VoE block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_BLOCK ((uint32_t)0xC0D4003AL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_INITCMDS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/VoE/InitCmds block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_INITCMDS_BLOCK ((uint32_t)0xC0D4003BL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/VoE/InitCmds/InitCmd block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_VOE_INITCMD_BLOCK ((uint32_t)0xC0D4003CL)

//
// MessageId: ERR_ECMV4_ENI_LOADING_NOT_ENABLED
//
// MessageText:
//
// ENI: Loading not enabled.
//
#define ERR_ECMV4_ENI_LOADING_NOT_ENABLED ((uint32_t)0xC0D4003DL)

//
// MessageId: ERR_ECMV4_ENI_COULD_NOT_OPEN_FILE
//
// MessageText:
//
// ENI: Could not open file.
//
#define ERR_ECMV4_ENI_COULD_NOT_OPEN_FILE ((uint32_t)0xC0D4003EL)

//
// MessageId: ERR_ECMV4_ENI_BASE_CYCLE_TIME_TOO_SMALL
//
// MessageText:
//
// ENI: Base cycle time too small.
//
#define ERR_ECMV4_ENI_BASE_CYCLE_TIME_TOO_SMALL ((uint32_t)0xC0D4003FL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info/PreviousPort block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK ((uint32_t)0xC0D40040L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_PORT_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK
//
// MessageText:
//
// ENI: Invalid port in Slave/Info/PreviousPort block.
//
#define ERR_ECMV4_ENI_INVALID_PORT_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK ((uint32_t)0xC0D40041L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_PHYSADDR_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK
//
// MessageText:
//
// ENI: Invalid PhysAddr in Slave/Info/PreviousPort block.
//
#define ERR_ECMV4_ENI_INVALID_PHYSADDR_IN_SLAVE_INFO_PREVIOUSPORT_BLOCK ((uint32_t)0xC0D40042L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_TRANSITION_IN_COE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid transition in CoE/InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_TRANSITION_IN_COE_INITCMD_BLOCK ((uint32_t)0xC0D40043L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_COE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing transitions in CoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_COE_INITCMD_BLOCK ((uint32_t)0xC0D40044L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_CCS_IN_COE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid Ccs in CoE/InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_CCS_IN_COE_INITCMD_BLOCK ((uint32_t)0xC0D40045L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SEND_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/Send block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_SEND_BLOCK ((uint32_t)0xC0D40046L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_RECV_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/Recv block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_RECV_BLOCK ((uint32_t)0xC0D40047L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_BOOTSTRAP_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Mailbox/Bootstrap block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_MAILBOX_BOOTSTRAP_BLOCK ((uint32_t)0xC0D40048L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_EOE_INITCMD
//
// MessageText:
//
// ENI: Invalid EoE InitCmd.
//
#define ERR_ECMV4_ENI_INVALID_EOE_INITCMD ((uint32_t)0xC0D40049L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_MAILBOXSTATES_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Master/MailboxStates block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_MASTER_MAILBOXSTATES_BLOCK ((uint32_t)0xC0D4004AL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PROCESSDATA_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info/ProcessData block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PROCESSDATA_BLOCK ((uint32_t)0xC0D4004BL)

//
// MessageId: ERR_ECMV4_ENI_INVALID_MBOX_STATE_BIT_NO_IN_SLAVE_INFO_PROCESSDATA_RECV_BLOCK
//
// MessageText:
//
// ENI: Invalid MBoxState Bit number in Slave/Info/ProcessData/Recv block.
//
#define ERR_ECMV4_ENI_INVALID_MBOX_STATE_BIT_NO_IN_SLAVE_INFO_PROCESSDATA_RECV_BLOCK ((uint32_t)0xC0D4004CL)

//
// MessageId: ERR_ECMV4_ENI_PROCESS_DATA_CONFIG_OFFSET_NOT_POSSIBLE
//
// MessageText:
//
// ENI: Process Data config offset not possible.
//
#define ERR_ECMV4_ENI_PROCESS_DATA_CONFIG_OFFSET_NOT_POSSIBLE ((uint32_t)0xC0D4004DL)

//
// MessageId: ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_SOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing transitions in SoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_SOE_INITCMD_BLOCK ((uint32_t)0xC0D4004EL)

//
// MessageId: ERR_ECMV4_ENI_INVALID_OPCODE_IN_SOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid OpCode in SoE/InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_OPCODE_IN_SOE_INITCMD_BLOCK ((uint32_t)0xC0D4004FL)

//
// MessageId: ERR_ECMV4_ENI_UNSUPPORTED_ECAT_CMD_IN_IDENTIFYCMD_BLOCK
//
// MessageText:
//
// ENI: Unsupported EtherCAT command in IdentifyCmd block.
//
#define ERR_ECMV4_ENI_UNSUPPORTED_ECAT_CMD_IN_IDENTIFYCMD_BLOCK ((uint32_t)0xC0D40050L)

//
// MessageId: ERR_ECMV4_ENI_UNSUPPORTED_ECAT_REG_IN_IDENTIFYCMD_BLOCK
//
// MessageText:
//
// ENI: Unsupported EtherCAT register in IdentifyCmd block.
//
#define ERR_ECMV4_ENI_UNSUPPORTED_ECAT_REG_IN_IDENTIFYCMD_BLOCK ((uint32_t)0xC0D40051L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_IDENTIFYCMD_DATA_FOR_HOT_CONNECT_SLAVE
//
// MessageText:
//
// ENI: Missing IdentifyCmd data for HotConnect slave.
//
#define ERR_ECMV4_ENI_MISSING_IDENTIFYCMD_DATA_FOR_HOT_CONNECT_SLAVE ((uint32_t)0xC0D40052L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_IDENTIFYCMD_DATA_FOR_HOT_CONNECT_SLAVE
//
// MessageText:
//
// ENI: Invalid IdentifyCmd data for HotConnect slave.
//
#define ERR_ECMV4_ENI_INVALID_IDENTIFYCMD_DATA_FOR_HOT_CONNECT_SLAVE ((uint32_t)0xC0D40053L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_SM_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info/Sm block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_SM_BLOCK ((uint32_t)0xC0D40054L)

//
// MessageId: ERR_ECMV4_ENI_TOO_MANY_PDOS_LISTED_IN_SLAVE_INFO_SM_BLOCK
//
// MessageText:
//
// ENI: Too many PDOs listed in Slave/Info/Sm block.
//
#define ERR_ECMV4_ENI_TOO_MANY_PDOS_LISTED_IN_SLAVE_INFO_SM_BLOCK ((uint32_t)0xC0D40055L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PDO_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info/Pdo block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PDO_BLOCK ((uint32_t)0xC0D40056L)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PDO_ENTRY_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Slave/Info/Pdo/Entry block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_SLAVE_INFO_PDO_ENTRY_BLOCK ((uint32_t)0xC0D40057L)

//
// MessageId: ERR_ECMV4_ENI_TOO_MANY_PDO_ENTRIES_LISTED_IN_SLAVE_INFO_PDO_BLOCK
//
// MessageText:
//
// ENI: Too many PDO entries listed in Slave/Info/PDO block.
//
#define ERR_ECMV4_ENI_TOO_MANY_PDO_ENTRIES_LISTED_IN_SLAVE_INFO_PDO_BLOCK ((uint32_t)0xC0D40058L)

//
// MessageId: ERR_ECMV4_ENI_INDEX_OF_PDO_ENTRY_INVALID
//
// MessageText:
//
// ENI: Index of PDO entry invalid.
//
#define ERR_ECMV4_ENI_INDEX_OF_PDO_ENTRY_INVALID ((uint32_t)0xC0D40059L)

//
// MessageId: ERR_ECMV4_ENI_SUBINDEX_OF_PDO_ENTRY_INVALID
//
// MessageText:
//
// ENI: Subindex of PDO entry invalid.
//
#define ERR_ECMV4_ENI_SUBINDEX_OF_PDO_ENTRY_INVALID ((uint32_t)0xC0D4005AL)

//
// MessageId: ERR_ECMV4_ENI_BIT_LENGTH_OF_PDO_ENTRY_EXCEEDS_MAXIMUM_BIT_LENGTH
//
// MessageText:
//
// ENI: Bit length of PDO entry exceeds maximum bit length.
//
#define ERR_ECMV4_ENI_BIT_LENGTH_OF_PDO_ENTRY_EXCEEDS_MAXIMUM_BIT_LENGTH ((uint32_t)0xC0D4005BL)

//
// MessageId: ERR_ECMV4_ENI_NO_SLAVES_IN_ENI
//
// MessageText:
//
// ENI: No slaves in ENI.
//
#define ERR_ECMV4_ENI_NO_SLAVES_IN_ENI   ((uint32_t)0xC0D4005CL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Cyclic/Frame/Cmd/CopyInfos block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_BLOCK ((uint32_t)0xC0D4005DL)

//
// MessageId: ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_COPYINFO_BLOCK
//
// MessageText:
//
// ENI: Unexpected closing tag in Cyclic/Frame/Cmd/CopyInfos/CopyInfo block.
//
#define ERR_ECMV4_ENI_UNEXPECTED_CLOSING_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_COPYINFO_BLOCK ((uint32_t)0xC0D4005EL)

//
// MessageId: ERR_ECMV4_ENI_DUPLICATE_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_COPYINFO_BLOCK
//
// MessageText:
//
// ENI: Duplicate tag in Cyclic/Frame/Cmd/CopyInfos/CopyInfo block.
//
#define ERR_ECMV4_ENI_DUPLICATE_TAG_IN_CYCLIC_FRAME_CMD_COPYINFOS_COPYINFO_BLOCK ((uint32_t)0xC0D4005FL)

//
// MessageId: ERR_ECMV4_ENI_INVALID_COPYINFO_BLOCK
//
// MessageText:
//
// ENI: Invalid CopyInfo block.
//
#define ERR_ECMV4_ENI_INVALID_COPYINFO_BLOCK ((uint32_t)0xC0D40060L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_ATTRIBUTE_IN_ECAT_INITCMD_VALIDATE_BLOCK
//
// MessageText:
//
// ENI: Invalid XML attribute in InitCmd/Validate block.
//
#define ERR_ECMV4_ENI_INVALID_ATTRIBUTE_IN_ECAT_INITCMD_VALIDATE_BLOCK ((uint32_t)0xC0D40061L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_TRANSITION_IN_VOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid transition in VoE/InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_TRANSITION_IN_VOE_INITCMD_BLOCK ((uint32_t)0xC0D40062L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_VOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing transitions in VoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_VOE_INITCMD_BLOCK ((uint32_t)0xC0D40063L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_DATA_IN_VOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing data in VoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_DATA_IN_VOE_INITCMD_BLOCK ((uint32_t)0xC0D40064L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_NETID_IN_AOE_BLOCK
//
// MessageText:
//
// ENI: Invalid Net ID in AoE block.
//
#define ERR_ECMV4_ENI_INVALID_NETID_IN_AOE_BLOCK ((uint32_t)0xC0D40065L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_TRANSITION_IN_AOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Invalid transition in AoE/InitCmd block.
//
#define ERR_ECMV4_ENI_INVALID_TRANSITION_IN_AOE_INITCMD_BLOCK ((uint32_t)0xC0D40066L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_AOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing transition in AoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_TRANSITIONS_IN_AOE_INITCMD_BLOCK ((uint32_t)0xC0D40067L)

//
// MessageId: ERR_ECMV4_ENI_MISSING_DATA_IN_AOE_INITCMD_BLOCK
//
// MessageText:
//
// ENI: Missing data in AoE/InitCmd block.
//
#define ERR_ECMV4_ENI_MISSING_DATA_IN_AOE_INITCMD_BLOCK ((uint32_t)0xC0D40068L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_BEFORE_SLAVE_VALUE
//
// MessageText:
//
// ENI: Invalid BeforeSlave value.
//
#define ERR_ECMV4_ENI_INVALID_BEFORE_SLAVE_VALUE ((uint32_t)0xC0D40069L)

//
// MessageId: ERR_ECMV4_ENI_INVALID_COMPLETE_ACCESS_ATTRIBUTE
//
// MessageText:
//
// ENI: Invalid CompleteAccess XML attribute.
//
#define ERR_ECMV4_ENI_INVALID_COMPLETE_ACCESS_ATTRIBUTE ((uint32_t)0xC0D4006AL)

//
// MessageId: ERR_ECMV4_ENI_ECAT_INITCMD_MISSING_TRANSITIONS
//
// MessageText:
//
// ENI: Ecat InitCmd misses transitions.
//
#define ERR_ECMV4_ENI_ECAT_INITCMD_MISSING_TRANSITIONS ((uint32_t)0xC0D4006BL)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 ALSTATUSCODE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_ALSTATUSCODE_UNSPECIFIED_ERROR
//
// MessageText:
//
// AlStatusCode: Unspecified Error.
//
#define ERR_ECMV4_ALSTATUSCODE_UNSPECIFIED_ERROR ((uint32_t)0xC0D50001L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_MEMORY
//
// MessageText:
//
// AlStatusCode: No Memory.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_MEMORY ((uint32_t)0xC0D50002L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_DEVICE_SETUP
//
// MessageText:
//
// AlStatusCode: Invalid device setup.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_DEVICE_SETUP ((uint32_t)0xC0D50003L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_REQUESTED_STATE_CHANGE
//
// MessageText:
//
// AlStatusCode: Invalid requested state change.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_REQUESTED_STATE_CHANGE ((uint32_t)0xC0D50011L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_UNKNOWN_REQUESTED_STATE
//
// MessageText:
//
// AlStatusCode: Unknown requested state.
//
#define ERR_ECMV4_ALSTATUSCODE_UNKNOWN_REQUESTED_STATE ((uint32_t)0xC0D50012L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_BOOTSTRAP_NOT_SUPPORTED
//
// MessageText:
//
// AlStatusCode: Bootstrap not supported.
//
#define ERR_ECMV4_ALSTATUSCODE_BOOTSTRAP_NOT_SUPPORTED ((uint32_t)0xC0D50013L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_VALID_FIRMWARE
//
// MessageText:
//
// AlStatusCode: No valid firmware.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_VALID_FIRMWARE ((uint32_t)0xC0D50014L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_BOOT_MAILBOX_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid BOOT mailbox configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_BOOT_MAILBOX_CONFIGURATION ((uint32_t)0xC0D50015L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_PREOP_MAILBOX_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid PREOP mailbox configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_PREOP_MAILBOX_CONFIGURATION ((uint32_t)0xC0D50016L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_SYNC_MANAGER_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid Sync Manager configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_SYNC_MANAGER_CONFIGURATION ((uint32_t)0xC0D50017L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_VALID_INPUTS_AVAILABLE
//
// MessageText:
//
// AlStatusCode: No valid inputs available.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_VALID_INPUTS_AVAILABLE ((uint32_t)0xC0D50018L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_VALID_OUTPUTS
//
// MessageText:
//
// AlStatusCode: No valid outputs.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_VALID_OUTPUTS ((uint32_t)0xC0D50019L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SYNCHRONIZATION_ERROR
//
// MessageText:
//
// AlStatusCode: Synchronization Error.
//
#define ERR_ECMV4_ALSTATUSCODE_SYNCHRONIZATION_ERROR ((uint32_t)0xC0D5001AL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SYNC_MANAGER_WATCHDOG
//
// MessageText:
//
// AlStatusCode: Sync Manager watchdog.
//
#define ERR_ECMV4_ALSTATUSCODE_SYNC_MANAGER_WATCHDOG ((uint32_t)0xC0D5001BL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_SYNC_MANAGER_TYPES
//
// MessageText:
//
// AlStatusCode: Invalid Sync Manager types.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_SYNC_MANAGER_TYPES ((uint32_t)0xC0D5001CL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_OUTPUT_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid output configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_OUTPUT_CONFIGURATION ((uint32_t)0xC0D5001DL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_INPUT_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid input configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_INPUT_CONFIGURATION ((uint32_t)0xC0D5001EL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_WATCHDOG_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid watchdog configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_WATCHDOG_CONFIGURATION ((uint32_t)0xC0D5001FL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_COLD_START
//
// MessageText:
//
// AlStatusCode: Slave needs cold start.
//
#define ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_COLD_START ((uint32_t)0xC0D50020L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_INIT
//
// MessageText:
//
// AlStatusCode: Slave needs INIT.
//
#define ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_INIT ((uint32_t)0xC0D50021L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_PREOP
//
// MessageText:
//
// AlStatusCode: Slave needs PREOP.
//
#define ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_PREOP ((uint32_t)0xC0D50022L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_SAFEOP
//
// MessageText:
//
// AlStatusCode: Slave needs SAFEOP.
//
#define ERR_ECMV4_ALSTATUSCODE_SLAVE_NEEDS_SAFEOP ((uint32_t)0xC0D50023L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_INPUT_MAPPING
//
// MessageText:
//
// AlStatusCode: Invalid input mapping.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_INPUT_MAPPING ((uint32_t)0xC0D50024L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_OUTPUT_MAPPING
//
// MessageText:
//
// AlStatusCode: Invalid output mapping.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_OUTPUT_MAPPING ((uint32_t)0xC0D50025L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INCONSISTENT_SETTINGS
//
// MessageText:
//
// AlStatusCode: Inconsistent settings.
//
#define ERR_ECMV4_ALSTATUSCODE_INCONSISTENT_SETTINGS ((uint32_t)0xC0D50026L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_FREERUN_NOT_SUPPORTED
//
// MessageText:
//
// AlStatusCode: Free Run not supported.
//
#define ERR_ECMV4_ALSTATUSCODE_FREERUN_NOT_SUPPORTED ((uint32_t)0xC0D50027L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SYNCMODE_NOT_SUPPORTED
//
// MessageText:
//
// AlStatusCode: SyncMode not supported.
//
#define ERR_ECMV4_ALSTATUSCODE_SYNCMODE_NOT_SUPPORTED ((uint32_t)0xC0D50028L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_FREERUN_NEEDS_3BUFFER_MODE
//
// MessageText:
//
// AlStatusCode: Free Run needs 3 Buffer Mode.
//
#define ERR_ECMV4_ALSTATUSCODE_FREERUN_NEEDS_3BUFFER_MODE ((uint32_t)0xC0D50029L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_BACKGROUND_WATCHDOG
//
// MessageText:
//
// AlStatusCode: Background Watchdog.
//
#define ERR_ECMV4_ALSTATUSCODE_BACKGROUND_WATCHDOG ((uint32_t)0xC0D5002AL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS
//
// MessageText:
//
// AlStatusCode: No valid inputs and outputs.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS ((uint32_t)0xC0D5002BL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_FATAL_SYNC_ERROR
//
// MessageText:
//
// AlStatusCode: Fatal Sync Error.
//
#define ERR_ECMV4_ALSTATUSCODE_FATAL_SYNC_ERROR ((uint32_t)0xC0D5002CL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_NO_SYNC_ERROR
//
// MessageText:
//
// AlStatusCode: No Sync Error.
//
#define ERR_ECMV4_ALSTATUSCODE_NO_SYNC_ERROR ((uint32_t)0xC0D5002DL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_DC_SYNC_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid DC Sync configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_DC_SYNC_CONFIGURATION ((uint32_t)0xC0D50030L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_INVALID_DC_LATCH_CONFIGURATION
//
// MessageText:
//
// AlStatusCode: Invalid DC Latch configuration.
//
#define ERR_ECMV4_ALSTATUSCODE_INVALID_DC_LATCH_CONFIGURATION ((uint32_t)0xC0D50031L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_PLL_ERROR
//
// MessageText:
//
// AlStatusCode: PLL Error.
//
#define ERR_ECMV4_ALSTATUSCODE_PLL_ERROR ((uint32_t)0xC0D50032L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DC_SYNC_IO_ERROR
//
// MessageText:
//
// AlStatusCode: DC Sync I/O Error.
//
#define ERR_ECMV4_ALSTATUSCODE_DC_SYNC_IO_ERROR ((uint32_t)0xC0D50033L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DC_SYNC_TIMEOUT_ERROR
//
// MessageText:
//
// AlStatusCode: DC Sync Timeout Error.
//
#define ERR_ECMV4_ALSTATUSCODE_DC_SYNC_TIMEOUT_ERROR ((uint32_t)0xC0D50034L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DC_INVALID_SYNC_CYCLE_TIME
//
// MessageText:
//
// AlStatusCode: Invalid DC Sync cycle time.
//
#define ERR_ECMV4_ALSTATUSCODE_DC_INVALID_SYNC_CYCLE_TIME ((uint32_t)0xC0D50035L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DC_SYNC0_CYCLE_TIME
//
// MessageText:
//
// AlStatusCode: DC Sync0 Cycle Time.
//
#define ERR_ECMV4_ALSTATUSCODE_DC_SYNC0_CYCLE_TIME ((uint32_t)0xC0D50036L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DC_SYNC1_CYCLE_TIME
//
// MessageText:
//
// AlStatusCode: DC Sync0 Cycle Time.
//
#define ERR_ECMV4_ALSTATUSCODE_DC_SYNC1_CYCLE_TIME ((uint32_t)0xC0D50037L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_AOE
//
// MessageText:
//
// AlStatusCode: Mbx AoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_AOE   ((uint32_t)0xC0D50041L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_EOE
//
// MessageText:
//
// AlStatusCode: Mbx EoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_EOE   ((uint32_t)0xC0D50042L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_COE
//
// MessageText:
//
// AlStatusCode: Mbx CoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_COE   ((uint32_t)0xC0D50043L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_FOE
//
// MessageText:
//
// AlStatusCode: Mbx FoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_FOE   ((uint32_t)0xC0D50044L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_SOE
//
// MessageText:
//
// AlStatusCode: Mbx SoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_SOE   ((uint32_t)0xC0D50045L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_MBX_VOE
//
// MessageText:
//
// AlStatusCode: Mbx VoE.
//
#define ERR_ECMV4_ALSTATUSCODE_MBX_VOE   ((uint32_t)0xC0D5004FL)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_EEPROM_NO_ACCESS
//
// MessageText:
//
// AlStatusCode: EEPROM: No PDI Access.
//
#define ERR_ECMV4_ALSTATUSCODE_EEPROM_NO_ACCESS ((uint32_t)0xC0D50050L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_EEPROM_ERROR
//
// MessageText:
//
// AlStatusCode: EEPROM: Error.
//
#define ERR_ECMV4_ALSTATUSCODE_EEPROM_ERROR ((uint32_t)0xC0D50051L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_SLAVE_RESTARTED_LOCALLY
//
// MessageText:
//
// AlStatusCode: Slave restarted locally.
//
#define ERR_ECMV4_ALSTATUSCODE_SLAVE_RESTARTED_LOCALLY ((uint32_t)0xC0D50060L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_DEVICE_IDENTIFICATION_VALUE_UPDATED
//
// MessageText:
//
// AlStatusCode: Device identification value updated.
//
#define ERR_ECMV4_ALSTATUSCODE_DEVICE_IDENTIFICATION_VALUE_UPDATED ((uint32_t)0xC0D50061L)

//
// MessageId: ERR_ECMV4_ALSTATUSCODE_APPLICATION_CONTROLLER_AVAILABLE
//
// MessageText:
//
// AlStatusCode: Application controller available.
//
#define ERR_ECMV4_ALSTATUSCODE_APPLICATION_CONTROLLER_AVAILABLE ((uint32_t)0xC0D500F0L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 IF Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_IF_COE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// CoE Support not available.
//
#define ERR_ECMV4_IF_COE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0D60001L)

//
// MessageId: ERR_ECMV4_IF_SOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// SoE Support not available.
//
#define ERR_ECMV4_IF_SOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0D60002L)

//
// MessageId: ERR_ECMV4_IF_FOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// FoE Support not available.
//
#define ERR_ECMV4_IF_FOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0D60003L)

//
// MessageId: ERR_ECMV4_IF_AOE_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// AoE Support not available.
//
#define ERR_ECMV4_IF_AOE_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0D60004L)

//
// MessageId: ERR_ECMV4_IF_INVALID_TRANSPORT_TYPE
//
// MessageText:
//
// Invalid transport type.
//
#define ERR_ECMV4_IF_INVALID_TRANSPORT_TYPE ((uint32_t)0xC0D60005L)

//
// MessageId: ERR_ECMV4_IF_SOE_INVALID_DRIVE_NO
//
// MessageText:
//
// SoE: Invalid drive number.
//
#define ERR_ECMV4_IF_SOE_INVALID_DRIVE_NO ((uint32_t)0xC0D60006L)

//
// MessageId: ERR_ECMV4_IF_SOE_INVALID_ELEMENT_FLAGS
//
// MessageText:
//
// Invalid element flags.
//
#define ERR_ECMV4_IF_SOE_INVALID_ELEMENT_FLAGS ((uint32_t)0xC0D60007L)

//
// MessageId: ERR_ECMV4_IF_INVALID_SOE_TRANSFER_ID
//
// MessageText:
//
// Invalid SoE Transfer Id.
//
#define ERR_ECMV4_IF_INVALID_SOE_TRANSFER_ID ((uint32_t)0xC0D60008L)

//
// MessageId: ERR_ECMV4_IF_TRANSFER_ABORTED
//
// MessageText:
//
// Transfer aborted.
//
#define ERR_ECMV4_IF_TRANSFER_ABORTED    ((uint32_t)0xC0D60009L)

//
// MessageId: ERR_ECMV4_IF_OUT_OF_PACKETS
//
// MessageText:
//
// Out of packets.
//
#define ERR_ECMV4_IF_OUT_OF_PACKETS      ((uint32_t)0xC0D6000AL)

//
// MessageId: ERR_ECMV4_IF_OUT_OF_TRANSFER_CONTEXTS
//
// MessageText:
//
// Out of transfer contexts.
//
#define ERR_ECMV4_IF_OUT_OF_TRANSFER_CONTEXTS ((uint32_t)0xC0D6000BL)

//
// MessageId: ERR_ECMV4_IF_INVALID_SUBINDEX_FOR_COMPLETE_ACCESS
//
// MessageText:
//
// Invalid Subindex for Complete Access.
//
#define ERR_ECMV4_IF_INVALID_SUBINDEX_FOR_COMPLETE_ACCESS ((uint32_t)0xC0D6000CL)

//
// MessageId: ERR_ECMV4_IF_INVALID_COE_TRANSFER_ID
//
// MessageText:
//
// Invalid CoE Transfer Id.
//
#define ERR_ECMV4_IF_INVALID_COE_TRANSFER_ID ((uint32_t)0xC0D6000DL)

//
// MessageId: ERR_ECMV4_IF_INVALID_COE_SDOINFO_LISTTYPE
//
// MessageText:
//
// Invalid CoE SDOINFO ListType.
//
#define ERR_ECMV4_IF_INVALID_COE_SDOINFO_LISTTYPE ((uint32_t)0xC0D6000EL)

//
// MessageId: ERR_ECMV4_IF_FILE_READ_ERROR
//
// MessageText:
//
// File Read Error.
//
#define ERR_ECMV4_IF_FILE_READ_ERROR     ((uint32_t)0xC0D6000FL)

//
// MessageId: ERR_ECMV4_IF_COULD_NOT_OPEN_FILE
//
// MessageText:
//
// Could not open file.
//
#define ERR_ECMV4_IF_COULD_NOT_OPEN_FILE ((uint32_t)0xC0D60010L)

//
// MessageId: ERR_ECMV4_IF_INVALID_CONFIG_NXD
//
// MessageText:
//
// Invalid config.nxd.
//
#define ERR_ECMV4_IF_INVALID_CONFIG_NXD  ((uint32_t)0xC0D60011L)

//
// MessageId: ERR_ECMV4_IF_CONFIG_NXD_WITHOUT_SLAVES
//
// MessageText:
//
// Config.nxd without slaves.
//
#define ERR_ECMV4_IF_CONFIG_NXD_WITHOUT_SLAVES ((uint32_t)0xC0D60012L)

//
// MessageId: ERR_ECMV4_IF_INVALID_FILE_NAME
//
// MessageText:
//
// Invalid filename.
//
#define ERR_ECMV4_IF_INVALID_FILE_NAME   ((uint32_t)0xC0D60013L)

//
// MessageId: ERR_ECMV4_IF_INVALID_FOE_TRANSFER_ID
//
// MessageText:
//
// Invalid FoE Transfer Id.
//
#define ERR_ECMV4_IF_INVALID_FOE_TRANSFER_ID ((uint32_t)0xC0D60014L)

//
// MessageId: ERR_ECMV4_IF_INVALID_GET_TOPOLOGY_TRANSFER_ID
//
// MessageText:
//
// Invalid Get Topology Transfer Id.
//
#define ERR_ECMV4_IF_INVALID_GET_TOPOLOGY_TRANSFER_ID ((uint32_t)0xC0D60015L)

//
// MessageId: ERR_ECMV4_IF_INVALID_AOE_TRANSFER_ID
//
// MessageText:
//
// Invalid AoE Transfer Id.
//
#define ERR_ECMV4_IF_INVALID_AOE_TRANSFER_ID ((uint32_t)0xC0D60016L)

//
// MessageId: ERR_ECMV4_IF_CONFIG_ACFG_SUPPORT_NOT_AVAILABLE
//
// MessageText:
//
// AutoCfg support not available.
//
#define ERR_ECMV4_IF_CONFIG_ACFG_SUPPORT_NOT_AVAILABLE ((uint32_t)0xC0D60017L)

/////////////////////////////////////////////////////////////////////////////////////
// ECMv4 AoE Error codes
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ECMV4_AOE_TARGET_PORT_NOT_FOUND
//
// MessageText:
//
// AoE: Target Port not found.
//
#define ERR_ECMV4_AOE_TARGET_PORT_NOT_FOUND ((uint32_t)0xC0CE0006L)

//
// MessageId: ERR_ECMV4_AOE_TARGET_MACHINE_NOT_FOUND
//
// MessageText:
//
// AoE: Target Machine not found.
//
#define ERR_ECMV4_AOE_TARGET_MACHINE_NOT_FOUND ((uint32_t)0xC0CE0007L)

//
// MessageId: ERR_ECMV4_AOE_SERVICE_NOT_SUPPORTED
//
// MessageText:
//
// AoE: Service not supported.
//
#define ERR_ECMV4_AOE_SERVICE_NOT_SUPPORTED ((uint32_t)0xC0CE0701L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_INDEX_GROUP
//
// MessageText:
//
// AoE: Invalid IndexGroup.
//
#define ERR_ECMV4_AOE_INVALID_INDEX_GROUP ((uint32_t)0xC0CE0702L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_INDEX_OFFSET
//
// MessageText:
//
// AoE: Invalid IndexOffset.
//
#define ERR_ECMV4_AOE_INVALID_INDEX_OFFSET ((uint32_t)0xC0CE0703L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_ACCESS
//
// MessageText:
//
// AoE: Invalid access.
//
#define ERR_ECMV4_AOE_INVALID_ACCESS     ((uint32_t)0xC0CE0704L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_SIZE
//
// MessageText:
//
// AoE: Invalid size.
//
#define ERR_ECMV4_AOE_INVALID_SIZE       ((uint32_t)0xC0CE0705L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_DATA
//
// MessageText:
//
// AoE: Invalid data.
//
#define ERR_ECMV4_AOE_INVALID_DATA       ((uint32_t)0xC0CE0706L)

//
// MessageId: ERR_ECMV4_AOE_NOTREADY
//
// MessageText:
//
// AoE: Not ready.
//
#define ERR_ECMV4_AOE_NOTREADY           ((uint32_t)0xC0CE0707L)

//
// MessageId: ERR_ECMV4_AOE_BUSY
//
// MessageText:
//
// AoE: Busy.
//
#define ERR_ECMV4_AOE_BUSY               ((uint32_t)0xC0CE0708L)

//
// MessageId: ERR_ECMV4_AOE_NOT_FOUND
//
// MessageText:
//
// AoE: Not Found.
//
#define ERR_ECMV4_AOE_NOT_FOUND          ((uint32_t)0xC0CE070CL)

//
// MessageId: ERR_ECMV4_AOE_INCOMPATIBLE
//
// MessageText:
//
// AoE: Incompatible.
//
#define ERR_ECMV4_AOE_INCOMPATIBLE       ((uint32_t)0xC0CE070EL)

//
// MessageId: ERR_ECMV4_AOE_WRONG_SLAVE_SATTE
//
// MessageText:
//
// AoE: Wrong slave state.
//
#define ERR_ECMV4_AOE_WRONG_SLAVE_SATTE  ((uint32_t)0xC0CE0712L)

//
// MessageId: ERR_ECMV4_AOE_NOTINIT
//
// MessageText:
//
// AoE: Not initialized.
//
#define ERR_ECMV4_AOE_NOTINIT            ((uint32_t)0xC0CE0718L)

//
// MessageId: ERR_ECMV4_AOE_DEVICE_TIMEOUT
//
// MessageText:
//
// AoE: Device Timeout.
//
#define ERR_ECMV4_AOE_DEVICE_TIMEOUT     ((uint32_t)0xC0CE0719L)

//
// MessageId: ERR_ECMV4_AOE_SHUTTING_DOWN
//
// MessageText:
//
// AoE: Shutting down.
//
#define ERR_ECMV4_AOE_SHUTTING_DOWN      ((uint32_t)0xC0CEF001L)

//
// MessageId: ERR_ECMV4_AOE_INITIALIZATION_ERROR
//
// MessageText:
//
// AoE: Initialization Error.
//
#define ERR_ECMV4_AOE_INITIALIZATION_ERROR ((uint32_t)0xC0CEF002L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_TRANSFER_HANDLE
//
// MessageText:
//
// AoE: Invalid transfer handle.
//
#define ERR_ECMV4_AOE_INVALID_TRANSFER_HANDLE ((uint32_t)0xC0CEF003L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_TRANSFER_STATE
//
// MessageText:
//
// AoE: Invalid transfer state.
//
#define ERR_ECMV4_AOE_INVALID_TRANSFER_STATE ((uint32_t)0xC0CEF004L)

//
// MessageId: ERR_ECMV4_AOE_PROTOCOL_TIMEOUT
//
// MessageText:
//
// AoE: Protocol Timeout.
//
#define ERR_ECMV4_AOE_PROTOCOL_TIMEOUT   ((uint32_t)0xC0CEF005L)

//
// MessageId: ERR_ECMV4_AOE_TRANSFER_SEGMENT_TOO_LONG
//
// MessageText:
//
// AoE: Transfer Segment too long.
//
#define ERR_ECMV4_AOE_TRANSFER_SEGMENT_TOO_LONG ((uint32_t)0xC0CEF006L)

//
// MessageId: ERR_ECMV4_AOE_NO_MAILBOX_AVAILABLE
//
// MessageText:
//
// AoE: No mailbox available.
//
#define ERR_ECMV4_AOE_NO_MAILBOX_AVAILABLE ((uint32_t)0xC0CEF007L)

//
// MessageId: ERR_ECMV4_AOE_RECONFIGURATION_IN_PROGRESS
//
// MessageText:
//
// AoE: Reconfiguration in progress.
//
#define ERR_ECMV4_AOE_RECONFIGURATION_IN_PROGRESS ((uint32_t)0xC0CEF008L)

//
// MessageId: ERR_ECMV4_AOE_INVALID_SLAVE_STATION_ADDRESS
//
// MessageText:
//
// AoE: Invalid slave station address.
//
#define ERR_ECMV4_AOE_INVALID_SLAVE_STATION_ADDRESS ((uint32_t)0xC0CEF009L)

//
// MessageId: ERR_ECMV4_AOE_TRANSFER_ABORTED
//
// MessageText:
//
// AoE: Transfer aborted.
//
#define ERR_ECMV4_AOE_TRANSFER_ABORTED   ((uint32_t)0xC0CEF00AL)

//
// MessageId: ERR_ECMV4_AOE_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// AoE: Request destination problem.
//
#define ERR_ECMV4_AOE_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0CEF00BL)

//
// MessageId: ERR_ECMV4_AOE_DUPLICATE_NETID
//
// MessageText:
//
// AoE: Duplicate Net ID.
//
#define ERR_ECMV4_AOE_DUPLICATE_NETID    ((uint32_t)0xC0CEF00CL)

//
// MessageId: ERR_ECMV4_AOE_INVALID_NETID_HANDLE
//
// MessageText:
//
// AoE: Invalid Net ID handle.
//
#define ERR_ECMV4_AOE_INVALID_NETID_HANDLE ((uint32_t)0xC0CEF00DL)

//
// MessageId: ERR_ECMV4_AOE_CONFIGURATION_IS_NOT_OPEN
//
// MessageText:
//
// AoE: Configuration is not open.
//
#define ERR_ECMV4_AOE_CONFIGURATION_IS_NOT_OPEN ((uint32_t)0xC0CEF00EL)

//
// MessageId: ERR_ECMV4_AOE_CONFIGURATION_IS_ALREADY_OPEN
//
// MessageText:
//
// AoE: Configuration is already open.
//
#define ERR_ECMV4_AOE_CONFIGURATION_IS_ALREADY_OPEN ((uint32_t)0xC0CEF00FL)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_INVALID_TRANSFER_HANDLE
//
// MessageText:
//
// AoE Client: Invalid transfer handle.
//
#define ERR_ECMV4_AOE_CLIENT_INVALID_TRANSFER_HANDLE ((uint32_t)0xC0CEF010L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_INVALID_TRANSFER_STATE
//
// MessageText:
//
// AoE Client: Invalid transfer state.
//
#define ERR_ECMV4_AOE_CLIENT_INVALID_TRANSFER_STATE ((uint32_t)0xC0CEF011L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_TRANSFER_ABORTED
//
// MessageText:
//
// AoE Client: Transfer aborted.
//
#define ERR_ECMV4_AOE_CLIENT_TRANSFER_ABORTED ((uint32_t)0xC0CEF012L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_PROTOCOL_TIMEOUT
//
// MessageText:
//
// AoE Client: Protocol Timeout.
//
#define ERR_ECMV4_AOE_CLIENT_PROTOCOL_TIMEOUT ((uint32_t)0xC0CEF013L)

//
// MessageId: ERR_ECMV4_AOE_UNKNOWN_RETURN_CODE
//
// MessageText:
//
// AoE: Unknown return code.
//
#define ERR_ECMV4_AOE_UNKNOWN_RETURN_CODE ((uint32_t)0xC0CEF014L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_UNKNOWN_AOE_ERROR
//
// MessageText:
//
// AoE Client: Unknown AoE Error.
//
#define ERR_ECMV4_AOE_CLIENT_UNKNOWN_AOE_ERROR ((uint32_t)0xC0CEF015L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_TRANSFER_SEGMENT_TOO_LONG
//
// MessageText:
//
// AoE Client: Transfer segment too long.
//
#define ERR_ECMV4_AOE_CLIENT_TRANSFER_SEGMENT_TOO_LONG ((uint32_t)0xC0CEF016L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_IS_INITIALIZING
//
// MessageText:
//
// AoE Client: Is initializing.
//
#define ERR_ECMV4_AOE_CLIENT_IS_INITIALIZING ((uint32_t)0xC0CEF017L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_REQUEST_DESTINATION_PROBLEM
//
// MessageText:
//
// AoE Client: Request Destination Problem.
//
#define ERR_ECMV4_AOE_CLIENT_REQUEST_DESTINATION_PROBLEM ((uint32_t)0xC0CEF018L)

//
// MessageId: ERR_ECMV4_AOE_CLIENT_MAX_SEGMENT_BYTES_TOO_LOW_FOR_FIRST_SEGMENT
//
// MessageText:
//
// AoE Client: Max segment bytes too low for first segment.
//
#define ERR_ECMV4_AOE_CLIENT_MAX_SEGMENT_BYTES_TOO_LOW_FOR_FIRST_SEGMENT ((uint32_t)0xC0CEF019L)

#endif  /* __ECMV4_RESULTS_H */
