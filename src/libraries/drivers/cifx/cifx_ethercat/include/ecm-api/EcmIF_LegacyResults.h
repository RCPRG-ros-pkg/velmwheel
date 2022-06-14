/**************************************************************************************
Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_LegacyResults.h 82353 2019-04-09 10:28:34Z Sven $:

Description:
  EtherCATMaster_Results.h
**************************************************************************************/

#ifndef __ETHERCATMASTER_RESULTS_H
#define __ETHERCATMASTER_RESULTS_H

#include<stdint.h>

/////////////////////////////////////////////////////////////////////////////////////
// EtherCAT Master Packet Status codes (EtherCAT Master Stack)
/////////////////////////////////////////////////////////////////////////////////////
//
// MessageId: ERR_ETHERCAT_MASTER_COMMAND_INVALID
//
// MessageText:
//
// Invalid command received.
//
#define ERR_ETHERCAT_MASTER_COMMAND_INVALID ((uint32_t)0xC0650001L)

//
// MessageId: ERR_ETHERCAT_MASTER_NO_LINK
//
// MessageText:
//
// No link exists.
//
#define ERR_ETHERCAT_MASTER_NO_LINK      ((uint32_t)0xC0650002L)

//
// MessageId: ERR_ETHERCAT_MASTER_ERROR_READING_BUSCONFIG
//
// MessageText:
//
// Error during reading the bus configuration.
//
#define ERR_ETHERCAT_MASTER_ERROR_READING_BUSCONFIG ((uint32_t)0xC0650003L)

//
// MessageId: ERR_ETHERCAT_MASTER_ERROR_PARSING_BUSCONFIG
//
// MessageText:
//
// Error during processing the bus configuration.
//
#define ERR_ETHERCAT_MASTER_ERROR_PARSING_BUSCONFIG ((uint32_t)0xC0650004L)

//
// MessageId: ERR_ETHERCAT_MASTER_ERROR_BUSSCAN_FAILED
//
// MessageText:
//
// Existing bus does not match configured bus.
//
#define ERR_ETHERCAT_MASTER_ERROR_BUSSCAN_FAILED ((uint32_t)0xC0650005L)

//
// MessageId: ERR_ETHERCAT_MASTER_NOT_ALL_SLAVES_AVAIL
//
// MessageText:
//
// Not all slaves are available.
//
#define ERR_ETHERCAT_MASTER_NOT_ALL_SLAVES_AVAIL ((uint32_t)0xC0650006L)

//
// MessageId: ERR_ETHERCAT_MASTER_STOPMASTER_ERROR
//
// MessageText:
//
// Error during Reset (stopping the master).
//
#define ERR_ETHERCAT_MASTER_STOPMASTER_ERROR ((uint32_t)0xC0650007L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEINITMASTER_ERROR
//
// MessageText:
//
// Error during Reset (deinitialize the master).
//
#define ERR_ETHERCAT_MASTER_DEINITMASTER_ERROR ((uint32_t)0xC0650008L)

//
// MessageId: ERR_ETHERCAT_MASTER_CLEANUP_ERROR
//
// MessageText:
//
// Error during Reset (cleanup the dynamic resources).
//
#define ERR_ETHERCAT_MASTER_CLEANUP_ERROR ((uint32_t)0xC0650009L)

//
// MessageId: ERR_ETHERCAT_MASTER_CRITIAL_ERROR_STATE
//
// MessageText:
//
// Master is in critical error state, reset required.
//
#define ERR_ETHERCAT_MASTER_CRITIAL_ERROR_STATE ((uint32_t)0xC065000AL)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_BUSCYCLETIME
//
// MessageText:
//
// The requested bus cycle time is invalid.
//
#define ERR_ETHERCAT_MASTER_INVALID_BUSCYCLETIME ((uint32_t)0xC065000BL)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_BROKEN_SLAVE_BEHAVIOUR_PARA
//
// MessageText:
//
// Invalid parameter for broken slave behaviour.
//
#define ERR_ETHERCAT_MASTER_INVALID_BROKEN_SLAVE_BEHAVIOUR_PARA ((uint32_t)0xC065000CL)

//
// MessageId: ERR_ETHERCAT_MASTER_WRONG_INTERNAL_STATE
//
// MessageText:
//
// Master is in wrong internal state.
//
#define ERR_ETHERCAT_MASTER_WRONG_INTERNAL_STATE ((uint32_t)0xC065000DL)

//
// MessageId: ERR_ETHERCAT_MASTER_WATCHDOG_TIMEOUT_EXPIRED
//
// MessageText:
//
// The watchdog expired.
//
#define ERR_ETHERCAT_MASTER_WATCHDOG_TIMEOUT_EXPIRED ((uint32_t)0xC065000EL)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALID_SLAVEID
//
// MessageText:
//
// Invalid SlaveId was used for CoE.
//
#define ERR_ETHERCAT_MASTER_COE_INVALID_SLAVEID ((uint32_t)0xC065000FL)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_NO_RESOURCE
//
// MessageText:
//
// No available resources for CoE transfer.
//
#define ERR_ETHERCAT_MASTER_COE_NO_RESOURCE ((uint32_t)0xC0650010L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INTERNAL_ERROR
//
// MessageText:
//
// Internal error during CoE usage.
//
#define ERR_ETHERCAT_MASTER_COE_INTERNAL_ERROR ((uint32_t)0xC0650011L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALID_INDEX
//
// MessageText:
//
// Invalid slave index requested.
//
#define ERR_ETHERCAT_MASTER_COE_INVALID_INDEX ((uint32_t)0xC0650012L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALID_COMMUNICATION_STATE
//
// MessageText:
//
// Invalid bus communication state for CoE usage.
//
#define ERR_ETHERCAT_MASTER_COE_INVALID_COMMUNICATION_STATE ((uint32_t)0xC0650013L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_FRAME_LOST
//
// MessageText:
//
// Frame with CoE data is lost.
//
#define ERR_ETHERCAT_MASTER_COE_FRAME_LOST ((uint32_t)0xC0650014L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_TIMEOUT
//
// MessageText:
//
// Timeout during CoE service.
//
#define ERR_ETHERCAT_MASTER_COE_TIMEOUT  ((uint32_t)0xC0650015L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_SLAVE_NOT_ADDRESSABLE
//
// MessageText:
//
// Slave is not addressable (not on bus or power down?).
//
#define ERR_ETHERCAT_MASTER_COE_SLAVE_NOT_ADDRESSABLE ((uint32_t)0xC0650016L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALID_LIST_TYPE
//
// MessageText:
//
// Invalid list type requested (during GetOdList).
//
#define ERR_ETHERCAT_MASTER_COE_INVALID_LIST_TYPE ((uint32_t)0xC0650017L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_SLAVE_RESPONSE_TOO_BIG
//
// MessageText:
//
// Data in Slave Response is too big for confirmation packet.
//
#define ERR_ETHERCAT_MASTER_COE_SLAVE_RESPONSE_TOO_BIG ((uint32_t)0xC0650018L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALID_ACCESSBITMASK
//
// MessageText:
//
// Invalid access mask selected (during GetEntryDesc).
//
#define ERR_ETHERCAT_MASTER_COE_INVALID_ACCESSBITMASK ((uint32_t)0xC0650019L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_WKC_ERROR
//
// MessageText:
//
// Slave Working Counter error during CoE service.
//
#define ERR_ETHERCAT_MASTER_COE_WKC_ERROR ((uint32_t)0xC065001AL)

//
// MessageId: ERR_ETHERCAT_MASTER_SERVICE_IN_USE
//
// MessageText:
//
// The service is already in use.
//
#define ERR_ETHERCAT_MASTER_SERVICE_IN_USE ((uint32_t)0xC065001BL)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_COMMUNICATION_STATE
//
// MessageText:
//
// Command is not useable in this communication state.
//
#define ERR_ETHERCAT_MASTER_INVALID_COMMUNICATION_STATE ((uint32_t)0xC065001CL)

//
// MessageId: ERR_ETHERCAT_MASTER_DC_NOT_ACTIVATED
//
// MessageText:
//
// Distributed Clocks must be activated for this command.
//
#define ERR_ETHERCAT_MASTER_DC_NOT_ACTIVATED ((uint32_t)0xC065001DL)

//
// MessageId: ERR_ETHERCAT_MASTER_BUS_SCAN_CURRENTLY_RUNNING
//
// MessageText:
//
// Bus Scan is currently running.
//
#define ERR_ETHERCAT_MASTER_BUS_SCAN_CURRENTLY_RUNNING ((uint32_t)0xC065001EL)

//
// MessageId: ERR_ETHERCAT_MASTER_BUS_SCAN_TIMEOUT
//
// MessageText:
//
// Bus Scan Timeout. No slave found.
//
#define ERR_ETHERCAT_MASTER_BUS_SCAN_TIMEOUT ((uint32_t)0xC065001FL)

//
// MessageId: ERR_ETHERCAT_MASTER_BUS_SCAN_NOT_READY_YET
//
// MessageText:
//
// Bus Scan is not ready yet.
//
#define ERR_ETHERCAT_MASTER_BUS_SCAN_NOT_READY_YET ((uint32_t)0xC0650020L)

//
// MessageId: ERR_ETHERCAT_MASTER_BUS_SCAN_INVALID_SLAVE
//
// MessageText:
//
// Invalid slave. No information available.
//
#define ERR_ETHERCAT_MASTER_BUS_SCAN_INVALID_SLAVE ((uint32_t)0xC0650021L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_INVALIDACCESS
//
// MessageText:
//
// Slave does not allow reading or writing (CoE access).
//
#define ERR_ETHERCAT_MASTER_COE_INVALIDACCESS ((uint32_t)0xC0650022L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_NO_MBX_SUPPORT
//
// MessageText:
//
// Slave does not support a mailbox.
//
#define ERR_ETHERCAT_MASTER_COE_NO_MBX_SUPPORT ((uint32_t)0xC0650023L)

//
// MessageId: ERR_ETHERCAT_MASTER_COE_NO_COE_SUPPORT
//
// MessageText:
//
// Slave does not support CoE.
//
#define ERR_ETHERCAT_MASTER_COE_NO_COE_SUPPORT ((uint32_t)0xC0650024L)

//
// MessageId: ERR_ETHERCAT_MASTER_TASK_CREATION_FAILED
//
// MessageText:
//
// Task could not be created during runtime.
//
#define ERR_ETHERCAT_MASTER_TASK_CREATION_FAILED ((uint32_t)0xC0650025L)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_SLAVE_SM_CONFIGURATION
//
// MessageText:
//
// The Sync Manager configuration of a slave is invalid.
//
#define ERR_ETHERCAT_MASTER_INVALID_SLAVE_SM_CONFIGURATION ((uint32_t)0xC0650026L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TOGGLE
//
// MessageText:
//
// SDO abort code: Toggle bit not alternated.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TOGGLE ((uint32_t)0xC0650027L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TIMEOUT
//
// MessageText:
//
// SDO abort code: SDO protocol timed out.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TIMEOUT ((uint32_t)0xC0650028L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_CCS_SCS
//
// MessageText:
//
// SDO abort code: Client/server command specifier not valid or unknown.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_CCS_SCS ((uint32_t)0xC0650029L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_BLK_SIZE
//
// MessageText:
//
// SDO abort code: Invalid block size (block mode only).
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_BLK_SIZE ((uint32_t)0xC065002AL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_SEQNO
//
// MessageText:
//
// SDO abort code: Invalid sequence number (block mode only).
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_SEQNO ((uint32_t)0xC065002BL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_CRC
//
// MessageText:
//
// SDO abort code: CRC error (block mode only).
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_CRC ((uint32_t)0xC065002CL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_MEMORY
//
// MessageText:
//
// SDO abort code: Out of memory.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_MEMORY ((uint32_t)0xC065002DL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_ACCESS
//
// MessageText:
//
// SDO abort code: Unsupported access to an object.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_ACCESS ((uint32_t)0xC065002EL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_WRITEONLY
//
// MessageText:
//
// SDO abort code:  Attempt to read a write only object.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_WRITEONLY ((uint32_t)0xC065002FL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_READONLY
//
// MessageText:
//
// SDO abort code: Attempt to write a read only object.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_READONLY ((uint32_t)0xC0650030L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_INDEX
//
// MessageText:
//
// SDO abort code: Object does not exist in the object dictionary.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_INDEX ((uint32_t)0xC0650031L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_PDO_MAP
//
// MessageText:
//
// SDO abort code: Object cannot be mapped to the PDO.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_PDO_MAP ((uint32_t)0xC0650032L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_PDO_LEN
//
// MessageText:
//
// SDO abort code: The number and length of the objects to be mapped would exceed PDO length.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_PDO_LEN ((uint32_t)0xC0650033L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_P_INCOMP
//
// MessageText:
//
// SDO abort code: General parameter incompatibility reason.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_P_INCOMP ((uint32_t)0xC0650034L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_I_INCOMP
//
// MessageText:
//
// SDO abort code: General internal incompatibility in the device.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_I_INCOMP ((uint32_t)0xC0650035L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_HARDWARE
//
// MessageText:
//
// SDO abort code: Access failed due to an hardware error.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_HARDWARE ((uint32_t)0xC0650036L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE
//
// MessageText:
//
// SDO abort code: Data type does not match, length of service parameter does not match.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE ((uint32_t)0xC0650037L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE1
//
// MessageText:
//
// SDO abort code: Data type does not match, length of service parameter too high.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE1 ((uint32_t)0xC0650038L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE2
//
// MessageText:
//
// SDO abort code: Data type does not match, length of service parameter too low.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_SIZE2 ((uint32_t)0xC0650039L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_OFFSET
//
// MessageText:
//
// SDO abort code: Sub-index does not exist.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_OFFSET ((uint32_t)0xC065003AL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE
//
// MessageText:
//
// SDO abort code: Value range of parameter exceeded (only for write access).
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE ((uint32_t)0xC065003BL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE1
//
// MessageText:
//
// SDO abort code: Value of parameter written too high.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE1 ((uint32_t)0xC065003CL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE2
//
// MessageText:
//
// SDO abort code: Value of parameter written too low.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DATA_RANGE2 ((uint32_t)0xC065003DL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_MINMAX
//
// MessageText:
//
// SDO abort code: Maximum value is less than minimum value.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_MINMAX ((uint32_t)0xC065003EL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_GENERAL
//
// MessageText:
//
// SDO abort code: general error.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_GENERAL ((uint32_t)0xC065003FL)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER
//
// MessageText:
//
// SDO abort code: Data cannot be transferred or stored to the application.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER ((uint32_t)0xC0650040L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER1
//
// MessageText:
//
// SDO abort code: Data cannot be transferred or stored to the application because of local control.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER1 ((uint32_t)0xC0650041L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER2
//
// MessageText:
//
// SDO abort code: Data cannot be transferred or stored to the application because of the present device state.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_TRANSFER2 ((uint32_t)0xC0650042L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DICTIONARY
//
// MessageText:
//
// SDO abort code: Object dictionary dynamic generation fails or no object dictionary is present (e.g. object dictionary is generated from file and generation fails because of an file error).
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_DICTIONARY ((uint32_t)0xC0650043L)

//
// MessageId: ERR_ETHERCAT_MASTER_SDO_ABORTCODE_UNKNOWN
//
// MessageText:
//
// SDO abort code: unknown code.
//
#define ERR_ETHERCAT_MASTER_SDO_ABORTCODE_UNKNOWN ((uint32_t)0xC0650044L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_ERROR
//
// MessageText:
//
// Slave status code: Unspecified error.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_ERROR ((uint32_t)0xC0650045L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVREQSTATECNG
//
// MessageText:
//
// Slave status code: Invalid requested state change.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVREQSTATECNG ((uint32_t)0xC0650046L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_UNKREQSTATE
//
// MessageText:
//
// Slave status code: Unknown requested state.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_UNKREQSTATE ((uint32_t)0xC0650047L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_BOOTSTRAPNSUPP
//
// MessageText:
//
// Slave status code: Bootstrap not supported.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_BOOTSTRAPNSUPP ((uint32_t)0xC0650048L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDFW
//
// MessageText:
//
// Slave status code: No valid firmware.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDFW ((uint32_t)0xC0650049L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDMBXCNF1
//
// MessageText:
//
// Slave status code: Invalid mailbox configuration1.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDMBXCNF1 ((uint32_t)0xC065004AL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDMBXCNF2
//
// MessageText:
//
// Slave status code: Invalid mailbox configuration2.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDMBXCNF2 ((uint32_t)0xC065004BL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDSMCNF
//
// MessageText:
//
// Slave status code: Invalid sync manager configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVALIDSMCNF ((uint32_t)0xC065004CL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDIN
//
// MessageText:
//
// Slave status code: No valid inputs available.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDIN ((uint32_t)0xC065004DL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDOUT
//
// MessageText:
//
// Slave status code: No valid outputs.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_NOVALIDOUT ((uint32_t)0xC065004EL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SYNCERROR
//
// MessageText:
//
// Slave status code: Synchronization error.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SYNCERROR ((uint32_t)0xC065004FL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SMWATCHDOG
//
// MessageText:
//
// Slave status code: Sync manager watchdog.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SMWATCHDOG ((uint32_t)0xC0650050L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVSMTYPES
//
// MessageText:
//
// Slave status code: Invalid Sync Manager Types.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVSMTYPES ((uint32_t)0xC0650051L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVOUTCONFIG
//
// MessageText:
//
// Slave status code: Invalid Output Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVOUTCONFIG ((uint32_t)0xC0650052L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVINCONFIG
//
// MessageText:
//
// Slave status code: Invalid Input Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVINCONFIG ((uint32_t)0xC0650053L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVWDCONFIG
//
// MessageText:
//
// Slave status code: Invalid Watchdog Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVWDCONFIG ((uint32_t)0xC0650054L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDCOLDRS
//
// MessageText:
//
// Slave status code: Slave needs cold start.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDCOLDRS ((uint32_t)0xC0650055L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDINIT
//
// MessageText:
//
// Slave status code: Slave needs INIT.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDINIT ((uint32_t)0xC0650056L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDPREOP
//
// MessageText:
//
// Slave status code: Slave needs PREOP.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDPREOP ((uint32_t)0xC0650057L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDSAFEOP
//
// MessageText:
//
// Slave status code: Slave needs SAFEOP.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_SLVNEEDSAFEOP ((uint32_t)0xC0650058L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVOUTFMMUCNFG
//
// MessageText:
//
// Slave status code: Invalid Output FMMU Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVOUTFMMUCNFG ((uint32_t)0xC0650059L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVINFMMUCNFG
//
// MessageText:
//
// Slave status code: Invalid Input FMMU Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVINFMMUCNFG ((uint32_t)0xC065005AL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCSYNCCNFG
//
// MessageText:
//
// Slave status code: Invalid DC SYNCH Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCSYNCCNFG ((uint32_t)0xC065005BL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCLATCHCNFG
//
// MessageText:
//
// Slave status code: Invalid DC Latch Configuration.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCLATCHCNFG ((uint32_t)0xC065005CL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_PLLERROR
//
// MessageText:
//
// Slave status code: PLL Error.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_PLLERROR ((uint32_t)0xC065005DL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCIOERROR
//
// MessageText:
//
// Slave status code: Invalid DC IO Error.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCIOERROR ((uint32_t)0xC065005EL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCTOERROR
//
// MessageText:
//
// Slave status code: Invalid DC Timeout Error.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_INVDCTOERROR ((uint32_t)0xC065005FL)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_EOE
//
// MessageText:
//
// Slave status code: MBX_EOE.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_EOE ((uint32_t)0xC0650060L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_COE
//
// MessageText:
//
// Slave status code: MBX_COE.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_COE ((uint32_t)0xC0650061L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_FOE
//
// MessageText:
//
// Slave status code: MBX_FOE.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_FOE ((uint32_t)0xC0650062L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_SOE
//
// MessageText:
//
// Slave status code: MBX_SOE.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_SOE ((uint32_t)0xC0650063L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_VOE
//
// MessageText:
//
// Slave status code: MBX_VOE.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_MBX_VOE ((uint32_t)0xC0650064L)

//
// MessageId: ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_OTHER
//
// MessageText:
//
// Slave status code: vendor specific error code.
//
#define ERR_ETHERCAT_MASTER_DEVICE_STATUSCODE_OTHER ((uint32_t)0xC0650065L)

//
// MessageId: ERR_ETHERCAT_MASTER_PREVIOUS_PORT_MISSING
//
// MessageText:
//
// Slave status code: PreviousPort configuration missing in bus configuration file (outdated configurator).
//
#define ERR_ETHERCAT_MASTER_PREVIOUS_PORT_MISSING ((uint32_t)0xC0650066L)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_ALREADY_STARTED
//
// MessageText:
//
// Configuration already started, cannot be started again.
//
#define ERR_ETHERCAT_MASTER_CONFIG_ALREADY_STARTED ((uint32_t)0xC0650067L)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_NOT_STARTED
//
// MessageText:
//
// Configuration was not started before.
//
#define ERR_ETHERCAT_MASTER_CONFIG_NOT_STARTED ((uint32_t)0xC0650068L)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_SLAVE_INDEX_ALREADY_EXISTS
//
// MessageText:
//
// Slave index already exists, cannot be created again.
//
#define ERR_ETHERCAT_MASTER_CONFIG_SLAVE_INDEX_ALREADY_EXISTS ((uint32_t)0xC0650069L)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_SLAVE_PHYS_ADDR_ALREADY_EXISTS
//
// MessageText:
//
// Slave physical address already exits, cannot be created again.
//
#define ERR_ETHERCAT_MASTER_CONFIG_SLAVE_PHYS_ADDR_ALREADY_EXISTS ((uint32_t)0xC065006AL)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_SLAVE_AUTOINC_ADDR_ALREADY_EXISTS
//
// MessageText:
//
// Slave auto increment address already exits, cannot be created again.
//
#define ERR_ETHERCAT_MASTER_CONFIG_SLAVE_AUTOINC_ADDR_ALREADY_EXISTS ((uint32_t)0xC065006BL)

//
// MessageId: ERR_ETHERCAT_MASTER_CONFIG_SLAVE_INDEX_NOT_EXISTS
//
// MessageText:
//
// Slave index does not exists, must be created before.
//
#define ERR_ETHERCAT_MASTER_CONFIG_SLAVE_INDEX_NOT_EXISTS ((uint32_t)0xC065006CL)

//
// MessageId: ERR_ETHERCAT_MASTER_WRONG_VALIDATE_DATA_LEN
//
// MessageText:
//
// Wrong length value for validate data.
//
#define ERR_ETHERCAT_MASTER_WRONG_VALIDATE_DATA_LEN ((uint32_t)0xC065006DL)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_ECAT_CMD
//
// MessageText:
//
// Invalid value for EtherCAT command.
//
#define ERR_ETHERCAT_MASTER_INVALID_ECAT_CMD ((uint32_t)0xC065006EL)

//
// MessageId: ERR_ETHERCAT_MASTER_PRECONFIGURED_DATA_CURRENTLY_NOT_SUPPORTED
//
// MessageText:
//
// Sending preconfigured cyclic data is currently not supported.
//
#define ERR_ETHERCAT_MASTER_PRECONFIGURED_DATA_CURRENTLY_NOT_SUPPORTED ((uint32_t)0xC065006FL)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_STATE
//
// MessageText:
//
// Invalid value for EtherCAT state.
//
#define ERR_ETHERCAT_MASTER_INVALID_STATE ((uint32_t)0xC0650070L)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_TRANSITION
//
// MessageText:
//
// Invalid value for EtherCAT transition.
//
#define ERR_ETHERCAT_MASTER_INVALID_TRANSITION ((uint32_t)0xC0650071L)

//
// MessageId: ERR_ETHERCAT_MASTER_COPY_INFOS_EXCEEDED
//
// MessageText:
//
// Maximum amount of copy infos exceeded.
//
#define ERR_ETHERCAT_MASTER_COPY_INFOS_EXCEEDED ((uint32_t)0xC0650072L)

//
// MessageId: ERR_ETHERCAT_MASTER_REDUNDANCY_AND_DC_ENABLED
//
// MessageText:
//
// Redundancy and Distributed clocks enabled at the same time (not possible).
//
#define ERR_ETHERCAT_MASTER_REDUNDANCY_AND_DC_ENABLED ((uint32_t)0xC0650073L)

//
// MessageId: ERR_ETHERCAT_MASTER_NO_SLAVES_CONFIGURED
//
// MessageText:
//
// At least one slave must be configured.
//
#define ERR_ETHERCAT_MASTER_NO_SLAVES_CONFIGURED ((uint32_t)0xC0650074L)

//
// MessageId: ERR_ETHERCAT_MASTER_STATE_CHANGE_BUSY
//
// MessageText:
//
// State change is currently busy.
//
#define ERR_ETHERCAT_MASTER_STATE_CHANGE_BUSY ((uint32_t)0xC0650075L)

//
// MessageId: ERR_ETHERCAT_MASTER_INVALID_TARGET_PHASE
//
// MessageText:
//
// Parameter target phase is invalid.
//
#define ERR_ETHERCAT_MASTER_INVALID_TARGET_PHASE ((uint32_t)0xC0650076L)

#endif  /* __ETHERCATMASTER_RESULTS_H */
