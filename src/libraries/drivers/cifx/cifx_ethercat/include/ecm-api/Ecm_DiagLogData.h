/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: Ecm_DiagLogData.h 91345 2020-03-27 11:11:06Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef _ECM_DIAGLOGDATA_H
#define _ECM_DIAGLOGDATA_H

/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_PACK_1(ECM_DIAGLOGDATA)
#endif

/******************************************************************************
 * Diagnostic Entry Structure
 */

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_HEADER_Ttag
{
  uint16_t usEntryType;
  uint64_t ullTimestampNs;
} ECM_DIAG_ENTRY_HEADER_T;

/* Entry Type: 0x0000 is reserved for no data */
enum ECM_DIAG_ENTRY_TYPE_Etag
{
  VAL_ECM_DIAG_ENTRY_TYPE_NEW_STATE                        = 0x0001,
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_ON                           = 0x0002,
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_OFF                          = 0x0003,
  VAL_ECM_DIAG_ENTRY_TYPE_CHANNEL_INIT                     = 0x0004,
  VAL_ECM_DIAG_ENTRY_TYPE_DPM_WATCHDOG                     = 0x0005,
  VAL_ECM_DIAG_ENTRY_TYPE_TOPOLOGY_CHANGED                 = 0x0006,
  VAL_ECM_DIAG_ENTRY_TYPE_INTERNAL_ERROR                   = 0x0007,
  VAL_ECM_DIAG_ENTRY_TYPE_ALL_SLAVES_LOST                  = 0x0008,
  VAL_ECM_DIAG_ENTRY_TYPE_BUS_SCAN_REQUESTED               = 0x0009,
  VAL_ECM_DIAG_ENTRY_TYPE_IDENTITY_MISMATCH                = 0x000A,
  VAL_ECM_DIAG_ENTRY_TYPE_COE_INITCMD_FAILED               = 0x000B,
  VAL_ECM_DIAG_ENTRY_TYPE_SOE_INITCMD_FAILED               = 0x000C,
  VAL_ECM_DIAG_ENTRY_TYPE_EOE_INITCMD_FAILED               = 0x000D,
  VAL_ECM_DIAG_ENTRY_TYPE_AOE_INITCMD_FAILED               = 0x000E,
  VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_WARNING              = 0x000F,
  VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_FAILED               = 0x0010,
  VAL_ECM_DIAG_ENTRY_TYPE_ALCONTROL_FAILED                 = 0x0011,
  VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_ECAT_FAILED        = 0x0012,
  VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_PDI_FAILED         = 0x0013,
  VAL_ECM_DIAG_ENTRY_TYPE_SII_READ_REQUEST_FAILED          = 0x0014,
  VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_WARNING                    = 0x0015,
  VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_ERROR                      = 0x0016,
  VAL_ECM_DIAG_ENTRY_TYPE_VOE_INITCMD_FAILED               = 0x0017,
};

/* used by type:
 * - VAL_ECM_DIAG_ENTRY_TYPE_NEW_STATE
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_NEW_STATE_Ttag
{
  uint8_t bState;
} ECM_DIAG_ENTRY_NEW_STATE_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_INTERNAL_ERROR
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_INTERNAL_ERROR_Ttag
{
  uint32_t ulFunctionId;
  uint32_t ulErrorCode;
} ECM_DIAG_ENTRY_INTERNAL_ERROR_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_IDENTITY_MISMATCH
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_IDENTITY_MISMATCH_Ttag
{
  uint16_t usTopologyPosition;
  uint16_t usCompareFlags;
  uint32_t ulExpectedVendorId;
  uint32_t ulExpectedProductCode;
  uint32_t ulExpectedRevisionNo;
  uint32_t ulExpectedSerialNo;
  uint32_t ulFoundVendorId;
  uint32_t ulFoundProductCode;
  uint32_t ulFoundRevisionNo;
  uint32_t ulFoundSerialNo;
} ECM_DIAG_ENTRY_IDENTITY_MISMATCH_T;

enum ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_FLAGS_Etag
{
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_VENDOR_ID = 0x0001,
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_PRODUCT_CODE = 0x0002,
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_REVISION_NO = 0x0004,
  MSK_ECM_DIAG_ENTRY_IDENTITY_MISMATCH_COMPARE_SERIAL_NO = 0x0008,
};


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_COE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_COE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint16_t usIndex;
  uint8_t bSubIndex;
  uint8_t bAction;
  uint16_t fIsCompleteAccess;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_COE_INITCMD_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SOE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint8_t bDriveNo;
  uint16_t usIDN;
  uint8_t bElements;
  uint8_t bAction;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_VOE_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_WARNING
 * - VAL_ECM_DIAG_ENTRY_TYPE_REG_INITCMD_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_REG_INITCMD_INFO_Ttag
{
  uint16_t usStationAddress;
  uint8_t bCmd;
  uint16_t usAdo;
  uint16_t usLength;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_REG_INITCMD_INFO_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_ALCONTROL_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_ALCONTROL_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint8_t bTargetState;
  uint16_t usAlStatusCode;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_ALCONTROL_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_ECAT_FAILED
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_ASSIGN_TO_PDI_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SII_READ_REQUEST_FAILED
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SII_REQUEST_FAILED_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulSiiWordOffset;
  uint32_t ulResult;
} ECM_DIAG_ENTRY_SII_REQUEST_FAILED_T;


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_WARNING
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SLAVE_WARNING_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulWarningType;
  uint32_t ulWarningParam; /* value depends on ulWarningType */
} ECM_DIAG_ENTRY_SLAVE_WARNING_T;

enum ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_Etag
{
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_ADVERTISED_64BIT_DC_NOT_WORKING = 0x00000001,
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_ADVERTISED_DC_NOT_WORKING = 0x00000002,
  ECM_DIAG_ENTRY_SLAVE_WARNING_TYPE_SLAVE_DID_NOT_ACCEPT_EOE_SET_IP_PARAMS = 0x00000003, /* ulWarningParam contains ECM_ERROR_CODE_E */
};


/* used by types:
 * - VAL_ECM_DIAG_ENTRY_TYPE_SLAVE_ERROR
 */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_SLAVE_ERROR_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulErrorType;
  uint32_t ulErrorParam; /* value depends on ulErrorType */
} ECM_DIAG_ENTRY_SLAVE_ERROR_T;

enum ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_Etag
{
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_SYNC_NOT_POSSIBLE_WITHOUT_WORKING_DC = 0x00000001,
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_SYNC_LATCHING_ERROR = 0x00000002, /* ulErrorParam contains ECM_ERROR_CODE_E */
  ECM_DIAG_ENTRY_SLAVE_ERROR_TYPE_STATE_CHANGE_ENCOUNTERED = 0x00000003, /* ulErrorParam contains ECM_ERROR_CODE_E */
};


typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_DIAG_ENTRY_DATA_Ttag
{
  ECM_DIAG_ENTRY_NEW_STATE_T tNewState;
  ECM_DIAG_ENTRY_INTERNAL_ERROR_T tInternalError;
  ECM_DIAG_ENTRY_IDENTITY_MISMATCH_T tIdentityMismatch;
  ECM_DIAG_ENTRY_COE_INITCMD_FAILED_T tCoEInitCmdFailed;
  ECM_DIAG_ENTRY_SOE_INITCMD_FAILED_T tSoEInitCmdFailed;
  ECM_DIAG_ENTRY_VOE_INITCMD_FAILED_T tVoEInitCmdFailed;
  ECM_DIAG_ENTRY_REG_INITCMD_INFO_T tRegInitCmdInfo;
  ECM_DIAG_ENTRY_ALCONTROL_FAILED_T tAlControlFailed;
  ECM_DIAG_ENTRY_SII_ASSIGN_FAILED_T tSiiAssignFailed;
  ECM_DIAG_ENTRY_SII_REQUEST_FAILED_T tSiiRequestFailed;
  ECM_DIAG_ENTRY_SLAVE_WARNING_T tSlaveWarning;
  ECM_DIAG_ENTRY_SLAVE_ERROR_T tSlaveError;
} ECM_DIAG_ENTRY_DATA_T;


typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_DIAG_ENTRY_Ttag
{
  ECM_DIAG_ENTRY_HEADER_T tHead;
  ECM_DIAG_ENTRY_DATA_T tData;
} ECM_DIAG_ENTRY_T;

/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_UNPACK_1(ECM_DIAGLOGDATA)
#endif

#endif
