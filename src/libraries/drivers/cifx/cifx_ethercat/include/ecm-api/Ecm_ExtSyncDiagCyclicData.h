/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: Ecm_ExtSyncDiagCyclicData.h 65109 2016-10-31 08:25:13Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef ECM_EXTSYNCDIAGCYCLICDATA_H
#define ECM_EXTSYNCDIAGCYCLICDATA_H

typedef __PACKED_PRE struct ECM_EXT_SYNC_DIAG_CYCLIC_DATA_Ttag
{
  uint32_t ulExtSyncInfoFlags;
  uint16_t usExtSyncStationAddress;
  uint16_t usControlledStationAddress;
  uint64_t ullDcToExtTimeOffsetNs; /* internal DC timestamp (ns) + ullDcToExtTimeOffsetNs => external clock time (ns) */
  uint32_t ulDcExtErrorDiffNsSignMag;
  uint32_t ulExtSyncUpdateCount;
} __PACKED_POST ECM_EXT_SYNC_DIAG_CYLIC_DATA_T;

/* ulExtSyncInfoFlags */
enum ECM_EXT_SYNC_INFO_FLAGS_Etag
{
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_SYNC_MODE_SLAVE = 0x00000001,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_IS_64BIT = 0x00000004,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_NOT_CONNECTED = 0x00000010,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_SYNC_MODE_MASTER = 0x00000020,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONNECTED_AS_SLAVE = 0x00008000, /* result of !(External Device Not Connected) && (Sync Mode.Bit 1) */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 0x00FF0000,
  SRT_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 16,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_DC_TO_EXT_OFFSET_VALID = 0x20000000,
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_ACTIVE = 0x40000000, /* master is actively using External Synchronization on device */
  MSK_ECM_CYC_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONFIGURED = 0x80000000
};

#endif
