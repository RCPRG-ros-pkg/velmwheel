/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_Slave_Diag.h 82277 2019-04-05 09:43:04Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef __STRUCT_ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T__
#define __STRUCT_ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T__

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_Ttag
{
  uint32_t ulStationAddress;
  uint32_t ulAutoIncAddress;
  uint32_t ulCurrentState;
  uint32_t ulLastError;
  uint8_t  szSlaveName[80];
  uint32_t fEmergencyReported;
} ETHERCAT_MASTER_DIAG_GET_SLAVE_DIAG_T;

#endif
