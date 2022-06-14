/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_BusScan_Diag.h 82276 2019-04-05 09:40:55Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef __ECM_BUS_SCAN_INFO_T__
#define __ECM_BUS_SCAN_INFO_T__

#include <Hil_Compiler.h>

/* this structure is deliberately compatible with ECM v3.x */

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_BUS_SCAN_INFO_Ttag
{
  uint32_t ulVendorId;
  uint32_t ulProductCode;
  uint32_t ulRevisionNumber;
  uint32_t ulSerialNumber;
  uint32_t ulPortState;
} ETHERCAT_MASTER_BUS_SCAN_INFO_T;

#endif
