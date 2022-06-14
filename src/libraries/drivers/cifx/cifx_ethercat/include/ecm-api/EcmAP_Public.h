/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmAP_Public.h 82317 2019-04-08 12:38:24Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef ECM_AP_PUBLIC_H_INCLUDED
#define ECM_AP_PUBLIC_H_INCLUDED

#include <Hil_Packet.h>

/***************************************************************************************/

#define ECM_AP_PROCESS_QUEUE_NAME       "QUE_ECM_AP"


/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_PACK_1(ECM_AP_PUBLIC)
#endif


/*
 * Packet Ranges:
 */

#define ECM_AP_CMD_SET_DPM_CFG_REQ                      0x00009F00
#define ECM_AP_CMD_SET_DPM_CFG_CNF                      0x00009F01

#ifndef __ECM_AP_PACKED_PRE
#define __ECM_AP_PACKED_PRE __HIL_PACKED_PRE
#endif

#ifndef __ECM_AP_PACKED_POST
#define __ECM_AP_PACKED_POST __HIL_PACKED_POST
#endif

#define ECM_AP_STATE_INFO_STRING_LENGTH 15

typedef __ECM_AP_PACKED_PRE struct __ECM_AP_PACKED_POST ECM_AP_EXTENDED_STATUS_DATA_Ttag
{
  uint32_t ulDcEnabled; /* always set on ECMV3.X */
  uint32_t aulReserved[12];

  uint32_t ulMarker3;

  uint32_t ulValidBufferedDpmInputDataExchangesCount;
  uint32_t ulBlockedBufferedDpmInputDataExchangesCount;

  uint32_t ulValidBufferedDpmOutputDataExchangesCount;
  uint32_t ulBlockedBufferedDpmOutputDataExchangesCount;

  uint8_t abState[ECM_AP_STATE_INFO_STRING_LENGTH];
  uint8_t bCurrentState;
  uint32_t aulLastFiveCommunicationErrors[5];

  uint32_t ulCompleteCyclesCount;
  uint32_t ulCyclesWithLostFramesCount;

  uint32_t ulMarker0;
  uint32_t ulValidSynchInputDataExchangesCount;
  uint32_t ulCompletedSynchInputDataExchangesCount;
  uint32_t ulBlockedSynchInputDataExchangesCount;

  uint32_t ulValidSynchOutputDataExchangesCount;
  uint32_t ulCompletedSynchOutputDataExchangesCount;
  uint32_t ulBlockedSynchOutputDataExchangesCount;

  uint32_t ulMarker1;
  uint32_t ulBufferedBusInputDataExchangesCount;
  uint32_t ulBufferedBusOutputDataExchangesCount;
  uint32_t ulCompletedBusInputDataExchangesCount;

  uint32_t ulMarker2;

  /* only 2 dwords left here */
} ECM_AP_EXTENDED_STATUS_DATA_T;

/******************************************************************************
 * Packet: ECM_AP_CMD_CFG_SET_DPM_CFG_REQ/ECM_CP_CMD_SET_DPM_CFG_CNF
 *
 *          Configure DPM modes
 *          - Bus-Synchronous
 *          - Buffered
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_AP_SET_DPM_CFG_REQ_DATA_Ttag
{
  /** reserved for IO status */
  uint32_t                          ulSystemFlags;
  /** watchdog time in millisecs */
  uint32_t                          ulWatchdogTime;

  /* Stack configuration flags */
  uint32_t                          ulStackCfgFlags;

  /* Frames Lost threshold */
  uint32_t                          ulFramesLostThreshold;

  /* Threshold for bus-synchronous modes */
  uint32_t                          ulBusSynchronousThreshold;

  uint32_t                          ulBusSynchronousInputThreshold;

  uint32_t                          ulBusSynchronousOutputThreshold;
} ECM_AP_SET_DPM_CFG_REQ_DATA_T;

#define ECM_AP_SET_DPM_CFG_SYSTEM_FLAGS_APP_CONTROLLED        0x00000001

#define ECM_AP_SET_DPM_CFG_STACK_CFG_FLAGS_BUS_SYNCHRONOUS    0x00000001


typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_AP_SET_DPM_CFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T                 tHead;
  ECM_AP_SET_DPM_CFG_REQ_DATA_T       tData;
} ECM_AP_SET_DPM_CFG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_AP_SET_DPM_CFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T                 tHead;
} ECM_AP_SET_DPM_CFG_CNF_T;


/* packet union */
typedef union ECM_AP_SET_DPM_CFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T                 tHead;
  ECM_AP_SET_DPM_CFG_REQ_T            tReq;
  ECM_AP_SET_DPM_CFG_CNF_T            tCnf;
} ECM_AP_SET_DPM_CFG_PCK_T;


/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_UNPACK_1(ECM_AP_PUBLIC)
#endif

#endif // ECM_AP_PUBLIC_H_INCLUDED
