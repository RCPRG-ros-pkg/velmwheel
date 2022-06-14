/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_Public.h 89417 2020-02-03 10:48:26Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 yyyy-mm-dd    created
**************************************************************************************/

#ifndef ECM_IF_PUBLIC_H_INCLUDED
#define ECM_IF_PUBLIC_H_INCLUDED

#include <stdint.h>
#include <Hil_Packet.h>

#include "Ecm_DiagLogData.h"

/***************************************************************************************/

#define ECM_IF_PROCESS_QUEUE_NAME       "QUE_ECM_IF"


/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_PACK_1(ECM_IF_PUBLIC)
#endif

// ECM_IF_COMMAND_START                         = 0x00009E00,

/******************************************************************************/
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_ADDR_PHYS_Ttag
{
  uint16_t usAdp;
  uint16_t usAdo;
} ECM_IF_SLAVE_ADDR_PHYS_T;

typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_SLAVE_ADDR_Ttag
{
  ECM_IF_SLAVE_ADDR_PHYS_T tPhys;
  uint32_t ulLogAddress;
} ECM_IF_SLAVE_ADDR_T;

/******************************************************************************/
/* ulMasterStatusFlags */
enum ECM_IF_MASTER_STATUS_FLAGS_Etag
{
  MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_LOST = 0x00000001,
  MSK_ECM_IF_MASTER_STATUS_FLAGS_DC_XRMW_STOPPED = 0x00000002,
  MSK_ECM_IF_MASTER_STATUS_FLAGS_AT_LEAST_ONE_MANDATORY_SLAVE_NOT_IN_OP = 0x00000004, /* signaled when Master in OP and required slave is not in OP */
};

/******************************************************************************/
/* bCmd */
typedef enum ECM_IF_CMD_TYPE_Etag
{
  ECM_IF_COMMAND_NOP = 0,
  ECM_IF_COMMAND_APRD = 1,
  ECM_IF_COMMAND_APWR = 2,
  ECM_IF_COMMAND_APRW = 3,
  ECM_IF_COMMAND_FPRD = 4,
  ECM_IF_COMMAND_FPWR = 5,
  ECM_IF_COMMAND_FPRW = 6,
  ECM_IF_COMMAND_BRD = 7,
  ECM_IF_COMMAND_BWR = 8,
  ECM_IF_COMMAND_BRW = 9,
  ECM_IF_COMMAND_LRD = 10,
  ECM_IF_COMMAND_LWR = 11,
  ECM_IF_COMMAND_LRW = 12,
  ECM_IF_COMMAND_ARMW = 13,
  ECM_IF_COMMAND_FRMW = 14
} ECM_IF_CMD_TYPE_E;

/**************************************************************************************************

#######   #####    #####         #   #####   ###  ###           #      #####    #####   #######   #####    #####
#        #     #  #     #       #   #     #   #    #           # #    #     #  #     #  #        #     #  #     #
#        #        #            #    #         #    #          #   #   #        #        #        #        #
#####     #####   #           #      #####    #    #         #     #  #        #        #####     #####    #####
#              #  #          #            #   #    #         #######  #        #        #              #        #
#        #     #  #     #   #       #     #   #    #         #     #  #     #  #     #  #        #     #  #     #
#######   #####    #####   #         #####   ###  ###        #     #   #####    #####   #######   #####    #####

 */

#define ECM_IF_CMD_READ_REGS_REQ 0x9E70
#define ECM_IF_CMD_READ_REGS_CNF 0x9E71

#define ECM_IF_CMD_WRITE_REGS_REQ 0x9E72
#define ECM_IF_CMD_WRITE_REGS_CNF 0x9E73

#define ECM_IF_CMD_READ_SII_REQ 0x9E80
#define ECM_IF_CMD_READ_SII_CNF 0x9E81

#define ECM_IF_CMD_WRITE_SII_REQ 0x9E82
#define ECM_IF_CMD_WRITE_SII_CNF 0x9E83

/******************************************************************************
 * Packet: ECM_IF_CMD_READ_REGS_REQ/ECM_IF_CMD_READ_REGS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usPhysAddr;
  uint16_t usPhysLength;
} ECM_IF_READ_REGS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_REGS_REQ_DATA_T tData;
} ECM_IF_READ_REGS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usPhysAddr;
  uint16_t usPhysLength;
  uint8_t abData[1024]; /* actual byte length based on usPhysLength */
} ECM_IF_READ_REGS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_REGS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_REGS_CNF_DATA_T tData;
} ECM_IF_READ_REGS_CNF_T;


/* packet union */
typedef union ECM_IF_READ_REGS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_REGS_REQ_T tReq;
  ECM_IF_READ_REGS_CNF_T tCnf;
} ECM_IF_READ_REGS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_WRITE_REGS_REQ/ECM_IF_CMD_WRITE_REGS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usPhysAddr;
  uint16_t usPhysLength;
  uint8_t abData[1024]; /* actual byte length based on usPhysLength */
} ECM_IF_WRITE_REGS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_REGS_REQ_DATA_T tData;
} ECM_IF_WRITE_REGS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usPhysAddr;
  uint16_t usPhysLength;
} ECM_IF_WRITE_REGS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_REGS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_REGS_CNF_DATA_T tData;
} ECM_IF_WRITE_REGS_CNF_T;


/* packet union */
typedef union ECM_IF_WRITE_REGS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_REGS_REQ_T tReq;
  ECM_IF_WRITE_REGS_CNF_T tCnf;
} ECM_IF_WRITE_REGS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_READ_SII_REQ/ECM_IF_CMD_READ_SII_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulSiiWordOffset;
  uint32_t ulSiiByteLength; /* must be a multiple of 2 */
} ECM_IF_READ_SII_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_SII_REQ_DATA_T tData;
} ECM_IF_READ_SII_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulSiiWordOffset;
  uint32_t ulSiiByteLength; /* must be a multiple of 2 */
  uint8_t abData[1024];
} ECM_IF_READ_SII_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_SII_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_SII_CNF_DATA_T tData;
} ECM_IF_READ_SII_CNF_T;


/* packet union */
typedef union ECM_IF_READ_SII_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_SII_REQ_T tReq;
  ECM_IF_READ_SII_CNF_T tCnf;
} ECM_IF_READ_SII_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_WRITE_SII_REQ/ECM_IF_CMD_WRITE_SII_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulSiiWordOffset;
  uint32_t ulReserved; /* kept free for specific use in confirmation */
  uint8_t abData[1024]; /* actual length is defined by ulLen - 8 */
} ECM_IF_WRITE_SII_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_SII_REQ_DATA_T tData;
} ECM_IF_WRITE_SII_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulSiiWordOffset;
  uint32_t ulWrittenByteLength;
} ECM_IF_WRITE_SII_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WRITE_SII_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_SII_CNF_DATA_T tData;
}  ECM_IF_WRITE_SII_CNF_T;


/* packet union */
typedef union ECM_IF_WRITE_SII_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_WRITE_SII_REQ_T tReq;
  ECM_IF_WRITE_SII_CNF_T tCnf;
} ECM_IF_WRITE_SII_PCK_T;


/**********************************************************************************************************************

 #####   #######     #     #######  #######         #####   #######  #     #  #######  ######   #######  #
#     #     #       # #       #     #              #     #  #     #  ##    #     #     #     #  #     #  #
#           #      #   #      #     #              #        #     #  # #   #     #     #     #  #     #  #
 #####      #     #     #     #     #####          #        #     #  #  #  #     #     ######   #     #  #
      #     #     #######     #     #              #        #     #  #   # #     #     #   #    #     #  #
#     #     #     #     #     #     #              #     #  #     #  #    ##     #     #    #   #     #  #
 #####      #     #     #     #     #######         #####   #######  #     #     #     #     #  #######  #######

 */

enum EMC_IF_STATE_Etag
{
  ECM_IF_STATE_BUSOFF = 0,
  ECM_IF_STATE_INIT = 1,
  ECM_IF_STATE_PREOP = 2,
  ECM_IF_STATE_BOOT = 3,
  ECM_IF_STATE_SAFEOP = 4,
  ECM_IF_STATE_OP = 8,

  /* Master States */
  ECM_IF_STATE_LEAVE_OP = 0x18,
  ECM_IF_STATE_MANUAL_MODE = 0x1C,
  ECM_IF_STATE_BUSSCAN_COMPLETE_NO_PREOP = 0x1D,
  ECM_IF_STATE_BUSSCAN = 0x1E,
  ECM_IF_STATE_BUSSCAN_COMPLETE = 0x1F,

  /* Slave Error States */
  ECM_IF_STATE_NOT_CONNECTED = 0x00, /* used when master is actively communicating and has some valid slaves connected */
  ECM_IF_STATE_INIT_ERR = 0x11,
  ECM_IF_STATE_PREOP_ERR = 0x12,
  ECM_IF_STATE_BOOT_ERR = 0x13,
  ECM_IF_STATE_SAFEOP_ERR = 0x14,
  ECM_IF_STATE_UNKNOWN = 0xFF, /* used in BusOff */
};

/******************************************************************************/

#define ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ            0x9E00
#define ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF            0x9E01

#define ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ           0x9E02
#define ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF           0x9E03

#define ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ             0x9E04
#define ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF             0x9E05

#define ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ            0x9E06
#define ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF            0x9E07

#define ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ              0x9E08
#define ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF              0x9E09

#define ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ                0x9E0A
#define ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF                0x9E0B

#define ECM_IF_CMD_EXT_START_BUSSCAN_REQ                  0x9E0E
#define ECM_IF_CMD_EXT_START_BUSSCAN_CNF                  0x9E0F

/******************************************************************************
 * Packet: ECM_IF_CMD_SET_MASTER_TARGET_STATE_REQ/ECM_IF_CMD_SET_MASTER_TARGET_STATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_Ttag
{
  uint8_t bTargetState;
} ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_MASTER_TARGET_STATE_REQ_DATA_T tData;
} ECM_IF_SET_MASTER_TARGET_STATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_Ttag
{
  uint8_t bTargetState;
} ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_MASTER_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_MASTER_TARGET_STATE_CNF_DATA_T tData;
} ECM_IF_SET_MASTER_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_MASTER_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_MASTER_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_MASTER_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_MASTER_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_MASTER_CURRENT_STATE_REQ/ECM_IF_CMD_GET_MASTER_CURRENT_STATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_Ttag
{
  uint8_t bCurrentState;
  uint8_t bTargetState;
  uint32_t ulStopReason;
  uint32_t ulMasterStatusFlags;
} ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_CURRENT_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_CURRENT_STATE_CNF_DATA_T tData;
} ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_GET_MASTER_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_CURRENT_STATE_REQ_T tReq;
  ECM_IF_GET_MASTER_CURRENT_STATE_CNF_T tCnf;
} ECM_IF_GET_MASTER_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SET_SLAVE_TARGET_STATE_REQ/ECM_IF_CMD_SET_SLAVE_TARGET_STATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bTargetState;
} ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_SLAVE_TARGET_STATE_REQ_DATA_T tData;
} ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bTargetState;
} ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_SLAVE_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_SLAVE_TARGET_STATE_CNF_DATA_T tData;
} ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_SLAVE_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_SLAVE_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_SLAVE_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_SLAVE_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_REQ/ECM_IF_CMD_GET_SLAVE_CURRENT_STATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bCurrentState;
  uint8_t bTargetState;
  uint32_t ulActiveError;
} ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_DATA_T tData;
} ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_CURRENT_STATE_REQ_T tReq;
  ECM_IF_GET_SLAVE_CURRENT_STATE_CNF_T tCnf;
} ECM_IF_GET_SLAVE_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_MASTER_STATE_DIAG_REQ/ECM_IF_CMD_GET_MASTER_STATE_DIAG_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_MASTER_STATE_DIAG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_Ttag
{
  uint8_t bCurrentState;
  uint8_t abReserved[3];
  uint32_t ulCurrentActivity;
} ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_T;

/* ulCurrentActivity */
enum ECM_IF_ACTIVITY_Etag
{
  ECM_IF_ACTIVITY_IDLE = 0x00000000,
  ECM_IF_ACTIVITY_ACTIVE = 0x00000001,
  ECM_IF_ACTIVITY_BUS_SCAN = 0x00000100,
  ECM_IF_ACTIVITY_MANUAL = 0x00000200,
  ECM_IF_ACTIVITY_ERROR_DETECTED = 0x00000300,

  ECM_IF_ACTIVITY_WAIT_FOR_SLAVES = 0x00010000,
  ECM_IF_ACTIVITY_OPEN_PORTS = 0x00010001,

  ECM_IF_ACTIVITY_IDENTIFY_SLAVES = 0x00010100,
  ECM_IF_ACTIVITY_VERIFY_SLAVES = 0x00010200,
  ECM_IF_ACTIVITY_HOTCONNECT_DETECT = 0x00010300,

  ECM_IF_ACTIVITY_WAIT_FOR_TIMESYNC = 0x00010400,
  ECM_IF_ACTIVITY_WAIT_FOR_LINK = 0x00010500,

  ECM_IF_ACTIVITY_RESET_SLAVES = 0x00020000,

  ECM_IF_ACTIVITY_INIT_TO_PREOP = 0x00030000,
  ECM_IF_ACTIVITY_PREOP_TO_SAFEOP = 0x00030100,
  ECM_IF_ACTIVITY_SAFEOP_TO_OP = 0x00030200,
  ECM_IF_ACTIVITY_OP_TO_INIT = 0x00030300,
  ECM_IF_ACTIVITY_OP_TO_PREOP = 0x00030400,
  ECM_IF_ACTIVITY_OP_TO_SAFEOP = 0x00030500,
  ECM_IF_ACTIVITY_SAFEOP_TO_INIT = 0x0030600,
  ECM_IF_ACTIVITY_SAFEOP_TO_PREOP = 0x0030700,
  ECM_IF_ACTIVITY_PREOP_TO_INIT = 0x0030800,
  ECM_IF_ACTIVITY_TO_BUS_OFF = 0x0030900,

  ECM_IF_ACTIVITY_PS_RESET_DC = 0x00040000,
  ECM_IF_ACTIVITY_PS_WAIT_FOR_RX_LATCH_READY = 0x00040100,
  ECM_IF_ACTIVITY_PS_WAIT_FOR_READ_DC_SLAVE_RX_PORTS = 0x00040200,
  ECM_IF_ACTIVITY_PS_BEGIN_DC_SYNC = 0x00040300,
  ECM_IF_ACTIVITY_PS_WAIT_FOR_DC_SYNC = 0x00040400,

  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_RETRIGGER_SYNC = 0x00050000,
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_RX_LATCH_READY = 0x00050100,
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_READ_DC_SLAVE_RX_PORTS = 0x00050200,
  ECM_IF_ACTIVITY_RESYNC_BEGIN_DC_SYNC = 0x00050300,
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_DC_SYNC = 0x00050400,
  ECM_IF_ACTIVITY_RESYNC_WAIT_FOR_SLAVES_REACHING_STATE = 0x00050500,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_MASTER_STATE_DIAG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_STATE_DIAG_CNF_DATA_T tData;
} ECM_IF_GET_MASTER_STATE_DIAG_CNF_T;

/* packet union */
typedef union ECM_IF_GET_MASTER_STATE_DIAG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_MASTER_STATE_DIAG_REQ_T tReq;
  ECM_IF_GET_MASTER_STATE_DIAG_CNF_T tCnf;
} ECM_IF_GET_MASTER_STATE_DIAG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_EXT_START_BUSSCAN_REQ/ECM_IF_CMD_EXT_START_BUSSCAN_CNF
 *
 * to be used instead of HIL_BUSSCAN_REQ ulAction=HIL_BUSSCAN_CMD_START
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_EXT_START_BUSSCAN_REQ_DATA_Ttag
{
  uint32_t ulAction; /* always HIL_BUSSCAN_CMD_START */
  uint32_t ulBusScanFlags;
  uint16_t usVlanId; /* only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
  uint8_t bVlanPriority; /* only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
} ECM_IF_EXT_START_BUSSCAN_REQ_DATA_T;

enum ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_Etag
{
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_WITHOUT_PREOP = 0x00000001,
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG = 0x00000002,
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_ENABLE_VLAN_TAG = 0x00000004, /* only evaluated when MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_FLAGS_OVERRIDE_VLAN_CFG is set */
  MSK_ECM_IF_EXT_START_BUS_SCAN_REQ_SKIP_TIME_SYNC_LATCH = 0x00000008,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_EXT_START_BUSSCAN_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_EXT_START_BUSSCAN_REQ_DATA_T tData;
} ECM_IF_EXT_START_BUSSCAN_REQ_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_MANUAL_MODE_CONTROL_REQ/ECM_IF_CMD_MANUAL_MODE_CONTROL_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_Ttag
{
  uint32_t ulCmd;
  uint16_t usPortSelect;
  uint16_t usAdp;
  uint16_t usAdo;
  uint16_t usLength;
  uint16_t usReserved;
  uint8_t abData[1024]; /* actual byte length based on usPhysLength */
} ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_T;

/* ulCmd */
enum ECM_IF_MANUAL_MODE_CONTROL_CMD_Etag
{
  ECM_IF_MANUAL_MODE_CONTROL_CMD_STOP = 0,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_START = 1,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APRD = 0x0101,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APWR = 0x0102,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_APRW = 0x0103,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPRD = 0x0104,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPWR = 0x0105,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_FPRW = 0x0106,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BRD = 0x0107,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BWR = 0x0108,
  ECM_IF_MANUAL_MODE_CONTROL_CMD_BRW = 0x0109,
};

/* usPortSelect */
enum ECM_IF_MANUAL_MODE_REQUEST_PORT_SELECTOR_E
{
  ECM_IF_MANUAL_MODE_REQUEST_ALL_PORTS = 0,
  ECM_IF_MANUAL_MODE_REQUEST_PORT_0 = 1,
  ECM_IF_MANUAL_MODE_REQUEST_PORT_1 = 2
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MANUAL_MODE_CONTROL_REQ_DATA_T tData;
} ECM_IF_MANUAL_MODE_CONTROL_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_Ttag
{
  uint32_t ulCmd;
  uint16_t usPortReceived;
  uint16_t usAdp;
  uint16_t usAdo;
  uint16_t usLength;
  uint16_t usRetWkc;
  uint8_t abData[1024]; /* actual byte length based on usPhysLength */
} ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_T;

/* usPortReceived */
enum ECM_IF_MANUAL_MODE_REG_RECEIVE_PORT_Etag
{
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_NOT_DETERMINED = 0,
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_0 = 1,
  ECM_IF_MANUAL_MODE_RECEIVE_PORT_1 = 2,
  ECM_IF_MANUAL_MODE_RECEIVE_ALL_PORTS = 3
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANUAL_MODE_CONTROL_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MANUAL_MODE_CONTROL_CNF_DATA_T tData;
} ECM_IF_MANUAL_MODE_CONTROL_CNF_T;


/* packet union */
typedef union ECM_IF_MANUAL_MODE_CONTROL_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MANUAL_MODE_CONTROL_REQ_T tReq;
  ECM_IF_MANUAL_MODE_CONTROL_CNF_T tCnf;
} ECM_IF_MANUAL_MODE_CONTROL_PCK_T;


/**********************************************************************************************************************

 #####   #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#        #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/* usTransportType */
typedef enum ECM_IF_COE_TRANSPORT_TYPE_Etag
{
  ECM_IF_COE_TRANSPORT_COE = 0,//!< ECM_IF_COE_TRANSPORT_COE
  ECM_IF_COE_TRANSPORT_AOE = 1 //!< ECM_IF_COE_TRANSPORT_AOE
} ECM_IF_COE_TRANSPORT_TYPE_E;


#define ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ                   0x9A00 /* complete access is integrated here */
#define ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF                   0x9A01

#define ECM_IF_CMD_COE_SDO_UPLOAD_REQ                     0x9A02 /* complete access is integrated here */
#define ECM_IF_CMD_COE_SDO_UPLOAD_CNF                     0x9A03

#define ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ              0x9A04
#define ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF              0x9A05

#define ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ             0x9A06
#define ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF             0x9A07

#define ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ           0x9A08
#define ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF           0x9A09

/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDO_DOWNLOAD_REQ/ECM_IF_CMD_COE_SDO_DOWNLOAD_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t fCompleteAccess;
  uint32_t ulTotalBytes; /* has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs;
  uint8_t abData[1024];
} ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_DOWNLOAD_REQ_DATA_T tData;
} ECM_IF_COE_SDO_DOWNLOAD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t fCompleteAccess;
  uint32_t ulTotalBytes;
  uint32_t ulTimeoutMs;
} ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_DOWNLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_DOWNLOAD_CNF_DATA_T tData;
} ECM_IF_COE_SDO_DOWNLOAD_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDO_DOWNLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_DOWNLOAD_REQ_T tReq;
  ECM_IF_COE_SDO_DOWNLOAD_CNF_T tCnf;
} ECM_IF_COE_SDO_DOWNLOAD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDO_UPLOAD_REQ/ECM_IF_CMD_COE_SDO_UPLOAD_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t fCompleteAccess;
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_COE_SDO_UPLOAD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_UPLOAD_REQ_DATA_T tData;
} ECM_IF_COE_SDO_UPLOAD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t fCompleteAccess;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T, abData) */
} ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDO_UPLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_UPLOAD_CNF_DATA_T tData;
} ECM_IF_COE_SDO_UPLOAD_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDO_UPLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDO_UPLOAD_REQ_T tReq;
  ECM_IF_COE_SDO_UPLOAD_CNF_T tCnf;
} ECM_IF_COE_SDO_UPLOAD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETODLIST_REQ/ECM_IF_CMD_COE_SDOINFO_GETODLIST_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usListType;
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_T;

enum ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_Etag
{
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_COUNTS = 0,
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_ALL = 1,
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_RXPDOMAPPABLE = 2,
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_TXPDOMAPPABLE = 3,
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_BACKUP = 4,
  ECM_IF_COE_SDOINFO_GETODLIST_LIST_TYPE_SETTINGS = 5
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETODLIST_REQ_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETODLIST_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usListType;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint16_t ausObjectIDs[512]; /* actual byte length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETODLIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETODLIST_CNF_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETODLIST_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETODLIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETODLIST_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETODLIST_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETODLIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_REQ/ECM_IF_CMD_COE_SDOINFO_GETOBJDESC_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex;
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usObjIndex;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETOBJDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETOBJDESC_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETOBJDESC_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETOBJDESC_PCK_T;


/* format of abData */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_Ttag
{
  uint16_t usReserved;
  uint16_t usDataType;
  uint8_t bMaxSubIndex;
  uint8_t bObjectCode;
  uint8_t abName[256]; /* actual length is defined by ulTotalBytes - offsetof(ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_T, abName) */
} ECM_IF_COE_SDOINFO_GETOBJDESC_DATALAYOUT_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_REQ/ECM_IF_CMD_COE_SDOINFO_GETENTRYDESC_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

enum ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_Etag
{
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_UNIT_TYPE = 0x08,
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_DEFAULT_VALUE = 0x10,
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MINIMUM_VALUE = 0x20,
  MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MAXIMUM_VALUE = 0x40,
};

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_COE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_COE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t bRequestedValueInfo;
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usObjIndex;
  uint8_t bSubIndex;
  uint8_t bValueInfo;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T, abData) */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_DATA_T tData;
} ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_T;


/* packet union */
typedef union ECM_IF_COE_SDOINFO_GETENTRYDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_REQ_T tReq;
  ECM_IF_COE_SDOINFO_GETENTRYDESC_CNF_T tCnf;
} ECM_IF_COE_SDOINFO_GETENTRYDESC_PCK_T;

/* format of abData */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_Ttag
{
  uint32_t ulReserved;
  uint16_t usDataType;
  uint16_t usBitLength;
  uint16_t usObjAccess;
  uint8_t abData[1024]; /* actual length is defined by ulTotalBytes - offsetof(ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_T, abName) */
  /* order of data
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_UNIT_TYPE): uint32_t ulEcatUnit
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MINIMUM_VALUE): uint8_t abMinimumValue[usFieldSize]
   * (if bValueInfo & MSK_ECM_IF_COE_SDOINFO_GETENTRYDESC_VALUE_INFO_FLAGS_MAXIMUM_VALUE): uint8_t abMaximumValue[usFieldSize]
   * uint8_t abName[remainingbytes]
   */
} ECM_IF_COE_SDOINFO_GETENTRYDESC_DATALAYOUT_T;




/**********************************************************************************************************************

 #####   #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
 #####   #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
      #  #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/* usTransportType */
typedef enum ECM_IF_SOE_TRANSPORT_TYPE_Etag
{
  ECM_IF_SOE_TRANSPORT_SOE = 0,
  ECM_IF_SOE_TRANSPORT_AOE = 1
} ECM_IF_SOE_TRANSPORT_TYPE_E;


/* bElementFlags */
typedef enum ECM_IF_SOE_ELEMENT_FLAGS_Etag
{
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_DATASTATE = 0x01,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_NAME = 0x02,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_ATTRIBUTE = 0x04,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_UNIT = 0x08,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_MIN = 0x10,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_MAX = 0x20,
  MSK_ECM_IF_SOE_ELEMENT_FLAGS_VALUE = 0x40
} ECM_IF_SOE_ELEMENT_FLAGS_E;


#define ECM_IF_CMD_SOE_WRITE_REQ                          0x9B00
#define ECM_IF_CMD_SOE_WRITE_CNF                          0x9B01

#define ECM_IF_CMD_SOE_READ_REQ                           0x9B02
#define ECM_IF_CMD_SOE_READ_CNF                           0x9B03

#define ECM_IF_CMD_SOE_EXEC_PROCCMD_REQ                   0x9B04
#define ECM_IF_CMD_SOE_EXEC_PROCCMD_CNF                   0x9B05


/******************************************************************************
 * Packet: ECM_IF_CMD_SOE_WRITE_REQ/ECM_IF_CMD_SOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_SOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_SOE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usIDN;
  uint32_t ulTotalBytes; /* has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs;
  uint8_t bDriveNo;
  uint8_t bElementFlags; /* see ECM_IF_SOE_ELEMENT_FLAGS_E */
  uint8_t abData[1024];
} ECM_IF_SOE_WRITE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_WRITE_REQ_DATA_T tData;
} ECM_IF_SOE_WRITE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usIDN;
  uint32_t ulTotalBytes;
  uint32_t ulTimeoutMs;
  uint8_t bDriveNo;
  uint8_t bElementFlags;
} ECM_IF_SOE_WRITE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_WRITE_CNF_DATA_T tData;
} ECM_IF_SOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_SOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_WRITE_REQ_T tReq;
  ECM_IF_SOE_WRITE_CNF_T tCnf;
} ECM_IF_SOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SOE_READ_REQ/ECM_IF_CMD_SOE_READ_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_SOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_SOE_TRANSPORT_TYPE_AOE is selected */
  uint16_t usIDN;
  uint32_t ulTimeoutMs;
  uint8_t bDriveNo;
  uint8_t bElementFlags; /* see ECM_IF_SOE_ELEMENT_FLAGS_E */
  uint32_t ulMaxTotalBytes;
} ECM_IF_SOE_READ_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_READ_REQ_DATA_T tData;
} ECM_IF_SOE_READ_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint16_t usIDN;
  uint32_t ulTimeoutMs;
  uint8_t bDriveNo;
  uint8_t bElementFlags;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_SOE_READ_CNF_DATA_T, abData) */
} ECM_IF_SOE_READ_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_READ_CNF_DATA_T tData;
} ECM_IF_SOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_SOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SOE_READ_REQ_T tReq;
  ECM_IF_SOE_READ_CNF_T tCnf;
} ECM_IF_SOE_READ_PCK_T;


/**********************************************************************************************************************

#######  #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
#        #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#####    #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#        #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
#        #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

/* usTransportType */
typedef enum ECM_IF_FOE_TRANSPORT_TYPE_Etag
{
  ECM_IF_FOE_TRANSPORT_FOE = 0,
  ECM_IF_FOE_TRANSPORT_AOE = 1
} ECM_IF_FOE_TRANSPORT_TYPE_E;


#define ECM_IF_CMD_FOE_WRITE_REQ                          0x9900
#define ECM_IF_CMD_FOE_WRITE_CNF                          0x9901

#define ECM_IF_CMD_FOE_READ_REQ                           0x9902
#define ECM_IF_CMD_FOE_READ_CNF                           0x9903


#define ECM_IF_FOE_MAX_ERROR_TEXT_BYTE_LEN 256

/******************************************************************************
 * Packet: ECM_IF_CMD_FOE_WRITE_REQ/ECM_IF_CMD_FOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0 (data format as in tFirstSegData)
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_FIRST_Ttag
{
  /* structure for first fragment */
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTotalBytes; /* has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs;
  uint32_t ulPassword;
  uint32_t ulFileNameBytes; /* number of bytes used for file name including its NUL terminator */
  uint8_t abData[1024]; /* [0 - (ulFileNameBytes - 1)] is a NUL-terminated filename */
} ECM_IF_FOE_WRITE_REQ_DATA_FIRST_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_SEG_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTotalBytes; /* has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs;
  uint8_t abData[1024];
} ECM_IF_FOE_WRITE_REQ_DATA_SEG_T;

typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_DATA_Ttag
{
  ECM_IF_FOE_WRITE_REQ_DATA_FIRST_T tFirst;
  ECM_IF_FOE_WRITE_REQ_DATA_SEG_T tSeg;
} ECM_IF_FOE_WRITE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_WRITE_REQ_DATA_T tData;
} ECM_IF_FOE_WRITE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint32_t ulTotalBytes;
  uint32_t ulTimeoutMs;
  uint8_t abErrorText[ECM_IF_FOE_MAX_ERROR_TEXT_BYTE_LEN]; /* NUL-terminated error text, valid when ulSta != 0 */
} ECM_IF_FOE_WRITE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_WRITE_CNF_DATA_T tData;
} ECM_IF_FOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_FOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_WRITE_REQ_T tReq;
  ECM_IF_FOE_WRITE_CNF_T tCnf;
} ECM_IF_FOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_FOE_READ_REQ/ECM_IF_CMD_FOE_READ_CNF
 *
 * first fragment has ulDestId == 0 (data format as in tFirstSegData)
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_FIRST_Ttag
{
  /* structure for first fragment */
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
  uint32_t ulPassword;
  uint8_t abFileName[1024]; /* byte size defined by tHead.ulLen - offsetof(ECM_IF_FOE_READ_REQ_DATA_FIRST_T, tData) */
} ECM_IF_FOE_READ_REQ_DATA_FIRST_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_SEG_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType; /* see ECM_IF_FOE_TRANSPORT_TYPE_E */
  uint16_t usAoEPort; /* used when ECM_IF_FOE_TRANSPORT_TYPE_AOE is selected */
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_FOE_READ_REQ_DATA_SEG_T;

typedef __HIL_PACKED_PRE union __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_DATA_Ttag
{
  ECM_IF_FOE_READ_REQ_DATA_FIRST_T tFirst;
  ECM_IF_FOE_READ_REQ_DATA_SEG_T tSeg;
}ECM_IF_FOE_READ_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_READ_REQ_DATA_T tData;
} ECM_IF_FOE_READ_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usTransportType;
  uint16_t usAoEPort;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_FOE_READ_CNF_DATA_T, abData) */
  /* in case of ulSta != 0, abData contains a NUL-terminated error string */
} ECM_IF_FOE_READ_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_FOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_READ_CNF_DATA_T tData;
} ECM_IF_FOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_FOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_FOE_READ_REQ_T tReq;
  ECM_IF_FOE_READ_CNF_T tCnf;
} ECM_IF_FOE_READ_PCK_T;


/**********************************************************************************************************************

   #     #######  #######        ###  #     #  #######  #######  ######   #######     #      #####   #######
  # #    #     #  #               #   ##    #     #     #        #     #  #          # #    #     #  #
 #   #   #     #  #               #   # #   #     #     #        #     #  #         #   #   #        #
#     #  #     #  #####           #   #  #  #     #     #####    ######   #####    #     #  #        #####
#######  #     #  #               #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #               #   #    ##     #     #        #    #   #        #     #  #     #  #
#     #  #######  #######        ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */


typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_ADDRESS_Ttag
{
  uint8_t abNetId[6];
  uint16_t usPort;
} ECM_IF_AOE_ADDRESS_T;

/******************************************************************************/
#define ECM_IF_CMD_AOE_WRITE_REQ                          0x9D00
#define ECM_IF_CMD_AOE_WRITE_CNF                          0x9D01

#define ECM_IF_CMD_AOE_READ_REQ                           0x9D02
#define ECM_IF_CMD_AOE_READ_CNF                           0x9D03

#define ECM_IF_CMD_AOE_READWRITE_REQ                      0x9D04
#define ECM_IF_CMD_AOE_READWRITE_CNF                      0x9D05

/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_WRITE_REQ/ECM_IF_CMD_AOE_WRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTotalBytes; /* has to be set to summed length of all abData of all fragments */
  uint32_t ulTimeoutMs;
  uint8_t abData[1024];
} ECM_IF_AOE_WRITE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_WRITE_REQ_DATA_T tData;
} ECM_IF_AOE_WRITE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTotalBytes;
  uint32_t ulTimeoutMs;
} ECM_IF_AOE_WRITE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_WRITE_CNF_DATA_T tData;
} ECM_IF_AOE_WRITE_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_WRITE_REQ_T tReq;
  ECM_IF_AOE_WRITE_CNF_T tCnf;
} ECM_IF_AOE_WRITE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_READ_REQ/ECM_IF_CMD_AOE_READ_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTimeoutMs;
  uint32_t ulMaxTotalBytes;
} ECM_IF_AOE_READ_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READ_REQ_DATA_T tData;
} ECM_IF_AOE_READ_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalBytes; /* summed length of all abData confirmation fragments */
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_AOE_READ_CNF_DATA_T, abData) */
} ECM_IF_AOE_READ_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READ_CNF_DATA_T tData;
} ECM_IF_AOE_READ_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READ_REQ_T tReq;
  ECM_IF_AOE_READ_CNF_T tCnf;
} ECM_IF_AOE_READ_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_AOE_READWRITE_REQ/ECM_IF_CMD_AOE_READWRITE_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_REQ_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTotalWriteBytes; /* has to be set to summed length of all abData of all write fragments */
  uint32_t ulMaxTotalReadBytes;
  uint32_t ulTimeoutMs;
  uint8_t abData[1024];
} ECM_IF_AOE_READWRITE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READWRITE_REQ_DATA_T tData;
} ECM_IF_AOE_READWRITE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_CNF_DATA_Ttag
{
  ECM_IF_AOE_ADDRESS_T tTarget;
  uint32_t ulIndexGroup;
  uint32_t ulIndexOffset;
  uint32_t ulTotalWriteBytes;
  uint32_t ulTotalReadBytes; /* summed length of all abData confirmation fragments */
  uint32_t ulTimeoutMs;
  uint8_t abData[1024]; /* actual length is given by ulLen - offsetof(ECM_IF_AOE_READWRITE_CNF_DATA_T, abData) */
} ECM_IF_AOE_READWRITE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AOE_READWRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READWRITE_CNF_DATA_T tData;
} ECM_IF_AOE_READWRITE_CNF_T;


/* packet union */
typedef union ECM_IF_AOE_READWRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AOE_READWRITE_REQ_T tReq;
  ECM_IF_AOE_READWRITE_CNF_T tCnf;
} ECM_IF_AOE_READWRITE_PCK_T;


/**********************************************************************************************************************

 #####   #######     #     #######  #######        ###  #     #  ######   ###   #####      #     #######  ###  #######  #     #   #####
#     #     #       # #       #     #               #   ##    #  #     #   #   #     #    # #       #      #   #     #  ##    #  #     #
#           #      #   #      #     #               #   # #   #  #     #   #   #         #   #      #      #   #     #  # #   #  #
 #####      #     #     #     #     #####           #   #  #  #  #     #   #   #        #     #     #      #   #     #  #  #  #   #####
      #     #     #######     #     #               #   #   # #  #     #   #   #        #######     #      #   #     #  #   # #        #
#     #     #     #     #     #     #               #   #    ##  #     #   #   #     #  #     #     #      #   #     #  #    ##  #     #
 #####      #     #     #     #     #######        ###  #     #  ######   ###   #####   #     #     #     ###  #######  #     #   #####

 */

#define ECM_IF_CMD_MASTER_CURRENT_STATE_IND               0x9E10
#define ECM_IF_CMD_MASTER_CURRENT_STATE_RES               0x9E11

#define ECM_IF_CMD_SLAVE_CURRENT_STATE_IND                0x9E12
#define ECM_IF_CMD_SLAVE_CURRENT_STATE_RES                0x9E13

#define ECM_IF_CMD_REGISTER_APP_REQ                       0x9E1C
#define ECM_IF_CMD_REGISTER_APP_CNF                       0x9E1D

#define ECM_IF_CMD_UNREGISTER_APP_REQ                     0x9E1E
#define ECM_IF_CMD_UNREGISTER_APP_CNF                     0x9E1F


/******************************************************************************
 * Packet: ECM_IF_CMD_MASTER_CURRENT_STATE_IND/ECM_IF_CMD_MASTER_CURRENT_STATE_RES
 */

/* indication packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_IND_DATA_Ttag
{
  uint8_t bCurrentState;
  uint8_t bTargetState;
  uint32_t ulStopReason;
  uint32_t ulMasterStatusFlags;
} ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T;

typedef ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T ECM_IF_GET_MASTER_CURRENT_STATE_IND_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MASTER_CURRENT_STATE_IND_DATA_T tData;
} ECM_IF_MASTER_CURRENT_STATE_IND_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MASTER_CURRENT_STATE_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_MASTER_CURRENT_STATE_RES_T;


/* packet union */
typedef union ECM_IF_MASTER_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_MASTER_CURRENT_STATE_IND_T tInd;
  ECM_IF_MASTER_CURRENT_STATE_RES_T tRes;
} ECM_IF_MASTER_CURRENT_STATE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SLAVE_CURRENT_STATE_IND/ECM_IF_CMD_SLAVE_CURRENT_STATE_RES
 */

/* indication packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress;
  uint16_t usCurrentStatus;
  uint32_t ulLastError;
} ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T;

#define ECM_IF_SLAVE_CURRENT_STATE_IND_MAX_ENTRIES (HIL_MAX_DATA_SIZE / sizeof(ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_Ttag
{
  ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_ENTRY_T atEntries[ECM_IF_SLAVE_CURRENT_STATE_IND_MAX_ENTRIES];
} ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SLAVE_CURRENT_STATE_IND_DATA_T tData;
} ECM_IF_SLAVE_CURRENT_STATE_IND_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SLAVE_CURRENT_STATE_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_SLAVE_CURRENT_STATE_RES_T;


/* packet union */
typedef union ECM_IF_SLAVE_CURRENT_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SLAVE_CURRENT_STATE_IND_T tInd;
  ECM_IF_SLAVE_CURRENT_STATE_RES_T tRes;
} ECM_IF_SLAVE_CURRENT_STATE_PCK_T;


/**********************************************************************************************************************

######   ###     #      #####   #     #  #######   #####   #######  ###   #####    #####
#     #   #     # #    #     #  ##    #  #     #  #     #     #      #   #     #  #     #
#     #   #    #   #   #        # #   #  #     #  #           #      #   #        #
#     #   #   #     #  #  ####  #  #  #  #     #   #####      #      #   #         #####
#     #   #   #######  #     #  #   # #  #     #        #     #      #   #              #
#     #   #   #     #  #     #  #    ##  #     #  #     #     #      #   #     #  #     #
######   ###  #     #   #####   #     #  #######   #####      #     ###   #####    #####

 */

#define ECM_IF_CMD_GET_DC_DEVIATION_REQ 0x9E60
#define ECM_IF_CMD_GET_DC_DEVIATION_CNF 0x9E61

#define ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ 0x9E62
#define ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF 0x9E63

#define ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ 0x9E64
#define ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF 0x9E65

#define ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ 0x9E66
#define ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF 0x9E67

#define ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ 0x9E52
#define ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF 0x9E53

#define ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ 0x9E68
#define ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_CNF 0x9E69

#define ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ 0x9E6A
#define ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF 0x9E6B

#define ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ 0x9E6C
#define ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF 0x9E6D

#define ECM_IF_CMD_GET_ERROR_COUNTERS_REQ 0x9E6E
#define ECM_IF_CMD_GET_ERROR_COUNTERS_CNF 0x9E6F

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_DC_DEVIATION_REQ/ECM_IF_CMD_GET_DC_DEVIATION_CNF
 */

/* ulDcStatusFlags */
enum ECM_IF_DC_CONTROL_STATUS_FLAGS_Etag
{
  ECM_IF_DC_CONTROL_STATUS_INACTIVE = 0,

  /* actual flags */
  ECM_IF_DC_CONTROL_STATUS_ACTIVE = 1,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_DC_RX_STATUS_MAIN = 2,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_DC_RX_STATUS_RED = 4,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_BRD_ALSTATUS_WKC_MAIN = 8,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_EXPECTED_BRD_ALSTATUS_WKC_RED = 16,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_DL_STATUS_IRQ = 32,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_DC_ALL_PORTS_RX_STATUS_TIMEOUT = 64,
  ECM_IF_DC_CONTROL_STATUS_STOPPED_BY_INTERNAL_REQUEST = 128
};

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_DC_DEVIATION_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_CNF_DATA_Ttag
{
  uint32_t ulDcSlaveBrdDeviationSignMag;
  uint32_t ulDcBusDeviationSignMag;
  uint32_t ulDcLocalSysTimeDeviationSignMag;
  uint32_t ulDcStatusFlags;

  /* max values */
  uint32_t ulDcSlaveBrdDeviationMaxMag;
  uint32_t ulDcBusDeviationPosMaxMag;
  uint32_t ulDcBusDeviationNegMaxMag;
  uint32_t ulDcLocalSysTimeDeviationPosMaxMag;
  uint32_t ulDcLocalSysTimeDeviationNegMaxMag;
} ECM_IF_GET_DC_DEVIATION_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_DC_DEVIATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_DC_DEVIATION_CNF_DATA_T tData;
} ECM_IF_GET_DC_DEVIATION_CNF_T;


/* packet union */
typedef union ECM_IF_GET_DC_DEVIATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_DC_DEVIATION_REQ_T tReq;
  ECM_IF_GET_DC_DEVIATION_CNF_T tCnf;
} ECM_IF_GET_DC_DEVIATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_DC_INFO_REQ/ECM_IF_CMD_GET_SLAVE_DC_INFO_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_DC_INFO_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_DC_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usFlags;
  uint32_t ulDcSystimeDelayNs;
  uint64_t ullDcSystimeOffsetNs;
  uint64_t ullDcSyncShiftTimeNs;
  uint32_t ulDcCyc0Time;
  uint32_t ulDcCyc1Time;
  uint64_t ullRxLatchTime0Ns;
  uint32_t ulRxLatchTime1Ns;
  uint32_t ulRxLatchTime2Ns;
  uint32_t ulRxLatchTime3Ns;
  uint32_t ulPort1SumDelayNs;
  uint32_t ulPort2SumDelayNs;
  uint32_t ulPort3SumDelayNs;
  uint32_t ulTotalSumDelayNs;
  uint64_t ullDcSync0StartingDelayTimeNs;
  uint64_t ullDcResyncSystimeOffsetNs;
} ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_T;

/* usFlags */
enum ECM_IF_GET_SLAVE_DC_INFO_FLAGS_Etag
{
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_IN_TOPOLOGY = 0x0001,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_TIME_CONFIGURED = 0x0002, /* DC synchronization (DcSystimeOffset, DcSystimeDelay) setup done */
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_SYNC_CONFIGURED = 0x0004,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_IS_64BIT = 0x0008,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_IS_SUPPORTED = 0x0010,

  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE = 0x0100,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE_SYNC0 = 0x0200,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_DC_ACTIVATE_SYNC1 = 0x0400,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_RX_TIMESTAMP_LATCH_SUPPORTED = 0x0800,

  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT0_EXISTS = 0x1000,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT1_EXISTS = 0x2000,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT2_EXISTS = 0x4000,
  ECM_IF_GET_SLAVE_DC_INFO_FLAGS_PORT3_EXISTS = 0x8000
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_DC_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_DC_INFO_CNF_DATA_T tData;
} ECM_IF_GET_SLAVE_DC_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_DC_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_DC_INFO_REQ_T tReq;
  ECM_IF_GET_SLAVE_DC_INFO_CNF_T tCnf;
} ECM_IF_GET_SLAVE_DC_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_REQ/ECM_IF_CMD_RESET_DC_MAX_DEVIATIONS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_T;


/* packet union */
typedef union ECM_IF_RESET_DC_MAX_DEVIATIONS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_RESET_DC_MAX_DEVIATIONS_REQ_T tReq;
  ECM_IF_RESET_DC_MAX_DEVIATIONS_CNF_T tCnf;
} ECM_IF_RESET_DC_MAX_DEVIATIONS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_EXT_SYNC_INFO_REQ/ECM_IF_CMD_GET_EXT_SYNC_INFO_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_EXT_SYNC_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_Ttag
{
  uint32_t ulExtSyncInfoFlags;
  uint64_t ullInternalTimestampNs;
  uint64_t ullExternalTimestampNs;
  int32_t lTimeControlValueBySlave;
  uint16_t usExtSyncStationAddress;
  uint64_t ullDcToExtTimeOffsetNs; /* internal DC timestamp (ns) + ullDcToExtTimeOffsetNs => external clock time (ns) */
  uint32_t ulLastUpdateDiffNs;
  int32_t lLastControlDeltaDiffNs;
  int32_t lLastControlDeltaDeltaDiffNs;
  uint16_t usControlledStationAddress;
  uint32_t ulExtSyncUpdateCount;
  uint32_t ulDeviationPosMaxMag;
  uint32_t ulDeviationNegMaxMag;
} ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_T;

/* ulExtSyncInfoFlags */
enum ECM_IF_EXT_SYNC_INFO_FLAGS_Etag
{
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_SYNC_MODE_SLAVE = 0x00000001,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_IS_64BIT = 0x00000004,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_NOT_CONNECTED = 0x00000010,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_SYNC_MODE_MASTER = 0x00000020,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONNECTED_AS_SLAVE = 0x00008000, /* result of !(External Device Not Connected) && (Sync Mode.Bit 1) */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 0x00FF0000,
  SRT_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_SYNC_CONTROL_STATE = 16,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_DC_TO_EXT_OFFSET_VALID = 0x20000000,
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_ACTIVE = 0x40000000, /* master is actively using External Synchronization on device */
  MSK_ECM_IF_EXT_SYNC_INFO_FLAGS_EXT_DEVICE_CONFIGURED = 0x80000000
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_EXT_SYNC_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_EXT_SYNC_INFO_CNF_DATA_T tData;
} ECM_IF_GET_EXT_SYNC_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_EXT_SYNC_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_EXT_SYNC_INFO_REQ_T tReq;
  ECM_IF_GET_EXT_SYNC_INFO_CNF_T tCnf;
} ECM_IF_GET_EXT_SYNC_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ/ECM_IF_CMD_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_T;


/* packet union */
typedef union ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_REQ_T tReq;
  ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_CNF_T tCnf;
} ECM_IF_RESET_EXT_SYNC_MAX_DEVIATIONS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_REQ/ECM_IF_CMD_GET_SLAVE_HANDLE_BIT_LIST_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_Ttag
{
  uint32_t ulListType; /* same enum as in RCX_GET_SLAVE_HANDLES_REQ */
  uint32_t ulStartHandle; /* first bit position in confirmation refers to the handle ulStartHandle */
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_Ttag
{
  uint32_t ulListType; /* same enum as in RCX_GET_SLAVE_HANDLES_REQ */
  uint32_t ulStartHandle;
  uint32_t ulNumHandleBits;
  uint8_t abBitMap[1024];
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_DATA_T tData;
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_REQ_T tReq;
  ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_CNF_T tCnf;
} ECM_IF_GET_SLAVE_HANDLE_BIT_LIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_REQ/ECM_IF_CMD_GET_FRAME_LOSS_COUNTERS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetAfterRead;
} ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_DATA_T tData;
} ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_Ttag
{
  uint32_t ulMainPortCount;
  uint32_t ulRedPortCount;
} ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_T;

#define ECM_IF_GET_FRAME_LOSS_COUNTERS_MAX_NUM_ENTRIES 8
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_Ttag
{
  uint32_t fRedundancyEnabled;
  ECM_IF_GET_FRAME_LOSS_COUNTER_ENTRY_T atEntries[ECM_IF_GET_FRAME_LOSS_COUNTERS_MAX_NUM_ENTRIES]; /* has to be last entry */
} ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_DATA_T tData;
} ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_FRAME_LOSS_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_REQ_T tReq;
  ECM_IF_GET_FRAME_LOSS_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_FRAME_LOSS_COUNTERS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_THRESHOLD_COUNTERS_REQ/ECM_IF_CMD_GET_THRESHOLD_COUNTERS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetOutOfRxWindowCumulativeCntAfterRead;
  uint32_t fResetFrameLostCumulativeCntAfterRead;
} ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T;

typedef ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T ECM_IF_GET_THRESHOLD_COUNTERS_DATA_REQ_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_THRESHOLD_COUNTERS_REQ_DATA_T tData;
} ECM_IF_GET_THRESHOLD_COUNTERS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_THRESHOLD_COUNTER_ENTRY_Ttag
{
  uint32_t ulThresholdCnt;
  uint32_t ulThreshold;
  uint32_t ulCumulativeCnt;
} ECM_IF_THRESHOLD_COUNTER_ENTRY_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_Ttag
{
  ECM_IF_THRESHOLD_COUNTER_ENTRY_T tOutOfRxWindow;
  ECM_IF_THRESHOLD_COUNTER_ENTRY_T tFrameLost;
} ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_THRESHOLD_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_THRESHOLD_COUNTERS_CNF_DATA_T tData;
} ECM_IF_GET_THRESHOLD_COUNTERS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_THRESHOLD_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_THRESHOLD_COUNTERS_REQ_T tReq;
  ECM_IF_GET_THRESHOLD_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_THRESHOLD_COUNTERS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_ERROR_COUNTERS_REQ/ECM_IF_CMD_GET_ERROR_COUNTERS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_Ttag
{
  uint32_t fResetAfterRead;
} ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_ERROR_COUNTERS_REQ_DATA_T tData;
} ECM_IF_GET_ERROR_COUNTERS_REQ_T;

/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_Ttag
{
  uint32_t fValid;
  uint32_t ulTransmittedOk;
  uint32_t ulLinkDownDuringTransmission;
  uint32_t ulUtxUnderflowDuringTransmission;
  uint32_t ulFramesReceivedOk;
  uint32_t ulFcsErrors;
  uint32_t ulAlignmentErrors;
  uint32_t ulFrameTooLongErrors;
  uint32_t ulRuntFramesReceived;
  uint32_t ulCollisionFragmentsReceived;
  uint32_t ulDroppedDueLowResource;
  uint32_t ulDroppedDueUrxOverflow;
  uint32_t ulRxFatalError;
} ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_Ttag
{
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T tMainPort;
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_PORT_T tRedPort;
} ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_ERROR_COUNTERS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_ERROR_COUNTERS_CNF_DATA_T tData;
} ECM_IF_GET_ERROR_COUNTERS_CNF_T;

/* packet union */
typedef union ECM_IF_GET_ERROR_COUNTERS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_ERROR_COUNTERS_REQ_T tReq;
  ECM_IF_GET_ERROR_COUNTERS_CNF_T tCnf;
} ECM_IF_GET_ERROR_COUNTERS_PCK_T;

/**********************************************************************************************************************/


#define ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ 0x9E58
#define ECM_IF_CMD_SELECT_SYNC_CONFIG_CNF 0x9E59

#define ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ 0x9E5A
#define ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF 0x9E5B

/******************************************************************************
 * Packet: ECM_IF_CMD_SELECT_SYNC_CONFIG_REQ/ECM_IF_CMD_SELECT_SYNC_CONFIG_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_Ttag
{
  uint32_t ulSyncModuleId;
  uint32_t ulTimeSyncModuleId;
  uint16_t usSlaveAddress;
  uint32_t aulSyncModuleParameters[10]; /* depends on ulSyncModuleTypeId */
  uint32_t aulTimeSyncModuleParameters[10]; /* depends on ulTimeSyncModuleTypeId */
} ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SELECT_SYNC_CONFIG_REQ_DATA_T tData;
} ECM_IF_SELECT_SYNC_CONFIG_REQ_T;

/* special value for usSlaveAddress */
#define ECM_IF_SELECT_FIRST_EXTSYNC_SLAVE 0xFFFF

/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SELECT_SYNC_CONFIG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_SELECT_SYNC_CONFIG_CNF_T;


/* packet union */
typedef union ECM_IF_SELECT_SYNC_CONFIG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SELECT_SYNC_CONFIG_REQ_T tReq;
  ECM_IF_SELECT_SYNC_CONFIG_CNF_T tCnf;
} ECM_IF_SELECT_SYNC_CONFIG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ENUM_SYNC_CONFIG_REQ/ECM_IF_CMD_ENUM_SYNC_CONFIG_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_Ttag
{
  uint32_t ulIdx;
} ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ENUM_SYNC_CONFIG_REQ_DATA_T tData;
} ECM_IF_ENUM_SYNC_CONFIG_REQ_T;

/* special values for ulIdx */
#define ECM_IF_ENUM_SYNC_ACTIVE_SYNC_MODULE 0xFFFFFFFF
#define ECM_IF_ENUM_SYNC_ACTIVE_TIMESYNC_MODULE 0xFFFFFFFE


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_Ttag
{
  uint32_t ulIdx;
  uint32_t ulEntryType;
  uint32_t ulModuleId;
  uint32_t aulExcludes[(HIL_MAX_DATA_SIZE - sizeof(uint16_t) * 5) / sizeof(uint32_t)];
} ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_T;

/* ulEntryType */
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_SYNC 0
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_TIMESYNC 1
#define ECM_IF_ENUM_SYNC_CONFIG_TYPE_EXTSYNC 2 /* ulModuleId is slave address */

/* ulModuleId for ECM_IF_ENUM_SYNC_CONFIG_TYPE_SYNC */
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_DEACTIVATED 0
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_RXEND_POSEDGE 1
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_RXEND_POSEDGE 2
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_CYCLESTART_POSEDGE 3
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_CYCLESTART_POSEDGE 4
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC0_CYCLESTART_NEGEDGE 5
#define ECM_IF_ENUM_SYNC_CONFIG_SYNC_TYPE_SYNC1_CYCLESTART_NEGEDGE 6

/* ulModuleId for ECM_IF_ENUM_SYNC_CONFIG_TYPE_TIMESYNC */
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_DEACTIVATED 0
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC0_LATCH_POSEDGE 1
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC1_LATCH_POSEDGE 2
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC0_LATCH_NEGEDGE 3
#define ECM_IF_ENUM_SYNC_CONFIG_TIMESYNC_TYPE_SYNC1_LATCH_NEGEDGE 4

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ENUM_SYNC_CONFIG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ENUM_SYNC_CONFIG_CNF_DATA_T tData;
} ECM_IF_ENUM_SYNC_CONFIG_CNF_T;


/* packet union */
typedef union ECM_IF_ENUM_SYNC_CONFIG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ENUM_SYNC_CONFIG_REQ_T tReq;
  ECM_IF_ENUM_SYNC_CONFIG_CNF_T tCnf;
} ECM_IF_ENUM_SYNC_CONFIG_PCK_T;


/**********************************************************************************************************************

#######  #######  ######   #######  #        #######   #####   #     #        ######   #######     #     ######   #######  #     #  #######
   #     #     #  #     #  #     #  #        #     #  #     #   #   #         #     #  #          # #    #     #  #     #  #     #     #
   #     #     #  #     #  #     #  #        #     #  #          # #          #     #  #         #   #   #     #  #     #  #     #     #
   #     #     #  ######   #     #  #        #     #  #  ####     #           ######   #####    #     #  #     #  #     #  #     #     #
   #     #     #  #        #     #  #        #     #  #     #     #           #   #    #        #######  #     #  #     #  #     #     #
   #     #     #  #        #     #  #        #     #  #     #     #           #    #   #        #     #  #     #  #     #  #     #     #
   #     #######  #        #######  #######  #######   #####      #           #     #  #######  #     #  ######   #######   #####      #

 */

#define ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ 0x9E50
#define ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF 0x9E51

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_TOPOLOGY_INFO_REQ/ECM_IF_CMD_GET_TOPOLOGY_INFO_CNF
 *
 * first fragment has ulDestId == 0
 * stack returns first fragment confirmation with ulDestId != 0
 * that ulDestId has to be provided to all subsequence fragments
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_TOPOLOGY_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_Ttag
{
  uint16_t usThisStationAddress;
  uint16_t ausPortConnectedTo[4]; /* 0xFFFF = NOT CONNECTED, 0 = CONNECTED TO MASTER */
  /* Entries in order of auto-increment position */
} ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T;

#define ECM_IF_GET_TOPOLOGY_INFO_MAX_ENTRIES (1024 / sizeof(ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T))

#define ECM_IF_TOPOLOGY_INFO_PORT_NOT_CONNECTED 65535

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_Ttag
{
  uint32_t ulTotalNumOfListEntries;
  uint32_t ulStartOfUnconnectedListEntries;
  ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_ENTRY_T atEntries[ECM_IF_GET_TOPOLOGY_INFO_MAX_ENTRIES];
} ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TOPOLOGY_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TOPOLOGY_INFO_CNF_DATA_T tData;
} ECM_IF_GET_TOPOLOGY_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_TOPOLOGY_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TOPOLOGY_INFO_REQ_T tReq;
  ECM_IF_GET_TOPOLOGY_INFO_CNF_T tCnf;
} ECM_IF_GET_TOPOLOGY_INFO_PCK_T;


/**********************************************************************************************************************

 #####   #######  #     #  #######  ###   #####         ######   #######     #     ######   #######  #     #  #######
#     #  #     #  ##    #  #         #   #     #        #     #  #          # #    #     #  #     #  #     #     #
#        #     #  # #   #  #         #   #              #     #  #         #   #   #     #  #     #  #     #     #
#        #     #  #  #  #  #####     #   #  ####        ######   #####    #     #  #     #  #     #  #     #     #
#        #     #  #   # #  #         #   #     #        #   #    #        #######  #     #  #     #  #     #     #
#     #  #     #  #    ##  #         #   #     #        #    #   #        #     #  #     #  #     #  #     #     #
 #####   #######  #     #  #        ###   #####         #     #  #######  #     #  ######   #######   #####      #

 */

#define ECM_IF_CMD_GET_TIMING_INFO_REQ 0x9E20
#define ECM_IF_CMD_GET_TIMING_INFO_CNF 0x9E21

#define ECM_IF_CMD_GET_WC_STATE_INFO_REQ 0x9E22
#define ECM_IF_CMD_GET_WC_STATE_INFO_CNF 0x9E23

#define ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ 0x9E24
#define ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF 0x9E25

#define ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ 0x9E26
#define ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF 0x9E27

#define ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ 0x9E28
#define ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF 0x9E29

#define ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ 0x9E2A
#define ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF 0x9E2B

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_TIMING_INFO_REQ/ECM_IF_CMD_GET_TIMING_INFO_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_TIMING_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_CNF_DATA_Ttag
{
  uint32_t ulBusCycleTimeNs;
  uint32_t ulFrameTransmitTimeNs;
  uint32_t ulExpectedBusDelayNs;
  uint32_t ulExpectedRxEndTimeNs; /* from start of bus cycle transmission */
  uint32_t ulExpectedTxDataTimeNs; /* from start of bus cycle transmission */
} ECM_IF_GET_TIMING_INFO_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_TIMING_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TIMING_INFO_CNF_DATA_T tData;
} ECM_IF_GET_TIMING_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_TIMING_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_TIMING_INFO_REQ_T tReq;
  ECM_IF_GET_TIMING_INFO_CNF_T tCnf;
} ECM_IF_GET_TIMING_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_WC_STATE_INFO_REQ/ECM_IF_CMD_GET_WC_STATE_INFO_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
} ECM_IF_GET_WCSTATE_INFO_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_WCSTATE_INFO_REQ_DATA_T tData;
} ECM_IF_GET_WCSTATE_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_WCSTATE_INFO_ENTRY_Ttag
{
  uint32_t ulWcStateBitPosition;
  uint16_t usTxImageStartByteOffset;
  uint16_t usRxImageStartByteOffset;
  uint16_t usImageByteLength;
  uint16_t usDirection;
} ECM_IF_WCSTATE_INFO_ENTRY_T;

enum ECM_IF_WCSTATE_DIRECTION_Etag
{
  MSK_ECM_IF_WCSTATE_INFO_DIRECTION_TXDATA = 0x0001,
  MSK_ECM_IF_WCSTATE_INFO_DIRECTION_RXDATA = 0x0002,
};

#define ECM_IF_MAX_WCSTATE_INFO_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
  uint32_t ulTotalEntries; /* this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_WCSTATE_INFO_ENTRY_T atEntries[ECM_IF_MAX_WCSTATE_INFO_ENTRIES];
} ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_WCSTATE_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T tData;
} ECM_IF_GET_WCSTATE_INFO_CNF_T;


/* packet union */
typedef union ECM_IF_GET_WCSTATE_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_WCSTATE_INFO_REQ_T tReq;
  ECM_IF_GET_WCSTATE_INFO_CNF_T tCnf;
} ECM_IF_GET_WCSTATE_INFO_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_REQ/ECM_IF_CMD_GET_CYCLIC_CMD_MAPPING_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
} ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_DATA_T tData;
} ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_Ttag
{
  uint16_t usTransmitType;
  uint16_t usReceiveType;
  uint16_t usTxImageStartByteOffset;
  uint16_t usRxImageStartByteOffset;
  uint16_t usImageByteLength;
  uint16_t usWkcCompareReceiveByteOffset; /* only valid if not set to 0xFFFF */
} ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T;

/* usTransmitType / usReceiveType */
enum ECM_IF_CYCLIC_CMD_DATATYPE_Etag
{
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_UNUSED = 0,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_PROCESS_DATA = 1,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_DC_SYSTIME = 2,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_ALSTATUS = 3,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_BRD_DC_SYSTIME_DIFF = 4,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_WCSTATE_BITS = 5,
  VAL_ECM_IF_CYCLIC_CMD_DATATYPE_EXTSYNC_STATUS = 6
};

#define ECM_IF_MAX_CYCLIC_CMD_MAPPING_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
  uint32_t ulTotalEntries; /* this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_CYCLIC_CMD_MAPPING_ENTRY_T atEntries[ECM_IF_MAX_CYCLIC_CMD_MAPPING_ENTRIES];
} ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_DATA_T tData;
} ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_T;


/* packet union */
typedef union ECM_IF_GET_CYCLIC_CMD_MAPPING_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_REQ_T tReq;
  ECM_IF_GET_CYCLIC_CMD_MAPPING_CNF_T tCnf;
} ECM_IF_GET_CYCLIC_CMD_MAPPING_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_REQ/ECM_IF_CMD_GET_CYCLIC_SLAVE_MAPPING_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_DATA_T tData;
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_Ttag
{
  uint16_t usDirection;
  uint16_t usStationAddress;
  uint16_t usWkcCompareReceiveByteOffset; /* only valid if not set to 0xFFFF */
  uint32_t ulWcStateBitOffset; /* only valid if not set to 0xFFFFFFFF */
  uint32_t ulImageStartBitOffset;
  uint32_t ulImageBitLength;

  uint32_t ulBitOffsetWithin; /* when bSmNo == 0xFF and bFmmmuNo = 0xFF, we provide register address multiplied by 8 */
  uint8_t bSmNo; /* 0xFF not set */
  uint8_t bFmmuNo; /* 0xFF not set */

  uint16_t usReserved;
} ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T;

/* usDirection */
enum ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_Etag
{
  VAL_ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_TRANSMIT = 1,
  VAL_ECM_IF_CYCLIC_SLAVE_MAPPING_TYPE_RECEIVE = 2
};

#define ECM_IF_MAX_CYCLIC_SLAVE_MAPPING_ENTRIES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t) * 2) / sizeof(ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_Ttag
{
  uint32_t ulEntriesStartOffset;
  uint32_t ulTotalEntries; /* this number can be larger than the atEntries field can hold */

  /* actual number of entries given by (ptPck->tHead.ulLen - offsetof(ECM_IF_GET_WCSTATE_INFO_CNF_DATA_T, atEntries)) / sizeof(ECM_IF_WCSTATE_INFO_ENTRY_T) */
  ECM_IF_CYCLIC_SLAVE_MAPPING_ENTRY_T atEntries[ECM_IF_MAX_CYCLIC_SLAVE_MAPPING_ENTRIES];
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_DATA_T tData;
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_T;


/* packet union */
typedef union ECM_IF_GET_CYCLIC_SLAVE_MAPPING_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_REQ_T tReq;
  ECM_IF_GET_CYCLIC_SLAVE_MAPPING_CNF_T tCnf;
} ECM_IF_GET_CYCLIC_SLAVE_MAPPING_PCK_T;

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_REQ/ECM_IF_CMD_GET_SLAVE_MBX_PROTOCOLS_CNF
 */

#define ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES 128

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress;
  uint16_t usMbxProtocols; /* filled in on confirmation */
  uint32_t ulReserved;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_Ttag
{
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES];
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_Ttag
{
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_MBX_PROTOCOLS_ENTRIES];
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_DATA_T tData;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_MBX_PROTOCOLS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_REQ_T tReq;
  ECM_IF_GET_SLAVE_MBX_PROTOCOLS_CNF_T tCnf;
} ECM_IF_GET_SLAVE_MBX_PROTOCOLS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_REQ/ECM_IF_CMD_GET_SLAVE_AOE_ADDRESSES_CNF
 */

#define ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES 128

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_Ttag
{
  uint16_t usStationAddress;
  uint8_t abNetId[6]; /* filled in on confirmation (SBZ if no aoe support) */
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_Ttag
{
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES];
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_DATA_T tData;
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_Ttag
{
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_DATA_ENTRY_T atEntries[ECM_MAX_NUM_OF_AOE_ADDRESSES_ENTRIES];
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_DATA_T tData;
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_T;


/* packet union */
typedef union ECM_IF_GET_SLAVE_AOE_ADDRESSES_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_REQ_T tReq;
  ECM_IF_GET_SLAVE_AOE_ADDRESSES_CNF_T tCnf;
} ECM_IF_GET_SLAVE_AOE_ADDRESSES_PCK_T;


/**********************************************************************************************************************

######   ###     #      #####   #     #  #######   #####   #######  ###   #####         #        #######   #####
#     #   #     # #    #     #  ##    #  #     #  #     #     #      #   #     #        #        #     #  #     #
#     #   #    #   #   #        # #   #  #     #  #           #      #   #              #        #     #  #
#     #   #   #     #  #  ####  #  #  #  #     #   #####      #      #   #              #        #     #  #  ####
#     #   #   #######  #     #  #   # #  #     #        #     #      #   #              #        #     #  #     #
#     #   #   #     #  #     #  #    ##  #     #  #     #     #      #   #     #        #        #     #  #     #
######   ###  #     #   #####   #     #  #######   #####      #     ###   #####         #######  #######   #####

 */

/* 9E30-9E4F */

#define ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ 0x9E30
#define ECM_IF_CMD_READ_DIAG_LOG_ENTRY_CNF 0x9E31

#define ECM_IF_CMD_CLEAR_DIAG_LOG_REQ 0x9E32
#define ECM_IF_CMD_CLEAR_DIAG_LOG_CNF 0x9E33

#define ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND 0x9E34
#define ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES 0x9E35

#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ 0x9E36
#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF 0x9E37

#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ 0x9E38
#define ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF 0x9E39

/******************************************************************************
 * Packet: ECM_IF_CMD_READ_DIAG_LOG_ENTRY_REQ/ECM_IF_CMD_READ_DIAG_LOG_ENTRY_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_REQ_Ttag
{
  HIL_PACKET_HEADER_T                               tHead;
} ECM_IF_READ_DIAG_LOG_ENTRY_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_Ttag
{
  uint32_t ulLostEntries;
  ECM_DIAG_ENTRY_T tDiagEntry;
} ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_READ_DIAG_LOG_ENTRY_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_DIAG_LOG_ENTRY_CNF_DATA_T tData;
} ECM_IF_READ_DIAG_LOG_ENTRY_CNF_T;


/* packet union */
typedef union ECM_IF_READ_DIAG_LOG_ENTRY_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_READ_DIAG_LOG_ENTRY_REQ_T tReq;
  ECM_IF_READ_DIAG_LOG_ENTRY_CNF_T tCnf;
} ECM_IF_READ_DIAG_LOG_ENTRY_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_CLEAR_DIAG_LOG_REQ/ECM_IF_CMD_CLEAR_DIAG_LOG_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CLEAR_DIAG_LOG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_CLEAR_DIAG_LOG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_CLEAR_DIAG_LOG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_CLEAR_DIAG_LOG_CNF_T;


/* packet union */
typedef union ECM_IF_CLEAR_DIAG_LOG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_CLEAR_DIAG_LOG_REQ_T tReq;
  ECM_IF_CLEAR_DIAG_LOG_CNF_T tCnf;
} ECM_IF_CLEAR_DIAG_LOG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_IND/ECM_IF_CMD_NEW_DIAG_LOG_ENTRIES_RES
 */

/* indication packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_Ttag
{
  uint16_t usNumOfDiagEntries;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_DATA_T tData;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_T;


/* response packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_T;


/* packet union */
typedef union ECM_IF_NEW_DIAG_LOG_ENTRIES_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_NEW_DIAG_LOG_ENTRIES_IND_T tInd;
  ECM_IF_NEW_DIAG_LOG_ENTRIES_RES_T tRes;
} ECM_IF_NEW_DIAG_LOG_ENTRIES_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_REQ/ECM_IF_CMD_DIAG_LOG_INDICATIONS_REGISTER_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_Ttag
{
  uint16_t usReserved;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_DATA_T tData;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_T;


/* packet union */
typedef union ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_REQ_T tReq;
  ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_CNF_T tCnf;
} ECM_IF_DIAG_LOG_INDICATIONS_REGISTER_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_REQ/ECM_IF_CMD_DIAG_LOG_INDICATIONS_UNREGISTER_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_T;


/* packet union */
typedef union ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_REQ_T tReq;
  ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_CNF_T tCnf;
} ECM_IF_DIAG_LOG_INDICATIONS_UNREGISTER_PCK_T;


/**********************************************************************************************************************

 #####   #     #  ######   ######   #######  ######   #######  #######  ######            #     ######   ###   #####
#     #  #     #  #     #  #     #  #     #  #     #     #     #        #     #          # #    #     #   #   #     #
#        #     #  #     #  #     #  #     #  #     #     #     #        #     #         #   #   #     #   #   #
 #####   #     #  ######   ######   #     #  ######      #     #####    #     #        #     #  ######    #    #####
      #  #     #  #        #        #     #  #   #       #     #        #     #        #######  #         #         #
#     #  #     #  #        #        #     #  #    #      #     #        #     #        #     #  #         #   #     #
 #####    #####   #        #        #######  #     #     #     #######  ######         #     #  #        ###   #####

 */

#define ECM_IF_CMD_GET_SUPPORTED_APIS_REQ 0x9E90
#define ECM_IF_CMD_GET_SUPPORTED_APIS_CNF 0x9E91

/******************************************************************************
 * Packet: ECM_IF_CMD_GET_SUPPORTED_APIS_REQ/ECM_IF_CMD_GET_SUPPORTED_APIS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_GET_SUPPORTED_APIS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_Ttag
{
  uint32_t aulSupportedApis[256];
} ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_GET_SUPPORTED_APIS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SUPPORTED_APIS_CNF_DATA_T tData;
} ECM_IF_GET_SUPPORTED_APIS_CNF_T;


enum ECM_IF_GET_SUPPORTED_APIS_CODING_Etag
{
  MSK_ECM_IF_SUPPORTED_API_TYPE = 0xFFFF0000,
  MSK_ECM_IF_SUPPORTED_API_VERSION = 0x0000FFFF,

  VAL_ECM_IF_SUPPORTED_API_TYPE_ECMV4 = 0x00010000, /* always used with version zero */
  VAL_ECM_IF_SUPPORTED_API_TYPE_MANUAL_MODE = 0x00020000,
};

/* packet union */
typedef union ECM_IF_GET_SUPPORTED_APIS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_GET_SUPPORTED_APIS_REQ_T tReq;
  ECM_IF_GET_SUPPORTED_APIS_CNF_T tCnf;
} ECM_IF_GET_SUPPORTED_APIS_PCK_T;


/**********************************************************************************************************************

 #####   #######  #     #  #######  ###   #####         ###  #     #  #######  #######  ######   #######     #      #####   #######
#     #  #     #  ##    #  #         #   #     #         #   ##    #     #     #        #     #  #          # #    #     #  #
#        #     #  # #   #  #         #   #               #   # #   #     #     #        #     #  #         #   #   #        #
#        #     #  #  #  #  #####     #   #  ####         #   #  #  #     #     #####    ######   #####    #     #  #        #####
#        #     #  #   # #  #         #   #     #         #   #   # #     #     #        #   #    #        #######  #        #
#     #  #     #  #    ##  #         #   #     #         #   #    ##     #     #        #    #   #        #     #  #     #  #
 #####   #######  #     #  #        ###   #####         ###  #     #     #     #######  #     #  #        #     #   #####   #######

 */

#define ECM_IF_CMD_BEGIN_CONFIGURATION_REQ 0x9EA0
#define ECM_IF_CMD_BEGIN_CONFIGURATION_CNF 0x9EA1

#define ECM_IF_CMD_END_CONFIGURATION_REQ 0x9EA2
#define ECM_IF_CMD_END_CONFIGURATION_CNF 0x9EA3

#define ECM_IF_CMD_ABORT_CONFIGURATION_REQ 0x9EA4
#define ECM_IF_CMD_ABORT_CONFIGURATION_CNF 0x9EA5

#define ECM_IF_CMD_LOAD_ENI_REQ 0x9EA6
#define ECM_IF_CMD_LOAD_ENI_CNF 0x9EA7

#define ECM_IF_CMD_ADD_SLAVE_REQ 0x9EA8
#define ECM_IF_CMD_ADD_SLAVE_CNF 0x9EA9

#define ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ 0x9EAA
#define ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF 0x9EAB

#define ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_REQ 0x9EAC
#define ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_CNF 0x9EAD

#define ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ 0x9EAE
#define ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF 0x9EAF

#define ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ 0x9EB0
#define ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF 0x9EB1

#define ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ 0x9EB2
#define ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_CNF 0x9EB3

#define ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ 0x9EB4
#define ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF 0x9EB5

#define ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ 0x9EB6
#define ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF 0x9EB7

#define ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ 0x9EB8
#define ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF 0x9EB9

#define ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ 0x9EBA
#define ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF 0x9EBB

#define ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ 0x9EBC
#define ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF 0x9EBD

#define ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ 0x9EBE
#define ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF 0x9EBF

#define ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ 0x9EC0
#define ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF 0x9EC1

#define ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ 0x9EC2
#define ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF 0x9EC3

#define ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ 0x9EC4
#define ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF 0x9EC5

#define ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ 0x9EC6
#define ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF 0x9EC7

#define ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ 0x9EC8
#define ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF 0x9EC9

#define ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ 0x9ECA
#define ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF 0x9ECB

#define ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ 0x9ECC
#define ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF 0x9ECD

#define ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ 0x9ECE
#define ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF 0x9ECF

/******************************************************************************
 * bMinState
 */
enum ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_Etag
{
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_BOOT = 0,  //!< ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_BOOT
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_PREOP = 1, //!< ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_PREOP
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_SAFEOP = 2,//!< ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_SAFEOP
  ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_OP = 3,    //!< ECM_IF_CONFIGURATION_SLAVE_MIN_STATE_OP
};

/******************************************************************************
 * ulTransitionFlags
 */
enum ECM_IF_TRANSITION_FLAGS_Etag
{
  MSK_ECM_IF_TRANSITION_FLAGS_IP_BEFORE_ALCONTROL = 1 << 0,
  MSK_ECM_IF_TRANSITION_FLAGS_PS_BEFORE_ALCONTROL = 1 << 1,
  MSK_ECM_IF_TRANSITION_FLAGS_PI_BEFORE_ALCONTROL = 1 << 2,
  MSK_ECM_IF_TRANSITION_FLAGS_SP_BEFORE_ALCONTROL = 1 << 3,
  MSK_ECM_IF_TRANSITION_FLAGS_SO_BEFORE_ALCONTROL = 1 << 4,
  MSK_ECM_IF_TRANSITION_FLAGS_SI_BEFORE_ALCONTROL = 1 << 5,
  MSK_ECM_IF_TRANSITION_FLAGS_OS_BEFORE_ALCONTROL = 1 << 6,
  MSK_ECM_IF_TRANSITION_FLAGS_OP_BEFORE_ALCONTROL = 1 << 7,
  MSK_ECM_IF_TRANSITION_FLAGS_OI_BEFORE_ALCONTROL = 1 << 8,
  MSK_ECM_IF_TRANSITION_FLAGS_IB_BEFORE_ALCONTROL = 1 << 9,
  MSK_ECM_IF_TRANSITION_FLAGS_BI_BEFORE_ALCONTROL = 1 << 10,
  MSK_ECM_IF_TRANSITION_FLAGS_II_BEFORE_ALCONTROL = 1 << 11,
  MSK_ECM_IF_TRANSITION_FLAGS_PP_BEFORE_ALCONTROL = 1 << 12,
  MSK_ECM_IF_TRANSITION_FLAGS_SS_BEFORE_ALCONTROL = 1 << 13,

  MSK_ECM_IF_TRANSITION_FLAGS_IP_AFTER_ALCONTROL = 1 << 16,
  MSK_ECM_IF_TRANSITION_FLAGS_PS_AFTER_ALCONTROL = 1 << 17,
  MSK_ECM_IF_TRANSITION_FLAGS_PI_AFTER_ALCONTROL = 1 << 18,
  MSK_ECM_IF_TRANSITION_FLAGS_SP_AFTER_ALCONTROL = 1 << 19,
  MSK_ECM_IF_TRANSITION_FLAGS_SO_AFTER_ALCONTROL = 1 << 20,
  MSK_ECM_IF_TRANSITION_FLAGS_SI_AFTER_ALCONTROL = 1 << 21,
  MSK_ECM_IF_TRANSITION_FLAGS_OS_AFTER_ALCONTROL = 1 << 22,
  MSK_ECM_IF_TRANSITION_FLAGS_OP_AFTER_ALCONTROL = 1 << 23,
  MSK_ECM_IF_TRANSITION_FLAGS_OI_AFTER_ALCONTROL = 1 << 24,
  MSK_ECM_IF_TRANSITION_FLAGS_IB_AFTER_ALCONTROL = 1 << 25,
  MSK_ECM_IF_TRANSITION_FLAGS_II_AFTER_ALCONTROL = 1 << 26,
  MSK_ECM_IF_TRANSITION_FLAGS_PP_AFTER_ALCONTROL = 1 << 27,
  MSK_ECM_IF_TRANSITION_FLAGS_SS_AFTER_ALCONTROL = 1 << 28,
};

/******************************************************************************
 * usSlaveReconnectRestartMode
 */
enum ECM_IF_SLAVE_RECONNECT_RESTART_MODE_Etag
{
  ECM_IF_SLAVE_RECONNECT_RESTART_NO_SLAVES = 0,
  ECM_IF_SLAVE_RECONNECT_RESTART_ALL_SLAVES = 2
};

/******************************************************************************
 * Packet: ECM_IF_CMD_BEGIN_CONFIGURATION_REQ/ECM_IF_CMD_BEGIN_CONFIGURATION_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_Ttag
{
  uint16_t usTxProcDataImageBytes;
  uint16_t usRxProcDataImageBytes;
  uint16_t usCcProcDataImageBytes;
  uint16_t usSlaveReconnectRestartMode;
  /* 8 bytes */
  uint32_t ulStackConfigFlags;
  /* 12 bytes */
  uint32_t ulWatchdogTime;
  /* 16 bytes */
  uint32_t ulBaseSyncOffsetPercentage; /* value range 0-10000 resembling 0-100.00% */
  uint16_t usVlanId;
  uint8_t bCyclicVlanPriority;
  uint8_t bAcyclicVlanPriority;
  uint32_t ulBaseCycleTimeNs;
  uint32_t aulReserved[14]; /* set to zero */
  /* 84 bytes */
} ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_T;

#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV1_SIZE 8
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV2_SIZE 12
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV3_SIZE 16
#define ECM_IF_BEGIN_CONFIGURATION_REQ_REV4_SIZE 84

/* ulStackConfigFlags */
enum
{
  ECM_IF_STACK_CONFIG_FLAGS_STAY_IN_INIT_WHEN_MASTER_LINK_DOWN = 0x00000001,
  ECM_IF_STACK_CONFIG_FLAGS_USE_CUSTOM_BASE_SYNC_OFFSET_PERCENTAGE = 0x00000004,
  ECM_IF_STACK_CONFIG_FLAGS_USE_VLAN_TAGGING = 0x00000008,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_BEGIN_CONFIGURATION_REQ_DATA_T tData;
} ECM_IF_BEGIN_CONFIGURATION_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_BEGIN_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_BEGIN_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_BEGIN_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_BEGIN_CONFIGURATION_REQ_T tReq;
  ECM_IF_BEGIN_CONFIGURATION_CNF_T tCnf;
} ECM_IF_BEGIN_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_END_CONFIGURATION_REQ/ECM_IF_CMD_END_CONFIGURATION_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_END_CONFIGURATION_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_CNF_DATA_Ttag
{
  uint32_t ulNumSlavesInConfig;
  uint32_t ulActualProcessDataOutputBytes;
  uint32_t ulActualProcessDataInputBytes;
} ECM_IF_END_CONFIGURATION_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_END_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_END_CONFIGURATION_CNF_DATA_T tData;
} ECM_IF_END_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_END_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_END_CONFIGURATION_REQ_T tReq;
  ECM_IF_END_CONFIGURATION_CNF_T tCnf;
} ECM_IF_END_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ABORT_CONFIGURATION_REQ/ECM_IF_CMD_ABORT_CONFIGURATION_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ABORT_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_ABORT_CONFIGURATION_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ABORT_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_ABORT_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_ABORT_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ABORT_CONFIGURATION_REQ_T tReq;
  ECM_IF_ABORT_CONFIGURATION_CNF_T tCnf;
} ECM_IF_ABORT_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_UNLOAD_CONFIGURATION_REQ/ECM_IF_CMD_UNLOAD_CONFIGURATION_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_UNLOAD_CONFIGURATION_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_UNLOAD_CONFIGURATION_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_UNLOAD_CONFIGURATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_UNLOAD_CONFIGURATION_CNF_T;


/* packet union */
typedef union ECM_IF_UNLOAD_CONFIGURATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_UNLOAD_CONFIGURATION_REQ_T tReq;
  ECM_IF_UNLOAD_CONFIGURATION_CNF_T tCnf;
} ECM_IF_UNLOAD_CONFIGURATION_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_LOAD_ENI_REQ/ECM_IF_CMD_LOAD_ENI_CNF
 */

/* request packet */
enum ECM_IF_LOAD_ENI_REQ_TYPE_Ttag
{
  ECM_IF_LOAD_ENI_XML_UNCOMPRESSED = 0,
  ECM_IF_LOAD_ENI_CONFIG_NXD_ECMv3 = 1,
  ECM_IF_LOAD_ENI_XML_LZMA = 2,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_REQ_DATA_Ttag
{
  uint32_t ulLoadType;
} ECM_IF_LOAD_ENI_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_LOAD_ENI_REQ_DATA_T tData;
} ECM_IF_LOAD_ENI_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_LOAD_ENI_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_LOAD_ENI_CNF_T;


/* packet union */
typedef union ECM_IF_LOAD_ENI_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_LOAD_ENI_REQ_T tReq;
  ECM_IF_LOAD_ENI_CNF_T tCnf;
} ECM_IF_LOAD_ENI_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_REQ/ECM_IF_CMD_ADD_SLAVE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulIdentificationFlags;
  uint32_t ulVendorId;
  uint32_t ulProductCode;
  uint32_t ulRevisionNumber;
  uint32_t ulSerialNumber;
  uint32_t ulSlaveConfigFlags;
} ECM_IF_ADD_SLAVE_REQ_DATA_T;

/* ulIdentificationFlags */
enum ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_Etag
{
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_VENDORID = 0x00000001,
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_PRODUCTCODE = 0x00000002,
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO_LO = 0x00000004,
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO_HI = 0x00000010,
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_REVISIONNO = 0x00000014,
  MSK_ECM_IF_CONFIGURATION_SLAVE_IDENTIFICATION_FLAGS_CHECK_SERIALNO = 0x00000008,
};

/* ulSlaveConfigFlags */
enum ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_Etag
{
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_REINIT_AFTER_COMM_ERROR = 0x00000001,
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_AUTO_RESTORE_STATE = 0x00000002,
  MSK_ECM_IF_CONFIGURATION_SLAVE_CONFIG_FLAGS_OPTIONAL_SLAVE = 0x00000004, /* every slave behind will be optional as segment as well even though the flag is not set on those */
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REQ_T tReq;
  ECM_IF_ADD_SLAVE_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_MAILBOX_REQ/ECM_IF_CMD_ADD_SLAVE_MAILBOX_CNF
 */

/* usMbxProtocols */
enum ECM_IF_SLAVE_MAILBOX_CFG_PROTOCOL_FLAGS_Etag
{
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_AOE = 1 << 1,
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_EOE = 1 << 2,
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_COE = 1 << 3,
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_FOE = 1 << 4,
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_SOE = 1 << 5,
  MSK_ECM_IF_SLAVE_MAILBOX_CFG_FLAGS_VOE = 1 << 15,
};

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_Ttag
{
  uint16_t usStationAddress;

  uint16_t usStdTxMbxPhysOffset;
  uint16_t usStdTxMbxSize;

  uint16_t usStdRxMbxPhysOffset;
  uint16_t usStdRxMbxSize;

  uint8_t bStdTxSmControlByte;
  uint8_t bStdRxSmControlByte;

  uint8_t bStdTxMbxSmNo;
  uint8_t bStdRxMbxSmNo;

  uint16_t usMbxStateBitNo;

  uint16_t usBootTxMbxPhysOffset;
  uint16_t usBootTxMbxSize;

  uint16_t usBootRxMbxPhysOffset;
  uint16_t usBootRxMbxSize;

  uint8_t bBootTxSmControlByte;
  uint8_t bBootRxSmControlByte;

  uint8_t bBootTxMbxSmNo;
  uint8_t bBootRxMbxSmNo;

  uint16_t usMbxProtocols;
} ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_MAILBOX_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_MAILBOX_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_MAILBOX_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_MAILBOX_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_MAILBOX_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_MAILBOX_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_MAILBOX_REQ_T tReq;
  ECM_IF_ADD_SLAVE_MAILBOX_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_MAILBOX_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_MBX_INITCMD_CNF
 */

/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_COE_INITCMD_CNF
 */

/* request packet */
enum ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_WRITE_SINGLE = 0,
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_COMPARE_SINGLE = 1,
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_WRITE_COMPLETE = 2,
  ECM_IF_ADD_SLAVE_COE_INITCMD_ACTION_COMPARE_COMPLETE = 3,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulTransitionFlags;
  uint32_t ulAction;
  uint16_t usStationAddress;
  uint16_t usIndex;
  uint8_t bSubIndex;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalSizeBytes;
  uint8_t abData[1024];
} ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_DATA_T tData;
}  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulTransitionFlags;
  uint32_t ulAction;
  uint16_t usStationAddress;
  uint16_t usIndex;
  uint8_t bSubIndex;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalSizeBytes;
} ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_COE_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_COE_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_COE_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_COE_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_SOE_INITCMD_CNF
 */

/* request packet */
enum ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_WRITE = 0,
  ECM_IF_ADD_SLAVE_SOE_INITCMD_ACTION_COMPARE = 1,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulTransitionFlags;
  uint32_t ulAction;
  uint16_t usStationAddress;
  uint16_t usIdn;
  uint8_t bDriveNo;
  uint8_t bElementFlags;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalSizeBytes;
  uint8_t abData[1024];
} ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulTransitionFlags;
  uint32_t ulAction;
  uint16_t usStationAddress;
  uint16_t usIdn;
  uint8_t bDriveNo;
  uint8_t bElementFlags;
  uint32_t ulTimeoutMs;
  uint32_t ulTotalSizeBytes;
} ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_SOE_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_SOE_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_SOE_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_REQ/ECM_IF_CMD_ADD_SLAVE_REG_INITCMD_CNF
 */

/* request packet */
enum ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_Etag
{
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_WRITE = 0,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE = 1,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_EQ = ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_NE = 2,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GE_UNSIGNED = 3,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LE_UNSIGNED = 4,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GT_UNSIGNED = 5,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LT_UNSIGNED = 6,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GE_SIGNED = 7,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LE_SIGNED = 8,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_GT_SIGNED = 9,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_LT_SIGNED = 10,
  ECM_IF_ADD_SLAVE_REG_INITCMD_ACTION_COMPARE_NONE = 11,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_Ttag
{
  uint32_t ulAction;
  uint32_t ulTransitionFlags;
  uint16_t usStationAddress;
  uint16_t usAdo;
  uint8_t abData[1024];
} ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_Ttag
{
  uint32_t ulAction;
  uint32_t ulTransitionFlags;
  uint16_t usStationAddress;
  uint16_t usAdo;
  uint16_t usLength;
} ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_REG_INITCMD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_REG_INITCMD_REQ_T tReq;
  ECM_IF_ADD_SLAVE_REG_INITCMD_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_REG_INITCMD_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_REQ/ECM_IF_CMD_ADD_SLAVE_DC_PARAMS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bCyclicUnitControl;
  uint8_t bDcActivate;
  uint32_t ulDcCyc0Time;
  uint32_t ulDcCyc1Time;
  uint64_t ullDcSyncShiftTime;
  uint8_t abDcLatchControl[2];
} ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_DC_PARAMS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_DC_PARAMS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_DC_PARAMS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_DC_PARAMS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_REQ/ECM_IF_CMD_ADD_SLAVE_ESM_TIMEOUTS_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulPreOpTimeoutMs;
  uint32_t ulSafeOpTimeoutMs;
  uint32_t ulBackToInitTimeoutMs;
  uint32_t ulBackToSafeOpTimeoutMs;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_Ttat
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_ESM_TIMEOUTS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_REQ/ECM_IF_CMD_ADD_SLAVE_ESC_TIMEOUTS_CNF
 */

/* request packet */
enum ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_FLAGS_Etag
{
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_TO_ALL_SLAVES = 0x8000,
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_WDG_TIME_PROCDATA = 0x0001,
  MSK_ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_SET_WDG_TIME_PDI = 0x0002,
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usSetFlags;
  uint16_t usWdgDivider;
  uint16_t usWdgTimePdi;
  uint16_t usWdgTimeProcData;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint16_t usSetFlags;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_REQ_T tReq;
  ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_ESC_TIMEOUTS_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_CYCLIC_FRAME_REQ/ECM_IF_CMD_ADD_CYCLIC_FRAME_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_Ttag
{
  uint32_t ulCycleTimeMultiplier; /* 1 => cycle time is equal to ulBaseCycleTimeNs */
  uint32_t ulCyclePrescalerStartIndex; /* value between 0 and ([Smallest common nominator of all ulCycleTimeMultiplier] - 1) */
  uint32_t ulRepetitionCount; /* 0 == one frame, 1 == two frames */
} ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_FRAME_REQ_DATA_T tData;
} ECM_IF_ADD_CYCLIC_FRAME_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_Ttag
{
  uint8_t bCyclicFrameIdx;
} ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_T;


typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_FRAME_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_FRAME_CNF_DATA_T tData;
} ECM_IF_ADD_CYCLIC_FRAME_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_CYCLIC_FRAME_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_FRAME_REQ_T tReq;
  ECM_IF_ADD_CYCLIC_FRAME_CNF_T tCnf;
} ECM_IF_ADD_CYCLIC_FRAME_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_REQ/ECM_IF_CMD_ADD_CYCLIC_TELEGRAM_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_Ttag
{
  uint8_t bCyclicFrameIdx;
  uint8_t bMinState; /* except BOOT all allowed */
  uint8_t bCmd;
  ECM_IF_SLAVE_ADDR_T tAddr;
  uint16_t usLength;
  uint16_t usInitialWkc;
  uint16_t usExpectedWkc;
  uint8_t bTransmitDataFlow;
  uint16_t usTransmitSourceOffset;
  uint8_t bReceiveDataFlow;
  uint16_t usReceiveDestinationOffset;
  uint16_t usWkcCompareReceiveDestinationOffset;
} ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_T;

/* bTransmitDataFlow/bReceiveDataFlow */

enum ECM_IF_CYCLIC_DATAFLOW_Etag
{
  /** Transmit telegram data filled with zeroes */
  ECM_IF_CYCLIC_DATA_FLOW_TX_ZEROFILL = 0,

  /** Ignore receive telegram data */
  ECM_IF_CYCLIC_DATA_FLOW_RX_UNUSED = ECM_IF_CYCLIC_DATA_FLOW_TX_ZEROFILL,

  /** Transmit buffer within telegram data */
  ECM_IF_CYCLIC_DATA_FLOW_TX_PROCESS_DATA_IMAGE = 1,

  /** Copy telegram data to receive buffer */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE = 2,

  /** Copy telegram data to receive buffer (clear if WKC does not match) */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE_CLEAR_IF_WKC_MISMATCH = 3,

  ECM_IF_CYCLIC_DATA_FLOW_CC_PROCESS_DATA_IMAGE = 4,

  /** Copy telegram data to mailbox state buffer */
  ECM_IF_CYCLIC_DATA_FLOW_MBX_STATE = 5,

  /** DC SysTime control command for external synchronization
   * Allowed commands:
   * FPWR (ADP=FixedAddr, ADO=0x910, Length=4 or 8)
   * APWR (ADP=AutoIncAddr, ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_DC_SYSTIME_EXT_SYNC = 6,

  /** DC SysTime receive timestamp
   * Allowed commands:
   * ARMW (ADO=0x910, Length=4 or 8)
   * FRMW (ADO=0x910, Length=4 or 8)
   * FPRD (ADP=FixedAddr, ADO=0x910, Length=4 or 8)
   * APRD (ADP=AutoIncAddr, ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_RX_DC_SYSTIME = 7,

  /** DC SysTime update control
   * Allowed commands:
   * ARMW (ADO=0x910, Length=4 or 8)
   * FRMW (ADO=0x910, Length=4 or 8)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_DC_SYSTIME_CONTROL = 8,

  /** Broadcast AlStatus read
   * Allowed commands:
   * BRD (ADO=0x130, Length=2)
   */
  ECM_IF_CYCLIC_DATA_FLOW_BRD_ALSTATUS = 9,

  /** Broadcast DC SysTimeDiff read
   * Allowed commands:
   * BRD (ADO=0x92C, Length=2)
   */
  ECM_IF_CYCLIC_DATA_FLOW_BRD_DC_SYSTIME_DIFF  = 10,

  /** Copy telegram data to receive buffer (freeze if WKC does not match) */
  ECM_IF_CYCLIC_DATA_FLOW_RX_PROCESS_DATA_IMAGE_FREEZE_IF_WKC_MISMATCH = 11,

  /** DC SysTime Latch Trigger Tx
   * Allowed commands:
   * BWR (ADO=0x900, Length=4)
   */
  ECM_IF_CYCLIC_DATA_FLOW_TX_BWR_DC_RX_LATCH = 12,

  /** DC SysTime Latch Trigger Rx
   * Allowed commands:
   * BWR (ADO=0x900, Length=4)
   */
  ECM_IF_CYCLIC_DATA_FLOW_RX_BWR_DC_RX_LATCH = ECM_IF_CYCLIC_DATA_FLOW_TX_BWR_DC_RX_LATCH,
};

/* Offsets in DPM when not mapped
 * usTransmitSourceOffset/usReceiveDestinationOffset/usWkcCompareReceiveDestinationOffset
 */
enum
{
  ECM_IF_CYCLIC_CMD_NOT_MAPPED_IN_PROCIMAGE = 0xFFFF
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_DATA_T tData;
} ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_Ttag
{
  uint8_t bCyclicFrameIdx;
} ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_T;


typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_DATA_T tData;
} ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_CYCLIC_TELEGRAM_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_CYCLIC_TELEGRAM_REQ_T tReq;
  ECM_IF_ADD_CYCLIC_TELEGRAM_CNF_T tCnf;
} ECM_IF_ADD_CYCLIC_TELEGRAM_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_SMCFG_REQ/ECM_IF_CMD_ADD_SLAVE_SMCFG_CNF
 */

/* add slave SM configs, not to be used for mailbox Sync Managers */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bSmNo;
  uint8_t bMinState;
  uint16_t usSmPhysAddr;
  uint16_t usSmPhysLength;
  uint8_t bControlByte;
  uint8_t bEnableByte;
} ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SMCFG_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_SMCFG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bSmNo;
} ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_SMCFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SMCFG_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_SMCFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_SMCFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_SMCFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_SMCFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_SMCFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_FMMUCFG_REQ/ECM_IF_CMD_ADD_SLAVE_FMMUCFG_CNF
 */

/* bFmmuType */
enum ECM_IF_SLAVE_FMMU_CFG_TYPE_Etag
{
  ECM_IF_SLAVE_FMMU_CFG_TYPE_E_FROM_SLAVE = 1,
  ECM_IF_SLAVE_FMMU_CFG_TYPE_E_TO_SLAVE = 2,
};

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bFmmuNo;
  uint8_t bFmmuType;
  uint8_t bMinState;
  uint32_t ulLogStartAddr;
  uint16_t usLogLength;
  uint16_t usSmPhysAddr;
  uint8_t bLogStartBit;
  uint8_t bLogEndBit;
  uint8_t bPhysStartBit;
} ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_FMMUCFG_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_FMMUCFG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t bFmmuNo;
} ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_FMMUCFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_FMMUCFG_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_FMMUCFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_FMMUCFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_FMMUCFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_FMMUCFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_FMMUCFG_PCK_T;

/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_REQ/ECM_IF_CMD_ADD_SLAVE_AOE_NETID_CFG_CNF
 */

#define ECM_IF_AOE_NETID_CFG_INITIALIZE_NETID 0x0002

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint8_t abNetId[6];
  uint16_t usFlags;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_AOE_NETID_CFG_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_REQ_T tReq;
  ECM_IF_ADD_SLAVE_AOE_NETID_CFG_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_AOE_NETID_CFG_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_REQ/ECM_IF_CMD_ADD_SLAVE_EOE_IP_PARAM_CNF
 */

#define ECM_IF_EOE_FLAGS_MAC_INCLUDED 0x01
#define ECM_IF_EOE_FLAGS_IP_ADDR_INCLUDED 0x02
#define ECM_IF_EOE_FLAGS_SUBNET_MASK_INCLUDED 0x04
#define ECM_IF_EOE_FLAGS_DEFAULT_GW_INCLUDED 0x08
#define ECM_IF_EOE_FLAGS_DNS_IP_INCLUDED 0x10
#define ECM_IF_EOE_FLAGS_DNS_NAME_INCLUDED 0x20

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_Ttag
{
  uint16_t usStationAddress;
  uint32_t ulEoEFlags;
  uint8_t abMacAddress[6];
  uint32_t ulIPAddress; /* in little endian order */
  uint32_t ulSubnetMask; /* in little endian order */
  uint32_t ulDefGatewayAddress; /* in little endian order */
  uint32_t ulDnsIpAddr; /* in little endian order */
  uint8_t abDnsName[32];
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_DATA_T tData;
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_Ttag
{
  uint16_t usStationAddress;
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_DATA_T tData;
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_SLAVE_EOE_IP_PARAM_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_REQ_T tReq;
  ECM_IF_ADD_SLAVE_EOE_IP_PARAM_CNF_T tCnf;
} ECM_IF_ADD_SLAVE_EOE_IP_PARAM_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_REQ/ECM_IF_CMD_ADD_MANDATORY_SLAVE_LIST_CNF
 *
 * Packet supports fragmentation
 *
 * Only a single transfer is allowed
 */

/* index in lists represents auto-increment position */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_Ttag
{
  uint16_t usThisStationAddress;
  /* ausPortConnectedTo is only needed on auto-inc startup */
  uint16_t ausPortConnectedTo[4]; /* 0xFFFF == not connected, 0 == connected to redundancy port (only one entry valid), otherwise slave station address */
  /* table index 0 in slave table entry 0 is always 0 for connected to main port */
  /* position equals auto-inc position */
} ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T;


/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_Ttag
{
  /* actual number of entries in packet is ptPck->tHead.ulLen / sizeof(ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T) */
  ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_MANDATORY_SLAVE_LIST_ENTRY_T)];
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_DATA_T tData;
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_MANDATORY_SLAVE_LIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_REQ_T tReq;
  ECM_IF_ADD_MANDATORY_SLAVE_LIST_CNF_T tCnf;
} ECM_IF_ADD_MANDATORY_SLAVE_LIST_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_REQ/ECM_IF_CMD_ADD_HOT_CONNECT_GROUP_CNF
 *
 * Packet supports fragmentation
 *
 * Only a single transfer is allowed
 */

/* index in lists represents auto-increment position */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_HOT_CONNECT_GROUP_ENTRY_Ttag
{
  uint16_t usThisStationAddress;
  /* ausPortConnectedTo is only needed on auto-inc startup */
  uint16_t ausPortConnectedTo[4]; /* 0xFFFF == not connected */
  /* table index 0 in slave table entry 0 is always 0xFFFF for in port of hot connect slave */
  /* position equals auto-inc position */
} ECM_IF_HOT_CONNECT_GROUP_ENTRY_T;


/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_Ttag
{
  uint16_t usIdentificationReg;
  uint16_t usIdentificationValue;
  uint32_t ulIdentificationTimeoutMs;
  /* actual number of entries in packet is ptPck->tHead.ulLen / sizeof(ECM_IF_HOT_CONNECT_GROUP_ENTRY_T) */
  ECM_IF_HOT_CONNECT_GROUP_ENTRY_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_HOT_CONNECT_GROUP_ENTRY_T)];
} ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_T;

/* some well-known identification registers */
enum ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_Etag
{
  ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_SECOND_STATION_ADDRESS = 0x0012,
  ECM_IF_ADD_HOT_CONNECT_GROUP_IDENTIFICATION_REG_ALSTATUSCODE = 0x0134 /* this represents the identification via ALCONTROL/ALSTATUS */
};

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_DATA_T tData;
} ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_T;


/* packet union */
typedef union ECM_IF_ADD_HOT_CONNECT_GROUP_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_ADD_HOT_CONNECT_GROUP_REQ_T tReq;
  ECM_IF_ADD_HOT_CONNECT_GROUP_CNF_T tCnf;
} ECM_IF_ADD_HOT_CONNECT_GROUP_PCK_T;


/******************************************************************************
 * Packet: ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_REQ/ECM_IF_CMD_SET_DEFAULT_TARGET_STATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_Ttag
{
  uint8_t bDefaultTargetState;
} ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_DATA_T tData;
} ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_T;


/* packet union */
typedef union ECM_IF_SET_DEFAULT_TARGET_STATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_DEFAULT_TARGET_STATE_REQ_T tReq;
  ECM_IF_SET_DEFAULT_TARGET_STATE_CNF_T tCnf;
} ECM_IF_SET_DEFAULT_TARGET_STATE_PCK_T;


/******************************************************************************
 * Packet:  ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ/ECM_IF_CMD_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_Ttag
{
  uint32_t ulBaseSyncOffsetPercentage; /* value range 0-10000 resembling 0%-100% */
  uint32_t ulFlags; /* set to zero */
  uint32_t aulReserved[16]; /* set to zero */
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_DATA_T tData;
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_T;


/* packet union */
typedef union ECM_IF_BASE_SYNC_OFFSET_PERCENTAGE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_REQ_T tReq;
  ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_CNF_T tCnf;
} ECM_IF_SET_BASE_SYNC_OFFSET_PERCENTAGE_PCK_T;


/******************************************************************************/
#define ECM_IF_CMD_AUTO_CONFIGURE_REQ 0x9ED0
#define ECM_IF_CMD_AUTO_CONFIGURE_CNF 0x9ED1

/******************************************************************************
 * Packet:  ECM_IF_CMD_AUTO_CONFIGURE_REQ/ECM_IF_CMD_AUTO_CONFIGURE_CNF
 */

/* ulACfgFlags */
enum ECM_IF_ACFG_FLAGS_Etag
{
  MSK_ECM_IF_ACFG_FLAGS_ENABLE_IP_CONFIG = 0x00000001,
  MSK_ECM_IF_ACFG_FLAGS_ENABLE_FIRST_ASSIGNED = 0x00000004,
};

/* ulSystemFlags */
#define MSK_ECM_IF_ACFG_SYSTEM_FLAGS_APP_CONTROLLED 0x00000001

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_REQ_DATA_Ttag
{
  uint32_t ulSystemFlags;
  uint32_t ulWatchdogTime;
  uint32_t ulBusCycleNs;
  /* 12 bytes */
  uint16_t usTxProcDataImageBytes;
  uint16_t usRxProcDataImageBytes;
  uint16_t usCcProcDataImageBytes;
  uint16_t usSlaveReconnectRestartMode;
  /* 20 bytes */
  uint32_t ulStackConfigFlags;
  uint32_t ulACfgFlags;
  /* 28 bytes */
  uint16_t usFirstAssignedFirstStationAddress;
  uint8_t bTargetState;
  /* 31 bytes */
  uint8_t abMasterIpAddress[4];
  uint8_t abSubnetMask[4];
  uint8_t abDefaultGateway[4];
  /* 43 bytes */
  uint8_t abReserved[5];
  /* 48 bytes */
  uint32_t ulBaseSyncOffsetPercentage; /* value range 0-10000 */
  /* 52 bytes */
  uint16_t usVlanId;
  uint8_t bCyclicVlanPriority;
  uint8_t bAcyclicVlanPriority;
  uint32_t aulReserved[15]; /* set to zero */
  /* 116 bytes */
} ECM_IF_AUTO_CONFIGURE_REQ_DATA_T;

#define ECM_IF_AUTO_CONFIGURE_REQ_REV1_SIZE 43
#define ECM_IF_AUTO_CONFIGURE_REQ_REV2_SIZE 116

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_REQ_DATA_T tData;
} ECM_IF_AUTO_CONFIGURE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_Ttag
{
  uint16_t usTopologyPosition;
  uint16_t usIdentityValidFlags;
  uint32_t ulVendorId;
  uint32_t ulProductCode;
  uint32_t ulRevisionNumber;
  uint32_t ulSerialNumber;
  uint32_t ulProcessedAction;
  uint16_t usIndex;
  uint8_t bSubIndex;
  uint32_t ulDetailResult;
} ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_CNF_DATA_Ttag
{
  uint32_t ulNumSlavesInConfig;
  uint32_t ulActualProcessDataOutputBytes;
  uint32_t ulActualProcessDataInputBytes;
  ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T atEntries[HIL_MAX_DATA_SIZE / sizeof(ECM_IF_AUTO_CONFIGURE_ERROR_INFO_DATA_T)];
} ECM_IF_AUTO_CONFIGURE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ECM_IF_AUTO_CONFIGURE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_CNF_DATA_T tData;
} ECM_IF_AUTO_CONFIGURE_CNF_T;


/* packet union */
typedef union ECM_IF_AUTO_CONFIGURE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ECM_IF_AUTO_CONFIGURE_REQ_T tReq;
  ECM_IF_AUTO_CONFIGURE_CNF_T tCnf;
} ECM_IF_AUTO_CONFIGURE_PCK_T;

/* pragma pack */
#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_UNPACK_1(ECM_IF_PUBLIC)
#endif

#endif
