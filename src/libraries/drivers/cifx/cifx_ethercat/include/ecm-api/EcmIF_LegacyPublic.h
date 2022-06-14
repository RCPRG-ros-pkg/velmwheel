/**************************************************************************************
 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
***************************************************************************************
$Id: EcmIF_LegacyPublic.h 82276 2019-04-05 09:40:55Z Sven $:


Changes:
 Date          Description
 -----------------------------------------------------------------------------------
 2014-10-22    derived from ECM v3 Public header
**************************************************************************************/

#ifndef __ECMIF_LEGACYPUBLIC_H
#define __ECMIF_LEGACYPUBLIC_H

#include <Hil_Packet.h>

#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_PACK_1(__ETHERCAT_MASTER_PACKET)
#endif

#define  ETHERCAT_MASTER_CMD_SDO_UPLOAD_REQ                  0x00650006
#define  ETHERCAT_MASTER_CMD_SDO_UPLOAD_CNF                  0x00650007

#define  ETHERCAT_MASTER_CMD_SDO_DOWNLOAD_REQ                0x00650008
#define  ETHERCAT_MASTER_CMD_SDO_DOWNLOAD_CNF                0x00650009

#define  ETHERCAT_MASTER_CMD_GET_ODLIST_REQ                  0x0065000A
#define  ETHERCAT_MASTER_CMD_GET_ODLIST_CNF                  0x0065000B

/* stack internal packet, not for host */
#define  ETHERCAT_MASTER_CMD_SET_BUSPARAM_REQ                0x0065000C
#define  ETHERCAT_MASTER_CMD_SET_BUSPARAM_CNF                0x0065000D

#define  ETHERCAT_MASTER_CMD_GET_OBJECTDESC_REQ              0x00650018
#define  ETHERCAT_MASTER_CMD_GET_OBJECTDESC_CNF              0x00650019

#define  ETHERCAT_MASTER_CMD_GET_ENTRYDESC_REQ               0x0065001A
#define  ETHERCAT_MASTER_CMD_GET_ENTRYDESC_CNF               0x0065001B

#define  ETHERCAT_MASTER_CMD_READ_EMERGENCY_REQ              0x0065001C
#define  ETHERCAT_MASTER_CMD_READ_EMERGENCY_CNF              0x0065001D

#define  ETHERCAT_MASTER_CMD_GET_DC_DEVIATION_REQ            0x0065001E
#define  ETHERCAT_MASTER_CMD_GET_DC_DEVIATION_CNF            0x0065001F

/* <"old" bus scan, replaced now by generic RCX_BUSSCAN_REQ and RCX_GET_DEVICE_INFO_REQ> */
#define  ETHERCAT_MASTER_CMD_START_BUS_SCAN_REQ              0x00650020
#define  ETHERCAT_MASTER_CMD_START_BUS_SCAN_CNF              0x00650021

#define  ETHERCAT_MASTER_CMD_GET_BUS_SCAN_INFO_REQ           0x00650022
#define  ETHERCAT_MASTER_CMD_GET_BUS_SCAN_INFO_CNF           0x00650023
/* </"old" bus scan, replaced now by generic RCX_BUSSCAN_REQ and RCX_GET_DEVICE_INFO_REQ> */

#define  ETHERCAT_MASTER_CMD_EEPROM_READ_REQ                 0x00650040
#define  ETHERCAT_MASTER_CMD_EEPROM_READ_CNF                 0x00650041

#define  ETHERCAT_MASTER_CMD_EEPROM_WRITE_REQ                0x00650042
#define  ETHERCAT_MASTER_CMD_EEPROM_WRITE_CNF                0x00650043

#define  ETHERCAT_MASTER_CMD_EEPROM_RELOAD_REQ               0x00650044
#define  ETHERCAT_MASTER_CMD_EEPROM_RELOAD_CNF               0x00650045

#define  ETHERCAT_MASTER_CMD_GET_ECSTATE_REQ                 0x00650046
#define  ETHERCAT_MASTER_CMD_GET_ECSTATE_CNF                 0x00650047

#define  ETHERCAT_MASTER_CMD_SET_ECSTATE_REQ                 0x00650048
#define  ETHERCAT_MASTER_CMD_SET_ECSTATE_CNF                 0x00650049

/******************************************************************************/

#define ETHERCAT_MASTER_BUSSTATE_UNKNOWN (0)  /* unknown */
#define ETHERCAT_MASTER_BUSSTATE_INIT    (1)  /* init */
#define ETHERCAT_MASTER_BUSSTATE_PREOP   (2)  /* pre-operational */
#define ETHERCAT_MASTER_BUSSTATE_BOOT    (3)  /* bootstrap mode */
#define ETHERCAT_MASTER_BUSSTATE_SAFEOP  (4)  /* safe operational */
#define ETHERCAT_MASTER_BUSSTATE_OP      (8)  /* operational */


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_SET_BUSPARAM_REQ/ETHERCAT_MASTER_CMD_SET_BUSPARAM_CNF
 */
/* this define is only valid for SET_BUSPARAM packet! */
#define ETHERCAT_MASTER_AUTO_START (0x00000001)

#define ETHERCAT_MASTER_LEAVE_ALL_BROKEN_SLAVES_DOWN   (0x00000000)
#define ETHERCAT_MASTER_LEAVE_ADDRESS_LESS_SLAVES_DOWN (0x00000001)
#define ETHERCAT_MASTER_LEAVE_NO_SLAVES_DOWN           (0x00000002)

/*-----------------------------------------------------------------------------*/

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_DATA_Ttag
{
  uint32_t ulSystemFlags;
  uint32_t ulWdgTime;
  uint32_t ulBusCycleTime; /* in us */
  uint32_t ulBrokenSlaveBehaviour; /* Handling of slaves with error */
  uint32_t fDcActivated; /* Activate Distributed Clocks */
  uint32_t ulReserved;
} ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_DATA_T  tData;  /** packet request data. */
} ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_DATA_Ttag
{
  uint32_t aulReserved[2];
  uint32_t ulInputDataSize;
  uint32_t ulOutputDataSize;
} ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_SET_BUSPARAM_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SET_BUSPARAM_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_SET_BUSPARAM_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_SET_BUSPARAM_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_SDO_DOWNLOAD_REQ/ETHERCAT_MASTER_CMD_SDO_DOWNLOAD_CNF
 */

/* request packet */
#define ETHERCAT_MASTER_COE_MAX_SDO_DOWNLOAD_DATA (HIL_MAX_DATA_SIZE - (sizeof(uint32_t) * 4))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
  uint32_t ulDataCnt;
  uint8_t abSdoData[ETHERCAT_MASTER_COE_MAX_SDO_DOWNLOAD_DATA];
} ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_SDO_DOWNLOAD_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_SDO_UPLOAD_REQ/ETHERCAT_MASTER_CMD_SDO_UPLOAD_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
} ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_T;


/* confirmation packet */
#define ETHERCAT_MASTER_COE_GET_SDO_UPLOAD_CNF_LEN_ON_ERROR (12)
#define ETHERCAT_MASTER_COE_MAX_SDO_UPLOAD_DATA (HIL_MAX_DATA_SIZE - (sizeof(uint32_t) * 4))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
  uint32_t ulDataCnt;
  uint8_t  abSdoData[ETHERCAT_MASTER_COE_MAX_SDO_UPLOAD_DATA];
} ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_SDO_UPLOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SDO_UPLOAD_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_SDO_UPLOAD_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_SDO_UPLOAD_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_ODLIST_REQ/ETHERCAT_MASTER_CMD_GET_ODLIST_CNF
 */

#define ETHERCAT_MASTER_COE_GET_ODLIST_TYPE_ALL      (1)
#define ETHERCAT_MASTER_COE_GET_ODLIST_TYPE_RXPDOMAP (2)
#define ETHERCAT_MASTER_COE_GET_ODLIST_TYPE_TXPDOMAP (3)
#define ETHERCAT_MASTER_COE_GET_ODLIST_TYPE_STORE    (4)
#define ETHERCAT_MASTER_COE_GET_ODLIST_TYPE_STARTUP  (5)

/*-----------------------------------------------------------------------------*/

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulListType;
} ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_T;


/* confirmation packet */
#define ETHERCAT_MASTER_COE_GET_ODLIST_CNF_LEN_ON_ERROR (8)
#define ETHERCAT_MASTER_COE_GET_ODLIST_DATA ((HIL_MAX_DATA_SIZE - (sizeof(uint32_t) * 3)) / sizeof(uint16_t))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulListType;
  uint32_t ulDataCnt;
  uint16_t ausObjectList[ETHERCAT_MASTER_COE_GET_ODLIST_DATA];
} ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_ODLIST_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ODLIST_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_ODLIST_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_ODLIST_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_OBJECTDESC_REQ/ETHERCAT_MASTER_CMD_GET_OBJECTDESC_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;
} ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_T;


/* confirmation packet */
#define ETHERCAT_MASTER_COE_GET_OBJECTDESC_CNF_LEN_ON_ERROR (8)
#define ETHERCAT_MASTER_COE_GET_OBJECTDESC_NAME_LEN (HIL_MAX_DATA_SIZE - (sizeof(uint32_t) * 7))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;/* Index in the object dictionary */
  uint32_t ulDataType; /* Data type of the object */
  uint32_t ulObjCode; /* Object code */
  uint32_t ulObjCategory; /* Object category */
  uint32_t ulMaxNumSubIndex; /* Maximum sub index number */
  uint32_t ulObNameLen; /* Length of the object name */
  uint8_t abObjName[ETHERCAT_MASTER_COE_GET_OBJECTDESC_NAME_LEN]; /* Object name (not NULL terminated!) */
} ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_OBJECTDESC_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_ENTRYDESC_REQ/ETHERCAT_MASTER_CMD_GET_ENTRYDESC_CNF
 */

/* EtherCat CoE entry description value information bit definitions */
#define ETHERCAT_MASTER_COE_ENTRY_OBJACCESS              0x01
#define ETHERCAT_MASTER_COE_ENTRY_OBJCATEGORY            0x02
#define ETHERCAT_MASTER_COE_ENTRY_PDOMAPPING             0x04
#define ETHERCAT_MASTER_COE_ENTRY_UNITTYPE               0x08
#define ETHERCAT_MASTER_COE_ENTRY_DEFAULTVALUE           0x10
#define ETHERCAT_MASTER_COE_ENTRY_MINVALUE               0x20
#define ETHERCAT_MASTER_COE_ENTRY_MAXVALUE               0x40

/*-----------------------------------------------------------------------------*/
/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;
  uint32_t ulSubIndex;
  uint32_t ulAccessBitMask;
} ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_T;


/* confirmation packet */
#define ETHERCAT_MASTER_COE_GET_ENTRYDESC_MAX_DATA (HIL_MAX_DATA_SIZE - (sizeof(uint32_t) * 11))
#define ETHERCAT_MASTER_COE_GET_ENTRYDESC_CNF_LEN_ON_ERROR (12)

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_DATA_Ttag
{
  uint32_t ulNodeId;
  uint32_t ulIndex;/* Index in the object dictionary */
  uint32_t ulSubIndex;
  uint32_t ulValueInfo; /* Bit mask to define which information is available */
  uint32_t ulDataType; /* Object data type */
  uint32_t ulBitLen; /* Object size (number of bits) */
  uint32_t ulObAccess; /* Access rights */
  uint32_t fRxPdoMapping; /* Is the object PDO-mappable? */
  uint32_t fTxPdoMapping; /* Can the PDO be changed */
  uint32_t ulUnitType; /* Unit*/
  uint32_t ulDataLen; /* Size of the remaining object data */
  uint8_t  abObjData[ETHERCAT_MASTER_COE_GET_ENTRYDESC_MAX_DATA]; /* Remaining object data (see EtherCAT specification) */
} ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_ENTRYDESC_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_READ_EMERGENCY_REQ/ETHERCAT_MASTER_CMD_READ_EMERGENCY_CNF
 */
#define ETHERCAT_MASTER_COE_NUMBER_OF_EMERGENCY (5)
#define ETHERCAT_MASTER_COE_EMERGENCY_DATA_BYTES (5)

/*-----------------------------------------------------------------------------*/
/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_DATA_Ttag
{
  uint32_t ulSlaveHandle;
  uint32_t fDeleteEmergency; /* Flag to decide (keep emergeny(s) / clear emergeny(s)) */
} ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_SLAVE_EMERGENCY_Ttag
{
  uint16_t usErrorCode;
  uint8_t bErrorRegister;
  uint8_t abErrorData[ETHERCAT_MASTER_COE_EMERGENCY_DATA_BYTES];
} ETHERCAT_MASTER_SLAVE_EMERGENCY_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_DATA_Ttag
{
  uint32_t ulSlaveHandle;
  uint32_t fDeleteEmergency;  /* value from request */
  uint32_t fOverflowOccured;  /* Emergency dropped cause of full buffer */
  ETHERCAT_MASTER_SLAVE_EMERGENCY_T atEmergenyBuffer[ETHERCAT_MASTER_COE_NUMBER_OF_EMERGENCY]; /* up to five emergencies */
} ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T                             tHead;  /** packet header. */
  ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_DATA_T  tData;  /** packet request data. */
} ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_SLAVE_EMERGENCY_INFO_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_DC_DEVIATION_REQ/ETHERCAT_MASTER_CMD_GET_DC_DEVIATION_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_REQ_Ttag
{
  /* the request needs no data part */
  HIL_PACKET_HEADER_T                                 tHead;  /** packet header. */
} ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_REQ_T;


/* confirmation packet */
#define  ETHERCAT_MASTER_GET_DC_DEVIATION_NUMOFSLAVES ((HIL_MAX_DATA_SIZE - sizeof(uint32_t))/sizeof(uint32_t))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_DATA_Ttag
{
  uint32_t ulBroadcastDeviation;
  uint32_t aulSlaveDeviation[ETHERCAT_MASTER_GET_DC_DEVIATION_NUMOFSLAVES] ;
} ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_DC_DEVIATION_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_START_BUS_SCAN_REQ/ETHERCAT_MASTER_CMD_START_BUS_SCAN_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_DATA_Ttag
{
  uint32_t ulTimeout; /* in ms */
} ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_DATA_Ttag
{
  uint32_t ulFoundSlaves;
} ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_START_BUS_SCAN_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_START_BUS_SCAN_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_START_BUS_SCAN_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_START_BUS_SCAN_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_BUS_SCAN_INFO_REQ/ETHERCAT_MASTER_CMD_GET_BUS_SCAN_INFO_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_DATA_Ttag
{
  uint16_t usAutoIncAddr;
} ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_DATA_Ttag
{
  uint32_t ulVendorId;
  uint32_t ulProductCode;
  uint32_t ulRevisionNumber;
  uint32_t ulSerialNumber;
  uint32_t ulPortState;
} ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_BUS_SCAN_INFO_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_EEPROM_READ_REQ/ETHERCAT_MASTER_CMD_EEPROM_READ_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_DATA_Ttag
{
  uint32_t fFixedAddressing; /* TRUE: use fixed addressing (requires configuration), FALSE: use auto increment addressing */
  uint16_t usSlaveAddress; /* Slave Address, fixed or auto increment address depending on fFixedAddressing */
  uint16_t usEEPromStartOffset; /* Address to start EEPRom read from, (16bit WORD count) */
  uint16_t usReadLen; /* number of 16bit-WORDs */
  uint16_t usTimeout; /* time in ms */
} ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_DATA_Ttag
{
  uint32_t fFixedAddressing; /* value from request */
  uint16_t usSlaveAddress; /* value from request */
  uint16_t usEEPromStartOffset; /* value from request */
  uint16_t ausReadData[750]; /* read data, up to 750 WORDs */
} ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_DATA_T;

#define ETHERCAT_MASTER_EEPROM_READ_EMPTY_SIZE ( sizeof(ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_DATA_T) - 750 * sizeof (uint16_t))

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_Ttag
{
  HIL_PACKET_HEADER_T                            tHead;  /** packet header.       */
  ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_DATA_T  tData;  /** packet request data. */
} ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_EEPROM_READ_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_READ_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_EEPROM_READ_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_EEPROM_READ_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_EEPROM_WRITE_REQ/ETHERCAT_MASTER_CMD_EEPROM_WRITE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_DATA_Ttag
{
  uint32_t fFixedAddressing; /* TRUE: use fixed addressing (requires configuration), FALSE: use auto increment addressing */
  uint16_t usSlaveAddress; /* Slave Address, fixed or auto increment address depending on fFixedAddressing */
  uint16_t usEEPromStartOffset; /* Address to start EEPRom write from (16bit WORD count) */
  uint32_t fAssignAccessBack; /* give slave the EEPROM control back? Set to TRUE to apply new data. Set to FALSE if further fragments follows. */
  uint16_t usTimeout; /* in ms */
  uint16_t ausWriteData[750]; /* data to write, up to 750 WORDs */
} ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_DATA_T;

/* packet without payload */
#define ETHERCAT_MASTER_EEPROM_WRITE_EMPTY_SIZE ( sizeof(ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_DATA_T) - 750 * sizeof (uint16_t) )

/* at least one WORD must be written */
#define ETHERCAT_MASTER_EEPROM_WRITE_MIN_SIZE ( ETHERCAT_MASTER_EEPROM_WRITE_EMPTY_SIZE + sizeof (uint16_t) )

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_DATA_Ttag
{
  uint32_t fFixedAddressing; /* value from request */
  uint16_t usSlaveAddress; /* value from request */
  uint16_t usEEPromStartOffset; /* value from request */
} ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_EEPROM_WRITE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_WRITE_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_EEPROM_WRITE_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_EEPROM_WRITE_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_EEPROM_RELOAD_REQ/ETHERCAT_MASTER_CMD_EEPROM_RELOAD_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_DATA_Ttag
{
  uint32_t fFixedAddressing; /* TRUE: use fixed addressing, FALSE: use auto increment addressing */
  uint16_t usSlaveAddress; /* Slave Address, fixed or auto increment address depending on fFixedAddressing */
  uint16_t usTimeout; /*  in ms */
} ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_DATA_Ttag
{
  uint32_t fFixedAddressing; /* value from request */
  uint16_t usSlaveAddress; /* value from request */
} ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_EEPROM_RELOAD_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_GET_ECSTATE_REQ/ETHERCAT_MASTER_CMD_GET_ECSTATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ECSTATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ETHERCAT_MASTER_PACKET_GET_ECSTATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_DATA_Ttag
{
  uint16_t    usCurrentEcState;  /* see defines ETHERCAT_MASTER_BUSSTATE_*, following values are reported:
  - ETHERCAT_MASTER_BUSSTATE_UNKNOWN: master not initialized
  - ETHERCAT_MASTER_BUSSTATE_INIT, ETHERCAT_MASTER_BUSSTATE_PREOP, ETHERCAT_MASTER_BUSSTATE_SAFEOP, ETHERCAT_MASTER_BUSSTATE_OP  */
} ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_DATA_T tData;
} ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_GET_ECSTATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_GET_ECSTATE_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_GET_ECSTATE_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_GET_ECSTATE_PCK_T;


/******************************************************************************
 * Packet: ETHERCAT_MASTER_CMD_SET_ECSTATE_REQ/ETHERCAT_MASTER_CMD_SET_ECSTATE_CNF
 */

/* request packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_DATA_Ttag
{
  uint16_t usNewEcState; /* see defines ETHERCAT_MASTER_BUSSTATE_*, allowed values are _INIT, _PREOP, _SAFEOP, _OP  */
} ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_DATA_T;

typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_DATA_T tData;
} ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_T;


/* confirmation packet */
typedef __HIL_PACKED_PRE struct __HIL_PACKED_POST ETHERCAT_MASTER_PACKET_SET_ECSTATE_CNF_Ttag
{
  HIL_PACKET_HEADER_T tHead;
} ETHERCAT_MASTER_PACKET_SET_ECSTATE_CNF_T;


/* packet union */
typedef union ETHERCAT_MASTER_PACKET_SET_ECSTATE_PCK_Ttag
{
  HIL_PACKET_HEADER_T tHead;
  ETHERCAT_MASTER_PACKET_SET_ECSTATE_REQ_T tReq;
  ETHERCAT_MASTER_PACKET_SET_ECSTATE_CNF_T tCnf;
} ETHERCAT_MASTER_PACKET_SET_ECSTATE_PCK_T;


#ifdef PRAGMA_PACK_ENABLE
#pragma PRAGMA_UNPACK_1(__ETHERCAT_MASTER_PACKET)
#endif

/*************************************************************************************/
#endif /* #ifndef __ETHERCAT_MASTER_PUBLIC_H */
