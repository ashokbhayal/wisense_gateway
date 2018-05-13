/*
 * File Name : fu.h
 * Author : ram krishnan
 * Created : aug/21/2015
 */

#ifndef __FU_H__
#define __FU_H__

#define UC_FU_PAGE_RESP_DFLT_TMO_SECS  10

#define UC_FU_PAGE_MSG_SEG_IDX_FIELD_SHIFT  13
#define UC_FU_PAGE_MSG_SEG_IDX_FIELD_BM  0x7
#define UC_FU_PAGE_MSG_SEG_IDX_FIELD_BM_SHIFTED  0xe000   // Upper 3 bits

#define FW_UPG_DFLT_PAGE_SZ  40

#define FW_MAX_SEG_CNT  16

#define FW_SEG_MIN_START_ADDR  0x2100
#define FW_SEG_MAX_START_ADDR  0xffff
#define FW_MAX_LEN  (FW_SEG_MAX_START_ADDR - FW_SEG_MIN_START_ADDR + 1)

#define FU_PAGE_MSG_SEG_IDX_PAGE_IDX_FIELD_LEN  2

#define FW_MAX_SEG_SIZE  65536

#define FW_UPG_MAX_TX_ATTEMPT_CNT 15

#define FW_UPG_MAX_CRC_MISMATCH_CNT  5

typedef struct 
{
   int segLen;
   unsigned int segCRC;
   unsigned int segStartAddr;
   unsigned int pageCnt;
   unsigned int lastPageSz;
   unsigned char *segBuff_p;
} FW_segInfo_s;

typedef enum 
{
  UC_FU_EVT_START = 0,
  UC_FU_EVT_NODE_REG_CHECK_SUCCESSFUL,
  UC_FU_EVT_NODE_REG_CHECK_FLR,
  UC_FU_EVT_RCVD_NODE_FU_MOD_STS,
  UC_FU_EVT_NO_RESP_FROM_NODE,
  UC_FU_EVT_GW_COMM_ERR,
  UC_FU_EVT_RCVD_MALFORMED_MSG,
  UC_FU_EVT_SYS_ERR,
  UC_FU_EVT_INTER_PAGE_TX_INTERVAL,
  UC_FU_EVT_IMAGE_PAGE_SENT
} UC_FU_evt_t;

typedef enum 
{
   UC_FU_STATE_INIT = 0,
   UC_FU_STATE_CHECK_NODE_REGN = 1,
   UC_FU_STATE_ERROR = 2,
   UC_FU_STATE_DONE = 3,
   UC_FU_STATE_WAIT_INIT_1_RESP = 4,
   UC_FU_STATE_WAIT_INIT_2_RESP = 5,
   UC_FU_STATE_IMAGE_PAGE_SENT = 6,
   UC_FU_STATE_SENT_IMAGE_SWITCH_REQUEST
} UC_FU_state_t;


typedef enum
{
  UC_FU_NODE_STATE_IDLE,
  UC_FU_NODE_STATE_WAIT_INIT_2,
  UC_FU_NODE_STATE_INIT_RESP_SENT,
  UC_FU_NODE_STATE_ABORT_RCVD,
  UC_FU_NODE_STATE_RECEIVING_IMAGE,
  UC_FU_NODE_STATE_RECEIVED_IMAGE,
  UC_FU_NODE_STATE_WRITING_SEG,
} UC_FU_nodeState_t;


typedef enum
{
  UC_FU_RC_SUCCESS = 0,
  UC_FU_RC_NO_STORAGE = 1,
  UC_FU_RC_INV_REQUEST = 2,
  UC_FU_RC_INV_PAGE_SZ = 3,
  UC_FU_RC_INV_TOO_MANY_SEGMENTS = 4,
  UC_FU_RC_SEG_TOO_BIG = 5,
  UC_FU_RC_INV_SEG_ADDR = 6,
  UC_FU_RC_EEPROM_ACCESS_ERROR = 7,
  UC_FU_RC_NO_ALTERNATE_IMAGE = 8,
  UC_FU_RC_MALFORMED_INIT_REQ = 9,
  UC_FU_RC_WRONG_PAGE_RCVD = 10,
  UC_FU_RC_SEG_CRC_MISMATCH = 11,
  UC_FU_RC_NO_STANDBY_IMAGE = 12,
  UC_FU_RC_STORE_IMAGE_INFO_CORRUPT = 13
} UC_FU_rc_t;

typedef struct
{
  unsigned char rcvd;
  unsigned char nodeFUState;
  unsigned char retCode;
  unsigned char segIdx;
  unsigned int pageIdx;
  unsigned int pageCRC;
} UC_FU_lastResp_s;

typedef struct
{
  unsigned char nextEvt;
  int segCnt;
  int txTryCnt;
  UC_FU_state_t currState;
  unsigned int shortAddr;
  FW_segInfo_s segList[FW_MAX_SEG_CNT];

  int currSegIdx;
  int currPageIdx;
  int crcMismatchCnt;

  UC_FU_lastResp_s lastResp;

} UC_FU_cntxt_s;

#endif
