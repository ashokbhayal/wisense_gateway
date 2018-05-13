/*
 * File Name : gw.h
 * Author : ram krishnan
 * Created : 10/21/2013
 */

#ifndef __GW_H__
#define __GW_H__


#define WIFI_MAC_ADDR_LEN 6
#define IPV4_ADDR_LEN 4


#define  GW_EVT_FILE_NAME  "./gw_evt_list.doc"
#define  GW_SNSR_DATA_LOG_FILE_NAME  "./gw_snsr_data_log.csv"

#define LPWMN_GW_MSG_TYPE_INVALID  0xffff

// Message type 0x0 is reserved for ACKs
#define LPWMN_GW_MSG_TYPE_NODE_CNT_REQ     0x1
#define LPWMN_GW_MSG_TYPE_NODE_LIST_REQ    0x3

#define LPWMN_GW_MSG_TYPE_GET_NODE_LAST_FATAL_ERR_ID  0x4

#define LPWMN_GW_MSG_TYPE_RELAY_TO_NODE    0x5
#define LPWMN_GW_MSG_TYPE_RELAY_FROM_NODE  0x6
#define LPWMN_GW_MSG_TYPE_KEEP_ALIVE_REQ   0x7
#define LPWMN_GW_MSG_TYPE_KEEP_ALIVE_RESP  0x8
#define LPWMN_GW_MSG_TYPE_PING_NODE_REQ    0x9
#define LPWMN_GW_MSG_TYPE_NODE_PING_RESP   0xa

#define LPWMN_GW_MSG_TYPE_REBOOT_COORD  0xb

#define LPWMN_GW_MSG_TYPE_NODE_RT_DISC_STS   0x10

#define LPWMN_GW_MSG_TYPE_PATH_DISC_REQ    0x13
#define LPWMN_GW_MSG_TYPE_PATH_DISC_RESP   0x14

#define LPWMN_GW_MSG_TYPE_NODE_TRAFFIC_STATS_REQ   0x18

#define LPWMN_GW_MSG_TYPE_RESET_NODE_TRAFFIC_STATS  0x1a

#define LPWMN_GW_MSG_TYPE_STOP_TRAFFIC_REQ   0x1e
#define LPWMN_GW_MSG_TYPE_START_TRAFFIC_REQ  0x1f


#define LPWMN_GW_MSG_TYPE_GET_COORD_STATS    0x20
#define LPWMN_GW_MSG_TYPE_RESET_COORD_STATS  0x21

#define LPWMN_GW_MSG_TYPE_STOP_NWK   0x30
#define LPWMN_GW_MSG_TYPE_START_NWK  0x31

#define LPWMN_GW_MSG_TYPE_ENABLE_NWK_JOINS  0x40
#define LPWMN_GW_MSG_TYPE_DISABLE_NWK_JOINS  0x41
#define LPWMN_GW_MSG_TYPE_GET_NWK_JOIN_CTRL_STATE  0x42
#define LPWMN_GW_MSG_TYPE_ADD_NODE_TO_WHITE_LIST  0x43
#define LPWMN_GW_MSG_TYPE_DEL_NODE_FROM_WHITE_LIST  0x44
#define LPWMN_GW_MSG_TYPE_DELETE_WHITE_LIST  0x45
#define LPWMN_GW_MSG_TYPE_GET_WHITE_LIST_ENTRY  0x46

#define LPWMN_GW_MSG_TYPE_ADD_NODE_TO_BLACK_LIST  0x47
#define LPWMN_GW_MSG_TYPE_DEL_NODE_FROM_BLACK_LIST  0x48
#define LPWMN_GW_MSG_TYPE_DELETE_BLACK_LIST  0x49


#define LPWMN_GW_MSG_TYPE_CFG_DROP_BCN_REQS  0x4a
#define LPWMN_GW_MSG_TYPE_CFG_ACCEPT_BCN_REQS  0x4b
#define LPWMN_GW_MSG_TYPE_GET_BCN_REQ_HANDLING_CFG  0x4c

#define LPWMN_GW_MSG_TYPE_GET_BLACK_LIST_ENTRY  0x4d

#define LPWMN_GW_MSG_TYPE_SET_NWK_ID          0x50
#define LPWMN_GW_MSG_TYPE_SET_RADIO_CHANNEL         0x51
#define LPWMN_GW_MSG_TYPE_SET_NWK_COORD_EXT_ADDR  0x52
#define LPWMN_GW_MSG_TYPE_SET_RADIO_TX_PWR        0x53
#define LPWMN_GW_MSG_TYPE_START_COORD             0x54
#define LPWMN_GW_MSG_TYPE_STOP_COORD              0x55


#define LPWMN_GW_MSG_TYPE_GET_NWK_ID          0x59
#define LPWMN_GW_MSG_TYPE_GET_NWK_CHANNEL         0x5a
#define LPWMN_GW_MSG_TYPE_GET_NWK_COORD_EXT_ADDR  0x5b
#define LPWMN_GW_MSG_TYPE_GET_RADIO_TX_PWR        0x5c
#define LPWMN_GW_MSG_TYPE_GET_RADIO_PART_NR       0x5d
#define LPWMN_GW_MSG_TYPE_GET_RADIO_FREQ_BAND     0x5e
#define LPWMN_GW_MSG_TYPE_GET_RADIO_CARRIER_FREQ  0x5f
#define LPWMN_GW_MSG_TYPE_GET_RADIO_BAUD_RATE     0x60
#define LPWMN_GW_MSG_TYPE_GET_RADIO_MOD_FMT       0x61

#define LPWMN_GW_MSG_TYPE_SET_WCT  0x70   // set wall clock time
#define LPWMN_GW_MSG_TYPE_GET_WCT  0x71   // get wall clock time

#define LPWMN_MSG_TYPE_GET_UP_TIME  0x80  // in seconds

#define LPWMN_GW_MSG_TYPE_BB_OPN_CLOSE  0x85
#define LPWMN_GW_MSG_TYPE_BB_OPN_OPEN   0x86


#define LPWMN_GW_MSG_TYPE_COORD_LOG           0x90
#define LPWMN_GW_MSG_TYPE_ENABLE_COORD_LOG    0x91
#define LPWMN_GW_MSG_TYPE_DISABLE_COORD_LOG   0x92


#define LPMWN_GW_MSG_TYPE_GET_COORD_FREE_MEM_DESC_CNT   0x93

#define LPWMN_GW_MSG_TYPE_DEREG_NODE   0xa0

#define LPWMN_GW_MSG_TYPE_EVENT   0xb0

#define LPWMN_GW_MSG_TYPE_GET_COORD_SW_BUILD_DATE  0xc0
#define LPWMN_GW_MSG_TYPE_GET_COORD_SW_BUILD_TIME  0xc1


#define LPWMN_GW_MSG_TYPE_GET_COORD_ATTR_VAL  0xc8
#define LPWMN_GW_MSG_TYPE_SET_COORD_ATTR_VAL  0xc9

#define LPWMN_MSG_TYPE_GET_RT_NH_ON_COORD  0xd0

#define LPWMN_GW_MSG_TYPE_GET_COORD_UP_TIME  0xda

#define LPWMN_GW_MSG_TYPE_MON_CHANN  0xe8

// rkris@wisense.in / may/23/16 - Coord FU
#define LPWMN_GW_MSG_TYPE_COORD_FU   0xf0


#define LPWMN_GW_MSG_TYPE_PHOTON_GET_MAC_ADDR         0x1000
#define LPWMN_GW_MSG_TYPE_PHOTON_GET_SSID             0x1001
#define LPWMN_GW_MSG_TYPE_PHOTON_GET_IPV4_ADDR        0x1002
#define LPWMN_GW_MSG_TYPE_PHOTON_GET_IP_GW_IPV4_ADDR  0x1003
#define LPWMN_GW_MSG_TYPE_PHOTON_GET_IPV4_MASK        0x1004
#define LPWMN_GW_MSG_TYPE_PHOTON_HAS_CREDENTIALS      0x1005
#define LPWMN_GW_MSG_TYPE_PHOTON_GET_CREDENTIALS      0x1006
#define LPWMN_GW_MSG_TYPE_PHOTON_CLEAR_ALL_CREDENTIALS   0x1007
#define LPWMN_GW_MSG_TYPE_PHOTON_SNSR_DATA_HANDLING_CFG_DROP  0x1008
#define LPWMN_GW_MSG_TYPE_PHOTON_SNSR_DATA_HANDLING_CFG_RELAY_TO_CLOUD  0x1009


#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_IPV4_ADDR      0x1200
#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_BATT_VOLTAGE   0x1201
#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_BATT_SOC       0x1202
#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_CELLULAR_CONN_STATE   0x1203
#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_CLOUD_CONN_STATE   0x1204  
#define LPWMN_GW_MSG_TYPE_TX_TO_CLOUD_MSG_CNT   0x1205
#define LPWMN_GW_MSG_TYPE_RX_FROM_CLOUD_MSG_CNT   0x1206
#define LPWMN_GW_MSG_TYPE_MSG_TO_CLOUD_DROPPED_CNT   0x1207
#define LPWMN_GW_MSG_TYPE_ELECTRON_GET_RADIO_RSSI  0x1208


// Events sent to host by the LPWMN coordinator
#define LPWMN_GW_EVT_TYPE_NODE_REG               0x1
#define LPWMN_GW_EVT_TYPE_LOCAL_RT_DISC_STARTED  0x2
#define LPWMN_GW_EVT_TYPE_SYS_BOOT               0x3
#define LPWMN_GW_EVT_TYPE_NODE_DEREG             0x4
#define LPWMN_GW_EVT_TYPE_LOCAL_RT_DISC_OVER     0x5
#define LPWMN_GW_EVT_TYPE_BCN_REQ_RCVD           0x6
#define LPWMN_GW_EVT_TYPE_MAC_CNFRM              0x7
#define LPWMN_GW_EVT_TYPE_PURGE_RT_TBL           0x8
#define LPWMN_GW_EVT_TYPE_RCVD_PKT_WITH_INV_MAC_SRC_ADDR   0x9
#define LPWMN_GW_EVT_TYPE_ASSOC_REQ_DROPPED      0xa
#define LPWMN_GW_EVT_TYPE_ASSOC_REQ_RCVD         0xb
#define LPWMN_GW_EVT_TYPE_PATH_DISC_STARTED      0xc
#define LPWMN_GW_EVT_TYPE_PATH_DISC_OVER         0xd
#define LPWMN_GW_EVT_TYPE_RCVD_RT_REQ_WITH_INV_SRC_ADDR   0xe
#define LPWMN_GW_EVT_TYPE_RCVD_PKT_WITH_INV_MESH_SRC_ADDR   0xf
#define LPWMN_GW_EVT_TYPE_ASSOC_REQ_FROM_BL_NODE  0x10
#define LPWMN_GW_EVT_TYPE_RELAYED_ASSOC_REQ_FROM_BL_NODE 0x11
#define LPWMN_GW_EVT_TYPE_RT_REQ_RCVD  0x12
#define LPWMN_GW_EVT_TYPE_BCN_REQ_DROPPED  0x13
#define LPWMN_GW_EVT_TYPE_BCN_TX_DONE  0x14
#define LPWMN_GW_EVT_TYPE_BCN_TX_FLR  0x15
#define LPWMN_GW_EVT_TYPE_WPDS_ALARM  0x20
#define LPWMN_GW_EVT_ID_LEN  0x1

#define LPWMN_GW_NODE_CNT_RESP_PYLD_LEN  2
#define LPWMN_GW_MSG_NODE_IDX_FIELD_LEN  2
#define LPWMN_GW_MSG_RADIO_TX_PWR_FIELD_LEN  2
#define LPWMN_GW_MSG_RADIO_CHANN_FIELD_LEN  2
#define LPWMN_GW_MSG_RADIO_CARRIER_FREQ_FIELD_LEN  4
#define LPWMN_GW_MSG_RADIO_BAUD_RATE_FIELD_LEN  4
#define LPWMN_GW_MSG_NODE_DATA_PUSH_INTERVAL_FIELD_LEN  2

#define LPWMN_GW_STS_SUCCESS               0x0
#define LPWMN_GW_STS_INV_REQ               0x1
#define LPWMN_GW_STS_INV_PARAMS            0x2
#define LPWMN_GW_STS_PREV_REQ_IN_PROGRESS  0x3
#define LPWMN_GW_STS_NWK_STOPPED           0x4
#define LPWMN_GW_STS_NO_ROUTE_TO_NODE      0x5
#define LPWMN_GW_STS_INV_PYLD_LEN          0x6
#define LPWMN_GW_STS_INV_RADIO_TX_PWR_SET_POINT   0x7


#define LPWMN_MSG_RSSI_LEN  1
#define LPWMN_MSG_CORR_LQI_LEN  1 

#define LPWMN_MAC_SHORT_ADDR_LEN  2
#define LPWMN_MAC_EXT_ADDR_LEN  8
#define LPWMN_MAC_NODE_CAPABILITY_INFO_LEN  1

#define LPWMN_COORD_SHORT_ADDR  0x1
#define LPWMN_MAX_UNICAST_SHORT_ADDR  0x7fff
#define LPWMN_MIN_MULTICAST_SHORT_ADDR  0x8000
#define LPWMN_MAX_MULTICAST_SHORT_ADDR  0xfffe
#define LPWMN_BROADCAST_SHORT_ADDR  0xffff

#define LPWMN_MESH_LEVEL_BROADCAST_ADDR   0xfff0

#define GW_NODE_MAX_CHAN_CNT  32

#define EUI64_ADDR_LEN  8


#define NODE_CAP_DEV_TYPE_BIT_SHIFT_MSK           (1 << 1)
#define NODE_CAP_INFO_POWER_SOURCE_BIT_SHIFT_MSK  (1 << 2)
#define NODE_CAP_INFO_RX_ON_IDLE_BIT_SHIFT_MSK    (1 << 3)
#define NODE_CAP_INFO_SEC_CAP_BIT_SHIFT_MSK       (1 << 6)
#define NODE_CAP_INFO_ALLOC_ADDR_BIT_SHIFT_MSK    (1 << 7)


#define LPWMN_GW_PATH_DISC_EVT_HOP_CNT_FIELD_LEN  1
#define LPWMN_GW_PATH_DISC_EVT_STS_FIELD_LEN  1

typedef enum
{
   ADP_STS_SUCCESS,
   ADP_STS_NODE_NOT_FOUND,
   ADP_STS_OOM,
   ADP_STS_TX_FLR,
   ADP_STS_RT_DISC_OOM,
   ADP_STS_RT_DISC_FAILED,
   ADP_STS_ROUTING_LOOP,
   ADP_STS_MAX_HOPS_EXCEEDED,
   ADP_STS_NO_ROUTE,
   ADP_STS_PATH_DISC_TIMED_OUT,
   ADP_STS_NODE_NOT_ASSOCIATED,
   ADP_STS_TOO_MANY_FRAGMENTS ,
   ADP_STS_FRAG_NOT_SUPPORTED
} ADP_sts_t;

typedef struct _LPWMN_nodeListEntry_
{
   unsigned int shortAddr;
   unsigned char extAddr[LPWMN_MAC_EXT_ADDR_LEN];
   unsigned int msgRcvdCnt;
   unsigned int regCnt;

   struct _LPWMN_nodeListEntry_ *next_p;
} LPWMN_nodeListEntry_s;

typedef struct
{
  int attrId;
  char descrStr[128];
  int readAllowed;
  int writeAllowed;
  int attrLen;
} GW_attrDescrListEntry_s;


typedef enum
{
   LPWMN_ADP_STS_SUCCESS,
   LPWMN_ADP_STS_NODE_NOT_FOUND,
   LPWMN_ADP_STS_OOM,
   LPWMN_ADP_STS_TX_FLR,
   LPWMN_ADP_STS_RT_DISC_OOM,
   LPWMN_ADP_STS_RT_DISC_FAILED,
   LPWMN_ADP_STS_ROUTING_LOOP,
   LPWMN_ADP_STS_MAX_HOPS_EXCEEDED,
   LPWMN_ADP_STS_NO_ROUTE,
   LPWMN_ADP_STS_PATH_DISC_TIMED_OUT,
   LPWMN_ADP_STS_NODE_NOT_ASSOCIATED,
   LPWMN_ADP_STS_TOO_MANY_FRAGMENTS ,
   LPWMN_ADP_STS_FRAG_NOT_SUPPORTED
} LPWMN_ADP_sts_t;


typedef enum
{
   LPWMN_PHY_MODULE = 1, 
   LPWMN_MAC_MODULE = 2,
   LPWMN_ADP_MODULE = 3,
   LPWMN_APP_MODULE = 4,
   LPWMN_PLTFRM_MODULE = 5,
   LPWMN_SYSTEM_MODULE = 6
} LPWMN_moduleId_t;


// Thingspeak interface
#define THINGSPEAK_CHANNEL_API_KEY_LEN  16
#define THINGSPEAK_CHANNEL_MIN_FIELD_NR  1
#define THINGSPEAK_CHANNEL_MAX_FIELD_NR  8

typedef struct
{
    void *_next_p;
    unsigned char _extAddr[LPWMN_MAC_EXT_ADDR_LEN];
    char _channAPIKey[64];
    unsigned char _snsrId;
    int _fieldIdx;
} GW_tsMappingEntry_s;

typedef int (*GW_cmdHndlrFunc_t)(void);

typedef struct
{
  char _name[32];
  int _cmdId;
} GW_electronCmd_s;

#endif
