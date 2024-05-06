#include "radio.h" 


//#define GATEWAY

//#define MESH_DEBUG

#define DISCOVERY_ANS_LENGTH        13
#define ANS_PAD_MS                  5

#define ANS_SIZE_BYTE       9   /* */

#define RETRY_LIMIT     5       /* maximum count of reqest retries until giving up */
/********************************************************/

#define SPREADING_FACTOR            8
#ifdef SX128x_H
    #define CF_MHZ                      2487.0
    #define BW_KHZ                      400
    #define TX_DBM                      12
#else
    #define CF_MHZ                      917.6
    #define BW_KHZ                      500
    #define TX_DBM                      17
#endif

#define SP_US               ((1<<SPREADING_FACTOR) / (BW_KHZ/1000.0))
#define N_PRE_SYMB          (((WAKEUP_INTERVAL_MS * 1000) / SP_US) + 8)

#define N_DISCOVERY_ANS             180   // enough time slots for randomized collision avoidance
#define N_HALF_DISCOVERY_ANS             (N_DISCOVERY_ANS / 2)

#define WAKEUP_INTERVAL_MS                  1000
#define CHANNEL_VACANT_REQUIRED_COUNT       3

#define ID_NONE             0x00000000
#define ANY_ID              0xffffffff

typedef enum {
    /*  0 */ CMD_UNUSED = 0,
    /*  1 */ CMD_ANS,
    /*  2 */ CMD_DISCOVERY_REQ,
    /*  3 */ CMD_DISCOVERY_ANS,
    /*  4 */ CMD_ATTACH_REQ,
    /*  5 */ CMD_USER_PAYLOAD_UP_REQ,
    /*  6 */ CMD_USER_PAYLOAD_DN_REQ,
    /*  7 */ CMD_NEW_DEVICE_ATTACHED_REQ,
    /*  8 */ CMD_REMOVE_DEVICE_REQ,
    /*  9 */ CMD_DOWNSTREAM_NOT_RESPONDING,
} cmd_e;

typedef enum {
    /* 0 */ NOT_ANSWERING = 0,
    /* 1 */ ANSWER_OK,
    /* 2 */ ANSWER_BUSY,
    /* 3 */ ANSWER_UNATTACHED,
} ans_e;

typedef union {
    struct {
        uint8_t currentOp : 5; // 0,1,2,3,4
        uint8_t txAns     : 3; // 5,6,7
    } bits;
    uint16_t octet;
} reqflags_t;
extern reqflags_t reqFlags;

typedef struct {
    uint8_t discoverAnswering : 1; // 0
    uint8_t sending_req       : 1; // 1
    uint8_t firstDiscoverAns  : 1; // 2
    uint8_t CallTXRequest     : 1; // 3
    uint8_t unused            : 1; // 4
    uint8_t getAns            : 1; // 5
    uint8_t vacantCheck       : 1; // 6
    uint8_t deferred_send     : 1; // 7
} flags_t;

extern volatile flags_t flags;

typedef struct local lid_list_t;
typedef struct children cid_list_t;

struct children {
    uint32_t id;
    children* next;
};

struct local {
    uint32_t id;    // device directly attached
    children* attachedList; // devices attached to id
    local* next;
};

struct _fwd_ {
    uint8_t buf[247];
    int len;
    uint32_t A_id;
    uint32_t B_id;
    uint32_t tx_dest_id;    // pkt destination
};
extern struct _fwd_ fwd;

struct _nr_ {
    uint32_t reporting_id;
    uint32_t device_not_respoding_id;
};
extern struct _nr_ notResponding;

/* from main.cpp: */
#define HFG_UNATTACHED          0xff
#ifdef GATEWAY
    extern const uint8_t hops_from_gateway;
#else
    extern uint8_t hops_from_gateway;
#endif
extern EventQueue queue;
extern RawSerial pc; 
extern char pcbuf[64]; /* local user terminal */
extern uint32_t my_id;
extern unsigned discovery_ans_time_step_us;
extern unsigned discovery_ans_time_total_us;
void setPreambleSize(bool wakesize, uint8_t by);
uint32_t getu32FromBuf(const uint8_t* in);
void putu32ToBuf(uint8_t* out, uint32_t v);
void putu16ToBuf(uint8_t* out, uint16_t v);
uint16_t getu16FromBuf(const uint8_t* in);
uint16_t crc16( uint8_t *buffer, uint16_t length );
void txBuf_send(bool sendingReq);
bool remove_directlyAttached_device(uint32_t id);
void remove_childDevice(uint32_t id, uint32_t* attachedTo);
uint32_t find_dest_id(uint32_t reqid);
#ifdef MESH_DEBUG
int _rx_log_printf(const char *format, ...);
#endif /* MESH_DEBUG */
extern uint32_t tx_dest_id;
extern uint8_t txBuf[];
extern uint8_t txBuf_idx;
extern uint16_t dbg_plCur;
extern uint8_t dbg_plSetBy;
extern const char* const cmdStrs[];
void start_periodic_rxing(uint8_t by);

/* radio specific */
void radio_print_status(void);
void cmd_op(uint8_t);
void radio_printOpMode(void);
bool isRadioRxing(void);

#ifndef GATEWAY
/* upstream interface */
extern uint32_t id_newDeviceNotification;
typedef struct {
    uint32_t id, cnt;
    int preference;
    uint8_t hfg;
} upstream_t;
extern upstream_t attUp;
void upstream_init(void);
void discovery_rx_end(void);
void upstream_print_status(void);
int uplink(const uint8_t* userPayload, uint8_t userPayloadSize);
void upstream_new_device_notify(void);
void upstream_ans_rxDoneCB(float rssi, float snr, uint8_t* idx, uint32_t, uint8_t);
#endif /* !GATEWAY */
void upstream_req_rxDoneCB(float rssi, float snr, uint8_t* idx, uint32_t, uint8_t);
void upstream_signal_check(float rssi, float snr, uint8_t rx_hfg, uint32_t rx_id);
void init_attached_upstream(void);
void upstream_attached_check(uint32_t);

/* downstream interface */
struct remove {
    uint32_t destID;
    uint32_t removeID;
};
extern struct remove downRemove;
extern lid_list_t* attachedDevices;
#ifdef GATEWAY
typedef struct {
    uint8_t len;
    uint32_t originating_src_id;
    uint8_t rxBufIdx;
} upInfo_t;
void downstream_req_rxDoneCB(float rssi, float snr, uint8_t* idx, uint32_t, uint8_t, upInfo_t*);
#else
void downstream_req_rxDoneCB(float rssi, float snr, uint8_t* idx, uint32_t, uint8_t);
#endif
void downstream_ans_rxDoneCB(float rssi, float snr, uint8_t* idx, uint32_t, uint8_t);
void request_remove_device(void);


void cmd_downlink(uint8_t argsAt);

/* application layer: */
void app_init(void);
void gateway_uplink(uint8_t len, uint32_t, const uint8_t* payload);   /* uplink handler */
void app_downlink(uint8_t len, const uint8_t* payload); /* downlink handler */
void app_uplink_complete(void);

#ifdef MESH_DEBUG
    #define Mdbg_printf(fmt, ...)       pc.printf((fmt), ##__VA_ARGS__)
    #define mdbg_putc(x)                pc.putc(x)
    #define Rx_log_printf(fmt, ...)     _rx_log_printf((fmt), ##__VA_ARGS__)
#else
    #define Mdbg_printf(fmt, ...)       
    #define mdbg_putc(x)
    #define Rx_log_printf(fmt, ...)    
#endif

