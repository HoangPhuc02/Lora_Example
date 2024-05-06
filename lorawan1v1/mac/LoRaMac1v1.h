#ifndef _LORAMAC_H_
#define _LORAMAC_H_

#include "Commissioning.h"

typedef enum {
    /*  0 */ LORAMAC_STATUS_OK = 0,
    /*  1 */ LORAMAC_STATUS_LSE,
    /*  2 */ LORAMAC_STATUS_WAITING_FOR_TXSTART,
    /*  3 */ LORAMAC_STATUS_WAITING_FOR_TXDONE,
    /*  4 */ LORAMAC_STATUS_WAITING_FOR_RX1,
    /*  5 */ LORAMAC_STATUS_WAITING_FOR_RX2,
    /*  6 */ LORAMAC_STATUS_BUSY_UPCONF,
    /*  7 */ LORAMAC_STATUS_SERVICE_UNKNOWN,
    /*  8 */ LORAMAC_STATUS_PARAMETER_INVALID,
    /*  9 */ LORAMAC_STATUS_DATARATE_INVALID,
    /* 10 */ LORAMAC_STATUS_FREQUENCY_INVALID,
    /* 11 */ LORAMAC_STATUS_LENGTH_ERROR,
    /* 12 */ LORAMAC_STATUS_DEVICE_OFF,
    /* 13 */ LORAMAC_STATUS_FREQ_AND_DR_INVALID,
    /* 14 */ LORAMAC_STATUS_EEPROM_FAIL,
    /* 15 */ LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR,
#ifdef LORAWAN_JOIN_EUI
    /* 16 */ LORAMAC_STATUS_NO_NETWORK_JOINED
#endif
} LoRaMacStatus_t;

typedef enum {
    LORAMAC_EVENT_INFO_STATUS_OK,
    LORAMAC_EVENT_INFO_STATUS_INCR_FAIL,
    LORAMAC_EVENT_INFO_STATUS_MLMEREQ,
    LORAMAC_EVENT_INFO_STATUS_UNKNOWN_MTYPE,
    LORAMAC_EVENT_INFO_STATUS_SENDING,
    LORAMAC_EVENT_INFO_STATUS_MCPSREQ,
    LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT,
    LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT,
    LORAMAC_EVENT_INFO_STATUS_RX2_ERROR,
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED,
    LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR,
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS,
    LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL,
    LORAMAC_EVENT_INFO_STATUS_MIC_FAIL,
    LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL,
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED,
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOST,
    LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND,
    LORAMAC_EVENT_INFO_STATUS_NO_APPKEY,
    LORAMAC_EVENT_INFO_BAD_RX_DELAY,
    LORAMAC_EVENT_INFO_STATUS_CHANNEL_BUSY,
#ifdef LORAWAN_JOIN_EUI
    LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL,
    LORAMAC_EVENT_INFO_STATUS_JOINNONCE
#endif
} LoRaMacEventInfoStatus_t;

typedef enum {
    MCPS_NONE,
    MCPS_UNCONFIRMED,
    MCPS_CONFIRMED,
    MCPS_MULTICAST,
    MCPS_PROPRIETARY
} Mcps_t;

#include <stdint.h>
typedef struct {
    LoRaMacEventInfoStatus_t Status;
    Mcps_t McpsRequest;
    uint8_t NbRetries;
    bool AckReceived;
    uint32_t UpLinkCounter;
    uint8_t Datarate;
    int8_t TxPower;
    uint32_t UpLinkFreqHz;
    //us_timestamp_t TxTimeOnAir;
} McpsConfirm_t;

typedef struct {
    uint8_t Snr;
    int16_t Rssi;
    int8_t RxSlot;
    LoRaMacEventInfoStatus_t Status;
    Mcps_t McpsIndication;
    bool RxData;
    uint8_t RxDatarate;
    uint8_t Port;
    uint8_t BufferSize;
    uint8_t *Buffer;
    uint8_t FramePending;
    bool AckReceived;
    uint32_t expectedFCntDown;
    uint16_t receivedFCntDown;
    uint16_t ADR_ACK_CNT;
} McpsIndication_t;

typedef enum {
    MLME_NONE = 0,
    MLME_LINK_CHECK,
    MLME_SWITCH_CLASS,
    MLME_PING_SLOT_INFO,
    MLME_BEACON_TIMING,
    MLME_BEACON_ACQUISITION,
    MLME_TIME_REQ,
    MLME_BEACON,
    MLME_TXCW,
#ifdef LORAWAN_JOIN_EUI
    MLME_JOIN,
    MLME_REJOIN_0,
    MLME_REJOIN_1,
    MLME_REJOIN_2
#endif
} Mlme_t;

#include "lorawan_board.h"

#define DR_0        0
#define DR_1        1
#define DR_2        2
#define DR_3        3
#define DR_4        4
#define DR_5        5
#define DR_6        6
#define DR_7        7
#define DR_8        8
#define DR_9        9
#define DR_10       10
#define DR_11       11
#define DR_12       12
#define DR_13       13
#define DR_14       14
#define DR_15       15


typedef union {
    uint8_t Value;

    struct {
        int8_t Min : 4;
        int8_t Max : 4;
    } Fields;
} DrRange_t;

typedef struct {
    uint32_t FreqHz;
    DrRange_t DrRange;
    uint8_t Band;
} ChannelParams_t;


typedef struct {
    Mcps_t Type;
    struct {
        void *fBuffer;
        uint16_t fBufferSize;
        uint8_t Datarate;
        uint8_t fPort;
    } Req;
} McpsReq_t;

typedef struct {
    uint8_t MaxPossiblePayload;
    uint8_t CurrentPayloadSize;
} LoRaMacTxInfo_t;

typedef enum {
    MIB_DEV_ADDR,
    MIB_DEVICE_CLASS,
    MIB_ADR,
    MIB_PUBLIC_NETWORK,
    MIB_RX2_CHANNEL,
    MIB_CHANNELS_MASK,
    MIB_FNwkSIntKey,
    MIB_APP_SKEY,
    MIB_SNwkSIntKey,
    MIB_NwkSEncKey,
    MIB_NwkSKey,    /* lorawan 1.0 */
    MIB_MAX_LISTEN_TIME,
#ifdef LORAWAN_JOIN_EUI
    MIB_NETWORK_JOINED
#endif
} Mib_t;

typedef enum {
    CLASS_A = 0,
    CLASS_B,
    CLASS_C
} DeviceClass_t;

typedef struct {
    uint32_t FrequencyHz;
    uint8_t  Datarate;
} Rx2ChannelParams_t;

typedef union {
    uint32_t DevAddr;
    bool AdrEnable;
    DeviceClass_t Class;
    bool IsNetworkJoined;
    bool EnablePublicNetwork;
    uint16_t* ChannelsMask;
    const uint8_t* key;
    Rx2ChannelParams_t Rx2Channel;
    us_timestamp_t MaxListenTime;
} MibParam_t;

typedef struct {
    Mib_t Type;
    MibParam_t Param;
} MibRequestConfirm_t;



typedef struct {
    Mlme_t Type;

    union {
        struct {
            DeviceClass_t Class;
        } SwitchClass;

        union {
            uint8_t Value;
            struct sInfoFields {
                uint8_t Periodicity     : 3;
                uint8_t RFU             : 5;
            } Fields;
        } PingSlotInfo;

#ifdef LORAWAN_JOIN_EUI
        struct {
            const uint8_t *DevEui;
            const uint8_t *JoinEui;
            const uint8_t *NwkKey;
            const uint8_t *AppKey;
            uint8_t NbTrials;
        } Join;
#endif /* LORAWAN_JOIN_EUI */

        struct {
            uint16_t Timeout;
        } TxCw;
    } Req;
} MlmeReq_t;


typedef struct {
    LoRaMacEventInfoStatus_t Status;
    Mlme_t MlmeRequest;
    union {
        struct {
            uint8_t DemodMargin;
            uint8_t NbGateways;
        } link;
        struct {
            uint32_t Seconds;   // seconds since epoch
            uint32_t uSeconds;  // fractional part
        } time;
        struct {
            uint32_t rxJoinNonce;
            uint32_t myJoinNonce;
        } join;
    } fields;
    //us_timestamp_t TxTimeOnAir;
} MlmeConfirm_t;


typedef struct {
    Mlme_t MlmeIndication;
    LoRaMacEventInfoStatus_t Status;
    uint32_t freqHz;
#ifdef LORAWAN_JOIN_EUI
    uint8_t JoinRequestTrials;
#endif /* LORAWAN_JOIN_EUI  */     
} MlmeIndication_t;


/*!
 * LoRaMAC events structure
 * Used to notify upper layers of MAC events
 */
typedef struct sLoRaMacPrimitives
{
    /*!
     * \brief   MCPS-Confirm primitive
     *
     * \param   [OUT] MCPS-Confirm parameters
     */
    void (* const MacMcpsConfirm )( const McpsConfirm_t *McpsConfirm );
    /*!
     * \brief   MCPS-Indication primitive
     *
     * \param   [OUT] MCPS-Indication parameters
     */
    void (* const MacMcpsIndication )( const McpsIndication_t *McpsIndication );
    /*!
     * \brief   MLME-Confirm primitive
     *
     * \param   [OUT] MLME-Confirm parameters
     */
    void (* const MacMlmeConfirm )( const MlmeConfirm_t *MlmeConfirm );
    /*!
     * \brief   MLME-Indication primitive
     *
     * \param   [OUT] MLME-Indication parameters
     */
    void (* const MacMlmeIndication )( const MlmeIndication_t *MlmeIndication );
} LoRaMacPrimitives_t;

typedef struct sLoRaMacCallback
{
    /*!
     * \brief   Measures the battery level
     *
     * \retval  Battery level [0: node is connected to an external
     *          power source, 1..254: battery level, where 1 is the minimum
     *          and 254 is the maximum value, 255: the node was not able
     *          to measure the battery level]
     */
    uint8_t (* const GetBatteryLevel )( void );
    /*!
     * \brief   Measures the temperature level
     *
     * \retval  Temperature level
     */
    float (* const GetTemperatureLevel )( void );
} LoRaMacCallback_t;

LoRaMacStatus_t LoRaMacInitialization( const LoRaMacPrimitives_t *primitives, const LoRaMacCallback_t *callbacks );
us_timestamp_t LoRaMacReadTimer(void);
LoRaMacStatus_t LoRaMacQueryTxPossible(uint8_t size, LoRaMacTxInfo_t* txInfo);
LoRaMacStatus_t LoRaMacMcpsRequest( McpsReq_t *mcpsRequest );
LoRaMacStatus_t LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet );
LoRaMacStatus_t LoRaMacMlmeRequest( const MlmeReq_t *mlmeRequest );
LoRaMacStatus_t LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet );
LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params );
void LoRaMacPrintStatus(void);
uint32_t get_fcntdwn(bool);
int8_t LoRaMacGetRxSlot(void);
void LoRaMacUserContext(void);

#endif /* _LORAMAC_H_ */
