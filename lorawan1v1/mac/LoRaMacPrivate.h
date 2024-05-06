#ifndef _LORAMACPRIVATE_H_
#define _LORAMACPRIVATE_H_
#include "LoRaMac1v1.h"

typedef struct sBand
{
    /*!
     * Duty cycle
     */
    uint16_t DCycle;
    /*!
     * Maximum Tx power
     */
    int8_t TxMaxPower;
#ifdef DUTY_ENABLE
    /*!
     * Time stamp of the last Tx frame
     */
    us_timestamp_t LastTxDoneTime;
    /*!
     * Holds the time where the device is off
     */
    us_timestamp_t TimeOff;
#endif /* DUTY_ENABLE */
} Band_t;

#ifdef USE_BAND_ARIB_8CH
    #include "region_arib8ch_private.h"
#elif defined(USE_BAND_780)
    #include "region_cn780_private.h"
#elif defined(USE_BAND_470)
    #include "region_cn470_private.h"
#elif defined(USE_BAND_433)
    #include "region_433_private.h"
#elif defined(USE_BAND_868)
    #include "region_eu868_private.h"
#elif defined(USE_BAND_915_HYBRID) || defined(USE_BAND_915)
    #include "region_us915_private.h"
#else
    #error define USE_BAND_*
#endif

#include "radio.h"

#define RECEIVE_DELAY1_us                              1000000
#define MAX_RX_WINDOW_us                               3000000

#define LC( channelIndex )            ( uint16_t )( 1 << ( channelIndex - 1 ) )

typedef struct {
    uint32_t uplink_in_progress  : 4; //0,1,2,3
    uint32_t uplink_mtype        : 3; //4,5,6
    uint32_t SrvAckRequested     : 1; //8
    uint32_t rxing               : 1; //9
    uint32_t MacCommandsInNextTx : 1; //10
    uint32_t AdrCtrlOn           : 1; //11
    uint32_t OptNeg              : 1; //12
    uint32_t PublicNetwork       : 1; //13
    uint32_t OnTxDelayed         : 1; //14

#ifdef LORAWAN_JOIN_EUI
    uint32_t need_ReKeyConf      : 1; //15
    uint32_t IsLoRaMacNetworkJoined: 1; //16
#else
    uint32_t have_SNwkSIntKey : 1; // 15
    uint32_t have_NwkSEncKey : 1; // 16
    uint32_t need_ResetConf: 1; // 17
#endif /* !LORAWAN_JOIN_EUI  */

} flags_t;

extern flags_t flags;


typedef enum eLoRaMacMoteCmd
{
#ifndef LORAWAN_JOIN_EUI
    MOTE_MAC_RESET_IND               = 0x01,
#endif /* !LORAWAN_JOIN_EUI  */
    /*!
     * LinkCheckReq
     */
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    /*!
     * LinkADRAns
     */
    MOTE_MAC_LINK_ADR_ANS            = 0x03,
    /*!
     * DutyCycleAns
     */
    MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
    /*!
     * RXParamSetupAns
     */
    MOTE_MAC_RX_PARAM_SETUP_ANS      = 0x05,
    /*!
     * DevStatusAns
     */
    MOTE_MAC_DEV_STATUS_ANS          = 0x06,
    /*!
     * NewChannelAns
     */
    MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
    /*!
     * RXTimingSetupAns
     */
    MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,

    SRV_MAC_ADR_PARAM_SETUP_ANS      = 0x0c,
#ifdef LORAWAN_JOIN_EUI
    MOTE_MAC_REKEY_IND               = 0x0b,
    MOTE_MAC_REJOIN_PARAM_ANS        = 0x0f,
#endif /* LORAWAN_JOIN_EUI  */
    MOTE_MAC_DEVICE_TIME_REQ         = 0x0d,
    /*!
     * PingSlotInfoReq
     */
    MOTE_MAC_PING_SLOT_INFO_REQ      = 0x10,
    /*!
     * PingSlotFreqAns
     */
    MOTE_MAC_PING_SLOT_FREQ_ANS      = 0x11,
    /*!
     * BeaconTimingReq
     */
    MOTE_MAC_BEACON_TIMING_REQ       = 0x12,
    /*!
     * BeaconFreqAns
     */
    MOTE_MAC_BEACON_FREQ_ANS         = 0x13,

    MOTE_MAC_MODE_IND                = 0x20
} LoRaMacMoteCmd_t;

#define LORA_MAC_COMMAND_MAX_LENGTH                 15


/*!
 * Global MAC layer parameters
 */
typedef struct sLoRaMacParams
{
    /*!
     * Channels TX power
     */
    int8_t ChannelsTxPower;
    /*!
     * Channels data rate
     */
    int8_t ChannelsDatarate;
    /*!
     * LoRaMac maximum time a reception window stays open
     */
    uint32_t MaxRxWindow_us;
    /*!
     * Receive delay 1
     */
    uint32_t ReceiveDelay1_us;
    /*!
     * Receive delay 2
     */
    uint32_t ReceiveDelay2_us;
#ifdef LORAWAN_JOIN_EUI
    /*!
     * Join accept delay 1
     */
    uint32_t JoinAcceptDelay1_us;
    /*!
     * Join accept delay 1
     */
    uint32_t JoinAcceptDelay2_us;
#endif /* LORAWAN_JOIN_EUI  */
    /*!
     * Number of uplink messages repetitions [1:15] (unconfirmed messages only)
     */
    uint8_t NbTrans;
    /*!
     * Datarate offset between uplink and downlink on first window
     */
    uint8_t Rx1DrOffset;
    /*!
     * LoRaMAC 2nd reception window settings
     */
    Rx2ChannelParams_t Rx2Channel;
    /*!
     * Mask indicating which channels are enabled
     */
    uint16_t ChannelsMask[6];
    uint8_t NbEnabledChannels;
    us_timestamp_t MaxListenTime;
} LoRaMacParams_t;

/*!
 * LoRaMAC header field definition (MHDR field)
 *
 * LoRaWAN Specification V1.0.1, chapter 4.2
 */
typedef union uLoRaMacHeader
{
    /*!
     * Byte-access to the bits
     */
    uint8_t Value;
    /*!
     * Structure containing single access to header bits
     */
    struct sHdrBits
    {
        /*!
         * Major version
         */
        uint8_t Major           : 2;
        /*!
         * RFU
         */
        uint8_t RFU             : 3;
        /*!
         * Message type
         */
        uint8_t MType           : 3;
    }Bits;
}LoRaMacHeader_t;

/*!
 * LoRaMAC frame types
 *
 * LoRaWAN Specification V1.0.1, chapter 4.2.1, table 1
 */
typedef enum eLoRaMacFrameType
{
#ifdef LORAWAN_JOIN_EUI
    /*!
     * LoRaMAC join request frame
     */
    FRAME_TYPE_JOIN_REQ              = 0x00,
    /*!
     * LoRaMAC join accept frame
     */
    FRAME_TYPE_JOIN_ACCEPT           = 0x01,
#endif /* LORAWAN_JOIN_EUI */
    /*!
     * LoRaMAC unconfirmed up-link frame
     */
    FRAME_TYPE_DATA_UNCONFIRMED_UP   = 0x02,
    /*!
     * LoRaMAC unconfirmed down-link frame
     */
    FRAME_TYPE_DATA_UNCONFIRMED_DOWN = 0x03,
    /*!
     * LoRaMAC confirmed up-link frame
     */
    FRAME_TYPE_DATA_CONFIRMED_UP     = 0x04,
    /*!
     * LoRaMAC confirmed down-link frame
     */
    FRAME_TYPE_DATA_CONFIRMED_DOWN   = 0x05,
#ifdef LORAWAN_JOIN_EUI
    /*!
     * LoRaMAC rejoin frame
     */
    FRAME_TYPE_REJOIN_REQ            = 0x06,
#endif /* LORAWAN_JOIN_EUI  */
    /*!
     * LoRaMAC proprietary frame
     */
    FRAME_TYPE_PROPRIETARY           = 0x07,
} LoRaMacFrameType_t;

/*!
 * LoRaMAC server MAC commands
 *
 * LoRaWAN Specification V1.0.1 chapter 5, table 4
 */
typedef enum eLoRaMacSrvCmd
{
#ifndef LORAWAN_JOIN_EUI
    SRV_MAC_RESET_CONF               = 0x01,
#endif /* !LORAWAN_JOIN_EUI  */
    /*!
     * LinkCheckAns
     */
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    /*!
     * LinkADRReq
     */
    SRV_MAC_LINK_ADR_REQ             = 0x03,
    /*!
     * DutyCycleReq
     */
    SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    /*!
     * RXParamSetupReq
     */
    SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    /*!
     * DevStatusReq
     */
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    /*!
     * NewChannelReq
     */
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
    /*!
     * RXTimingSetupReq
     */
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,

    SRV_MAC_ADR_PARAM_SETUP_REQ      = 0x0c,
#ifdef LORAWAN_JOIN_EUI
    SRV_MAC_REKEY_CONF               = 0x0b,
    SRV_MAC_FORCE_REJOIN_REQ         = 0x0e,
    SRV_MAC_REJOIN_PARAM_REQ         = 0x0f,
#endif /* LORAWAN_JOIN_EUI  */
    SRV_MAC_DEVICE_TIME_ANS          = 0x0d,
    /*!
     * PingSlotInfoAns
     */
    SRV_MAC_PING_SLOT_INFO_ANS       = 0x10,
    /*!
     * PingSlotChannelReq
     */
    SRV_MAC_PING_SLOT_CHANNEL_REQ    = 0x11,
    /*!
     * BeaconTimingAns
     */
    SRV_MAC_BEACON_TIMING_ANS        = 0x12,
    /*!
     * BeaconFreqReq
     */
    SRV_MAC_BEACON_FREQ_REQ          = 0x13,

    SRV_MAC_MODE_CONF                = 0x20
} LoRaMacSrvCmd_t;

typedef union uLoRaMacFrameCtrl
{
    /*!
     * Byte-access to the bits
     */
    uint8_t Value;
    /*!
     * Structure containing single access to bits
     */
    struct sCtrlBits
    {
        /*!
         * Frame options length
         */
        uint8_t FOptsLen        : 4;
        /*!
         * Frame pending bit
         */
        uint8_t FPending        : 1;
        /*!
         * Message acknowledge bit
         */
        uint8_t Ack             : 1;
        /*!
         * ADR acknowledgment request bit
         */
        uint8_t AdrAckReq       : 1;
        /*!
         * ADR control in frame header
         */
        uint8_t Adr             : 1;
    }Bits;
} LoRaMacFrameCtrl_t;

typedef struct {
    uint16_t channelsMask[6];
    uint16_t chMask;
    uint8_t status;
    uint8_t chMaskCntl;
    int8_t datarate;
} adr_t;

/* from region: */
extern const int8_t TxPowers[];
extern const uint8_t MaxPayloadOfDatarate[];
extern ChannelParams_t Channels[];
extern const uint8_t Datarates[];
LoRaMacStatus_t LoRaMacChannelRemove( uint8_t id );
LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params );

//void region_adr_next_dr(int8_t* dr, bool);
void region_ScheduleTx( void );
uint32_t region_GetRxBandwidth( int8_t datarate );
uint16_t region_GetRxSymbolTimeout( int8_t datarate );
void region_mac_init(void);
void region_rx1_setup(uint8_t channel);
void region_adr_request(adr_t*);
void region_tx_setup(int8_t dbm, uint8_t pkt_len);
void region_session_start(LoRaMacEventInfoStatus_t);
int8_t region_LimitTxPower( int8_t txPower );
uint8_t region_CountNbEnabledChannels(void);
#ifdef LORAWAN_JOIN_EUI
int8_t region_AlternateDatarate( uint16_t nbTrials );
#endif /* LORAWAN_JOIN_EUI  */

/* from LoRaMac.cpp: */
extern LoRaMacParams_t LoRaMacParams;
extern const LoRaMacParams_t LoRaMacParamsDefaults;
extern uint8_t Channel;
void SendFrameOnChannel( uint8_t ch_num );
void RxWindowSetup( unsigned freq, int8_t datarate, unsigned bandwidth, uint16_t timeout);
uint8_t CountBits( uint16_t mask, uint8_t nbBits );
LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 );
void OnTxDelayedIsr(void);
extern LowPowerTimeout TxDelayedEvent;
extern DeviceClass_t LoRaMacDeviceClass;

#endif /* _LORAMACPRIVATE_H_ */
