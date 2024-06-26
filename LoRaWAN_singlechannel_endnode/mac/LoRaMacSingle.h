/*!
 */
#ifndef __LORAMAC_H__
#define __LORAMAC_H__

// Includes board dependent definitions such as channels frequencies
#include "LoRaMac-definitions.h"

using namespace std::chrono;
/*!
 * Beacon interval in microseconds
 */
#define BEACON_INTERVAL_us                   128000000        /*16000000*/      /*8000000*/
#define PPM_BEACON_INTERVAL              (BEACON_INTERVAL_us / 5000)   /* timer crystal max error accomidation */

/*!
 * Class A&B receive delay 1 in us
 */
#define RECEIVE_DELAY_us                              100000

#define JOIN_ACCEPT_DELAY_us                          100000

/*!
 * Class A&B maximum receive window delay in ms
 */
#define MAX_RX_WINDOW                               3000

/*!
 * Maximum allowed gap for the FCNT field
 */
#define MAX_FCNT_GAP                                16384

/*!
 * ADR acknowledgement counter limit
 */
#define ADR_ACK_LIMIT                               64

/*!
 * Number of ADR acknowledgement requests before returning to default datarate
 */
#define ADR_ACK_DELAY                               32

/*!
 * Number of seconds after the start of the second reception window without
 * receiving an acknowledge.
 * AckTimeout = \ref ACK_TIMEOUT + Random( -\ref ACK_TIMEOUT_RND, \ref ACK_TIMEOUT_RND )
 */
#define ACK_TIMEOUT_us                                 2000000

/*!
 * Random number of seconds after the start of the second reception window without
 * receiving an acknowledge
 * AckTimeout = \ref ACK_TIMEOUT + Random( -\ref ACK_TIMEOUT_RND, \ref ACK_TIMEOUT_RND )
 */
#define ACK_TIMEOUT_RND_us                             1000000

/*!
 * Check the Mac layer state every MAC_STATE_CHECK_TIMEOUT in us
 */
#define MAC_STATE_CHECK_TIMEOUT_us                     1000000

/*!
 * Maximum number of times the MAC layer tries to get an acknowledge.
 */
#define MAX_ACK_RETRIES                             8

/*!
 * RSSI free threshold [dBm]
 */
#define RSSI_FREE_TH                                ( int8_t )( -90 )

/*!
 * Frame direction definition for up-link communications
 */
#define UP_LINK                                     0

/*!
 * Frame direction definition for down-link communications
 */
#define DOWN_LINK                                   1

/*!
 * Sets the length of the LoRaMAC footer field.
 * Mainly indicates the MIC field length
 */
#define LORAMAC_MFR_LEN                             4

/*!
 * LoRaWAN devices classes definition
 */
typedef enum eDeviceClass
{
    /*!
     * LoRaWAN device class A
     *
     * LoRaWAN Specification V1.0.1, chapter 3ff
     */
    CLASS_A,
    /*!
     * LoRaWAN device class B
     *
     * LoRaWAN Specification V1.0.1, chapter 8ff
     */
    CLASS_B,
    /*!
     * LoRaWAN device class C
     *
     * LoRaWAN Specification V1.0.1, chapter 17ff
     */
    CLASS_D,
}DeviceClass_t;

/*!
 * LoRaMAC channels parameters definition
 */
typedef union uDrRange
{
    /*!
     * Byte-access to the bits
     */
    int8_t Value;
    /*!
     * Structure to store the minimum and the maximum datarate
     */
    struct sFields
    {
         /*!
         * Minimum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Min : 4;
        /*!
         * Maximum data rate
         *
         * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
         *
         * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
         */
        int8_t Max : 4;
    }Fields;
}DrRange_t;

/*!
 * LoRaMAC band parameters definition
 */
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
    /*!
     * Time stamp of the last Tx frame
     */
    //TimerTime_t LastTxDoneTime;
    unsigned int last_tx_done_us;
    /*!
     * Holds the time where the device is off
     */
    //TimerTime_t TimeOff;
}Band_t;

/*!
 * LoRaMAC channel definition
 */
typedef struct sChannelParams
{
    /*!
     * Frequency in Hz
     */
    uint32_t Frequency;
    /*!
     * Data rate definition
     */
    DrRange_t DrRange;
    /*!
     * Band index
     */
    uint8_t Band;
}ChannelParams_t;

#if 0
/*!
 * LoRaMAC receive window 2 channel parameters
 */
typedef struct sRx2ChannelParams
{
    /*!
     * Frequency in Hz
     */
    uint32_t Frequency;
    /*!
     * Data rate
     *
     * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
     *
     * US915 - [DR_8, DR_9, DR_10, DR_11, DR_12, DR_13]
     */
    uint8_t  Datarate;
}Rx2ChannelParams_t;
#endif /* #if 0 */

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
    int8_t ChannelsDatarate_fixed;
    /*!
     * System overall timing error in milliseconds. 
     * [-SystemMaxRxError : +SystemMaxRxError]
     * Default: +/-10 ms
     */
    uint32_t SystemMaxRxError_ms;
    /*!
     * Minimum required number of symbols to detect an Rx frame
     * Default: 6 symbols
     */
    uint8_t MinRxSymbols;
    /*!
     * LoRaMac maximum time a reception window stays open
     */
    uint32_t MaxRxWindow;

    uint32_t JoinAcceptDelay_us;
    uint32_t ReceiveDelay_us;

    /*!
     * Number of uplink messages repetitions [1:15] (unconfirmed messages only)
     */
    uint8_t ChannelsNbRep;
    /*!
     * Datarate offset between uplink and downlink on first window
     */
    uint8_t Rx1DrOffset;
    /*!
     * LoRaMAC 2nd reception window settings
     */
}LoRaMacParams_t;

/*!
 * LoRaMAC multicast channel parameter
 */
typedef struct sMulticastParams
{
    /*!
     * Address
     */
    uint32_t Address;
    /*!
     * Network session key
     */
    uint8_t NwkSKey[16];
    /*!
     * Application session key
     */
    uint8_t AppSKey[16];
    /*!
     * Downlink counter
     */
    uint32_t DownLinkCounter;
    /*!
     * Reference pointer to the next multicast channel parameters in the list
     */
    struct sMulticastParams *Next;
}MulticastParams_t;

/*!
 * LoRaMAC frame types
 *
 * LoRaWAN Specification V1.0.1, chapter 4.2.1, table 1
 */
typedef enum eLoRaMacFrameType
{
    /*!
     * LoRaMAC join request frame
     */
    FRAME_TYPE_JOIN_REQ              = 0x00,
    /*!
     * LoRaMAC join accept frame
     */
    FRAME_TYPE_JOIN_ACCEPT           = 0x01,
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
    /*!
     * LoRaMAC RFU frame
     */
    FRAME_TYPE_RFU                   = 0x06,
    /*!
     * LoRaMAC proprietary frame
     */
    FRAME_TYPE_PROPRIETARY           = 0x07,
}LoRaMacFrameType_t;

/*!
 * LoRaMAC mote MAC commands
 *
 * LoRaWAN Specification V1.0.1, chapter 5, table 4
 */
typedef enum eLoRaMacMoteCmd
{
    /*!
     * LinkCheckReq
     */
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
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
    /*!
     * RXTimingSetupAns
     */
    MOTE_MAC_RX_TIMING_SETUP_ANS     = 0x08,
}LoRaMacMoteCmd_t;

/*!
 * LoRaMAC server MAC commands
 *
 * LoRaWAN Specification V1.0.1 chapter 5, table 4
 */
typedef enum eLoRaMacSrvCmd
{
    /*!
     * LinkCheckAns
     */
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    /*!
     * RXParamSetupReq
     */
    //SRV_MAC_RX_PARAM_SETUP_REQ       = 0x05,
    /*!
     * DevStatusReq
     */
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    /*!
     * NewChannelReq
     */
    /*!
     * RXTimingSetupReq
     */
    SRV_MAC_RX_TIMING_SETUP_REQ      = 0x08,
}LoRaMacSrvCmd_t;

/*!
 * LoRaMAC Battery level indicator
 */
typedef enum eLoRaMacBatteryLevel
{
    /*!
     * External power source
     */
    BAT_LEVEL_EXT_SRC                = 0x00,
    /*!
     * Battery level empty
     */
    BAT_LEVEL_EMPTY                  = 0x01,
    /*!
     * Battery level full
     */
    BAT_LEVEL_FULL                   = 0xFE,
    /*!
     * Battery level - no measurement available
     */
    BAT_LEVEL_NO_MEASURE             = 0xFF,
}LoRaMacBatteryLevel_t;

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
 * LoRaMAC frame control field definition (FCtrl)
 *
 * LoRaWAN Specification V1.0.1, chapter 4.3.1
 */
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
}LoRaMacFrameCtrl_t;

/*!
 * Enumeration containing the status of the operation of a MAC service
 */
typedef enum eLoRaMacEventInfoStatus
{
    /*!
     * Service performed successfully
     */
    LORAMAC_EVENT_INFO_STATUS_OK = 0,
    /*!
     * An error occurred during the execution of the service
     */
    LORAMAC_EVENT_INFO_STATUS_ERROR_RX_MTYPE,
    LORAMAC_EVENT_INFO_STATUS_ERROR_SEND,
    LORAMAC_EVENT_INFO_STATUS_ERROR_JOIN_ACCEPT,
    LORAMAC_EVENT_INFO_STATUS_ERROR_MLMEREQ,
    LORAMAC_EVENT_INFO_STATUS_ERROR_MCPSREQ,
    /*!
     * A Tx timeout occurred
     */
    LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT,
    /*!
     * An Rx timeout occurred on receive window 2
     */
    //LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT,
    LORAMAC_EVENT_INFO_STATUS_RX_TIMEOUT,
    /*!
     * An Rx error occurred on receive window 1
     */
    //LORAMAC_EVENT_INFO_STATUS_RX1_ERROR,
    LORAMAC_EVENT_INFO_STATUS_RX_ERROR,
    /*!
     * An Rx error occurred on receive window 2
     */
    //LORAMAC_EVENT_INFO_STATUS_RX2_ERROR,
    /*!
     * An error occurred in the join procedure
     */
    LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL,
    /*!
     * A frame with an invalid downlink counter was received. The
     * downlink counter of the frame was equal to the local copy
     * of the downlink counter of the node.
     */
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED,
    /*!
     * The MAC could not retransmit a frame since the MAC decreased the datarate. The
     * payload size is not applicable for the datarate.
     */
    LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR,
    /*!
     * The node has lost MAX_FCNT_GAP or more frames.
     */
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS,
    /*!
     * An address error occurred
     */
    LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL,
    /*!
     * message integrity check failure
     */
    LORAMAC_EVENT_INFO_STATUS_MIC_FAIL,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_LOST,
    /*!
     * ToDo
     */
    LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND,
    
    LORAMAC_EVENT_INFO_STATUS_STOP
} LoRaMacEventInfoStatus_t;

/*!
 * LoRaMac tx/rx operation state
 */
typedef union eLoRaMacFlags_t
{
    /*!
     * Byte-access to the bits
     */
    uint8_t Value;
    /*!
     * Structure containing single access to bits
     */
    struct sMacFlagBits
    {
        /*!
         * MCPS-Req pending
         */
        uint16_t McpsReq         : 1; // 0
        /*!
         * MCPS-Ind pending
         */
        uint16_t McpsInd         : 1; // 1
        /*!
         * MCPS-Ind pending. Skip indication to the application layer
         */
        uint16_t McpsIndSkip     : 1; // 2
        /*!
         * MLME-Req pending
         */
        uint16_t MlmeReq         : 1; // 3
        /*!
         * MAC cycle done
         */
        uint16_t MacDone         : 1; // 4
        /*!
         * MLME-Ind pending
         */
        uint16_t MlmeInd         : 1; // 5

        uint16_t uplink_pending  : 1; // 6

        uint16_t IsLoRaMacNetworkJoined : 1; // 7
        uint16_t join_send              : 1; // 8
        uint16_t send                   : 1; // 9
        uint16_t expecting_beacon       : 1; // 10
        uint16_t reJoin                 : 1; // 11
        uint16_t NodeAckRequested       : 1; // 12
        uint16_t SrvAckRequested        : 1; // 13
    }Bits;
} LoRaMacFlags_t;

/*!
 *
 * \brief   LoRaMAC data services
 *
 * \details The following table list the primitives which are supported by the
 *          specific MAC data service:
 *
 * Name                  | Request | Indication | Response | Confirm
 * --------------------- | :-----: | :--------: | :------: | :-----:
 * \ref MCPS_UNCONFIRMED | YES     | YES        | NO       | YES
 * \ref MCPS_CONFIRMED   | YES     | YES        | NO       | YES
 * \ref MCPS_MULTICAST   | NO      | YES        | NO       | NO
 * \ref MCPS_PROPRIETARY | YES     | YES        | NO       | YES
 *
 * The following table provides links to the function implementations of the
 * related MCPS primitives:
 *
 * Primitive        | Function
 * ---------------- | :---------------------:
 * MCPS-Request     | \ref LoRaMacMlmeRequest
 * MCPS-Confirm     | MacMcpsConfirm in \ref LoRaMacPrimitives_t
 * MCPS-Indication  | MacMcpsIndication in \ref LoRaMacPrimitives_t
 */
typedef enum eMcps
{
    /*!
     * Unconfirmed LoRaMAC frame
     */
    MCPS_UNCONFIRMED,
    /*!
     * Confirmed LoRaMAC frame
     */
    MCPS_CONFIRMED,
    /*!
     * Multicast LoRaMAC frame
     */
    MCPS_MULTICAST,
    /*!
     * Proprietary frame
     */
    MCPS_PROPRIETARY,
}Mcps_t;

/*!
 * LoRaMAC MCPS-Request for an unconfirmed frame
 */
typedef struct sMcpsReqUnconfirmed
{
    /*!
     * Frame port field. Must be set if the payload is not empty. Use the
     * application specific frame port values: [1...223]
     *
     * LoRaWAN Specification V1.0.1, chapter 4.3.2
     */
    uint8_t fPort;
    /*!
     * Pointer to the buffer of the frame payload
     */
    void *fBuffer;
    /*!
     * Size of the frame payload
     */
    uint16_t fBufferSize;
    /*!
     * Uplink datarate, if ADR is off
     */
    //int8_t Datarate;
}McpsReqUnconfirmed_t;

/*!
 * LoRaMAC MCPS-Request for a confirmed frame
 */
typedef struct sMcpsReqConfirmed
{
    /*!
     * Frame port field. Must be set if the payload is not empty. Use the
     * application specific frame port values: [1...223]
     *
     * LoRaWAN Specification V1.0.1, chapter 4.3.2
     */
    uint8_t fPort;
    /*!
     * Pointer to the buffer of the frame payload
     */
    void *fBuffer;
    /*!
     * Size of the frame payload
     */
    uint16_t fBufferSize;
    /*!
     * Uplink datarate, if ADR is off
     */
    //int8_t Datarate;
    /*!
     * Number of trials to transmit the frame, if the LoRaMAC layer did not
     * receive an acknowledgment. The MAC performs a datarate adaptation,
     * according to the LoRaWAN Specification V1.0.1, chapter 19.4, according
     * to the following table:
     *
     * Transmission nb | Data Rate
     * ----------------|-----------
     * 1 (first)       | DR
     * 2               | DR
     * 3               | max(DR-1,0)
     * 4               | max(DR-1,0)
     * 5               | max(DR-2,0)
     * 6               | max(DR-2,0)
     * 7               | max(DR-3,0)
     * 8               | max(DR-3,0)
     *
     * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
     * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
     */
    uint8_t NbTrials;
}McpsReqConfirmed_t;

/*!
 * LoRaMAC MCPS-Request for a proprietary frame
 */
typedef struct sMcpsReqProprietary
{
    /*!
     * Pointer to the buffer of the frame payload
     */
    void *fBuffer;
    /*!
     * Size of the frame payload
     */
    uint16_t fBufferSize;
    /*!
     * Uplink datarate, if ADR is off
     */
    //int8_t Datarate;
}McpsReqProprietary_t;

/*!
 * LoRaMAC MCPS-Request structure
 */
typedef struct sMcpsReq
{
    /*!
     * MCPS-Request type
     */
    Mcps_t Type;

    /*!
     * MCPS-Request parameters
     */
    union uMcpsParam
    {
        /*!
         * MCPS-Request parameters for an unconfirmed frame
         */
        McpsReqUnconfirmed_t Unconfirmed;
        /*!
         * MCPS-Request parameters for a confirmed frame
         */
        McpsReqConfirmed_t Confirmed;
        /*!
         * MCPS-Request parameters for a proprietary frame
         */
        McpsReqProprietary_t Proprietary;
    }Req;
}McpsReq_t;

/*!
 * LoRaMAC MCPS-Confirm
 */
typedef struct sMcpsConfirm
{
    /*!
     * Holds the previously performed MCPS-Request
     */
    Mcps_t McpsRequest;
    /*!
     * Status of the operation
     */
    LoRaMacEventInfoStatus_t Status;
    /*!
     * Uplink datarate
     */
    //uint8_t Datarate;
    /*!
     * Transmission power
     */
    int8_t TxPower;
    /*!
     * Set if an acknowledgement was received
     */
    bool AckReceived;
    /*!
     * Provides the number of retransmissions
     */
    //uint8_t NbRetries;
    /*!
     * The transmission time on air of the frame
     *
    uint32_t TxTimeOnAir;*/
    /*!
     * The uplink counter value related to the frame
     */
    uint32_t UpLinkCounter;
    /*!
     * The uplink frequency related to the frame
     */
    uint32_t UpLinkFrequency;
}McpsConfirm_t;

/*!
 * LoRaMAC MCPS-Indication primitive
 */
typedef struct sMcpsIndication
{
    /*!
     * MCPS-Indication type
     */
    Mcps_t McpsIndication;
    /*!
     * Status of the operation
     */
    LoRaMacEventInfoStatus_t Status;
    /*!
     * Multicast
     */
    uint8_t Multicast;
    /*!
     * Application port
     */
    uint8_t Port;
    /*!
     * Downlink datarate
     */
    uint8_t RxDatarate;
    /*!
     * Frame pending status
     */
    uint8_t FramePending;
    /*!
     * Pointer to the received data stream
     */
    uint8_t *Buffer;
    /*!
     * Size of the received data stream
     */
    uint8_t BufferSize;
    /*!
     * Indicates, if data is available
     */
    bool RxData;
    /*!
     * Rssi of the received packet
     */
    int16_t Rssi;
    /*!
     * Snr of the received packet
     */
    uint8_t Snr;
    /*!
     * Receive window
     *
     * [0: Rx window 1, 1: Rx window 2]
     */
    //uint8_t RxSlot;
    /*!
     * Set if an acknowledgement was received
     */
    bool AckReceived;
    /*!
     * The downlink counter value for the received frame
     */
    uint32_t DownLinkCounter;
}McpsIndication_t;

/*!
 * \brief LoRaMAC management services
 *
 * \details The following table list the primitives which are supported by the
 *          specific MAC management service:
 *
 * Name                  | Request | Indication | Response | Confirm
 * --------------------- | :-----: | :--------: | :------: | :-----:
 * \ref MLME_JOIN        | YES     | NO         | NO       | YES
 * \ref MLME_LINK_CHECK  | YES     | NO         | NO       | YES
 * \ref MLME_TXCW        | YES     | NO         | NO       | YES
 *
 * The following table provides links to the function implementations of the
 * related MLME primitives.
 *
 * Primitive        | Function
 * ---------------- | :---------------------:
 * MLME-Request     | \ref LoRaMacMlmeRequest
 * MLME-Confirm     | MacMlmeConfirm in \ref LoRaMacPrimitives_t
 */
typedef enum eMlme
{
    /*!
     * Initiates the Over-the-Air activation
     *
     * LoRaWAN Specification V1.0.1, chapter 6.2
     */
    MLME_JOIN,
    /*!
     * LinkCheckReq - Connectivity validation
     *
     * LoRaWAN Specification V1.0.1, chapter 5, table 4
     */
    MLME_LINK_CHECK,
    /*!
     * Sets Tx continuous wave mode
     *
     * LoRaWAN end-device certification
     */
    MLME_TXCW,
    /*!
     * Sets Tx continuous wave mode (new LoRa-Alliance CC definition)
     *
     * LoRaWAN end-device certification
     */
    MLME_TXCW_1,
    
    MLME_TXDONE,

    MLME_BEACON
}Mlme_t;

/*!
 * LoRaMAC MLME-Request for the join service
 */
typedef struct sMlmeReqJoin
{
    /*!
     * Globally unique end-device identifier
     *
     * LoRaWAN Specification V1.0.1, chapter 6.2.1
     */
    uint8_t *DevEui;
    /*!
     * Application identifier
     *
     * LoRaWAN Specification V1.0.1, chapter 6.1.2
     */
    uint8_t *AppEui;
    /*!
     * AES-128 application key
     *
     * LoRaWAN Specification V1.0.1, chapter 6.2.2
     */
    uint8_t *AppKey;
    /*!
     * Number of trials for the join request.
     */
    uint8_t NbTrials;
}MlmeReqJoin_t;

/*!
 * LoRaMAC MLME-Request for Tx continuous wave mode
 */
typedef struct sMlmeReqTxCw
{
    /*!
     * Time in seconds while the radio is kept in continuous wave mode
     */
    uint16_t Timeout;
    /*!
     * RF frequency to set (Only used with new way)
     */
    uint32_t Frequency;
    /*!
     * RF output power to set (Only used with new way)
     */
    uint8_t Power;
}MlmeReqTxCw_t;

/*!
 * LoRaMAC MLME-Request structure
 */
typedef struct sMlmeReq
{
    /*!
     * MLME-Request type
     */
    Mlme_t Type;

    /*!
     * MLME-Request parameters
     */
    union uMlmeParam
    {
        /*!
         * MLME-Request parameters for a join request
         */
        MlmeReqJoin_t Join;
        /*!
         * MLME-Request parameters for Tx continuous mode request
         */
        MlmeReqTxCw_t TxCw;
    }Req;
}MlmeReq_t;

/*!
 * LoRaMAC MLME-Confirm primitive
 */
typedef struct sMlmeConfirm
{
    /*!
     * Holds the previously performed MLME-Request
     */
    Mlme_t MlmeRequest;
    /*!
     * Status of the operation
     */
    LoRaMacEventInfoStatus_t Status;
    /*!
     * The transmission time on air of the frame
     *
    uint32_t TxTimeOnAir;*/
    /*!
     * Demodulation margin. Contains the link margin [dB] of the last
     * successfully received LinkCheckReq
     */
    uint8_t DemodMargin;
    /*!
     * Number of gateways which received the last LinkCheckReq
     */
    uint8_t NbGateways;
    /*!
     * Provides the number of retransmissions
     */
    //uint8_t NbRetries;
}MlmeConfirm_t;

typedef struct sMlmeIndication
{
    /*!
     * Holds the previously performed MLME-Request
     */
    Mlme_t MlmeIndication;
    /*!
     * Status of the operation
     */
    LoRaMacEventInfoStatus_t Status;

    //BeaconInfo_t BeaconInfo;
}MlmeIndication_t;

/*!
 * LoRa Mac Information Base (MIB)
 *
 * The following table lists the MIB parameters and the related attributes:
 *
 * Attribute                         | Get | Set
 * --------------------------------- | :-: | :-:
 * \ref MIB_DEVICE_CLASS             | YES | YES
 * \ref MIB_NETWORK_JOINED           | YES | YES
 * \ref MIB_ADR                      | YES | YES
 * \ref MIB_NET_ID                   | YES | YES
 * \ref MIB_DEV_ADDR                 | YES | YES
 * \ref MIB_NWK_SKEY                 | YES | YES
 * \ref MIB_APP_SKEY                 | YES | YES
 * \ref MIB_PUBLIC_NETWORK           | YES | YES
 * \ref MIB_REPEATER_SUPPORT         | YES | YES
 * \ref MIB_CHANNELS                 | YES | NO
 * \ref MIB_RX2_CHANNEL              | YES | YES
 * \ref MIB_CHANNELS_MASK            | YES | YES
 * \ref MIB_CHANNELS_DEFAULT_MASK    | YES | YES
 * \ref MIB_CHANNELS_NB_REP          | YES | YES
 * \ref MIB_MAX_RX_WINDOW_DURATION   | YES | YES
 * \ref MIB_RECEIVE_DELAY_1          | YES | YES
 * \ref MIB_RECEIVE_DELAY_2          | YES | YES
 * \ref MIB_JOIN_ACCEPT_DELAY_1      | YES | YES
 * \ref MIB_JOIN_ACCEPT_DELAY_2      | YES | YES
 * \ref MIB_CHANNELS_DATARATE        | YES | YES
 * \ref MIB_CHANNELS_DEFAULT_DATARATE| YES | YES
 * \ref MIB_CHANNELS_TX_POWER        | YES | YES
 * \ref MIB_CHANNELS_DEFAULT_TX_POWER| YES | YES
 * \ref MIB_UPLINK_COUNTER           | YES | YES
 * \ref MIB_DOWNLINK_COUNTER         | YES | YES
 * \ref MIB_MULTICAST_CHANNEL        | YES | NO
 * \ref MIB_SYSTEM_MAX_RX_ERROR      | YES | YES
 * \ref MIB_MIN_RX_SYMBOLS           | YES | YES
 *
 * The following table provides links to the function implementations of the
 * related MIB primitives:
 *
 * Primitive        | Function
 * ---------------- | :---------------------:
 * MIB-Set          | \ref LoRaMacMibSetRequestConfirm
 * MIB-Get          | \ref LoRaMacMibGetRequestConfirm
 */
typedef enum eMib
{
    /*!
     * LoRaWAN device class
     *
     * LoRaWAN Specification V1.0.1
     */
    //MIB_DEVICE_CLASS,
    /*!
     * LoRaWAN Network joined attribute
     *
     * LoRaWAN Specification V1.0.1
     */
    MIB_NETWORK_JOINED,
    /*!
     * Adaptive data rate
     *
     * LoRaWAN Specification V1.0.1, chapter 4.3.1.1
     *
     * [true: ADR enabled, false: ADR disabled]
     */
    /*!
     * Network identifier
     *
     * LoRaWAN Specification V1.0.1, chapter 6.1.1
     */
    MIB_NET_ID,
    /*!
     * End-device address
     *
     * LoRaWAN Specification V1.0.1, chapter 6.1.1
     */
    MIB_DEV_ADDR,
    /*!
     * Network session key
     *
     * LoRaWAN Specification V1.0.1, chapter 6.1.3
     */
    MIB_NWK_SKEY,
    /*!
     * Application session key
     *
     * LoRaWAN Specification V1.0.1, chapter 6.1.4
     */
    MIB_APP_SKEY,
    /*!
     * Set the network type to public or private
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * [true: public network, false: private network]
     */
    MIB_PUBLIC_NETWORK,
    /*!
     * Support the operation with repeaters
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * [true: repeater support enabled, false: repeater support disabled]
     */
    //MIB_REPEATER_SUPPORT,
    /*!
     * Communication channels. A get request will return a
     * pointer which references the first entry of the channel list. The
     * list is of size LORA_MAX_NB_CHANNELS
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    //MIB_CHANNELS,
    /*!
     * Set receive window 2 channel
     *
     * LoRaWAN Specification V1.0.1, chapter 3.3.2
     */
    /*!
     * Set receive window 2 channel
     *
     * LoRaWAN Specification V1.0.1, chapter 3.3.2
     */
    /*!
     * LoRaWAN channels mask
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    /*!
     * LoRaWAN default channels mask
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    //MIB_CHANNELS_DEFAULT_MASK,
    /*!
     * Set the number of repetitions on a channel
     *
     * LoRaWAN Specification V1.0.1, chapter 5.2
     */
    MIB_CHANNELS_NB_REP,
    /*!
     * Maximum receive window duration in [ms]
     *
     * LoRaWAN Specification V1.0.1, chapter 3.3.3
     */
    MIB_MAX_RX_WINDOW_DURATION,
    /*!
     * Receive delay 1 in [ms]
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    MIB_RECEIVE_DELAY_1,
    /*!
     * Receive delay 2 in [ms]
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    /*!
     * Join accept delay 1 in [ms]
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    MIB_JOIN_ACCEPT_DELAY_1,
    /*!
     * Join accept delay 2 in [ms]
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     */
    /*!
     * Default Data rate of a channel
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
     *
     * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_8, DR_9, DR_10, DR_11, DR_12, DR_13]
     */
    /*!
     * Data rate of a channel
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
     *
     * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_8, DR_9, DR_10, DR_11, DR_12, DR_13]
     */
    /*!
     * Transmission power of a channel
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * EU868 - [TX_POWER_20_DBM, TX_POWER_14_DBM, TX_POWER_11_DBM,
     *          TX_POWER_08_DBM, TX_POWER_05_DBM, TX_POWER_02_DBM]
     *
     * US915 - [TX_POWER_30_DBM, TX_POWER_28_DBM, TX_POWER_26_DBM,
     *          TX_POWER_24_DBM, TX_POWER_22_DBM, TX_POWER_20_DBM,
     *          TX_POWER_18_DBM, TX_POWER_14_DBM, TX_POWER_12_DBM,
     *          TX_POWER_10_DBM]
     */
    MIB_CHANNELS_TX_POWER,
    /*!
     * Transmission power of a channel
     *
     * LoRaWAN Specification V1.0.1, chapter 7
     *
     * EU868 - [TX_POWER_20_DBM, TX_POWER_14_DBM, TX_POWER_11_DBM,
     *          TX_POWER_08_DBM, TX_POWER_05_DBM, TX_POWER_02_DBM]
     *
     * US915 - [TX_POWER_30_DBM, TX_POWER_28_DBM, TX_POWER_26_DBM,
     *          TX_POWER_24_DBM, TX_POWER_22_DBM, TX_POWER_20_DBM,
     *          TX_POWER_18_DBM, TX_POWER_14_DBM, TX_POWER_12_DBM,
     *          TX_POWER_10_DBM]
     */
    MIB_CHANNELS_DEFAULT_TX_POWER,
    /*!
     * LoRaWAN Up-link counter
     *
     * LoRaWAN Specification V1.0.1, chapter 4.3.1.5
     */
    MIB_UPLINK_COUNTER,
    /*!
     * LoRaWAN Down-link counter
     *
     * LoRaWAN Specification V1.0.1, chapter 4.3.1.5
     */
    MIB_DOWNLINK_COUNTER,
    /*!
     * Multicast channels. A get request will return a pointer to the first
     * entry of the multicast channel linked list. If the pointer is equal to
     * NULL, the list is empty.
     */
    MIB_MULTICAST_CHANNEL,
    /*!
     * System overall timing error in milliseconds. 
     * [-SystemMaxRxError : +SystemMaxRxError]
     * Default: +/-10 ms
     */
    MIB_SYSTEM_MAX_RX_ERROR,
    /*!
     * Minimum required number of symbols to detect an Rx frame
     * Default: 6 symbols
     */
    MIB_MIN_RX_SYMBOLS,
}Mib_t;

/*!
 * LoRaMAC MIB parameters
 */
typedef union uMibParam
{
    /*!
     * LoRaWAN device class
     *
     * Related MIB type: \ref MIB_DEVICE_CLASS
     */
    DeviceClass_t Class;
    /*!
     * LoRaWAN network joined attribute
     *
     * Related MIB type: \ref MIB_NETWORK_JOINED
     */
    bool IsNetworkJoined;
    /*!
     * Activation state of ADR
     *
     * Related MIB type: \ref MIB_ADR
     */
    /*!
     * Network identifier
     *
     * Related MIB type: \ref MIB_NET_ID
     */
    uint32_t NetID;
    /*!
     * End-device address
     *
     * Related MIB type: \ref MIB_DEV_ADDR
     */
    uint32_t DevAddr;
    /*!
     * Network session key
     *
     * Related MIB type: \ref MIB_NWK_SKEY
     */
    uint8_t *NwkSKey;
    /*!
     * Application session key
     *
     * Related MIB type: \ref MIB_APP_SKEY
     */
    uint8_t *AppSKey;
    /*!
     * Enable or disable a public network
     *
     * Related MIB type: \ref MIB_PUBLIC_NETWORK
     */
    bool EnablePublicNetwork;
    /*!
     * Number of frame repetitions
     *
     * Related MIB type: \ref MIB_CHANNELS_NB_REP
     */
    uint8_t ChannelNbRep;
    /*!
     * Maximum receive window duration
     *
     * Related MIB type: \ref MIB_MAX_RX_WINDOW_DURATION
     */
    uint32_t MaxRxWindow;

    uint32_t ReceiveDelay;
    uint32_t JoinAcceptDelay;

    /*!
     * Channels data rate
     *
     * Related MIB type: \ref MIB_CHANNELS_DEFAULT_DATARATE
     */
    //int8_t ChannelsDefaultDatarate;
    /*!
     * Channels data rate
     *
     * Related MIB type: \ref MIB_CHANNELS_DATARATE
     */
    //int8_t ChannelsDatarate;
    /*!
     * Channels TX power
     *
     * Related MIB type: \ref MIB_CHANNELS_DEFAULT_TX_POWER
     */
    int8_t ChannelsDefaultTxPower;
    /*!
     * Channels TX power
     *
     * Related MIB type: \ref MIB_CHANNELS_TX_POWER
     */
    int8_t ChannelsTxPower;
    /*!
     * LoRaWAN Up-link counter
     *
     * Related MIB type: \ref MIB_UPLINK_COUNTER
     */
    uint32_t UpLinkCounter;
    /*!
     * LoRaWAN Down-link counter
     *
     * Related MIB type: \ref MIB_DOWNLINK_COUNTER
     */
    uint32_t DownLinkCounter;
    /*!
     * Multicast channel
     *
     * Related MIB type: \ref MIB_MULTICAST_CHANNEL
     */
    MulticastParams_t* MulticastList;
    /*!
     * System overall timing error in milliseconds. 
     *
     * Related MIB type: \ref MIB_SYSTEM_MAX_RX_ERROR
     */
    uint32_t SystemMaxRxError_ms;
    /*!
     * Minimum required number of symbols to detect an Rx frame
     *
     * Related MIB type: \ref MIB_MIN_RX_SYMBOLS
     */
    uint8_t MinRxSymbols;
}MibParam_t;

/*!
 * LoRaMAC MIB-RequestConfirm structure
 */
typedef struct eMibRequestConfirm
{
    /*!
     * MIB-Request type
     */
    Mib_t Type;

    /*!
     * MLME-RequestConfirm parameters
     */
    MibParam_t Param;
}MibRequestConfirm_t;

/*!
 * LoRaMAC tx information
 */
typedef struct sLoRaMacTxInfo
{
    /*!
     * Defines the size of the applicative payload which can be processed
     */
    uint8_t MaxPossiblePayload;
    /*!
     * The current payload size, dependent on the current datarate
     */
    uint8_t CurrentPayloadSize;
}LoRaMacTxInfo_t;

/*!
 * LoRaMAC Status
 */
typedef enum eLoRaMacStatus
{
    /*!
     * Service started successfully
     */
    LORAMAC_STATUS_OK,
    /*!
     * Service not started - LoRaMAC is busy
     */
    LORAMAC_STATUS_BUSY,
    /*!
     * Service unknown
     */
    LORAMAC_STATUS_SERVICE_UNKNOWN,
    /*!
     * Service not started - invalid parameter
     */
    LORAMAC_STATUS_PARAMETER_INVALID,
    /*!
     * Service not started - invalid frequency
     */
    LORAMAC_STATUS_FREQUENCY_INVALID,
    /*!
     * Service not started - invalid datarate
     */
    LORAMAC_STATUS_DATARATE_INVALID,
    /*!
     * Service not started - invalid frequency and datarate
     */
    LORAMAC_STATUS_FREQ_AND_DR_INVALID,
    /*!
     * Service not started - the device is not in a LoRaWAN
     */
    LORAMAC_STATUS_NO_NETWORK_JOINED,
    /*!
     * Service not started - payload lenght error
     */
    LORAMAC_STATUS_LENGTH_ERROR,
    /*!
     * Service not started - payload lenght error
     */
    LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR,
    /*!
     * Service not started - the device is switched off
     */
    LORAMAC_STATUS_DEVICE_OFF,
} LoRaMacStatus_t;

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
    void ( *MacMcpsConfirm )( McpsConfirm_t *McpsConfirm );
    /*!
     * \brief   MCPS-Indication primitive
     *
     * \param   [OUT] MCPS-Indication parameters
     */
    void ( *MacMcpsIndication )( McpsIndication_t *McpsIndication );
    /*!
     * \brief   MLME-Confirm primitive
     *
     * \param   [OUT] MLME-Confirm parameters
     */
    void ( *MacMlmeConfirm )( MlmeConfirm_t *MlmeConfirm );

    /*!
     * \brief   MLME-Indication primitive
     *
     * \param   [OUT] MLME-Indication parameters
     */
    void ( *MacMlmeIndication )( MlmeIndication_t *MlmeIndication );
}LoRaMacPrimitives_t;

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
    uint8_t ( *GetBatteryLevel )( void );
}LoRaMacCallback_t;

/*!
 * \brief   LoRaMAC layer initialization
 *
 * \details In addition to the initialization of the LoRaMAC layer, this
 *          function initializes the callback primitives of the MCPS and
 *          MLME services. Every data field of \ref LoRaMacPrimitives_t must be
 *          set to a valid callback function.
 *
 * \param   [IN] events - Pointer to a structure defining the LoRaMAC
 *                        event functions. Refer to \ref LoRaMacPrimitives_t.
 *
 * \param   [IN] events - Pointer to a structure defining the LoRaMAC
 *                        callback functions. Refer to \ref LoRaMacCallback_t.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacInitialization( LoRaMacPrimitives_t *primitives, LoRaMacCallback_t *callbacks );

/*!
 * \brief   Queries the LoRaMAC if it is possible to send the next frame with
 *          a given payload size. The LoRaMAC takes scheduled MAC commands into
 *          account and reports, when the frame can be send or not.
 *
 * \param   [IN] size - Size of applicative payload to be send next
 *
 * \param   [OUT] txInfo - The structure \ref LoRaMacTxInfo_t contains
 *                         information about the actual maximum payload possible
 *                         ( according to the configured datarate or the next
 *                         datarate according to ADR ), and the maximum frame
 *                         size, taking the scheduled MAC commands into account.
 *
 * \retval  LoRaMacStatus_t Status of the operation. When the parameters are
 *          not valid, the function returns \ref LORAMAC_STATUS_PARAMETER_INVALID.
 *          In case of a length error caused by the applicative payload size, the
 *          function returns LORAMAC_STATUS_LENGTH_ERROR. In case of a length error
 *          due to additional MAC commands in the queue, the function returns
 *          LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR. In case the query is valid, and
 *          the LoRaMAC is able to send the frame, the function returns LORAMAC_STATUS_OK. *
 */
LoRaMacStatus_t LoRaMacQueryTxPossible( uint8_t size, LoRaMacTxInfo_t* txInfo );

/*!
 * \brief   LoRaMAC channel add service
 *
 * \details Adds a new channel to the channel list and activates the id in
 *          the channel mask. For the US915 band, all channels are enabled
 *          by default. It is not possible to activate less than 6 125 kHz
 *          channels.
 *
 * \param   [IN] id - Id of the channel. Possible values are:
 *
 *          0-15 for EU868
 *          0-72 for US915
 *
 * \param   [IN] params - Channel parameters to set.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params );

/*!
 * \brief   LoRaMAC channel remove service
 *
 * \details Deactivates the id in the channel mask.
 *
 * \param   [IN] id - Id of the channel.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacChannelRemove( uint8_t id );

/*!
 * \brief   LoRaMAC multicast channel link service
 *
 * \details Links a multicast channel into the linked list.
 *
 * \param   [IN] channelParam - Multicast channel parameters to link.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacMulticastChannelLink( MulticastParams_t *channelParam );

/*!
 * \brief   LoRaMAC multicast channel unlink service
 *
 * \details Unlinks a multicast channel from the linked list.
 *
 * \param   [IN] channelParam - Multicast channel parameters to unlink.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacMulticastChannelUnlink( MulticastParams_t *channelParam );

/*!
 * \brief   LoRaMAC MIB-Get
 *
 * \details The mac information base service to get attributes of the LoRaMac
 *          layer.
 *
 *          The following code-snippet shows how to use the API to get the
 *          parameter AdrEnable, defined by the enumeration type
 *          \ref MIB_ADR.
 * \code
 * MibRequestConfirm_t mibReq;
 * mibReq.Type = MIB_ADR;
 *
 * if( LoRaMacMibGetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
 * {
 *   // LoRaMAC updated the parameter mibParam.AdrEnable
 * }
 * \endcode
 *
 * \param   [IN] mibRequest - MIB-GET-Request to perform. Refer to \ref MibRequestConfirm_t.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_SERVICE_UNKNOWN,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet );

/*!
 * \brief   LoRaMAC MIB-Set
 *
 * \details The mac information base service to set attributes of the LoRaMac
 *          layer.
 *
 *          The following code-snippet shows how to use the API to set the
 *          parameter AdrEnable, defined by the enumeration type
 *          \ref MIB_ADR.
 *
 * \code
 * MibRequestConfirm_t mibReq;
 * mibReq.Type = MIB_ADR;
 * mibReq.Param.AdrEnable = true;
 *
 * if( LoRaMacMibGetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
 * {
 *   // LoRaMAC updated the parameter
 * }
 * \endcode
 *
 * \param   [IN] mibRequest - MIB-SET-Request to perform. Refer to \ref MibRequestConfirm_t.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_SERVICE_UNKNOWN,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
LoRaMacStatus_t LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet );

/*!
 * \brief   LoRaMAC MLME-Request
 *
 * \details The Mac layer management entity handles management services. The
 *          following code-snippet shows how to use the API to perform a
 *          network join request.
 *
 * \code
 * static uint8_t DevEui[] =
 * {
 *   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * };
 * static uint8_t AppEui[] =
 * {
 *   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
 * };
 * static uint8_t AppKey[] =
 * {
 *   0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
 *   0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
 * };
 *
 * MlmeReq_t mlmeReq;
 * mlmeReq.Type = MLME_JOIN;
 * mlmeReq.Req.Join.DevEui = DevEui;
 * mlmeReq.Req.Join.AppEui = AppEui;
 * mlmeReq.Req.Join.AppKey = AppKey;
 *
 * if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
 * {
 *   // Service started successfully. Waiting for the Mlme-Confirm event
 * }
 * \endcode
 *
 * \param   [IN] mlmeRequest - MLME-Request to perform. Refer to \ref MlmeReq_t.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_SERVICE_UNKNOWN,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID,
 *          \ref LORAMAC_STATUS_NO_NETWORK_JOINED,
 *          \ref LORAMAC_STATUS_LENGTH_ERROR,
 *          \ref LORAMAC_STATUS_DEVICE_OFF.
 */
LoRaMacStatus_t LoRaMacMlmeRequest( MlmeReq_t *mlmeRequest );

/*!
 * \brief   LoRaMAC MCPS-Request
 *
 * \details The Mac Common Part Sublayer handles data services. The following
 *          code-snippet shows how to use the API to send an unconfirmed
 *          LoRaMAC frame.
 *
 * \code
 * uint8_t myBuffer[] = { 1, 2, 3 };
 *
 * McpsReq_t mcpsReq;
 * mcpsReq.Type = MCPS_UNCONFIRMED;
 * mcpsReq.Req.Unconfirmed.fPort = 1;
 * mcpsReq.Req.Unconfirmed.fBuffer = myBuffer;
 * mcpsReq.Req.Unconfirmed.fBufferSize = sizeof( myBuffer );
 *
 * if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
 * {
 *   // Service started successfully. Waiting for the MCPS-Confirm event
 * }
 * \endcode
 *
 * \param   [IN] mcpsRequest - MCPS-Request to perform. Refer to \ref McpsReq_t.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_SERVICE_UNKNOWN,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID,
 *          \ref LORAMAC_STATUS_NO_NETWORK_JOINED,
 *          \ref LORAMAC_STATUS_LENGTH_ERROR,
 *          \ref LORAMAC_STATUS_DEVICE_OFF.
 */
LoRaMacStatus_t LoRaMacMcpsRequest( McpsReq_t *mcpsRequest );

/*! \} defgroup LORAMAC */

void loramac_print_status(void);

void LoRaMacBottomHalf(void);

#endif // __LORAMAC_H__
