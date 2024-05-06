
#include "LoRaMacPrivate.h"
#include "LoRaMacCrypto.h"

//#define ADR_ACK_LIMIT                               64
//#define ADR_ACK_DELAY                               32
#define LORA_MAC_FRMPAYLOAD_OVERHEAD                13 // MHDR(1) + FHDR(7) + Port(1) + MIC(4)
#define RECEIVE_DELAY1_us                           1000000
#define LORAMAC_MFR_LEN                             4

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
 * LoRaMac internal state
 */
flags_t flags;

void (*function_pending)(void); // one-shot

/*!
 * Current channel index
 */
uint8_t Channel;

static MlmeIndication_t MlmeIndication;
static MlmeConfirm_t MlmeConfirm;
static McpsIndication_t McpsIndication;
static McpsConfirm_t McpsConfirm;

uint32_t LoRaMacDevAddr;
uint32_t LoRaMacNetID;
static skey_t keys;

uint8_t MacCommandsBufferToRepeatIndex;
uint8_t MacCommandsBufferIndex;
uint8_t MacCommandsBuffer[LORA_MAC_COMMAND_MAX_LENGTH];
static uint8_t MacCommandsBufferToRepeat[LORA_MAC_COMMAND_MAX_LENGTH];

LoRaMacParams_t LoRaMacParams;

#ifdef LORAWAN_JOIN_EUI
    static const uint8_t *LoRaMacDevEui;
    static const uint8_t *LoRaMacJoinEui;
    static const uint8_t *RootNwkKey;
    static const uint8_t *RootAppKey;
    static uint8_t MaxJoinRequestTrials;
    static uint8_t JSEncKey[16]; // TODO move to keys
    static uint8_t JSIntKey[16]; // TODO move to keys
    static uint8_t JoinReqType;
    static uint16_t LoRaMacDevNonce;
    uint16_t RJcount0;

    static uint32_t FCntUp;
    static uint32_t NFCntDown;  /**< set to next expected value */
    static uint32_t AFCntDown;  /**< set to next expected value */
#endif /* LORAWAN_JOIN_EUI  */

static uint16_t ADR_ACK_LIMIT;
static uint16_t ADR_ACK_DELAY;

DeviceClass_t LoRaMacDeviceClass;
uint8_t tx_buf_len;
const LoRaMacHeader_t* uplinkMHDR = (LoRaMacHeader_t*)&Radio::radio.tx_buf[0];

static uint32_t RxWindow1Delay_us;
static uint32_t RxWindow2Delay_us;

LoRaMacHeader_t last_up_macHdr;
static uint8_t rxFRMPayload[244];

static const LoRaMacPrimitives_t *LoRaMacPrimitives;
static const LoRaMacCallback_t *LoRaMacCallbacks;

/*!
 * LoRaMAC frame counter. Each time a packet is received the counter is incremented.
 * Only the 16 LSB bits are received
 */
static uint16_t ConfFCntDown;
static uint16_t ConfFCntUp;

static void PrepareRxDoneAbort(LoRaMacEventInfoStatus_t);

void OnRadioRxTimeout(void);

LoRaMacStatus_t
AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    // The maximum buffer length must take MAC commands to re-send into account.
    uint8_t bufLen = LORA_MAC_COMMAND_MAX_LENGTH - MacCommandsBufferToRepeatIndex;

    MAC_PRINTF("AddMacCommand(%02x, %02x, %02x)\r\n", cmd, p1, p2);
    switch( cmd )
    {
        case MOTE_MAC_LINK_CHECK_REQ:
        case MOTE_MAC_DEVICE_TIME_REQ:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this command
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_LINK_ADR_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MAC_PRINTF("LINK_ADR_ANS %02x ", p1);
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DUTY_CYCLE_ANS:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Status: Datarate ACK, Channel ACK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DEV_STATUS_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 2 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // 1st byte Battery
                // 2nd byte Margin
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                MacCommandsBuffer[MacCommandsBufferIndex++] = p2;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_NEW_CHANNEL_ANS:
            if( MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // Status: Datarate range OK, Channel frequency OK
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
#ifdef LORAWAN_JOIN_EUI
        case MOTE_MAC_REKEY_IND:
            if( MacCommandsBufferIndex < bufLen-1 )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // minor version:
                if (RootAppKey == NULL)
                    MacCommandsBuffer[MacCommandsBufferIndex++] = 0;    // lorawan1v0
                else
                    MacCommandsBuffer[MacCommandsBufferIndex++] = 1;    // lorawan1v1

                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_REJOIN_PARAM_ANS:
            if( MacCommandsBufferIndex < bufLen-1 )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
#else
        case MOTE_MAC_RESET_IND:
            if( MacCommandsBufferIndex < bufLen-1 )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // minor version:
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
#endif /* !LORAWAN_JOIN_EUI  */
        case MOTE_MAC_MODE_IND:
            if( MacCommandsBufferIndex < bufLen-1 )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                MacCommandsBuffer[MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        default:
            MAC_PRINTF("unknown-addCmd %02x\r\n", cmd);
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    } // ..switch( cmd )

    if( status == LORAMAC_STATUS_OK )
    {
        flags.MacCommandsInNextTx = true;
    }
    return status;
} // ..AddMacCommand()


LoRaMacStatus_t SetTxContinuousWave( uint16_t timeout )
{
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;

    txPowerIndex = region_LimitTxPower( LoRaMacParams.ChannelsTxPower );
    txPower = TxPowers[txPowerIndex];

    Radio::SetTxContinuousWave( Channels[Channel].FreqHz, txPower, timeout );

    flags.uplink_in_progress = 1;

    return LORAMAC_STATUS_OK;
}

__attribute__((weak)) LoRaMacStatus_t
LoRaMacMlmeRequestClassB( const MlmeReq_t *mlmeRequest )
{
    return LORAMAC_STATUS_SERVICE_UNKNOWN;
}

__attribute__((weak)) LoRaMacStatus_t
AddMacCommandClassB( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
    return LORAMAC_STATUS_SERVICE_UNKNOWN;
}

__attribute__((weak)) void
ResetMacParametersClassB()
{
}

static void ResetMacParameters( void )
{
#ifdef LORAWAN_JOIN_EUI
    flags.IsLoRaMacNetworkJoined = 0;
    NFCntDown = 0;
    AFCntDown = 0;
#endif /* LORAWAN_JOIN_EUI  */

#ifdef DUTY_ENABLE
    DutyInit();
#endif /* DUTY_ENABLE */

    MacCommandsBufferIndex = 0;
    MacCommandsBufferToRepeatIndex = 0;

    //IsRxWindowsEnabled = true;

    LoRaMacParams.ChannelsTxPower = LoRaMacParamsDefaults.ChannelsTxPower;
    LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;

    LoRaMacParams.MaxRxWindow_us = LoRaMacParamsDefaults.MaxRxWindow_us;
    LoRaMacParams.ReceiveDelay1_us = LoRaMacParamsDefaults.ReceiveDelay1_us;
    LoRaMacParams.ReceiveDelay2_us = LoRaMacParamsDefaults.ReceiveDelay2_us;
#ifdef LORAWAN_JOIN_EUI
    LoRaMacParams.JoinAcceptDelay1_us = LoRaMacParamsDefaults.JoinAcceptDelay1_us;
    LoRaMacParams.JoinAcceptDelay2_us = LoRaMacParamsDefaults.JoinAcceptDelay2_us;
#endif /* LORAWAN_JOIN_EUI */

    LoRaMacParams.Rx1DrOffset = LoRaMacParamsDefaults.Rx1DrOffset;
    LoRaMacParams.NbTrans = LoRaMacParamsDefaults.NbTrans;

    LoRaMacParams.Rx2Channel = LoRaMacParamsDefaults.Rx2Channel;

    memcpy( ( uint8_t* ) LoRaMacParams.ChannelsMask, ( uint8_t* ) LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
    LoRaMacParams.NbEnabledChannels = LoRaMacParamsDefaults.NbEnabledChannels;

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    memcpy( ( uint8_t* ) ChannelsMaskRemaining, ( uint8_t* ) LoRaMacParamsDefaults.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
#endif

    LoRaMacParams.MaxListenTime = LoRaMacParamsDefaults.MaxListenTime;


    flags.SrvAckRequested = false;
    flags.MacCommandsInNextTx = false;

    // Initialize channel index.
    Channel = LORA_MAX_NB_CHANNELS;

    ResetMacParametersClassB();

} // ..ResetMacParameters()

static bool ValidatePayloadLength( uint8_t lenN, int8_t datarate, uint8_t fOptsLen )
{
    uint16_t maxN = 0;
    uint8_t payloadSize;

    // Get the maximum payload length
    maxN = MaxPayloadOfDatarate[datarate];

    // Calculate the resulting payload size
    payloadSize = ( lenN + fOptsLen );

    // Validation of the application payload size
    if( payloadSize <= maxN )
    {
        return true;
    }
    return false;
}

static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut )
{
    uint8_t i = 0;
    uint8_t cmdCount = 0;

    if( ( cmdBufIn == NULL ) || ( cmdBufOut == NULL ) )
    {
        return 0;
    }

    for( i = 0; i < length; i++ )
    {
        switch( cmdBufIn[i] )
        {
            // STICKY
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i++];
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            case MOTE_MAC_RX_TIMING_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            // NON-STICKY
            case MOTE_MAC_DEV_STATUS_ANS:
            { // 2 bytes payload
                i += 2;
                break;
            }
            case MOTE_MAC_MODE_IND:
            case MOTE_MAC_LINK_ADR_ANS:
            case MOTE_MAC_NEW_CHANNEL_ANS:
            { // 1 byte payload
                i++;
                break;
            }
            case SRV_MAC_ADR_PARAM_SETUP_ANS:
            case MOTE_MAC_DUTY_CYCLE_ANS:
            case MOTE_MAC_LINK_CHECK_REQ:
            { // 0 byte payload
                break;
            }
            default:
                break;
        }
    }

    return cmdCount;
} // ..ParseMacCommandsToRepeat()

LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    uint32_t fcnt_up;
    uint16_t i;
#ifdef LORAWAN_JOIN_EUI
    uint32_t mic = 0;
#endif /* LORAWAN_JOIN_EUI  */
    const void* payload = fBuffer;
    uint8_t framePort = fPort;
    uint8_t LoRaMacTxPayloadLen = 0;

    tx_buf_len = 0;

    if( fBuffer == NULL )
    {
        fBufferSize = 0;
    }

    LoRaMacTxPayloadLen = fBufferSize;

    Radio::radio.tx_buf[tx_buf_len++] = macHdr->Value;

    switch( macHdr->Bits.MType )
    {
#ifdef LORAWAN_JOIN_EUI
        case FRAME_TYPE_JOIN_REQ:
            if (LoRaMacJoinEui == NULL || LoRaMacDevEui == NULL)
                return LORAMAC_STATUS_PARAMETER_INVALID;

            RxWindow1Delay_us = LoRaMacParams.JoinAcceptDelay1_us - RADIO_WAKEUP_TIME_us;
            RxWindow2Delay_us = LoRaMacParams.JoinAcceptDelay2_us - RADIO_WAKEUP_TIME_us;

            memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacJoinEui, 8 );
            tx_buf_len += 8;
            memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacDevEui, 8 );
            tx_buf_len += 8;

            if (RootAppKey == NULL)
                LoRaMacDevNonce = Radio::Random();  /* lorawan 1.0 */
            else {
                LoRaMacDevNonce = eeprom_read(EEPROM_DEVNONCE);
                /* joinReq DevNonce value is never re-used in 1v1 */
                if (eeprom_increment_value(EEPROM_DEVNONCE) < 0)
                    return LORAMAC_STATUS_EEPROM_FAIL;
            }
            MAC_PRINTF("DevNonce:%u ", LoRaMacDevNonce);

            Radio::radio.tx_buf[tx_buf_len++] = LoRaMacDevNonce & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

            if (LoRaMacJoinComputeMic(false, Radio::radio.tx_buf, tx_buf_len & 0xFF, RootNwkKey, &mic) < 0)
                return LORAMAC_STATUS_SERVICE_UNKNOWN;

            Radio::radio.tx_buf[tx_buf_len++] = mic & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 24 ) & 0xFF;

            break;
        case FRAME_TYPE_REJOIN_REQ:
            RxWindow1Delay_us = LoRaMacParams.JoinAcceptDelay1_us - RADIO_WAKEUP_TIME_us;
            RxWindow2Delay_us = LoRaMacParams.JoinAcceptDelay2_us - RADIO_WAKEUP_TIME_us;

            Radio::radio.tx_buf[tx_buf_len++] = JoinReqType;

            tx_buf_len = tx_buf_len;

            if (JoinReqType == 0 || JoinReqType == 2) {
                LoRaMacDevNonce = RJcount0++;
                /* NetID + DevEUI */
                Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacNetID ) & 0xFF;
                Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacNetID >> 8 ) & 0xFF;
                Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacNetID >> 16 ) & 0xFF;

                memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacDevEui, 8 );
                tx_buf_len += 8;

                Radio::radio.tx_buf[tx_buf_len++] = LoRaMacDevNonce & 0xFF;
                Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

                if (LoRaMacJoinComputeMic(false, Radio::radio.tx_buf, tx_buf_len & 0xFF, keys.SNwkSIntKey, &mic) < 0)
                    return LORAMAC_STATUS_SERVICE_UNKNOWN;

            } else if (JoinReqType == 1) {
                /* JoinEUI + DevEUI */
                LoRaMacDevNonce = eeprom_read(EEPROM_RJCOUNT1);
                if (eeprom_increment_value(EEPROM_RJCOUNT1) < 0)
                    return LORAMAC_STATUS_EEPROM_FAIL;

                memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacJoinEui, 8 );
                tx_buf_len += 8;
                memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacDevEui, 8 );
                tx_buf_len += 8;

                Radio::radio.tx_buf[tx_buf_len++] = LoRaMacDevNonce & 0xFF;
                Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

                //print_buf(JSIntKey, 16, "JSIntKey");
                if (LoRaMacJoinComputeMic(false, Radio::radio.tx_buf, tx_buf_len & 0xFF, JSIntKey, &mic) < 0)
                    return LORAMAC_STATUS_SERVICE_UNKNOWN;

            }

            Radio::radio.tx_buf[tx_buf_len++] = mic & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 24 ) & 0xFF;
            MAC_PRINTF("up-rejoin-frame len%u type%u\r\n", tx_buf_len, JoinReqType);
            break;
#endif /* LORAWAN_JOIN_EUI  */
        case FRAME_TYPE_DATA_CONFIRMED_UP:
            //Intentional fallthrough
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:
#ifdef LORAWAN_JOIN_EUI
            if (!flags.IsLoRaMacNetworkJoined)
            {
                return LORAMAC_STATUS_NO_NETWORK_JOINED; // No network has been joined yet
            }

            if (flags.OptNeg && flags.need_ReKeyConf) {
                /* lorawan1v1 need rekeying confirmation */
                LoRaMacStatus_t s = AddMacCommand(MOTE_MAC_REKEY_IND, 0, 0);
                if (s != LORAMAC_STATUS_OK)
                    return s;
                if (McpsIndication.ADR_ACK_CNT > ADR_ACK_DELAY) {
                    /* give up sending rekey indication, try joining again */
                    flags.IsLoRaMacNetworkJoined = false;
                    return LORAMAC_STATUS_NO_NETWORK_JOINED;
                }
            }
            fcnt_up = FCntUp;
#else
            if (flags.need_ResetConf) {
                LoRaMacStatus_t s = AddMacCommand(MOTE_MAC_RESET_IND, flags.OptNeg, 0);
                if (s != LORAMAC_STATUS_OK)
                    return s;
            }
            fcnt_up = eeprom_read(EEPROM_FCNTUP);
#endif /* LORAWAN_JOIN_EUI  */


            if( ValidatePayloadLength( LoRaMacTxPayloadLen, LoRaMacParams.ChannelsDatarate, MacCommandsBufferIndex ) == false )
            {
                MAC_PRINTF("LoRaMacTxPayloadLen%u, FOptsLen%u\r\n", LoRaMacTxPayloadLen, MacCommandsBufferIndex);
                return LORAMAC_STATUS_LENGTH_ERROR;
            }

            RxWindow1Delay_us = LoRaMacParams.ReceiveDelay1_us - RADIO_WAKEUP_TIME_us;
            RxWindow2Delay_us = LoRaMacParams.ReceiveDelay2_us - RADIO_WAKEUP_TIME_us;

            if( flags.SrvAckRequested == true )
            {
                flags.SrvAckRequested = false;
                fCtrl->Bits.Ack = 1;
            }

            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;

            Radio::radio.tx_buf[tx_buf_len++] = fCtrl->Value;

            // FCntUp will be inserted in SendFrameOnChannel(), where MIC is inserted also
            tx_buf_len += 2;

            ConfFCntUp = 0;
            if (flags.OptNeg) {
                if (macHdr->Bits.MType == FRAME_TYPE_DATA_CONFIRMED_UP) {
#ifdef LORAWAN_JOIN_EUI
                    ConfFCntUp = FCntUp;
#else
                    ConfFCntUp = eeprom_read(EEPROM_FCNTUP);
#endif /* LORAWAN_JOIN_EUI */
                }
            }

            // Copy the MAC commands which must be re-send into the MAC command buffer
            memcpy( &MacCommandsBuffer[MacCommandsBufferIndex], MacCommandsBufferToRepeat, MacCommandsBufferToRepeatIndex );
            MacCommandsBufferIndex += MacCommandsBufferToRepeatIndex;

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                if( ( MacCommandsBufferIndex <= LORA_MAC_COMMAND_MAX_LENGTH ) && ( flags.MacCommandsInNextTx == true ) )
                {
                    MAC_PRINTF("uplink mac-cmds %u into FOpts at %u ", MacCommandsBufferIndex, tx_buf_len);
                    fCtrl->Bits.FOptsLen += MacCommandsBufferIndex;
                    // Update FCtrl field with new value of OptionsLength
                    Radio::radio.tx_buf[0x05] = fCtrl->Value;

                    /* lorawan1v1: encode FOpts using NWkSEncKey */
                    if (flags.OptNeg) {
                        LoRaMacEncrypt(0, MacCommandsBuffer, MacCommandsBufferIndex, keys.NwkSEncKey, LoRaMacDevAddr, UP_LINK, fcnt_up, Radio::radio.tx_buf + tx_buf_len);
                        tx_buf_len += MacCommandsBufferIndex;
                    } else {
                        for( i = 0; i < MacCommandsBufferIndex; i++ )
                        {
                            Radio::radio.tx_buf[tx_buf_len++] = MacCommandsBuffer[i];
                            MAC_PRINTF("%02x ", MacCommandsBuffer[i]);
                        }
                        MAC_PRINTF(" fCtrl->Value:%02x\r\n", Radio::radio.tx_buf[0x05]);
                    }
                }
            }
            else
            {
                if( ( MacCommandsBufferIndex > 0 ) && ( flags.MacCommandsInNextTx ) )
                {
                    MAC_PRINTF("uplink mac-cmds %u port0 ", MacCommandsBufferIndex);
                    for (i = 0; i < MacCommandsBufferIndex; i++)
                        MAC_PRINTF("%02x ", MacCommandsBuffer[i]);
                    MAC_PRINTF("\r\n");
                    LoRaMacTxPayloadLen = MacCommandsBufferIndex;
                    payload = MacCommandsBuffer;
                    framePort = 0;
                }
            }
            flags.MacCommandsInNextTx = false;
            // Store MAC commands which must be re-send in case the device does not receive a downlink anymore
            MacCommandsBufferToRepeatIndex = ParseMacCommandsToRepeat( MacCommandsBuffer, MacCommandsBufferIndex, MacCommandsBufferToRepeat );
            if( MacCommandsBufferToRepeatIndex > 0 )
            {
                flags.MacCommandsInNextTx = true;
            }
            MacCommandsBufferIndex = 0;

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                const uint8_t* keyPtr;
                Radio::radio.tx_buf[tx_buf_len++] = framePort;

                if( framePort == 0 )
                {
                    DEBUG_CRYPT_BUF(keys.NwkSEncKey, 16, "NwkSEncKey", 0);
                    keyPtr = keys.NwkSEncKey;
                }
                else
                {
                    DEBUG_CRYPT_BUF(keys.AppSKey, 16, "AppSKey", 0);
                    keyPtr = keys.AppSKey;
                }
                LoRaMacEncrypt(1, (uint8_t* ) payload, LoRaMacTxPayloadLen, keyPtr, LoRaMacDevAddr, UP_LINK, fcnt_up, Radio::radio.tx_buf + tx_buf_len);
            }
            tx_buf_len = tx_buf_len + LoRaMacTxPayloadLen;
            /* mic cacluation in SendFrameOnChannel() */
            break;
        case FRAME_TYPE_PROPRIETARY:
            if( ( fBuffer != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                memcpy( Radio::radio.tx_buf + tx_buf_len, ( uint8_t* ) fBuffer, LoRaMacTxPayloadLen );
                tx_buf_len = tx_buf_len + LoRaMacTxPayloadLen;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }

    flags.uplink_mtype = macHdr->Bits.MType;
    flags.uplink_in_progress = LoRaMacParams.NbTrans;

    return LORAMAC_STATUS_OK;
} // ..PrepareFrame()

LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    LoRaMacFrameCtrl_t fCtrl;
    LoRaMacStatus_t status = LORAMAC_STATUS_PARAMETER_INVALID;

    fCtrl.Value = 0;
    fCtrl.Bits.FOptsLen      = 0;
    if( LoRaMacDeviceClass == CLASS_B )
    {
        fCtrl.Bits.FPending      = 1;
    }
    else
    {
        fCtrl.Bits.FPending      = 0;
    }
    fCtrl.Bits.Ack           = false;
    fCtrl.Bits.AdrAckReq     = false;
    fCtrl.Bits.Adr           = flags.AdrCtrlOn;

    // Prepare the frame
    status = PrepareFrame( macHdr, &fCtrl, fPort, fBuffer, fBufferSize );

    // Validate status
    if( status != LORAMAC_STATUS_OK )
    {
        return status;
    }

    // Reset confirm parameters
    McpsConfirm.NbRetries = 0;
    McpsConfirm.AckReceived = false;

    status = LORAMAC_STATUS_OK;
    if (flags.rxing)
        function_pending = region_ScheduleTx;
    else
        region_ScheduleTx( );

    return status;
} // ..Send()

LoRaMacStatus_t waitingFor;

LoRaMacStatus_t
LoRaMacMlmeRequest( const MlmeReq_t *mlmeRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
#ifdef LORAWAN_JOIN_EUI
    LoRaMacHeader_t macHdr;
#endif /* LORAWAN_JOIN_EUI  */

    if( mlmeRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if (flags.uplink_in_progress > 0) {
        MAC_PRINTF("LoRaMacMlmeRequest() BUSY\r\n");
        return waitingFor;
    }

    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_MLMEREQ;
    MlmeIndication.MlmeIndication = mlmeRequest->Type;

    waitingFor = LORAMAC_STATUS_WAITING_FOR_TXSTART;

    MAC_PRINTF("LoRaMacMlmeRequest() ");
    switch( mlmeRequest->Type )
    {
#ifdef LORAWAN_JOIN_EUI
        case MLME_JOIN:
        {
            if( ( mlmeRequest->Req.Join.DevEui == NULL ) ||
                ( mlmeRequest->Req.Join.JoinEui == NULL ) ||
                ( mlmeRequest->Req.Join.NwkKey == NULL ) ||
                ( mlmeRequest->Req.Join.NbTrials == 0 ) )
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }

            LoRaMacDevEui = mlmeRequest->Req.Join.DevEui;
            LoRaMacJoinEui = mlmeRequest->Req.Join.JoinEui;
            RootNwkKey = mlmeRequest->Req.Join.NwkKey;
            RootAppKey = mlmeRequest->Req.Join.AppKey;
            MaxJoinRequestTrials = mlmeRequest->Req.Join.NbTrials;

            /*if (RootAppKey != NULL) {*/
                LoRaMacGenerateJoinKey(0x05, RootNwkKey, LoRaMacDevEui, JSEncKey);
                //print_buf(JSEncKey, 16, "new-JSEncKey");
                LoRaMacGenerateJoinKey(0x06, RootNwkKey, LoRaMacDevEui, JSIntKey);
                //print_buf(JSIntKey, 16, "new-JSIntKey");
            /*}*/
            JoinReqType = 0xff;

            // Reset variable JoinRequestTrials
            MlmeIndication.JoinRequestTrials = 0;

            // Setup header information
            macHdr.Value = 0;
            macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

            ResetMacParameters( );

            // Add a +1, since we start to count from 0
            LoRaMacParams.ChannelsDatarate = region_AlternateDatarate( MlmeIndication.JoinRequestTrials + 1 );

            status = Send( &macHdr, 0, NULL, 0 );
            break;
        }
        case MLME_REJOIN_1:
            if ( mlmeRequest->Req.Join.JoinEui == NULL )
                return LORAMAC_STATUS_PARAMETER_INVALID;

            LoRaMacJoinEui = mlmeRequest->Req.Join.JoinEui;
            JoinReqType = 0x01;
            // fall-thru
        case MLME_REJOIN_0:
        case MLME_REJOIN_2: // Type2 can only be sent via mac-command
            if( ( mlmeRequest->Req.Join.DevEui == NULL ) ||
                ( mlmeRequest->Req.Join.NwkKey == NULL ) ||
                ( mlmeRequest->Req.Join.NbTrials == 0 ) )
            {
                MAC_PRINTF(" (missing %p %p %d)\n",
                    mlmeRequest->Req.Join.DevEui,
                    mlmeRequest->Req.Join.NwkKey,
                    mlmeRequest->Req.Join.NbTrials
                );
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }

            RootNwkKey = mlmeRequest->Req.Join.NwkKey;
            LoRaMacDevEui = mlmeRequest->Req.Join.DevEui;
            LoRaMacGenerateJoinKey(0x05, RootNwkKey, LoRaMacDevEui, JSEncKey);
            LoRaMacGenerateJoinKey(0x06, RootNwkKey, LoRaMacDevEui, JSIntKey);

            RootAppKey = mlmeRequest->Req.Join.AppKey;

            macHdr.Value = 0;
            macHdr.Bits.MType = FRAME_TYPE_REJOIN_REQ;

            if (mlmeRequest->Type == MLME_REJOIN_0)
                JoinReqType = 0x00;
            else if (mlmeRequest->Type == MLME_REJOIN_2)
                JoinReqType = 0x02;

            MaxJoinRequestTrials = mlmeRequest->Req.Join.NbTrials;

            status = Send( &macHdr, 0, NULL, 0 );
            break;
#endif /* LORAWAN_JOIN_EUI  */
        case MLME_LINK_CHECK:
            status = AddMacCommand( MOTE_MAC_LINK_CHECK_REQ, 0, 0 );
            break;
        case MLME_TIME_REQ:
            status = AddMacCommand( MOTE_MAC_DEVICE_TIME_REQ, 0, 0 );
            break;
        case MLME_TXCW:
            status = SetTxContinuousWave( mlmeRequest->Req.TxCw.Timeout );
            break;
        case MLME_PING_SLOT_INFO:
        {
            uint8_t value = mlmeRequest->Req.PingSlotInfo.Value;
            status = LoRaMacMlmeRequestClassB(mlmeRequest);
            if (status == LORAMAC_STATUS_OK)
                status = AddMacCommandClassB( MOTE_MAC_PING_SLOT_INFO_REQ, value, 0 );
            break;
        }
        case MLME_BEACON_ACQUISITION:
        case MLME_BEACON_TIMING:
            status = LoRaMacMlmeRequestClassB(mlmeRequest);
            break;
        default:
            break;
    } // ...switch( mlmeRequest->Type )

    if( status != LORAMAC_STATUS_OK )
        MlmeConfirm.MlmeRequest = MLME_NONE;
    else
    {
        MlmeConfirm.MlmeRequest = mlmeRequest->Type;
    }

    return status;
} // ..LoRaMacMlmeRequest()

LoRaMacStatus_t
LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    switch( mibGet->Type ) {
        case MIB_APP_SKEY:
            mibGet->Param.key = keys.AppSKey;
            break;
        case MIB_FNwkSIntKey:
            mibGet->Param.key = keys.FNwkSIntKey;
            break;
        case MIB_SNwkSIntKey:
            mibGet->Param.key = keys.SNwkSIntKey;
            break;
        case MIB_NwkSEncKey:
            mibGet->Param.key = keys.NwkSEncKey;
            break;
        case MIB_NwkSKey:
            /* lorawan 1.0 */
            mibGet->Param.key = keys.FNwkSIntKey;
            break;
        case MIB_RX2_CHANNEL:
            mibGet->Param.Rx2Channel = LoRaMacParams.Rx2Channel;
            break;
        case MIB_DEVICE_CLASS:
            mibGet->Param.Class = LoRaMacDeviceClass;
            break;
        case MIB_ADR:
            mibGet->Param.AdrEnable = flags.AdrCtrlOn;
            break;
        case MIB_DEV_ADDR:
            mibGet->Param.DevAddr = LoRaMacDevAddr;
            break;
        case MIB_PUBLIC_NETWORK:
            mibGet->Param.EnablePublicNetwork = flags.PublicNetwork;
            break;
        case MIB_CHANNELS_MASK:
            mibGet->Param.ChannelsMask = LoRaMacParams.ChannelsMask;
            LoRaMacParams.NbEnabledChannels = region_CountNbEnabledChannels();
            break;
        case MIB_MAX_LISTEN_TIME:
            mibGet->Param.MaxListenTime = LoRaMacParams.MaxListenTime;
            break;
#ifdef LORAWAN_JOIN_EUI
        case MIB_NETWORK_JOINED:
            mibGet->Param.IsNetworkJoined = flags.IsLoRaMacNetworkJoined;
            break;
#endif /* LORAWAN_JOIN_EUI */
    } // ..switch( mibGet->Type )

    return status;
}


LoRaMacStatus_t
LoRaMacQueryTxPossible(uint8_t size, LoRaMacTxInfo_t* txInfo)
{
    int8_t datarate = LoRaMacParamsDefaults.ChannelsDatarate;
    uint8_t fOptLen = MacCommandsBufferIndex + MacCommandsBufferToRepeatIndex;

    if( txInfo == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    //AdrNextDr( flags.AdrCtrlOn, false, &datarate );

    txInfo->CurrentPayloadSize = MaxPayloadOfDatarate[datarate];

    if( txInfo->CurrentPayloadSize >= fOptLen )
    {
        txInfo->MaxPossiblePayload = txInfo->CurrentPayloadSize - fOptLen;
    }
    else
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, 0 ) == false )
    {
        return LORAMAC_STATUS_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, fOptLen ) == false )
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t
LoRaMacMcpsRequest( McpsReq_t *mcpsRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;
    uint8_t fPort = 0;
    void *fBuffer;
    uint16_t fBufferSize;
    int8_t datarate;
    bool readyToSend = false;

    if( mcpsRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if (flags.uplink_in_progress > 0) {
        MAC_PRINTF("LoRaMacMcpsRequest() in_progress BUSY%u\r\n", flags.uplink_in_progress);
        return waitingFor;
    }
    if (ConfFCntUp > 0) {
        // unacknowledged confirmed uplink pending, must resend previous uplink
        MAC_PRINTF("LoRaMacMcpsRequest() ConfFCntUp%u\r\n", ConfFCntUp);
        return LORAMAC_STATUS_BUSY_UPCONF;
    }

#ifdef LORAWAN_JOIN_EUI
    if (!flags.IsLoRaMacNetworkJoined)
        return LORAMAC_STATUS_NO_NETWORK_JOINED;
#endif /* LORAWAN_JOIN_EUI  */

    macHdr.Value = 0;
    memset ( ( uint8_t* ) &McpsConfirm, 0, sizeof( McpsConfirm ) );
    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_MCPSREQ;
    McpsConfirm.McpsRequest = mcpsRequest->Type;

    datarate = mcpsRequest->Req.Datarate;
    fBufferSize = mcpsRequest->Req.fBufferSize;
    fBuffer = mcpsRequest->Req.fBuffer;
    readyToSend = true;

    switch( mcpsRequest->Type )
    {
        case MCPS_UNCONFIRMED:
        {
            macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            fPort = mcpsRequest->Req.fPort;
            break;
        }
        case MCPS_CONFIRMED:
        {
            //AckTimeoutRetriesCounter = 1;
            //AckTimeoutRetries = mcpsRequest->Req.NbTrials;

            macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
            fPort = mcpsRequest->Req.fPort;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            macHdr.Bits.MType = FRAME_TYPE_PROPRIETARY;
            break;
        }
        default:
            readyToSend = false;
            break;
    }

    if( readyToSend == true )
    {
        if( flags.AdrCtrlOn == false )
        {
            if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == true )
            {
                LoRaMacParams.ChannelsDatarate = datarate;
            }
            else
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }
        }

        status = Send( &macHdr, fPort, fBuffer, fBufferSize );
    }

    return status;
} // ..LoRaMacMcpsRequest()

__attribute__((weak)) LoRaMacStatus_t
SwitchClassB( DeviceClass_t deviceClass )
{
    return LORAMAC_STATUS_DEVICE_OFF;
}

void
RxWindowSetup( unsigned freq, int8_t datarate, unsigned bandwidth, uint16_t timeout)
{
    uint8_t downlinkDatarate = Datarates[datarate];
    unsigned bwKHz;

    switch (bandwidth) {
        case 0: bwKHz = 125; break;
        case 1: bwKHz = 250; break;
        case 2: bwKHz = 500; break;
        default: bwKHz = 0; /* fail */ break;
    }

    MAC_PRINTF(" rxwin-dr%u-sf%u ", datarate, downlinkDatarate);
    Radio::SetChannel( freq );

    // Store downlink datarate
    McpsIndication.RxDatarate = ( uint8_t ) datarate;

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868) || defined(USE_BAND_ARIB_8CH)
    if( datarate == DR_7 )
    {
        Radio::GFSKModemConfig(downlinkDatarate * 1000, 50, 25000);
        Radio::GFSKPacketConfig(5, false, true);
    }
    else
    {
        Radio::LoRaModemConfig(bwKHz, downlinkDatarate, 1);
        Radio::LoRaPacketConfig(8, false, false, true);
        Radio::SetLoRaSymbolTimeout(timeout);
    }
#elif defined( USE_BAND_470 ) || defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    Radio::LoRaModemConfig(bwKHz, downlinkDatarate, 1);
    Radio::LoRaPacketConfig(8, false, false, true);
    Radio::SetLoRaSymbolTimeout(timeout);
#endif

    Radio::SetRxMaxPayloadLength(MaxPayloadOfDatarate[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD);
} //..RxWindowSetup()

LowPowerTimeout RxWindowEvent1;
LowPowerTimeout RxWindowEvent2;

static void RxWindow2Start( void )
{
    /* TODO: join accept rx2 channel unique */
    if (LoRaMacDeviceClass == CLASS_C)
        Radio::Rx( 0 ); // Continuous mode
    else
        Radio::Rx( LoRaMacParams.MaxRxWindow_us );

    McpsIndication.RxSlot = 2;
}

static void mlme_confirm(LoRaMacEventInfoStatus_t status)
{
    MlmeConfirm.Status = status;

    if (MlmeConfirm.MlmeRequest != MLME_NONE) {
        if (LoRaMacPrimitives->MacMlmeConfirm != NULL)
            LoRaMacPrimitives->MacMlmeConfirm( &MlmeConfirm );

        MlmeConfirm.MlmeRequest = MLME_NONE;
        MlmeIndication.MlmeIndication = MLME_NONE;
    }
}

static void mcps_confirm(LoRaMacEventInfoStatus_t status)
{
    McpsConfirm.Status = status;

    if (McpsConfirm.McpsRequest != MCPS_NONE) {
        if (LoRaMacPrimitives->MacMcpsConfirm)
            LoRaMacPrimitives->MacMcpsConfirm( &McpsConfirm );

        McpsConfirm.McpsRequest = MCPS_NONE;
    }
}

#if defined(LORAWAN_JOIN_EUI)
static struct {
    bool forced;
    uint8_t dr;
    uint8_t type;
    uint8_t retries;
    uint8_t Period;
    LowPowerTimeout event;
    struct {
        uint8_t MaxTimeN;
        uint8_t MaxCountN;
        unsigned uplinks_since;
        bool enabled;
    } type0;
} rejoin;
void _rejoin_retry(void);

void rejoin_retry()
{
    LoRaMacStatus_t status;
    LoRaMacHeader_t macHdr;

    macHdr.Value = 0;
    macHdr.Bits.MType = FRAME_TYPE_REJOIN_REQ;
    LoRaMacParams.ChannelsDatarate = rejoin.dr;
    JoinReqType = rejoin.type;
    status = Send( &macHdr, 0, NULL, 0 );
    if (status != LORAMAC_STATUS_OK) {
        MAC_PRINTF("rejoin-send-failed%d ", status);
    }

    MAC_PRINTF("Rejoin%u ", JoinReqType);
    if (rejoin.forced) {
        if (--rejoin.retries > 0) {
            us_timestamp_t period_us = (1 << rejoin.Period) + random_at_most(32000000);
            rejoin.event.attach_us(_rejoin_retry, period_us);
            MAC_PRINTF("try%u", rejoin.retries);
        } else
            rejoin.forced = false;
    }
    MAC_PRINTF("\r\n");
}

void _rejoin_retry()
{
    if (flags.uplink_in_progress > 0) {
        function_pending = rejoin_retry;
    } else
        rejoin_retry();
}
#endif /* LORAWAN_JOIN_EUI  */

void
finish_uplink(LoRaMacEventInfoStatus_t status)
{
    if ((flags.uplink_in_progress > 0 && McpsIndication.RxSlot == 2) || status == LORAMAC_EVENT_INFO_STATUS_OK) {
        if ((uplinkMHDR->Bits.MType == FRAME_TYPE_DATA_CONFIRMED_UP && status == LORAMAC_EVENT_INFO_STATUS_OK) || uplinkMHDR->Bits.MType != FRAME_TYPE_DATA_CONFIRMED_UP) {
            flags.uplink_in_progress--;
        }

        if (flags.uplink_in_progress > 0) {
            McpsIndication.Status = status;
            if (LoRaMacPrimitives->MacMcpsIndication != NULL)
                LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );

            if (flags.rxing)
                function_pending = region_ScheduleTx;
            else {
                region_ScheduleTx( );
            }
        } else
            region_session_start(status);
    }

#ifdef LORAWAN_JOIN_EUI
    LoRaMacHeader_t macHdr;
    macHdr.Value = Radio::radio.tx_buf[0];
    if (macHdr.Bits.MType != FRAME_TYPE_REJOIN_REQ) {
        if (rejoin.type0.enabled && --rejoin.type0.uplinks_since == 0) {
            rejoin.type0.uplinks_since = 1 << (rejoin.type0.MaxCountN + 4);

            rejoin.type = 0;
            rejoin_retry();
            return;
        }
    }
#endif /* LORAWAN_JOIN_EUI  */

    waitingFor = LORAMAC_STATUS_OK;

    if (function_pending != NULL) {
        function_pending();
        function_pending = NULL;
    }
} // ..finish_uplink()

LowPowerTimeout TxDelayedEvent;

void OnTxDelayedIsr()
{
    flags.OnTxDelayed = true;
}

static void OnTxDelayedTimerEvent()
{
    MAC_PRINTF("OnTxDelayedTimerEvent() ");
#ifdef LORAWAN_JOIN_EUI
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;

    if (!flags.IsLoRaMacNetworkJoined)
    {
        // Add a +1, since we start to count from 0
        LoRaMacParams.ChannelsDatarate = region_AlternateDatarate( MlmeIndication.JoinRequestTrials + 1 );

        macHdr.Value = 0;
        macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

        fCtrl.Value = 0;
        fCtrl.Bits.Adr = flags.AdrCtrlOn;

        /* In case of join request retransmissions, the stack must prepare
         * the frame again, because the network server keeps track of the random
         * LoRaMacDevNonce values to prevent reply attacks. */
        PrepareFrame( &macHdr, &fCtrl, 0, NULL, 0 );
        /* TODO PrepareFrame() != LORAMAC_STATUS_OK */
    }
#endif /* LORAWAN_JOIN_EUI  */

    if (flags.rxing)
        function_pending = region_ScheduleTx;
    else {
        region_ScheduleTx( );
    }
} // ..OnTxDelayedTimerEvent()

static void RxWindow2Setup(void)
{
    MAC_PRINTF("RxWindow2Setup %uhz dr%u", LoRaMacParams.Rx2Channel.FrequencyHz, LoRaMacParams.Rx2Channel.Datarate);
    RxWindowSetup(
        LoRaMacParams.Rx2Channel.FrequencyHz,
        LoRaMacParams.Rx2Channel.Datarate,
        region_GetRxBandwidth( LoRaMacParams.Rx2Channel.Datarate ),
        region_GetRxSymbolTimeout( LoRaMacParams.Rx2Channel.Datarate )
    );

    waitingFor = LORAMAC_STATUS_WAITING_FOR_RX2;
}

static void
PrepareRxDoneAbort(LoRaMacEventInfoStatus_t status)
{
    MAC_PRINTF("rxAbort ");
    if( ( McpsIndication.RxSlot == 1 ) && ( LoRaMacDeviceClass == CLASS_C ) )
    {
        RxWindow2Setup();
        RxWindow2Start();   // start continuous rx2 reception
    }

#ifdef LORAWAN_JOIN_EUI
    if (!flags.IsLoRaMacNetworkJoined && LoRaMacJoinEui != NULL && LoRaMacDevEui == NULL)  {
        TxDelayedEvent.attach_us(OnTxDelayedIsr, 1000000);
        MAC_PRINTF("RxDoneAbort-join-tx-delay");
    }
#endif /* LORAWAN_JOIN_EUI  */


    McpsIndication.Status = status;
    if (LoRaMacPrimitives->MacMcpsIndication != NULL)
        LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );    // RxAbort

    mcps_confirm(status);  // RXAbort
    mlme_confirm(status);

    finish_uplink(status);

    MAC_PRINTF("\r\n");

} // ..PrepareRxDoneAbort()


static LoRaMacStatus_t SwitchClass( DeviceClass_t deviceClass )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_PARAMETER_INVALID;

    switch( LoRaMacDeviceClass )
    {
        case CLASS_A:
        {
            MAC_PRINTF("CLASS_A ");
            if (deviceClass == CLASS_B)
                status = SwitchClassB(deviceClass);

            if (deviceClass == CLASS_C)
            {
                MAC_PRINTF("->C ");
                LoRaMacDeviceClass = deviceClass;
                RxWindow2Setup();
                RxWindow2Start(); // continuous rx2 reception
                if (flags.OptNeg) {
                    AddMacCommand(MOTE_MAC_MODE_IND, LoRaMacDeviceClass, 0);
                }

                status = LORAMAC_STATUS_OK;
            }
            break;
        }
        case CLASS_B:
        {
            MAC_PRINTF("CLASS_B ");
            if( deviceClass == CLASS_A )
            {
                MAC_PRINTF("->A ");
                LoRaMacDeviceClass = deviceClass;
                status = LORAMAC_STATUS_OK;
            }
            break;
        }
        case CLASS_C:
        {
            MAC_PRINTF("CLASS_C ");
            if( deviceClass == CLASS_A )
            {
                MAC_PRINTF("->A ");
                LoRaMacDeviceClass = deviceClass;
                if (flags.OptNeg) {
                    AddMacCommand(MOTE_MAC_MODE_IND, LoRaMacDeviceClass, 0);
                }

                // Set the radio into sleep to setup a defined state
                Radio::Sleep();

                status = LORAMAC_STATUS_OK;
            }
            break;
        }
    }

    MAC_PRINTF("\r\n");
    return status;
}

LoRaMacStatus_t
LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if (mibSet == NULL)
        return LORAMAC_STATUS_PARAMETER_INVALID;

    if (flags.uplink_in_progress > 0)
        return waitingFor;

    switch (mibSet->Type) {
        case MIB_CHANNELS_MASK:
            if( mibSet->Param.ChannelsMask )
            {
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                bool chanMaskState = true;

#if defined( USE_BAND_915_HYBRID )
                chanMaskState = ValidateChannelMask( mibSet->Param.ChannelsMask );
#endif
                if( chanMaskState == true )
                {
                    if( ( CountNbEnabled125kHzChannels( mibSet->Param.ChannelsMask ) < 2 ) &&
                        ( CountNbEnabled125kHzChannels( mibSet->Param.ChannelsMask ) > 0 ) )
                    {
                        status = LORAMAC_STATUS_PARAMETER_INVALID;
                    }
                    else
                    {
                        memcpy( ( uint8_t* ) LoRaMacParams.ChannelsMask,
                                 ( uint8_t* ) mibSet->Param.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
                        for ( uint8_t i = 0; i < sizeof( LoRaMacParams.ChannelsMask ) / 2; i++ )
                        {
                            // Disable channels which are no longer available
                            ChannelsMaskRemaining[i] &= LoRaMacParams.ChannelsMask[i];
                        }
                    }
                }
                else
                {
                    status = LORAMAC_STATUS_PARAMETER_INVALID;
                }
#elif defined( USE_BAND_470 )
                memcpy( ( uint8_t* ) LoRaMacParams.ChannelsMask,
                         ( uint8_t* ) mibSet->Param.ChannelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
#else
                memcpy( ( uint8_t* ) LoRaMacParams.ChannelsMask,
                         ( uint8_t* ) mibSet->Param.ChannelsMask, 2 );
#endif
                LoRaMacParams.NbEnabledChannels = region_CountNbEnabledChannels();
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
#ifdef LORAWAN_JOIN_EUI
        /* values which cannot be set in OTA */
        case MIB_NwkSKey:
        case MIB_SNwkSIntKey:
        case MIB_NwkSEncKey:
        case MIB_FNwkSIntKey:
        case MIB_APP_SKEY:
        case MIB_DEV_ADDR:
        case MIB_NETWORK_JOINED:
#endif
        case MIB_RX2_CHANNEL:
            return LORAMAC_STATUS_PARAMETER_INVALID;
        case MIB_DEVICE_CLASS:
            status = SwitchClass(mibSet->Param.Class);
            break;
        case MIB_ADR:
            flags.AdrCtrlOn = mibSet->Param.AdrEnable;
            break;
        case MIB_PUBLIC_NETWORK:
            flags.PublicNetwork = mibSet->Param.EnablePublicNetwork;
            Radio::SetPublicNetwork( flags.PublicNetwork );
            break;
        case MIB_MAX_LISTEN_TIME:
            LoRaMacParams.MaxListenTime = mibSet->Param.MaxListenTime;
            break;
#ifndef LORAWAN_JOIN_EUI
        case MIB_SNwkSIntKey:
            flags.have_SNwkSIntKey = 1;
            memcpy( keys.SNwkSIntKey, mibSet->Param.key, sizeof(keys.SNwkSIntKey) );
            if (flags.have_NwkSEncKey) {
                flags.OptNeg = 1;
                flags.need_ResetConf = 1;
            }
            break;
        case MIB_NwkSEncKey:
            flags.have_NwkSEncKey = 1;
            memcpy( keys.NwkSEncKey, mibSet->Param.key, sizeof(keys.NwkSEncKey) );
            if (flags.have_SNwkSIntKey) {
                flags.OptNeg = 1;
                flags.need_ResetConf = 1;
            }
            break;
        case MIB_APP_SKEY:
            memcpy( keys.AppSKey, mibSet->Param.key, sizeof( keys.AppSKey ) );
            break;
        case MIB_FNwkSIntKey:
            memcpy( keys.FNwkSIntKey, mibSet->Param.key, sizeof( keys.FNwkSIntKey ) );
            break;
        case MIB_NwkSKey:
            /* lorawan 1.0 ABP */
            memcpy( keys.FNwkSIntKey, mibSet->Param.key, sizeof( keys.FNwkSIntKey ) );
            memcpy( keys.SNwkSIntKey, mibSet->Param.key, sizeof( keys.SNwkSIntKey) );
            memcpy( keys.NwkSEncKey, mibSet->Param.key, sizeof( keys.NwkSEncKey) );
            flags.OptNeg = 0;
            break;
        case MIB_DEV_ADDR:
            LoRaMacDevAddr = mibSet->Param.DevAddr;
            break;
#endif /* !LORAWAN_JOIN_EUI */
    } // ..switch( mibSet->Type )

    return status;
} // ..LoRaMacMibSetRequestConfirm()

__attribute__((weak)) LoRaMacStatus_t
LoRaMacClassBInitialization( void )
{
    return LORAMAC_STATUS_OK;
}


static void RxWindow1Start( void )
{      
    if (LoRaMacDeviceClass == CLASS_C) {
        Radio::Standby();
        region_rx1_setup(Channel);
    }

    Radio::Rx( LoRaMacParams.MaxRxWindow_us );

    McpsIndication.RxSlot = 1;
}

volatile us_timestamp_t tx_done_at;

static void OnRadioTxDone( )
{
    if ((RxWindow1Delay_us < 100000 || RxWindow1Delay_us > 10000000) ||
        (RxWindow2Delay_us < 100000 || RxWindow2Delay_us > 10000000))
    {
        PrepareRxDoneAbort(LORAMAC_EVENT_INFO_BAD_RX_DELAY);
        return;
    }
    // Setup timers
    RxWindowEvent1.attach_us(RxWindow1Start, RxWindow1Delay_us);
    waitingFor = LORAMAC_STATUS_WAITING_FOR_RX1;
    McpsIndication.RxSlot = 0;

    tx_done_at = Radio::irqAt;

    if (LoRaMacDeviceClass != CLASS_C)
    {
        RxWindowEvent2.attach_us(RxWindow2Start, RxWindow2Delay_us);
        region_rx1_setup(Channel);
        Radio::Sleep();
    }
    else
    {
        RxWindow2Setup();
        RxWindow2Start();
        // simulate timeout to complete uplink
        RxWindowEvent2.attach_us(OnRadioRxTimeout, RxWindow2Delay_us + 256000);
    }

    // Store last tx channel
    //LastTxChannel = Channel;
#ifdef DUTY_ENABLE
    DutyTxDone(at_us);
#endif /* DUTY_ENABLE */

} // ..OnRadioTxDone()

static void OnRadioTxTimeout( void )
{
    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio::Sleep();
    }
    else
    {
        RxWindow2Setup();
        RxWindow2Start();
    }

    finish_uplink(LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT);

    mcps_confirm(LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT);
    mlme_confirm(LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT);
} // ..OnRadioTxTimeout()

__attribute__((weak)) bool
beacon_rx_done_payload(uint16_t size)
{
    return false;
}

static void
print_mtype(uint8_t mt)
{
#ifdef MAC_DEBUG
    const char* cp;
    switch (mt) {
#ifdef LORAWAN_JOIN_EUI
        case FRAME_TYPE_JOIN_REQ: cp = "JOIN_REQ "; break;
        case FRAME_TYPE_JOIN_ACCEPT: cp = "JOIN_ACC "; break;
        case FRAME_TYPE_REJOIN_REQ: cp = "REJOIN_REQ"; break;
#endif /* LORAWAN_JOIN_EUI */
        case FRAME_TYPE_DATA_UNCONFIRMED_UP: cp = "UNCONF_UP"; break;
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN: cp = "UNCONF_DN"; break;
        case FRAME_TYPE_DATA_CONFIRMED_UP: cp = "CONF_UP"; break;
        case FRAME_TYPE_DATA_CONFIRMED_DOWN: cp = "CONF_DN"; break;
        case FRAME_TYPE_PROPRIETARY: cp = "P"; break;
        default: return;
    }
    MAC_PRINTF("MTYPE_%s ", cp);
#endif /* MAC_DEBUG */
}

/* bool a: true=AFCntDown, false=NFCntDown */
uint32_t get_fcntdwn(bool a)
{
#ifdef LORAWAN_JOIN_EUI
    if (a)
        return AFCntDown;
    else
        return NFCntDown;
#else
    if (a)
        return eeprom_read(EEPROM_AFCNTDWN);
    else
        return eeprom_read(EEPROM_NFCNTDWN);
#endif
}

__attribute__((weak)) bool
ProcessMacCommandsClassB(const uint8_t* payload, uint8_t* macIndex)
{
    return false;   /* false: not taken */
}


static bool ValidateDatarate( int8_t datarate, uint16_t* channelsMask )
{
    if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == false )
    {
        return false;
    }
    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( ( channelsMask[k] & ( 1 << j ) ) != 0 ) )
            {// Check datarate validity for enabled channels
                if( ValueInRange( datarate, Channels[i + j].DrRange.Fields.Min, Channels[i + j].DrRange.Fields.Max ) == true )
                {
                    // At least 1 channel has been found we can return OK.
                    return true;
                }
            }
        }
    }
    return false;
}

static bool Rx2FreqInRange( uint32_t freq )
{
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 ) || defined (USE_BAND_ARIB_8CH)
    if( Radio::CheckRfFrequency( freq ) == true )
#elif defined( USE_BAND_470 ) || defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    if( ( Radio::CheckRfFrequency( freq ) == true ) &&
        ( freq >= LORAMAC_FIRST_RX1_CHANNEL ) &&
        ( freq <= LORAMAC_LAST_RX1_CHANNEL ) &&
        ( ( ( freq - ( uint32_t ) LORAMAC_FIRST_RX1_CHANNEL ) % ( uint32_t ) LORAMAC_STEPWIDTH_RX1_CHANNEL ) == 0 ) )
#endif
    {
        return true;
    }
    return false;
}

__attribute__((weak)) void
deviceTimeClassB(uint32_t secs, uint32_t subsecs)
{
}

/* return -1 for unknown mac cmd */
static int
ProcessMacCommands(const uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, float snr)
{
    uint8_t buf[2];
    int ret = 0;

    MACC_PRINTF("ProcessMacCommands(, %u, %u,,) ", macIndex, commandsSize);
    while (macIndex < commandsSize)
    {
        MACC_PRINTF("ProcessMacCommands %u(0x%02x): ", macIndex, payload[macIndex]);
        // Decode Frame MAC commands
        switch (payload[macIndex++])
        {
            case SRV_MAC_LINK_CHECK_ANS:
                MACC_PRINTF("LINK_CHECK_ANS ");
                buf[0] = payload[macIndex++];
                buf[1] = payload[macIndex++];
                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                MlmeConfirm.fields.link.DemodMargin = buf[0];
                MlmeConfirm.fields.link.NbGateways = buf[1];
                break;
            case SRV_MAC_MODE_CONF:
                break;
            case SRV_MAC_LINK_ADR_REQ:
                MACC_PRINTF("LINK_ADR_REQ ");
                {
                    uint8_t i;
                    int8_t txPower = 0;
                    uint8_t Redundancy = 0;
                    adr_t adr;

                    adr.status = 0x07;
                    // Initialize local copy of the channels mask array
                    for( i = 0; i < 6; i++ )
                    {
                        adr.channelsMask[i] = LoRaMacParams.ChannelsMask[i];
                    }
                    adr.datarate = payload[macIndex++];
                    txPower = adr.datarate & 0x0F;
                    adr.datarate = ( adr.datarate >> 4 ) & 0x0F;
                    MACC_PRINTF("dr%u power%u ", adr.datarate, txPower);

                    if( ( flags.AdrCtrlOn == false ) &&
                        ( ( LoRaMacParams.ChannelsDatarate != adr.datarate ) || ( LoRaMacParams.ChannelsTxPower != txPower ) ) )
                    { // ADR disabled don't handle ADR requests if server tries to change datarate or txpower
                        MACC_PRINTF("AdrCtrlOn:%u dr%u != dr%u, %d != %d\r\n", flags.AdrCtrlOn,
                            LoRaMacParams.ChannelsDatarate, adr.datarate,
                            LoRaMacParams.ChannelsTxPower, txPower
                        );
                        // Answer the server with fail status
                        // Power ACK     = 0
                        // Data rate ACK = 0
                        // Channel mask  = 0
                        AddMacCommand( MOTE_MAC_LINK_ADR_ANS, 0, 0 );
                        macIndex += 3;  // Skip over the remaining bytes of the request
                        break;
                    }
                    adr.chMask = ( uint16_t )payload[macIndex++];
                    adr.chMask |= ( uint16_t )payload[macIndex++] << 8;

                    Redundancy = payload[macIndex++];
                    adr.chMaskCntl = ( Redundancy >> 4 ) & 0x07;
                    if ((Redundancy & 0x0f) > 0)
                        LoRaMacParams.NbTrans = Redundancy & 0x0f;

                    MACC_PRINTF("chMask:%04x chMaskCntl:%x nbTrans:%u ", adr.chMask, adr.chMaskCntl, LoRaMacParams.NbTrans);

                    region_adr_request(&adr);

                    if( ValidateDatarate( adr.datarate, adr.channelsMask ) == false )
                    {
                        MACC_PRINTF("badDr ");
                        adr.status &= 0xFD; // Datarate KO
                    }

                    //
                    // Remark MaxTxPower = 0 and MinTxPower = 5
                    //
                    if( ValueInRange( txPower, LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) == false )
                    {
                        MACC_PRINTF("badPower(max:%d given:%d min:%d) ", LORAMAC_MAX_TX_POWER, txPower, LORAMAC_MIN_TX_POWER);
                        adr.status &= 0xFB; // TxPower KO
                    }
                    MACC_PRINTF("status:%x (idx %u) ", adr.status, macIndex);
                    if( ( adr.status & 0x07 ) == 0x07 )
                    {
                        LoRaMacParams.ChannelsDatarate = adr.datarate;
                        LoRaMacParams.ChannelsTxPower = txPower;

                        memcpy( ( uint8_t* )LoRaMacParams.ChannelsMask, ( uint8_t* )adr.channelsMask, sizeof( LoRaMacParams.ChannelsMask ) );
                        LoRaMacParams.NbEnabledChannels = region_CountNbEnabledChannels();

                    }
                    AddMacCommand( MOTE_MAC_LINK_ADR_ANS, adr.status, 0 );
                }
                break;
            case SRV_MAC_RX_PARAM_SETUP_REQ:
                {
                    uint8_t status = 0x07;
                    int8_t datarate = 0;
                    int8_t drOffset = 0;
                    uint32_t freq = 0;

                    drOffset = ( payload[macIndex] >> 4 ) & 0x07;
                    datarate = payload[macIndex] & 0x0F;
                    macIndex++;

                    freq =  ( uint32_t )payload[macIndex++];
                    freq |= ( uint32_t )payload[macIndex++] << 8;
                    freq |= ( uint32_t )payload[macIndex++] << 16;
                    freq *= 100;
                    MACC_PRINTF("RX_PARAM_SETUP_REQ %uhz drOffset:%u dr%u ", freq, drOffset, datarate);

                    if( Rx2FreqInRange( freq ) == false )
                    {
                        status &= 0xFE; // Channel frequency KO
                    }

                    if( ValueInRange( datarate, LORAMAC_RX_MIN_DATARATE, LORAMAC_RX_MAX_DATARATE ) == false )
                    {
                        status &= 0xFD; // Datarate KO
                    }
#if ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
                    if( ( ValueInRange( datarate, DR_5, DR_7 ) == true ) ||
                        ( datarate > DR_13 ) )
                    {
                        status &= 0xFD; // Datarate KO
                    }
#endif
                    if( ValueInRange( drOffset, LORAMAC_MIN_RX1_DR_OFFSET, LORAMAC_MAX_RX1_DR_OFFSET ) == false )
                    {
                        status &= 0xFB; // Rx1DrOffset range KO
                    }

                    MACC_PRINTF("status:0x%02x ", status);
                    if( ( status & 0x07 ) == 0x07 )
                    {
                        LoRaMacParams.Rx2Channel.Datarate = datarate;
                        LoRaMacParams.Rx2Channel.FrequencyHz = freq;
                        LoRaMacParams.Rx1DrOffset = drOffset;
                    }
                    AddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0 );
                }
                break;
            case SRV_MAC_DEV_STATUS_REQ:
                MACC_PRINTF("DEV_STATUS_REQ ");
                {
                    uint8_t batteryLevel = BAT_LEVEL_NO_MEASURE;
                    if( ( LoRaMacCallbacks != NULL ) && ( LoRaMacCallbacks->GetBatteryLevel != NULL ) )
                    {
                        batteryLevel = LoRaMacCallbacks->GetBatteryLevel( );
                    }
                    AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, batteryLevel, snr );
                    break;
                }
            case SRV_MAC_NEW_CHANNEL_REQ:
                {
                    uint8_t status = 0x03;

#if defined( USE_BAND_470 ) || defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                    status &= 0xFC; // Channel frequency and datarate KO
                    macIndex += 5;
#else
                    int8_t channelIndex = 0;
                    ChannelParams_t chParam;

                    channelIndex = payload[macIndex++];
                    chParam.FreqHz = ( uint32_t )payload[macIndex++];
                    chParam.FreqHz |= ( uint32_t )payload[macIndex++] << 8;
                    chParam.FreqHz |= ( uint32_t )payload[macIndex++] << 16;
                    chParam.FreqHz *= 100;
                    chParam.DrRange.Value = payload[macIndex++];
                    MACC_PRINTF("NEW_CHANNEL_REQ ch%u %uhz drRange:%02x ", channelIndex, chParam.Frequency, chParam.DrRange.Value);

                    if( chParam.FreqHz == 0 )
                    {
                        if( channelIndex < 3 )
                        {
                            status &= 0xFC;
                        }
                        else
                        {
                            if( LoRaMacChannelRemove( channelIndex ) != LORAMAC_STATUS_OK )
                            {
                                status &= 0xFC;
                            }
                        }
                    }
                    else
                    {
                        switch( LoRaMacChannelAdd( channelIndex, chParam ) )
                        {
                            case LORAMAC_STATUS_OK:
                            {
                                MACC_PRINTF("add-ok ");
                                break;
                            }
                            case LORAMAC_STATUS_FREQUENCY_INVALID:
                            {
                                MACC_PRINTF("add-bad-freq ");
                                status &= 0xFE;
                                break;
                            }
                            case LORAMAC_STATUS_DATARATE_INVALID:
                            {
                                MACC_PRINTF("add-bad-dr ");
                                status &= 0xFD;
                                break;
                            }
                            case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
                            {
                                MACC_PRINTF("add-bad-both ");
                                status &= 0xFC;
                                break;
                            }
                            default:
                            {
                                MACC_PRINTF("add-bad-? ");
                                status &= 0xFC;
                                break;
                            }
                        }
                    }
#endif
                    MACC_PRINTF("status:%x ", status);
                    AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
                }
                break;
            case SRV_MAC_ADR_PARAM_SETUP_REQ:
                MACC_PRINTF("ADR_PARAM_SETUP_REQ");
                {
                    uint8_t exps = payload[macIndex++] & 0x0F;
                    ADR_ACK_LIMIT = 1 << (exps >> 4);
                    ADR_ACK_DELAY = 1 << (exps & 0x0f);
                }
                AddMacCommand(SRV_MAC_ADR_PARAM_SETUP_ANS, 0, 0);
                break;
            case SRV_MAC_RX_TIMING_SETUP_REQ:
                MACC_PRINTF("RX_TIMING_SETUP_REQ");
                {
                    uint8_t delay = payload[macIndex++] & 0x0F;

                    if( delay == 0 )
                    {
                        delay++;
                    }
                    LoRaMacParams.ReceiveDelay1_us = delay * 1e6;
                    LoRaMacParams.ReceiveDelay2_us = LoRaMacParams.ReceiveDelay1_us + 1e6;
                    AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
                }
                break;
#ifdef LORAWAN_JOIN_EUI
            case SRV_MAC_REKEY_CONF:
                macIndex++; //TODO server_version = payload[macIndex++];
                
                flags.need_ReKeyConf = 0;
                break;
            case SRV_MAC_FORCE_REJOIN_REQ:
                {
                    uint16_t cmd_payload = payload[macIndex++];
                    cmd_payload |= payload[macIndex++] << 8;
                    rejoin.type = (cmd_payload >> 4) & 7;
                    if (rejoin.type == 2)
                        JoinReqType = 2;
                    else {
                        JoinReqType = 0;
                        rejoin.type = 0;
                    }

                    rejoin.dr = cmd_payload & 0x0f;
                    LoRaMacParams.ChannelsDatarate = rejoin.dr;
                    rejoin.retries = 1 + ((cmd_payload >> 8) & 7);
                    MAC_PRINTF("FORCE_REJOIN 0x%04x dr%u type%u tries%u ", cmd_payload, LoRaMacParams.ChannelsDatarate, JoinReqType, rejoin.retries);

                    {
                        rejoin.Period = (cmd_payload >> 11) & 7;
                        /* first forced-rejoin attempt must be immediate */
                        rejoin.event.attach_us(_rejoin_retry, 50000);
                        rejoin.forced = true;
                    }
                }
                break;
            case SRV_MAC_REJOIN_PARAM_REQ:
                {
                    uint8_t p = payload[macIndex++];
                    rejoin.type0.MaxTimeN = p >> 4;
                    rejoin.type0.MaxCountN = p & 0xf;
                    rejoin.type0.enabled = true;
                    MACC_PRINTF("REJOIN_PARAM MaxTimeN%u MaxCountN%u ", rejoin.type0.MaxTimeN, rejoin.type0.MaxCountN);
                    rejoin.type0.uplinks_since = 1 << (rejoin.type0.MaxCountN + 4);
                    AddMacCommand(MOTE_MAC_REJOIN_PARAM_ANS, 0, 0);
                }
                break;
#else
            case SRV_MAC_RESET_CONF:
                macIndex++; //TODO server_version = payload[macIndex++];
                flags.need_ResetConf = 0;
                break;
#endif /* !LORAWAN_JOIN_EUI  */
            case SRV_MAC_DEVICE_TIME_ANS:
                {
                    uint32_t subusecs, secs;
                    us_timestamp_t us_since_tx_done;
                    secs = payload[macIndex++];
                    secs += payload[macIndex++] << 8;
                    secs += payload[macIndex++] << 16;
                    secs += payload[macIndex++] << 24;
                    subusecs = payload[macIndex++] * 3906.5;

                    //MAC_PRINTF("secs:%u, subusecs:%u\r\n", secs, subusecs);
                    deviceTimeClassB(secs, subusecs);

                    us_since_tx_done = Radio::lpt.read_us() - tx_done_at;
                    MlmeConfirm.fields.time.uSeconds += us_since_tx_done;
                    while (us_since_tx_done >= 1000000) {
                        MlmeConfirm.fields.time.Seconds++;
                        us_since_tx_done -= 1000000;
                    }
                    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                    MlmeConfirm.fields.time.Seconds = secs;
                    MlmeConfirm.fields.time.uSeconds = subusecs;
                }
                break;
            default:
                --macIndex;
                if (ProcessMacCommandsClassB(payload, &macIndex)) {
                    /* mac command was taken */
                    //MACC_PRINTF("B-cont\r\n");
                }
#ifdef DUTY_ENABLE
                else if (ProcessMacCommandsDuty(payload, &macIndex)) {
                    /* mac command was taken */
                }
#endif /* DUTY_ENABLE */
                else {
                    ret = -1;
                    MAC_PRINTF("[31munknown mac:0x%02x at %u[0m\r\n", payload[macIndex-1], macIndex-1);
                }
                break;
        } // ..switch(payload[macIndex++])

        MACC_PRINTF("\r\n");

    } // ..while( macIndex < commandsSize )

    return ret;
} // ..ProcessMacCommands()

#define MAX_FCNT_GAP        0x4000
/* return: true == send downlink ack */
static int
rx_downlink(uint8_t pktHeaderLen, uint16_t rx_size, int8_t snr)
{
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    uint32_t myFCntDwn32, rxFCnt32;
    uint8_t rxFPort;
    uint16_t rxFCnt16;
    uint32_t address;
    uint32_t mic, micRx;
    uint8_t appPayloadStartIndex = 0;
    bool skipIndication = false;
    uint8_t frameLen = 0;
    bool is_AFCntDown;
    block_t block;
    const uint8_t* rx_payload = Radio::radio.rx_buf;

    macHdr.Value = rx_payload[0];

    address = rx_payload[pktHeaderLen++];
    address |= ( (uint32_t)rx_payload[pktHeaderLen++] << 8 );
    address |= ( (uint32_t)rx_payload[pktHeaderLen++] << 16 );
    address |= ( (uint32_t)rx_payload[pktHeaderLen++] << 24 );

    fCtrl.Value = rx_payload[pktHeaderLen++];

    rxFCnt16 = ( uint16_t )rx_payload[pktHeaderLen++];
    rxFCnt16 |= ( uint16_t )rx_payload[pktHeaderLen++] << 8;

    is_AFCntDown = false;
    if (( ( rx_size - 4 ) - (8 + fCtrl.Bits.FOptsLen) ) > 0) {
        appPayloadStartIndex = 8 + fCtrl.Bits.FOptsLen;
        rxFPort = rx_payload[appPayloadStartIndex++];
        if (flags.OptNeg && rxFPort > 0)
            is_AFCntDown = true;
    } /* else no payload/fport present */

    myFCntDwn32 = get_fcntdwn(is_AFCntDown);

    McpsIndication.expectedFCntDown = myFCntDwn32;
    McpsIndication.receivedFCntDown = rxFCnt16;

    rxFCnt32 = (myFCntDwn32 & 0xffff0000) | rxFCnt16;
    DEBUG_MIC_DOWN(" rxFCnt32:%" PRIu32" ", rxFCnt32);

    micRx = ( uint32_t )rx_payload[rx_size - LORAMAC_MFR_LEN];
    micRx |= ( ( uint32_t )rx_payload[rx_size - LORAMAC_MFR_LEN + 1] << 8 );
    micRx |= ( ( uint32_t )rx_payload[rx_size - LORAMAC_MFR_LEN + 2] << 16 );
    micRx |= ( ( uint32_t )rx_payload[rx_size - LORAMAC_MFR_LEN + 3] << 24 );

    block.b.header = 0x49;
    if (flags.OptNeg)
        block.b.confFCnt = ConfFCntUp;
    else
        block.b.confFCnt = 0;
    block.b.dr = 0;
    block.b.ch = 0;
    block.b.dir = DOWN_LINK;
    block.b.DevAddr = address;
    block.b.FCnt = rxFCnt32;
    block.b.zero8 = 0;
    block.b.lenMsg = rx_size - LORAMAC_MFR_LEN;
    mic = LoRaMacComputeMic(&block, rx_payload, keys.SNwkSIntKey);
    if (micRx != mic) {
        bool ignore_rx = true;
        PrepareRxDoneAbort(LORAMAC_EVENT_INFO_STATUS_MIC_FAIL);
        MAC_PRINTF("\e[31mmicFail");
        if (flags.OptNeg) {
            block.b.confFCnt = ConfFCntUp - 1;
            mic = LoRaMacComputeMic(&block, rx_payload, keys.SNwkSIntKey);
            if (micRx == mic)
                ignore_rx = false;
            else {
                block.b.confFCnt = 0;
                mic = LoRaMacComputeMic(&block, rx_payload, keys.SNwkSIntKey);
                if (micRx == mic)
                    ignore_rx = false;
            }
        }
        if (ignore_rx) {
            MAC_PRINTF("\e[0m\r\n");
            return -1;
        } else {
            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;
            if (LoRaMacPrimitives->MacMcpsIndication != NULL)
                LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );
        }
    } else {
        /* downlink with good MIC means previous confirmed uplink was receied */
        ConfFCntUp = 0;
        if (McpsIndication.RxSlot == 1) {   // no need for RX2 with good mic on rx1
            RxWindowEvent2.detach();
        }
    }

    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
    //McpsIndication.Multicast = 0;//multicast;
    McpsIndication.FramePending = fCtrl.Bits.FPending;
    McpsIndication.Buffer = NULL;
    McpsIndication.BufferSize = 0;

    MacCommandsBufferToRepeatIndex = 0;

    if (macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_DOWN)
    {
        flags.SrvAckRequested = true;
        McpsIndication.McpsIndication = MCPS_CONFIRMED;

#ifdef LORAWAN_JOIN_EUI
        if (flags.IsLoRaMacNetworkJoined)
#endif /* LORAWAN_JOIN_EUI  */

        if (flags.OptNeg)
            ConfFCntDown = rxFCnt16;
    }
    else
    {   // FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
        ConfFCntDown = 0;
        flags.SrvAckRequested = false;
        McpsIndication.McpsIndication = MCPS_UNCONFIRMED;
    }

    if (fCtrl.Bits.FOptsLen > 0)
    {
        // Decode Options field MAC commands
        if (flags.OptNeg) {
            uint32_t FCnt32;
            bool fromStored;
            uint8_t macDecrypt[16];
            DEBUG_CRYPT_BUF(keys.NwkSEncKey, 16, "NwkSEncKey", 0);
            if (appPayloadStartIndex > 0) {
                /* rx header has AFCntDown: not for use with FOpts */
                FCnt32 = get_fcntdwn(false);
                fromStored = true;
            } else {
                /* NFCntDown received in rx header */
                FCnt32 = (get_fcntdwn(false) & 0xffff0000) | rxFCnt16;
                fromStored = false;
            }
            DEBUG_CRYPT("FCnt32:%" PRIu32" ", FCnt32);
            DEBUG_CRYPT_BUF(rx_payload+8, fCtrl.Bits.FOptsLen, "FOpts-rx", 0);
            LoRaMacEncrypt(0, rx_payload+8, fCtrl.Bits.FOptsLen, keys.NwkSEncKey, LoRaMacDevAddr, DOWN_LINK, FCnt32, macDecrypt);
            DEBUG_CRYPT_BUF(macDecrypt, fCtrl.Bits.FOptsLen, "FOpts-decrypt", 0);
            if (ProcessMacCommands(macDecrypt, 0, fCtrl.Bits.FOptsLen, snr) < 0) {
                if (fromStored) {
                    MAC_PRINTF("fromStored-");
                }
                MAC_PRINTF("FCnt32:%lu ", FCnt32);
            }
        } else {
            MAC_PRINTF("ProcessMacCommands-FOpts ");
            ProcessMacCommands( rx_payload, 8, fCtrl.Bits.FOptsLen + 8, snr);
        }
    }

    if (appPayloadStartIndex > 0)
    {
        frameLen = ( rx_size - 4 ) - appPayloadStartIndex;

        McpsIndication.Port = rxFPort;

        if (rxFPort == 0)
        {
            if( ( fCtrl.Bits.FOptsLen == 0 ) /*&& ( multicast == 0 )*/ )
            {
                uint8_t macDecrypt[16];
                LoRaMacPayloadDecrypt(rx_payload + appPayloadStartIndex,
                                      frameLen,
                                      keys.NwkSEncKey,
                                      address,
                                      DOWN_LINK,
                                      rxFCnt32,
                                      macDecrypt
                );

                // Decode frame payload MAC commands
                MAC_PRINTF("ProcessMacCommands-payload ");
                if (ProcessMacCommands( macDecrypt, 0, frameLen, snr) < 0) {
                    MAC_PRINTF(" rxFCnt32:%" PRIu32 " ", rxFCnt32);
                }
            }
            else
            {
                skipIndication = true;
            }
        }
        else
        {   /* rxFPort > 0 */
            MAC_PRINTF("rxFCnt32:%" PRIu32" %08" PRIx32" ", rxFCnt32, address);
            MAC_PRINTF("FCntDown%" PRIu32 " ", rxFCnt32);
            DEBUG_CRYPT(" addr%" PRIx32" len%u\r\n", address, frameLen);
            DEBUG_CRYPT_BUF(rx_payload + appPayloadStartIndex, frameLen, "rxEncd", 0);
            DEBUG_CRYPT_BUF(keys.AppSKey, 16, "AppSKey", 0);
            LoRaMacPayloadDecrypt(rx_payload + appPayloadStartIndex,
                                  frameLen,
                                  keys.AppSKey,
                                  address,
                                  DOWN_LINK,
                                  rxFCnt32,
                                  rxFRMPayload
            );

            if( skipIndication == false )
            {
                McpsIndication.Buffer = rxFRMPayload;
                McpsIndication.BufferSize = frameLen;
                McpsIndication.RxData = true;
            }
        }
    } // ..if have payload

    if (!skipIndication)
    {
        McpsIndication.AckReceived = fCtrl.Bits.Ack;
        McpsConfirm.AckReceived = fCtrl.Bits.Ack;

        if (LoRaMacPrimitives->MacMcpsIndication != NULL)
            LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );    // RxDone
    }

    if (McpsIndication.RxSlot == 1 || McpsIndication.RxSlot == 2)
        McpsIndication.ADR_ACK_CNT = 0;

    /* set FCntDwn to next expected value, if last NbTrans */
#ifdef LORAWAN_JOIN_EUI
    if (flags.uplink_in_progress <= 1 && last_up_macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_UP) {
        if (fCtrl.Bits.Ack) {
            FCntUp++;
        } else {
            MAC_PRINTF("\e[31mrx-!ack\e[0m\n");
        }
    }

    if (is_AFCntDown) {
        AFCntDown = rxFCnt32 + 1;
    } else {
        NFCntDown = rxFCnt32 + 1;
    }
#else
    /* if last NbTrans confirmed uplink ack'd ok: increment FCntUp */
    if (flags.uplink_in_progress <= 1 && fCtrl.Bits.Ack && last_up_macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_UP) {
        eeprom_increment_value(EEPROM_FCNTUP);  /* TODO handle ee-failure return */
    }

    if (is_AFCntDown)
        eeprom_write_word(EEPROM_AFCNTDWN, rxFCnt32 + 1);
    else
        eeprom_write_word(EEPROM_NFCNTDWN, rxFCnt32 + 1);
#endif /* !LORAWAN_JOIN_EUI */

    return 0;
} // ...rx_downlink()


#ifdef LORAWAN_JOIN_EUI
typedef union {
    struct {
        uint8_t mhdr;
        unsigned int joinNonce : 24;
        unsigned int Home_NetID : 24;
        uint32_t DevAddr;
        struct {
            uint8_t RX2dr       : 4;  // 0,1,2,3
            uint8_t RX1DRoffset : 3;  // 4,5,6
            uint8_t OptNeg      : 1;  // 7
        } DLSettings;
        uint8_t RxDelay;
    } __attribute__((packed)) fields;
    uint8_t octets[13];
} joinAccept_t;
#endif /* LORAWAN_JOIN_EUI  */


#define JOIN_ACCEPT_MAX_SIZE        34
static void
//OnRadioRxDone(uint8_t *rx_payload, uint16_t rx_size, int16_t rssi, int8_t snr, us_timestamp_t us_rxDone_at)
OnRadioRxDone(uint8_t rx_size, float rssi, float snr)
{
    LoRaMacEventInfoStatus_t status = LORAMAC_EVENT_INFO_STATUS_UNKNOWN_MTYPE;
    LoRaMacHeader_t macHdr;
#ifdef LORAWAN_JOIN_EUI
    uint8_t _jaDecrypted[JOIN_ACCEPT_MAX_SIZE];
    uint32_t mic, micRx;
    const uint8_t* key;
    const joinAccept_t* ja;
#endif /* LORAWAN_JOIN_EUI  */
    uint8_t pktHeaderLen = 0;

    McpsConfirm.AckReceived = false;
    McpsIndication.Rssi = rssi;
    McpsIndication.Snr = snr;
    McpsIndication.Port = 0;
    //McpsIndication.Multicast = 0;
    McpsIndication.FramePending = 0;
    McpsIndication.Buffer = NULL;
    McpsIndication.BufferSize = 0;
    McpsIndication.RxData = false;
    McpsIndication.AckReceived = false;
    McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

    if (MlmeConfirm.Status == LORAMAC_EVENT_INFO_STATUS_SENDING) {
        /* when regular downlink is sent in response to rejoin request */
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
    }

    if (LoRaMacDeviceClass != CLASS_C)
    {
        Radio::Sleep();
    }

    MAC_PRINTF("OnRadioRxDone(%u) RxSlot%d ", rx_size, McpsIndication.RxSlot);
    if (beacon_rx_done_payload(rx_size))
        return;

    macHdr.Value = Radio::radio.rx_buf[pktHeaderLen++];

    MAC_PRINTF(" rx-");
    print_mtype(macHdr.Bits.MType);
    switch (macHdr.Bits.MType)
    {
#ifdef LORAWAN_JOIN_EUI
        case FRAME_TYPE_JOIN_ACCEPT:
            /* always permitting join accept because it might be due to rejoin */
            if (rx_size >= JOIN_ACCEPT_MAX_SIZE) {
                MAC_PRINTF("joinAccept overSize %u\r\n", rx_size);
                return;
            }

            DEBUG_CRYPT_BUF(Radio::radio.rx_buf, rx_size, "rxBuf", 0);
            MAC_PRINTF("JoinReqType:%02x ", JoinReqType);
            if (JoinReqType == 0xff) {
                DEBUG_CRYPT_BUF(RootNwkKey, 16, "NwkKey", 0);
                key = RootNwkKey;
            } else {
                key = JSEncKey;
                DEBUG_CRYPT_BUF(JSEncKey, 16, "JSEncKey", 0);
            }
            LoRaMacJoinDecrypt( Radio::radio.rx_buf + 1, rx_size - 1, key, &_jaDecrypted[1]);
            DEBUG_CRYPT_BUF(_jaDecrypted, rx_size, "macbuf", 0);

            _jaDecrypted[0] = macHdr.Value;
            ja = (joinAccept_t*)_jaDecrypted;
            flags.OptNeg = ja->fields.DLSettings.OptNeg;

            if (flags.OptNeg) {
                uint8_t micBuf[40];
                uint8_t* ptr = micBuf;
                if (RootAppKey == NULL) {
                    MAC_PRINTF("OptNeg-without-AppKey ");
                    PrepareRxDoneAbort(LORAMAC_EVENT_INFO_STATUS_NO_APPKEY);
                    return;
                }
                *ptr++ = JoinReqType;
                memcpyr(ptr, LoRaMacJoinEui, 8);
                ptr += 8;
                *ptr++ = LoRaMacDevNonce & 0xff;
                *ptr++ = LoRaMacDevNonce >> 8;
                memcpy(ptr, _jaDecrypted, rx_size - LORAMAC_MFR_LEN);
                ptr += rx_size - LORAMAC_MFR_LEN;
                DEBUG_MIC_BUF_DOWN(JSIntKey, 16, "JSIntKey", ROW_MIC);
                DEBUG_MIC_BUF_DOWN(micBuf, ptr - micBuf, "jaMic-in", ROW_MIC+1);
                if (LoRaMacJoinComputeMic(false, micBuf, ptr - micBuf, JSIntKey, &mic ) < 0) {
                    MAC_PRINTF("cryptFail\r\n");
                    return;
                }
            } else {
                if (LoRaMacJoinComputeMic(false, _jaDecrypted, rx_size - LORAMAC_MFR_LEN, RootNwkKey, &mic ) < 0) {
                    MAC_PRINTF("cryptFail\r\n");
                    return;
                }
            }

            micRx = ( uint32_t )_jaDecrypted[rx_size - LORAMAC_MFR_LEN];
            micRx |= ( ( uint32_t )_jaDecrypted[rx_size - LORAMAC_MFR_LEN + 1] << 8 );
            micRx |= ( ( uint32_t )_jaDecrypted[rx_size - LORAMAC_MFR_LEN + 2] << 16 );
            micRx |= ( ( uint32_t )_jaDecrypted[rx_size - LORAMAC_MFR_LEN + 3] << 24 );

            MAC_PRINTF("JOIN_ACCEPT %u,OptNeg%" PRIu32" ", rx_size, flags.OptNeg);

            if (micRx == mic)
            {
                if (McpsIndication.RxSlot == 1) {  // no need for RX2 with good mic on rx1
                    RxWindowEvent2.detach();
                }

#ifdef LORAWAN_ROOT_APPKEY        
                if (flags.OptNeg) {
                    MlmeConfirm.fields.join.myJoinNonce = eeprom_read(EEPROM_JOINNONCE);
                    MlmeConfirm.fields.join.rxJoinNonce = ja->fields.joinNonce;
                    if (MlmeConfirm.fields.join.rxJoinNonce <= MlmeConfirm.fields.join.myJoinNonce) {
                        /* replay attack */
                        PrepareRxDoneAbort(LORAMAC_EVENT_INFO_STATUS_JOINNONCE);
                        return;
                    }
                    flags.need_ReKeyConf = 1;
                    LoRaMacJoinComputeSKeys_1v1( RootNwkKey, RootAppKey, _jaDecrypted+1, LoRaMacJoinEui, LoRaMacDevNonce, &keys);
                    eeprom_write_word(EEPROM_JOINNONCE, MlmeConfirm.fields.join.rxJoinNonce);
                } else
#endif /* LORAWAN_ROOT_APPKEY */
                    LoRaMacJoinComputeSKeys_1v0( RootNwkKey, _jaDecrypted+1, LoRaMacDevNonce, &keys);

                DEBUG_CRYPT_BUF(keys.AppSKey , 16, "create-AppSKey", 0);

                LoRaMacNetID = ja->fields.Home_NetID;
                LoRaMacDevAddr = ja->fields.DevAddr;

                // DLSettings
                LoRaMacParams.Rx1DrOffset = ja->fields.DLSettings.RX1DRoffset;
                LoRaMacParams.Rx2Channel.Datarate = ja->fields.DLSettings.RX2dr;

                // RxDelay
                LoRaMacParams.ReceiveDelay1_us = (ja->fields.RxDelay & 0x0f) * 1000000;
                if( LoRaMacParams.ReceiveDelay1_us == 0 )
                {
                    LoRaMacParams.ReceiveDelay1_us = 1000000;
                }
                LoRaMacParams.ReceiveDelay2_us = LoRaMacParams.ReceiveDelay1_us + 1000000;
                MAC_PRINTF("rx1droffset:%u, rx2dr%u rxDelays:%" PRIu32" %" PRIu32", devaddr:%08" PRIx32" ",
                    LoRaMacParams.Rx1DrOffset,
                    LoRaMacParams.Rx2Channel.Datarate,
                    LoRaMacParams.ReceiveDelay1_us,
                    LoRaMacParams.ReceiveDelay2_us,
                    LoRaMacDevAddr
                );

#if !( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )  // TODO
                //CFList
                if( ( rx_size - 1 ) > 16 )
                {
                    ChannelParams_t param;
                    param.DrRange.Value = ( LORAMAC_TX_MAX_DATARATE << 4 ) | LORAMAC_TX_MIN_DATARATE;

                    for( uint8_t i = 3, j = 0; i < ( 5 + 3 ); i++, j += 3 )
                    {
                        param.FreqHz = ( ( uint32_t )_jaDecrypted[13 + j] | ( ( uint32_t )_jaDecrypted[14 + j] << 8 ) | ( ( uint32_t )_jaDecrypted[15 + j] << 16 ) ) * 100;
                        if( param.FreqHz != 0 )
                        {
                            LoRaMacChannelAdd( i, param );
                        }
                        else
                        {
                            LoRaMacChannelRemove( i );
                        }
                    }
                }
#endif
                status = LORAMAC_EVENT_INFO_STATUS_OK;
                flags.IsLoRaMacNetworkJoined = true;
                LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;

                MAC_PRINTF("JoinReqType%x\r\n", JoinReqType);
                FCntUp = 0;
                NFCntDown = 0;
                AFCntDown = 0;
                ConfFCntDown = 0;
                RJcount0 = 0;
                McpsIndication.ADR_ACK_CNT = 0;

                rejoin.event.detach();

                // must always notify application layer
                MlmeConfirm.MlmeRequest = MLME_JOIN;
                region_session_start(LORAMAC_EVENT_INFO_STATUS_OK);
            }
            else
            {   /* join-accept mic fail */
                status = LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL;
                MAC_PRINTF("ja-mic-fail rx:%" PRIx32 " calc:%" PRIx32" size:%d\r\n", micRx, mic, rx_size);  
                if (MlmeIndication.MlmeIndication != MLME_NONE) {
                    MlmeIndication.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;      
                    if (LoRaMacPrimitives->MacMlmeIndication != NULL)
                        LoRaMacPrimitives->MacMlmeIndication(&MlmeIndication);
                }
            }
            break;
#endif /* LORAWAN_JOIN_EUI  */
        case FRAME_TYPE_DATA_CONFIRMED_DOWN:
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
            if (rx_downlink(pktHeaderLen, rx_size, snr) == 0)
                status = LORAMAC_EVENT_INFO_STATUS_OK;
            break;
        case FRAME_TYPE_PROPRIETARY:
            {
                McpsIndication.McpsIndication = MCPS_PROPRIETARY;
                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                McpsIndication.Buffer = Radio::radio.rx_buf;
                McpsIndication.BufferSize = rx_size - pktHeaderLen;

                if (LoRaMacPrimitives->MacMcpsIndication != NULL)
                    LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );    // RxDone
                break;
            }
        default:
            MAC_PRINTF("unknown frame type:%02x\r\n", macHdr.Value);
            PrepareRxDoneAbort(LORAMAC_EVENT_INFO_STATUS_UNKNOWN_MTYPE);
            break;
    } // ..switch( macHdr.Bits.MType )

    if (LoRaMacDeviceClass == CLASS_C) {
        if (McpsIndication.RxSlot == 1) {
            RxWindow2Setup();
            RxWindow2Start();   // immiediately to continuous rx2 reception
        } else if (McpsIndication.RxSlot == 2) {
            /* stop simulated rx timeout for classC rx2-continuous */
            RxWindowEvent2.detach();
        }
    }

    {
        if (flags.uplink_in_progress > 0)
            finish_uplink(status);

        mcps_confirm(status);  // RxDone
        mlme_confirm(status);
    }

    MAC_PRINTF("\r\n");

    flags.rxing = false;
} // ..OnRadioRxDone()

__attribute__((weak)) bool
beacon_rx_timeout()
{
    return false;
}

void OnRadioRxTimeout( void )
{
    MAC_PRINTF("OnRadioRxTimeout()%d ", McpsIndication.RxSlot);
#ifdef LORAWAN_JOIN_EUI
    MAC_PRINTF(",%u ", flags.IsLoRaMacNetworkJoined);
#endif /* LORAWAN_JOIN_EUI  */

    if (beacon_rx_timeout())
        return;

    if (McpsIndication.RxSlot == 1)
        RxWindow2Setup();

    if (LoRaMacDeviceClass != CLASS_C)
        Radio::Sleep();

    if (McpsIndication.RxSlot == 2)
    {
#ifdef LORAWAN_JOIN_EUI
        if (flags.IsLoRaMacNetworkJoined) {
#endif /* LORAWAN_JOIN_EUI  */
            if (uplinkMHDR->Bits.MType == FRAME_TYPE_DATA_UNCONFIRMED_UP) {
                /* sent once, stoping now */
                mcps_confirm(LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT);
            }

            mlme_confirm(LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT);
#ifdef LORAWAN_JOIN_EUI
        } else {
            if (++MlmeIndication.JoinRequestTrials < MaxJoinRequestTrials) {
                TxDelayedEvent.attach_us(OnTxDelayedIsr, 1000000);
                MAC_PRINTF("RxTImeout-join-tx-delay\r\n");
                MAC_PRINTF("join-try%u of%u ", MlmeIndication.JoinRequestTrials, MaxJoinRequestTrials);

                if (MlmeIndication.MlmeIndication != MLME_NONE) {
                    MlmeIndication.Status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;
                    if (LoRaMacPrimitives->MacMlmeIndication)
                        LoRaMacPrimitives->MacMlmeIndication(&MlmeIndication);
                }
            } else {
                mlme_confirm(LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL);
            }
        }
#endif /* LORAWAN_JOIN_EUI  */

        finish_uplink(LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT);
    } // ..if (McpsIndication.RxSlot == 2)

    if (LoRaMacDeviceClass == CLASS_C) {
        if (McpsIndication.RxSlot != 2) {
            RxWindow2Start();
        }
    } else
        flags.rxing = false;

    MAC_PRINTF("\r\n");

} // ..OnRadioRxTimeout()

__attribute__((weak)) void 
on_dio0_top_half(us_timestamp_t dio0_at)
{
}

static void OnRadioRxError( void )
{
    if( LoRaMacDeviceClass != CLASS_C )
    {
        Radio::Sleep();
    }
    else
    {
        RxWindow2Setup();
        RxWindow2Start();
    }

    if (McpsIndication.RxSlot == 2)
    {
        flags.uplink_in_progress = 0;  // TODO check
        mlme_confirm(LORAMAC_EVENT_INFO_STATUS_RX2_ERROR);
        mcps_confirm(LORAMAC_EVENT_INFO_STATUS_RX2_ERROR);
    }
} // ..OnRadioRxError

const RadioEvents_t rev = {
    /* Dio0_top_half */     on_dio0_top_half,
    /* TxDone_topHalf */    OnRadioTxDone,
    /* TxDone_botHalf */    NULL,
    /* TxTimeout  */        OnRadioTxTimeout,
    /* RxDone  */           OnRadioRxDone,
    /* RxTimeout  */        OnRadioRxTimeout,
    /* RxError  */          OnRadioRxError,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

LoRaMacStatus_t
LoRaMacInitialization( const LoRaMacPrimitives_t *primitives, const LoRaMacCallback_t *callbacks )
{
    if( primitives == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if( ( primitives->MacMcpsConfirm == NULL ) ||
        ( primitives->MacMcpsIndication == NULL ) ||
        ( primitives->MacMlmeConfirm == NULL ) ||
        ( primitives->MacMlmeIndication == NULL ) )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if (targetCheckLSE() < 0)
        return LORAMAC_STATUS_LSE;

    LoRaMacPrimitives = primitives;
    LoRaMacCallbacks = callbacks;

    LoRaMacDeviceClass = CLASS_A;

#ifdef DUTY_ENABLE
    DutyInit();
#endif /* DUTY_ENABLE */

    region_mac_init();

    ResetMacParameters( );

    // Initialize Radio driver

    LoRaMacClassBInitialization();
    Radio::Init( &rev );

    // Random seed initialization
    srand(Radio::Random());

    flags.PublicNetwork = true;
    Radio::SetPublicNetwork( flags.PublicNetwork );
    Radio::Sleep();

    McpsIndication.RxSlot = -1;
    function_pending = NULL;

    McpsConfirm.McpsRequest = MCPS_NONE;

#ifdef LORAWAN_JOIN_EUI
    MaxJoinRequestTrials = 1;
#else
    flags.have_SNwkSIntKey = 0;
    flags.have_NwkSEncKey = 0;
    flags.OptNeg = 0;
#endif /* !LORAWAN_JOIN_EUI  */

    LoRaMacCryptoInit();

    MlmeIndication.MlmeIndication = MLME_NONE;

    ADR_ACK_LIMIT = DEFAULT_ADR_ACK_LIMIT;
    ADR_ACK_DELAY = DEFAULT_ADR_ACK_DELAY;

    return LORAMAC_STATUS_OK;
} // ..LoRaMacInitialization()

void SendFrameOnChannel( uint8_t ch_num )
{
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;
    uint8_t tx_len = tx_buf_len;
    uint32_t mic;

    /* TODO: if beacon guard, defer until pingslot 0 */

    last_up_macHdr.Value = Radio::radio.tx_buf[0];
    if (last_up_macHdr.Bits.MType == FRAME_TYPE_DATA_UNCONFIRMED_UP ||
        last_up_macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_UP)
    {
        LoRaMacFrameCtrl_t* fCtrl = (LoRaMacFrameCtrl_t*)&Radio::radio.tx_buf[5];
        block_t block;
        uint32_t fcnt_up;
#ifdef LORAWAN_JOIN_EUI
        fcnt_up = FCntUp;
#else
        fcnt_up = eeprom_read(EEPROM_FCNTUP);
#endif /* LORAWAN_JOIN_EUI  */

        fCtrl->Bits.AdrAckReq = false;
        if (fCtrl->Bits.Adr) {
            if (McpsIndication.ADR_ACK_CNT >= ADR_ACK_LIMIT) {
                if (LoRaMacParamsDefaults.ChannelsDatarate > LORAMAC_TX_MIN_DATARATE || LoRaMacParams.ChannelsTxPower > LORAMAC_DEFAULT_TX_POWER || LoRaMacParams.NbEnabledChannels < LoRaMacParamsDefaults.NbEnabledChannels)
                    fCtrl->Bits.AdrAckReq = true;

                if (McpsIndication.ADR_ACK_CNT >= (ADR_ACK_LIMIT + ADR_ACK_DELAY)) {
                    /* if tx power less than default: increase tx power, otherwise decrease datarate */
                    if (LoRaMacParams.ChannelsTxPower > LORAMAC_DEFAULT_TX_POWER) {
                        LoRaMacParams.ChannelsTxPower--;
                    } else if (LoRaMacParams.ChannelsDatarate > LORAMAC_TX_MIN_DATARATE) {
                        LoRaMacParams.ChannelsDatarate--;
                        McpsIndication.ADR_ACK_CNT -= ADR_ACK_DELAY;
                    } else {
                        memcpy(LoRaMacParams.ChannelsMask, LoRaMacParamsDefaults.ChannelsMask, sizeof(LoRaMacParams.ChannelsMask));
                        LoRaMacParams.NbEnabledChannels = LoRaMacParamsDefaults.NbEnabledChannels;
                    }
                }
            }
        } // ..if (fCtrl->Bits.Adr)

        Radio::radio.tx_buf[6] = fcnt_up & 0xFF;
        Radio::radio.tx_buf[7] = ( fcnt_up >> 8 ) & 0xFF;

        block.b.header = 0x49;
        block.b.confFCnt = 0;
        block.b.dr = 0;
        block.b.ch = 0;
        block.b.dir = UP_LINK;
        block.b.DevAddr = LoRaMacDevAddr;
        block.b.FCnt = fcnt_up;
        block.b.zero8 = 0;
        block.b.lenMsg = tx_len;
        if (flags.OptNeg) {
            uint16_t cmacF, cmacS;
            cmacF = LoRaMacComputeMic(&block, Radio::radio.tx_buf, keys.FNwkSIntKey);

            block.b.confFCnt = ConfFCntDown;
            block.b.dr = LoRaMacParams.ChannelsDatarate;
            block.b.ch = ch_num;
            cmacS = LoRaMacComputeMic(&block, Radio::radio.tx_buf, keys.SNwkSIntKey) & 0xffff;
            mic = cmacS | (cmacF << 16);
            ConfFCntDown = 0;   /* single use */
        } else
            mic = LoRaMacComputeMic(&block, Radio::radio.tx_buf, keys.FNwkSIntKey);

        Radio::radio.tx_buf[tx_buf_len + 0] = mic & 0xFF;
        Radio::radio.tx_buf[tx_buf_len + 1] = ( mic >> 8 ) & 0xFF;
        Radio::radio.tx_buf[tx_buf_len + 2] = ( mic >> 16 ) & 0xFF;
        Radio::radio.tx_buf[tx_buf_len + 3] = ( mic >> 24 ) & 0xFF;
        tx_len += LORAMAC_MFR_LEN;
        MAC_PRINTF("FCntUp%u ", fcnt_up);

#ifdef LORAWAN_JOIN_EUI
        McpsConfirm.UpLinkCounter = FCntUp;
#else
        McpsConfirm.UpLinkCounter = eeprom_read(EEPROM_FCNTUP);
#endif /* !LORAWAN_JOIN_EUI  */

        if (flags.uplink_in_progress <= 1)
            McpsIndication.ADR_ACK_CNT++;

    } // ..if sending (un)conf

    txPowerIndex = region_LimitTxPower( LoRaMacParams.ChannelsTxPower );
    txPower = TxPowers[txPowerIndex];

    if (MlmeConfirm.MlmeRequest != MLME_NONE) {
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_SENDING;
    }
    if (McpsConfirm.McpsRequest != MCPS_NONE) {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_SENDING;
        McpsConfirm.Datarate = LoRaMacParams.ChannelsDatarate;
        McpsConfirm.TxPower = txPowerIndex;
        McpsConfirm.UpLinkFreqHz = Channels[ch_num].FreqHz;
    }

    Radio::SetChannel(Channels[ch_num].FreqHz);
    if (MlmeIndication.MlmeIndication != MLME_NONE)
        MlmeIndication.freqHz = Channels[ch_num].FreqHz;

    region_tx_setup(txPower, tx_len);

    // Store the time on air
    //McpsConfirm.TxTimeOnAir = TxTimeOnAir_us;
    //MlmeConfirm.TxTimeOnAir = TxTimeOnAir_us;

    // Send now
    if (Radio::Send(tx_len, LoRaMacParams.MaxListenTime, REGION_LBT_CHANNEL_FREE_TIME_us, REGION_LBT_RSSI_THRESHOLD_DBM) < 0) {
        mcps_confirm(LORAMAC_EVENT_INFO_STATUS_CHANNEL_BUSY);  // SendFrame fail
        return;
    }
    waitingFor = LORAMAC_STATUS_WAITING_FOR_TXDONE;
    MAC_PRINTF(" sfoc %u, %" PRIu32"hz\r\n", tx_len, Channels[ch_num].FreqHz);

    /* if this is unconfirmed up, and last NbTrans */
    if (last_up_macHdr.Bits.MType == FRAME_TYPE_DATA_UNCONFIRMED_UP && flags.uplink_in_progress <= 1) {
#ifdef LORAWAN_JOIN_EUI
        FCntUp++;
#else
        if (eeprom_increment_value(EEPROM_FCNTUP) < 0) {
            mcps_confirm(LORAMAC_EVENT_INFO_STATUS_INCR_FAIL);  // SendFrame fail
        }
#endif
    }
} // ..SendFrameOnChannel()

uint8_t CountBits( uint16_t mask, uint8_t nbBits )
{
    uint8_t nbActiveBits = 0;

    for( uint8_t j = 0; j < nbBits; j++ )
    {
        if( ( mask & ( 1 << j ) ) == ( 1 << j ) )
        {
            nbActiveBits++;
        }
    }
    return nbActiveBits;
}

void LoRaMacPrintStatus()
{
#ifdef MAC_DEBUG
    switch (waitingFor) {
        case LORAMAC_STATUS_WAITING_FOR_TXSTART: MAC_PRINTF("wait-TXSTART "); break;
        case LORAMAC_STATUS_WAITING_FOR_TXDONE:  MAC_PRINTF("wait-TXDONE "); break;
        case LORAMAC_STATUS_WAITING_FOR_RX1:     MAC_PRINTF("wait-RX1 "); break;
        case LORAMAC_STATUS_WAITING_FOR_RX2:     MAC_PRINTF("wait-RX2 "); break;
        default: break;
    }
    if (flags.uplink_in_progress > 0)
        MAC_PRINTF("uplink_in_progress:%u ", flags.uplink_in_progress);
    MAC_PRINTF("ConfFCntUp%u\r\n", ConfFCntUp);
    MAC_PRINTF("function_pending:%p\r\n", function_pending);
    MAC_PRINTF("rx delays:%u, %u, %u, %u\r\n",
        RxWindow1Delay_us,
        LoRaMacParams.ReceiveDelay1_us,
        RxWindow2Delay_us,
        LoRaMacParams.ReceiveDelay2_us
    );
    if (flags.uplink_in_progress) {
        MAC_PRINTF("since txDone:%u\r\n", Radio::lpt.read_us() - tx_done_at);
    }

    MAC_PRINTF("class-");
    switch (LoRaMacDeviceClass) {
        case CLASS_A:   MAC_PRINTF("A "); break;
        case CLASS_B:   MAC_PRINTF("B "); break;
        case CLASS_C:   MAC_PRINTF("C "); break;
    }
    MAC_PRINTF("\r\n");

    //Radio::PrintStatus();
#endif /* MAC_DEBUG */
}

us_timestamp_t LoRaMacReadTimer()
{
    return Radio::lpt.read_us();
}

int8_t
LoRaMacGetRxSlot()
{
    return McpsIndication.RxSlot;
}

void
LoRaMacUserContext()
{
    Radio::service();

    if (flags.OnTxDelayed) {
        OnTxDelayedTimerEvent();
        flags.OnTxDelayed = false;
    }
}

