//#include "Commissioning.h"
#include "lorawan_board.h"
#if defined(USE_BAND_ARIB_8CH)
#include <stdint.h>
#include "LoRaMacPrivate.h"

const uint8_t MaxPayloadOfDatarate[] = { 51, 51, 51, 115, 242, 242, 242, 242 };
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2
    /* other channels given by mac command */
};

uint32_t region_GetRxBandwidth( int8_t datarate )
{
    if( datarate == DR_6 )
    {// LoRa 250 kHz
        return 1;
    }
    return 0; // LoRa 125 kHz
}

uint16_t region_GetRxSymbolTimeout( int8_t datarate )
{
    if( ( datarate == DR_3 ) || ( datarate == DR_4 ) )
    { // DR_4, DR_3
        return 8;
    }
    else if( datarate == DR_5 )
    {
        return 10;
    }
    else if( datarate == DR_6 )
    {
        return 14;
    }
    return 5; // DR_2, DR_1, DR_0
}

void region_rx1_setup(uint8_t chan)
{
    int8_t datarate = LoRaMacParams.ChannelsDatarate - LoRaMacParams.Rx1DrOffset;
    if( datarate < 0 )
    {
        datarate = DR_0;
    }

    RxWindowSetup(
        Channels[chan].FreqHz,
        datarate,
        region_GetRxBandwidth(datarate),
        region_GetRxSymbolTimeout(datarate)
    );
}

static bool SetNextChannel(LoRaMacStatus_t* status)
{
    uint8_t nbEnabledChannels = 0;
    uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];

    memset( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

    *status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    if( CountBits( LoRaMacParams.ChannelsMask[0], 16 ) == 0 )
    {
        // Re-enable default channels, if no channel is enabled
        LoRaMacParams.ChannelsMask[0] = LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
    }

    // Search how many channels are enabled
    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( LoRaMacParams.ChannelsMask[k] & ( 1 << j ) ) != 0 )
            {
                if( Channels[i + j].FreqHz == 0 )
                { // Check if the channel is enabled
                    continue;
                }
#ifdef OVER_THE_AIR_ACTIVATION
                if (!flags.IsLoRaMacNetworkJoined)
                {
                    if( ( JOIN_CHANNELS & ( 1 << j ) ) == 0 )
                    {
                        continue;
                    }
                }
#endif /* OVER_THE_AIR_ACTIVATION  */
                if( ( ( Channels[i + j].DrRange.Fields.Min <= LoRaMacParams.ChannelsDatarate ) &&
                      ( LoRaMacParams.ChannelsDatarate <= Channels[i + j].DrRange.Fields.Max ) ) == false )
                { // Check if the current channel selection supports the given datarate
                    *status = LORAMAC_STATUS_DATARATE_INVALID;
                    continue;
                }
                enabledChannels[nbEnabledChannels++] = i + j;
            }
        }
    }

    if( nbEnabledChannels > 0 )
    {
        Channel = enabledChannels[random_at_most(nbEnabledChannels - 1)];
        *status = LORAMAC_STATUS_OK;
        return true;
    }
    else
    {
        // Datarate not supported by any channel
        return false;
    }
}


static us_timestamp_t defer_uplink_us;

void region_ScheduleTx( )
{
    LoRaMacStatus_t ret;

    if (defer_uplink_us > 0) {
        TxDelayedEvent.attach_us(OnTxDelayedIsr, defer_uplink_us);
        defer_uplink_us = 0;
        return;
    }

    if (LoRaMacDeviceClass == CLASS_C) {
        Radio::Standby();
    }

    // Select channel
    if (!SetNextChannel(&ret))
    {
        // Set the default datarate
        LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;
        // re-enable default channels
        LoRaMacParams.ChannelsMask[0] = LoRaMacParamsDefaults.ChannelsMask[0];
        if (!SetNextChannel(&ret))
            return;
    }

    // Schedule transmission of frame
    // Try to send now
    SendFrameOnChannel( Channel );
}

#define RECEIVE_DELAY2_us                           2000000
#define JOIN_ACCEPT_DELAY1_us                          5000000
#define JOIN_ACCEPT_DELAY2_us                          6000000
const LoRaMacParams_t LoRaMacParamsDefaults = {
    /* int8_t ChannelsTxPower */    LORAMAC_DEFAULT_TX_POWER,
    /* int8_t ChannelsDatarate */   LORAMAC_DEFAULT_DATARATE,
    /* uint32_t MaxRxWindow_us */   MAX_RX_WINDOW_us,
    /* uint32_t ReceiveDelay1_us */ RECEIVE_DELAY1_us,
    /* uint32_t ReceiveDelay2_us */ RECEIVE_DELAY2_us,
#ifdef LORAWAN_JOIN_EUI
    /* uint32_t JoinAcceptDelay1_us */  JOIN_ACCEPT_DELAY1_us,
    /* uint32_t JoinAcceptDelay2_us */  JOIN_ACCEPT_DELAY2_us,
#endif /* LORAWAN_JOIN_EUI  */
    /* uint8_t NbTrans */   1,
    /* uint8_t Rx1DrOffset */   0,
    /* Rx2ChannelParams_t Rx2Channel */ RX_WND_2_CHANNEL,
    /* uint16_t ChannelsMask[6] */ { (LC(1)+LC(2)), 0, 0, 0, 0, 0},/* only boot channels enabled */
    /* uint8_t NbEnabledChannels */ 2,
    /* us_timestamp_t MaxListenTime */ 4000000
};

void region_mac_init()
{
}

void region_adr_request(adr_t* adr)
{
    uint8_t i;

    if( ( adr->chMaskCntl == 0 ) && ( adr->chMask == 0 ) )
    {
        adr->status &= 0xFE; // Channel mask KO
    }
    else if( ( ( adr->chMaskCntl >= 1 ) && ( adr->chMaskCntl <= 5 )) ||
             ( adr->chMaskCntl >= 7 ) )
    {
        // RFU
        adr->status &= 0xFE; // Channel mask KO
    }
    else
    {
        for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
        {
            if( adr->chMaskCntl == 6 )
            {
                if( Channels[i].FreqHz != 0 )
                {
                    adr->chMask |= 1 << i;
                }
            }
            else
            {
                if( ( ( adr->chMask & ( 1 << i ) ) != 0 ) &&
                    ( Channels[i].FreqHz == 0 ) )
                {// Trying to enable an undefined channel
                    adr->status &= 0xFE; // Channel mask KO
                }
            }
        }
        adr->channelsMask[0] = adr->chMask;
        MAC_PRINTF("arib set channelsMask:%04x ", adr->channelsMask[0]);
    }
}

void region_tx_setup(int8_t txPower, uint8_t pktLen)
{
    int8_t datarate = Datarates[LoRaMacParams.ChannelsDatarate];

    Radio::set_tx_dbm(txPower);

    if( LoRaMacParams.ChannelsDatarate == DR_7 )
    { // High Speed FSK channel
        //TxTimeOnAir_us = Radio.TimeOnAir_us( MODEM_FSK, LoRaMacBufferPktLen );
        Radio::GFSKModemConfig(datarate * 1000, 50, 25000);
        Radio::GFSKPacketConfig(5, false, true);
    }
    else if( LoRaMacParams.ChannelsDatarate == DR_6 )
    { // High speed LoRa channel
        //TxTimeOnAir_us = Radio.TimeOnAir_us( MODEM_LORA, LoRaMacBufferPktLen );
        Radio::LoRaModemConfig(250, datarate, 1);
        Radio::LoRaPacketConfig(8, false, true, false);
    }
    else
    { // Normal LoRa channel
        Radio::LoRaModemConfig(125, datarate, 1);
        Radio::LoRaPacketConfig(8, false, true, false);
        //TxTimeOnAir_us = Radio.TimeOnAir_us( MODEM_LORA, LoRaMacBufferPktLen );
    }
}

static bool DisableChannelInMask( uint8_t id, uint16_t* mask )
{
    uint8_t index = 0;
    index = id / 16;

    if( ( index > 4 ) || ( id >= LORA_MAX_NB_CHANNELS ) )
    {
        return false;
    }

    // Deactivate channel
    mask[index] &= ~( 1 << ( id % 16 ) );

    return true;
}

LoRaMacStatus_t LoRaMacChannelRemove( uint8_t id )
{
    /*if( ( LoRaMacState & LORAMAC_TX_RUNNING ) == LORAMAC_TX_RUNNING )
    {
        if( ( LoRaMacState & LORAMAC_TX_CONFIG ) != LORAMAC_TX_CONFIG )
        {
            return LORAMAC_STATUS_BUSY;
        }
    }*/

    if( ( id < 3 ) || ( id >= LORA_MAX_NB_CHANNELS ) )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    else
    {
        // Remove the channel from the list of channels
        Channels[id] = ( ChannelParams_t ){ 0, { 0 }, 0 };

        // Disable the channel as it doesn't exist anymore
        if( DisableChannelInMask( id, LoRaMacParams.ChannelsMask ) == false )
        {
            return LORAMAC_STATUS_PARAMETER_INVALID;
        }
    }
    return LORAMAC_STATUS_OK;
}

static bool ValidateDrRange( DrRange_t drRange, int8_t min, int8_t max )
{
    int8_t drMin = drRange.Fields.Min & 0x0F;
    int8_t drMax = drRange.Fields.Max & 0x0F;

    if( drMin > drMax )
    {
        return false;
    }
    if( ValueInRange( drMin, min, max ) == false )
    {
        return false;
    }
    if( ValueInRange( drMax, min, max ) == false )
    {
        return false;
    }
    return true;
}

LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params )
{
    bool datarateInvalid = false;
    bool frequencyInvalid = false;
    uint8_t band = 0;

    //MAC_PRINTF("channelAdd(%u,%u)\r\n", id, params.Frequency);
    // The id must not exceed LORA_MAX_NB_CHANNELS
    if( id >= LORA_MAX_NB_CHANNELS )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    // Validate the datarate
    if( ValidateDrRange( params.DrRange, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == false )
    {
        datarateInvalid = true;
    }

    // Validate the frequency
    if( ( Radio::CheckRfFrequency( params.FreqHz ) == true ) && ( params.FreqHz > 0 ) && ( frequencyInvalid == false ) )
    {
        frequencyInvalid = false;
    }
    else
    {
        frequencyInvalid = true;
    }

    if( ( datarateInvalid == true ) && ( frequencyInvalid == true ) )
    {
        return LORAMAC_STATUS_FREQ_AND_DR_INVALID;
    }
    if( datarateInvalid == true )
    {
        return LORAMAC_STATUS_DATARATE_INVALID;
    }
    if( frequencyInvalid == true )
    {
        return LORAMAC_STATUS_FREQUENCY_INVALID;
    }

    // Every parameter is valid, activate the channel
    Channels[id] = params;
    Channels[id].Band = band;
    LoRaMacParams.ChannelsMask[0] |= ( 1 << id );

    return LORAMAC_STATUS_OK;
}

void
region_session_start(LoRaMacEventInfoStatus_t status)
{
    McpsReq_t mcpsReq;
        
    if (LoRaMacParams.ChannelsMask[0] == 0x003c) {
        return;
    }
    /* send mac commands until desired channel mask */

    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.fPort = 1;
    mcpsReq.Req.fBuffer = NULL;
    mcpsReq.Req.fBufferSize = 0;
    mcpsReq.Req.Datarate = DR_3;

    defer_uplink_us = 2000000 + random_at_most(2000000);
    LoRaMacMcpsRequest( &mcpsReq );
}

int8_t region_LimitTxPower( int8_t txPower )
{
    return txPower;
}

uint8_t region_CountNbEnabledChannels()
{
    return CountBits(LoRaMacParams.ChannelsMask[4], 16);
}

#ifdef LORAWAN_JOIN_EUI
int8_t
region_AlternateDatarate( uint16_t nbTrials )
{
    if( ( nbTrials % 48 ) == 0 )
    {
        return DR_0;
    }
    else if( ( nbTrials % 32 ) == 0 )
    {
        return DR_1;
    }
    else if( ( nbTrials % 24 ) == 0 )
    {
        return DR_2;
    }
    else if( ( nbTrials % 16 ) == 0 )
    {
        return DR_3;
    }
    else if( ( nbTrials % 8 ) == 0 )
    {
        return DR_4;
    }
    else
    {
        return DR_5;
    }
}
#endif /* LORAWAN_JOIN_EUI  */

#endif /* USE_BAND_ARIB_8CH */

