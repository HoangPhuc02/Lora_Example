#include "lorawan_board.h"
#if defined(USE_BAND_915_HYBRID) || defined(USE_BAND_915)
#include <stdint.h>
#include "LoRaMacPrivate.h"

ChannelParams_t Channels[LORA_MAX_NB_CHANNELS];
uint16_t ChannelsMaskRemaining[6];
const int8_t TxPowers[]    = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 };

//                        DR:   0  1  2  3  4  5  6  7   8   9  10  11  12  13  14  15
const uint8_t Datarates[]  = { 10, 9, 8, 7, 8, 0, 0, 0, 12, 11, 10,  9,  8,  7,  0,  0};

const uint8_t MaxPayloadOfDatarate[] = { 11, 53, 125, 242, 242, 0, 0, 0, 53, 129, 242, 242, 242, 242, 0, 0 };

bool DisableChannelInMask( uint8_t id, uint16_t* mask )
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

/*!
 * Up/Down link data rates offset definition
 */
const int8_t datarateOffsets[5][4] =
{
    { DR_10, DR_9 , DR_8 , DR_8  }, // DR_0
    { DR_11, DR_10, DR_9 , DR_8  }, // DR_1
    { DR_12, DR_11, DR_10, DR_9  }, // DR_2
    { DR_13, DR_12, DR_11, DR_10 }, // DR_3
    { DR_13, DR_13, DR_12, DR_11 }, // DR_4
};

uint32_t region_GetRxBandwidth( int8_t datarate )
{
    if( datarate >= DR_4 )
    {// LoRa 500 kHz
        return 2;
    }
    return 0; // LoRa 125 kHz
}

uint16_t region_GetRxSymbolTimeout( int8_t datarate )
{
    switch( datarate )
    {
        case DR_0:       // SF10 - BW125
            return 5;

        case DR_1:       // SF9  - BW125
        case DR_2:       // SF8  - BW125
        case DR_8:       // SF12 - BW500
        case DR_9:       // SF11 - BW500
        case DR_10:      // SF10 - BW500
            return 8;

        case DR_3:       // SF7  - BW125
        case DR_11:     // SF9  - BW500
            return 10;

        case DR_4:       // SF8  - BW500
        case DR_12:      // SF8  - BW500
            return 14;

        case DR_13:      // SF7  - BW500
            return 16;

        default:
            return 0;   // LoRa 125 kHz
    }
}

void region_rx1_setup(uint8_t ch)
{
    int8_t datarate = datarateOffsets[LoRaMacParams.ChannelsDatarate][LoRaMacParams.Rx1DrOffset];
    if( datarate < 0 )
    {
        datarate = DR_0;
    }

    RxWindowSetup(
        LORAMAC_FIRST_RX1_CHANNEL + ( ch % 8 ) * LORAMAC_STEPWIDTH_RX1_CHANNEL,
        datarate,
        region_GetRxBandwidth(datarate),
        region_GetRxSymbolTimeout(datarate)
    );
}

bool ValidateChannelMask( uint16_t* channelsMask )
{
    bool chanMaskState = false;
    uint16_t block1 = 0;
    uint16_t block2 = 0;
    uint8_t index = 0;

    for( uint8_t i = 0; i < 4; i++ )
    {
        block1 = channelsMask[i] & 0x00FF;
        block2 = channelsMask[i] & 0xFF00;

        if( ( CountBits( block1, 16 ) > 5 ) && ( chanMaskState == false ) )
        {
            channelsMask[i] &= block1;
            channelsMask[4] = 1 << ( i * 2 );
            chanMaskState = true;
            index = i;
        }
        else if( ( CountBits( block2, 16 ) > 5 ) && ( chanMaskState == false ) )
        {
            channelsMask[i] &= block2;
            channelsMask[4] = 1 << ( i * 2 + 1 );
            chanMaskState = true;
            index = i;
        }
    }

    // Do only change the channel mask, if we have found a valid block.
    if( chanMaskState == true )
    {
        for( uint8_t i = 0; i < 4; i++ )
        {
            if( i != index )
            {
                channelsMask[i] = 0;
            }
        }
    }
    return chanMaskState;
}

void region_adr_request(adr_t* adr)
{
    if( adr->chMaskCntl == 6 )
    {
        // Enable all 125 kHz channels
        adr->channelsMask[0] = 0xFFFF;
        adr->channelsMask[1] = 0xFFFF;
        adr->channelsMask[2] = 0xFFFF;
        adr->channelsMask[3] = 0xFFFF;
        // Apply chMask to channels 64 to 71
        adr->channelsMask[4] = adr->chMask;
    }
    else if( adr->chMaskCntl == 7 )
    {
        // Disable all 125 kHz channels
        adr->channelsMask[0] = 0x0000;
        adr->channelsMask[1] = 0x0000;
        adr->channelsMask[2] = 0x0000;
        adr->channelsMask[3] = 0x0000;
        // Apply chMask to channels 64 to 71
        adr->channelsMask[4] = adr->chMask;
    }
    else if( adr->chMaskCntl == 5 )
    {
        // RFU
        adr->status &= 0xFE; // Channel mask KO
    }
    else
    {
        adr->channelsMask[adr->chMaskCntl] = adr->chMask;

        // FCC 15.247 paragraph F mandates to hop on at least 2 125 kHz channels
        if( ( adr->datarate < DR_4 ) && ( CountNbEnabled125kHzChannels( adr->channelsMask ) < 2 ) )
        {
            adr->status &= 0xFE; // Channel mask KO
        }

#if defined( USE_BAND_915_HYBRID )
        if( ValidateChannelMask( adr->channelsMask ) == false )
        {
            adr->status &= 0xFE; // Channel mask KO
        }
#endif
    }


    if ((adr->status & 0x07) == 0x07) {
        // Reset ChannelsMaskRemaining to the new ChannelsMask
        ChannelsMaskRemaining[0] &= adr->channelsMask[0];
        ChannelsMaskRemaining[1] &= adr->channelsMask[1];
        ChannelsMaskRemaining[2] &= adr->channelsMask[2];
        ChannelsMaskRemaining[3] &= adr->channelsMask[3];
        ChannelsMaskRemaining[4] = adr->channelsMask[4];
        ChannelsMaskRemaining[5] = adr->channelsMask[5];
    }
}

uint8_t region_CountNbEnabledChannels()
{
    return CountNbEnabled125kHzChannels(LoRaMacParams.ChannelsMask) + CountBits(LoRaMacParams.ChannelsMask[4], 16);
}

uint8_t CountNbEnabled125kHzChannels( uint16_t *channelsMask )
{
    uint8_t nb125kHzChannels = 0;

    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS - 8; i += 16, k++ )
    {
        nb125kHzChannels += CountBits( channelsMask[k], 16 );
    }

    return nb125kHzChannels;
}

static bool SetNextChannel( )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];

    memset( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

    if( CountNbEnabled125kHzChannels( ChannelsMaskRemaining ) == 0 )
    { // Restore default channels
        memcpy( ( uint8_t* ) ChannelsMaskRemaining, ( uint8_t* ) LoRaMacParams.ChannelsMask, 8 );
    }
    if( ( LoRaMacParams.ChannelsDatarate >= DR_4 ) && ( ( ChannelsMaskRemaining[4] & 0x00FF ) == 0 ) )
    { // Make sure, that the channels are activated
        ChannelsMaskRemaining[4] = LoRaMacParams.ChannelsMask[4];
    }

    // Search how many channels are enabled
    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( ChannelsMaskRemaining[k] & ( 1 << j ) ) != 0 )
            {
                if( Channels[i + j].FreqHz == 0 )
                { // Check if the channel is enabled
                    continue;
                }
                if( ( ( Channels[i + j].DrRange.Fields.Min <= LoRaMacParams.ChannelsDatarate ) &&
                      ( LoRaMacParams.ChannelsDatarate <= Channels[i + j].DrRange.Fields.Max ) ) == false )
                { // Check if the current channel selection supports the given datarate
                    continue;
                }
                enabledChannels[nbEnabledChannels++] = i + j;
            }
        }
    }

    if( nbEnabledChannels > 0 )
    {
        Channel = enabledChannels[random_at_most( nbEnabledChannels - 1 )];
        if( Channel < ( LORA_MAX_NB_CHANNELS - 8 ) )
        {
            DisableChannelInMask( Channel, ChannelsMaskRemaining );
        }
        return true;
    }
    else
    {
        // Datarate not supported by any channel
        return false;
    }
}

void region_ScheduleTx( )
{
    // Select channel
    while( SetNextChannel() == false )
    {
        // Set the default datarate
        LoRaMacParams.ChannelsDatarate = LoRaMacParamsDefaults.ChannelsDatarate;
    }

    //MAC_PRINTF("ch%u ", Channel);
    SendFrameOnChannel( Channel );
}

void region_tx_setup(int8_t dbm, uint8_t pktLen)
{
    int8_t datarate = Datarates[LoRaMacParams.ChannelsDatarate];

    Radio::set_tx_dbm(dbm);

    //MAC_PRINTF("txsetup sf%d dr%u ", datarate, LoRaMacParams.ChannelsDatarate);

    if( LoRaMacParams.ChannelsDatarate >= DR_4 )
    { // High speed LoRa channel BW500 kHz
        //TxTimeOnAir_us = Radio.TimeOnAir_us( MODEM_LORA, pktLen );
        Radio::LoRaModemConfig(500, datarate, 1);
        Radio::LoRaPacketConfig(8, false, true, false);
    }
    else
    { // Normal LoRa channel
        //TxTimeOnAir_us = Radio.TimeOnAir_us( MODEM_LORA, pktLen );
        Radio::LoRaModemConfig(125, datarate, 1);
        Radio::LoRaPacketConfig(8, false, true, false);
    }
}

#define RECEIVE_DELAY2_us                           2000000
#define JOIN_ACCEPT_DELAY1_us                          5000000
#define JOIN_ACCEPT_DELAY2_us                          6000000
const LoRaMacParams_t LoRaMacParamsDefaults = {
    /* int8_t ChannelsTxPower; */   LORAMAC_DEFAULT_TX_POWER,
    /* int8_t ChannelsDatarate;*/   LORAMAC_DEFAULT_DATARATE,
    /* uint32_t MaxRxWindow_us;*/   MAX_RX_WINDOW_us,
    /* uint32_t ReceiveDelay1_us;*/ RECEIVE_DELAY1_us,
    /* uint32_t ReceiveDelay2_us;*/ RECEIVE_DELAY2_us,
#ifdef LORAWAN_JOIN_EUI
    /* uint32_t JoinAcceptDelay1_us;*/  JOIN_ACCEPT_DELAY1_us,
    /* uint32_t JoinAcceptDelay2_us;*/  JOIN_ACCEPT_DELAY2_us,
#endif /* LORAWAN_JOIN_EUI  */
    /* uint8_t NbTrans;*/   1,
    /* uint8_t Rx1DrOffset;*/   0,
    /* Rx2ChannelParams_t Rx2Channel;*/ RX_WND_2_CHANNEL,
#if defined( USE_BAND_915 )
    /* uint16_t ChannelsMask[6];*/ { 0xffff, 0xffff, 0xffff, 0xffff, 0x00ff, 0x0000},
    /* uint8_t NbEnabledChannels;*/ 72,
#elif defined( USE_BAND_915_HYBRID )
    #if defined(HYBRID_H)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0x0000, 0x0000, 0xff00, 0x0080, 0x0000},
    #elif defined(HYBRID_G)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0x0000, 0x0000, 0x00ff, 0x0040, 0x0000},
    #elif defined(HYBRID_F)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0x0000, 0xff00, 0x0000, 0x0020, 0x0000},
    #elif defined(HYBRID_E)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0x0000, 0x00ff, 0x0000, 0x0010, 0x0000},
    #elif defined(HYBRID_D)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0xff00, 0x0000, 0x0000, 0x0008, 0x0000},
    #elif defined(HYBRID_C)
        /* uint16_t ChannelsMask[6];*/ { 0x0000, 0x00ff, 0x0000, 0x0000, 0x0004, 0x0000},
    #elif defined(HYBRID_B)
        /* uint16_t ChannelsMask[6];*/ { 0xff00, 0x0000, 0x0000, 0x0000, 0x0002, 0x0000},
    #else
        /* uint16_t ChannelsMask[6];*/ { 0x00ff, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000},
    #endif
    /* uint8_t NbEnabledChannels;*/ 9,
#endif
    /* us_timestamp_t MaxListenTime */ 0    /* no LBT in USA */
};

void region_mac_init()
{
    // 125 kHz channels
    for( uint8_t i = 0; i < LORA_MAX_NB_CHANNELS - 8; i++ )
    {
        Channels[i].FreqHz = 902.3e6 + i * 200e3;
        Channels[i].DrRange.Value = ( DR_3 << 4 ) | DR_0;
        Channels[i].Band = 0;
    }
    // 500 kHz channels
    for( uint8_t i = LORA_MAX_NB_CHANNELS - 8; i < LORA_MAX_NB_CHANNELS; i++ )
    {
        Channels[i].FreqHz = 903.0e6 + ( i - ( LORA_MAX_NB_CHANNELS - 8 ) ) * 1.6e6;
        Channels[i].DrRange.Value = ( DR_4 << 4 ) | DR_4;
        Channels[i].Band = 0;
    }
}

int8_t
region_LimitTxPower( int8_t txPower )
{
    int8_t resultTxPower = txPower;
 
    if( ( LoRaMacParams.ChannelsDatarate == DR_4 ) ||
        ( ( LoRaMacParams.ChannelsDatarate >= DR_8 ) && ( LoRaMacParams.ChannelsDatarate <= DR_13 ) ) )
    {// Limit tx power to max 26dBm
        resultTxPower =  MAX( txPower, TX_POWER_26_DBM );
    }
    else
    {
        if( CountNbEnabled125kHzChannels( LoRaMacParams.ChannelsMask ) < 50 )
        {// Limit tx power to max 21dBm
            resultTxPower = MAX( txPower, TX_POWER_20_DBM );
        }
    }
    return resultTxPower;
}

#ifdef LORAWAN_JOIN_EUI
int8_t
region_AlternateDatarate( uint16_t nbTrials )
{
    if (region_CountNbEnabledChannels() < LoRaMacParamsDefaults.NbEnabledChannels) {
        memcpy(LoRaMacParams.ChannelsMask, LoRaMacParamsDefaults.ChannelsMask, sizeof(LoRaMacParams.ChannelsMask));
        LoRaMacParams.NbEnabledChannels = LoRaMacParamsDefaults.NbEnabledChannels;
    }

    if( ( nbTrials & 0x01 ) == 0x01 )
    {
        return DR_4;
    }
    else
    {
        return DR_0;
    }
}
#endif /* LORAWAN_JOIN_EUI  */

void region_session_start(LoRaMacEventInfoStatus_t status) { }

#endif /* USE_BAND_915_HYBRID || USE_BAND_915 */
