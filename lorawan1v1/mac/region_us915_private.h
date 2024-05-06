
#include "LoRaMacPrivate.h"

#define TX_POWER_30_DBM                             0
#define TX_POWER_28_DBM                             1
#define TX_POWER_26_DBM                             2
#define TX_POWER_24_DBM                             3
#define TX_POWER_22_DBM                             4
#define TX_POWER_20_DBM                             5
#define TX_POWER_18_DBM                             6
#define TX_POWER_16_DBM                             7
#define TX_POWER_14_DBM                             8
#define TX_POWER_12_DBM                             9
#define TX_POWER_10_DBM                             10

#define LORAMAC_MIN_TX_POWER                        TX_POWER_10_DBM
#define LORAMAC_MAX_TX_POWER                        TX_POWER_30_DBM
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_20_DBM

#define LORA_MAX_NB_CHANNELS                        72
//
// Enables at least the usage of the 2 datarates.
#define JOIN_TRIAL_LIMIT            2

#define LORAMAC_TX_MIN_DATARATE                     DR_0
#define LORAMAC_TX_MAX_DATARATE                     DR_4
#define LORAMAC_DEFAULT_DATARATE                    DR_1

#define LORAMAC_RX_MIN_DATARATE                     DR_8
#define LORAMAC_RX_MAX_DATARATE                     DR_13
#define LORAMAC_MIN_RX1_DR_OFFSET                   0
#define LORAMAC_MAX_RX1_DR_OFFSET                   3
#define LORAMAC_FIRST_RX1_CHANNEL           ( (uint32_t) 923.3e6 )
#define LORAMAC_LAST_RX1_CHANNEL            ( (uint32_t) 927.5e6 )
#define LORAMAC_STEPWIDTH_RX1_CHANNEL       ( (uint32_t) 600e3 )

#define RX_WND_2_CHANNEL                                  { 923300000, DR_8 }

#define DEFAULT_ADR_ACK_LIMIT               64
#define DEFAULT_ADR_ACK_DELAY               32

#define REGION_LBT_RSSI_THRESHOLD_DBM           0
#define REGION_LBT_CHANNEL_FREE_TIME_us         0   /* no LBT in USA */

extern uint16_t ChannelsMaskRemaining[];

uint8_t CountNbEnabled125kHzChannels( uint16_t *channelsMask );
bool ValidateChannelMask( uint16_t* channelsMask );

