#include "LoRaMacPrivate.h"

#define LORA_MAX_NB_CHANNELS                        8

#define LORAMAC_TX_MIN_DATARATE                     DR_3
#define LORAMAC_TX_MAX_DATARATE                     DR_5

#define LORAMAC_RX_MIN_DATARATE                     DR_2
#define LORAMAC_RX_MAX_DATARATE                     DR_5

#define LORAMAC_DEFAULT_DATARATE                    DR_3

#define TX_POWER_20_DBM                             0
#define TX_POWER_14_DBM                             1
#define TX_POWER_11_DBM                             2
#define TX_POWER_08_DBM                             3
#define TX_POWER_05_DBM                             4
#define TX_POWER_02_DBM                             5

#define LORAMAC_MAX_TX_POWER                        TX_POWER_14_DBM
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_14_DBM
#define LORAMAC_MIN_TX_POWER                        TX_POWER_02_DBM

#define LORAMAC_MIN_RX1_DR_OFFSET                   0
#define LORAMAC_MAX_RX1_DR_OFFSET                   0

#define RX_WND_2_CHANNEL                                  { 921400000, DR_2 }

// Channel = { Frequency [Hz], { ( ( DrMax << 4 ) | DrMin ) }, Band }
#define LC1                { 923200000, { ( ( DR_5 << 4 ) | DR_2 ) }, 0 } // sf7-10 CH37
#define LC2                { 923400000, { ( ( DR_5 << 4 ) | DR_2 ) }, 0 } // sf7-10 CH38
/* other channels given by mac command */

#define DEFAULT_ADR_ACK_LIMIT               64
#define DEFAULT_ADR_ACK_DELAY               32

#define REGION_LBT_RSSI_THRESHOLD_DBM           -80
#define REGION_LBT_CHANNEL_FREE_TIME_us         5000

