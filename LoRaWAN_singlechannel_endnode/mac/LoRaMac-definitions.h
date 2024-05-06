/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRa MAC layer global definitions

License: Revised BSD License, see LICENSE.TXT file include in the project
*/
#ifndef __LORAMAC_BOARD_H__
#define __LORAMAC_BOARD_H__

/*!
 * Returns individual channel mask
 *
 * \param[IN] channelIndex Channel index 1 based
 * \retval channelMask
 */
#define LC( channelIndex )            ( uint16_t )( 1 << ( channelIndex - 1 ) )
#if defined(SX128x_H)

    #define LORAMAC_DEFAULT_DATARATE                    5       // 5=sf7
    #define LORAMAC_DEFAULT_TX_POWER                    0
    #define LORAMAC_MAX_TX_POWER                        0
    #define LORAMAC_MIN_TX_POWER                        1
    #define LORAMAC_MIN_DATARATE                        0   // slowest
    #define LORAMAC_MAX_DATARATE                        9   // fastest

#elif defined( USE_BAND_915_SINGLE ) 

/*!
 * LoRaMac datarates definition
 */
//#define DR_0                                        0  // SF10 - BW125 |
//#define DR_1                                        1  // SF9  - BW125 |
//#define DR_2                                        2  // SF8  - BW125 +-> Up link
//#define DR_3                                        3  // SF7  - BW125 |
//#define DR_4                                        4  // SF8  - BW500 |
//#define DR_5                                        5  // RFU
//#define DR_6                                        6  // RFU
//#define DR_7                                        7  // RFU
#define DR_8                                        8  // SF12 - BW500 |
#define DR_9                                        9  // SF11 - BW500 |
#define DR_10                                       10 // SF10 - BW500 |
#define DR_11                                       11 // SF9  - BW500 |
#define DR_12                                       12 // SF8  - BW500 +-> Down link
#define DR_13                                       13 // SF7  - BW500 |
#define DR_14                                       14 // RFU          |
#define DR_15                                       15 // RFU          |

#define LORAMAC_MAX_TX_POWER                        TX_POWER_20_DBM
#define LORAMAC_MIN_TX_POWER                        TX_POWER_00_DBM

/*!
 * LoRaMac TxPower definition
 */
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
#define TX_POWER_08_DBM                             11
#define TX_POWER_06_DBM                             12
#define TX_POWER_04_DBM                             13
#define TX_POWER_02_DBM                             14
#define TX_POWER_00_DBM                             15

/*!
 * LoRaMac maximum number of bands
 */
#define LORA_MAX_NB_BANDS                           1
//
// Band = { DutyCycle, TxMaxPower, LastTxDoneTime, TimeOff }
#define BAND0              { 1, TX_POWER_20_DBM, 0/*,  0*/ } //  100.0 %


/*!
 * Default datarate used by the node
 */
//#define LORAMAC_DEFAULT_DATARATE                    DR_8   // SINGLE sf12      8192us
//#define LORAMAC_DEFAULT_DATARATE                    DR_9   // SINGLE sf11      4096us
//#define LORAMAC_DEFAULT_DATARATE                    DR_10   // SINGLE sf10   2048us
//#define LORAMAC_DEFAULT_DATARATE                    DR_11   // SINGLE sf9    1024us
//#define LORAMAC_DEFAULT_DATARATE                    DR_12   // SINGLE sf8     512us
#define LORAMAC_DEFAULT_DATARATE                    DR_13   // SINGLE sf7     256us

/*!
 * Default Tx output power used by the node
 */
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_20_DBM

/*!
 * Minimal (slowest) datarate that can be used by the node
 */
#define LORAMAC_MIN_DATARATE                     DR_8

/*!
 * Maximal (fastest) datarate that can be used by the node
 */
#define LORAMAC_MAX_DATARATE                     DR_13

    /* end us915 */
#elif defined (USE_BAND_433)
    #define TX_POWER_10_DBM    0
    #define TX_POWER_07_DBM    1
    #define TX_POWER_04_DBM    2
    #define TX_POWER_01_DBM    3
    #define TX_POWER_M2_DBM    4
    #define TX_POWER_M5_DBM    5

    #define LORAMAC_DEFAULT_TX_POWER            TX_POWER_10_DBM

    #define LORAMAC_MIN_TX_POWER                TX_POWER_M5_DBM
    #define LORAMAC_MAX_TX_POWER                TX_POWER_10_DBM

    #define DR_0                             0  // SF12 - BW125
    #define DR_1                             1  // SF11 - BW125
    #define DR_2                             2  // SF10 - BW125
    #define DR_3                             3  // SF9  - BW125
    #define DR_4                             4  // SF8  - BW125
    #define DR_5                             5  // SF7  - BW125
    #define DR_6                             6  // SF7  - BW250
    #define DR_7                             7  // FSK

    #define LORAMAC_MIN_DATARATE             DR_0   // slowest
    #define LORAMAC_MAX_DATARATE             DR_5   // fastest

    //#define LORAMAC_DEFAULT_DATARATE                    DR_0   //  sf12, 32768us
    //#define LORAMAC_DEFAULT_DATARATE                    DR_1   //  sf11, 16384us
    //#define LORAMAC_DEFAULT_DATARATE                    DR_2   //  sf10, 8192us
    //#define LORAMAC_DEFAULT_DATARATE                    DR_3   //  sf9, 4096us
    //#define LORAMAC_DEFAULT_DATARATE                    DR_4   //  sf8, 2048us
    #define LORAMAC_DEFAULT_DATARATE                    DR_5   //  sf7, 1024us

    /* end USE_BAND_433 */
#else
    #error "Please define a frequency band in the compiler options."
#endif

#endif // __LORAMAC_BOARD_H__
