/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project
*/
#include "mbed.h"
#include "board.h"

#include "LoRaMacSingle.h"
#include "Commissioning.h"
#include "commands.h"

#ifdef ENABLE_LUMINOSITY
    #include "TSL2561.h"
    //TSL2561 tsl2561(TSL2561_ADDR_FLOAT); // https://developer.mbed.org/components/Grove-Digital-Light-Sensor/ 
    TSL2561 tsl2561(TSL2561_ADDR_LOW);
#endif

#ifdef ENABLE_RGB
    #include "ChainableLED.h"
    ChainableLED rgb(D6, D7, 1); // https://developer.mbed.org/components/Grove-Chainable-RGB-LED/
#endif

#ifdef TYPE_ABZ
    InterruptIn user_button(PB_2);
#else
    InterruptIn user_button(USER_BUTTON);
#endif

#ifdef SENSORS
    #ifdef TYPE_ABZ /* DISCO_L072CZ_LRWAN1 */
        PwmOut pwm(PB_5);
        DigitalIn pc8_in(PB_13);
        DigitalOut pc6_out(PB_14);   
        AnalogIn a0(PA_0);
        AnalogIn a2(PA_4);
    #else
        PwmOut pwm(PB_11);
        DigitalIn pc8_in(PC_8);
        DigitalOut pc6_out(PC_6);   
        AnalogIn a1(A1); // https://developer.mbed.org/teams/Seeed/wiki/Potentiometer
        AnalogIn a3(A3); // https://developer.mbed.org/teams/Seeed/wiki/Potentiometer
    #endif

    #define N_SAMPLES       4
    struct sens {
        uint16_t seq;
        uint16_t ain_A, ain_B;
        uint8_t pins;
    } sensor[N_SAMPLES];
    uint8_t sensor_in_idx, sensor_out_idx;
    uint16_t sample_seqnum;
    bool sensor_overflow;
    #define POLL_RATE       1.0

    void sensor_poll()
    {
        sensor[sensor_in_idx].seq = sample_seqnum++;
#ifdef TYPE_ABZ
        sensor[sensor_in_idx].ain_A = a0.read_u16();
        sensor[sensor_in_idx].ain_B = a2.read_u16();
#else
        sensor[sensor_in_idx].ain_A = a1.read_u16();
        sensor[sensor_in_idx].ain_B = a3.read_u16();
#endif
        sensor[sensor_in_idx].pins = pc8_in.read();
        sensor[sensor_in_idx].pins <<= 1;
        sensor[sensor_in_idx].pins = pc6_out.read();
        if (++sensor_in_idx == N_SAMPLES)
            sensor_in_idx = 0;

        if (sensor_in_idx == sensor_out_idx)
            sensor_overflow = true;
    }
    LowPowerTicker sensor_ticker;
#endif /* SENSORS */

#if defined(TARGET_DISCO_L072CZ_LRWAN1) || ( defined(TARGET_NUCLEO_L073RZ) && defined(TYPE_ABZ) )
    DigitalOut jumper_out(PA_11);
    InterruptIn jumper_in(PA_12);
#else
    DigitalOut jumper_out(PC_10);
    InterruptIn jumper_in(PC_12);
#endif /* murata */

char pcbuf[128];
int pcbuf_len;

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            15

/*!
 * User application data buffer size
 */
#if ( LORAWAN_CONFIRMED_MSG_ON == 1 )
#define LORAWAN_APP_DATA_SIZE                       6

#else
#define LORAWAN_APP_DATA_SIZE                       1

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
#if defined(SENSORS)
    static uint8_t AppPort = SENSOR_PORT;
#else
    static uint8_t AppPort = LORAWAN_APP_PORT;
#endif

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;
static unsigned int uplink_length;   // user assigned

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           128

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
LowPowerTimeout TxNextPacketTimeout;

volatile bool send_at_beacon;
volatile bool awaiting_mcps_indic;
bool join_retry;
/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
    DEVICE_STATE_REJOIN,
    DEVICE_STATE_UPLINK
}DeviceState;

/*!
 * Strucure containing the Uplink status
 */
struct sLoRaMacUplinkStatus
{
    uint8_t Acked;
    //int8_t Datarate;
    uint16_t UplinkCounter;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacUplinkStatus;

/*!
 * Strucure containing the Downlink status
 */
struct sLoRaMacDownlinkStatus
{
    int16_t Rssi;
    int8_t Snr;
    uint16_t DownlinkCounter;
    bool RxData;
    uint8_t Port;
    uint8_t *Buffer;
    uint8_t BufferSize;
}LoRaMacDownlinkStatus;

static uint8_t missed_count;


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 15:
        {
            AppData[0] = AppLedStateOn;
            if( IsTxConfirmed == true )
            {
                AppData[1] = LoRaMacDownlinkStatus.DownlinkCounter >> 8;
                AppData[2] = LoRaMacDownlinkStatus.DownlinkCounter;
                AppData[3] = LoRaMacDownlinkStatus.Rssi >> 8;
                AppData[4] = LoRaMacDownlinkStatus.Rssi;
                AppData[5] = LoRaMacDownlinkStatus.Snr;
            }
        }
        break;
#if defined(SENSORS)
    case SENSOR_PORT:
        {
            uint16_t x = 0xffff;
#ifdef ENABLE_LUMINOSITY
            x = tsl2561.getLuminosity(TSL2561_VISIBLE);
#endif
            if (sensor_overflow) {
                app_printf("sensor_overflow\r\n");
                sensor_overflow = false;
            }
            AppDataSize = 0;
            AppData[AppDataSize++] = x >> 8;
            AppData[AppDataSize++] = x & 0xff;
            while (sensor_in_idx != sensor_out_idx) {
                AppData[AppDataSize++] = sensor[sensor_out_idx].seq >> 8;
                AppData[AppDataSize++] = sensor[sensor_out_idx].seq & 0xff;
                AppData[AppDataSize++] = sensor[sensor_out_idx].ain_A >> 8;
                AppData[AppDataSize++] = sensor[sensor_out_idx].ain_A & 0xff;
                AppData[AppDataSize++] = sensor[sensor_out_idx].ain_B >> 8;
                AppData[AppDataSize++] = sensor[sensor_out_idx].ain_B & 0xff;
                AppData[AppDataSize++] = sensor[sensor_out_idx].pins;
                if (++sensor_out_idx == N_SAMPLES)
                    sensor_out_idx = 0;
            }
        }
        break;
#endif /* SENSORS */
    default:
        break;
    }
}

void
LoRaMacEventInfoStatus_to_string(LoRaMacEventInfoStatus_t status, char* dst)
{
    const char* ptr = NULL;

    switch (status) {
        case LORAMAC_EVENT_INFO_STATUS_RX_ERROR: ptr = "RX_ERROR"; break;
        case LORAMAC_EVENT_INFO_STATUS_OK: ptr = "OK"; break;
        case LORAMAC_EVENT_INFO_STATUS_ERROR_RX_MTYPE: ptr = "ERROR_RX_MTYPE"; break;
        case LORAMAC_EVENT_INFO_STATUS_ERROR_SEND: ptr = "ERROR_SEND"; break;
        case LORAMAC_EVENT_INFO_STATUS_ERROR_JOIN_ACCEPT: ptr = "ERROR_JOIN_ACCEPT"; break;
        case LORAMAC_EVENT_INFO_STATUS_ERROR_MLMEREQ: ptr = "ERROR_MLMEREQ"; break;
        case LORAMAC_EVENT_INFO_STATUS_ERROR_MCPSREQ: ptr = "ERROR_MCPSREQ"; break;
        //case LORAMAC_EVENT_INFO_STATUS_ERROR: ptr = "ERROR"; break;
        
        case LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT: ptr = "TX_TIMEOUT"; break;
        case LORAMAC_EVENT_INFO_STATUS_RX_TIMEOUT: ptr = "RX_TIMEOUT"; break;
        case LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL: ptr = "JOIN_FAIL"; break;
        case LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED: ptr = "DOWNLINK_REPEATED"; break;
        case LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR: ptr = "TX_DR_PAYLOAD_SIZE_ERROR"; break;
        case LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS: ptr = "DOWNLINK_TOO_MANY_FRAMES_LOSS"; break;
        case LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL: ptr = "ADDRESS_FAIL"; break;
        case LORAMAC_EVENT_INFO_STATUS_MIC_FAIL: ptr = "MIC_FAIL"; break;
        case LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED: ptr = "BEACON_LOCKED"; break;
        case LORAMAC_EVENT_INFO_STATUS_BEACON_LOST: ptr = "BEACON_LOST"; break;
        case LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND: ptr = "BEACON_NOT_FOUND"; break;
        case LORAMAC_EVENT_INFO_STATUS_STOP: ptr = "STOP"; break;
    }

    if (ptr != NULL)
        strcpy(dst, ptr);
}

void
LoRaMacStatus_to_string(LoRaMacStatus_t status, char* dst)
{
    const char* ptr = NULL;

    switch (status) {
        case LORAMAC_STATUS_OK: ptr = "OK"; break;
        case LORAMAC_STATUS_BUSY: ptr = "BUSY"; break;
        case LORAMAC_STATUS_SERVICE_UNKNOWN: ptr = "SERVICE_UNKNOWN"; break;
        case LORAMAC_STATUS_PARAMETER_INVALID: ptr = "PARAMETER_INVALID"; break;
        case LORAMAC_STATUS_FREQUENCY_INVALID: ptr = "FREQUENCY_INVALID"; break;
        case LORAMAC_STATUS_DATARATE_INVALID: ptr = "DATARATE_INVALID"; break;
        case LORAMAC_STATUS_FREQ_AND_DR_INVALID: ptr = "FREQ_AND_DR_INVALID"; break;
        case LORAMAC_STATUS_NO_NETWORK_JOINED: ptr = "NO_NETWORK_JOINED"; break;
        case LORAMAC_STATUS_LENGTH_ERROR: ptr = "LENGTH_ERROR"; break;
        case LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR: ptr = "MAC_CMD_LENGTH_ERROR"; break;
        case LORAMAC_STATUS_DEVICE_OFF: ptr = "DEVICE_OFF"; break;
    }
    if (ptr != NULL)
        strcpy(dst, ptr);
}

static void OnTxNextPacketTimerEvent( void );

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame(uint8_t port)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    LoRaMacStatus_t status;
    char str[64];

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;

        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = 0;
        LoRaMacUplinkStatus.Buffer = NULL;
        LoRaMacUplinkStatus.BufferSize = 0;
    }
    else
    {
        LoRaMacUplinkStatus.Acked = false;
        LoRaMacUplinkStatus.Port = port;
        LoRaMacUplinkStatus.Buffer = AppData;
        LoRaMacUplinkStatus.BufferSize = AppDataSize;

        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = port;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = port;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
        }
    }

    status = LoRaMacMcpsRequest( &mcpsReq );
    if( status == LORAMAC_STATUS_OK )
    {
        awaiting_mcps_indic = true;
        app_printf("send ok\r\n");
        return false;
    }
    LoRaMacStatus_to_string(status, str);
    app_printf("send failed:%s\r\n", str);

    if (status == LORAMAC_STATUS_NO_NETWORK_JOINED) {
        join_retry = true;
#ifdef SENSORS
        sensor_ticker.detach();
#endif
    } else {
        AppPort = port;
        TxNextPacketTimeout.attach(&OnTxNextPacketTimerEvent, microseconds(randr(1000000, 5000000) + 1000000));
        if (status == LORAMAC_STATUS_BUSY)
            send_at_beacon = true;
    }

    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;
    
    if (DeviceState == DEVICE_STATE_REJOIN) {
        app_printf("unjoin\r\n");
        mibReq.Type = MIB_NETWORK_JOINED;
        mibReq.Param.IsNetworkJoined = false;
#ifdef SENSORS
        sensor_ticker.detach();
#endif
        if (LoRaMacMibSetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK) {
            TxNextPacketTimeout.attach(&OnTxNextPacketTimerEvent, 500ms);
            return;
        }
    }

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    app_printf("OnTxNextPacketTimerEvent() ");
    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            app_printf("send");
            DeviceState = DEVICE_STATE_UPLINK;
        }
        else
        {
            app_printf("join");
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
    app_printf("\r\n");
}

void
send_uplink()
{
    AppDataSize = uplink_length;
    DeviceState = DEVICE_STATE_UPLINK;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    static uint8_t fail_count = 0;
    char str[64];
    
    app_printf("McpsConfirm ");
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        fail_count = 0;
        app_printf("OK ");
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                app_printf("UNCONFIRMED");
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                app_printf("CONFIRMED");
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                LoRaMacUplinkStatus.Acked = mcpsConfirm->AckReceived;
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
        LoRaMacUplinkStatus.UplinkCounter = mcpsConfirm->UpLinkCounter;

    } else {
        LoRaMacEventInfoStatus_to_string(mcpsConfirm->Status, str);
        app_printf("%s ", str);

        /* mcpsIndication may not come. last uplink done, send another uplink */
        if (jumper_in.read()) {   /* jumper installed: auto uplink */
            TxNextPacketTimeout.attach(&send_uplink, 100ms);
        }

        if (++fail_count > 10) {
            /* cause re-join */
            MibRequestConfirm_t mibReq;
            mibReq.Type = MIB_NETWORK_JOINED;
            mibReq.Param.IsNetworkJoined = false;
            LoRaMacMibSetRequestConfirm( &mibReq );
            fail_count = 0;
#ifdef SENSORS
            sensor_ticker.detach();
#endif
        }
    }

    app_printf("\r\n");
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    char str[64];
    MibRequestConfirm_t mibReq;

    app_printf("McpsIndication ");

    /* last uplink done, send another uplink */
    if (jumper_in.read() && mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_STOP) {
        /* jumper installed: auto uplink */
        TxNextPacketTimeout.attach(&send_uplink, 100ms);
    }

    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        LoRaMacEventInfoStatus_to_string(mcpsIndication->Status, str);
        app_printf("%s\r\n", str);
        return;
    }

    awaiting_mcps_indic = false;

    switch( mcpsIndication->McpsIndication )
    {
        /* this refers to the downlink from gateway, not the uplink that was sent to it */
        case MCPS_UNCONFIRMED:
        {
            app_printf("UNCONFIRMED ");
            break;
        }
        case MCPS_CONFIRMED:
        {
            /* downlink has requested Ack */
            app_printf("CONFIRMED ");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            app_printf("MCPS_MULTICAST ");
            /*if (mcpsIndication->RxData) {
            }*/
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    LoRaMacDownlinkStatus.Rssi = mcpsIndication->Rssi;
    if( mcpsIndication->Snr & 0x80 ) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        LoRaMacDownlinkStatus.Snr = ( ( ~mcpsIndication->Snr + 1 ) & 0xFF ) >> 2;
        LoRaMacDownlinkStatus.Snr = -LoRaMacDownlinkStatus.Snr;
    }
    else
    {
        // Divide by 4
        LoRaMacDownlinkStatus.Snr = ( mcpsIndication->Snr & 0xFF ) >> 2;
    }
    LoRaMacDownlinkStatus.DownlinkCounter++;
    LoRaMacDownlinkStatus.RxData = mcpsIndication->RxData;
    LoRaMacDownlinkStatus.Port = mcpsIndication->Port;
    LoRaMacDownlinkStatus.Buffer = mcpsIndication->Buffer;
    LoRaMacDownlinkStatus.BufferSize = mcpsIndication->BufferSize;

    if( mcpsIndication->RxData == true )
    {
        int i;
        app_printf("RxData %u ", mcpsIndication->BufferSize);
        for (i = 0; i < mcpsIndication->BufferSize; i++) {
            app_printf("%02x ", mcpsIndication->Buffer[i]);
        }
        app_printf("\r\n");
        
        switch (mcpsIndication->Buffer[0]) {
            default:
            case CMD_NONE: break;
#if defined (ENABLE_RGB)
            case CMD_LED_RGB:
                rgb.setColorRGB(0, 
                    mcpsIndication->Buffer[1],  // R
                    mcpsIndication->Buffer[2],  // G
                    mcpsIndication->Buffer[3]   // B
                );
                app_printf("rgb %u %u %u\r\n",
                    mcpsIndication->Buffer[1],
                    mcpsIndication->Buffer[2],
                    mcpsIndication->Buffer[3]
                );
                break;
#endif /* ENABLE_RGB */
#if defined(SENSORS)
            case CMD_GPIO_OUT:
                pc6_out = mcpsIndication->Buffer[1];
                app_printf("gpo %d\r\n", mcpsIndication->Buffer[1]);
                break;
            case CMD_PWM:
                pwm.period(1.0 / mcpsIndication->Buffer[1]);
                pwm.write(mcpsIndication->Buffer[2] / 255.0);
                app_printf("pwm %u %u\r\n", mcpsIndication->Buffer[1], mcpsIndication->Buffer[2]);
                break;
#endif /* SENSORS */
            case CMD_TX_POWER:
                app_printf("txp %u\r\n", mcpsIndication->Buffer[1]);
                mibReq.Type = MIB_CHANNELS_TX_POWER;
                mibReq.Param.ChannelsTxPower = mcpsIndication->Buffer[1];
                LoRaMacMibSetRequestConfirm( &mibReq );        
                break;
        } // ..switch (mcpsIndication->Buffer[3])        

        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
            }
            break;
        default:
            break;
        }
    }
    
    if (mcpsIndication->Status == LORAMAC_EVENT_INFO_STATUS_STOP) {
        app_printf(" STOP-SCC");
        TxNextPacketTimeout.detach();
        DeviceState = DEVICE_STATE_SLEEP;
    }
    
    app_printf("\r\n");
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    char str[64];
    LoRaMacEventInfoStatus_to_string(mlmeConfirm->Status, str);
    app_printf("MlmeConfirm %s ", str);

    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            app_printf("MLME_JOIN ");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SLEEP;
#ifdef SENSORS
                sensor_ticker.attach(&sensor_poll, POLL_RATE);
                sensor_in_idx = 0;
                sensor_out_idx = 0;
                sample_seqnum = 0;
#endif
                missed_count = 0;
                if (jumper_in.read())  /* jumper installed: auto uplink */
                    TxNextPacketTimeout.attach(&send_uplink, 100ms);            
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            app_printf("LINK_CHECK");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                //mlmeConfirm->DemodMargin;
                // Check NbGateways
                //mlmeConfirm->NbGateways;
            }
            break;
        }
        default:
            break;
    }

    app_printf("\r\n");
}


static void MlmeIndication( MlmeIndication_t *MlmeIndication )
{
    char str[64];

    app_printf("MlmeIndication ");
    switch( MlmeIndication->MlmeIndication )
    {
        case MLME_BEACON:
        {
            app_printf("BEACON ", missed_count);
            if (missed_count)
                app_printf("missed:%u ", missed_count);

            LoRaMacEventInfoStatus_to_string(MlmeIndication->Status, str);
            app_printf("%s ", str);

            if (send_at_beacon) {
                TxNextPacketTimeout.attach(&send_uplink, 100ms);
                send_at_beacon = false;
            }

            if (LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED == MlmeIndication->Status) {
                missed_count = 0;
#if defined(ENABLE_RGB)
                rgb.setColorRGB(0, 0, 0, 0);    // off
#endif /* ENABLE_RGB */
            } else {
#if defined(ENABLE_RGB)
                rgb.setColorRGB(0, 255, 64, 0);    // orange
#endif /* ENABLE_RGB */
                if (++missed_count > 4) {
                    /* cause re-join */
                    LoRaMacStatus_t status;
                    MibRequestConfirm_t mibReq;
#ifdef SENSORS
                    sensor_ticker.detach();
#endif
                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = false;
                    status = LoRaMacMibSetRequestConfirm(&mibReq);
                    if (status != LORAMAC_STATUS_OK)
                        DeviceState = DEVICE_STATE_REJOIN;
    
                    LoRaMacStatus_to_string(status, str);
                    app_printf("app-rejoin %s\r\n", str);
                    TxNextPacketTimeout.attach(&OnTxNextPacketTimerEvent, microseconds(randr(1000000, 5000000) + 1000000));
                }
            }
            break;

        }
        case MLME_TXDONE:
            app_printf("MLME_TXDONE ");
            break;
        default:
            app_printf("<%d> ", MlmeIndication->MlmeIndication);
            break;
    }

    app_printf("\r\n");
}

void
send_pcbuf_uplink()
{
    bool ret;
    memcpy(AppData, pcbuf, pcbuf_len);
    AppDataSize = pcbuf_len;

    ret = SendFrame(TEXT_PORT);
    app_printf("%d = SendFrame()\r\n", ret);
}


void cmd_status(uint8_t idx)
{
    MibRequestConfirm_t mibReq;
    
    app_printf("DevEUI ");
    for (int i = 0; i < 8; i++)
        app_printf("%02x ", DevEui[i]);
    mibReq.Type = MIB_DEV_ADDR;
    LoRaMacMibGetRequestConfirm( &mibReq );         
    app_printf(", DevAddr:%lx\r\n", mibReq.Param.DevAddr);
    
#if defined(SENSORS)
    #ifdef TYPE_ABZ
        app_printf("a0:%u a2:%u\r\n", a0.read_u16(), a2.read_u16());
    #else
        app_printf("a1:%u a3:%u\r\n", a1.read_u16(), a3.read_u16());
    #endif
    app_printf("button:%d ", user_button.read());
#endif
    app_printf("DEVICE_STATE_");
    switch (DeviceState) {
        case DEVICE_STATE_JOIN: printf("JOIN"); break;
        case DEVICE_STATE_CYCLE: printf("CYCLE"); break;
        case DEVICE_STATE_SLEEP: printf("SLEEP"); break;
        case DEVICE_STATE_REJOIN: printf("REJOIN"); break;
        case DEVICE_STATE_INIT: printf("INIT"); break;
        case DEVICE_STATE_UPLINK: printf("UPLINK"); break;
    }
    app_printf(" send_at_beacon:%d\r\n", send_at_beacon);
    app_printf("awaiting_mcps_indic:%d\r\n", awaiting_mcps_indic);
    
    loramac_print_status();

}

void cmd_uplink_length(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%u", &uplink_length);
    }
    app_printf("uplink_length:%u\r\n", uplink_length);
}

#if defined(ENABLE_RGB)
void cmd_rgb(uint8_t idx)
{
    int r, g, b;
    sscanf(pcbuf+idx, "%d %d %d", &r, &g, &b);
    rgb.setColorRGB(0, r, g, b);
    app_printf("\r\nrgb: %d %d %d\r\n", r, g, b);
}
#endif /* ENABLE_RGB */

#if defined(SENSORS)
void cmd_pwm(uint8_t idx)
{
    float period, duty;
    unsigned p, d;

    if (sscanf(pcbuf+idx, "%u %u", &p, &d) == 2) {
        period = 1.0 / p;
        duty = d / 255.0;
        pwm.period(period);
        pwm.write(duty);
        printf("pwm period:%f, duty:%f\r\n", period, duty);
    } else
        printf("pwm parse fail\r\n");
}

#endif /* SENSORS */

void cmd_ChannelsTxPower(uint8_t idx)
{
    MibRequestConfirm_t mibReq;    
    unsigned int i;
    
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%u", &i);
        mibReq.Type = MIB_CHANNELS_TX_POWER;
        mibReq.Param.ChannelsTxPower = i;
        LoRaMacMibSetRequestConfirm( &mibReq );        
    }
    
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    LoRaMacMibGetRequestConfirm( &mibReq );
    app_printf("ChannelsTxPower:%u\r\n", mibReq.Param.ChannelsTxPower);
}

typedef struct {
    const char* const cmd;
    void (*handler)(uint8_t args_at);
    const char* const arg_descr;
    const char* const description;
} menu_item_t;

void cmd_help(uint8_t args_at);

const menu_item_t menu_items[] = 
{   /* after first character, command names must be [A-Za-z] */
    { "?", cmd_help, "","show available commands"},
    { ".", cmd_status, "","print status"}, 
    { "ul", cmd_uplink_length, "%u","set uplink payload length"}, 
    { "ctxp", cmd_ChannelsTxPower, "%u","get/set ChannelsTxPower"},
#if defined(ENABLE_RGB)
    { "rgb", cmd_rgb, "%d %d %d", "set led R G B"},
#endif
#if defined(SENSORS)
    { "p", cmd_pwm, "%u %u", "set pwm period, duty"},
#endif /* SENSORS */
    { NULL, NULL, NULL, NULL }
};

void cmd_help(uint8_t args_at)
{
    int i;
    
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        app_printf("%s%s\t%s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
    }
    
}

void
console()
{
    bool parsed;
    int i;
    uint8_t user_cmd_len;
    
    if (pcbuf_len < 0) {    // ctrl-C
        return;
    }
    if (pcbuf_len == 0)
        return;
        
    app_printf("\r\n");
        
    /* get end of user-entered command */
    user_cmd_len = 1;   // first character can be any character
    for (i = 1; i <= pcbuf_len; i++) {
        if (pcbuf[i] < 'A' || (pcbuf[i] > 'Z' && pcbuf[i] < 'a') || pcbuf[i] > 'z') {
            user_cmd_len = i;
            break;
        }
    }

    parsed = false;
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        int mi_len = strlen(menu_items[i].cmd);

        if (menu_items[i].handler && user_cmd_len == mi_len && (strncmp(pcbuf, menu_items[i].cmd, mi_len) == 0)) {
            while (pcbuf[mi_len] == ' ')   // skip past spaces
                mi_len++;
            menu_items[i].handler(mi_len);
            parsed = true;
            break;
        }
    }

    if (!parsed)
        send_pcbuf_uplink();
   
    pcbuf_len = 0;
    app_printf("> ");
    fflush(stdout); 
}

void rx_callback()
{
    static uint8_t pcbuf_idx = 0;
    static uint8_t prev_len = 0;;
    //char c = pc.getc();
    char c;
    pc.read(&c, 1);

    if (c == 8) {
        if (pcbuf_idx > 0) {
            uint8_t buf[3] = {8, ' ', 8};
            pc.write(buf, 3);
            pcbuf_idx--;
        }
    } else if (c == 3) {    // ctrl-C
        pcbuf_len = -1;
    } else if (c == '\r') {
        if (pcbuf_idx == 0) {
            pcbuf_len = prev_len;
        } else {
            pcbuf[pcbuf_idx] = 0;   // null terminate
            prev_len = pcbuf_idx;
            pcbuf_idx = 0;
            pcbuf_len = prev_len;
        }
    } else if (pcbuf_idx < sizeof(pcbuf)) {
        pcbuf[pcbuf_idx++] = c;
        pc.write(&c, 1);
    }
}

void button_isr()
{
    app_printf("button_isr\r\n");

    AppPort = SENSOR_PORT;
    send_uplink();
}

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    pc.baud(115200);
    pc.attach(&rx_callback);
    app_printf("\r\nreset %s\r\n", FW_VERS);
    
    BoardInit( );

    DeviceState = DEVICE_STATE_INIT;
    
    while (!user_button) {
        printf("button-lo\r\n");
        ThisThread::sleep_for(10ms);//wait(0.01);
    }
    user_button.fall(&button_isr);
    
#ifdef ENABLE_LUMINOSITY
    app_printf("TSL2561 Sensor ");
    if (tsl2561.begin()) {    
        app_printf("Found\r\n");        
    } else {    
        app_printf("not-found\r\n");   
    }
    
    // You can change the gain on the fly, to adapt to brighter/dimmer tsl2561 situations
    tsl2561.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
    //tsl2561.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
    
    // Changing the integration time gives you a longer time over which to sense tsl2561
    // longer timelines are slower, but are good in very low tsl2561 situtations!
    //tsl2561.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright tsl2561)
    //tsl2561.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium tsl2561)
    tsl2561.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim tsl2561)    
#endif /* ENABLE_LUMINOSITY */

    jumper_out = 1;
    jumper_in.mode(PullDown);

    while( 1 )
    {
        console();
        
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                app_printf("DEVICE_STATE_INIT\r\n");
#if defined(SENSORS)
                uplink_length = 7;
#else                
                uplink_length = 2;
#endif
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                if (LORAMAC_STATUS_OK != LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks )) {
                    app_printf("LoRaMacInitialization() failed\r\n");
                    for (;;) ;
                }
                app_printf("INIT-ok\r\n");

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                LoRaMacDownlinkStatus.DownlinkCounter = 0;

                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                if (jumper_in.read())   /* if jumper installed: auto uplink */
                    send_at_beacon = true;
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;
                // override software definition with hardware value
                BoardGetUniqueId(DevEui);
                app_printf("DevEUI ");
                for (int i = 0; i < 8; i++)
                    app_printf("%02x ", DevEui[i]);
                app_printf("\r\n");

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 255;

                LoRaMacStatus_t status = LoRaMacMlmeRequest( &mlmeReq );
                if (status != LORAMAC_STATUS_OK) {
                    char str[48];
                    LoRaMacStatus_to_string(status, str);
                    app_printf("join-req-failed:%s\r\n", str);
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif
                join_retry = false;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                {
                    milliseconds ms(TxDutyCycleTime * 1000);
                    TxNextPacketTimeout.attach(&OnTxNextPacketTimerEvent, ms);
                }
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                //bottom_half();
                LoRaMacBottomHalf();
                // Wake up through events
                sleep();
                break;
            }
            case DEVICE_STATE_UPLINK:
                app_printf("DEVICE_STATE_UPLINK\r\n");
                PrepareTxFrame(AppPort);
                SendFrame(AppPort);
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }

        LoRaMacBottomHalf();

        if (join_retry) {
            app_printf("join_retry\r\n");
            DeviceState = DEVICE_STATE_JOIN;
            join_retry = false;
        }

    } // ..while( 1 )
}
