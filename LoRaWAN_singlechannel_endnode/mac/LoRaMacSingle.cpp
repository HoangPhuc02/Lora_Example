/* */

#include <math.h>
#include "board.h"
#include "radio.h"

#include "LoRaMacCrypto.h"
#include "LoRaMacSingle.h"
#include "LoRaMacTest.h"

#define BEACONS_MISSED_LIMIT            16

#define PING_SLOT_RESOLUTION_us         30000

#define PREAMBLE_SYMBS                  8

#define TARGET_PRECESSION_us            3000
#define BEACON_RX_TIMEOUT_LOCKED_us     8000

/*!
 * Maximum PHY layer payload size
 */
#define LORAMAC_PHY_MAXPAYLOAD                      255

/*!
 * Maximum MAC commands buffer size
 */
#define LORA_MAC_COMMAND_MAX_LENGTH                 15

LowPowerClock::time_point txDoneAt;

/*!
 * Device IEEE EUI
 */
static uint8_t *LoRaMacDevEui;

/*!
 * Application IEEE EUI
 */
static uint8_t *LoRaMacAppEui;

/*!
 * AES encryption/decryption cipher application key
 */
static uint8_t *LoRaMacAppKey;

/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t LoRaMacNwkSKey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t LoRaMacAppSKey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*!
 * Device nonce is a random value extracted by issuing a sequence of RSSI
 * measurements
 */
static uint16_t LoRaMacDevNonce;

/*!
 * Network ID ( 3 bytes )
 */
static uint32_t LoRaMacNetID;

/*!
 * Mote Address
 */
static uint32_t LoRaMacDevAddr;

/*!
 * Multicast channels linked list
 */
static MulticastParams_t *MulticastChannels = NULL;

/*!
 * Indicates if the node is connected to a private or public network
 */
static bool PublicNetwork;

uint8_t tx_buf_len;

static uint8_t rxFRMPayload[244];

/*!
 * LoRaMAC frame counter. Each time a packet is sent the counter is incremented.
 * Only the 16 LSB bits are sent
 */
static uint32_t UpLinkCounter = 0;

/*!
 * LoRaMAC frame counter. Each time a packet is received the counter is incremented.
 * Only the 16 LSB bits are received
 */
static uint32_t DownLinkCounter = 0;

/*!
 * Used for test purposes. Disables the opening of the reception windows.
 */
static bool IsRxWindowsEnabled = true;

LowPowerTimeout rx_timeout;

/* how long this MCU (we're running on) takes to wake-up from deep-sleep */
microseconds mcu_wakeup_latency;

LowPowerClock::time_point rx_timeout_setAt;

/*!
 * Indicates if the MAC layer wants to send MAC commands
 */
static bool MacCommandsInNextTx = false;

/*!
 * Contains the current MacCommandsBuffer index
 */
static uint8_t MacCommandsBufferIndex = 0;

/*!
 * Contains the current MacCommandsBuffer index for MAC commands to repeat
 */
static uint8_t MacCommandsBufferToRepeatIndex = 0;

/*!
 * Buffer containing the MAC layer commands
 */
static uint8_t MacCommandsBuffer[LORA_MAC_COMMAND_MAX_LENGTH];

/*!
 * Buffer containing the MAC layer commands which must be repeated
 */
static uint8_t MacCommandsBufferToRepeat[LORA_MAC_COMMAND_MAX_LENGTH];

#define BEACON_SIZE                 6   /* bytes */
#define BEACON_CHANNEL_DR           LORAMAC_DEFAULT_DATARATE


#ifdef SX128x_H 
    //                             0    1   2  3  4  5  6  7
    const uint8_t Datarates[]  = { 12, 11, 10, 9, 8, 7, 6, 5 };
    #define SF_FROM_DR0     12  /* DR0 is sf12 */
    #define SF_FROM_DR1     11  /* DR1 is sf11 */
    #define SF_FROM_DR2     10  /* DR2 is sf10 */
    #define SF_FROM_DR3      9  /* DR3 is sf9 */
    #define SF_FROM_DR4      8  /* DR4 is sf8 */
    #define SF_FROM_DR5      7  /* DR5 is sf7 */
    #define SF_FROM_DR6      6  /* DR5 is sf6 */
    #define SF_FROM_DR7      5  /* DR7 is sf5 */


    const int8_t TxPowers[] = { 12, 5 };
    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)2486.9e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)300e3 )
    #define LORA_MAX_NB_CHANNELS            8

    #define LORA_BANDWIDTH_KHZ           200

    /* end sx1280 */
#elif defined( USE_BAND_915_SINGLE )
    /*!
     * Data rates table definition
                    DR:             0  1  2   3   4   5   6  7   8  9   10  11 12 13 14 15*/
    const uint8_t Datarates[]  = { 10, 9, 8,  7,  8,  0,  0, 0, 12, 11, 10, 9, 8, 7, 0, 0 };
    #define SF_FROM_DR8        12
    #define SF_FROM_DR9        11
    #define SF_FROM_DR10       10
    #define SF_FROM_DR11        9
    #define SF_FROM_DR12        8
    #define SF_FROM_DR13        7

    /*!
     * Tx output powers table definition
     */
                                // 0   1   2   3   4   5   6   7   8   9   10  11 12 13 14 15
    const int8_t TxPowers[]    = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0 };
    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)910.0e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)800e3 )
    #define LORA_MAX_NB_CHANNELS                        8

    #define LORA_BANDWIDTH_KHZ           500

    /* end us915 */
#elif defined(USE_BAND_433)
                       //   DR:    0   1   2   3  4  5  6  7
    const uint8_t Datarates[]  = { 12, 11, 10, 9, 8, 7, 7, 50 };
    #define SF_FROM_DR0        12
    #define SF_FROM_DR1        11
    #define SF_FROM_DR2        10
    #define SF_FROM_DR3         9
    #define SF_FROM_DR4         8
    #define SF_FROM_DR5         7

    const int8_t TxPowers[] = { 10, 7, 4, 1, -2, -5 };
    #define LORA_BANDWIDTH_KHZ           125

    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)433.32e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)200e3 )
    #define LORA_MAX_NB_CHANNELS            7

    /* end USE_BAND_433 */
#else
    #error "Please define a frequency band in the compiler options."
#endif

#define _SF_FROM_DR(dr)         SF_FROM_DR ## dr
#define SF_FROM_DR_(x)          _SF_FROM_DR(x)

#ifdef SX128x_H 

    #define FASTEST_SF      5
    /* ratio of symbol period to time from end of packet to RxDone interrupt */
//  sp       200KHz                         160    320   640   1280  2560  5120  10240 20480
// usLatency 200KHz implicit-6byte           90    228   436   880   2020  4200  8800  19300
                                    // SF:   5     6     7     8     9     10    11    12
    const float rxLatencyFactorFromSF[8] = { 0.56, 0.72, 0.68, 0.69, 0.79, 0.82, 0.86, 0.94 };

#elif defined(SX126x_H) || defined(SX127x_H)

    #define FASTEST_SF      5
    /* ratio of symbol period to time from end of packet to RxDone interrupt */
//  sp       500KHz                          64    128   256   512   1024  2048  4096  8192
// usLatency 500KHz implicit-6byte                 92    184   376   780   1680  3680  7720
                                    // SF:   5     6     7     8     9     10    11    12
    const float rxLatencyFactorFromSF[8] = { 0.72, 0.72, 0.72, 0.73, 0.76, 0.82, 0.90, 0.94 };

#endif /* SX126x_H || SX127x_H */

static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS];  // populated in init

#define BEACON_GUARD_us         2000000     // pre-beacon start
#define BEACON_RESERVED_us      2120000     // post-beacon start

#define MIN_SYMBOL_TIMEOUT       8   // number of symbols to keep receiver open for

/*!
 * LoRaMac parameters
 */
LoRaMacParams_t LoRaMacParams;

/*!
 * LoRaMac default parameters
 */
LoRaMacParams_t LoRaMacParamsDefaults;

/*!
 * Uplink messages repetitions counter
 */
static uint8_t ChannelsNbRepCounter = 0;

/*!
 * Current channel index
 */
static uint8_t Channel;

/*!
 * LoRaMac upper layer event functions
 */
static LoRaMacPrimitives_t *LoRaMacPrimitives;

/*!
 * LoRaMac upper layer callback functions
 */
static LoRaMacCallback_t *LoRaMacCallbacks;

/*!
 * LoRaMac duty cycle delayed Tx timer
 */
LowPowerTimeout tx_timeout;

/*!
 * LoRaMac reception windows delay
 * \remark normal frame: RxWindowXDelay = ReceiveDelayX - RADIO_WAKEUP_TIME
 *         join frame  : RxWindowXDelay = JoinAcceptDelayX - RADIO_WAKEUP_TIME
 */
microseconds RxWindowDelay_us;

typedef enum {
    BEACON_STATE_NONE = 0,
    BEACON_STATE_FIRST_ACQ,
    BEACON_STATE_ACQ_ERROR,
    BEACON_STATE_LOCKED,
} beacon_state_e;

static struct beacon_struct {
    int rx_precession_us; // positive: rxing before tx start, negative: rxing after tx start
    LowPowerClock::time_point rx_setup_at;
    LowPowerClock::time_point LastBeaconRxAt;   // was LastBeaconRx_us, updated only at beacon reception
    LowPowerClock::time_point sendAt;
    microseconds lastSendAtErr;
    int last_BeaconRxTimerError_us;
    int known_working_BeaconRxTimerError_us;

    unsigned symbol_period_us;

    uint8_t Precess_symbols;    // how many symbols we want to start receiver before expected transmitter
    uint16_t nSymbsTimeout;
    unsigned SymbolTimeout_us;
    uint8_t num_missed;
    uint8_t num_consecutive_ok;

    beacon_state_e state;

    uint16_t tx_slot_offset;
    uint16_t periodicity_slots;
    uint32_t beaconStartToRxDone;
    uint16_t sendOpportunities;

    LowPowerTimeout timeout_rx;
    LowPowerTimeout timeout_guard;
    bool guard;
} BeaconCtx;

/*!
 * Rx window parameters
 */
typedef struct
{
    int8_t Datarate;
    uint32_t RxWindowTimeout;
} RxConfigParams_t;

/*!
 * Rx windows params
 */
static RxConfigParams_t RxWindowsParam;

/*!
 * Acknowledge timeout timer. Used for packet retransmissions.
 */
static LowPowerTimeout AckTimeoutTimer;

/*!
 * Number of trials for the Join Request
 */
static uint8_t JoinRequestTrials;

/*!
 * Maximum number of trials for the Join Request
 */
static uint8_t MaxJoinRequestTrials;

/*!
 * Structure to hold an MCPS indication data.
 */
static McpsIndication_t McpsIndication;

/*!
 * Structure to hold MCPS confirm data.
 */
static McpsConfirm_t McpsConfirm;

/*!
 * Structure to hold MLME confirm data.
 */
static MlmeConfirm_t MlmeConfirm;

/*!
 * Structure to hold MLME indication data.
 */
static MlmeIndication_t MlmeIndication;

/*!
 * LoRaMac tx/rx operation state
 */
volatile LoRaMacFlags_t LoRaMacFlags;

/*!
 * \brief This function prepares the MAC to abort the execution of function
 *        OnRadioRxDone in case of a reception error.
 */
static void PrepareRxDoneAbort( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
static void OnRadioTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx error event
 */
static void OnRadioRxError( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
static void OnRadioRxTimeout( void );

/*!
 * \brief Function executed on AckTimeout timer event
 */
static void OnAckTimeoutTimerEvent( void );

/*!
 * \brief Adds a new MAC command to be sent.
 *
 * \Remark MAC layer internal function
 *
 * \param [in] cmd MAC command to be added
 *                 [MOTE_MAC_LINK_CHECK_REQ,
 *                  MOTE_MAC_LINK_ADR_ANS,
 *                  MOTE_MAC_DUTY_CYCLE_ANS,
 *                  MOTE_MAC_RX2_PARAM_SET_ANS,
 *                  MOTE_MAC_DEV_STATUS_ANS
 *                  MOTE_MAC_NEW_CHANNEL_ANS]
 * \param [in] p1  1st parameter ( optional depends on the command )
 * \param [in] p2  2nd parameter ( optional depends on the command )
 *
 * \retval status  Function status [0: OK, 1: Unknown command, 2: Buffer full]
 */
static LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 );

/*!
 * \brief Parses the MAC commands which must be repeated.
 *
 * \Remark MAC layer internal function
 *
 * \param [IN] cmdBufIn  Buffer which stores the MAC commands to send
 * \param [IN] length  Length of the input buffer to parse
 * \param [OUT] cmdBufOut  Buffer which stores the MAC commands which must be
 *                         repeated.
 *
 * \retval Size of the MAC commands to repeat.
 */
static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut );

/*!
 * \brief Verifies, if a value is in a given range.
 *
 * \param value Value to verify, if it is in range
 *
 * \param min Minimum possible value
 *
 * \param max Maximum possible value
 *
 * \retval Returns the maximum valid tx power
 */
static bool ValueInRange( int8_t value, int8_t min, int8_t max );


/*!
 * \brief Decodes MAC commands in the fOpts field and in the payload
 */
static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr );

/*!
 * \brief LoRaMAC layer generic send frame
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*!
 * \brief LoRaMAC layer frame buffer initialization
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fCtrl       MAC frame control field
 * \param [IN] fOpts       MAC commands buffer
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*!
 * \brief Sets the radio in continuous transmission mode
 *
 * \remark Uses the radio parameters set on the previous transmission.
 *
 * \param [IN] timeout     Time in seconds while the radio is kept in continuous wave mode
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SetTxContinuousWave( uint16_t timeout );

/*!
 * \brief Sets the radio in continuous transmission mode
 *
 * \remark Uses the radio parameters set on the previous transmission.
 *
 * \param [IN] timeout     Time in seconds while the radio is kept in continuous wave mode
 * \param [IN] frequency   RF frequency to be set.
 * \param [IN] power       RF ouptput power to be set.
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SetTxContinuousWave1( uint16_t timeout, uint32_t frequency, uint8_t power );

#ifdef SX127x_H
void printLoraIrqs(bool clear)
{
    pc_printf("\r\nIrqFlags:");
    if (Radio::lora.RegIrqFlags.bits.CadDetected)
        pc_printf("CadDetected ");
    if (Radio::lora.RegIrqFlags.bits.FhssChangeChannel) {
        pc_printf("FhssChangeChannel:%d ", Radio::lora.RegHopChannel.bits.FhssPresentChannel);
    }
    if (Radio::lora.RegIrqFlags.bits.CadDone)
        pc_printf("CadDone ");
    if (Radio::lora.RegIrqFlags.bits.TxDone)
        pc_printf("TxDone-dio0:%d ", Radio::radio.dio0.read());
    if (Radio::lora.RegIrqFlags.bits.ValidHeader)
        pc_printf("[42mValidHeader[0m ");
    if (Radio::lora.RegIrqFlags.bits.PayloadCrcError)
        pc_printf("[41mPayloadCrcError[0m ");
    if (Radio::lora.RegIrqFlags.bits.RxDone)
        pc_printf("[42mRxDone[0m ");  
    if (Radio::lora.RegIrqFlags.bits.RxTimeout)
        pc_printf("RxTimeout ");

    pc_printf("\r\n");

    if (clear)
        Radio::radio.write_reg(REG_LR_IRQFLAGS, Radio::lora.RegIrqFlags.octet);
}

void printOpMode()
{
    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    switch (Radio::radio.RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: pc_printf("[7msleep[0m"); break;
        case RF_OPMODE_STANDBY: pc_printf("[7mstby[0m"); break;
        case RF_OPMODE_SYNTHESIZER_TX: pc_printf("[33mfstx[0m"); break;
        case RF_OPMODE_TRANSMITTER: pc_printf("[31mtx[0m"); break;
        case RF_OPMODE_SYNTHESIZER_RX: pc_printf("[33mfsrx[0m"); break;
        case RF_OPMODE_RECEIVER: pc_printf("[32mrx[0m"); break;
        case 6:
            if (Radio::radio.RegOpMode.bits.LongRangeMode)
                pc_printf("[42mrxs[0m");
            else
                pc_printf("-6-");
            break;  // todo: different lora/fsk
        case 7:
            if (Radio::radio.RegOpMode.bits.LongRangeMode)
                pc_printf("[45mcad[0m");
            else
                pc_printf("-7-");
            break;  // todo: different lora/fsk
    }
}
#endif /* SX127x_H */

/*!
 * \brief Resets MAC specific parameters to default
 */
static void ResetMacParameters( void );

void
loramac_print_status()
{
    int until_beacon = BeaconCtx.rx_setup_at.time_since_epoch().count() - LowPowerClock::now().time_since_epoch().count();
    mac_printf("until_beacon:%d ", until_beacon);
    mac_printf("DR%u=sf%u guard:%d\r\n",
        LoRaMacParams.ChannelsDatarate_fixed,
        Datarates[LoRaMacParams.ChannelsDatarate_fixed],
        BeaconCtx.guard
    );
#ifdef SX128x_H 
    status_t status;
    Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
    switch (status.bits.cmdStatus) {
        case 1: pc_printf("success"); break;
        case 2: pc_printf("dataAvail"); break;
        case 3: pc_printf("cmdTimeout"); break;
        case 4: pc_printf("cmdErr"); break;
        case 5: pc_printf("exeFail"); break;
        case 6: pc_printf("txdone"); break;
        default: pc_printf("cmdStatus:<%u>", status.bits.cmdStatus); break;
    }
    pc_printf(" ");
    switch (status.bits.chipMode) {
        case 2: pc_printf("stdby_rc"); break;
        case 3: pc_printf("stdby_xosc"); break;
        case 4: pc_printf("fs"); break;
        case 5: pc_printf("\e[32mrx\e[0m"); break;
        case 6: pc_printf("\e[31mtx\e[0m"); break;
        default: pc_printf("chipMode:<%u>", status.bits.chipMode); break;
    }
    LoRaPktPar0_t LoRaPktPar0; 
    LoRaPktPar0.octet = Radio::radio.readReg(REG_ADDR_LORA_PKTPAR0, 1);
    pc_printf(" bw:%u sf%u ", LoRaPktPar0.bits.modem_bw, LoRaPktPar0.bits.modem_sf);
    mac_printf("loraSync:%04x\r\n", Radio::radio.readReg(REG_ADDR_LORA_SYNC, 2));
#elif defined(SX127x_H)
    Radio::radio.RegPaConfig.octet = Radio::radio.read_reg(REG_PACONFIG);
    if (Radio::radio.RegPaConfig.bits.PaSelect)
        pc_printf("PA_BOOST ");
    else
        pc_printf("RFO ");

    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    pc_printf("%.3fMHz sf%ubw%u ", Radio::radio.get_frf_MHz(), Radio::lora.getSf(), Radio::lora.getBw());
    pc_printf("dio0pin:%u ", Radio::radio.dio0.read());
    printOpMode();
    if (!Radio::radio.RegOpMode.bits.LongRangeMode) {
        pc_printf("FSK\r\n");
        return;
    }

    Radio::lora.RegIrqFlags.octet = Radio::radio.read_reg(REG_LR_IRQFLAGS);
    printLoraIrqs(false);

    Radio::lora.RegTest33.octet = Radio::radio.read_reg(REG_LR_TEST33);     // invert_i_q
    Radio::lora.RegDriftInvert.octet = Radio::radio.read_reg(REG_LR_DRIFT_INVERT);
    pc_printf("modemstat:%02x, rxinv:%x,%x\r\n", Radio::radio.read_reg(REG_LR_MODEMSTAT), Radio::lora.RegTest33.octet, Radio::lora.RegDriftInvert.octet);
    Radio::radio.RegDioMapping1.octet = Radio::radio.read_reg(REG_DIOMAPPING1);
    pc_printf("\r\ndio0map:%u\r\n", Radio::radio.RegDioMapping1.bits.Dio0Mapping);
    pc_printf("FIfoAddrPtr:%02x RxBase:%02x\r\n", Radio::radio.read_reg(REG_LR_FIFOADDRPTR), Radio::radio.read_reg(REG_LR_FIFORXBASEADDR));
#elif defined(SX126x_H)
    status_t status;
    Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
    switch (status.bits.chipMode) {
        case 2: mac_printf("STBY_RC"); break;
        case 3: mac_printf("STBY_XOSC"); break;
        case 4: mac_printf("FS"); break;
        case 5: mac_printf("RX"); break;
        case 6: mac_printf("TX"); break;
        default: mac_printf("%u", status.bits.chipMode); break;
    }
    pc_printf(" ");
    switch (status.bits.cmdStatus) {
        case 1: mac_printf("rfu"); break;
        case 2: mac_printf("dataAvail"); break;
        case 3: mac_printf("timeout"); break;
        case 4: mac_printf("err"); break;
        case 5: mac_printf("fail"); break;
        case 6: mac_printf("txdone"); break;
        default: mac_printf("%u", status.bits.cmdStatus); break;
    }
    loraConfig0_t conf0;
    conf0.octet = Radio::radio.readReg(REG_ADDR_LORA_CONFIG0, 1);
    // bw7=125 bw8=250 b9=500
    mac_printf(" bw:%u sf%u\r\n", conf0.bits.modem_bw, conf0.bits.modem_sf);
    loraConfig1_t conf1;
    conf1.octet = Radio::radio.readReg(REG_ADDR_LORA_CONFIG1, 1);
    mac_printf("inviq:%u cr%u\r\n", conf1.bits.rx_invert_iq, conf1.bits.tx_coding_rate);
    mac_printf("loraSync:%04x\r\n", Radio::radio.readReg(REG_ADDR_LORA_SYNC, 2));
#endif /* ..SX126x_H */
}

static void RxWindowSetup( uint32_t freq, int8_t datarate, uint16_t timeout, bool rxContinuous )
{
    uint8_t downlinkDatarate = Datarates[datarate];

    Radio::SetChannel( freq );

    // Store downlink datarate
    McpsIndication.RxDatarate = ( uint8_t ) datarate;

    Radio::LoRaModemConfig(LORA_BANDWIDTH_KHZ, downlinkDatarate, 1);

                        //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, false, false, true);
    Radio::SetLoRaSymbolTimeout(timeout);
    Radio::SetRxMaxPayloadLength(LORAMAC_PHY_MAXPAYLOAD);

} // ..RxWindowSetup()

static void OnRxWindowTimerEvent( void )
{
    RxWindowSetup(Channels[Channel].Frequency, RxWindowsParam.Datarate, RxWindowsParam.RxWindowTimeout, false);
    Radio::Rx( LoRaMacParams.MaxRxWindow );
}

static RxConfigParams_t ComputeRxWindowParameters( int8_t datarate, uint32_t rxError );

LowPowerClock::time_point rxto_irqAt;

static void OnRadioTxDone_topHalf()
{
    rxto_irqAt = LowPowerClock::now();
}

static void OnRadioTxDone_bh()
{
    // Setup timers
    if (IsRxWindowsEnabled)
    {
        rx_timeout_setAt = rxto_irqAt + RxWindowDelay_us;
        rx_timeout.attach_absolute(&OnRxWindowTimerEvent, rx_timeout_setAt);
        if (LoRaMacFlags.Bits.NodeAckRequested)
        {
            microseconds us(ACK_TIMEOUT_us + randr(-ACK_TIMEOUT_RND_us, ACK_TIMEOUT_RND_us));
            us += RxWindowDelay_us;
            AckTimeoutTimer.attach(&OnAckTimeoutTimerEvent, us - mcu_wakeup_latency);
        }
    }
    else
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX_TIMEOUT;

        if( LoRaMacFlags.Value == 0 )
        {
            LoRaMacFlags.Bits.McpsReq = 1;
        }
        LoRaMacFlags.Bits.MacDone = 1;
    }

    if (!LoRaMacFlags.Bits.NodeAckRequested)
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
        ChannelsNbRepCounter++;
    }

    MlmeIndication.MlmeIndication = MLME_TXDONE;
    MlmeIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
    LoRaMacPrimitives->MacMlmeIndication( &MlmeIndication );

    Radio::Standby( );

    txDoneAt = Radio::irqAt /*+ lpt_offset*/;
}


static void application_callbacks()
{
    if (LoRaMacFlags.Bits.McpsInd) {
        LoRaMacFlags.Bits.McpsInd = 0;
        LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );
    }

    if (LoRaMacFlags.Bits.McpsReq) {
        LoRaMacPrimitives->MacMcpsConfirm( &McpsConfirm );
        LoRaMacFlags.Bits.McpsReq = 0;
    }
}

static void PrepareRxDoneAbort( void )
{
    if (LoRaMacFlags.Bits.NodeAckRequested)
    {
        OnAckTimeoutTimerEvent( );
    }

    LoRaMacFlags.Bits.McpsInd = 1;
    LoRaMacFlags.Bits.MacDone = 1;

    application_callbacks();
}

void send_bh()
{
    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_SEND;
    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_SEND;
    McpsConfirm.TxPower = LoRaMacParams.ChannelsTxPower;
    McpsConfirm.UpLinkFrequency = Channels[Channel].Frequency;

    if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined)    // joining is channel hunting
        Radio::SetChannel( Channels[Channel].Frequency );

    if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined)
    {
        JoinRequestTrials++;
        mac_printf("join %luhz try%u DR%u\r\n", Channels[Channel].Frequency, JoinRequestTrials, LoRaMacParams.ChannelsDatarate_fixed);
    }

    Radio::set_tx_dbm(TxPowers[LoRaMacParams.ChannelsTxPower]);
    Radio::LoRaModemConfig(LORA_BANDWIDTH_KHZ, Datarates[LoRaMacParams.ChannelsDatarate_fixed], 1);

                        //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, false, true, false);
    Radio::Send(tx_buf_len, 0, 0, 0);

    LoRaMacFlags.Bits.uplink_pending = 0;   // sent

    // Compute Rx1 windows parameters, taken at TxDone
    if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined) {
        microseconds us(LoRaMacParams.JoinAcceptDelay_us - TARGET_PRECESSION_us);
        RxWindowDelay_us = us - mcu_wakeup_latency;
    } else {
        microseconds us(LoRaMacParams.ReceiveDelay_us - TARGET_PRECESSION_us);
        RxWindowDelay_us = us - mcu_wakeup_latency;
    }

} // ..send_bh()

void send_callback()
{
    LowPowerClock::time_point now = BeaconCtx.sendAt + BeaconCtx.lastSendAtErr;

    if (BeaconCtx.guard) {
        /* last send, this will be restarted after beacon sent */
        return;
    }
    BeaconCtx.sendOpportunities++;

    microseconds ping_slot_us(BeaconCtx.periodicity_slots * PING_SLOT_RESOLUTION_us);
    BeaconCtx.sendAt += ping_slot_us;
    if (BeaconCtx.state == BEACON_STATE_LOCKED) {
        float sinceBeacon = now.time_since_epoch().count() - BeaconCtx.LastBeaconRxAt.time_since_epoch().count();
        float ratio = sinceBeacon / BEACON_INTERVAL_us;
        microseconds duration_err_us((int)(BeaconCtx.last_BeaconRxTimerError_us * ratio));
        tx_timeout.attach_absolute(&send_callback, BeaconCtx.sendAt + duration_err_us - mcu_wakeup_latency);
        BeaconCtx.lastSendAtErr = duration_err_us;
    } else {
        microseconds duration_err_us(0);
        tx_timeout.attach_absolute(&send_callback, BeaconCtx.sendAt - mcu_wakeup_latency);
        BeaconCtx.lastSendAtErr = duration_err_us;
    }

    if (LoRaMacFlags.Bits.uplink_pending) {
        us_timestamp_t untilGuard = (BeaconCtx.rx_setup_at.time_since_epoch().count() - BEACON_GUARD_us) - LowPowerClock::now().time_since_epoch().count();
        if (untilGuard > (RECEIVE_DELAY_us + TARGET_PRECESSION_us))
            LoRaMacFlags.Bits.send = 1;
    }
}

void OnRxBeaconSetup()
{
    BeaconCtx.guard = false;
    LoRaMacFlags.Bits.expecting_beacon = true;

    Radio::Rx(2000);

    BeaconCtx.lastSendAtErr = microseconds(0);
}

void guard_callback()
{
    BeaconCtx.guard = true;
    Radio::Standby( );
    Radio::SetChannel( Channels[Channel].Frequency );

    tx_timeout.detach();
    mac_printf("sendOpportunities:%u\r\n", BeaconCtx.sendOpportunities);
    BeaconCtx.sendOpportunities = 0;

                        //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, true, false, false);
    Radio::LoRaModemConfig(LORA_BANDWIDTH_KHZ, Datarates[BEACON_CHANNEL_DR], 1);
    Radio::SetFixedPayloadLength(BEACON_SIZE);

#ifdef SX128x_H 
    /* explicit to implicit header: does sx1280 really need this a 2nd time? */
                        //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, true, false, false);
#endif /* SX128x_H */
    Radio::SetLoRaSymbolTimeout(BeaconCtx.nSymbsTimeout);
}

static void us_to_nSymbTimeout(unsigned us)
{
    mac_printf("symTo:%u ", us);
    BeaconCtx.nSymbsTimeout = us / BeaconCtx.symbol_period_us;
    if (BeaconCtx.nSymbsTimeout < (MIN_SYMBOL_TIMEOUT+BeaconCtx.Precess_symbols)) {
        BeaconCtx.nSymbsTimeout = MIN_SYMBOL_TIMEOUT+BeaconCtx.Precess_symbols;
    } else if (BeaconCtx.nSymbsTimeout > 255)
        BeaconCtx.nSymbsTimeout = 255;

    BeaconCtx.SymbolTimeout_us = BeaconCtx.nSymbsTimeout * BeaconCtx.symbol_period_us;
    mac_printf("%u\r\n", BeaconCtx.nSymbsTimeout);
}

static uint16_t beacon_crc( uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT
    const uint16_t polynom = 0x1021;
    // CRC initial value
    uint16_t crc = 0x0000;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint16_t ) buffer[i] << 8;
        for( uint16_t j = 0; j < 8; ++j )
        {
            crc = ( crc & 0x8000 ) ? ( crc << 1 ) ^ polynom : ( crc << 1 );
        }
    }

    return crc;
}

void rx_beacon(uint16_t size)
{
    static bool compensate_precession = false;
    int32_t compensation = 0;
    /* have beacon end, need beacon start */
    microseconds beacon_start_offset_us(BeaconCtx.beaconStartToRxDone);
    LowPowerClock::time_point ThisBeaconRxAt = Radio::irqAt - beacon_start_offset_us;

    BeaconCtx.num_consecutive_ok++;
    mac_printf("rx_beacon %llu ", Radio::irqAt.time_since_epoch().count());
    BeaconCtx.rx_precession_us = ThisBeaconRxAt.time_since_epoch().count() - BeaconCtx.rx_setup_at.time_since_epoch().count();
    if (BeaconCtx.state != BEACON_STATE_FIRST_ACQ) {
        BeaconCtx.known_working_BeaconRxTimerError_us = BeaconCtx.last_BeaconRxTimerError_us;
        BeaconCtx.last_BeaconRxTimerError_us = (ThisBeaconRxAt.time_since_epoch().count() - BeaconCtx.LastBeaconRxAt.time_since_epoch().count()) - (BEACON_INTERVAL_us * (BeaconCtx.num_missed+1));

        if (BeaconCtx.num_missed > 0) {
            /* Timer error is measured over more than one beacon period.
             * Scale to error seen over single beacon period */
            BeaconCtx.last_BeaconRxTimerError_us /= BeaconCtx.num_missed + 1;
        }

        if (BeaconCtx.state == BEACON_STATE_ACQ_ERROR) {
            mac_printf("-->LOCKED ");
            BeaconCtx.state = BEACON_STATE_LOCKED;
            compensate_precession = true;
        }
    } else {
        /* ignore precession at first acquisition because it has slot resolution added */
        mac_printf("-->ACQ_ERROR ");
        // next beacon will give us our crystal error
        BeaconCtx.state = BEACON_STATE_ACQ_ERROR;
    }

    mac_printf("err%d=%llu-%llu ", BeaconCtx.last_BeaconRxTimerError_us, ThisBeaconRxAt.time_since_epoch().count(), BeaconCtx.LastBeaconRxAt.time_since_epoch().count());
    if (BeaconCtx.num_missed > 0)
        mac_printf("missed%u ", BeaconCtx.num_missed);

    mac_printf(" rx-before-tx:%d ", BeaconCtx.rx_precession_us);
    if (BeaconCtx.last_BeaconRxTimerError_us > 40000 || BeaconCtx.last_BeaconRxTimerError_us < -40000) {
        BeaconCtx.timeout_rx.detach();
        BeaconCtx.timeout_guard.detach();
        mac_printf("halt\r\n");
        for (;;) asm("nop");
    }
    BeaconCtx.LastBeaconRxAt = ThisBeaconRxAt;

    if (BeaconCtx.state == BEACON_STATE_LOCKED) {
        if (compensate_precession) {
            compensation = BeaconCtx.rx_precession_us - TARGET_PRECESSION_us + BeaconCtx.last_BeaconRxTimerError_us;
            mac_printf(" comp%ld", compensation);
        }
    }

    // reference tick for uplink schedule: when gateway started beacon
    microseconds send_offset_us(BeaconCtx.rx_precession_us + (BeaconCtx.tx_slot_offset * PING_SLOT_RESOLUTION_us));
    BeaconCtx.sendAt = BeaconCtx.rx_setup_at + send_offset_us;
    tx_timeout.attach_absolute(&send_callback, BeaconCtx.sendAt - mcu_wakeup_latency);
    mac_printf("sendAt:%llu ", BeaconCtx.sendAt.time_since_epoch().count());

    microseconds us_to_next_beacon(BEACON_INTERVAL_us + compensation);
    BeaconCtx.rx_setup_at += us_to_next_beacon;

    BeaconCtx.timeout_rx.attach_absolute(&OnRxBeaconSetup, BeaconCtx.rx_setup_at - mcu_wakeup_latency);
    microseconds beacon_guard_us(BEACON_GUARD_us);
    BeaconCtx.timeout_guard.attach_absolute(&guard_callback, BeaconCtx.rx_setup_at - beacon_guard_us - mcu_wakeup_latency);

    BeaconCtx.num_missed = 0;

    MlmeIndication.MlmeIndication = MLME_BEACON;
    MlmeIndication.Status = LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED;
    LoRaMacPrimitives->MacMlmeIndication( &MlmeIndication );

    /* check beacon payload */
    uint16_t calc_crc = beacon_crc(Radio::radio.rx_buf, 4);
    uint16_t rx_crc = Radio::radio.rx_buf[4];
    rx_crc |= Radio::radio.rx_buf[5] << 8;
    if (rx_crc == calc_crc) {
        unsigned int rx = Radio::radio.rx_buf[0];
        rx |= Radio::radio.rx_buf[1] << 8;
        rx |= Radio::radio.rx_buf[2] << 16;
        rx |= Radio::radio.rx_buf[3] << 24;
        if (rx != 0) {
            McpsIndication.McpsIndication = MCPS_MULTICAST;
            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
            McpsIndication.Buffer = Radio::radio.rx_buf;
            McpsIndication.BufferSize = 4;
            McpsIndication.RxData = true;
            LoRaMacPrimitives->MacMcpsIndication( &McpsIndication );
        }
    } else
        mac_printf("calc_crc:%04x rx_crc:%04x\r\n", calc_crc, rx_crc);
} // ..rx_beacon()


#define JOIN_ACCEPT_MAX_SIZE        34
static void OnRadioRxDone(uint8_t size, float rssi, float snr )
{
    uint8_t _jaDecrypted[JOIN_ACCEPT_MAX_SIZE];
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    bool skipIndication = false;

    uint8_t pktHeaderLen = 0;
    uint32_t address = 0;
    uint8_t appPayloadStartIndex = 0;
    uint8_t port = 0xFF;
    uint8_t frameLen = 0;
    uint32_t mic = 0;
    uint32_t micRx = 0;

    uint16_t sequenceCounter = 0;
    uint16_t sequenceCounterPrev = 0;
    uint16_t sequenceCounterDiff = 0;
    uint32_t downLinkCounter = 0;

    MulticastParams_t *curMulticastParams = NULL;
    uint8_t *nwkSKey = LoRaMacNwkSKey;
    uint8_t *appSKey = LoRaMacAppSKey;

    uint8_t multicast = 0;

    bool isMicOk = false;

    McpsConfirm.AckReceived = false;
    McpsIndication.Rssi = rssi;
    McpsIndication.Snr = snr;
    McpsIndication.Port = 0;
    McpsIndication.Multicast = 0;
    McpsIndication.FramePending = 0;
    McpsIndication.Buffer = NULL;
    McpsIndication.BufferSize = 0;
    McpsIndication.RxData = false;
    McpsIndication.AckReceived = false;
    McpsIndication.DownLinkCounter = 0;
    McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

    Radio::Sleep( );

    if (LoRaMacFlags.Bits.expecting_beacon) {
        rx_beacon(size);
        LoRaMacFlags.Bits.expecting_beacon = false;
        return;
    }

    macHdr.Value = Radio::radio.rx_buf[pktHeaderLen++];

    switch( macHdr.Bits.MType )
    {
        case FRAME_TYPE_JOIN_ACCEPT:
            if (LoRaMacFlags.Bits.IsLoRaMacNetworkJoined)
            {
                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_JOIN_ACCEPT;
                PrepareRxDoneAbort( );
                return;
            }
            LoRaMacJoinDecrypt( Radio::radio.rx_buf + 1, size - 1, LoRaMacAppKey, &_jaDecrypted[1]);

            _jaDecrypted[0] = macHdr.Value;

            LoRaMacJoinComputeMic( _jaDecrypted, size - LORAMAC_MFR_LEN, LoRaMacAppKey, &mic );

            micRx |= ( uint32_t )_jaDecrypted[size - LORAMAC_MFR_LEN];
            micRx |= ( ( uint32_t )_jaDecrypted[size - LORAMAC_MFR_LEN + 1] << 8 );
            micRx |= ( ( uint32_t )_jaDecrypted[size - LORAMAC_MFR_LEN + 2] << 16 );
            micRx |= ( ( uint32_t )_jaDecrypted[size - LORAMAC_MFR_LEN + 3] << 24 );

            if( micRx == mic )
            {
                uint32_t beaconDur;
                LoRaMacJoinComputeSKeys( LoRaMacAppKey, _jaDecrypted + 1, LoRaMacDevNonce, LoRaMacNwkSKey, LoRaMacAppSKey );

                LoRaMacNetID = ( uint32_t )_jaDecrypted[4];
                LoRaMacNetID |= ( ( uint32_t )_jaDecrypted[5] << 8 );
                LoRaMacNetID |= ( ( uint32_t )_jaDecrypted[6] << 16 );

                LoRaMacDevAddr = ( uint32_t )_jaDecrypted[7];
                LoRaMacDevAddr |= ( ( uint32_t )_jaDecrypted[8] << 8 );
                LoRaMacDevAddr |= ( ( uint32_t )_jaDecrypted[9] << 16 );
                LoRaMacDevAddr |= ( ( uint32_t )_jaDecrypted[10] << 24 );

                // DLSettings
                LoRaMacParams.Rx1DrOffset = ( _jaDecrypted[11] >> 4 ) & 0x07;

                LoRaMacParams.ReceiveDelay_us = ( _jaDecrypted[12] & 0x0F );
                if( LoRaMacParams.ReceiveDelay_us == 0 )
                    LoRaMacParams.ReceiveDelay_us = RECEIVE_DELAY_us;
                else
                    LoRaMacParams.ReceiveDelay_us *= 10;

                uint16_t beaconTimingDelay = _jaDecrypted[13] & 0xff;
                beaconTimingDelay |= _jaDecrypted[14] << 8;

                mac_printf("%lx slots:%x (rxdelay %lu)", LoRaMacDevAddr, beaconTimingDelay, LoRaMacParams.ReceiveDelay_us);
                {
                    unsigned us_to_beacon = ( PING_SLOT_RESOLUTION_us * beaconTimingDelay );
                    mac_printf(" us_to_beacon:%u ", us_to_beacon);
                    // time to beacon given as referenced to end of join request uplink
                    microseconds tx_done_to_rx_start_us(us_to_beacon - PPM_BEACON_INTERVAL);
                    BeaconCtx.rx_setup_at = txDoneAt + tx_done_to_rx_start_us;
                    BeaconCtx.timeout_rx.attach_absolute(&OnRxBeaconSetup, BeaconCtx.rx_setup_at - mcu_wakeup_latency);
                    microseconds beacon_guard_us(BEACON_GUARD_us);
                    BeaconCtx.timeout_guard.attach_absolute(&guard_callback, BeaconCtx.rx_setup_at - (beacon_guard_us + mcu_wakeup_latency));

                    mac_printf("beaconIn:%llu\r\n", BeaconCtx.rx_setup_at.time_since_epoch().count() - LowPowerClock::now().time_since_epoch().count());
                }

                BeaconCtx.tx_slot_offset = _jaDecrypted[15];
                BeaconCtx.tx_slot_offset |= _jaDecrypted[16] << 8;
                BeaconCtx.periodicity_slots = _jaDecrypted[17];
                BeaconCtx.periodicity_slots |= _jaDecrypted[18] << 8;

                beaconDur = _jaDecrypted[22];
                beaconDur <<= 8;
                beaconDur |= _jaDecrypted[21];
                beaconDur <<= 8;
                beaconDur |= _jaDecrypted[20];
                beaconDur <<= 8;
                beaconDur |= _jaDecrypted[19];
                BeaconCtx.beaconStartToRxDone = beaconDur + (rxLatencyFactorFromSF[SF_FROM_DR_(LORAMAC_DEFAULT_DATARATE)-FASTEST_SF] * BeaconCtx.symbol_period_us);


                /* nowSlot: now vs previous beacon */
                microseconds beacon_interval_us(BEACON_INTERVAL_us);
                BeaconCtx.LastBeaconRxAt = BeaconCtx.rx_setup_at - beacon_interval_us;
                unsigned us_since_last_beacon = LowPowerClock::now().time_since_epoch().count() - BeaconCtx.LastBeaconRxAt.time_since_epoch().count();
                unsigned nowSlot = us_since_last_beacon / PING_SLOT_RESOLUTION_us;
                unsigned useSlot = BeaconCtx.tx_slot_offset;
                while (useSlot < nowSlot)
                    useSlot += BeaconCtx.periodicity_slots;

                mac_printf("beaconDur:0x%lx (%lu) useSlot:%u nowSlot:%u ", beaconDur, BeaconCtx.beaconStartToRxDone, useSlot, nowSlot);
                microseconds ping_slot_offset_us(useSlot * PING_SLOT_RESOLUTION_us);
                BeaconCtx.sendAt = BeaconCtx.LastBeaconRxAt + ping_slot_offset_us;

                mac_printf("sendIn:%lld\r\n", BeaconCtx.sendAt.time_since_epoch().count() - LowPowerClock::now().time_since_epoch().count());
                tx_timeout.attach_absolute(send_callback, BeaconCtx.sendAt - mcu_wakeup_latency);
                BeaconCtx.sendOpportunities = 0;

                BeaconCtx.state = BEACON_STATE_FIRST_ACQ;
                BeaconCtx.guard = false;
                BeaconCtx.num_missed = 0;
                BeaconCtx.rx_precession_us = 0;
                BeaconCtx.num_consecutive_ok = 0;
                BeaconCtx.last_BeaconRxTimerError_us = -PPM_BEACON_INTERVAL;
                BeaconCtx.known_working_BeaconRxTimerError_us = -PPM_BEACON_INTERVAL;
                /* first beacon reception needs to open for 30ms timing resolution */
                BeaconCtx.Precess_symbols = ceil((float)(TARGET_PRECESSION_us / BeaconCtx.symbol_period_us));
                BeaconCtx.SymbolTimeout_us = PING_SLOT_RESOLUTION_us + (PPM_BEACON_INTERVAL * 4);   // error unknown at start
                BeaconCtx.nSymbsTimeout = BeaconCtx.SymbolTimeout_us / BeaconCtx.symbol_period_us;
                if (BeaconCtx.nSymbsTimeout < MIN_SYMBOL_TIMEOUT)
                    BeaconCtx.nSymbsTimeout = MIN_SYMBOL_TIMEOUT;
                else if (BeaconCtx.nSymbsTimeout > 255)
                    BeaconCtx.nSymbsTimeout = 255;
                mac_printf("startSymbTo:%u ", BeaconCtx.nSymbsTimeout);

                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                LoRaMacFlags.Bits.IsLoRaMacNetworkJoined = true;
                LoRaMacParams.ChannelsDatarate_fixed = LoRaMacParamsDefaults.ChannelsDatarate_fixed;
            }
            else
            {
                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL;
                mac_printf("join-mic-fail size:%u\r\n", size);
                JoinRequestTrials = MaxJoinRequestTrials; // stop trying
            }
            LoRaMacPrimitives->MacMlmeConfirm( &MlmeConfirm );
            LoRaMacFlags.Bits.MlmeReq = 0;  // MacMlmeConfirm() called
            break;
        case FRAME_TYPE_DATA_CONFIRMED_DOWN:
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
            {
                address = Radio::radio.rx_buf[pktHeaderLen++];
                address |= ( (uint32_t)Radio::radio.rx_buf[pktHeaderLen++] << 8 );
                address |= ( (uint32_t)Radio::radio.rx_buf[pktHeaderLen++] << 16 );
                address |= ( (uint32_t)Radio::radio.rx_buf[pktHeaderLen++] << 24 );

                if( address != LoRaMacDevAddr )
                {
                    curMulticastParams = MulticastChannels;
                    while( curMulticastParams != NULL )
                    {
                        if( address == curMulticastParams->Address )
                        {
                            multicast = 1;
                            nwkSKey = curMulticastParams->NwkSKey;
                            appSKey = curMulticastParams->AppSKey;
                            downLinkCounter = curMulticastParams->DownLinkCounter;
                            break;
                        }
                        curMulticastParams = curMulticastParams->Next;
                    }
                    if( multicast == 0 )
                    {
                        // We are not the destination of this frame.
                        McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL;
                        PrepareRxDoneAbort( );
                        return;
                    }
                }
                else
                {
                    multicast = 0;
                    nwkSKey = LoRaMacNwkSKey;
                    appSKey = LoRaMacAppSKey;
                    downLinkCounter = DownLinkCounter;
                }

                fCtrl.Value = Radio::radio.rx_buf[pktHeaderLen++];

                sequenceCounter = ( uint16_t )Radio::radio.rx_buf[pktHeaderLen++];
                sequenceCounter |= ( uint16_t )Radio::radio.rx_buf[pktHeaderLen++] << 8;

                appPayloadStartIndex = 8 + fCtrl.Bits.FOptsLen;

                micRx |= ( uint32_t )Radio::radio.rx_buf[size - LORAMAC_MFR_LEN];
                micRx |= ( ( uint32_t )Radio::radio.rx_buf[size - LORAMAC_MFR_LEN + 1] << 8 );
                micRx |= ( ( uint32_t )Radio::radio.rx_buf[size - LORAMAC_MFR_LEN + 2] << 16 );
                micRx |= ( ( uint32_t )Radio::radio.rx_buf[size - LORAMAC_MFR_LEN + 3] << 24 );

                sequenceCounterPrev = ( uint16_t )downLinkCounter;
                sequenceCounterDiff = ( sequenceCounter - sequenceCounterPrev );

                if( sequenceCounterDiff < ( 1 << 15 ) )
                {
                    downLinkCounter += sequenceCounterDiff;
                    LoRaMacComputeMic( Radio::radio.rx_buf, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounter, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                    }
                }
                else
                {
                    // check for sequence roll-over
                    uint32_t  downLinkCounterTmp = downLinkCounter + 0x10000 + ( int16_t )sequenceCounterDiff;
                    LoRaMacComputeMic( Radio::radio.rx_buf, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, downLinkCounterTmp, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                        downLinkCounter = downLinkCounterTmp;
                    }
                }

                // Check for a the maximum allowed counter difference
                if( sequenceCounterDiff >= MAX_FCNT_GAP )
                {
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS;
                    McpsIndication.DownLinkCounter = downLinkCounter;
                    PrepareRxDoneAbort( );
                    return;
                }

                if( isMicOk == true )
                {
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                    McpsIndication.Multicast = multicast;
                    McpsIndication.FramePending = fCtrl.Bits.FPending;
                    McpsIndication.Buffer = NULL;
                    McpsIndication.BufferSize = 0;
                    McpsIndication.DownLinkCounter = downLinkCounter;

                    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;

                    MacCommandsBufferToRepeatIndex = 0;

                    // Update 32 bits downlink counter
                    if( multicast == 1 )
                    {
                        McpsIndication.McpsIndication = MCPS_MULTICAST;

                        if( ( curMulticastParams->DownLinkCounter == downLinkCounter ) &&
                            ( curMulticastParams->DownLinkCounter != 0 ) )
                        {
                            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                            McpsIndication.DownLinkCounter = downLinkCounter;
                            PrepareRxDoneAbort( );
                            return;
                        }
                        curMulticastParams->DownLinkCounter = downLinkCounter;
                    }
                    else
                    {
                        if( macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_DOWN )
                        {
                            LoRaMacFlags.Bits.SrvAckRequested = true;
                            McpsIndication.McpsIndication = MCPS_CONFIRMED;

                            if( ( DownLinkCounter == downLinkCounter ) &&
                                ( DownLinkCounter != 0 ) )
                            {
                                // Duplicated confirmed downlink. Skip indication.
                                // In this case, the MAC layer shall accept the MAC commands
                                // which are included in the downlink retransmission.
                                // It should not provide the same frame to the application
                                // layer again.
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            LoRaMacFlags.Bits.SrvAckRequested = false;
                            McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

                            if( ( DownLinkCounter == downLinkCounter ) &&
                                ( DownLinkCounter != 0 ) )
                            {
                                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                                McpsIndication.DownLinkCounter = downLinkCounter;
                                PrepareRxDoneAbort( );
                                return;
                            }
                        }
                        DownLinkCounter = downLinkCounter;
                    }

                    // This must be done before parsing the payload and the MAC commands.
                    // We need to reset the MacCommandsBufferIndex here, since we need
                    // to take retransmissions and repititions into account. Error cases
                    // will be handled in function OnMacStateCheckTimerEvent.
                    if( McpsConfirm.McpsRequest == MCPS_CONFIRMED )
                    {
                        if( fCtrl.Bits.Ack == 1 )
                        {// Reset MacCommandsBufferIndex when we have received an ACK.
                            MacCommandsBufferIndex = 0;
                        }
                    }
                    else
                    {// Reset the variable if we have received any valid frame.
                        MacCommandsBufferIndex = 0;
                    }

                    // Process payload and MAC commands
                    if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
                    {
                        port = Radio::radio.rx_buf[appPayloadStartIndex++];
                        frameLen = ( size - 4 ) - appPayloadStartIndex;

                        McpsIndication.Port = port;

                        if( port == 0 )
                        {
                            // Only allow frames which do not have fOpts
                            if( fCtrl.Bits.FOptsLen == 0 )
                            {
                                uint8_t macDecrypt[16];
                                LoRaMacPayloadDecrypt( Radio::radio.rx_buf + appPayloadStartIndex,
                                                       frameLen,
                                                       nwkSKey,
                                                       address,
                                                       DOWN_LINK,
                                                       downLinkCounter,
                                                       macDecrypt);

                                // Decode frame payload MAC commands
                                ProcessMacCommands( macDecrypt, 0, frameLen, snr );
                            }
                            else
                            {
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            if( fCtrl.Bits.FOptsLen > 0 )
                            {
                                // Decode Options field MAC commands. Omit the fPort.
                                ProcessMacCommands( Radio::radio.rx_buf, 8, appPayloadStartIndex - 1, snr );
                            }

                            LoRaMacPayloadDecrypt( Radio::radio.rx_buf + appPayloadStartIndex,
                                                   frameLen,
                                                   appSKey,
                                                   address,
                                                   DOWN_LINK,
                                                   downLinkCounter,
                                                   rxFRMPayload);

                            if( skipIndication == false )
                            {
                                McpsIndication.Buffer = rxFRMPayload;
                                McpsIndication.BufferSize = frameLen;
                                McpsIndication.RxData = true;
                            }
                        }
                    }
                    else
                    {
                        if( fCtrl.Bits.FOptsLen > 0 )
                        {
                            // Decode Options field MAC commands
                            ProcessMacCommands( Radio::radio.rx_buf, 8, appPayloadStartIndex, snr );
                        }
                    }

                    if( skipIndication == false )
                    {
                        // Check if the frame is an acknowledgement
                        if( fCtrl.Bits.Ack == 1 )
                        {
                            McpsConfirm.AckReceived = true;
                            McpsIndication.AckReceived = true;

                            // Stop the AckTimeout timer as no more retransmissions
                            // are needed.
                        }
                        else
                        {
                            McpsConfirm.AckReceived = false;
                        }
                    }
                    // Provide always an indication, skip the callback to the user application,
                    // in case of a confirmed downlink retransmission.
                    LoRaMacFlags.Bits.McpsInd = 1;
                    LoRaMacFlags.Bits.McpsIndSkip = skipIndication;
                }
                else
                {
                    McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;
                    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;
                    PrepareRxDoneAbort( );
                    return;
                }
            }
            break;
        case FRAME_TYPE_PROPRIETARY:
            {
                McpsIndication.McpsIndication = MCPS_PROPRIETARY;
                McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                McpsIndication.Buffer = Radio::radio.rx_buf;
                McpsIndication.BufferSize = size - pktHeaderLen;

                LoRaMacFlags.Bits.McpsInd = 1;
                break;
            }
        default:
            McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_RX_MTYPE;
            McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_RX_MTYPE;
            mac_printf("%d:macHdr:%02x(%f,%f) ", size, macHdr.Value, rssi, snr);
            PrepareRxDoneAbort( );
            break;
    }
    LoRaMacFlags.Bits.MacDone = 1;

    application_callbacks();

} // ..OnRadioRxDone()

static void OnRadioTxTimeout( void )
{
    Radio::Sleep( );

    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    LoRaMacFlags.Bits.MacDone = 1;
}

static void OnRadioRxError( void )
{
    Radio::Sleep( );

    if (LoRaMacFlags.Bits.NodeAckRequested)
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX_ERROR;
    }
    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX_ERROR;

    LoRaMacFlags.Bits.MacDone = 1;
}

static void
join_send()
{
    LoRaMacFlags.Bits.join_send = 1;
}

static void ScheduleTx( void )
{
    if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined)
        LoRaMacFlags.Bits.send = 1; // immediately send join asychronously
    else
        LoRaMacFlags.Bits.uplink_pending = 1;   // send synchronously
}

static void
join_send_bh()
{
    if (JoinRequestTrials < MaxJoinRequestTrials) {
        LoRaMacHeader_t macHdr;
        LoRaMacFrameCtrl_t fCtrl;

        if (++Channel == LORA_MAX_NB_CHANNELS)
            Channel = 0;
        mac_printf("<join-ch%u>", Channel);

        macHdr.Value = 0;
        macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

        fCtrl.Value = 0;
        fCtrl.Bits.Adr = 0;

        /* In case of join request retransmissions, the stack must prepare
         * the frame again, because the network server keeps track of the random
         * LoRaMacDevNonce values to prevent reply attacks. */
        PrepareFrame( &macHdr, &fCtrl, 0, NULL, 0 );

        ScheduleTx();
    } else {
        MlmeConfirm.MlmeRequest = MLME_JOIN;
        MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL;
        LoRaMacPrimitives->MacMlmeConfirm( &MlmeConfirm );
    }
} // ..join_send_bh()

static void OnRadioRxTimeout( void )
{
    Radio::Sleep( );

    pc_printf("OnRadioRxTimeout eb%u\r\n", LoRaMacFlags.Bits.expecting_beacon);
    if (LoRaMacFlags.Bits.expecting_beacon) {
        float ourErrSecs = BeaconCtx.known_working_BeaconRxTimerError_us / 1000000.0;

        LoRaMacFlags.Bits.expecting_beacon = false;

        microseconds rx_next_us(BEACON_INTERVAL_us + BeaconCtx.known_working_BeaconRxTimerError_us);
        BeaconCtx.rx_setup_at += rx_next_us;
        microseconds pre_rx_us(PPM_BEACON_INTERVAL / 4); // come up early when missed
        BeaconCtx.rx_setup_at -= pre_rx_us;
        us_to_nSymbTimeout(BeaconCtx.SymbolTimeout_us + PPM_BEACON_INTERVAL);
        mac_printf("beacon timeout ourErr:%f SymbTo:%u(%uus)\r\n", ourErrSecs, BeaconCtx.nSymbsTimeout, BeaconCtx.SymbolTimeout_us);

        BeaconCtx.timeout_rx.attach_absolute(&OnRxBeaconSetup, BeaconCtx.rx_setup_at - mcu_wakeup_latency);
        microseconds guard_us(BEACON_GUARD_us);
        BeaconCtx.timeout_guard.attach_absolute(&guard_callback, BeaconCtx.rx_setup_at - (guard_us + mcu_wakeup_latency));

        if (++BeaconCtx.num_missed > BEACONS_MISSED_LIMIT) {
            LoRaMacFlags.Bits.reJoin = 1;
        } else {
            MlmeIndication.MlmeIndication = MLME_BEACON;
            MlmeIndication.Status = LORAMAC_EVENT_INFO_STATUS_BEACON_LOST;
            LoRaMacPrimitives->MacMlmeIndication( &MlmeIndication );
        }

        microseconds base((BEACON_INTERVAL_us + BeaconCtx.known_working_BeaconRxTimerError_us) * BeaconCtx.num_missed);
        BeaconCtx.sendAt = BeaconCtx.LastBeaconRxAt + base;
        microseconds tx_slot_us(BeaconCtx.tx_slot_offset * PING_SLOT_RESOLUTION_us);
        BeaconCtx.sendAt += tx_slot_us;
        tx_timeout.attach_absolute(&send_callback, BeaconCtx.sendAt - mcu_wakeup_latency);

        BeaconCtx.num_consecutive_ok = 0;
    } else {
        if (LoRaMacFlags.Bits.MlmeReq && ( MlmeConfirm.MlmeRequest == MLME_JOIN )) {
            /* no join accept received: join retry */
            microseconds us((JoinRequestTrials*20000) + randr(0, 70000));
            tx_timeout.attach(&join_send, us - mcu_wakeup_latency);
        }
    }

    if (LoRaMacFlags.Bits.NodeAckRequested)
    {
        McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX_TIMEOUT;
    }
    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_RX_TIMEOUT;
    LoRaMacFlags.Bits.MacDone = 1;

    application_callbacks();
} // ..OnRadioRxTimeout();

static void OnAckTimeoutTimerEvent( void )
{
}

static bool ValueInRange( int8_t value, int8_t min, int8_t max )
{
    if( ( value >= min ) && ( value <= max ) )
    {
        return true;
    }
    return false;
}

static LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_BUSY;
    // The maximum buffer length must take MAC commands to re-send into account.
    uint8_t bufLen = LORA_MAC_COMMAND_MAX_LENGTH - MacCommandsBufferToRepeatIndex;

    switch( cmd )
    {
        case MOTE_MAC_LINK_CHECK_REQ:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this command
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
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
            if( MacCommandsBufferIndex < bufLen )
            {
                MacCommandsBuffer[MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }
    if( status == LORAMAC_STATUS_OK )
    {
        MacCommandsInNextTx = true;
    }
    return status;
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
            case MOTE_MAC_LINK_CHECK_REQ:
            { // 0 byte payload
                break;
            }
            default:
                break;
        }
    }

    return cmdCount;
}

static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr )
{
    while( macIndex < commandsSize )
    {
        // Decode Frame MAC commands
        switch( payload[macIndex++] )
        {
            case SRV_MAC_LINK_CHECK_ANS:
                MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                MlmeConfirm.DemodMargin = payload[macIndex++];
                MlmeConfirm.NbGateways = payload[macIndex++];
                break;
            case SRV_MAC_DEV_STATUS_REQ:
                {
                    uint8_t batteryLevel = BAT_LEVEL_NO_MEASURE;
                    if( ( LoRaMacCallbacks != NULL ) && ( LoRaMacCallbacks->GetBatteryLevel != NULL ) )
                    {
                        batteryLevel = LoRaMacCallbacks->GetBatteryLevel( );
                    }
                    AddMacCommand( MOTE_MAC_DEV_STATUS_ANS, batteryLevel, snr );
                    break;
                }
            case SRV_MAC_RX_TIMING_SETUP_REQ:
                {
                    uint8_t delay = payload[macIndex++] & 0x0F;

                    if( delay == 0 )
                    {
                        delay++;
                    }
                    LoRaMacParams.ReceiveDelay_us = delay * 1e6;
                    AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
                }
                break;
            default:
                // Unknown command. ABORT MAC commands processing
                return;
        }
    }
}

LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    LoRaMacFrameCtrl_t fCtrl;
    LoRaMacStatus_t status = LORAMAC_STATUS_PARAMETER_INVALID;

    fCtrl.Value = 0;
    fCtrl.Bits.FOptsLen      = 0;
    fCtrl.Bits.FPending      = 0;
    fCtrl.Bits.Ack           = false;
    fCtrl.Bits.AdrAckReq     = false;
    fCtrl.Bits.Adr           = false;

    // Prepare the frame
    status = PrepareFrame( macHdr, &fCtrl, fPort, fBuffer, fBufferSize );

    // Validate status
    if( status != LORAMAC_STATUS_OK )
    {
        return status;
    }

    // Reset confirm parameters
    McpsConfirm.AckReceived = false;
    McpsConfirm.UpLinkCounter = UpLinkCounter;

    ScheduleTx();

    return LORAMAC_STATUS_OK;
}


static void ResetMacParameters( void )
{
    LoRaMacFlags.Bits.IsLoRaMacNetworkJoined = false;

    // Counters
    UpLinkCounter = 0;
    DownLinkCounter = 0;

    ChannelsNbRepCounter = 0;

    MacCommandsBufferIndex = 0;
    MacCommandsBufferToRepeatIndex = 0;

    IsRxWindowsEnabled = true;

    LoRaMacParams.ChannelsTxPower = LoRaMacParamsDefaults.ChannelsTxPower;
    LoRaMacParams.ChannelsDatarate_fixed = LoRaMacParamsDefaults.ChannelsDatarate_fixed;

    LoRaMacParams.Rx1DrOffset = LoRaMacParamsDefaults.Rx1DrOffset;

    LoRaMacFlags.Bits.NodeAckRequested = false;
    LoRaMacFlags.Bits.SrvAckRequested = false;
    MacCommandsInNextTx = false;

    // Reset Multicast downlink counters
    MulticastParams_t *cur = MulticastChannels;
    while( cur != NULL )
    {
        cur->DownLinkCounter = 0;
        cur = cur->Next;
    }

}

LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    uint16_t i;
    uint32_t mic = 0;
    const void* payload = fBuffer;
    uint8_t framePort = fPort;
    uint8_t LoRaMacTxPayloadLen = 0;

    LoRaMacFlags.Bits.NodeAckRequested = false;
    tx_buf_len = 0;

    if( fBuffer == NULL )
    {
        fBufferSize = 0;
    }

    LoRaMacTxPayloadLen = fBufferSize;

    Radio::radio.tx_buf[tx_buf_len++] = macHdr->Value;

    switch( macHdr->Bits.MType )
    {
        case FRAME_TYPE_JOIN_REQ:
            memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacAppEui, 8 );
            tx_buf_len += 8;
            memcpyr( Radio::radio.tx_buf + tx_buf_len, LoRaMacDevEui, 8 );
            tx_buf_len += 8;

            LoRaMacDevNonce = Radio::Random( );

            Radio::radio.tx_buf[tx_buf_len++] = LoRaMacDevNonce & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

            LoRaMacJoinComputeMic( Radio::radio.tx_buf, tx_buf_len & 0xFF, LoRaMacAppKey, &mic );

            Radio::radio.tx_buf[tx_buf_len++] = mic & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( mic >> 24 ) & 0xFF;

            break;
        case FRAME_TYPE_DATA_CONFIRMED_UP:
            LoRaMacFlags.Bits.NodeAckRequested = true;
            //Intentional fallthrough
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:
            if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined)
            {
                return LORAMAC_STATUS_NO_NETWORK_JOINED; // No network has been joined yet
            }

            fCtrl->Bits.AdrAckReq = 0;

            if( LoRaMacFlags.Bits.SrvAckRequested == true )
            {
                LoRaMacFlags.Bits.SrvAckRequested = false;
                fCtrl->Bits.Ack = 1;
            }

            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;

            Radio::radio.tx_buf[tx_buf_len++] = fCtrl->Value;

            Radio::radio.tx_buf[tx_buf_len++] = UpLinkCounter & 0xFF;
            Radio::radio.tx_buf[tx_buf_len++] = ( UpLinkCounter >> 8 ) & 0xFF;

            // Copy the MAC commands which must be re-send into the MAC command buffer
            memcpy1( &MacCommandsBuffer[MacCommandsBufferIndex], MacCommandsBufferToRepeat, MacCommandsBufferToRepeatIndex );
            MacCommandsBufferIndex += MacCommandsBufferToRepeatIndex;

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                if( ( MacCommandsBufferIndex <= LORA_MAC_COMMAND_MAX_LENGTH ) && ( MacCommandsInNextTx == true ) )
                {
                    fCtrl->Bits.FOptsLen += MacCommandsBufferIndex;

                    // Update FCtrl field with new value of OptionsLength
                    Radio::radio.tx_buf[0x05] = fCtrl->Value;
                    for( i = 0; i < MacCommandsBufferIndex; i++ )
                    {
                        Radio::radio.tx_buf[tx_buf_len++] = MacCommandsBuffer[i];
                    }
                }
            }
            else
            {
                if( ( MacCommandsBufferIndex > 0 ) && ( MacCommandsInNextTx ) )
                {
                    LoRaMacTxPayloadLen = MacCommandsBufferIndex;
                    payload = MacCommandsBuffer;
                    framePort = 0;
                }
            }
            MacCommandsInNextTx = false;
            // Store MAC commands which must be re-send in case the device does not receive a downlink anymore
            MacCommandsBufferToRepeatIndex = ParseMacCommandsToRepeat( MacCommandsBuffer, MacCommandsBufferIndex, MacCommandsBufferToRepeat );
            if( MacCommandsBufferToRepeatIndex > 0 )
            {
                MacCommandsInNextTx = true;
            }

            if( ( payload != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                Radio::radio.tx_buf[tx_buf_len++] = framePort;

                if( framePort == 0 )
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, LoRaMacTxPayloadLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &Radio::radio.tx_buf[tx_buf_len] );
                }
                else
                {
                    LoRaMacPayloadEncrypt( (uint8_t* ) payload, LoRaMacTxPayloadLen, LoRaMacAppSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &Radio::radio.tx_buf[tx_buf_len] );
                }
            }
            tx_buf_len = tx_buf_len + LoRaMacTxPayloadLen;

            LoRaMacComputeMic( Radio::radio.tx_buf, tx_buf_len, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &mic );

            Radio::radio.tx_buf[tx_buf_len + 0] = mic & 0xFF;
            Radio::radio.tx_buf[tx_buf_len + 1] = ( mic >> 8 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len + 2] = ( mic >> 16 ) & 0xFF;
            Radio::radio.tx_buf[tx_buf_len + 3] = ( mic >> 24 ) & 0xFF;

            tx_buf_len += LORAMAC_MFR_LEN;

            break;
        case FRAME_TYPE_PROPRIETARY:
            if( ( fBuffer != NULL ) && ( LoRaMacTxPayloadLen > 0 ) )
            {
                memcpy1( Radio::radio.tx_buf + tx_buf_len, ( uint8_t* ) fBuffer, LoRaMacTxPayloadLen );
                tx_buf_len = tx_buf_len + LoRaMacTxPayloadLen;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }

    return LORAMAC_STATUS_OK;
}


LoRaMacStatus_t SetTxContinuousWave( uint16_t timeout )
{
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;

    txPowerIndex = LoRaMacParams.ChannelsTxPower;
    txPower = TxPowers[txPowerIndex];
    Radio::SetTxContinuousWave( Channels[Channel].Frequency, txPower, timeout );

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t SetTxContinuousWave1( uint16_t timeout, uint32_t frequency, uint8_t power )
{
    Radio::SetTxContinuousWave( frequency, power, timeout );

    return LORAMAC_STATUS_OK;
}

void seconds()
{
    mac_printf("second\r\n");
}

void on_dio0_top_half()
{
}

unsigned get_symbol_period_us(uint8_t sf)
{
    float bwMHz = LORA_BANDWIDTH_KHZ / 1000.0;
    // return symbol period in microseconds 
    return (1 << sf) / bwMHz;
}

const RadioEvents_t rev = {
    /* Dio0_top_half */     on_dio0_top_half,
    /* TxDone_topHalf */    OnRadioTxDone_topHalf,
    /* TxDone_botHalf */    OnRadioTxDone_bh,
    /* TxTimeout  */        OnRadioTxTimeout,
    /* RxDone  */           OnRadioRxDone,
    /* RxTimeout  */        OnRadioRxTimeout,
    /* RxError  */          OnRadioRxError,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

osThreadId_t tid_main;

void sleep_test_callback()
{
    txDoneAt = LowPowerClock::now();
    osThreadFlagsSet(tid_main, 1);
}

LoRaMacStatus_t LoRaMacInitialization( LoRaMacPrimitives_t *primitives, LoRaMacCallback_t *callbacks )
{
    if (primitives == NULL)
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if( ( primitives->MacMcpsConfirm == NULL ) ||
        ( primitives->MacMcpsIndication == NULL ) ||
        ( primitives->MacMlmeConfirm == NULL ) )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    LoRaMacPrimitives = primitives;
    LoRaMacCallbacks = callbacks;

    LoRaMacFlags.Value = 0;

    JoinRequestTrials = 0;
    MaxJoinRequestTrials = 255;

    BeaconCtx.symbol_period_us = get_symbol_period_us(Datarates[LORAMAC_DEFAULT_DATARATE]);

    // Reset to defaults
    LoRaMacParamsDefaults.ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;
    LoRaMacParamsDefaults.ChannelsDatarate_fixed = LORAMAC_DEFAULT_DATARATE;

    LoRaMacParamsDefaults.SystemMaxRxError_ms = 20;
    LoRaMacParamsDefaults.MinRxSymbols = (LoRaMacParamsDefaults.SystemMaxRxError_ms * 1000) / BeaconCtx.symbol_period_us; 
    if (LoRaMacParamsDefaults.MinRxSymbols < MIN_SYMBOL_TIMEOUT)
        LoRaMacParamsDefaults.MinRxSymbols = MIN_SYMBOL_TIMEOUT;

    LoRaMacParamsDefaults.MaxRxWindow = MAX_RX_WINDOW;

    LoRaMacParamsDefaults.ReceiveDelay_us = RECEIVE_DELAY_us;
    LoRaMacParamsDefaults.JoinAcceptDelay_us = JOIN_ACCEPT_DELAY_us;

    LoRaMacParamsDefaults.ChannelsNbRep = 1;
    LoRaMacParamsDefaults.Rx1DrOffset = 0;

    for( uint8_t i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
    {
        Channels[i].Frequency = LORAMAC_FIRST_CHANNEL + (i * LORAMAC_STEPWIDTH_CHANNEL);
        Channels[i].DrRange.Value = (LORAMAC_MAX_DATARATE << 4) | LORAMAC_MIN_DATARATE;
        Channels[i].Band = 0;
    }

    // Init parameters which are not set in function ResetMacParameters
    LoRaMacParams.SystemMaxRxError_ms = LoRaMacParamsDefaults.SystemMaxRxError_ms;
    LoRaMacParams.MinRxSymbols = LoRaMacParamsDefaults.MinRxSymbols;
    LoRaMacParams.MaxRxWindow = LoRaMacParamsDefaults.MaxRxWindow;
    LoRaMacParams.ReceiveDelay_us = LoRaMacParamsDefaults.ReceiveDelay_us;
    LoRaMacParams.JoinAcceptDelay_us = LoRaMacParamsDefaults.JoinAcceptDelay_us;
    LoRaMacParams.ChannelsNbRep = LoRaMacParamsDefaults.ChannelsNbRep;

    ResetMacParameters( );
    if (LoRaMacCryptoInit() < 0) {
        return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }

    // Initialize Radio driver
    Radio::Init(&rev);

    {
        unsigned sum = 0;
        sleep_manager_unlock_deep_sleep();
        tid_main = ThisThread::get_id();
        /*** How long this MCU takes to wake up from deep-sleep ***/
        for (unsigned n = 0; n < 16; n++) {
            long end, start;
            txDoneAt = LowPowerClock::now();
            start = txDoneAt.time_since_epoch().count();

            LoRaMacFlags.Value = 0;
            rx_timeout.attach(sleep_test_callback, 10ms);
            ThisThread::flags_wait_any(1);
            end = LowPowerClock::now().time_since_epoch().count();
            sum += (end - start) - 10000;
        }
        LoRaMacFlags.Value = 0;
        sum >>= 4;  // 16 samples = 2^4
        printf("mcu takes %uus to wake up\r\n", sum);
        mcu_wakeup_latency = microseconds(sum);
    }

    // Random seed initialization
    srand1( Radio::Random( ) );

    PublicNetwork = true;
    Radio::SetPublicNetwork( PublicNetwork );
    Radio::Sleep( );

    RxWindowsParam = ComputeRxWindowParameters(LORAMAC_DEFAULT_DATARATE, LoRaMacParams.SystemMaxRxError_ms);

    return LORAMAC_STATUS_OK;
} // ..LoRaMacInitialization()

LoRaMacStatus_t LoRaMacQueryTxPossible( uint8_t size, LoRaMacTxInfo_t* txInfo )
{
    uint8_t fOptLen = MacCommandsBufferIndex + MacCommandsBufferToRepeatIndex;

    if( txInfo == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    txInfo->CurrentPayloadSize = LORAMAC_PHY_MAXPAYLOAD;

    if( txInfo->CurrentPayloadSize >= fOptLen )
    {
        txInfo->MaxPossiblePayload = txInfo->CurrentPayloadSize - fOptLen;
    }
    else
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibGet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    switch( mibGet->Type )
    {
        case MIB_NETWORK_JOINED:
        {
            mibGet->Param.IsNetworkJoined = LoRaMacFlags.Bits.IsLoRaMacNetworkJoined;
            break;
        }
        case MIB_NET_ID:
        {
            mibGet->Param.NetID = LoRaMacNetID;
            break;
        }
        case MIB_DEV_ADDR:
        {
            mibGet->Param.DevAddr = LoRaMacDevAddr;
            break;
        }
        case MIB_NWK_SKEY:
        {
            mibGet->Param.NwkSKey = LoRaMacNwkSKey;
            break;
        }
        case MIB_APP_SKEY:
        {
            mibGet->Param.AppSKey = LoRaMacAppSKey;
            break;
        }
        case MIB_PUBLIC_NETWORK:
        {
            mibGet->Param.EnablePublicNetwork = PublicNetwork;
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            mibGet->Param.ChannelNbRep = LoRaMacParams.ChannelsNbRep;
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            mibGet->Param.MaxRxWindow = LoRaMacParams.MaxRxWindow;
            break;
        }
        case MIB_CHANNELS_DEFAULT_TX_POWER:
        {
            mibGet->Param.ChannelsDefaultTxPower = LoRaMacParamsDefaults.ChannelsTxPower;
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            mibGet->Param.ChannelsTxPower = LoRaMacParams.ChannelsTxPower;
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            mibGet->Param.UpLinkCounter = UpLinkCounter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            mibGet->Param.DownLinkCounter = DownLinkCounter;
            break;
        }
        case MIB_MULTICAST_CHANNEL:
        {
            mibGet->Param.MulticastList = MulticastChannels;
            break;
        }
        case MIB_SYSTEM_MAX_RX_ERROR:
        {
            mibGet->Param.SystemMaxRxError_ms = LoRaMacParams.SystemMaxRxError_ms;
            break;
        }
        case MIB_MIN_RX_SYMBOLS:
        {
            mibGet->Param.MinRxSymbols = LoRaMacParams.MinRxSymbols;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibSet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    switch( mibSet->Type )
    {
        case MIB_NETWORK_JOINED:
        {
            LoRaMacFlags.Bits.IsLoRaMacNetworkJoined = mibSet->Param.IsNetworkJoined;
            if (!LoRaMacFlags.Bits.IsLoRaMacNetworkJoined) {
                mac_printf("beaconDetach\r\n");
                BeaconCtx.timeout_rx.detach();
                BeaconCtx.timeout_guard.detach();
                BeaconCtx.state = BEACON_STATE_NONE;
            }
            break;
        }
        case MIB_NET_ID:
        {
            LoRaMacNetID = mibSet->Param.NetID;
            break;
        }
        case MIB_DEV_ADDR:
        {
            LoRaMacDevAddr = mibSet->Param.DevAddr;
            break;
        }
        case MIB_NWK_SKEY:
        {
            if( mibSet->Param.NwkSKey != NULL )
            {
                memcpy1( LoRaMacNwkSKey, mibSet->Param.NwkSKey,
                               sizeof( LoRaMacNwkSKey ) );
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_APP_SKEY:
        {
            if( mibSet->Param.AppSKey != NULL )
            {
                memcpy1( LoRaMacAppSKey, mibSet->Param.AppSKey,
                               sizeof( LoRaMacAppSKey ) );
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_PUBLIC_NETWORK:
        {
            PublicNetwork = mibSet->Param.EnablePublicNetwork;
            Radio::SetPublicNetwork( PublicNetwork );
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            if( ( mibSet->Param.ChannelNbRep >= 1 ) &&
                ( mibSet->Param.ChannelNbRep <= 15 ) )
            {
                LoRaMacParams.ChannelsNbRep = mibSet->Param.ChannelNbRep;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            LoRaMacParams.MaxRxWindow = mibSet->Param.MaxRxWindow;
            break;
        }
        case MIB_CHANNELS_DEFAULT_TX_POWER:
        {
            if( ValueInRange( mibSet->Param.ChannelsDefaultTxPower,
                              LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) )
            {
                LoRaMacParamsDefaults.ChannelsTxPower = mibSet->Param.ChannelsDefaultTxPower;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            if( ValueInRange( mibSet->Param.ChannelsTxPower,
                              LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) )
            {
                LoRaMacParams.ChannelsTxPower = mibSet->Param.ChannelsTxPower;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            UpLinkCounter = mibSet->Param.UpLinkCounter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            DownLinkCounter = mibSet->Param.DownLinkCounter;
            break;
        }
        case MIB_SYSTEM_MAX_RX_ERROR:
        {
            LoRaMacParams.SystemMaxRxError_ms = LoRaMacParamsDefaults.SystemMaxRxError_ms = mibSet->Param.SystemMaxRxError_ms;
            break;
        }
        case MIB_MIN_RX_SYMBOLS:
        {
            LoRaMacParams.MinRxSymbols = LoRaMacParamsDefaults.MinRxSymbols = mibSet->Param.MinRxSymbols;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMulticastChannelLink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    // Reset downlink counter
    channelParam->DownLinkCounter = 0;

    if( MulticastChannels == NULL )
    {
        // New node is the fist element
        MulticastChannels = channelParam;
    }
    else
    {
        MulticastParams_t *cur = MulticastChannels;

        // Search the last node in the list
        while( cur->Next != NULL )
        {
            cur = cur->Next;
        }
        // This function always finds the last node
        cur->Next = channelParam;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMulticastChannelUnlink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if( MulticastChannels != NULL )
    {
        if( MulticastChannels == channelParam )
        {
          // First element
          MulticastChannels = channelParam->Next;
        }
        else
        {
            MulticastParams_t *cur = MulticastChannels;

            // Search the node in the list
            while( cur->Next && cur->Next != channelParam )
            {
                cur = cur->Next;
            }
            // If we found the node, remove it
            if( cur->Next )
            {
                cur->Next = channelParam->Next;
            }
        }
        channelParam->Next = NULL;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMlmeRequest( MlmeReq_t *mlmeRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;

    if( mlmeRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    memset1( ( uint8_t* ) &MlmeConfirm, 0, sizeof( MlmeConfirm ) );

    MlmeConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_MLMEREQ;

    switch( mlmeRequest->Type )
    {
        case MLME_JOIN:
        {
            if( ( mlmeRequest->Req.Join.DevEui == NULL ) ||
                ( mlmeRequest->Req.Join.AppEui == NULL ) ||
                ( mlmeRequest->Req.Join.AppKey == NULL ) ||
                ( mlmeRequest->Req.Join.NbTrials == 0 ) )
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }

            // Enables at least the usage of all datarates.
            if( mlmeRequest->Req.Join.NbTrials < 48 )
            {
                mlmeRequest->Req.Join.NbTrials = 48;
            }

            LoRaMacFlags.Bits.MlmeReq = 1;
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;

            LoRaMacDevEui = mlmeRequest->Req.Join.DevEui;
            LoRaMacAppEui = mlmeRequest->Req.Join.AppEui;
            LoRaMacAppKey = mlmeRequest->Req.Join.AppKey;
            MaxJoinRequestTrials = mlmeRequest->Req.Join.NbTrials;

            // Reset variable JoinRequestTrials
            JoinRequestTrials = 0;

            // Setup header information
            macHdr.Value = 0;
            macHdr.Bits.MType  = FRAME_TYPE_JOIN_REQ;

            ResetMacParameters( );
            LoRaMacFlags.Bits.expecting_beacon = false;
            BeaconCtx.state = BEACON_STATE_NONE;

            Channel = 0;    // start with first channel
            mac_printf("<ch0>");
            mac_printf("mlme-join-send ch%u\r\n", Channel);
            status = Send( &macHdr, 0, NULL, 0 );
            break;
        }
        case MLME_LINK_CHECK:
        {
            LoRaMacFlags.Bits.MlmeReq = 1;
            // LoRaMac will send this command piggy-pack
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;

            status = AddMacCommand( MOTE_MAC_LINK_CHECK_REQ, 0, 0 );
            break;
        }
        case MLME_TXCW:
        {
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;
            LoRaMacFlags.Bits.MlmeReq = 1;
            status = SetTxContinuousWave( mlmeRequest->Req.TxCw.Timeout );
            break;
        }
        case MLME_TXCW_1:
        {
            MlmeConfirm.MlmeRequest = mlmeRequest->Type;
            LoRaMacFlags.Bits.MlmeReq = 1;
            status = SetTxContinuousWave1( mlmeRequest->Req.TxCw.Timeout, mlmeRequest->Req.TxCw.Frequency, mlmeRequest->Req.TxCw.Power );
            break;
        }
        default:
            break;
    }

    if( status != LORAMAC_STATUS_OK )
    {
        LoRaMacFlags.Bits.NodeAckRequested = false;
        LoRaMacFlags.Bits.MlmeReq = 0;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMcpsRequest( McpsReq_t *mcpsRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;
    uint8_t fPort = 0;
    void *fBuffer;
    uint16_t fBufferSize;
    bool readyToSend = false;

    if( mcpsRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    macHdr.Value = 0;
    memset1 ( ( uint8_t* ) &McpsConfirm, 0, sizeof( McpsConfirm ) );
    McpsConfirm.Status = LORAMAC_EVENT_INFO_STATUS_ERROR_MCPSREQ;

    switch( mcpsRequest->Type )
    {
        case MCPS_UNCONFIRMED:
        {
            readyToSend = true;

            macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            fPort = mcpsRequest->Req.Unconfirmed.fPort;
            fBuffer = mcpsRequest->Req.Unconfirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Unconfirmed.fBufferSize;
            break;
        }
        case MCPS_CONFIRMED:
        {
            readyToSend = true;

            macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
            fPort = mcpsRequest->Req.Confirmed.fPort;
            fBuffer = mcpsRequest->Req.Confirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Confirmed.fBufferSize;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            readyToSend = true;

            macHdr.Bits.MType = FRAME_TYPE_PROPRIETARY;
            fBuffer = mcpsRequest->Req.Proprietary.fBuffer;
            fBufferSize = mcpsRequest->Req.Proprietary.fBufferSize;
            break;
        }
        default:
            break;
    }

    if (readyToSend)
    {
        status = Send( &macHdr, fPort, fBuffer, fBufferSize );
        if (status == LORAMAC_STATUS_OK)
        {
            McpsConfirm.McpsRequest = mcpsRequest->Type;
            LoRaMacFlags.Bits.McpsReq = 1;
        }
        else
        {
            LoRaMacFlags.Bits.NodeAckRequested = false;
        }
    }

    return status;
}

void LoRaMacTestRxWindowsOn( bool enable )
{
    IsRxWindowsEnabled = enable;
}

void LoRaMacTestSetMic( uint16_t txPacketCounter )
{
    UpLinkCounter = txPacketCounter;
    //IsUpLinkCounterFixed = true;
}

void LoRaMacTestSetChannel( uint8_t channel )
{
    mac_printf("set-testch%u\r\n", channel);
    Channel = channel;
}


static RxConfigParams_t ComputeRxWindowParameters( int8_t datarate, uint32_t rxError )
{
    RxConfigParams_t rxConfigParams = { 0, 0 };
    double tSymbol = 0.0;
 
    rxConfigParams.Datarate = datarate;
 
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    if( datarate == DR_7 )
    { // FSK
        tSymbol = ( 1.0 / ( double )Datarates[datarate] ) * 8.0; // 1 symbol equals 1 byte
    }
    else
#endif
    { // LoRa
        tSymbol = ( ( double )( 1 << Datarates[datarate] ) / LORA_BANDWIDTH_KHZ ) * 1e3;
    }
 
    rxConfigParams.RxWindowTimeout = MAX( ( uint32_t )ceil( ( ( 2 * LoRaMacParams.MinRxSymbols - 8 ) * tSymbol + 2 * rxError ) / tSymbol ), LoRaMacParams.MinRxSymbols ); // Computed number of symbols
    mac_printf("RxWindowTimeout:%lu\r\n", rxConfigParams.RxWindowTimeout);
 
    return rxConfigParams;
}

void LoRaMacBottomHalf()
{
    if (LoRaMacFlags.Bits.join_send) {
        join_send_bh();
        LoRaMacFlags.Bits.join_send = 0;
    }
    if (LoRaMacFlags.Bits.send) {
        send_bh();
        LoRaMacFlags.Bits.send = 0;
    }

    if (LoRaMacFlags.Bits.reJoin) {
        MlmeReq_t mlmeReq;
        mlmeReq.Type = MLME_JOIN;

        mlmeReq.Req.Join.DevEui = LoRaMacDevEui;
        mlmeReq.Req.Join.AppEui = LoRaMacAppEui;
        mlmeReq.Req.Join.AppKey = LoRaMacAppKey;
        mlmeReq.Req.Join.NbTrials = 255;

        if (LoRaMacMlmeRequest(&mlmeReq) == LORAMAC_STATUS_OK)
            LoRaMacFlags.Bits.reJoin = 0;
    }

    Radio::service();
}
