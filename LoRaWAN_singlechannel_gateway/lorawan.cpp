#define MBEDTLS_CMAC_C
#include <stdint.h>
#include "lorawan.h"
#include "Commissioning.h"

#include "cmac.h"

//#define ANY_DEVEUI

const microseconds RECEIVE_DELAY_us(100000);

Timeout timeout;

#define LORA_FRAMEMICBYTES              4
#define LORA_ENCRYPTIONBLOCKBYTES       16
#define LORA_AUTHENTICATIONBLOCKBYTES   16
#define LORA_MAXFRAMELENGTH             235
#define LORA_MACHEADERLENGTH            1
#define LORA_MINDATAHEADERLENGTH        7
#define LORA_PORTLENGTH                 1
#define LORA_MAXDATABYTES    (LORA_MAXFRAMELENGTH - (LORA_MACHEADERLENGTH + LORA_MINDATAHEADERLENGTH + LORA_PORTLENGTH + LORA_FRAMEMICBYTES)) //excluding port
#define LORA_NETWORKADDRESSBITS         25


#define DEFAULT_DOWNLINK_PORT       2

const uint32_t network_id = 0x24;
uint32_t networkAddress = 0;  // bits 24..0 of DevAddr, for join accept
uint16_t next_available_tx_slot = 0;

uint8_t LoRaWan::user_downlink[128];
volatile uint16_t LoRaWan::rx_slot;
volatile uint32_t LoRaWan::beaconDur;
HighResClock::time_point LoRaWan::rx_at;
volatile flags_t LoRaWan::flags;
uint32_t LoRaWan::dev_addr_filter;
uint8_t LoRaWan::_tmp_payload_length;

#ifdef SX128x_H
    //                                      0    1   2  3  4  5  6  7
    const uint8_t LoRaWan::Datarates[]  = { 12, 11, 10, 9, 8, 7, 6, 5 };

    #if (BANDWIDTH_KHZ == 200)
        #if (LORAMAC_DEFAULT_DATARATE == 0)
        #elif (LORAMAC_DEFAULT_DATARATE == 1)
        #elif (LORAMAC_DEFAULT_DATARATE == 2)
        #elif (LORAMAC_DEFAULT_DATARATE == 3)
        #elif (LORAMAC_DEFAULT_DATARATE == 4)
            #define TX_SLOT_STEPPING        40  //approx 1.2 seconds        sf8bw200
            #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 8) // max num end-nodes 8
        #elif (LORAMAC_DEFAULT_DATARATE == 5)
            #define TX_SLOT_STEPPING        20  //approx 0.6 seconds        sf7bw200
            #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 8) // max num end-nodes 8
        #elif (LORAMAC_DEFAULT_DATARATE == 6)
            #define TX_SLOT_STEPPING        20  //approx 0.6 seconds        sf6bw200
            #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 16) // max num end-nodes 16
        #elif (LORAMAC_DEFAULT_DATARATE == 7)
            #define TX_SLOT_STEPPING        20  //approx 0.6 seconds        sf5bw200
            #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 32) // max num end-nodes 32
        #endif
    #elif (BANDWIDTH_KHZ == 400)
        /* DR0 to DR7 */
    #elif (BANDWIDTH_KHZ == 800)
        /* DR0 to DR7 */
    #elif (BANDWIDTH_KHZ == 1600)
        /* DR0 to DR7 */
    #endif /* BANDWIDTH_KHZ */

#elif defined(USE_BAND_915)
    /* us915-single-channel spreading factors: */
    const uint8_t LoRaWan::Datarates[]  = { 10, 9, 8,  7,  8,  0,  0, 0, 12, 11, 10, 9, 8, 7, 0, 0 };
    /* 
    * TX_SLOT_STEPPING: time for each mote
    * PERIODICITY_SLOTS: slot at which first mote can again transmit
    */
    #if (LORAMAC_DEFAULT_DATARATE == DR_8)
        #define TX_SLOT_STEPPING        267  //approx 8 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 15)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_9)
        #define TX_SLOT_STEPPING        133  //approx 4 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 24)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_10)
        #define TX_SLOT_STEPPING        67  //approx 2 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 24)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_11)
        #define TX_SLOT_STEPPING        33  //approx 1.0 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 24)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_12)
        #define TX_SLOT_STEPPING        16  //approx 0.5 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 24)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_13)
        #define TX_SLOT_STEPPING        8  //approx 0.25 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 8)
    #endif
    /* end USE_BAND_915 */
#elif defined(USE_BAND_433)
    const uint8_t LoRaWan::Datarates[]  = { 12, 11, 10, 9, 8, 7, 7, 50 };
    #if (LORAMAC_DEFAULT_DATARATE == DR_0)
        #define TX_SLOT_STEPPING        1067  //approx 32 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_1)
        #define TX_SLOT_STEPPING        533  //approx 16 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_2)
        #define TX_SLOT_STEPPING        267  //approx 8 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_3)
        #define TX_SLOT_STEPPING        133  //approx 4 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_4)
        #define TX_SLOT_STEPPING        67  //approx 2 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #elif (LORAMAC_DEFAULT_DATARATE == DR_5)
        #define TX_SLOT_STEPPING        33  //approx 1.0 seconds
        #define PERIODICITY_SLOTS       (TX_SLOT_STEPPING * 3)
    #else
        #error datarate
    #endif
    /* end USE_BAND_433 */
#endif

#if (PERIODICITY_SLOTS > 4095)
    #error "PERIODICITY_SLOTS too large"
#endif


typedef enum {
    MTYPE_JOIN_REQ = 0,
    MTYPE_JOIN_ACC,//1
    MTYPE_UNCONF_UP,//2
    MTYPE_UNCONF_DN,//3
    MTYPE_CONF_UP,//4
    MTYPE_CONF_DN,//5
    MTYPE_RFU,//6
    MTYPE_P,//7
} mtype_e;

typedef union {
    struct {
        uint8_t major   : 2;    // 0 1
        uint8_t rfu     : 3;    // 2 3 4
        uint8_t MType   : 3;    // 5 6 7
    } bits;
    uint8_t octet;
} mhdr_t;

typedef union {
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t FPending        : 1;    // 4 
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } dlBits;   // downlink  (gwtx)
    struct {
        uint8_t FOptsLen        : 4;    // 0 1 2 3
        uint8_t classB          : 1;    // 4    unused in classA
        uint8_t ACK             : 1;    // 5
        uint8_t ADCACKReq       : 1;    // 6
        uint8_t ADR             : 1;    // 7
    } ulBits;   // uplink   (gwrx)
    uint8_t octet;
} FCtrl_t;

typedef struct {
    uint32_t DevAddr;
    FCtrl_t FCtrl;
    uint16_t FCnt;
} __attribute__((packed)) fhdr_t;


typedef struct {
    mhdr_t mhdr;
    uint8_t AppEUI[LORA_EUI_LENGTH];
    uint8_t DevEUI[LORA_EUI_LENGTH];
    uint16_t DevNonce;
} __attribute__((packed)) join_req_t;

typedef enum eLoRaMacMoteCmd
{
    /*!
     * LinkCheckReq
     */
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    /*!
     * LinkADRAns
     */
    //MOTE_MAC_LINK_ADR_ANS            = 0x03,
    /*!
     * DutyCycleAns
     */
    //MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
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
}LoRaMacMoteCmd_t;

typedef enum eLoRaMacSrvCmd
{
    /*!
     * LinkCheckAns
     */
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    /*!
     * LinkADRReq
     */
    //SRV_MAC_LINK_ADR_REQ             = 0x03,
    /*!
     * DutyCycleReq
     */
    //SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
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
}LoRaMacSrvCmd_t;

mtype_e user_dowlink_mtype = MTYPE_UNCONF_DN;

static mbedtls_cipher_context_t ctx;

void pc_printf(const char *fmt, ...)
{
    char n, str[64];
    va_list arg_ptr;

    va_start(arg_ptr, fmt);
    n = vsnprintf(str, sizeof(str), fmt, arg_ptr);
    va_end(arg_ptr);
    pc.write(str, n);
}

void print_octets(char const* label, uint8_t const* buf, uint8_t buf_len)
{
    int i;
    pc_printf("%s:", label);
    for (i = 0; i < buf_len; i++)
        pc_printf(" %02x", buf[i]);
//    pc_printf("\n");
}

void LoRaWan::print_octets_rev(char const* label, uint8_t const* buf, uint8_t buf_len)
{
    int i;
    pc_printf("%s:", label);
    for (i = buf_len-1; i >= 0; i--)
        pc_printf(" %02x", buf[i]);
//    pc_printf("\n");
}

void LoRaWan::filtered_printf(uint32_t dev_addr, layer_e l, const char* format, ...)
{
    va_list args;

    if (dev_addr_filter != 0) {
        if (dev_addr_filter != dev_addr)
            return;
    }

    switch (l) {
        case MAC:
            if (!flags.show_mac)
                return;
            break;
        case APP:
            if (!flags.show_app)
                return;
            break;
        case BOTH:
            break;
    }

    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

uint8_t* Write4ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}


uint8_t* Write3ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write2ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input,
    input >>= 8;
    *(ptr++) = (uint8_t)input;

    return ptr;
}

uint8_t* Write1ByteValue(uint8_t output[], uint32_t input)
{
    uint8_t* ptr = output;

    *(ptr++) = (uint8_t)input;

    return ptr;
}

int LoRa_GenerateJoinFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint8_t* output)
{
    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];

    int ret;
    ret = mbedtls_cipher_cmac_starts(&ctx, key, 128);
    if (ret < 0)
        return ret;
    ret = mbedtls_cipher_cmac_update(&ctx, input, dataLength & 0xff);
    if (ret < 0)
        return ret;
    ret = mbedtls_cipher_cmac_finish(&ctx, temp);
    if (ret < 0)
        return ret;

    memcpy(output, temp, LORA_FRAMEMICBYTES);

    return 0;
}

int GenerateSessionKey(bool generateNetworkKey, const uint8_t* applicationKey, uint32_t networkId, uint32_t applicationNonce, uint16_t deviceNonce, uint8_t* output)
{
    uint8_t input[LORA_ENCRYPTIONBLOCKBYTES];

    {
        mbedtls_aes_context actx;
        mbedtls_aes_init(&actx);
        if (mbedtls_aes_setkey_enc(&actx, applicationKey, 128) < 0)
            return -1;

        input[0] = generateNetworkKey ? 0x01 : 0x02;
        uint8_t* ptr = &input[1];

        ptr = Write3ByteValue(ptr, applicationNonce);
        ptr = Write3ByteValue(ptr, networkId);
        ptr = Write2ByteValue(ptr, deviceNonce);
        memset(ptr, 0, LORA_ENCRYPTIONBLOCKBYTES - (ptr - input));

        mbedtls_aes_encrypt(&actx, input, output);

        mbedtls_aes_free(&actx);
    }

    return 0;
}

int CryptJoinServer(uint8_t const* key, uint8_t const* input, uint16_t length, uint8_t* output)
{
    int ret;
    mbedtls_aes_context actx;

    mbedtls_aes_init(&actx);
    ret = mbedtls_aes_setkey_dec(&actx, key, 128);
    if (ret < 0)
        return -1;

    mbedtls_aes_decrypt(&actx, input, output);

    if (length >= 16) {
        mbedtls_aes_decrypt(&actx, input + 16, output + 16);
    }

    mbedtls_aes_free(&actx);
    return 0;
}

void LoRaWan::SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, ota_mote_t* mote)
{
    uint8_t secondReceiveWindowDataRateNibble = 0;  // unused
    uint8_t networkSessionKey[LORA_CYPHERKEYBYTES];
    uint8_t uncyphered[LORA_MAXDATABYTES];
    uint8_t* current = uncyphered;
    uint32_t applicationNonce = rand() & 0xffffff;  // 24bit

    if (flags.do_downlink) {
        pc_printf("\e[41mSendJoinComplete(): tx busy\e[0m\r\n");
        return;
    }

    GenerateSessionKey(true, mote->app_key, network_id, applicationNonce, deviceNonce, networkSessionKey);

    memcpy(mote->network_session_key, networkSessionKey, LORA_CYPHERKEYBYTES);
    
    pc_printf("SendJoinComplete() ");
    if (mote->dev_addr == DEVADDR_NONE) {
        // new mote joining
        pc_printf("new-mote ");
        if ( mote->tx_slot_offset >= PERIODICITY_SLOTS) {
            pc_printf("max motes reached\r\n");
            return;
        }
        mote->dev_addr = ++networkAddress | (network_id << LORA_NETWORKADDRESSBITS);
        mote->tx_slot_offset = next_available_tx_slot;
        next_available_tx_slot += TX_SLOT_STEPPING;
    } else
        pc_printf("rejoin ");

    pc_printf(" mote->dev_addr:%lx ", mote->dev_addr);
    pc_printf("networkAddress:%lu\r\n", networkAddress);
    memset(current, 0 , LORA_MAXDATABYTES);
    *(current++) = MTYPE_JOIN_ACC << 5; // MHDR     0
    current = Write3ByteValue(current, applicationNonce);// 1 2 3
    current = Write3ByteValue(current, network_id);// 4 5 6
    current = Write4ByteValue(current, mote->dev_addr); // 7 8 9 10
    current = Write1ByteValue(current, (firstReceiveWindowDataRateoffset << 4) | (secondReceiveWindowDataRateNibble & 0xf)); // 11 
    //current = Write1ByteValue(current, classARxWindowDelay_s - 1); // 12
    current = Write1ByteValue(current, 0); // 12

    /* put beacon timing answer */
    pc_printf("slots:%u\r\n", rx_slot);
    current = Write2ByteValue(current, rx_slot); // 13, 14
    current = Write2ByteValue(current, mote->tx_slot_offset); // 15, 16
    current = Write2ByteValue(current, PERIODICITY_SLOTS); // 17, 18
    current = Write4ByteValue(current, beaconDur); // 19, 20, 21, 22
    current = Write4ByteValue(current, 0); //
    current = Write2ByteValue(current, 0); //

    uint16_t authenticatedBytes = current - uncyphered;
    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, uncyphered, authenticatedBytes, current);
    current += LORA_FRAMEMICBYTES;

    Radio::radio.tx_buf[0] = MTYPE_JOIN_ACC << 5; // MHDR
    //encrypt
    uint16_t cypherBytes = (current - uncyphered) - LORA_MACHEADERLENGTH;
    CryptJoinServer(mote->app_key, &uncyphered[LORA_MACHEADERLENGTH], cypherBytes, &Radio::radio.tx_buf[LORA_MACHEADERLENGTH]);

    /**** RF TX ********/
    _tmp_payload_length = current - uncyphered;
    timeout.attach_absolute(send_downlink, rx_at + RECEIVE_DELAY_us);
    flags.do_downlink = true;
    mote->FCntDown = 0;

    GenerateSessionKey(false, mote->app_key, network_id, applicationNonce, deviceNonce, mote->app_session_key);
}

int LoRa_GenerateDataFrameIntegrityCode(const uint8_t key[], uint8_t const input[], uint16_t dataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t* output)
{
    uint8_t temp[LORA_AUTHENTICATIONBLOCKBYTES];
    /*
    Generate artificial B[0] block
    Encrypt B[0] to give X[1]

    for n = 1 to number of blocks
        exclusive OR B[n] with X[n] to give Y[n]
        encrypt Yi using key to give X[n+1]
    */
    uint8_t b0[LORA_AUTHENTICATIONBLOCKBYTES];
    memset(b0, 0 , LORA_AUTHENTICATIONBLOCKBYTES);

    b0[ 0] = 0x49; //authentication flags

    b0[ 5] = up ? 0 : 1;
    Write4ByteValue(&b0[6], address);
    Write4ByteValue(&b0[10], sequenceNumber);

    b0[15] = (uint8_t)dataLength;

    if (mbedtls_cipher_cmac_starts(&ctx, key, 128))
        return -1;
 
    mbedtls_cipher_cmac_update(&ctx, b0, LORA_AUTHENTICATIONBLOCKBYTES);
 
    mbedtls_cipher_cmac_update(&ctx, input, dataLength);
 
    mbedtls_cipher_cmac_finish(&ctx, temp);
    memcpy(output, temp, LORA_FRAMEMICBYTES);
    return 0;
}

static uint16_t FindBlockOverhang(uint16_t inputDataLength)
{
    return inputDataLength & (LORA_ENCRYPTIONBLOCKBYTES - 1);
}

uint16_t CountBlocks(uint16_t inputDataLength)
{
    uint16_t blockSizeMinus1 = LORA_ENCRYPTIONBLOCKBYTES - 1;
    uint16_t inRoundDown = inputDataLength & ~blockSizeMinus1;
    uint16_t roundUp = (FindBlockOverhang(inputDataLength) > 0) ? 1 : 0;
    uint16_t result = inRoundDown / LORA_ENCRYPTIONBLOCKBYTES + roundUp;

    return result;
}

void BlockExOr(uint8_t const l[], uint8_t const r[], uint8_t out[], uint16_t bytes)
{
    uint8_t const* lptr = l;
    uint8_t const* rptr = r;
    uint8_t* optr = out;
    uint8_t const* const end = out + bytes;

    for (;optr < end; lptr++, rptr++, optr++)
        *optr = *lptr ^ *rptr;
}

void LoRa_EncryptPayload(const uint8_t key[], const uint8_t* in, uint16_t inputDataLength, uint32_t address, bool up, uint32_t sequenceNumber, uint8_t out[])
{
    uint8_t A[LORA_ENCRYPTIONBLOCKBYTES];
    mbedtls_aes_context actx;
    if (inputDataLength == 0)
        return;

    mbedtls_aes_init(&actx);
    mbedtls_aes_setkey_enc(&actx, key, 128);

    memset(A, 0, LORA_ENCRYPTIONBLOCKBYTES);

    A[ 0] = 0x01; //encryption flags
    A[ 5] = up ? 0 : 1;

    Write4ByteValue(&A[6], address);
    Write4ByteValue(&A[10], sequenceNumber);

    uint16_t const blocks = CountBlocks(inputDataLength);
    uint16_t const overHangBytes = FindBlockOverhang(inputDataLength);

    uint8_t const* blockInput = in;
    uint8_t* blockOutput = out;
    for (uint16_t i = 1; i <= blocks; i++, blockInput += LORA_ENCRYPTIONBLOCKBYTES, blockOutput += LORA_ENCRYPTIONBLOCKBYTES)
    {
        A[15] = (uint8_t)i;

        uint8_t S[LORA_CYPHERKEYBYTES];

        mbedtls_aes_encrypt(&actx, A, S);

        uint16_t bytesToExOr;
        if ((i < blocks) || (overHangBytes == 0))
            bytesToExOr = LORA_CYPHERKEYBYTES;
        else
            bytesToExOr = overHangBytes;

        BlockExOr(S, blockInput, blockOutput, bytesToExOr);
    }

    mbedtls_aes_free(&actx);
}

void put_queue_mac_cmds(ota_mote_t* mote, uint8_t cmd_len, uint8_t* cmd_buf)
{
    int i;
    uint8_t* this_cmd_buf = mote->macCmd_queue[mote->macCmd_queue_in_idx];
    this_cmd_buf[0] = cmd_len;

    pc_printf("put_queue_mac_cmds %u: ", cmd_len);
    for (i = 0; i < cmd_len; i++) {
        this_cmd_buf[i+1] = cmd_buf[i];
        pc_printf("%02x ", cmd_buf[i]);
    }
    pc_printf("\r\n");

    if (++mote->macCmd_queue_in_idx == MAC_CMD_QUEUE_SIZE)
        mote->macCmd_queue_in_idx = 0;

    if (mote->macCmd_queue_in_idx == mote->macCmd_queue_out_idx) {
        pc_printf("macCmd_queue full\r\n");
    }
}

void
LoRaWan::parse_mac_command(ota_mote_t* mote, uint8_t* rx_cmd_buf, uint8_t rx_cmd_buf_len)
{
    uint8_t cmd_buf[MAC_CMD_SIZE];
    uint8_t rx_cmd_buf_idx = 0;
    int i;
    pc_printf("rx_mac_command(s):");
    for (i = 0; i < rx_cmd_buf_len; i++)
        pc_printf("%02x ", rx_cmd_buf[i]);
    pc_printf("\n");

    while (rx_cmd_buf_idx < rx_cmd_buf_len) {

        switch (rx_cmd_buf[rx_cmd_buf_idx++]) {
            //float diff;
            uint16_t i_diff;
            case MOTE_MAC_LINK_CHECK_REQ:   // 0x02
                pc_printf("MOTE_MAC_LINK_CHECK_REQ\n");
                /* no payload in request */
                cmd_buf[0] = SRV_MAC_LINK_CHECK_ANS;
                cmd_buf[1] = 20;  // db margin above noise floor
                cmd_buf[2] = 1;  // gateway count
                put_queue_mac_cmds(mote, 3, cmd_buf);
                break;
#if 0
#endif
            case MOTE_MAC_BEACON_TIMING_REQ:    // 0x12
                /* no payload in request */
                /*diff = (float)(tick_at_next_beacon - tick_at_RxDone) / 30.0;
                i_diff = (int)floor(diff);*/
                i_diff = rx_slot;
                //pc_printf("MOTE_MAC_BEACON_TIMING_REQ slots:%.1f=%.1fms (int:%u,%u)", diff, diff*30.0, i_diff, i_diff*30);
                pc_printf("MOTE_MAC_BEACON_TIMING_REQ slots:%u", i_diff);
                cmd_buf[0] = SRV_MAC_BEACON_TIMING_ANS;   // 0x12
                cmd_buf[1] = i_diff & 0xff; //lsbyte first byte
                cmd_buf[2] = (i_diff >> 8) & 0xff;
                cmd_buf[3] = 0;   // beacon channel index
                put_queue_mac_cmds(mote, 4, cmd_buf);
                pc_printf("%02x %02x %02x\n", cmd_buf[1], cmd_buf[2], cmd_buf[3]);
                break;
            case MOTE_MAC_PING_SLOT_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                pc_printf("PING_SLOT_FREQ_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_BEACON_FREQ_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                pc_printf("BEACON_FREQ_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                pc_printf("RX_PARAM_SETUP_ANS status:0x%02x\n", i);
                break;
            case MOTE_MAC_NEW_CHANNEL_ANS:
                i = rx_cmd_buf[rx_cmd_buf_idx++];
                pc_printf("NEW_CHANNEL_ANS status:0x%02x\n", i);
                break;
            default:
                pc_printf("[31mTODO mac cmd %02x[0m\n", rx_cmd_buf[rx_cmd_buf_idx-1]);
                return;
        } // ..switch (<mac_command>)
    } // .. while have mac comannds

}

void LoRaWan::classA_downlink(ota_mote_t* mote)
{
    fhdr_t* fhdr = (fhdr_t*)&Radio::radio.tx_buf[1];
    uint8_t* mic_ptr;

    fhdr->DevAddr = mote->dev_addr;
    fhdr->FCnt = mote->FCntDown++;
    _tmp_payload_length += LORA_MACHEADERLENGTH + sizeof(fhdr_t) + fhdr->FCtrl.dlBits.FOptsLen;
    mic_ptr = &Radio::radio.tx_buf[_tmp_payload_length];

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, Radio::radio.tx_buf, _tmp_payload_length, fhdr->DevAddr, false, fhdr->FCnt, mic_ptr);
    _tmp_payload_length += LORA_FRAMEMICBYTES;

    timeout.attach_absolute(send_downlink, rx_at + RECEIVE_DELAY_us);
    flags.do_downlink = true;
}

void LoRaWan::parse_uplink(ota_mote_t* mote, uint8_t rx_size)
{
    uint8_t decrypted[256];
    uint32_t calculated_mic, rx_mic;
    fhdr_t *rx_fhdr = (fhdr_t*)&Radio::radio.rx_buf[1];
    mhdr_t* rx_mhdr = (mhdr_t*)&Radio::radio.rx_buf[0];
    int rxofs = sizeof(mhdr_t) + sizeof(fhdr_t) + rx_fhdr->FCtrl.ulBits.FOptsLen;
    int rxFRMPayload_length = 0;
    uint8_t* rxFRMPayload = NULL;
    uint8_t* rx_fport_ptr = NULL;
    bool decrypt_payload = false;

    if ((rx_size - LORA_FRAMEMICBYTES) > rxofs) {
        rxFRMPayload_length = (rx_size - LORA_FRAMEMICBYTES) - (rxofs + 1);
        rxFRMPayload = &Radio::radio.rx_buf[rxofs+1];
        rx_fport_ptr = &Radio::radio.rx_buf[rxofs];
        filtered_printf(mote->dev_addr, MAC, "port:%d, len:%d", *rx_fport_ptr, rxFRMPayload_length);
    } else {
        filtered_printf(mote->dev_addr, MAC, "no-payload");
    }

    LoRa_GenerateDataFrameIntegrityCode(mote->network_session_key, Radio::radio.rx_buf, rx_size-LORA_FRAMEMICBYTES, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, (uint8_t*)&calculated_mic);

    rx_mic = Radio::radio.rx_buf[rx_size-1] << 24;
    rx_mic += Radio::radio.rx_buf[rx_size-2] << 16;
    rx_mic += Radio::radio.rx_buf[rx_size-3] << 8;
    rx_mic += Radio::radio.rx_buf[rx_size-4];
    if (calculated_mic != rx_mic) {
        filtered_printf(mote->dev_addr, BOTH, "[31mgenMic:%08lx, rxMic:%08lx\r\n", calculated_mic, rx_mic);
        filtered_printf(mote->dev_addr, BOTH, "mic fail[0m\n");
        return;
    }

    if (rx_fport_ptr != NULL && *rx_fport_ptr == 0) {
        /* mac commands are encrypted onto port 0 */
        LoRa_EncryptPayload(mote->network_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
        filtered_printf(mote->dev_addr, MAC, ", mac commands encrypted on port 0");
        parse_mac_command(mote, decrypted, rxFRMPayload_length);
    } else {
        if (rx_fhdr->FCtrl.ulBits.FOptsLen > 0) {
            /* mac commands are in header */
            filtered_printf(mote->dev_addr, MAC, ", mac commands in header");
            rxofs = sizeof(mhdr_t) + sizeof(fhdr_t);
            parse_mac_command(mote, &Radio::radio.rx_buf[rxofs], rx_fhdr->FCtrl.ulBits.FOptsLen);
        }
        if (rxFRMPayload != NULL) {
            decrypt_payload = true;
        }
    }

    fhdr_t* tx_fhdr = (fhdr_t*)&Radio::radio.tx_buf[1];
    tx_fhdr->FCtrl.dlBits.FOptsLen = 0;

    /* TODO get queued mac cmds */

    _tmp_payload_length = 0;

    if (tx_fhdr->FCtrl.dlBits.FOptsLen > 0 || rx_mhdr->bits.MType == MTYPE_CONF_UP ||
        mote->user_downlink_length > 0 || rx_fhdr->FCtrl.ulBits.ADCACKReq)
    {
        /* something to send via downlink */
        if (rx_mhdr->bits.MType == MTYPE_CONF_UP)
            tx_fhdr->FCtrl.dlBits.ACK = 1;
        else
            tx_fhdr->FCtrl.dlBits.ACK = 0;

        if (mote->user_downlink_length > 0) {
            /* add user payload */
            int txo = sizeof(mhdr_t) + sizeof(fhdr_t) + tx_fhdr->FCtrl.dlBits.FOptsLen;
            uint8_t* tx_fport_ptr = &Radio::radio.tx_buf[txo];
            uint8_t* txFRMPayload = &Radio::radio.tx_buf[txo+1];
            LoRa_EncryptPayload(mote->app_session_key, user_downlink, mote->user_downlink_length, mote->dev_addr, false, mote->FCntDown, txFRMPayload);
            if (rx_fport_ptr != NULL)
                *tx_fport_ptr = *rx_fport_ptr;
            else
                *tx_fport_ptr = DEFAULT_DOWNLINK_PORT;

            _tmp_payload_length = tx_fhdr->FCtrl.dlBits.FOptsLen + mote->user_downlink_length + 1; // +1 for fport
            Radio::radio.tx_buf[0] = user_dowlink_mtype << 5; // MHDR
            pc_printf(", DL-send %d", mote->user_downlink_length);
        } else {
            /* downlink not triggered by user_downlink */
            /* downlink triggered by FOpotsLen > 0 or conf_uplink */
            Radio::radio.tx_buf[0] = MTYPE_UNCONF_DN << 5; // MHDR
        }

        classA_downlink(mote);

        mote->user_downlink_length = 0;  // mark as sent
    }

    if (decrypt_payload) {
        LoRa_EncryptPayload(mote->app_session_key, rxFRMPayload, rxFRMPayload_length, rx_fhdr->DevAddr, true, rx_fhdr->FCnt, decrypted);
        filtered_printf(mote->dev_addr, MAC, ", ");
        decrypted_uplink(mote->dev_addr, decrypted, rxFRMPayload_length, *rx_fport_ptr);
    }

    filtered_printf(mote->dev_addr, BOTH, "\r\n");
}

void LoRaWan::parse_join_req(ota_mote_t* mote, uint8_t rx_size)
{
    join_req_t* jreq_ptr = (join_req_t*)&Radio::radio.rx_buf[0];
    uint32_t rx_mic, calculated_mic;

    LoRa_GenerateJoinFrameIntegrityCode(mote->app_key, Radio::radio.rx_buf, rx_size-LORA_FRAMEMICBYTES, (uint8_t*)&calculated_mic);


    rx_mic = Radio::radio.rx_buf[rx_size-1] << 24;
    rx_mic += Radio::radio.rx_buf[rx_size-2] << 16;
    rx_mic += Radio::radio.rx_buf[rx_size-3] << 8;
    rx_mic += Radio::radio.rx_buf[rx_size-4];
    if (calculated_mic != rx_mic) {
        pc_printf("join_req mic fail: %08lx, %08lx\r\n", calculated_mic, rx_mic);
        return;
    }

    /* TODO check devNonce */
    SendJoinComplete(jreq_ptr->DevNonce, 0, mote);
}


int memcmp_rev(const uint8_t* a, const uint8_t* b, uint8_t len)
{
    int a_i, b_i = len - 1;
    for (a_i = 0; a_i < len; a_i++) {
        if (a[a_i] != b[b_i])
            return a[a_i] - b[b_i];
        else
            b_i--;
    }
    return 0;
}

void memcpy_rev(uint8_t* out, uint8_t* in, uint8_t len)
{
    int i;
    out += len;
    for (i = 0; i < len; i++) {
        out--;
        *out = in[i]; 
    }
}

int LoRaWan::init()
{
    const mbedtls_cipher_info_t *cipher_info;
    int i, ret;
    for (i = 0; i < N_MOTES; i++) {
        motes[i].dev_addr = DEVADDR_NONE;
    }

    mbedtls_cipher_init(&ctx);
    cipher_info = mbedtls_cipher_info_from_type( MBEDTLS_CIPHER_AES_128_ECB );
    if (cipher_info == NULL) {
        return -1;
    }
    ret = mbedtls_cipher_setup(&ctx, cipher_info);
    return ret;
}

#ifdef ANY_DEVEUI 
volatile uint8_t num_motes_joined = 0;
#endif /* ANY_DEVEUI */

int LoRaWan::parse_receive(uint8_t rx_size, float rssi, float snr)
{
    int i;
    ota_mote_t* mote = NULL;
    mhdr_t *mhdr = (mhdr_t*)Radio::radio.rx_buf;

    if (rx_size <= (sizeof(fhdr_t) + LORA_FRAMEMICBYTES)) {
        pc_printf("too small %d, snr:%.1f %ddBm\r\n", rx_size, snr, rssi);
        return 1;
    }
    if (mhdr->bits.major != 0) {
        pc_printf("unsupported major:%u\r\n", mhdr->bits.major);
        return 0;
    }

    if (mhdr->bits.MType == MTYPE_JOIN_REQ) {
        join_req_t* join_req = (join_req_t*)&Radio::radio.rx_buf[0];
        pc_printf("MTYPE_JOIN_REQ, ");

#ifdef ANY_DEVEUI
        if (num_motes_joined < N_MOTES && 
            (memcmp_rev(join_req->AppEUI, motes[num_motes_joined].app_eui, LORA_EUI_LENGTH) == 0))
        {
            pc_printf("assigning to mote %u\r\n", num_motes_joined);
            i = num_motes_joined++;
            mote = &motes[i];
            memcpy_rev(mote->dev_eui, join_req->DevEUI, LORA_EUI_LENGTH);
        }
#else
        for (i = 0; i < N_MOTES; i++) {
            if ((memcmp_rev(join_req->AppEUI, motes[i].app_eui, LORA_EUI_LENGTH) == 0) &&
                (memcmp_rev(join_req->DevEUI, motes[i].dev_eui, LORA_EUI_LENGTH) == 0))
            {
                pc_printf("found mote\r\n");
                mote = &motes[i];
            }
        }
#endif /* !ANY_DEVEUI */

        if (mote != NULL) {
            pc_printf("Join-Found\r\n");
            parse_join_req(mote, rx_size);
        } else {
            pc_printf("join-not-found:\r\n");
            print_octets_rev("app_eui", join_req->AppEUI, LORA_EUI_LENGTH);
            print_octets_rev("\r\ndev_eui", join_req->DevEUI, LORA_EUI_LENGTH);
            pc_printf("\r\n");
        }
    } else if (mhdr->bits.MType == MTYPE_UNCONF_UP || mhdr->bits.MType == MTYPE_CONF_UP) {
        fhdr_t *fhdr = (fhdr_t*)&Radio::radio.rx_buf[1];

        for (i = 0; i < N_MOTES; i++) {
            if (motes[i].dev_addr == fhdr->DevAddr) {
                mote = &motes[i];
            }
        }

        filtered_printf(mote->dev_addr, MAC, "%u, %lu, %.1fdB, %ddBm, ",
            LoRaWan::rx_slot, time(NULL), snr, rssi
        );

        if (mhdr->bits.MType == MTYPE_UNCONF_UP)
            filtered_printf(mote->dev_addr, MAC, "MTYPE_UNCONF_UP, ");
        else if (mhdr->bits.MType == MTYPE_CONF_UP)
            filtered_printf(mote->dev_addr, MAC, "MTYPE_CONF_UP, ");

        if (mote != NULL) {
            char *ptr, devEuiStr[(LORA_EUI_LENGTH*2)+1];
            for (i = 0, ptr = devEuiStr; i < LORA_EUI_LENGTH; i++) {
                sprintf(ptr, "%02x", mote->dev_eui[i]);
                ptr += 2;
            }
            filtered_printf(mote->dev_addr, MAC, "mote %s / %lx, ", devEuiStr, mote->dev_addr);
            parse_uplink(mote, rx_size);
            filtered_printf(mote->dev_addr, MAC, "\r\n");
        } else {
            pc_printf("mote-not-found %08lx\r\n", fhdr->DevAddr);
        }

    } else
        pc_printf(" [31m%02x mtype:%d[0m\r\n", Radio::radio.rx_buf[0], mhdr->bits.MType);


    return 0;
}

