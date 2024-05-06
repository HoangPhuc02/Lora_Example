#include "sx127x_lora.h"
#include "sx127x_fsk.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

//#include "kermit.h"

//#define FSK_PER
//#define START_EIGER_RX
//#define START_EIGER_TX
//#define START_OOK_TX_TEST

DigitalOut led1(LED1);
Serial pc(USBTX, USBRX);

uint8_t tx_cnt;
char pcbuf[64];
int pcbuf_len;

typedef enum {
    APP_NONE = 0,
    APP_CHAT
} app_e;

app_e app = APP_NONE;

#define FSK_LARGE_PKT_THRESHOLD  0x3f

bool crc32_en;  // ethcrc

/*********** cmd_ulrx()... ************/
typedef enum {
    ULRX_STATE_OFF = 0,
    ULRX_STATE_NEED_LENGTH,
    ULRX_STATE_PAYLOAD,
    ULRX_STATE_SYNC1
} ulrx_state_e;
ulrx_state_e ulrx_state = ULRX_STATE_OFF;
bool ulrx_enable;
/*********** ...cmd_ulrx() ************/

uint8_t rx_payload_idx;

/************** fsk modeReady isr... **********/
bool rx_payloadReady_int_en;  // cmd_prrx()
#define N_RX_PKTS         32
#define RX_PKT_SIZE_LIMIT      32
uint8_t rx_pkts[N_RX_PKTS][RX_PKT_SIZE_LIMIT];
uint8_t n_rx_pkts;
/************** ...fsk modeReady isr **********/

#ifdef TARGET_STM
CRC_HandleTypeDef   CrcHandle;
#endif /* TARGET_STM */

int rssi_polling_thresh; // 0 = polling off
bool ook_test_en;
bool poll_irq_en;
volatile RegIrqFlags2_t fsk_RegIrqFlags2_prev;
volatile RegIrqFlags1_t fsk_RegIrqFlags1_prev;
Timer rx_start_timer;
uint32_t secs_rx_start;

/***************************** eiger per: *************************************************/

uint32_t num_cads;
bool cadper_enable;
bool per_en;
float per_tx_delay = 0.1;
int per_id;
uint32_t PacketTxCnt, PacketTxCntEnd;
uint32_t PacketPerOkCnt;
int PacketRxSequencePrev;
uint32_t PacketPerKoCnt;
uint32_t PacketNormalCnt;
Timeout per_timeout;



/******************************************************************************/
#ifdef TARGET_MTS_MDOT_F411RE
//                mosi,      miso,     sclk,       cs,        rst,      dio0,      dio1
//SX127x radio(LORA_MOSI, LORA_MISO, LORA_SCK, LORA_NSS, LORA_RESET, LORA_DIO0, LORA_DIO1);
SPI spi(LORA_MOSI, LORA_MISO, LORA_SCK); // mosi, miso, sclk
//           dio0, dio1, nss, spi, rst
SX127x radio(LORA_DIO0, LORA_DIO1, LORA_NSS, spi, LORA_RESET); // multitech mdot

DigitalIn dio3(LORA_DIO3);
DigitalOut txctl(LORA_TXCTL);
DigitalOut rxctl(LORA_RXCTL);

void rfsw_callback()
{
    /* SKY13350 */
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {  // start of transmission
        txctl = 1;
        rxctl = 0;
    } else { // reception:
        txctl = 0;
        rxctl = 1;
    }
}

#define FSK_RSSI_OFFSET         0
#define FSK_RSSI_SMOOTHING      2
DigitalIn dio2(LORA_DIO2);
DigitalIn dio4(LORA_DIO4);

#elif defined(TARGET_DISCO_L072CZ_LRWAN1)    /********************* ...mDot **********************/ 
    /* Murata TypeABZ discovery board B-L072Z-LRWAN1 */
    //           mosi, miso, sclk,   cs,  rst,  dio0, dio1
    //SX127x radio(PA_7, PA_6, PB_3, PA_15, PC_0, PB_4, PB_1);

    SPI spi(PA_7, PA_6, PB_3); // mosi, miso, sclk
    //           dio0, dio1,  nss,  spi,  rst
    SX127x radio(PB_4, PB_1, PA_15, spi, PC_0); // sx1276 arduino shield

    DigitalIn dio2(PB_0);
    DigitalIn dio3(PC_13);
    #define FSK_RSSI_OFFSET         0
    #define FSK_RSSI_SMOOTHING      2
    #define CRF1    PA_1
    #define CRF2    PC_2
    #define CRF3    PC_1
    DigitalOut Vctl1(CRF1);
    DigitalOut Vctl2(CRF2);
    DigitalOut Vctl3(CRF3);
    
    void rfsw_callback()
    {
        if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
            Vctl1 = 0;        
            if (radio.RegPaConfig.bits.PaSelect) {
                Vctl2 = 0;
                Vctl3 = 1;                        
            } else {
                Vctl2 = 1;
                Vctl3 = 0;            
            }
        } else {
            if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE)
                Vctl1 = 1;
            else
                Vctl1 = 0;
            
            Vctl2 = 0;
            Vctl3 = 0;        
        }
    }    
#else /***************** ..Type-ABZ and L073RZ *************/
SPI spi(D11, D12, D13); // mosi, miso, sclk
//           dio0, dio1, nss, spi, rst
SX127x radio(  D2,   D3, D10, spi, A0); // sx1276 arduino shield

// for SX1276 arduino shield:
#ifdef TARGET_LPC11U6X
DigitalInOut rfsw(P0_23);
#else
DigitalInOut rfsw(A4);
#endif

InterruptIn dio0int(D2);
InterruptIn dio1int(D3);
InterruptIn dio2int(D4);
InterruptIn dio4int(D8);
DigitalIn dio2(D4);
DigitalIn dio3(D5);
DigitalIn dio4(D8);
DigitalIn dio5(D9);

#if defined(TARGET_STM)
DigitalOut pc3(PC_3);   // nucleo corner pin for misc indication
#endif

void rfsw_callback()
{
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER)
        rfsw = 1;
    else
        rfsw = 0;
}

#define FSK_RSSI_OFFSET         5
#define FSK_RSSI_SMOOTHING      2

typedef enum {
    SHIELD_TYPE_NONE = 0,
    SHIELD_TYPE_LAS,
    SHIELD_TYPE_MAS,
} shield_type_e;
shield_type_e shield_type;

#endif /* !TARGET_MTS_MDOT_F411RE */

SX127x_fsk fsk(radio);
SX127x_lora lora(radio);
//Kermit kermit(lora);

#ifndef TARGET_DISCO_L072CZ_LRWAN1
volatile bool saved_dio4;
#endif

uint32_t crcTable[256];
void make_crc_table()
{
    const uint32_t POLYNOMIAL = 0xEDB88320;
    uint32_t remainder;
    uint8_t b = 0;
    do{
        // Start with the data byte
        remainder = b;
        for (unsigned long bit = 8; bit > 0; --bit)
        {
            if (remainder & 1)
                remainder = (remainder >> 1) ^ POLYNOMIAL;
            else
                remainder = (remainder >> 1);
        }
        crcTable[(size_t)b] = remainder;
    } while(0 != ++b);
}

uint32_t gen_crc(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xffffffff;
    size_t i;
    for(i = 0; i < n; i++) {
        crc = crcTable[*p++ ^ (crc&0xff)] ^ (crc>>8);
    }
        
    return(~crc);
}


void printLoraIrqs_(bool clear)
{
    //in radio class -- RegIrqFlags_t RegIrqFlags;

    //already read RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    printf("\r\nIrqFlags:");
    if (lora.RegIrqFlags.bits.CadDetected)
        printf("CadDetected ");
    if (lora.RegIrqFlags.bits.FhssChangeChannel) {
        //radio.RegHopChannel.octet = radio.read_reg(REG_LR_HOPCHANNEL);
        printf("FhssChangeChannel:%d ", lora.RegHopChannel.bits.FhssPresentChannel);
    }
    if (lora.RegIrqFlags.bits.CadDone)
        printf("CadDone ");
    if (lora.RegIrqFlags.bits.TxDone)
        printf("TxDone ");
    if (lora.RegIrqFlags.bits.ValidHeader)
        printf("[42mValidHeader[0m ");
    if (lora.RegIrqFlags.bits.PayloadCrcError)
        printf("[41mPayloadCrcError[0m ");
    if (lora.RegIrqFlags.bits.RxDone)
        printf("[42mRxDone[0m ");  
    if (lora.RegIrqFlags.bits.RxTimeout)
        printf("RxTimeout ");

    printf("\r\n");

    if (clear)
        radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);

}

void lora_printCodingRate(bool from_rx)
{
    uint8_t d = lora.getCodingRate(from_rx);
    printf("CodingRate:");
    switch (d) {
        case 1: printf("4/5 "); break;
        case 2: printf("4/6 "); break;
        case 3: printf("4/7 "); break;
        case 4: printf("4/8 "); break;
        default:
            printf("%d ", d);
            break;
    }
}

void lora_printHeaderMode()
{
    if (lora.getHeaderMode())
        printf("implicit ");
    else
        printf("explicit ");
}

void lora_printBw()
{
    (void)lora.getBw();
    
    printf("Bw:");
    if (radio.type == SX1276) {
        switch (lora.RegModemConfig.sx1276bits.Bw) {
            case 0: printf("7.8KHz "); break;
            case 1: printf("10.4KHz "); break;
            case 2: printf("15.6KHz "); break;
            case 3: printf("20.8KHz "); break;
            case 4: printf("31.25KHz "); break;
            case 5: printf("41.7KHz "); break;
            case 6: printf("62.5KHz "); break;
            case 7: printf("125KHz "); break;
            case 8: printf("250KHz "); break;
            case 9: printf("500KHz "); break;
            default: printf("%x ", lora.RegModemConfig.sx1276bits.Bw); break;
        }
    } else if (radio.type == SX1272) {
        switch (lora.RegModemConfig.sx1272bits.Bw) {
            case 0: printf("125KHz "); break;
            case 1: printf("250KHz "); break;
            case 2: printf("500KHz "); break;
            case 3: printf("11b "); break;
        }
    }
}

void lora_printAllBw()
{
    int i, s;
    
    if (radio.type == SX1276) {
        s = lora.RegModemConfig.sx1276bits.Bw;    
        for (i = 0; i < 10; i++ ) {
            lora.RegModemConfig.sx1276bits.Bw = i;
            printf("%d ", i);
            lora_printBw();
            printf("\r\n");
        }
        lora.RegModemConfig.sx1276bits.Bw = s;
    } else if (radio.type == SX1272) {
        s = lora.RegModemConfig.sx1272bits.Bw;    
        for (i = 0; i < 3; i++ ) {
            lora.RegModemConfig.sx1272bits.Bw = i;
            printf("%d ", i);
            lora_printBw();
            printf("\r\n");
        }
        lora.RegModemConfig.sx1272bits.Bw = s;    
    }
}

void lora_printSf()
{
    // spreading factor same between sx127[26]
    printf("sf:%d ", lora.getSf());
}

void lora_printRxPayloadCrcOn()
{
    bool on = lora.getRxPayloadCrcOn();
    printf("RxPayloadCrcOn:%d = ", on);
    if (lora.getHeaderMode())
        printf("Rx/");  // implicit mode
        
    if (on)
        printf("Tx CRC Enabled\r\n");
    else
        printf("Tx CRC disabled\r\n");
}

void lora_printTxContinuousMode()
{
    printf("TxContinuousMode:%d ", lora.RegModemConfig2.sx1276bits.TxContinuousMode);    // same for sx1272 and sx1276
}

void lora_printAgcAutoOn()
{
    printf("AgcAutoOn:%d", lora.getAgcAutoOn());
}

void lora_print_dio()
{
    radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    printf("DIO5:");
    switch (radio.RegDioMapping2.bits.Dio5Mapping) {
        case 0: printf("ModeReady"); break;
        case 1: printf("ClkOut"); break;
        case 2: printf("ClkOut"); break;
    }
    printf(" DIO4:");
    switch (radio.RegDioMapping2.bits.Dio4Mapping) {
        case 0: printf("CadDetected"); break;
        case 1: printf("PllLock"); break;
        case 2: printf("PllLock"); break;
    }    
    radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
    printf(" DIO3:");
    switch (radio.RegDioMapping1.bits.Dio3Mapping) {
        case 0: printf("CadDone"); break;
        case 1: printf("ValidHeader"); break;
        case 2: printf("PayloadCrcError"); break;
    }    
    printf(" DIO2:");
    switch (radio.RegDioMapping1.bits.Dio2Mapping) {
        case 0:
        case 1:
        case 2:
            printf("FhssChangeChannel");
            break;
    }    
    printf(" DIO1:");
    switch (radio.RegDioMapping1.bits.Dio1Mapping) {
        case 0: printf("RxTimeout"); break;
        case 1: printf("FhssChangeChannel"); break;
        case 2: printf("CadDetected"); break;
    }    
    printf(" DIO0:");
    switch (radio.RegDioMapping1.bits.Dio0Mapping) {
        case 0: printf("RxDone"); break;
        case 1: printf("TxDone"); break;
        case 2: printf("CadDone"); break;
    }    
    
    printf("\r\n"); 
}

void fsk_print_dio()
{
    radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    
    printf("DIO5:");
    switch (radio.RegDioMapping2.bits.Dio5Mapping) {
        case 0: printf("ClkOut"); break;
        case 1: printf("PllLock"); break;
        case 2:
            if (fsk.RegPktConfig2.bits.DataModePacket)
                printf("data");
            else {
                if (radio.RegDioMapping2.bits.MapPreambleDetect)
                    printf("preamble");
                else
                    printf("rssi");
            }
            break;
        case 3: printf("ModeReady"); break;
    }
    
    printf(" DIO4:");
    switch (radio.RegDioMapping2.bits.Dio4Mapping) {
        case 0: printf("temp/eol"); break;
        case 1: printf("PllLock"); break;
        case 2: printf("TimeOut"); break;
        case 3:
            if (fsk.RegPktConfig2.bits.DataModePacket) {
                if (radio.RegDioMapping2.bits.MapPreambleDetect)
                    printf("preamble");
                else
                    printf("rssi");
            } else
                printf("ModeReady");
            break;
    }
    
    radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
    
    printf(" DIO3:");
    if (fsk.RegPktConfig2.bits.DataModePacket) {
        if (radio.RegDioMapping1.bits.Dio3Mapping == 1)
            printf("TxReady");
        else
            printf("FifoEmpty");
    } else {
        switch (radio.RegDioMapping1.bits.Dio3Mapping) {
            case 0: printf("Timeout"); break;
            case 1:
                if (radio.RegDioMapping2.bits.MapPreambleDetect)
                    printf("preamble");
                else
                    printf("rssi");
                break;
            case 2: printf("?automode_status?"); break;
            case 3: printf("TempChange/LowBat"); break;
        }
    }
    
    printf(" DIO2:");
    if (fsk.RegPktConfig2.bits.DataModePacket) {
        switch (radio.RegDioMapping1.bits.Dio2Mapping) {
            case 0: printf("FifoFull"); break;
            case 1: printf("RxReady"); break;
            case 2: printf("FifoFull/rx-timeout"); break;
            case 3: printf("FifoFull/rx-syncadrs"); break;
        }
    } else {
        printf("Data");
    }
    
    printf(" DIO1:");
    if (fsk.RegPktConfig2.bits.DataModePacket) {
        switch (radio.RegDioMapping1.bits.Dio1Mapping) {
            case 0: printf("FifoThresh"); break;
            case 1: printf("FifoEmpty"); break;
            case 2: printf("FifoFull"); break;
            case 3: printf("[41m-3-[0m"); break;
        }
    } else {
        switch (radio.RegDioMapping1.bits.Dio1Mapping) {
            case 0: printf("Dclk"); break;
            case 1:
                if (radio.RegDioMapping2.bits.MapPreambleDetect)
                    printf("preamble");
                else
                    printf("rssi");
                break;
            case 2: printf("[41m-2-[0m"); break;
            case 3: printf("[41m-3-[0m"); break;
        }
    }
    
    printf(" DIO0:");
    if (fsk.RegPktConfig2.bits.DataModePacket) {
        switch (radio.RegDioMapping1.bits.Dio0Mapping) {
            case 0: printf("PayloadReady/PacketSent"); break;
            case 1: printf("CrcOk"); break;
            case 2: printf("[41m-2-[0m"); break;
            case 3: printf("TempChange/LowBat"); break;
        }
    } else {
        switch (radio.RegDioMapping1.bits.Dio0Mapping) {
            case 0: printf("SyncAdrs/TxReady"); break;
            case 1:
                if (radio.RegDioMapping2.bits.MapPreambleDetect)
                    printf("preamble");
                else
                    printf("rssi");
                break;
            case 2: printf("RxReady"); break;
            case 3: printf("[41m-3-[0m"); break;
        }
    }
    printf("\r\n"); 
}

void lora_print_status()
{    
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (!radio.RegOpMode.bits.LongRangeMode) {
        printf("FSK\r\n");
        return;
    }
    
    lora_print_dio();
    printf("LoRa ");
    
    // printing LoRa registers at 0x0d -> 0x3f

    lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
    lora.RegModemConfig2.octet = radio.read_reg(REG_LR_MODEMCONFIG2);

    lora_printCodingRate(false); // false: transmitted coding rate
    lora_printHeaderMode();
    lora_printBw();
    lora_printSf();
    lora_printRxPayloadCrcOn();
    // RegModemStat
    printf("ModemStat:0x%02x\r\n", radio.read_reg(REG_LR_MODEMSTAT));

    // fifo ptrs:
    lora.RegPayloadLength = radio.read_reg(REG_LR_PAYLOADLENGTH);
    lora.RegRxMaxPayloadLength = radio.read_reg(REG_LR_RX_MAX_PAYLOADLENGTH);
    printf("fifoptr=0x%02x txbase=0x%02x rxbase=0x%02x payloadLength=0x%02x maxlen=0x%02x",
        radio.read_reg(REG_LR_FIFOADDRPTR),
        radio.read_reg(REG_LR_FIFOTXBASEADDR),
        radio.read_reg(REG_LR_FIFORXBASEADDR),
        lora.RegPayloadLength,
        lora.RegRxMaxPayloadLength
    );

    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    printLoraIrqs_(false);

    lora.RegHopPeriod = radio.read_reg(REG_LR_HOPPERIOD);
    if (lora.RegHopPeriod != 0) {
        printf("\r\nHopPeriod:0x%02x\r\n", lora.RegHopPeriod);
    }

    printf("SymbTimeout:%d ", radio.read_u16(REG_LR_MODEMCONFIG2) & 0x3ff);

    lora.RegPreamble = radio.read_u16(REG_LR_PREAMBLEMSB);
    printf("PreambleLength:%d ", lora.RegPreamble);

    if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE) {
        printf("rssi:%ddBm ", lora.get_current_rssi());
    }

    lora_printTxContinuousMode();

    printf("\r\n");
    lora_printAgcAutoOn();
    if (radio.type == SX1272) {
        printf(" LowDataRateOptimize:%d\r\n", lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    }

    printf("\r\nHeaderCount:%d PacketCount:%d, ",
        radio.read_u16(REG_LR_RXHEADERCNTVALUE_MSB), radio.read_u16(REG_LR_RXPACKETCNTVALUE_MSB));

    printf("Lora detection threshold:%02x\r\n", radio.read_reg(REG_LR_DETECTION_THRESHOLD));
    lora.RegTest31.octet = radio.read_reg(REG_LR_TEST31);
    printf("detect_trig_same_peaks_nb:%d\r\n", lora.RegTest31.bits.detect_trig_same_peaks_nb);

    if (radio.type == SX1272) {
        lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
        printf("LowDataRateOptimize:%d ", lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    } else if (radio.type == SX1276) {
        lora.RegModemConfig3.octet = radio.read_reg(REG_LR_MODEMCONFIG3);
        printf("LowDataRateOptimize:%d ", lora.RegModemConfig3.sx1276bits.LowDataRateOptimize);        
    }
    
    printf(" invert: rx=%d tx=%d\r\n", lora.RegTest33.bits.invert_i_q, !lora.RegTest33.bits.chirp_invert_tx);
    
    printf("\r\n");
    //printf("A %02x\r\n", radio.RegModemConfig2.octet);
}

uint16_t
fsk_get_PayloadLength(void)
{
    fsk.RegPktConfig2.word = radio.read_u16(REG_FSK_PACKETCONFIG2);

    return fsk.RegPktConfig2.bits.PayloadLength;
}

void fsk_printAddressFiltering()
{
    uint8_t FSKRegNodeAdrs, FSKRegBroadcastAdrs;
    
    printf(" AddressFiltering:");
    switch (fsk.RegPktConfig1.bits.AddressFiltering) {
        case 0: printf("off"); break;
        case 1: // NodeAddress
            FSKRegNodeAdrs = radio.read_reg(REG_FSK_NODEADRS);
            printf("NodeAdrs:%02x\r\n", FSKRegNodeAdrs);
            break;
        case 2: // NodeAddress & BroadcastAddress
            FSKRegNodeAdrs = radio.read_reg(REG_FSK_NODEADRS);
            printf("NodeAdrs:%02x ", FSKRegNodeAdrs);
            FSKRegBroadcastAdrs = radio.read_reg(REG_FSK_BROADCASTADRS);
            printf("BroadcastAdrs:%02x\r\n", FSKRegBroadcastAdrs );
            break;
        default:
            printf("%d", fsk.RegPktConfig1.bits.AddressFiltering);
            break;
    }
}

void fsk_print_IrqFlags2()
{
    RegIrqFlags2_t RegIrqFlags2;
    
    printf("IrqFlags2: ");
    RegIrqFlags2.octet = radio.read_reg(REG_FSK_IRQFLAGS2);
    if (RegIrqFlags2.bits.FifoFull)
        printf("FifoFull ");
    if (RegIrqFlags2.bits.FifoEmpty)
        printf("FifoEmpty ");
    if (RegIrqFlags2.bits.FifoLevel)
        printf("FifoLevel ");
    if (RegIrqFlags2.bits.FifoOverrun)
        printf("FifoOverrun ");
    if (RegIrqFlags2.bits.PacketSent)
        printf("PacketSent ");
    if (RegIrqFlags2.bits.PayloadReady)
        printf("PayloadReady ");
    if (RegIrqFlags2.bits.CrcOk)
        printf("CrcOk ");
    if (RegIrqFlags2.bits.LowBat)
        printf("LowBat ");
    printf("\r\n");
}

void
fsk_print_status()
{
    //uint16_t s;
    RegIrqFlags1_t RegIrqFlags1;
    
    if (radio.RegOpMode.bits.LongRangeMode) {
        printf("LoRa\r\n");
        return;
    }
    
    if (radio.RegOpMode.bits.ModulationType == 0) {
        printf("FSK ");
        switch (radio.RegOpMode.bits.ModulationShaping) {
            case 1: printf("BT1.0 "); break;
            case 2: printf("BT0.5 "); break;
            case 3: printf("BT0.3 "); break;
        }
    } else if (radio.RegOpMode.bits.ModulationType == 1) {
        printf("OOK ");
        switch (radio.RegOpMode.bits.ModulationShaping) {
            case 1: printf("Fcutoff=bitrate"); break;
            case 2: printf("Fcutoff=2*bitrate"); break;
            case 3: printf("?"); break;
        }        
    }

    printf("%" PRIu32 "bps fdev:%" PRIu32 "Hz\r\n", fsk.get_bitrate(), fsk.get_tx_fdev_hz());    
    
    fsk.RegPktConfig2.word = radio.read_u16(REG_FSK_PACKETCONFIG2);
    
    fsk_print_dio();
    
    printf("rxbw:%" PRIu32 "Hz ", fsk.get_rx_bw_hz(REG_FSK_RXBW));
    printf("afcbw:%" PRIu32 "Hz\r\n", fsk.get_rx_bw_hz(REG_FSK_AFCBW));

    fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);
    printf("RssiOffset:%ddB smoothing:%dsamples\r\n", fsk.RegRssiConfig.bits.RssiOffset, 1 << (fsk.RegRssiConfig.bits.RssiSmoothing+1));


    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);

    if (fsk.RegPktConfig2.bits.DataModePacket) {
        uint16_t len;
        /* packet mode */
        len = fsk_get_PayloadLength();
        printf("packet RegPayloadLength:0x%03x ", len);

        if (fsk.RegPktConfig2.bits.BeaconOn)
            printf("BeaconOn ");

        fsk.RegFifoThreshold.octet = radio.read_reg(REG_FSK_FIFOTHRESH);
        printf("FifoThreshold:%d TxStartCondition:", fsk.RegFifoThreshold.bits.FifoThreshold);
        if (fsk.RegFifoThreshold.bits.TxStartCondition)
            printf("!FifoEmpty");
        else
            printf("FifoLevel");

        printf("\r\nAutoRestartRxMode:");
        switch (fsk.RegSyncConfig.bits.AutoRestartRxMode) {
            case 0: printf("off "); break;
            case 1: printf("no-pll-wait "); break;
            case 2: printf("pll-wait "); break;
            case 3: printf("3 "); break;
        }
        //...todo

        printf("PreambleSize:%d ", radio.read_u16(REG_FSK_PREAMBLEMSB));

        fsk.RegOokPeak.octet = radio.read_reg(REG_FSK_OOKPEAK);
        if (fsk.RegOokPeak.bits.barker_en)
            printf("barker ");
        if (!fsk.RegOokPeak.bits.BitSyncOn)
            printf("BitSyncOff ");
        //...todo

        fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
        if (fsk.RegPktConfig1.bits.PacketFormatVariable)
            printf("variable");
        else
            printf("fixed");
        printf("-length\r\ncrc");
        if (fsk.RegPktConfig1.bits.CrcOn) {
            printf("On");
        } else
            printf("Off");
        printf(" crctype:");
        if (fsk.RegPktConfig1.bits.CrCWhiteningType)
            printf("IBM");
        else
            printf("CCITT");
        printf(" dcFree:");
        switch (fsk.RegPktConfig1.bits.DcFree) {
            case 0: printf("none "); break;
            case 1: printf("Manchester "); break;
            case 2: printf("Whitening "); break;
            case 3: printf("[41mreserved[0m "); break;
        }
        fsk_printAddressFiltering();

        printf("\r\n");
        fsk_print_IrqFlags2();
    } else {
        /* continuous mode */
        printf("[7mcontinuous[27m ");
    }

    fsk.RegPreambleDetect.octet = radio.read_reg(REG_FSK_PREAMBLEDETECT);
    printf("PreambleDetect:");
    if (fsk.RegPreambleDetect.bits.PreambleDetectorOn) {
        printf("size=%d,tol=%d ",
            fsk.RegPreambleDetect.bits.PreambleDetectorSize,
            fsk.RegPreambleDetect.bits.PreambleDetectorTol);
    } else
        printf("Off ");

    if (fsk.RegSyncConfig.bits.SyncOn) {
        printf(" syncsize:%d ", fsk.RegSyncConfig.bits.SyncSize);
        printf(" : %02x ", radio.read_reg(REG_FSK_SYNCVALUE1));
        printf("%02x ", radio.read_reg(REG_FSK_SYNCVALUE2));
        printf("%02x ", radio.read_reg(REG_FSK_SYNCVALUE3));
        printf("%02x ", radio.read_reg(REG_FSK_SYNCVALUE4));
    } else
        printf("Sync Off");
    printf("\r\n");   // end sync config

    fsk.RegAfcFei.octet = radio.read_reg(REG_FSK_AFCFEI);
    printf("afcAutoClear:");
    if (fsk.RegAfcFei.bits.AfcAutoClearOn)
        printf("On");
    else
        printf("OFF");
    printf(" afc:%dHz ", (int)(FREQ_STEP_HZ * radio.read_s16(REG_FSK_AFCMSB)));

    printf("fei:%dHz\r\n", (int)(FREQ_STEP_HZ * radio.read_s16(REG_FSK_FEIMSB)));

    fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);
    printf("RxTrigger:");
    switch (fsk.RegRxConfig.bits.RxTrigger) {
        case 0: printf("none "); break;
        case 1: printf("rssi "); break;
        case 6: printf("preamble "); break;
        case 7: printf("both "); break;
        default: printf("-%d- ", fsk.RegRxConfig.bits.RxTrigger); break;
    }
    printf("AfcAuto:");
    if (fsk.RegRxConfig.bits.AfcAutoOn)
        printf("On ");
    else
        printf("OFF ");
        
    radio.RegLna.octet = radio.read_reg(REG_LNA);
    if (!fsk.RegRxConfig.bits.AgcAutoOn) {
        printf("AgcAutoOff:G%d ", radio.RegLna.bits.LnaGain);
    }
    printf("LnaBoostHF:%d ", radio.RegLna.bits.LnaBoostHF);

    fsk.RegTimerResol.octet = radio.read_reg(REG_FSK_TIMERRESOL);
    if (fsk.RegTimerResol.bits.hlm_started)
        printf("[35mhlm_started[0m ");
    else
        printf("hlm_stopped ");

    fsk.RegRssiThresh = radio.read_reg(REG_FSK_RSSITHRESH);
    printf("rssiThreshold:-%.1f@%02x ", fsk.RegRssiThresh / 2.0, REG_FSK_RSSITHRESH);

    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER ||
        radio.RegOpMode.bits.Mode == RF_OPMODE_SYNTHESIZER_RX)
    {
        printf("rssi:-%.1f ", radio.read_reg(REG_FSK_RSSIVALUE) / 2.0);
    }

    fsk.RegSeqConfig1.octet = radio.read_reg(REG_FSK_SEQCONFIG1);
    printf("\r\nsequencer: ");
    printf("FromStart:");
    switch (fsk.RegSeqConfig1.bits.FromStart) {
        case 0:
            printf("lowPowerSelection-");
            if (fsk.RegSeqConfig1.bits.LowPowerSelection)
                printf("idle");
            else
                printf("sequencerOff");
            break;
        case 1: printf("rx"); break;
        case 2: printf("tx"); break;
        case 3: printf("tx on fifolevel"); break;
    }
    printf(" lowPowerSelection:");
    if (fsk.RegSeqConfig1.bits.LowPowerSelection)
        printf("idle");
    else
        printf("SequencerOff");
    if (fsk.RegSeqConfig1.bits.FromStart != 0 && 
        fsk.RegSeqConfig1.bits.LowPowerSelection != 0)
    {   // if sequencer enabled:
        printf("\r\nsequencer: IdleMode:");
        if (fsk.RegSeqConfig1.bits.IdleMode)
            printf("Sleep");
        else
            printf("standby");
        printf("\r\nsequencer: FromIdle to:");
        if (fsk.RegSeqConfig1.bits.FromIdle)
            printf("rx");
        else
            printf("tx");
        printf("\r\nsequencer: FromTransmit to:");
        if (fsk.RegSeqConfig1.bits.FromTransmit)
            printf("rx-on-PacketSent");
        else {
            printf("lowPowerSelection-");
            if (fsk.RegSeqConfig1.bits.LowPowerSelection)
                printf("idle");
            else
                printf("SequencerOff");
            printf("-on-PacketSent");
        }
        fsk.RegSeqConfig2.octet = radio.read_reg(REG_FSK_SEQCONFIG2);
        printf("\r\nsequencer: FromReceive:");
        switch (fsk.RegSeqConfig2.bits.FromReceive) {
            case 1: printf("PacketRecevied on PayloadReady"); break;
            case 2: 
                printf("lowPowerSelection-");
                if (fsk.RegSeqConfig1.bits.LowPowerSelection)
                    printf("idle");
                else
                    printf("SequencerOff");
                printf("-on-payloadReady");
                break;
            case 3: printf("PacketRecevied-on-CrcOk"); break;
            case 4: printf("SequencerOff-on-Rssi"); break;
            case 5: printf("SequencerOff-on-SyncAddress"); break;
            case 6: printf("SequencerOff-PreambleDetect"); break;
            default: printf("-%d-", fsk.RegSeqConfig2.bits.FromReceive); break;
        }
        printf("\r\nsequencer: FromRxTimeout:");
        switch (fsk.RegSeqConfig2.bits.FromRxTimeout) {
            case 0: printf("rx"); break;
            case 1: printf("tx"); break;
            case 2:
                printf("lowPowerSelection-");
                if (fsk.RegSeqConfig1.bits.LowPowerSelection)
                    printf("idle");
                else
                    printf("SequencerOff");
                break;
            case 3: printf("SequencerOff"); break;
        }
        printf("\r\nsequencer: FromPacketReceived to:");
        switch (fsk.RegSeqConfig2.bits.FromPacketReceived) {
            case 0: printf("SequencerOff"); break;
            case 1: printf("tx on FifoEmpty"); break;
            case 2:
                printf("lowPowerSelection-");
                if (fsk.RegSeqConfig1.bits.LowPowerSelection)
                printf("idle");
                else
                printf("sequencerOff");
                break;
            case 3: printf("rx via fs"); break;
            case 4: printf("rx"); break;
        }

        fsk.RegTimerResol.octet = radio.read_reg(REG_FSK_TIMERRESOL);
        printf("\r\nsequencer: timer1:");
        switch (fsk.RegTimerResol.bits.timer1_resol) {
            case 0: printf("off"); break;
            case 1: printf("%dus", radio.read_reg(REG_FSK_TIMER1COEF) * 64); break;
            case 2: printf("%.1fms", radio.read_reg(REG_FSK_TIMER1COEF) * 4.1); break;
            case 3: printf("%.1fs", radio.read_reg(REG_FSK_TIMER1COEF) * 0.262); break;
        }

        printf(" timer2:");
        switch (fsk.RegTimerResol.bits.timer2_resol) {
            case 0: printf("off"); break;
            case 1: printf("%dus", radio.read_reg(REG_FSK_TIMER2COEF) * 64); break;
            case 2: printf("%.1fms", radio.read_reg(REG_FSK_TIMER2COEF) * 4.1); break;
            case 3: printf("%.1fs", radio.read_reg(REG_FSK_TIMER2COEF) * 0.262); break;
        }
    } // ..if sequencer enabled

    printf("\r\nIrqFlags1:");
    RegIrqFlags1.octet = radio.read_reg(REG_FSK_IRQFLAGS1);
    if (RegIrqFlags1.bits.ModeReady)
        printf("ModeReady ");
    if (RegIrqFlags1.bits.RxReady)
        printf("RxReady ");
    if (RegIrqFlags1.bits.TxReady)
        printf("TxReady ");
    if (RegIrqFlags1.bits.PllLock)
        printf("PllLock ");
    if (RegIrqFlags1.bits.Rssi)
        printf("Rssi ");
    if (RegIrqFlags1.bits.Timeout)
        printf("Timeout ");
    if (RegIrqFlags1.bits.PreambleDetect)
        printf("PreambleDetect ");
    if (RegIrqFlags1.bits.SyncAddressMatch)
        printf("SyncAddressMatch ");

    printf("\r\n");

/* TODO    if (!SX1272FSK->RegPktConfig1.bits.PacketFormatVariable) { // if fixed-length packet format:
        s = fsk_get_PayloadLength();
        if (s > FSK_LARGE_PKT_THRESHOLD)
            flags.fifo_flow_ctl = 1;
        else
            flags.fifo_flow_ctl = 0;
    }*/

    fsk.RegImageCal.octet = radio.read_reg(REG_FSK_IMAGECAL);
    if (fsk.RegImageCal.bits.TempMonitorOff) {
        printf("[42mTempMonitorOff[\r0m\n");
    } else {
        printf("TempThreshold:");
        switch (fsk.RegImageCal.bits.TempThreshold) {
            case 0: printf("5C"); break;
            case 1: printf("10C"); break;
            case 2: printf("15C"); break;
            case 3: printf("20C"); break;
        }
        printf("\r\n");
    }
    if (fsk.RegImageCal.bits.ImageCalRunning)
        printf("[33mImageCalRunning[\r0m\n");
    
    if (rx_payloadReady_int_en) {
#ifdef TARGET_DISCO_L072CZ_LRWAN1
        printf("n_rx_pkts:%u, dio:%u,%u,%u,%u\r\n", n_rx_pkts, dio3.read(), dio2.read(), radio.dio1.read(), radio.dio0.read());
#else
        printf("n_rx_pkts:%u, dio:%u,%u,%u,%u,%u\r\n", n_rx_pkts, dio4.read(), dio3.read(), dio2.read(), radio.dio1.read(), radio.dio0.read());
#endif
    }
}

void printOpMode()
{
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    switch (radio.RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: printf("[7msleep[0m"); break;
        case RF_OPMODE_STANDBY: printf("[7mstby[0m"); break;
        case RF_OPMODE_SYNTHESIZER_TX: printf("[33mfstx[0m"); break;
        case RF_OPMODE_TRANSMITTER: printf("[31mtx[0m"); break;
        case RF_OPMODE_SYNTHESIZER_RX: printf("[33mfsrx[0m"); break;
        case RF_OPMODE_RECEIVER: printf("[32mrx[0m"); break;
        case 6:
            if (radio.RegOpMode.bits.LongRangeMode)
                printf("[42mrxs[0m");
            else
                printf("-6-");
            break;  // todo: different lora/fsk
        case 7:
            if (radio.RegOpMode.bits.LongRangeMode)
                printf("[45mcad[0m");
            else
                printf("-7-");
            break;  // todo: different lora/fsk
    }
}

void
printPa()
{
    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
    if (radio.RegPaConfig.bits.PaSelect) {
        float output_dBm = 17 - (15-radio.RegPaConfig.bits.OutputPower);
        printf(" PABOOST OutputPower=%.1fdBm", output_dBm);
    } else {
        float pmax = (0.6*radio.RegPaConfig.bits.MaxPower) + 10.8;
        float output_dBm = pmax - (15-radio.RegPaConfig.bits.OutputPower);
#ifdef TARGET_MTS_MDOT_F411RE
        printf(" \x1b[31mRFO pmax=%.1fdBm OutputPower=%.1fdBm\x1b[0m", pmax, output_dBm);  // not connected
#else
        printf(" RFO pmax=%.1fdBm OutputPower=%.1fdBm", pmax, output_dBm);
#endif
    }
}

void /* things always present, whether lora or fsk */
common_print_status()
{
    printf("version:0x%02x %.3fMHz ", radio.read_reg(REG_VERSION), radio.get_frf_MHz());
    printOpMode();

    printPa();

    radio.RegOcp.octet = radio.read_reg(REG_OCP);
    if (radio.RegOcp.bits.OcpOn) {
        int imax = 0;
        if (radio.RegOcp.bits.OcpTrim < 16)
            imax = 45 + (5 * radio.RegOcp.bits.OcpTrim);
        else if (radio.RegOcp.bits.OcpTrim < 28)
            imax = -30 + (10 * radio.RegOcp.bits.OcpTrim);
        else
            imax = 240;
        printf(" OcpOn %dmA ", imax);
    } else
        printf(" OcpOFF ");

    printf("\r\n");
    
    if (per_en) {
        if (cadper_enable) {
            printf("cadper %" PRIu32 ", ", num_cads);
        }
        printf("per_tx_delay:%f\r\n", per_tx_delay);
        printf("PER device ID:%d\r\n", per_id);
    }    
    
    if (poll_irq_en) {
        printf("poll_irq_en\r\n");
        if (!radio.RegOpMode.bits.LongRangeMode) {
            printf("saved irqs: %02x %02x\r\n", fsk_RegIrqFlags1_prev.octet, fsk_RegIrqFlags2_prev.octet);
        }
    }

}

void print_rx_buf(int len)
{
    int i;

    printf("000:");
    for (i = 0; i < len; i++) {
        //printf("(%d)%02x ", i % 16, rx_buf[i]);
        printf("%02x ", radio.rx_buf[i]);
        if (i % 16 == 15 && i != len-1)
            printf("\r\n%03d:", i+1);

    }
    printf("\r\n");
}

void lora_print_rx_verbose(uint8_t dlen)
{
    float dbm;
    printLoraIrqs_(false);
    if (lora.RegHopPeriod > 0) {
        lora.RegHopChannel.octet = radio.read_reg(REG_LR_HOPCHANNEL);
        printf("HopCH:%d ", lora.RegHopChannel.bits.FhssPresentChannel);
    }
    printf("%dHz ", lora.get_freq_error_Hz());
    lora_printCodingRate(true);  // true: of received packet
    dbm = lora.get_pkt_rssi();
    printf(" crc%s %.1fdB  %.1fdBm\r\n",
        lora.RegHopChannel.bits.RxPayloadCrcOn ? "On" : "OFF",
        lora.RegPktSnrValue / 4.0,
        dbm
    );
    print_rx_buf(dlen);
}

void set_per_en(bool en)
{
    if (en) {
        if (radio.RegOpMode.bits.LongRangeMode) {
            if (radio.type == SX1272) {
                lora.RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
                radio.write_reg(REG_LR_MODEMCONFIG, lora.RegModemConfig.octet);
            } else if (radio.type == SX1276) {
                lora.RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
                radio.write_reg(REG_LR_MODEMCONFIG3, lora.RegModemConfig3.octet);
            }
            lora.RegPayloadLength = 9;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
            radio.RegDioMapping1.bits.Dio3Mapping = 1;  // to ValidHeader
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);                                  
        } else {    // fsk..
            //fsk_tx_length = 9;
        }
        PacketRxSequencePrev = 0; // transmitter side PacketTxCnt is 1 at first TX
        //PacketRxSequence = 0;
        PacketPerKoCnt = 0;
        PacketPerOkCnt = 0;
        PacketNormalCnt = 0;
    } // ..if (per_en)
    else {
        per_timeout.detach();
    }    
    
    per_en = en; 
}   

void per_cb()
{
    int i;
    
    PacketTxCnt++;

    radio.tx_buf[0] = per_id;
    radio.tx_buf[1] = PacketTxCnt >> 24;
    radio.tx_buf[2] = PacketTxCnt >> 16;
    radio.tx_buf[3] = PacketTxCnt >> 8;
    radio.tx_buf[4] = PacketTxCnt;
    radio.tx_buf[5] = 'P';
    radio.tx_buf[6] = 'E';
    radio.tx_buf[7] = 'R';
    radio.tx_buf[8] = 0;
    for (i = 0; i < 8; i++)
        radio.tx_buf[8] += radio.tx_buf[i];

    if (radio.RegOpMode.bits.LongRangeMode) {
        lora.start_tx(lora.RegPayloadLength);
    } else {
        fsk.start_tx(9);
    }    
    
    led1 = !led1.read();
    
    if (PacketTxCnt == PacketTxCntEnd) {
        set_per_en(false);
        return;
    }    
}

int per_parse_rx(uint8_t len)
{
    if (len > 8 && radio.rx_buf[5] == 'P' && radio.rx_buf[6] == 'E' && radio.rx_buf[7] == 'R') {
        int i;
        float per;

        /* this is PER packet */
        int PacketRxSequence = (radio.rx_buf[1] << 24) | (radio.rx_buf[2] << 16) | (radio.rx_buf[3] << 8) | radio.rx_buf[4];
        PacketPerOkCnt++;
        
        if( PacketRxSequence <= PacketRxSequencePrev )
        { // Sequence went back => resynchronization
            // dont count missed packets this time
            i = 0;
        }
        else
        {
            // determine number of missed packets
            i = PacketRxSequence - PacketRxSequencePrev - 1;
        }
        
        led1 = !led1.read();
        // be ready for the next
        PacketRxSequencePrev = PacketRxSequence;
        // increment 'missed' counter for the RX session
        PacketPerKoCnt += i;
        per = ( (float)1.0 - ( float )PacketPerOkCnt / ( float )( PacketPerOkCnt + PacketPerKoCnt ) ) * (float)100.0;
        printf("%d, ok=%" PRIu32 " missed=%" PRIu32 " normal=%" PRIu32 " per:%.3f ", PacketRxSequence, PacketPerOkCnt, PacketPerKoCnt, PacketNormalCnt, per);
        if (radio.RegOpMode.bits.LongRangeMode)
            printf("pkt:%ddBm, snr:%.1fdB, %ddBm\r\n", lora.get_pkt_rssi(), lora.RegPktSnrValue / 4.0, lora.get_current_rssi());
        else {
            wait_us(10000);
            printf(" -%.1fdBm\r\n", radio.read_reg(REG_FSK_RSSIVALUE) / 2.0); 
        }

        return 1;
    } else {
        return 0;
    }    
}

typedef enum {
    ON_TXDONE_STATE_NONE = 0,
    ON_TXDONE_STATE_SYNC_HI_NIBBLE,
    ON_TXDONE_STATE_SYNC_LO_NIBBLE,
    ON_TXDONE_STATE_PAYLOAD_LENGTH,
} on_txdone_state_e;

on_txdone_state_e on_txdone_state;

uint8_t lora_sync_byte;
float on_txdone_delay;
Timeout on_txdone_timeout;
uint8_t on_txdone_repeat_cnt;

void txdone_timeout_cb()
{
    uint8_t nib;
    
    switch (on_txdone_state) {
        case ON_TXDONE_STATE_SYNC_HI_NIBBLE:
            nib = lora_sync_byte >> 4;
            if (nib >= 15) {
                on_txdone_state = ON_TXDONE_STATE_SYNC_LO_NIBBLE;
                lora_sync_byte = 0x00;
            } else
                nib++;
                
            lora_sync_byte = nib << 4;     
            
            radio.write_reg(REG_LR_SYNC_BYTE, lora_sync_byte);
            printf("upper %02x\r\n", lora_sync_byte);               
            break;
        case ON_TXDONE_STATE_SYNC_LO_NIBBLE:
            nib = lora_sync_byte & 0x0f;
            if (nib >= 15) {
                on_txdone_state = ON_TXDONE_STATE_SYNC_LO_NIBBLE;
                lora_sync_byte = 0x00;
            } else
                nib++;
            
            lora_sync_byte = nib & 0x0f;   
            
            radio.write_reg(REG_LR_SYNC_BYTE, lora_sync_byte);
            printf("lower %02x\r\n", lora_sync_byte);                 
            break;
        case ON_TXDONE_STATE_PAYLOAD_LENGTH:
            if (++on_txdone_repeat_cnt >= 10) {
                on_txdone_repeat_cnt = 0;
                if (lora.RegPayloadLength == 255) {
                    lora.RegPayloadLength = 1;
                    printf("done\r\n");
                    on_txdone_state = ON_TXDONE_STATE_NONE;
                    return;
                }
                lora.RegPayloadLength++;
                radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
                printf("pl %d\r\n", lora.RegPayloadLength);
            }
            tx_cnt++;
            radio.tx_buf[0] = tx_cnt;     
            radio.tx_buf[1] = ~tx_cnt;              
            break;
        default:
            return;
    } // ..switch (on_txdone_state)

    lora.start_tx(lora.RegPayloadLength);
}

  
void
poll_service_radio()
{
    if (radio.RegOpMode.bits.LongRangeMode) {
    } else { // fsk:
        if (rx_payloadReady_int_en)
            return;

        /*RegIrqFlags2_t RegIrqFlags2;
        if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
            RegIrqFlags2.octet = radio.read_reg(REG_FSK_IRQFLAGS2);
            if (RegIrqFlags2.bits.PacketSent) {
                radio.set_opmode(RF_OPMODE_SLEEP);
                printf("poll mode fsk tx done\r\n");
            }
        }*/
        static uint8_t rssi;    
        RegIrqFlags2_t RegIrqFlags2;
        RegIrqFlags1_t RegIrqFlags1;
        RegIrqFlags1.octet = radio.read_reg(REG_FSK_IRQFLAGS1);
        if (RegIrqFlags1.octet != fsk_RegIrqFlags1_prev.octet) {
            printf("iF1:");
            if (RegIrqFlags1.bits.ModeReady ^ fsk_RegIrqFlags1_prev.bits.ModeReady) {
                printf("ModeReady-");
                if (RegIrqFlags1.bits.ModeReady)
                    printf("on ");
                else
                    printf("off ");
            }
            if (RegIrqFlags1.bits.RxReady ^ fsk_RegIrqFlags1_prev.bits.RxReady) {
                printf("RxReady-");
                if (RegIrqFlags1.bits.RxReady)
                    printf("on ");
                else
                    printf("off ");
            }
            if (RegIrqFlags1.bits.TxReady ^ fsk_RegIrqFlags1_prev.bits.TxReady) {
                printf("TxReady-");
                if (RegIrqFlags1.bits.TxReady)
                    printf("on ");
                else
                    printf("off ");                
            }
            if (RegIrqFlags1.bits.PllLock ^ fsk_RegIrqFlags1_prev.bits.PllLock) {
                printf("PllLock-");
                if (RegIrqFlags1.bits.PllLock)
                    printf("on ");
                else
                    printf("off ");                    
            }
            if (RegIrqFlags1.bits.Rssi ^ fsk_RegIrqFlags1_prev.bits.Rssi) {
                printf("Rssi-");
                if (RegIrqFlags1.bits.Rssi)
                    printf("on ");
                else
                    printf("off ");                   
            }
            if (RegIrqFlags1.bits.Timeout ^ fsk_RegIrqFlags1_prev.bits.Timeout) {
                printf("Timeout-");
                if (RegIrqFlags1.bits.Timeout)
                    printf("on ");
                else
                    printf("off ");                   
            }
            if (RegIrqFlags1.bits.PreambleDetect ^ fsk_RegIrqFlags1_prev.bits.PreambleDetect) {
                printf("PreambleDetect-");
                if (RegIrqFlags1.bits.PreambleDetect)
                    printf("on ");
                else
                    printf("off ");                   
            }
            if (RegIrqFlags1.bits.SyncAddressMatch ^ fsk_RegIrqFlags1_prev.bits.SyncAddressMatch) {
                printf("SyncAddressMatch-");
                if (RegIrqFlags1.bits.SyncAddressMatch)
                    printf("on ");
                else
                    printf("off ");                   
            }
            fsk_RegIrqFlags1_prev.octet = RegIrqFlags1.octet; 
            printf("\r\n");
            fflush(stdout); 
        }    
        RegIrqFlags2.octet = radio.read_reg(REG_FSK_IRQFLAGS2);
        if (RegIrqFlags2.octet != fsk_RegIrqFlags2_prev.octet) {
            printf("iF2:");
            if (RegIrqFlags2.bits.FifoFull ^ fsk_RegIrqFlags2_prev.bits.FifoFull) {
                printf("FifoFull-");
                if (RegIrqFlags2.bits.FifoFull)
                    printf("on ");
                else
                    printf("off ");                   
            }
            if (RegIrqFlags2.bits.FifoEmpty ^ fsk_RegIrqFlags2_prev.bits.FifoEmpty) {
                printf("FifoEmpty-");
                if (RegIrqFlags2.bits.FifoEmpty)
                    printf("on ");
                else {
                    printf("off ");                 
                    rssi = radio.read_reg(REG_FSK_RSSIVALUE);
                }
            }
            if (RegIrqFlags2.bits.FifoLevel ^ fsk_RegIrqFlags2_prev.bits.FifoLevel) {
                printf("FifoLevel-");
                if (RegIrqFlags2.bits.FifoLevel)
                    printf("on ");
                else
                    printf("off ");                 
            }
            if (RegIrqFlags2.bits.FifoOverrun ^ fsk_RegIrqFlags2_prev.bits.FifoOverrun) {
                printf("FifoOverrun-");
                if (RegIrqFlags2.bits.FifoOverrun)
                    printf("on ");
                else
                    printf("off ");                 
            }
            if (RegIrqFlags2.bits.PacketSent ^ fsk_RegIrqFlags2_prev.bits.PacketSent) {
                printf("PacketSent-");
                if (RegIrqFlags2.bits.PacketSent) {
                    printf("on ");
                } else
                    printf("off ");                 
            }
            if (RegIrqFlags2.bits.PayloadReady ^ fsk_RegIrqFlags2_prev.bits.PayloadReady) {
                printf("PayloadReady-");
                if (RegIrqFlags2.bits.PayloadReady)
                    printf("on ");
                else
                    printf("off ");                 
            }
            if (RegIrqFlags2.bits.CrcOk ^ fsk_RegIrqFlags2_prev.bits.CrcOk) {
                printf("CrcOk-");
                if (RegIrqFlags2.bits.CrcOk)
                    printf("on ");
                else
                    printf("off ");                 
            }
            if (RegIrqFlags2.bits.LowBat ^ fsk_RegIrqFlags2_prev.bits.LowBat) {
                printf("LowBat-");
                if (RegIrqFlags2.bits.LowBat)
                    printf("on ");
                else
                    printf("off ");                 
            }
            fsk_RegIrqFlags2_prev.octet = RegIrqFlags2.octet;
            printf("\r\n");
            fflush(stdout); 
            
            if (RegIrqFlags2.bits.PacketSent) {
                if (fsk.tx_done_sleep)
                    radio.set_opmode(RF_OPMODE_SLEEP);
                else
                    radio.set_opmode(RF_OPMODE_STANDBY);                    
            }

            if (RegIrqFlags2.bits.CrcOk || RegIrqFlags2.bits.PayloadReady) {
                if (fsk.RegRxConfig.bits.AfcAutoOn) {
                    fsk.RegAfcValue = radio.read_s16(REG_FSK_AFCMSB);      
                    printf("%dHz ", (int)(FREQ_STEP_HZ * fsk.RegAfcValue));  
                    if (rssi != 0) {
                        printf("pkt:-%.1fdBm ", rssi / 2.0);
                        rssi = 0;
                    }                                         
                }
                if (fsk.RegPktConfig1.bits.PacketFormatVariable) {
                    fsk.rx_buf_length = radio.read_reg(REG_FIFO);
                } else {
                    fsk.rx_buf_length = fsk.RegPktConfig2.bits.PayloadLength;
                }
                
                radio.m_cs = 0;
                radio.m_spi.write(REG_FIFO); // bit7 is low for reading from radio
                for (int i = 0; i < fsk.rx_buf_length; i++) {
                    radio.rx_buf[i] = radio.m_spi.write(0);
                }
                radio.m_cs = 1;                        
                /****/
                if (per_en) { 
                    if (!per_parse_rx(fsk.rx_buf_length)) {
                        PacketNormalCnt++;
                        print_rx_buf(fsk.rx_buf_length);                                                 
                    }                                       
                } else {
                    print_rx_buf(fsk.rx_buf_length);                            
                }
                fflush(stdout);                    
            } // ..if CrcOk or PayloadReady
        } // ..if RegIrqFlags2 changed
    } // ...fsk
}

void cadper_service()
{
    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    

    if (lora.RegIrqFlags.bits.CadDetected) {
        lora.start_rx(RF_OPMODE_RECEIVER_SINGLE);
        do {
            lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
            if (lora.RegIrqFlags.bits.RxDone) {
                service_action_e act = lora.service();
                if (act == SERVICE_READ_FIFO) {
                    if (!per_parse_rx(lora.RegRxNbBytes)) {
                        PacketNormalCnt++;
                        lora_print_rx_verbose(lora.RegRxNbBytes);                            
                    }                        
                }
                break;
            }
        } while (!lora.RegIrqFlags.bits.RxTimeout);
        radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
    }
    
    if (lora.RegIrqFlags.bits.CadDone) {
        lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
        num_cads++;
        radio.set_opmode(RF_OPMODE_CAD);
    }        

}

void cmd_restart_rx(uint8_t);
int preamble_to_sync_us;
#ifndef TARGET_DISCO_L072CZ_LRWAN1
Timeout timeout_syncAddress;
bool get_syncAddress;
#endif
float preamble_detect_at;

void callback_sa_timeout()
{
    printf("syncAddress timeout ");
    if (dio2.read() == 0) {
        //cmd_restart_rx(0);
        rx_start_timer.reset();
        radio.set_opmode(RF_OPMODE_STANDBY);
        printf("(false preamble detect at %f, secs:%lu)\r\n", preamble_detect_at, time(NULL) - secs_rx_start);
        secs_rx_start = time(NULL);
        radio.set_opmode(RF_OPMODE_RECEIVER);
    } else
        printf("\r\n");
}   

void
service_radio()
{
    service_action_e act;
    static uint8_t rssi = 0;
    
    if (radio.RegOpMode.bits.LongRangeMode) {
        if (rssi_polling_thresh != 0 && radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
            rssi = lora.get_current_rssi(); // dBm returned, negative value
#if defined(TARGET_STM) && !defined(TARGET_DISCO_L072CZ_LRWAN1) && !defined(TARGET_MTS_MDOT_F411RE)
            if (rssi < rssi_polling_thresh)
                pc3 = 0;    // signal weaker than threshold
            else
                pc3 = 1;    // signal stronger than threshold            
#endif
        }
        
        if (cadper_enable) {
            cadper_service();
        }

        act = lora.service();
    
        switch (act) {
            case SERVICE_READ_FIFO:
                if (app == APP_NONE) {
                    if (per_en) {
                        if (!per_parse_rx(lora.RegRxNbBytes)) {
                            PacketNormalCnt++;
                            lora_print_rx_verbose(lora.RegRxNbBytes);                            
                        }
                    } else                     
                        lora_print_rx_verbose(lora.RegRxNbBytes);
                    fflush(stdout);
                } else if (app == APP_CHAT) {
                    if (lora.RegHopChannel.bits.RxPayloadCrcOn) {
                        if (lora.RegIrqFlags.bits.PayloadCrcError)
                            printf("crcError\r\n");
                        else {
                            int n = lora.RegRxNbBytes;
                            radio.rx_buf[n++] = '\r';
                            radio.rx_buf[n++] = '\n';
                            radio.rx_buf[n] = 0; // null terminate
                            printf((char *)radio.rx_buf);
                        }
                    } else
                        printf("crcOff\r\n");
                        
                    // clear Irq flags
                    radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
                    // should still be in receive mode
                }
                break;
            case SERVICE_TX_DONE:
                if (app == APP_CHAT) {
                    lora.start_rx(RF_OPMODE_RECEIVER);
                } else if (per_en) {
                    per_timeout.attach(&per_cb, per_tx_delay); // start next TX              
                } else if (on_txdone_state != ON_TXDONE_STATE_NONE) {
                    on_txdone_timeout.attach(&txdone_timeout_cb, on_txdone_delay);
                }
                break;
            case SERVICE_ERROR:
                printf("error\r\n");
                break;
            case SERVICE_NONE:
                break;
        } // ...switch (act)
    } else {
        /* FSK: */
        
        if (rx_payloadReady_int_en)
            return; // radio service by ISR only

        if (ulrx_enable)
            return;

        act = fsk.service();
        
         switch (act) {
             case SERVICE_READ_FIFO:
                if (app == APP_CHAT) {
                    int n = fsk.rx_buf_length;
                    radio.rx_buf[n++] = '\r';
                    radio.rx_buf[n++] = '\n';
                    radio.rx_buf[n] = 0; // null terminate
                    printf((char *)radio.rx_buf);                    
                } else {
                    if (fsk.RegRxConfig.bits.AfcAutoOn) {
                        printf("%dHz ", (int)(FREQ_STEP_HZ * fsk.RegAfcValue));   
                        if (rssi != 0) {
                            printf("pkt:-%.1fdBm ", rssi / 2.0);
                            rssi = 0;
                        }    
                    }
                    if (per_en) { 
                        if (!per_parse_rx(fsk.rx_buf_length)) {
                            PacketNormalCnt++;
                            print_rx_buf(fsk.rx_buf_length);                                                 
                        }                                       
                    } else {
                        print_rx_buf(fsk.rx_buf_length);                            
                    }
                    
                }
                if (crc32_en) {
                    uint32_t c, *u32_ptr = (uint32_t*)&radio.rx_buf[fsk.rx_buf_length-4];
                    printf("rx crc:%08x, ", (unsigned int)(*u32_ptr));
                    c = gen_crc(radio.rx_buf, fsk.rx_buf_length-4);
                    printf("calc crc:%08x\r\n", (unsigned int)c);                    
                }
                fflush(stdout);
                break;
            case SERVICE_TX_DONE:
                if (ook_test_en)
                    radio.set_opmode(RF_OPMODE_SLEEP);
                if (app == APP_CHAT) {
                    fsk.start_rx();
                } else if (per_en) {
                    per_timeout.attach(&per_cb, per_tx_delay); // start next TX
                }                
                break;                
            case SERVICE_ERROR:
            case SERVICE_NONE:
                break;                
        } // ...switch (act)

#ifndef TARGET_DISCO_L072CZ_LRWAN1
        /* FSK receiver handling of preamble detection */
        if (radio.RegDioMapping2.bits.MapPreambleDetect && radio.RegDioMapping2.bits.Dio4Mapping == 3) {
            if (saved_dio4 != dio4.read()) {
                //printf("predet-dio4:%d\r\n", dio4.read());
                /* FSK: preamble detect state change */
                if (dio4.read()) {
                    if (radio.RegDioMapping1.bits.Dio2Mapping == 3) {   // if we can see SyncAddress
                        get_syncAddress = true;
                        timeout_syncAddress.attach_us(callback_sa_timeout, preamble_to_sync_us);
                    }
                    /* how long after RX start is preamble detection occuring? */
                    //printf("preamble detect at %f\r\n", rx_start_timer.read());
                    preamble_detect_at = rx_start_timer.read(); 
                } else {
                    get_syncAddress = false;
                    //printf("preamble detect clear\r\n");
                }
                saved_dio4 = dio4.read();
             }
         }  // ..if dio4 is 
             
         if (radio.RegDioMapping1.bits.Dio2Mapping == 3) {
             if (dio2.read()) {
                 if (get_syncAddress) {
                     timeout_syncAddress.detach();
                     get_syncAddress = false;
                 }                 
                 rssi = radio.read_reg(REG_FSK_RSSIVALUE);
             }
         }
#endif /* !TARGET_DISCO_L072CZ_LRWAN1 */           

        if (rssi_polling_thresh != 0 && radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
            rssi = radio.read_reg(REG_FSK_RSSIVALUE);
            rssi = -rssi;
#if defined(TARGET_STM) && !defined(TARGET_DISCO_L072CZ_LRWAN1) && !defined(TARGET_MTS_MDOT_F411RE)
            if (rssi < rssi_polling_thresh)
                pc3 = 0;    // signal weaker than threshold
            else
                pc3 = 1;    // signal stronger than threshold
#endif
        }

    } // ...!radio.RegOpMode.bits.LongRangeMode
}

/*int get_kbd_str(char* buf, int size)
{
    char c;
    int i;
    static int prev_len;
    
    for (i = 0;;) {
        if (pc.readable()) {
            c = pc.getc();
            if (c == 8 && i > 0) {
                pc.putc(8);
                pc.putc(' ');
                pc.putc(8);
                i--;
            } else if (c == '\r') {
                if (i == 0) {
                    return prev_len; // repeat previous
                } else {
                    buf[i] = 0; // null terminate
                    prev_len = i;
                    return i;
                }
            } else if (c == 3) {
                // ctrl-C abort
                return -1;
            } else if (i < size) {
                buf[i++] = c;
                pc.putc(c);
            }
        } else {
            service_radio();
        }
    } // ...for()
}*/

void
console_chat()
{
    //int i, len = get_kbd_str(pcbuf, sizeof(pcbuf));
    
    service_radio();
    
    if (pcbuf_len < 0) {
        printf("chat abort\r\n");
        pcbuf_len = 0;
        app = APP_NONE;
        return;
    } else if (pcbuf_len == 0) {
        return;
    } else {
        int i;
        for (i = 0; i < pcbuf_len; i++)
            radio.tx_buf[i] = pcbuf[i];
        if (radio.RegOpMode.bits.LongRangeMode) {
            lora.RegPayloadLength = pcbuf_len;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
            lora.start_tx(pcbuf_len);
        } else {
            fsk.start_tx(pcbuf_len);
        }
        pcbuf_len = 0;
        printf("\r\n");
    }
}

const uint8_t ookt_tx_payload[29] = {
    0x55, 0x55, 0x55, 0x55, 0xA9, 0x66, 0x69, 0x65,
    0x39, 0x53, 0xAA, 0xC3, 0xA6, 0x95, 0xC6, 0x3C,
    0x6A, 0x33, 0x33, 0xC6, 0xCA, 0xA6, 0x33, 0x33,
    0x55, 0x6A, 0xA6, 0xAA, 0x53
};
volatile unsigned int ook_tx_cnt = 0;

void callback_ook_tx_test()
{
    unsigned int i;  
    
    //radio.write_reg(REG_FSK_SYNCCONFIG, 0);
      
    printf("%u ookTx: ", ook_tx_cnt++);
    for (i = 0; i < sizeof(ookt_tx_payload); i++) {
        radio.tx_buf[i] = ookt_tx_payload[i];
        printf("%02x ", radio.tx_buf[i]);
    }
    printf("\r\n");
    
    //printf("syncConf:%x\r\n", radio.read_reg(REG_FSK_SYNCCONFIG));
    fsk.start_tx(sizeof(ookt_tx_payload));     
}

typedef enum {
    TXTICKER_STATE_OFF = 0,
    TXTICKER_STATE_TOGGLE_PAYLOAD_BIT,
    TXTICKER_STATE_CYCLE_PAYLOAD_LENGTH,
    TXTICKER_STATE_CYCLE_CODING_RATE,
    TXTICKER_STATE_TOG_HEADER_MODE,
    TXTICKER_STATE_TOG_CRC_ON,
    TXTICKER_STATE_CYCLE_SYNC_1,
    TXTICKER_STATE_CYCLE_SYNC_2,
    TXTICKER_STATE_RAMP_PAYLOAD_DATA_START,
    TXTICKER_STATE_RAMP_PAYLOAD_DATA,
    TXTICKER_STATE_SYMBOL_SWEEP,
    TXTICKER_STATE_TOGGLE_ALL_BITS_START,
    TXTICKER_STATE_TOGGLE_ALL_BITS,
} txticker_state_e;

txticker_state_e txticker_state;
float tx_ticker_rate = 0.5;
Ticker tx_ticker;

uint8_t txticker_sync_byte;
uint8_t payload_length_stop;
uint8_t symbol_num;
uint32_t symbol_sweep_bit_counter = 0;
unsigned int symbol_sweep_bit_counter_stop; 
uint8_t symbol_sweep_nbits;
uint8_t byte_pad_length;

uint8_t tab_current_byte_num;
uint8_t tab_current_bit_in_byte;

void fp_cb()
{   
    int i;
    if (!radio.RegOpMode.bits.LongRangeMode)
        return;
          
    switch (txticker_state) {
    case TXTICKER_STATE_TOGGLE_PAYLOAD_BIT:
/*
        {
            if (fp_tog_bit_ < 32) {
                uint32_t bp = 1 << fp_tog_bit_;
                fp_data ^= bp;
                //printf("bp%02x ", bp);
            }
            memcpy(radio.tx_buf, &fp_data, fp_data_length);
            printf("TX ");
            for (i = 0; i < fp_data_length; i++)
                printf("%02x ", radio.tx_buf[i]);
                
            printf("\r\n");
            lora.start_tx(lora.RegPayloadLength);        
            break;
        }
*/
        tx_ticker.detach();
        break;
    case TXTICKER_STATE_CYCLE_PAYLOAD_LENGTH:
        {
            if (lora.RegPayloadLength > payload_length_stop)
                lora.RegPayloadLength = 0;
            else
                lora.RegPayloadLength++;
                
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);        
            lora.start_tx(lora.RegPayloadLength);
            printf("RegPayloadLength:%d\r\n", lora.RegPayloadLength);        
            break;
        }
    case TXTICKER_STATE_CYCLE_CODING_RATE:
        {
            uint8_t cr = lora.getCodingRate(false); // false: TX coding rate
            if (cr == 4)
                cr = 0;
            else
                cr++;
                
            lora.setCodingRate(cr);
            lora.start_tx(lora.RegPayloadLength);
            printf("tx cr:%d\r\n", cr);        
            break;
        }
    case TXTICKER_STATE_TOG_HEADER_MODE:
        {
            lora.setHeaderMode(!lora.getHeaderMode());
            lora.start_tx(lora.RegPayloadLength);
            lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
            lora_printHeaderMode();
            printf("\r\n");          
            break;
        }
    case TXTICKER_STATE_TOG_CRC_ON:
        {
            lora.setRxPayloadCrcOn(!lora.getRxPayloadCrcOn());
            lora.start_tx(lora.RegPayloadLength);
            printf("crc on:%d\r\n", lora.getRxPayloadCrcOn());        
            break;
        }
    case TXTICKER_STATE_CYCLE_SYNC_1:
        {
            /* cycle hi nibble of 0x39 register */
            if ((txticker_sync_byte & 0xf0) == 0xf0)
                txticker_sync_byte &= 0x0f;
            else
                txticker_sync_byte += 0x10;
            radio.write_reg(REG_LR_SYNC_BYTE, txticker_sync_byte);
            lora.start_tx(lora.RegPayloadLength);
            printf("0x39: %02x\r\n", txticker_sync_byte);        
            break;
        }
    case TXTICKER_STATE_CYCLE_SYNC_2:
        {
            /* cycle lo nibble of 0x39 register */
            if ((txticker_sync_byte & 0x0f) == 0x0f)
                txticker_sync_byte &= 0xf0;
            else
                txticker_sync_byte += 0x01;
            radio.write_reg(REG_LR_SYNC_BYTE, txticker_sync_byte);
            lora.start_tx(lora.RegPayloadLength);
            printf("0x39: %02x\r\n", txticker_sync_byte);         
            break;
        }
    case TXTICKER_STATE_RAMP_PAYLOAD_DATA_START:
        txticker_state = TXTICKER_STATE_RAMP_PAYLOAD_DATA;
        for (i = 0; i < lora.RegPayloadLength; i++)
            radio.tx_buf[i] = 0;

        lora.start_tx(lora.RegPayloadLength);
        printf("payload start, len:%d\r\n", lora.RegPayloadLength);
        break;
    case TXTICKER_STATE_RAMP_PAYLOAD_DATA:
        for (i = lora.RegPayloadLength-1; i >= 0; i--) {
            //printf("i:%d ", i);
            if (radio.tx_buf[i] == 255) {
                radio.tx_buf[i] = 0;
            } else {
                radio.tx_buf[i]++;
                break;
            }
        }
        //printf("\r\n");
        printf("send:");
        for (i = 0; i < lora.RegPayloadLength; i++) {
            printf("%02x ", radio.tx_buf[i]);
        }
        printf("\r\n");
        lora.start_tx(lora.RegPayloadLength);
        if (radio.tx_buf[0] == 255) {
            printf("payload ramp done\r\n");
            tx_ticker.detach();
        }
        break;
    case TXTICKER_STATE_SYMBOL_SWEEP:       // fpsNL command, where N=symbol num, L=nbytes
        {
            uint32_t mask;
            /*for (i = 0; i < lora.RegPayloadLength; i++)
                radio.tx_buf[i] = 0;*/
            i = byte_pad_length;
            printf("bit_counter 0x%" PRIx32 " : ", symbol_sweep_bit_counter);
            for (int bn = 0; bn < symbol_sweep_nbits; bn += 2) {
                /* 2 lsbits going into first byte */
                mask = 1 << bn;
                if (symbol_sweep_bit_counter & mask)
                    radio.tx_buf[i] |= 1 << symbol_num;
                else
                    radio.tx_buf[i] &= ~(1 << symbol_num);
                mask = 2 << bn;
                if (symbol_sweep_bit_counter & mask)
                    radio.tx_buf[i] |= 0x10 << symbol_num;
                else
                    radio.tx_buf[i] &= ~(0x10 << symbol_num);
                //printf("%02x ", radio.tx_buf[i]);
                i++;
            }
            for (i = 0; i < lora.RegPayloadLength; i++)
                printf("%02x ", radio.tx_buf[i]);
            printf("\r\n");
            lora.start_tx(lora.RegPayloadLength);
            if (++symbol_sweep_bit_counter == symbol_sweep_bit_counter_stop) {
                printf("stop\r\n");
                tx_ticker.detach();
            }
        }
        break;
    case TXTICKER_STATE_TOGGLE_ALL_BITS_START:
        tab_current_byte_num = byte_pad_length;
        tab_current_bit_in_byte = 0;
        printf("tx ");
        for (i = 0; i < lora.RegPayloadLength; i++) {
            radio.tx_buf[i] = 0;
            printf("%02x ", radio.tx_buf[i]);
        }
        printf("\r\n");
        txticker_state = TXTICKER_STATE_TOGGLE_ALL_BITS;
        lora.start_tx(lora.RegPayloadLength);
        break;
    case TXTICKER_STATE_TOGGLE_ALL_BITS:
        {
            uint8_t mask = 1 << tab_current_bit_in_byte;
            radio.tx_buf[tab_current_byte_num] = mask;
            printf("bit%d in [%d]: tx ", tab_current_bit_in_byte, tab_current_byte_num);
            for (i = 0; i < lora.RegPayloadLength; i++) {
                printf("%02x ", radio.tx_buf[i]);
            }
            printf("\r\n");            
            lora.start_tx(lora.RegPayloadLength);
            if (++tab_current_bit_in_byte == 8) {
                radio.tx_buf[tab_current_byte_num] = 0;
                tab_current_bit_in_byte = 0;
                if (++tab_current_byte_num == lora.RegPayloadLength) {
                    tx_ticker.detach();
                }
            }
        }
        break;
    default:
            tx_ticker.detach();
            break;
    } // ...switch (txticker_state)
}

void ook_test_tx(int len)
{    
    int i;
    /*
    fsk.RegPktConfig2.bits.PayloadLength = i;
                    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);fsk.RegPktConfig2.bits.PayloadLength = i;
                    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);
                    */
    for (i = 0; i < 4; i++) {
        radio.tx_buf[i] = 0xaa;
    }
    
    printf("ooktx:");
    for (i = 0; i < len; i++) {
        radio.tx_buf[i+4] = rand() & 0xff;
        printf("%02x ", radio.tx_buf[i+4]);
    }
    printf("\r\n");
    fsk.start_tx(len+4); 
    
    while (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
        if (poll_irq_en)
            poll_service_radio();
        else
            service_radio();      
    }
}

void cmd_init(uint8_t args_at)
{
    printf("init\r\n");
    radio.init();
    if (!radio.RegOpMode.bits.LongRangeMode) {
        fsk.init();   // put FSK modem to some functioning default
    } else {
        // lora configuration is more simple
    }    
}

void cmd_per_tx_delay(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        per_tx_delay = i / 1000.0;
    }
    printf("per_tx_delay:%dms\r\n", (int)(per_tx_delay * 1000));      
}

const uint8_t test_payload_A[7] = {
    0x80, 0x02, 0x58, 0xF5, 0xDF, 0xB8, 0x9E
};

const uint8_t test_payload_B[] = {
    0xca, 0xfe, 0xba, 0xbe
};

const uint8_t test_payload_C[0x1a] = {
    0x88, 0x39, 0x1F, 0xC6, 0xD3, 0xEB, 0xA4, 0xAC,
    0xFB, 0xB9, 0xBA, 0xB9, 0xBE, 0x13, 0x61, 0x4C,
    0x43, 0x83, 0x00, 0x92, 0x84, 0x00, 0x6F, 0x87,
    0x7C, 0xB2
};

void cmd_tx(uint8_t idx)
{
    int i;
    static uint16_t fsk_tx_length;

    if (radio.RegOpMode.bits.LongRangeMode) {          
        if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
            sscanf(pcbuf+idx, "%d", &i);
            lora.RegPayloadLength = i;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        }
        
        if (pcbuf[idx] == 'A') {
        } else if (pcbuf[idx] == 'B') {
        } else if (pcbuf[idx] == 'C') {
        } else {        
            tx_cnt++;
            printf("payload:%02x\r\n", tx_cnt);
            
            for (i = 0; i < lora.RegPayloadLength; i++)
                radio.tx_buf[i] = tx_cnt;
        }
        
        lora.start_tx(lora.RegPayloadLength);
    } else {    // FSK:
    
        /* always variable-length format */
        fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
        if (!fsk.RegPktConfig1.bits.PacketFormatVariable) {
            printf("fsk fixed->variable\r\n");
            fsk.RegPktConfig1.bits.PacketFormatVariable = 1;
            radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
        }
        
        if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
            sscanf(pcbuf+idx, "%d", &i);
            fsk_tx_length = i;
        }
        if (ook_test_en) {
            ook_test_tx(fsk_tx_length);
        } else {
            if (radio.RegOpMode.bits.Mode != RF_OPMODE_TRANSMITTER) { // if not already busy transmitting
                if (pcbuf[idx] == 'A') {
                    fsk_tx_length = sizeof(test_payload_A);
                    memcpy(radio.tx_buf, test_payload_A, fsk_tx_length);
                } else if (pcbuf[idx] == 'B') {
                    fsk_tx_length = sizeof(test_payload_B);
                    memcpy(radio.tx_buf, test_payload_B, fsk_tx_length);                    
                } else if (pcbuf[idx] == 'C') {
                    fsk_tx_length = sizeof(test_payload_C);
                    memcpy(radio.tx_buf, test_payload_C, fsk_tx_length);                    
                } else {   
                    tx_cnt++;
                    printf("payload:%02x\r\n", tx_cnt);
                    for (i = 0; i < fsk_tx_length; i++) {
                        radio.tx_buf[i] = tx_cnt;
                    }
                }
                
                fsk.start_tx(fsk_tx_length);
            }
        }
    } // !LoRa                

}

volatile uint16_t long_byte_count, long_byte_count_at_full;

const uint8_t test_preamble_sync[] = {
    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0x33, 0xcb, 0x82
};

void _ulm_write_fifo(uint8_t len)
{
    uint8_t i;
    
    //dio2 is FifoFull
    radio.m_cs = 0;
    radio.m_spi.write(REG_FIFO | 0x80); // bit7 is high for writing to radio
    
    for (i = 0; i < len; ) {
        //printf("_%02x\r\n", radio.tx_buf[i]);
        radio.m_spi.write(radio.tx_buf[i++]);
        long_byte_count++;
        if (dio2) {
            long_byte_count_at_full = long_byte_count;
            while (radio.dio1)
                ;
        }
    }
    radio.m_cs = 1;
}

int write_buf_to_fifo(const uint8_t* send_buf, uint8_t target_length)
{
    /* block until all is written */
    uint8_t total_sent = 0;
    
    //printf("wbtf %u\r\n", target_length);
    while (target_length > total_sent) {
        uint8_t this_length = target_length - total_sent;
        memcpy(radio.tx_buf+total_sent, send_buf+total_sent, this_length);
        _ulm_write_fifo(this_length);
        total_sent += this_length;
    }
    return total_sent;
}

#define TEST_PAYLOAD        test_payload_C
//#define TEST_PAYLOAD        test_payload_A
void cmd_long_tx(uint8_t idx)
{
    unsigned int pkt_cnt = 0;
    bool first_pkt;
    /* transmit multipe packets without any time between packets (back to back) */
    
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%u", &pkt_cnt);
    }
    
    printf("tx %u pkts\r\n", pkt_cnt);
    if (pkt_cnt < 1)
        return;
        
    radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    radio.RegDioMapping2.bits.Dio5Mapping = 2;  // data output to observation
    radio.RegDioMapping2.bits.Dio4Mapping = 3;  // output preamble detect indication
    radio.RegDioMapping2.bits.MapPreambleDetect = 1;
    radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);
        
    //unlimited packet length mode
    fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
    fsk.RegPktConfig1.bits.PacketFormatVariable = 0;    // fixed length format
    radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
    
    fsk.RegPktConfig2.word = radio.read_u16(REG_FSK_PACKETCONFIG2);
    fsk.RegPktConfig2.bits.PayloadLength = 0;
    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);
    //DIO3 to FifoEmpty (for end of tx)
    radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
    radio.RegDioMapping1.bits.Dio3Mapping = 0;  // FifoEmpty
    //DIO2 to FifoFull
    radio.RegDioMapping1.bits.Dio2Mapping = 0;  // FIfoFull
    //DIO1 to FifoLevel
    radio.RegDioMapping1.bits.Dio1Mapping = 0;  // FifoLevel
    radio.RegDioMapping1.bits.Dio0Mapping = 0;  // PacketSent
    radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet); 
    //FifoThreshold to approx 1/5th full
    fsk.RegFifoThreshold.octet = radio.read_reg(REG_FSK_FIFOTHRESH);  
    fsk.RegFifoThreshold.bits.FifoThreshold = sizeof(TEST_PAYLOAD)-1; // allow single packet
    // tx start condition to FifoLevel
    fsk.RegFifoThreshold.bits.TxStartCondition = 0; // start on FifoLevel
    radio.write_reg(REG_FSK_FIFOTHRESH, fsk.RegFifoThreshold.octet);
    
    long_byte_count = 0;
    long_byte_count_at_full = 0xffff;
    
    radio.set_opmode(RF_OPMODE_TRANSMITTER);
    first_pkt = true;   // preamble+sync sent by packet engine only for first packet
    for (; pkt_cnt > 0; pkt_cnt--) {
        uint8_t len;
        if (first_pkt)
            first_pkt = false;
        else {
            if (dio3) {
                printf("fail-empty\r\n");
            }
            write_buf_to_fifo(test_preamble_sync, sizeof(test_preamble_sync));
        }
        
        len = sizeof(TEST_PAYLOAD); //TEST_PAYLOAD doesnt start with length
        write_buf_to_fifo(&len, 1);
        write_buf_to_fifo(TEST_PAYLOAD, sizeof(TEST_PAYLOAD));        
    } // ..

    rx_start_timer.reset();
    rx_start_timer.start();
    while (!dio3) {
        if (rx_start_timer.read() > 1) {
            printf("fifoEmpty fail\r\n");
            radio.set_opmode(RF_OPMODE_STANDBY);
            return;
        }
    }

    rx_start_timer.reset();
    rx_start_timer.start();
    while (!radio.dio0) {
        if (rx_start_timer.read() > 3) {
            printf("PacketSent fail\r\n");
            radio.set_opmode(RF_OPMODE_STANDBY);
            return;
        }        
    }
    wait_us(100);
    radio.set_opmode(RF_OPMODE_STANDBY);
    printf("done ok %u, %u\r\n", long_byte_count, long_byte_count_at_full);

}

void cmd_hw_reset(uint8_t idx)
{
    printf("hw_reset()\r\n");
    radio.hw_reset();
    ook_test_en = false;
    poll_irq_en = false;
}

void cmd_read_all_regs(uint8_t idx)
{
    uint8_t a, d;
    
    // read all registers
    for (a = 1; a < 0x71; a++) {
        d = radio.read_reg(a);
        printf("%02x: %02x\r\n", a, d);
    }
}

void cmd_read_current_rssi(uint8_t idx)
{
    if (radio.RegOpMode.bits.Mode != RF_OPMODE_RECEIVER) {
        radio.set_opmode(RF_OPMODE_RECEIVER);
        wait_us(10000);
    }
    if (radio.RegOpMode.bits.LongRangeMode)
        printf("rssi:%ddBm\r\n", lora.get_current_rssi());
    else
        printf("rssi:-%.1f\r\n", radio.read_reg(REG_FSK_RSSIVALUE) / 2.0);
}

void cmd_rssi_polling(uint8_t idx)
{
    if ((pcbuf[idx] >= '0' && pcbuf[idx] <= '9') || pcbuf[idx] == '-') {
        sscanf(pcbuf+idx, "%d", &rssi_polling_thresh);  
    }
    printf("rssi_polling_thresh:%d\r\n", rssi_polling_thresh);
}

void cmd_lora_continuous_tx(uint8_t idx)
{
    /* TxContinuousMode same for sx1272 and sx1276 */
    lora.RegModemConfig2.octet = radio.read_reg(REG_LR_MODEMCONFIG2);
    lora.RegModemConfig2.sx1276bits.TxContinuousMode ^= 1;   
    radio.write_reg(REG_LR_MODEMCONFIG2, lora.RegModemConfig2.octet);
    lora.RegModemConfig2.octet = radio.read_reg(REG_LR_MODEMCONFIG2);
    
    lora_printTxContinuousMode();
    printf("\r\n");
}

void cmd_fsk_test_case(uint8_t idx)
{
    if (pcbuf[idx] < '0' || pcbuf[idx] > '9') {
        printf("%" PRIu32 "bps fdev:%" PRIu32 "hz ", fsk.get_bitrate(), fsk.get_tx_fdev_hz());
        printf("rxbw:%" PRIu32 "Hz ", fsk.get_rx_bw_hz(REG_FSK_RXBW));
        printf("afcbw:%" PRIu32 "Hz preambleLen:%" PRIu16 "\r\n", fsk.get_rx_bw_hz(REG_FSK_AFCBW), radio.read_u16(REG_FSK_PREAMBLEMSB));        
    } else {
        radio.set_opmode(RF_OPMODE_STANDBY);
        per_tx_delay = 0.3;
        
        if (radio.read_reg(REG_FSK_SYNCVALUE1) == 0x55 && radio.read_reg(REG_FSK_SYNCVALUE2)) {
            fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);    
            fsk.RegSyncConfig.bits.SyncSize = 2;
            radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
            radio.write_reg(REG_FSK_SYNCVALUE3, 0x90);
            radio.write_reg(REG_FSK_SYNCVALUE2, 0x4e);
            radio.write_reg(REG_FSK_SYNCVALUE1, 0x63);              
        }
        
        fsk.RegPreambleDetect.octet = radio.read_reg(REG_FSK_PREAMBLEDETECT);
        fsk.RegPreambleDetect.bits.PreambleDetectorOn = 1;
        radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet);                 
        
        fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);
        fsk.RegRxConfig.bits.AfcAutoOn = 1;
        fsk.RegRxConfig.bits.AgcAutoOn = 1;
        fsk.RegRxConfig.bits.RxTrigger = 7; // both
        radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);    
        
        fsk.RegPreambleDetect.bits.PreambleDetectorOn = 1;
        fsk.RegPreambleDetect.bits.PreambleDetectorSize = 1;
        fsk.RegPreambleDetect.bits.PreambleDetectorTol = 10;
        radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet);            
        
        switch (pcbuf[idx]) {
            case '0':
                fsk.set_bitrate(4800);
                fsk.set_tx_fdev_hz(5005);
                fsk.set_rx_dcc_bw_hz(10417, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(50000, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 8);                    
                break;
            case '1':
                fsk.set_bitrate(50000);
                fsk.set_tx_fdev_hz(25000);
                fsk.set_rx_dcc_bw_hz(62500, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(100000, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 9);                    
                break;           
            case '2':
                fsk.set_bitrate(38400);
                fsk.set_tx_fdev_hz(20020);
                fsk.set_rx_dcc_bw_hz(50000, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(100000, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 8);                        
                break;
            case '3':
                fsk.set_bitrate(1201);
                fsk.set_tx_fdev_hz(20020);
                fsk.set_rx_dcc_bw_hz(25000, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(50000, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 8);                 
                break;    
            case '4':
                fsk.set_bitrate(1201);
                fsk.set_tx_fdev_hz(4028);
                fsk.set_rx_dcc_bw_hz(7813, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(25000, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 8);
                break;
            case '5':
                fsk.set_bitrate(1201);
                fsk.set_tx_fdev_hz(4028);
                fsk.set_rx_dcc_bw_hz(5208, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(10417, 1);  // afcbw
                radio.write_u16(REG_FSK_PREAMBLEMSB, 8);                    
                break;   
            case '6':
                fsk.set_bitrate(65536);
                fsk.set_tx_fdev_hz(16384);
                fsk.set_rx_dcc_bw_hz(62500, 0);  // rxbw
                fsk.set_rx_dcc_bw_hz(100000, 1);  // afcbw
                
                fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
                fsk.RegPktConfig1.bits.CrcOn = 0;
                radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
        
                radio.write_u16(REG_FSK_PREAMBLEMSB, 5);             
                fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);    
                fsk.RegSyncConfig.bits.SyncSize = 2;
                fsk.RegSyncConfig.bits.SyncOn = 1;
                radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
                radio.write_reg(REG_FSK_SYNCVALUE1, 0x33);
                radio.write_reg(REG_FSK_SYNCVALUE2, 0xcb);
                radio.write_reg(REG_FSK_SYNCVALUE3, 0x82);    
                
                radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
                radio.RegOpMode.bits.ModulationType = 0;    // 0 = FSK 
                radio.RegOpMode.bits.ModulationShaping = 2; // 2=BT0.5
                radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);
                
                fsk.RegAfcFei.octet = radio.read_reg(REG_FSK_AFCFEI);
                fsk.RegAfcFei.bits.AfcAutoClearOn = 0;
                radio.write_reg(REG_FSK_AFCFEI, fsk.RegAfcFei.octet);
                
                fsk.RegRxConfig.bits.RxTrigger = 6; // preamble
                radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);
                radio.RegDioMapping2.bits.Dio4Mapping = 3;
                radio.RegDioMapping2.bits.MapPreambleDetect = 1;    // dio4 to preambleDetect in RX
                radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);     
                radio.RegDioMapping1.bits.Dio2Mapping = 3;  // dio2 to SyncAddress in RX
                radio.RegDioMapping1.bits.Dio1Mapping = 1;  // to FifoEmpty
                radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);                                   
                break;                                                                                           
        } // ...switch (pcbuf[idx])
        printf("%" PRIu32 "bps fdev:%" PRIu32 "hz ", fsk.get_bitrate(), fsk.get_tx_fdev_hz());
        printf("rxbw:%" PRIu32 "Hz ", fsk.get_rx_bw_hz(REG_FSK_RXBW));
        printf("afcbw:%" PRIu32 "Hz preambleLen:%" PRIu16 "\r\n", fsk.get_rx_bw_hz(REG_FSK_AFCBW), radio.read_u16(REG_FSK_PREAMBLEMSB));  
        
        /* time between preamble occurring and syncAddress occuring 
         * = bitrate in microseconds * 8 * (preamble bytes + sync bytes)
         */
        preamble_to_sync_us = (1.0 / fsk.get_bitrate())*1e6 * 8 * (radio.read_u16(REG_FSK_PREAMBLEMSB)+1 + fsk.RegSyncConfig.bits.SyncSize+2);
        //printf("bitrate:%d, %f\r\n", fsk.get_bitrate(), 1.0 / fsk.get_bitrate()*1e6);
        printf("preamble_to_sync_us:%d\r\n", preamble_to_sync_us);
    }
}

void cmd_restart_rx(uint8_t idx)
{
    fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);
    fsk.RegRxConfig.bits.RestartRxWithoutPllLock = 1;
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);
    rx_start_timer.reset();
    secs_rx_start = time(NULL);
    fsk.RegRxConfig.bits.RestartRxWithoutPllLock = 0;
    printf("RestartRxWithoutPllLock\r\n");
}

void cmd_toggle_modem(uint8_t idx)
{
    ook_test_en = false;
    poll_irq_en = false;
    if (radio.RegOpMode.bits.LongRangeMode)
        fsk.enable(false);
    else
        lora.enable();

    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (radio.RegOpMode.bits.LongRangeMode)
        printf("LoRa\r\n");
    else
        printf("FSK\r\n");
}

void cmd_empty_fifo(uint8_t idx)
{
    RegIrqFlags2_t RegIrqFlags2;
    RegIrqFlags2.octet = radio.read_reg(REG_FSK_IRQFLAGS2);
    while (!RegIrqFlags2.bits.FifoEmpty) {
        if (pc.readable())
            break;
        printf("%02x\r\n", radio.read_reg(REG_FIFO));
        RegIrqFlags2.octet = radio.read_reg(REG_FSK_IRQFLAGS2);
    }
}

void cmd_print_status(uint8_t idx)
{
    if (radio.type == SX1276) {
#if defined(TARGET_MTS_MDOT_F411RE)
        printf("\r\nSX1276 ");
#else
        if (shield_type == SHIELD_TYPE_LAS)
            printf("\r\nSX1276LAS ");
        if (shield_type == SHIELD_TYPE_MAS)
            printf("\r\nSX1276MAS ");
#endif /* !TARGET_MTS_MDOT_F411RE */                       
    } else if (radio.type == SX1272)
        printf("\r\nSX1272 ");
        
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (radio.RegOpMode.bits.LongRangeMode)
        lora_print_status();
    else
        fsk_print_status();
    common_print_status();
}
                
void cmd_hop_period(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        lora.RegHopPeriod = i;
        radio.write_reg(REG_LR_HOPPERIOD, lora.RegHopPeriod);
        if (radio.RegDioMapping1.bits.Dio1Mapping != 1) {
            radio.RegDioMapping1.bits.Dio1Mapping = 1;
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
        }
    }
    lora.RegHopPeriod = radio.read_reg(REG_LR_HOPPERIOD);
    printf("HopPeriod:0x%02x\r\n", lora.RegHopPeriod);
}       

void cmd_lora_ppg(uint8_t idx)
{
    int i;
    if (pcbuf[idx] != 0) {
        sscanf(pcbuf+idx, "%x", &i);
        radio.write_reg(REG_LR_SYNC_BYTE, i);
    }
    printf("lora sync:0x%02x\r\n", radio.read_reg(REG_LR_SYNC_BYTE));
}

void cmd_rssi_offset(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        fsk.RegRssiConfig.bits.RssiOffset = i;
        radio.write_reg(REG_FSK_RSSICONFIG, fsk.RegRssiConfig.octet);
    }            
    fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);          
    printf("RssiOffset:%d\r\n", fsk.RegRssiConfig.bits.RssiOffset); 
} 

void cmd_rssi_smoothing(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        fsk.RegRssiConfig.bits.RssiSmoothing = i;
        radio.write_reg(REG_FSK_RSSICONFIG, fsk.RegRssiConfig.octet);
    }                        
    fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);
    printf("RssiSmoothing:%d\r\n", fsk.RegRssiConfig.bits.RssiSmoothing); 
}

void cmd_rssi_threshold(uint8_t idx)
{
    if ((pcbuf[idx] >= '0' && pcbuf[idx] <= '9') || pcbuf[idx] == '-') {
        float dbm;
        sscanf(pcbuf+idx, "%f", &dbm);
        dbm *= (float)2.0;
        fsk.RegRssiThresh = (int)fabs(dbm);
        radio.write_reg(REG_FSK_RSSITHRESH, fsk.RegRssiThresh);
    }                                    
    fsk.RegRssiThresh = radio.read_reg(REG_FSK_RSSITHRESH);
    printf("rssiThreshold:-%.1f\r\n", fsk.RegRssiThresh / 2.0); 
}

void cmd_rx_trigger(uint8_t idx)
{
    printf("RxTrigger:");
    switch (fsk.RegRxConfig.bits.RxTrigger) {
        case 0: fsk.RegRxConfig.bits.RxTrigger = 1;
            printf("rssi\r\n");
            break;
        case 1: fsk.RegRxConfig.bits.RxTrigger = 6;
            printf("preamble\r\n");
            break;
        case 6: fsk.RegRxConfig.bits.RxTrigger = 7;
            printf("both\r\n");
            break;
        case 7: fsk.RegRxConfig.bits.RxTrigger = 0;
            printf("none\r\n");
            break;
        default: fsk.RegRxConfig.bits.RxTrigger = 0;
            printf("none\r\n");
            break;
        }
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);
}

void cmd_cadper(uint8_t idx)
{
    set_per_en(true);

    PacketNormalCnt = 0;
    PacketRxSequencePrev = 0;   // transmitter side PacketTxCnt is 1 at first TX
    PacketPerKoCnt = 0;
    PacketPerOkCnt = 0;
            
    cadper_enable = true;

    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    /* clear any stale flag */
    radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
    
    /* start first CAD */
    radio.set_opmode(RF_OPMODE_CAD);
    num_cads = 0;    
}

#if 0
void cmd_cadrx(uint8_t idx)
{
    int n_tries = 1;
    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    /* clear any stale flag */
    radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
    
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &n_tries);
    }
    
    while (n_tries > 0) {
        radio.set_opmode(RF_OPMODE_CAD);
        
        do {
            lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
        } while (!lora.RegIrqFlags.bits.CadDetected && !lora.RegIrqFlags.bits.CadDone);
        if (lora.RegIrqFlags.bits.CadDetected) {
            lora.start_rx(RF_OPMODE_RECEIVER_SINGLE);            
            n_tries = 1;    // end
            printf("CadDetected ");
        }
        if (lora.RegIrqFlags.bits.CadDone) {
            printf("CadDone ");
        }
        
        radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
        printf("\r\n");
        n_tries--;
    }    
}
#endif /* #if 0 */

void cmd_cad(uint8_t idx)
{
    int n_tries = 1;
    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    /* clear any stale flag */
    radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
    
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &n_tries);
    }
    
    while (n_tries > 0) {
        radio.set_opmode(RF_OPMODE_CAD);
        
        do {
            lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
        } while (!lora.RegIrqFlags.bits.CadDetected && !lora.RegIrqFlags.bits.CadDone);
        if (lora.RegIrqFlags.bits.CadDetected) {
            n_tries = 1;    // end
            printf("CadDetected ");
        }
        if (lora.RegIrqFlags.bits.CadDone) {
            if (n_tries == 1)  // print on last try
                printf("CadDone ");
        }
        
        radio.write_reg(REG_LR_IRQFLAGS, lora.RegIrqFlags.octet);
        n_tries--;
    }
    printf("\r\n");
}

void cmd_rx_timeout(uint8_t idx)
{
    int symb_timeout;
    uint16_t reg_u16 = radio.read_u16(REG_LR_MODEMCONFIG2);
    
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &symb_timeout);
        reg_u16 &= 0xfc00;
        reg_u16 |= symb_timeout;
        radio.write_u16(REG_LR_MODEMCONFIG2, reg_u16);
    }
    reg_u16 = radio.read_u16(REG_LR_MODEMCONFIG2);
    printf("SymbTimeout:%d\r\n", reg_u16 & 0x3ff);
}

void cmd_rx_single(uint8_t idx)
{
    lora.start_rx(RF_OPMODE_RECEIVER_SINGLE);
}

void preamble_without_sync()
{
    printf("preamble_without_sync Afc:%dHz\r\n", (int)(FREQ_STEP_HZ * radio.read_s16(REG_FSK_AFCMSB))); 
    fsk.RegRxConfig.bits.RestartRxWithoutPllLock = 1;
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet); 
}

Timeout pd_timeout;
Timeout sync_timeout;
void preamble_detect_isr()
{
    // only used between frames, on background noise
    pd_timeout.attach_us(&preamble_without_sync, 1500); // 122us per byte
}

void cmd_rx(uint8_t idx)
{    
    set_per_en(false);
  
    if (radio.RegOpMode.bits.LongRangeMode)
        lora.start_rx(RF_OPMODE_RECEIVER);
    else {
        if (poll_irq_en) {
            fsk_RegIrqFlags2_prev.octet = 0;
            fsk_RegIrqFlags1_prev.octet = 0; 
        }
        
        rx_start_timer.start();
        secs_rx_start = time(NULL);
        fsk.start_rx();
        radio.RegDioMapping1.bits.Dio2Mapping = 3;  // dio2 to syncadrs
        radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet); 
        if (radio.HF) {
            fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);
            fsk.RegRssiConfig.bits.RssiOffset = FSK_RSSI_OFFSET;
            fsk.RegRssiConfig.bits.RssiSmoothing = FSK_RSSI_SMOOTHING;
            radio.write_reg(REG_FSK_RSSICONFIG, fsk.RegRssiConfig.octet);
        }               
        
        // sync shadow regsiters
        radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
        radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    }
}

void cmd_radio_reg_read(uint8_t idx)
{
    int i;
    sscanf(pcbuf+idx, "%x", &i);
    printf("%02x: %02x\r\n", i, radio.read_reg(i));
}

void cmd_radio_reg_write(uint8_t idx)
{
    int i, n;
    sscanf(pcbuf+idx, "%x %x", &i, &n);
    radio.write_reg(i, n);
    printf("%02x: %02x\r\n", i, radio.read_reg(i));
}

void cmd_mod_shaping(uint8_t idx)
{
    uint8_t s = fsk.get_modulation_shaping();
    
    if (s == 3)
        s = 0;
    else
        s++;
        
    fsk.set_modulation_shaping(s);
    
    if (radio.RegOpMode.bits.ModulationType == 0) {
        printf("FSK ");
        switch (s) {
            case 0: printf("off"); break;
            case 1: printf("BT1.0 "); break;
            case 2: printf("BT0.5 "); break;
            case 3: printf("BT0.3 "); break;
        }
    } else if (radio.RegOpMode.bits.ModulationType == 1) {
        printf("OOK ");
        switch (s) {
            case 0: printf("off"); break;
            case 1: printf("Fcutoff=bitrate"); break;
            case 2: printf("Fcutoff=2*bitrate"); break;
            case 3: printf("?"); break;
        }        
    }    
    
    printf("\r\n");      
}

void cmd_MapPreambleDetect(uint8_t idx)
{
    radio.RegDioMapping2.bits.MapPreambleDetect ^= 1;
    radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);
    printf("MapPreambleDetect:");
    if (radio.RegDioMapping2.bits.MapPreambleDetect)
        printf("preamble\r\n");
    else
        printf("rssi\r\n");
}

void cmd_bgr(uint8_t idx)
{
    RegPdsTrim1_t pds_trim;
    uint8_t adr;
    if (radio.type == SX1276)
        adr = REG_PDSTRIM1_SX1276;
    else
        adr = REG_PDSTRIM1_SX1272;
       
    pds_trim.octet = radio.read_reg(adr);         
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        int i;
        sscanf(&pcbuf[idx], "%d", &i);
        pds_trim.bits.prog_txdac = i;
    }
    radio.write_reg(adr, pds_trim.octet);
    printf("prog_txdac:%.1fuA\r\n", 2.5 + (pds_trim.bits.prog_txdac * 0.625));
    /* increase OCP threshold to allow more power */
    radio.RegOcp.octet = radio.read_reg(REG_OCP);
    if (radio.RegOcp.bits.OcpTrim < 16) {
        radio.RegOcp.bits.OcpTrim = 16;
        radio.write_reg(REG_OCP, radio.RegOcp.octet);
    }    
}   

void cmd_ook(uint8_t idx)
{
    fsk.set_bitrate(32768);
    radio.write_u16(REG_FSK_PREAMBLEMSB, 0);    // zero preamble length
    radio.RegOpMode.bits.ModulationType = 1; // to ook mode
    radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);
    fsk.RegSyncConfig.bits.SyncOn = 0;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    ook_test_en = true;
    printf("OOK\r\n");
}

void cmd_ocp(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        if (i < 130)
            radio.RegOcp.bits.OcpTrim = (i - 45) / 5;
        else
            radio.RegOcp.bits.OcpTrim = (i + 30) / 10;
        radio.write_reg(REG_OCP, radio.RegOcp.octet);
    }            
    radio.RegOcp.octet = radio.read_reg(REG_OCP);
    if (radio.RegOcp.bits.OcpTrim < 16)
        i = 45 + (5 * radio.RegOcp.bits.OcpTrim);
    else if (radio.RegOcp.bits.OcpTrim < 28)
        i = (10 * radio.RegOcp.bits.OcpTrim) - 30;
    else
        i = 240;
    printf("Ocp: %dmA\r\n", i); 
}

void cmd_op(uint8_t idx)
{
    int i, dbm;
    RegPdsTrim1_t pds_trim;
    uint8_t adr;
    if (radio.type == SX1276)
        adr = REG_PDSTRIM1_SX1276;
    else
        adr = REG_PDSTRIM1_SX1272;
       
    pds_trim.octet = radio.read_reg(adr);   
                
    if (pcbuf[idx] >= '0' && (pcbuf[idx] <= '9' || pcbuf[idx] == '-')) {
        sscanf(pcbuf+idx, "%d", &i);
        if (radio.RegPaConfig.bits.PaSelect) {
            /* PABOOST used: +2dbm to +17, or +20 */
            if (i == 20) {
                printf("+20dBm PADAC bias\r\n");
                i -= 3;
                pds_trim.bits.prog_txdac = 7;
                radio.write_reg(adr, pds_trim.octet);
            }
            if (i > 1)
                    radio.RegPaConfig.bits.OutputPower = i - 2;
        } else {
            /* RFO used: -1 to +14dbm */
            if (i < 15)
                radio.RegPaConfig.bits.OutputPower = i + 1;
        }
        radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
    }
    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
    if (radio.RegPaConfig.bits.PaSelect) {
        printf("PA_BOOST ");
        dbm = radio.RegPaConfig.bits.OutputPower + pds_trim.bits.prog_txdac - 2;
    } else {
        printf("RFO ");
        dbm = radio.RegPaConfig.bits.OutputPower - 1;
    }
    printf("OutputPower:%ddBm\r\n", dbm);
}



void cmd_fsk_agcauto(uint8_t idx)
{
    fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);
    fsk.RegRxConfig.bits.AgcAutoOn ^= 1;
    printf("AgcAuto:");
    if (fsk.RegRxConfig.bits.AgcAutoOn)
        printf("On\r\n");
    else
        printf("OFF\r\n");          
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet); 
}

void cmd_fsk_afcauto(uint8_t idx)
{
    fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);            
    fsk.RegRxConfig.bits.AfcAutoOn ^= 1;
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);
    printf("AfcAuto:");
    if (fsk.RegRxConfig.bits.AfcAutoOn)
        printf("On\r\n");
    else
        printf("OFF\r\n");
}

void cmd_crc32(uint8_t idx)
{
    crc32_en ^= true;
    printf("crc32_en:%u\r\n", crc32_en);
}

void cmd_crcOn(uint8_t idx)
{
    if (radio.RegOpMode.bits.LongRangeMode) {
        lora.setRxPayloadCrcOn(!lora.getRxPayloadCrcOn());
        lora_printRxPayloadCrcOn();
    } else {
        printf("CrcOn:");
        fsk.RegPktConfig1.bits.CrcOn ^= 1;
        radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
        if (fsk.RegPktConfig1.bits.CrcOn)
            printf("On\r\n");
        else
            printf("Off\r\n");
        if (fsk.RegPktConfig2.bits.DataModePacket && radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
            fsk.config_dio0_for_pktmode_rx();
        }
    }
    printf("\r\n");  
} 

#ifdef LORA_TX_TEST
void cmd_lora_fixed_payload_symbol(uint8_t idx) // fixed payload, symbol test
{
    int n, i;
    
    symbol_num = pcbuf[idx] - '0';
    sscanf(pcbuf+idx+2, "%d", &i);
    n = i >> 2; // num nibbles
    printf("%d nibbles: ", n);
    lora.RegPayloadLength = byte_pad_length;
    while (n > 0) {
        lora.RegPayloadLength++;
        n -= 2; // one byte = two nibbles
    }
    printf("%d bytes\r\n", lora.RegPayloadLength);
    symbol_sweep_nbits = i >> 2;
    symbol_sweep_bit_counter = 0;
    symbol_sweep_bit_counter_stop = 1 << symbol_sweep_nbits;    // one bit per nibble used in symbol (2bits per byte)
    printf("sweep symbol %d, length bytes:%d nbits:%d stop:0x%x\r\n", symbol_num, lora.RegPayloadLength, symbol_sweep_nbits, symbol_sweep_bit_counter_stop);
    txticker_state = TXTICKER_STATE_SYMBOL_SWEEP;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_fixed_payload_offset(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >='0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        byte_pad_length = i;
    }
    printf("byte_pad_length:%d\r\n", byte_pad_length);
}

void cmd_lora_fixed_payload(uint8_t idx)
{
    int n, a, i, d = 0;
    for (i = idx; i < pcbuf_len; ) {
        //printf("scan:\"%s\"\r\n", pcbuf+i);
        sscanf(pcbuf+i, "%x", &n);
        //printf("n:%x\r\n", n);
        radio.tx_buf[d] = n;
        printf("%02x ", n);
        while (pcbuf[i] == ' ')
            i++;
        //printf("%d pcbuf[i]:%x\r\n", i, pcbuf[i]);
        for (a = i; pcbuf[a] != ' '; a++)
            if (a >= pcbuf_len)
                break;
        i = a;
        while (pcbuf[i] == ' ') {
            i++;                 
            if (i >= pcbuf_len)
                break;
        }   
        d++;
    }
    lora.RegPayloadLength = d;
    printf("\r\nlora.RegPayloadLength:%d\r\n", lora.RegPayloadLength);
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
    lora.start_tx(lora.RegPayloadLength);
}

void cmd_lora_toggle_crcOn(uint8_t idx)
{
    /* test lora crc on/off */
    lora.RegPayloadLength = 1;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);           
    txticker_state = TXTICKER_STATE_TOG_CRC_ON;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void lora_cycle_payload_length(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        payload_length_stop = i;
    }
    txticker_state = TXTICKER_STATE_CYCLE_PAYLOAD_LENGTH;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_lora_data_ramp(uint8_t idx)
{
    // lora payload data ramping
    lora.RegPayloadLength = pcbuf[idx] - '0';
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);           
    txticker_state = TXTICKER_STATE_RAMP_PAYLOAD_DATA_START;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_lora_sync_lo_nibble(uint8_t idx)
{
    lora_sync_byte = 0x00;
    on_txdone_state = ON_TXDONE_STATE_SYNC_LO_NIBBLE;
    on_txdone_delay = 0.100;
    txdone_timeout_cb();
    //sync_sweep_timeout.attach(&txdone_timeout_cb, sync_sweep_delay);
}

void cmd_lora_toggle_header_mode(uint8_t idx)
{
    lora.RegPayloadLength = 1;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);           
    txticker_state = TXTICKER_STATE_TOG_HEADER_MODE;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_lora_sync_sweep(uint8_t idx)
{
    lora.RegPayloadLength = 1;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
    txticker_sync_byte = 0x12;
    if (pcbuf[idx] == '1')
        txticker_state = TXTICKER_STATE_CYCLE_SYNC_1;
    else if (pcbuf[idx] == '2')
        txticker_state = TXTICKER_STATE_CYCLE_SYNC_2;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_lora_all_payload_lengths(uint8_t idx)
{
    on_txdone_repeat_cnt = 0;
    on_txdone_state = ON_TXDONE_STATE_PAYLOAD_LENGTH;
    on_txdone_delay = 0.200;
    txdone_timeout_cb();
    lora.RegPayloadLength = 0;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
}

void cmd_lora_toggle_all_bits(uint8_t idx)
{
    lora.RegPayloadLength = (pcbuf[idx] - '0') + byte_pad_length;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength); 
    txticker_state = TXTICKER_STATE_TOGGLE_ALL_BITS_START;
    printf("tab byte length:%d\r\n", lora.RegPayloadLength);

    if (lora.RegPayloadLength > 0)
        tx_ticker.attach(&fp_cb, tx_ticker_rate);
}

void cmd_lora_cycle_codingrates(uint8_t idx)
{
    lora.RegPayloadLength = 1;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);     
    txticker_state = TXTICKER_STATE_CYCLE_CODING_RATE;
    tx_ticker.attach(&fp_cb, tx_ticker_rate);
}
#endif /* LORA_TX_TEST */

void cmd_codingRate(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9')
        lora.setCodingRate(pcbuf[idx] - '0');
     lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
     lora_printCodingRate(false);    // false: transmitted
     printf("\r\n");
}

void cmd_lora_header_mode(uint8_t idx)
{
    lora.setHeaderMode(!lora.getHeaderMode());
    lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
    lora_printHeaderMode();
    printf("\r\n");
}

void cmd_fsk_AfcAutoClearOn(uint8_t idx)
{
    fsk.RegAfcFei.bits.AfcAutoClearOn ^= 1;
    printf("AfcAutoClearOn: ");
    radio.write_reg(REG_FSK_AFCFEI, fsk.RegAfcFei.octet);
    if (fsk.RegAfcFei.bits.AfcAutoClearOn)
        printf("ON\r\n");
    else
        printf("off\r\n");
}

void cmd_fsk_AutoRestartRxMode(uint8_t idx)
{
    fsk.RegSyncConfig.bits.AutoRestartRxMode++;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);
    printf("AutoRestartRxMode:");
    switch (fsk.RegSyncConfig.bits.AutoRestartRxMode) {
        case 0: printf("off "); break;
        case 1: printf("no-pll-wait "); break;
        case 2: printf("pll-wait "); break;
        case 3: printf("3 "); break;
    }
    printf("\r\n");   
}

void cmd_AfcClear(uint8_t idx)
{
    printf("clear afc: ");
    fsk.RegAfcFei.bits.AfcClear = 1;
    radio.write_reg(REG_FSK_AFCFEI, fsk.RegAfcFei.octet);
    fsk.RegAfcFei.bits.AfcClear = 0; 
    printf("%dHz\r\n", (int)(FREQ_STEP_HZ * radio.read_s16(REG_FSK_AFCMSB)));
}

void cmd_fsk_bitrate(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        float kbits;
        sscanf(&pcbuf[idx], "%f", &kbits);
        fsk.set_bitrate((int)(kbits*1000));
    }
    printf("%fkbps\r\n", fsk.get_bitrate()/(float)1000.0); 
}

void cmd_bandwidth(uint8_t idx)
{
    int i;
    float f;
    if (radio.RegOpMode.bits.LongRangeMode) {
        if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
            radio.set_opmode(RF_OPMODE_STANDBY);
            sscanf(&pcbuf[idx], "%d", &i);
            lora.setBw_KHz(i);
        } else
            lora_printAllBw();
        lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
        printf("current ");
        lora_printBw();
        printf("\r\n");
    } else { // FSK:
        if (pcbuf[idx] == 'a') {
            idx++;
            if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
                radio.set_opmode(RF_OPMODE_STANDBY);
                sscanf(&pcbuf[idx], "%f", &f);
                fsk.set_rx_dcc_bw_hz((int)(f*(float)1000.0), 1);
            }
            printf("afcbw:%.3fkHz\r\n", fsk.get_rx_bw_hz(REG_FSK_AFCBW)/1000.0);
        } else {
            if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
                radio.set_opmode(RF_OPMODE_STANDBY);
                sscanf(&pcbuf[idx], "%f", &f);
                fsk.set_rx_dcc_bw_hz((int)(f*(float)1000.0), 0);
            }
            printf("rxbw:%.3fkHz\r\n", fsk.get_rx_bw_hz(REG_FSK_RXBW)/1000.0);
        }
    }
}

void cmd_lora_poll_validHeader(uint8_t idx)
{
    lora.poll_vh ^= 1;
    printf("poll_vh:%d\r\n", lora.poll_vh);
}

void cmd_fsk_syncword(uint8_t idx)
{
    int i, d = 0;
    uint8_t reg_addr = REG_FSK_SYNCVALUE1;
    
    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);
    
    if (pcbuf_len != idx) { // something to write?
        for (i = idx; i < pcbuf_len; ) {
            int a, n;
            sscanf(pcbuf+i, "%x", &n);
            radio.write_reg(reg_addr++, n);
            //printf("%02x ", n);
            while (pcbuf[i] == ' ')
                i++;
            for (a = i; pcbuf[a] != ' '; a++)
                if (a >= pcbuf_len)
                    break;
            i = a;
            while (pcbuf[i] == ' ') {
                i++;                 
                if (i >= pcbuf_len)
                    break;
            }   
            d++;
        }    
     
        fsk.RegSyncConfig.bits.SyncSize = reg_addr - REG_FSK_SYNCVALUE1 - 1;
        radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    }
    
    printf("%d: ", fsk.RegSyncConfig.bits.SyncSize);
    for (i = 0; i <= fsk.RegSyncConfig.bits.SyncSize; i++)
        printf("%02x ", radio.read_reg(REG_FSK_SYNCVALUE1+i));
    printf("\r\n");
}

void cmd_fsk_syncOn(uint8_t idx)
{
    fsk.RegSyncConfig.bits.SyncOn ^= 1;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    printf("SyncOn:%d\r\n", fsk.RegSyncConfig.bits.SyncOn);
}

void cmd_fsk_bitsync(uint8_t idx)
{
    fsk.RegOokPeak.octet = radio.read_reg(REG_FSK_OOKPEAK);
    fsk.RegOokPeak.bits.BitSyncOn ^= 1;
    radio.write_reg(REG_FSK_OOKPEAK, fsk.RegOokPeak.octet);
    if (fsk.RegOokPeak.bits.BitSyncOn)
        printf("BitSyncOn\r\n");
    else
        printf("BitSync Off\r\n");
}

void cmd_lora_sf(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        lora.setSf(i);
        if (i == 6 && !lora.getHeaderMode()) {
            printf("SF6: to implicit header mode\r\n");
            lora.setHeaderMode(true);
        }
    }
    lora.RegModemConfig2.octet = radio.read_reg(REG_LR_MODEMCONFIG2);
    lora_printSf();
    printf("\r\n");
}

void cmd_fsk_TxStartCondition(uint8_t idx)
{
    fsk.RegFifoThreshold.bits.TxStartCondition ^= 1;
    radio.write_reg(REG_FSK_FIFOTHRESH, fsk.RegFifoThreshold.octet);
    printf("TxStartCondition:");
    if (fsk.RegFifoThreshold.bits.TxStartCondition)
        printf("!FifoEmpty\r\n");
    else
        printf("FifoLevel\r\n"); 
}        

void cmd_fsk_read_fei(uint8_t idx)
{
    printf("fei:%dHz\r\n", (int)(FREQ_STEP_HZ * radio.read_s16(REG_FSK_FEIMSB)));
}

void cmd_fsk_fdev(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        float khz;
        sscanf(pcbuf+idx, "%f", &khz);
        fsk.set_tx_fdev_hz((int)(khz*1000));
    }
    printf("fdev:%fKHz\r\n", fsk.get_tx_fdev_hz()/(float)1000.0);
}

void cmd_spifreq(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        int hz, MHz;
        sscanf(pcbuf+idx, "%d", &MHz);
        hz = MHz * 1000000;
        printf("spi hz:%u\r\n", hz);
        radio.m_spi.frequency(hz);
    }
}

void cmd_frf(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        float MHz;
        sscanf(pcbuf+idx, "%f", &MHz);
        //printf("MHz:%f\r\n", MHz);
        radio.set_frf_MHz(MHz);
    }
    printf("%fMHz\r\n", radio.get_frf_MHz());
#if !defined(TARGET_MTS_MDOT_F411RE) && !defined(TARGET_DISCO_L072CZ_LRWAN1)
    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
    if (shield_type == SHIELD_TYPE_LAS) {
        // LAS HF=PA_BOOST  LF=RFO
        if (radio.HF)
            radio.RegPaConfig.bits.PaSelect = 1;
        else
            radio.RegPaConfig.bits.PaSelect = 0;
    }
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);          
#endif /* !TARGET_MTS_MDOT_F411RE */
}

void cmd_fsk_PacketFormat(uint8_t idx)
{
    printf("PacketFormat:");
    fsk.RegPktConfig1.bits.PacketFormatVariable ^= 1;
    radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
    if (fsk.RegPktConfig1.bits.PacketFormatVariable)
        printf("variable\r\n");
    else
        printf("fixed\r\n");
}

void cmd_payload_length(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        if (radio.RegOpMode.bits.LongRangeMode) {
            lora.RegPayloadLength = i;
            radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        } else {
            fsk.RegPktConfig2.bits.PayloadLength = i;
            radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);
        }
    }
    if (radio.RegOpMode.bits.LongRangeMode) {
        lora.RegPayloadLength = radio.read_reg(REG_LR_PAYLOADLENGTH);
        printf("PayloadLength:%d\r\n", lora.RegPayloadLength);
    } else {
        printf("PayloadLength:%d\r\n", fsk_get_PayloadLength());
    }
}

void cmd_paRamp(uint8_t idx)
{
    int i;
    uint8_t reg_par = radio.read_reg(REG_PARAMP);
    uint8_t PaRamp = reg_par & 0x0f;
    reg_par &= 0xf0;
    if (PaRamp == 15)
        PaRamp = 0;
    else
        PaRamp++;
    radio.write_reg(REG_PARAMP, reg_par | PaRamp);
    printf("PaRamp:");
    switch (PaRamp) {
        case 0: i = 3400; break;
        case 1: i = 2000; break;
        case 2: i = 1000; break;
        case 3: i = 500; break;                
        case 4: i = 250; break;
        case 5: i = 125; break;
        case 6: i = 100; break;
        case 7: i = 62; break;                 
        case 8: i = 50; break;
        case 9: i = 40; break;
        case 10: i = 31; break;
        case 11: i = 25; break;                
        case 12: i = 20; break;
        case 13: i = 15; break;
        case 14: i = 12; break;
        case 15: i = 10; break;                
    }
    printf("%dus\r\n", i);
}

void cmd_paSelect(uint8_t idx)
{
    radio.RegPaConfig.bits.PaSelect ^= 1;
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
    printPa();
    printf("\r\n");
}

void cmd_poll_irq_en(uint8_t idx)
{
    poll_irq_en ^= 1;
    printf("poll_irq_en:");
    if (poll_irq_en) {
        printf("irqFlags register\r\n");
        fsk_RegIrqFlags1_prev.octet = 0;
        fsk_RegIrqFlags2_prev.octet = 0;
    } else
        printf("DIO pin interrupt\r\n");
}

void cmd_per_id(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &per_id);
    }
    printf("PER device ID:%d\r\n", per_id);
}

void cmd_pertx(uint8_t idx)
{
    int i;
    
    if (cadper_enable)
        cadper_enable = false;
    
    set_per_en(true);
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        PacketTxCntEnd = i;
    }
    PacketTxCnt = 0;
    per_timeout.attach(&per_cb, per_tx_delay);  
}

void cmd_perrx(uint8_t idx)
{
    set_per_en(true);

    PacketNormalCnt = 0;
    PacketRxSequencePrev = 0; // transmitter side PacketTxCnt is 1 at first TX
    PacketPerKoCnt = 0;
    PacketPerOkCnt = 0;                
    //dio3.rise(&dio3_cb);

    if (radio.RegOpMode.bits.LongRangeMode)
        lora.start_rx(RF_OPMODE_RECEIVER);
    else {
        fsk.start_rx();
        radio.RegDioMapping1.bits.Dio2Mapping = 3;  // dio2 to syncadrs
        radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet); 
        if (radio.HF) {
            fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);
            fsk.RegRssiConfig.bits.RssiOffset = FSK_RSSI_OFFSET;
            fsk.RegRssiConfig.bits.RssiSmoothing = FSK_RSSI_SMOOTHING;
            radio.write_reg(REG_FSK_RSSICONFIG, fsk.RegRssiConfig.octet);
        }                                
    }    
}

void cmd_fsk_PreambleDetectorOn(uint8_t idx)
{
    fsk.RegPreambleDetect.bits.PreambleDetectorOn ^= 1;
    radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet);
    printf("PreambleDetector:");
    if (fsk.RegPreambleDetect.bits.PreambleDetectorOn)
        printf("On\r\n");
    else
        printf("OFF\r\n"); 
}

void cmd_fsk_PreambleDetectorSize(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        fsk.RegPreambleDetect.bits.PreambleDetectorSize = i;
    }
    printf("PreambleDetectorSize:%d\r\n", fsk.RegPreambleDetect.bits.PreambleDetectorSize);
    radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet); 
}

void cmd_fsk_PreambleDetectorTol(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        fsk.RegPreambleDetect.bits.PreambleDetectorTol = i;
    }
    printf("PreambleDetectorTol:%d\r\n", fsk.RegPreambleDetect.bits.PreambleDetectorTol);                
    radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet);  
}

void cmd_PreambleSize(uint8_t idx)
{
    int i;
    if (radio.RegOpMode.bits.LongRangeMode) {
        if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
            sscanf(pcbuf+idx, "%d", &i);
            radio.write_u16(REG_LR_PREAMBLEMSB, i);
        }
        lora.RegPreamble = radio.read_u16(REG_LR_PREAMBLEMSB);
        printf("lora PreambleLength:%d\r\n", lora.RegPreamble);                
    } else {    
        if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
            sscanf(pcbuf+idx, "%d", &i);  
            radio.write_u16(REG_FSK_PREAMBLEMSB, i);
        }
        printf("PreambleSize:%d\r\n", radio.read_u16(REG_FSK_PREAMBLEMSB));
    }
}

void cmd_fsk_PreamblePolarity(uint8_t idx)
{
    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);
    fsk.RegSyncConfig.bits.PreamblePolarity ^= 1;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    if (fsk.RegSyncConfig.bits.PreamblePolarity)
        printf("0x55\r\n");
    else
        printf("0xaa\r\n");
}

void cmd_pllbw(uint8_t idx)
{
    RegPll_t pll;
    if (radio.type == SX1272) {
        // 0x5c and 0x5e registers
        pll.octet = radio.read_reg(REG_PLL_SX1272);
        if (pll.bits.PllBandwidth == 3)
            pll.bits.PllBandwidth = 0;
        else
            pll.bits.PllBandwidth++;
        radio.write_reg(REG_PLL_SX1272, pll.octet);
        pll.octet = radio.read_reg(REG_PLL_LOWPN_SX1272);
        if (pll.bits.PllBandwidth == 3)
            pll.bits.PllBandwidth = 0;
        else
            pll.bits.PllBandwidth++;
        radio.write_reg(REG_PLL_LOWPN_SX1272, pll.octet);                
    } else if (radio.type == SX1276) {
        // 0x70 register
        pll.octet = radio.read_reg(REG_PLL_SX1276);
        if (pll.bits.PllBandwidth == 3)
            pll.bits.PllBandwidth = 0;
        else
            pll.bits.PllBandwidth++;
        radio.write_reg(REG_PLL_SX1276, pll.octet);                  
    }
    switch (pll.bits.PllBandwidth) {
        case 0: printf("75"); break;
        case 1: printf("150"); break;
        case 2: printf("225"); break;
        case 3: printf("300"); break;
    }
    printf("KHz\r\n");
}

void cmd_lna_boost(uint8_t idx)
{
    radio.RegLna.octet = radio.read_reg(REG_LNA);
    if (radio.RegLna.bits.LnaBoostHF == 3)
        radio.RegLna.bits.LnaBoostHF = 0; 
    else 
        radio.RegLna.bits.LnaBoostHF++;
    radio.write_reg(REG_LNA, radio.RegLna.octet);            
    printf("LNA-boost:%d\r\n", radio.RegLna.bits.LnaBoostHF);
}

void cmd_LowDataRateOptimize(uint8_t idx)
{
    if (radio.type == SX1272) {
        lora.RegModemConfig.octet = radio.read_reg(REG_LR_MODEMCONFIG);
        lora.RegModemConfig.sx1272bits.LowDataRateOptimize ^= 1;
        printf("LowDataRateOptimize:%d\r\n", lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
        radio.write_reg(REG_LR_MODEMCONFIG, lora.RegModemConfig.octet);
    } else if (radio.type == SX1276) {
        lora.RegModemConfig3.octet = radio.read_reg(REG_LR_MODEMCONFIG3);
        lora.RegModemConfig3.sx1276bits.LowDataRateOptimize ^= 1;
        printf("LowDataRateOptimize:%d\r\n", lora.RegModemConfig3.sx1276bits.LowDataRateOptimize); 
        radio.write_reg(REG_LR_MODEMCONFIG3, lora.RegModemConfig3.octet);
    }
}

void cmd_fsk_FifoThreshold(uint8_t idx)
{
    int i;
    fsk.RegFifoThreshold.octet = radio.read_reg(REG_FSK_FIFOTHRESH);  
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        fsk.RegFifoThreshold.bits.FifoThreshold = i;
    }
    radio.write_reg(REG_FSK_FIFOTHRESH, fsk.RegFifoThreshold.octet);
    printf("FifoThreshold:%d\r\n", fsk.RegFifoThreshold.bits.FifoThreshold);
    fsk.RegFifoThreshold.octet = radio.read_reg(REG_FSK_FIFOTHRESH); 
}

void cmd_tx_ticker_rate(uint8_t idx)
{
    if (pcbuf[idx] != 0) {
        sscanf(pcbuf+idx, "%f", &tx_ticker_rate);
    }
    printf("tx_ticker_rate:%f\r\n", tx_ticker_rate);
}

void cmd_lora_tx_invert(uint8_t idx)
{
    lora.invert_tx(lora.RegTest33.bits.chirp_invert_tx);
    printf("chirp_invert_tx :%d\r\n", lora.RegTest33.bits.chirp_invert_tx);    
}
    
void cmd_lora_rx_invert(uint8_t idx)
{
    lora.invert_rx(!lora.RegTest33.bits.invert_i_q);
    printf("rx invert_i_q:%d\r\n", lora.RegTest33.bits.invert_i_q);    
}

void cmd_fsk_dcfree(uint8_t idx)
{
    fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
    if (fsk.RegPktConfig1.bits.DcFree == 3)
        fsk.RegPktConfig1.bits.DcFree = 0;
    else
        fsk.RegPktConfig1.bits.DcFree++;
    printf(" dcFree:");
    switch (fsk.RegPktConfig1.bits.DcFree) {
        case 0: printf("none "); break;
        case 1: printf("Manchester "); break;
        case 2: printf("Whitening "); break;
        case 3: printf("[41mreserved[0m "); break;
    }
    radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet); 
    printf("\r\n"); 
}

void cmd_bt(uint8_t idx)
{
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (radio.RegOpMode.bits.ModulationType != 0) {
        printf("!fsk\r\n");
        return;
    }
        
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        float bt;
        sscanf(pcbuf+idx, "%f", &bt);
        if (bt > 1.0)
            radio.RegOpMode.bits.ModulationShaping = 0;    // 0 = no shaping
        else if (bt > 0.6)
            radio.RegOpMode.bits.ModulationShaping = 1;    // 1 = BT1.0
        else if (bt > 0.4)
            radio.RegOpMode.bits.ModulationShaping = 2;    // 2 = BT0.5
        else
            radio.RegOpMode.bits.ModulationShaping = 3;    // 3 = BT0.3
    }
    radio.write_reg(REG_OPMODE, radio.RegOpMode.octet);
    switch (radio.RegOpMode.bits.ModulationShaping) {
        case 0: printf("no-shaping "); break;
        case 1: printf("BT1.0 "); break;
        case 2: printf("BT0.5 "); break;
        case 3: printf("BT0.3 "); break;
    }    
    printf("\r\n");
}

void cmd_fsk_DataMode(uint8_t idx)
{
    fsk.RegPktConfig2.word = radio.read_u16(REG_FSK_PACKETCONFIG2);
    fsk.RegPktConfig2.bits.DataModePacket ^= 1;
    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);  
    printf("datamode:");
    if (fsk.RegPktConfig2.bits.DataModePacket)
        printf("packet\r\n");
    else
        printf("continuous\r\n");
}

void cmd_show_dio(uint8_t idx)
{
    if (radio.RegOpMode.bits.LongRangeMode)
        lora_print_dio();
    else
        fsk_print_dio(); 
}

void cmd_set_dio(uint8_t idx)
{
    switch (pcbuf[idx]) {
        case '0':
            radio.RegDioMapping1.bits.Dio0Mapping++;
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
            break;
        case '1':
            radio.RegDioMapping1.bits.Dio1Mapping++;
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
            break;    
        case '2':
            radio.RegDioMapping1.bits.Dio2Mapping++;
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
            break;
        case '3':
            radio.RegDioMapping1.bits.Dio3Mapping++;
            radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
            break;
        case '4':
            radio.RegDioMapping2.bits.Dio4Mapping++;
            radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);
            break; 
        case '5':
            radio.RegDioMapping2.bits.Dio5Mapping++;
            radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);
            break;                                                                                             
    } // ...switch (pcbuf[idx])
    if (radio.RegOpMode.bits.LongRangeMode)
        lora_print_dio();
    else
        fsk_print_dio();
}

void cmd_mode_standby(uint8_t idx)
{
    radio.set_opmode(RF_OPMODE_STANDBY);
    printf("standby\r\n");
}
    
void cmd_mode_sleep(uint8_t idx)
{
    radio.set_opmode(RF_OPMODE_SLEEP);
    printf("sleep\r\n");
}

void cmd_mode_fstx(uint8_t idx)
    {
    radio.set_opmode(RF_OPMODE_SYNTHESIZER_TX);
    printf("fstx\r\n");
}

void cmd_mode_fsrx(uint8_t idx)
{
    radio.set_opmode(RF_OPMODE_SYNTHESIZER_RX);
    printf("fsrx\r\n");
}

void cmd_chat(uint8_t idx)
{
    app = APP_CHAT;
    lora.start_rx(RF_OPMODE_RECEIVER);
    printf("chat start\r\n");
}

void cmd_OokThreshType(uint8_t idx)
{
    fsk.RegOokPeak.octet = radio.read_reg(REG_FSK_OOKPEAK);
    if (fsk.RegOokPeak.bits.OokThreshType == 2)
        fsk.RegOokPeak.bits.OokThreshType = 0;
    else
        fsk.RegOokPeak.bits.OokThreshType++;

    radio.write_reg(REG_FSK_OOKPEAK, fsk.RegOokPeak.octet);
    printf("OokThreshType:");
    switch (fsk.RegOokPeak.bits.OokThreshType) {
        case 0: printf("fixed"); break;
        case 1: printf("peak"); break;
        case 2: printf("average"); break;
        case 3: printf("?"); break;
    }
    printf("\r\n");
}

void cmd_OokPeakTheshStep(uint8_t idx)
{
    float f;
    fsk.RegOokPeak.octet = radio.read_reg(REG_FSK_OOKPEAK);
    if (fsk.RegOokPeak.bits.OokPeakThreshStep == 7)
        fsk.RegOokPeak.bits.OokPeakThreshStep = 0;
    else
        fsk.RegOokPeak.bits.OokPeakThreshStep++;    

    radio.write_reg(REG_FSK_OOKPEAK, fsk.RegOokPeak.octet);
    switch (fsk.RegOokPeak.bits.OokPeakThreshStep) {
        case 0: f = 0.5; break;
        case 1: f = 1; break;
        case 2: f = 1.5; break;
        case 3: f = 2; break;
        case 4: f = 3; break;
        case 5: f = 4; break;
        case 6: f = 5; break;
        case 7: f = 6; break;                                                        
    }    
    printf("OokPeakThreshStep:%.1fdB\r\n", f); 
}

void cmd_OokFixedThresh(uint8_t idx)
{
    int i;
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%d", &i);
        radio.write_reg(REG_FSK_OOKFIX, i);
    }
    i = radio.read_reg(REG_FSK_OOKFIX);
    printf("OokFixedThreshold:%d\r\n", i);
} 

void cmd_clkout(uint8_t idx)
{
    RegOsc_t reg_osc;
    reg_osc.octet = radio.read_reg(REG_FSK_OSC);
    if (reg_osc.bits.ClkOut == 7)
        reg_osc.bits.ClkOut = 0;
    else
        reg_osc.bits.ClkOut++;
        
    printf("ClkOut:%d\r\n", reg_osc.bits.ClkOut);
    radio.write_reg(REG_FSK_OSC, reg_osc.octet);
}

void cmd_ook_tx_test(uint8_t idx)
{
    radio.set_frf_MHz(915.0);

    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    if (radio.RegOpMode.bits.LongRangeMode)
        cmd_toggle_modem(0);
        
    fsk.RegPktConfig1.octet = radio.read_reg(REG_FSK_PACKETCONFIG1);
    fsk.RegPktConfig1.bits.CrcOn = 0;
    radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);    
    
    radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    radio.RegDioMapping2.bits.Dio5Mapping = 2;  // Data
    radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);
    
    radio.RegDioMapping1.octet = radio.read_reg(REG_DIOMAPPING1);
    radio.RegDioMapping1.bits.Dio3Mapping = 1;  // TxReady
    radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet);
    
    //radio.write_reg(REG_FSK_SYNCCONFIG, 0);
    cmd_ook(0);
    radio.write_reg(REG_FSK_SYNCCONFIG, 0);
/*    radio.write_u16(REG_FSK_PREAMBLEMSB, 4);    // preamble length
    
    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);
    fsk.RegSyncConfig.bits.SyncOn = 1;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);

    //              0123456
    sprintf(pcbuf, "syncw a9 66 69 65");
    cmd_fsk_syncword(6);*/
    tx_ticker.attach(&callback_ook_tx_test, tx_ticker_rate);
}
     
void cmd_help(uint8_t args_at);

typedef enum {
    MODEM_BOTH,
    MODEM_FSK,
    MODEM_LORA
} modem_e;

typedef struct {
    modem_e modem;
    const char* const cmd;
    void (*handler)(uint8_t args_at);
    const char* const arg_descr;
    const char* const description;
} menu_item_t;

const menu_item_t menu_items[] = 
{   /* after first character, command names must be [A-Za-z] */
    { MODEM_BOTH, "chat", cmd_chat, "","start keyboard chat"}, 
    { MODEM_BOTH, "rssi", cmd_read_current_rssi, "","(RX) read instantaneous RSSI"}, 
    { MODEM_BOTH, "prssi", cmd_rssi_polling, "<%d>","dbm of rssi polling, 0 = off"}, 
    { MODEM_BOTH, "txpd", cmd_per_tx_delay, "<%d>","get/set PER tx delay (in milliseconds)"},
    { MODEM_BOTH, "pertx", cmd_pertx, "<%d pkt count>","start Eiger PER TX"},
    { MODEM_BOTH, "perrx", cmd_perrx, "","start Eiger PER RX"},
    { MODEM_BOTH, "pres", cmd_PreambleSize, "<%d>", "get/set PreambleSize"},
    { MODEM_BOTH, "pllbw", cmd_pllbw, "", "increment pllbw"},
    { MODEM_BOTH, "lnab", cmd_lna_boost, "", "(RX) increment LNA boost"},
    { MODEM_BOTH, "stby", cmd_mode_standby, "", "set chip mode to standby"},
    { MODEM_BOTH, "sleep", cmd_mode_sleep, "", "set chip mode to sleep"},
    { MODEM_BOTH, "fstx", cmd_mode_fstx, "", "set chip mode to fstx"},
    { MODEM_BOTH, "fsrx", cmd_mode_fsrx, "", "set chip mode to fsrx"}, 
    { MODEM_BOTH, "crcon", cmd_crcOn, "","toggle crcOn"},
    { MODEM_BOTH, "ethcrc", cmd_crc32, "","toggle enable software crc32"},
    { MODEM_BOTH, "spif", cmd_spifreq, "<MHz>","change SPI clock frequency"},
    { MODEM_BOTH, "payl", cmd_payload_length, "<%d>","get/set payload length"},   
    { MODEM_BOTH, "bgr", cmd_bgr, "<%d>","(TX) get/set reference for TX DAC"},
    { MODEM_BOTH, "ocp", cmd_ocp, "<%d>","(TX) get/set milliamps current limit"},
    { MODEM_BOTH, "frf", cmd_frf, "<MHz>","get/set RF center frequency"},
    { MODEM_BOTH, "pas", cmd_paSelect, "","(TX) toggle RFO/PA_BOOST"},
    { MODEM_BOTH, "pid", cmd_per_id, "<%d>","get/set ID number in Eiger PER packet"},
    { MODEM_BOTH, "dio", cmd_show_dio, "","print dio mapping"},         
    
    { MODEM_FSK, "clkout", cmd_clkout, "","increment ClkOut divider"}, 
    { MODEM_FSK, "ookt", cmd_OokThreshType, "","(RX) increment OokThreshType"}, 
    { MODEM_FSK, "ooks", cmd_OokPeakTheshStep, "","(RX) increment OokPeakTheshStep"}, 
    { MODEM_FSK, "sqlch", cmd_OokFixedThresh, "<%d>","(RX) get/set OokFixedThresh"}, 
    { MODEM_FSK, "rssit", cmd_rssi_threshold, "<-dBm>","(RX) get/set rssi threshold"}, 
    { MODEM_FSK, "rssis", cmd_rssi_smoothing, "<%d>","(RX) get/set rssi smoothing"}, 
    { MODEM_FSK, "rssio", cmd_rssi_offset, "<%d>","(RX) get/set rssi offset"},
    { MODEM_FSK, "mods", cmd_mod_shaping, "", "(TX) increment modulation shaping"},  
    { MODEM_FSK, "agcauto", cmd_fsk_agcauto, "", "(RX) toggle AgcAutoOn"},
    { MODEM_FSK, "afcauto", cmd_fsk_afcauto, "", "(RX) toggle AfcAutoOn"},
    { MODEM_FSK, "syncw", cmd_fsk_syncword, "<hex bytes>", "get/set syncword"},
    { MODEM_FSK, "syncon", cmd_fsk_syncOn, "", "toggle SyncOn (frame sync, SFD enable)"},
    { MODEM_FSK, "bitsync", cmd_fsk_bitsync, "", "toggle BitSyncOn (continuous mode only)"},
    { MODEM_FSK, "fifot", cmd_fsk_TxStartCondition, "", "(TX) toggle TxStartCondition"},
    { MODEM_FSK, "pktf", cmd_fsk_PacketFormat, "", "toggle PacketFormat fixed/variable length"},
    { MODEM_FSK, "poll", cmd_poll_irq_en, "", "toggle poll_irq_en"},
    { MODEM_FSK, "prep", cmd_fsk_PreamblePolarity, "", "toggle PreamblePolarity"},
    { MODEM_FSK, "datam", cmd_fsk_DataMode, "", "toggle DataMode (packet/continuous)"},    
    { MODEM_FSK, "rxt", cmd_rx_trigger, "","(RX) increment RxTrigger"},
    { MODEM_FSK, "ook", cmd_ook, "","enter OOK mode"},
    { MODEM_FSK, "otx", cmd_ook_tx_test, "","start ook tx repeat"},
    { MODEM_FSK, "fei", cmd_fsk_read_fei, "","(RX) read FEI"},
    { MODEM_FSK, "fdev", cmd_fsk_fdev, "<kHz>","(TX) get/set fdev"},
    { MODEM_FSK, "par", cmd_paRamp, "","(TX) increment paRamp"},
    { MODEM_FSK, "pde", cmd_fsk_PreambleDetectorOn, "","(RX) toggle PreambleDetectorOn"},
    { MODEM_FSK, "pds", cmd_fsk_PreambleDetectorSize, "<%d>","(RX) get/set PreambleDetectorSize"},
    { MODEM_FSK, "pdt", cmd_fsk_PreambleDetectorTol, "<%d>","(RX) get/set PreambleDetectorTol"},
    { MODEM_FSK, "thr", cmd_fsk_FifoThreshold, "<%d>","get/set FifoThreshold"},
    { MODEM_FSK, "dcf", cmd_fsk_dcfree, "","(RX) increment DcFree"},
    { MODEM_FSK, "br", cmd_fsk_bitrate, "<%f kbps>","get/set bitrate"},
    { MODEM_FSK, "ac", cmd_AfcClear, "","(RX) AfcClear"},
    { MODEM_FSK, "ar", cmd_fsk_AutoRestartRxMode, "","(RX) increment AutoRestartRxMode"},
    { MODEM_FSK, "alc", cmd_fsk_AfcAutoClearOn, "","(RX) toggle AfcAutoClearOn"},
    { MODEM_FSK, "mp", cmd_MapPreambleDetect, "","(RX) toggle MapPreambleDetect"},
    { MODEM_FSK, "rrx", cmd_restart_rx, "","restart RX"},  
    { MODEM_BOTH, "op", cmd_op, "<dBm>","(TX) get/set TX power"}, 
    { MODEM_FSK, "bt", cmd_bt, "","get/set BT"},  
    { MODEM_FSK, "ltx", cmd_long_tx, "<%d>","long tx"},
    
#ifdef LORA_TX_TEST
    { MODEM_LORA, "apl", cmd_lora_all_payload_lengths, "","(TXTEST) sweep payload lengths 0->255"},
    { MODEM_LORA, "csn", cmd_lora_sync_sweep, "[12]","(TXTEST) sweep ppg symbol"},
    { MODEM_LORA, "ss", cmd_lora_sync_lo_nibble, "","(TXTEST) ppg low nibble"},
    { MODEM_LORA, "cpl", lora_cycle_payload_length, "[%d stop]","(TXTEST) sweep payload length"},
    { MODEM_LORA, "ro", cmd_lora_data_ramp, "[%d bytes]","(TXTEST) sweep payload data"},
    { MODEM_LORA, "ccr", cmd_lora_cycle_codingrates, "","(TXTEST) cycle coding rates"},
    { MODEM_LORA, "fps", cmd_lora_fixed_payload_symbol, "[symbol_num n_bits]","(TXTEST) sweep symbol, n_bits=bits per symbol set (sf8=24, sf9=28, etc)"},    
    { MODEM_LORA, "fpo", cmd_fixed_payload_offset, "<nbytes>","(TXTEST) padding offset for fp tests"},
    { MODEM_LORA, "fp", cmd_lora_fixed_payload, "[bytes]","(TXTEST) fixed payload"},
    { MODEM_LORA, "tab", cmd_lora_toggle_all_bits, "[byte length]","(TXTEST) toggle all bits"},
    { MODEM_LORA, "tcrc", cmd_lora_toggle_crcOn, "","(TXTEST) toggle crcOn"},
    { MODEM_LORA, "thm", cmd_lora_toggle_header_mode, "","(TXTEST) toggle explicit/implicit"},
#endif /* LORA_TX_TEST */ 
    
    { MODEM_BOTH, "ttr", cmd_tx_ticker_rate, "<%f seconds>","(TXTEST) get/set tx_ticker rate"},
    { MODEM_LORA, "cadper", cmd_cadper, "","Eiger PER RX using CAD" },
    { MODEM_LORA, "cad", cmd_cad, "<%d num tries>","(RX) run channel activity detection" },
    { MODEM_LORA, "iqinv", cmd_lora_rx_invert, "","(RX) toggle RX IQ invert" },
    { MODEM_LORA, "cin", cmd_lora_tx_invert, "","(TX) toggle TX IQ invert" },   
    { MODEM_LORA, "lhp", cmd_hop_period, "<%d>","(RX) get/set hop period"},
    { MODEM_LORA, "sync", cmd_lora_ppg, "<%x>","get/set sync (post-preamble gap)"},
    { MODEM_LORA, "cr", cmd_codingRate, "<1-4>","get/set codingRate"},
    { MODEM_LORA, "lhm", cmd_lora_header_mode, "","toggle explicit/implicit"},
    { MODEM_LORA, "vh", cmd_lora_poll_validHeader, "","toggle polling of validHeader"},
    { MODEM_LORA, "sf", cmd_lora_sf, "<%d>","get/set spreadingFactor"}, 
    { MODEM_LORA, "ldr", cmd_LowDataRateOptimize, "","toggle LowDataRateOptimize"}, 
    { MODEM_LORA, "txc", cmd_lora_continuous_tx, "","(TX) toggle TxContinuousMode"},
    { MODEM_BOTH, "tx", cmd_tx, "<%d>","transmit packet. optional payload length"},    
    { MODEM_BOTH, "bw", cmd_bandwidth, "<kHz>","get/set bandwith"},
    { MODEM_LORA, "rxt", cmd_rx_timeout, "<%d>","(RX) get/set SymbTimeout"},
    { MODEM_LORA, "rxs", cmd_rx_single, "","start RX_SINGLE"},
    { MODEM_BOTH, "rx", cmd_rx, "","start RX"},   
        
    { MODEM_BOTH, "h", cmd_hw_reset, "","hardware reset"},
    { MODEM_BOTH, "i", cmd_init, "","initialize radio driver"},
    { MODEM_BOTH, "R", cmd_read_all_regs, "","read all radio registers"},
    { MODEM_BOTH, "r", cmd_radio_reg_read, "[%x]","read single radio register"},
    { MODEM_BOTH, "w", cmd_radio_reg_write, "[%x %x]","write single radio register"},

    { MODEM_BOTH, "L", cmd_toggle_modem, "","toggle between LoRa / FSK"},   
    { MODEM_FSK, "E", cmd_empty_fifo, "","empty out FIFO"}, 
    { MODEM_FSK, "c", cmd_fsk_test_case, "<%d>","get/set test cases"},
    { MODEM_BOTH, "d", cmd_set_dio, "<%d pin num>","increment dio mapping"},
    { MODEM_BOTH, ".", cmd_print_status, "","print status"},
    { MODEM_BOTH, "?", cmd_help, "","this list of commands"},
    { MODEM_BOTH, NULL, NULL, NULL, NULL }
};

void cmd_help(uint8_t args_at)
{
    int i;
    
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        if (menu_items[i].modem == MODEM_BOTH)
            printf("%s%s\t%s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
    }
    
    if (radio.RegOpMode.bits.LongRangeMode) {
        for (i = 0; menu_items[i].cmd != NULL ; i++) {
            if (menu_items[i].modem == MODEM_LORA)
                printf("%s%s\t(LoRa) %s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);            
        }
    } else {
        for (i = 0; menu_items[i].cmd != NULL ; i++) {
            if (menu_items[i].modem == MODEM_FSK)
                printf("%s%s\t(FSK) %s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
        }
    }
}

void
console()
{
    int i;
    uint8_t user_cmd_len;
    
    if (poll_irq_en)
        poll_service_radio();
    else
        service_radio();    
        
    if (pcbuf_len < 0) {
        printf("abort\r\n");
        rx_payloadReady_int_en = false;
        cadper_enable = false;
        per_en = false;
        pcbuf_len = 0;
        if ((radio.RegOpMode.bits.Mode != RF_OPMODE_SLEEP) && (radio.RegOpMode.bits.Mode != RF_OPMODE_STANDBY)) {
            radio.set_opmode(RF_OPMODE_STANDBY);
        }
        on_txdone_state = ON_TXDONE_STATE_NONE;
        tx_ticker.detach();
        return;
    }
    if (pcbuf_len == 0)
        return;
        
    printf("\r\n");
        
    /* get end of user-entered command */
    user_cmd_len = 1;   // first character can be any character
    for (i = 1; i <= pcbuf_len; i++) {
        if (pcbuf[i] < 'A' || (pcbuf[i] > 'Z' && pcbuf[i] < 'a') || pcbuf[i] > 'z') {
            user_cmd_len = i;
            break;
        }
    }

    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        int mi_len = strlen(menu_items[i].cmd);
        if (radio.RegOpMode.bits.LongRangeMode) {
            if (menu_items[i].modem == MODEM_FSK)
                continue;  // FSK commands not used in LoRa
        } else {
            if (menu_items[i].modem == MODEM_LORA)
                continue;  // LoRa commands not used in FSK            
        }

        if (menu_items[i].handler && user_cmd_len == mi_len && (strncmp(pcbuf, menu_items[i].cmd, mi_len) == 0)) {
            while (pcbuf[mi_len] == ' ')   // skip past spaces
                mi_len++;
            menu_items[i].handler(mi_len);
            break;
        }
    }
   
    pcbuf_len = 0;
    printf("> ");
    fflush(stdout); 
}

void rx_callback()
{
    static uint8_t pcbuf_idx = 0;
    static uint8_t prev_len = 0;;
    char c = pc.getc();
    /*if (kermit.uart_rx_enabled) {
        kermit.rx_callback(c);
    } else*/ {
        if (c == 8) {
            if (pcbuf_idx > 0) {
                pc.putc(8);
                pc.putc(' ');
                pc.putc(8);
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
        }/* else if (c == SOH) {
            kermit.uart_rx_enable();
        }*/ else if (pcbuf_idx < sizeof(pcbuf)) {
            pcbuf[pcbuf_idx++] = c;
            pc.putc(c);
        }
    }
}

int main()
{    
#if defined(TARGET_NUCLEO_L152RE) && defined(USE_DEBUGGER)
    DBGMCU_Config(DBGMCU_SLEEP,   ENABLE);
    DBGMCU_Config(DBGMCU_STOP,    ENABLE);
    DBGMCU_Config(DBGMCU_STANDBY, ENABLE);
#endif

    pc.baud(57600);
    printf("\r\nmain()\r\n");
    
    pc.attach(rx_callback);
    
    make_crc_table();
    
#if !defined(TARGET_MTS_MDOT_F411RE) && !defined(TARGET_DISCO_L072CZ_LRWAN1)
    rfsw.input();
    if (rfsw.read()) {
        shield_type = SHIELD_TYPE_LAS;
        printf("LAS\r\n");
    } else {
        shield_type = SHIELD_TYPE_MAS;
        printf("MAS\r\n");
    }
    
    rfsw.output();
#endif /* !TARGET_MTS_MDOT_F411RE */    
    radio.rf_switch = rfsw_callback;
    
#ifdef FSK_PER
    fsk.enable(false);
    fsk.RegSyncConfig.octet = radio.read_reg(REG_FSK_SYNCCONFIG);    
    fsk.RegSyncConfig.bits.SyncSize = 2;
    radio.write_reg(REG_FSK_SYNCCONFIG, fsk.RegSyncConfig.octet);
    radio.write_reg(REG_FSK_SYNCVALUE3, 0x90);
    radio.write_reg(REG_FSK_SYNCVALUE2, 0x4e);
    radio.write_reg(REG_FSK_SYNCVALUE1, 0x63);  
    
    fsk.RegPreambleDetect.bits.PreambleDetectorOn = 1;
    fsk.RegPreambleDetect.bits.PreambleDetectorSize = 1;
    fsk.RegPreambleDetect.bits.PreambleDetectorTol = 10;
    radio.write_reg(REG_FSK_PREAMBLEDETECT, fsk.RegPreambleDetect.octet);     
    
    fsk.RegRxConfig.octet = radio.read_reg(REG_FSK_RXCONFIG);
    fsk.RegRxConfig.bits.AfcAutoOn = 1;
    fsk.RegRxConfig.bits.AgcAutoOn = 1;
    fsk.RegRxConfig.bits.RxTrigger = 7;
    radio.write_reg(REG_FSK_RXCONFIG, fsk.RegRxConfig.octet);   
    
    radio.set_opmode(RF_OPMODE_STANDBY);
    fsk.set_rx_dcc_bw_hz(41666, 1);  // afcbw
    fsk.set_rx_dcc_bw_hz(20833, 0);  // rxbw
    
    fsk.set_tx_fdev_hz(10000);
    
    radio.write_u16(REG_FSK_PREAMBLEMSB, 8);
    
    fsk.RegPktConfig2.bits.PayloadLength = 64;
    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);    
    
    radio.RegDioMapping2.octet = radio.read_reg(REG_DIOMAPPING2);
    radio.RegDioMapping2.bits.Dio5Mapping = 2;  // data output to observation
    radio.RegDioMapping2.bits.Dio4Mapping = 3;  // output preamble detect indication
    radio.RegDioMapping2.bits.MapPreambleDetect = 1;
    radio.write_reg(REG_DIOMAPPING2, radio.RegDioMapping2.octet);    
    
    RegPreambleDetect.bits.PreambleDetectorOn = 1;
    RegPreambleDetect.bits.PreambleDetectorSize = 1;
    RegPreambleDetect.bits.PreambleDetectorTol = 10;
    write_reg(REG_FSK_PREAMBLEDETECT, RegPreambleDetect.octet);  
                
#endif /* FSK_PER */

#ifdef START_EIGER_TX
    uint8_t addr;
    radio.set_frf_MHz(915.0);
    
    radio.RegOcp.octet = radio.read_reg(REG_OCP);    
    radio.RegOcp.bits.OcpTrim = 20;
    radio.write_reg(REG_OCP, radio.RegOcp.octet);
    
    RegPdsTrim1_t pds_trim;
    if (radio.type == SX1276)
        addr = REG_PDSTRIM1_SX1276;
    else
        addr = REG_PDSTRIM1_SX1272;
        
    pds_trim.octet = radio.read_reg(addr);
    pds_trim.bits.prog_txdac = 7;
    radio.write_reg(addr, pds_trim.octet);          
                  
#ifndef FSK_PER
    lora.enable();
    lora.setSf(10);
    if (radio.type == SX1276)
        lora.setBw(7);  // 7 == 125khz
#endif
    
    toggle_per_en();
    PacketTxCnt = 0;
    per_timeout.attach(&per_cb, per_tx_delay);
#endif /* START_EIGER_TX */

#ifdef START_EIGER_RX

    radio.set_frf_MHz(915.0);
    radio.RegPaConfig.bits.PaSelect = 1;    // 0: use RFO for sx1276 shield,  1==PA_BOOST
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);   
    
    toggle_per_en();
    PacketNormalCnt = 0;
    PacketRxSequencePrev = 0;
    PacketPerKoCnt = 0;
    PacketPerOkCnt = 0;       
    
#ifndef FSK_PER
    lora.enable();
    lora.setSf(10); 
    if (radio.type == SX1276)
        lora.setBw(7);  // 7 == 125khz
    lora.start_rx();
#else
    fsk.start_rx();
    radio.RegDioMapping1.bits.Dio2Mapping = 3;  // dio2 to syncadrs
    radio.write_reg(REG_DIOMAPPING1, radio.RegDioMapping1.octet); 
    
    if (radio.HF) {
        fsk.RegRssiConfig.octet = radio.read_reg(REG_FSK_RSSICONFIG);
        fsk.RegRssiConfig.bits.RssiOffset = FSK_RSSI_OFFSET;
        fsk.RegRssiConfig.bits.RssiSmoothing = FSK_RSSI_SMOOTHING;        
        radio.write_reg(REG_FSK_RSSICONFIG, fsk.RegRssiConfig.octet);
    }
#endif

    if (radio.HF) {
        radio.RegLna.bits.LnaBoostHF = 3;
        radio.write_reg(REG_LNA, radio.RegLna.octet);
    }

#endif /* START_EIGER_RX */

    (void)radio.get_frf_MHz();
    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
#if defined(TARGET_MTS_MDOT_F411RE)
    radio.RegPaConfig.bits.PaSelect = 1;    // mDot uses PA_BOOST
#else
    if (shield_type == SHIELD_TYPE_LAS) {
        // LAS HF=PA_BOOST  LF=RFO
        if (radio.HF)
            radio.RegPaConfig.bits.PaSelect = 1;
        else
            radio.RegPaConfig.bits.PaSelect = 0;
    } else if (shield_type == SHIELD_TYPE_MAS) {
        // MAS HF=RFO  LF=RFO
        radio.RegPaConfig.bits.PaSelect = 0;        
    }
#endif
    radio.RegPaConfig.bits.OutputPower = 15;
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
      
#ifdef START_OOK_TX_TEST
    cmd_ook_tx_test(0);
#endif /* START_OOK_TX_TEST */

    while(1) {
        switch (app) {
            case APP_NONE:
                console();
                break;
            case APP_CHAT:
                console_chat();
                break;
        } // ...switch (app)

#if TARGET_NUCLEO_L152RE        
        //sleep();
#endif
    } // ...while(1)
}

