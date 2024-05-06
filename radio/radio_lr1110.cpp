#include "radio.h"
#ifdef SX1265_H 
#include "SPIu.h"

#ifdef TARGET_FF_ARDUINO
    SPI spi(D11, D12, D13); // mosi, miso, sclk
                   //spi, nss, busy, dio9, nreset
    SX1265 Radio::radio(spi, D7, D3, D5, A0);

    static void initRfSwDIO()
    {
        /*stat_t stat;*/
        uint8_t dioBuf[8];
        /* antenna truth table
         * V1    V2    port
         *  0     0    shutdown
         *  1     0    J2   rx RFI
         *  0     1    J1   HP tx RFO
         *  1     1    J3   LP tx RFO
         * DIO5  DIO6
         */
        dioBuf[  DIO_en_IDX] = DIO5_BIT | DIO6_BIT;
        dioBuf[DIO_stby_IDX] = 0;
        dioBuf[  DIO_rx_IDX] = DIO5_BIT;
        dioBuf[  DIO_tx_IDX] = DIO5_BIT | DIO6_BIT;
        dioBuf[DIO_txhp_IDX] = DIO6_BIT;
        dioBuf[DIO_gnss_IDX] = 0;
        dioBuf[DIO_wifi_IDX] = 0;
        /*stat.word =*/ Radio::radio.xfer(OPCODE_SET_DIO_AS_RFSWITCH, 8, 0, dioBuf);
    }
#else
    /* declare pins for form-factor */
    #error non-ardiuno-form-factor
#endif /* TARGET_FF_ARDUINO */

uint8_t gfsk_pp_buf[9];
uint8_t gfsk_mp_buf[10];
uint8_t lora_pp_buf[6];
uint8_t lora_mp_buf[4];

const RadioEvents_t* RadioEvents;
LowPowerTimer Radio::lpt;
volatile us_timestamp_t Radio::irqAt;
uint8_t Radio::loraTimeoutSymbols;

void Radio::LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    /*stat_t stat;*/
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    radio.to_big_endian16(preambleLen, lora_pp_buf); 
    lora_pp_buf[2] = fixLen;
    /* lora_pp_buf[3] =  set when txing, initialized to zero in Init */
    lora_pp_buf[4] = crcOn;
    lora_pp_buf[5] = invIQ;
    /*stat.word =*/ radio.xfer(OPCODE_SET_PACKET_PARAM, 6, 0, lora_pp_buf);
}

void Radio::set_tx_dbm(int8_t dbm)
{
    /*stat_t stat;*/
    uint8_t buf[4];
    int8_t txpower;
    unsigned PaSel, RegPaSupply, PaDutyCycle, PaHPSel;

    if (dbm > 20) {
        txpower = 22;
        PaSel = 1;
        RegPaSupply = 1;
        PaDutyCycle = 4;
        PaHPSel = 7;
    } else if (dbm > 17) {
        txpower = 22;
        PaSel = 1;
        RegPaSupply = 1;
        PaDutyCycle = 2;
        PaHPSel = 7;
    } else if (dbm > 15) {
        txpower = 22;
        PaSel = 1;
        RegPaSupply = 1;
        PaDutyCycle = 4;
        PaHPSel = 3;
    } else {
        txpower = 14;
        PaSel = 0;
        RegPaSupply = 0;
        if (dbm > 14)
            PaDutyCycle = 7;
        else if (dbm > 10)
            PaDutyCycle = 4;
        else
            PaDutyCycle = 0;
        PaHPSel = 0;
    }

    buf[0] = PaSel;
    buf[1] = RegPaSupply;
    buf[2] = PaDutyCycle;
    buf[3] = PaHPSel;
    /*stat.word =*/ radio.xfer(OPCODE_SET_PA_CONFIG, 4, 0, buf);

    buf[0] = txpower;
    buf[1] = 4;
    /*stat.word =*/ radio.xfer(OPCODE_SET_TXPARAMS, 2, 0, buf);
}

void Radio::SetChannel(unsigned hz)
{
    /*stat_t stat;*/
    uint8_t buf[4];
    radio.to_big_endian32(hz, buf);
    /*stat.word =*/ radio.xfer(OPCODE_SET_RF_FREQ_HZ, 4, 0, buf);
}

void Radio::Standby()
{
    /*stat_t stat;*/
    uint8_t buf = 0;    // 0=rc, 1=xosc
    /*stat.word =*/ radio.xfer(OPCODE_SET_STANDBY, 1, 0, &buf);
}

void Radio::LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t cr)
{
    /*stat_t stat;*/
    uint8_t buf[4];
    uint8_t ldro = 0;

    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    if (bwKHz > 500)
        buf[1] = LORA_BW_1000KHz;
    else if (bwKHz > 250)
        buf[1] = LORA_BW_500KHz;
    else if (bwKHz > 125) {
        buf[1] = LORA_BW_250KHz;
    } else if (bwKHz > 80) {
        buf[1] = LORA_BW_125KHz;
        if (sf > 11)
            ldro = 1;
    } else if (bwKHz > 16) {
        buf[1] = LORA_BW_31_25KHz;
        if (sf > 10)
            ldro = 1;
    } else if (bwKHz > 8) {
        buf[1] = LORA_BW_15_6KHz;
        if (sf > 9)
            ldro = 1;
    } else {
        buf[1] = LORA_BW_7_8KHz;
        if (sf > 8)
            ldro = 1;
    }

    buf[0] = sf;
    buf[2] = cr;
    buf[3] = ldro;
    /*stat.word =*/ radio.xfer(OPCODE_SET_MODULATION, 4, 0, buf);
}

int Radio::Send(uint8_t size, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh)
{
    /*stat_t stat;*/
    uint8_t pktType;

    pktType = radio.getPacketType();
    if (pktType == PACKET_TYPE_LORA) {
        lora_pp_buf[3] = size;
        /*stat.word =*/ radio.xfer(OPCODE_SET_PACKET_PARAM, 6, 0, lora_pp_buf);
    } else if (pktType == PACKET_TYPE_GFSK) {
        gfsk_pp_buf[6] = size;
        /*stat.word =*/ radio.xfer(OPCODE_SET_PACKET_PARAM, 9, 0, gfsk_pp_buf);
    }

    /* TODO: LBT */

    radio.start_tx(size);
    return 0;
}

void Radio::service()
{
    irq_t irq;
    irq.dword = radio.service();
    if (irq.dword != 0) {
    }
}

bool Radio::init_irq;
void Radio::my_irq_handler()
{
    /* radio irq immediately after hardware reset: hf_xosc start due to TCXO */
    init_irq = true;
}

void Radio::dio9_top_half()
{
    irqAt = lpt.read_us();

    if (RadioEvents->DioPin_top_half)
        RadioEvents->DioPin_top_half();

    if (radio.chipMode == CHIPMODE_TX) {
        /* TxDone handling requires low latency */
        if (RadioEvents->TxDone_topHalf) {
            RadioEvents->TxDone_topHalf();
        } 
    } else {
#ifdef RX_INDICATION
        RX_INDICATION = 0;
#endif
    }
}

void Radio::timeout_callback(bool tx)
{
    if (!tx) {
        if (RadioEvents->RxTimeout)
            RadioEvents->RxTimeout();
    } // else TODO tx timeout
}

void Radio::rx_done(uint8_t size, float rssi, float snr)
{
    RadioEvents->RxDone(size, rssi, snr);
}

void Radio::txDoneBottom()
{
    if (RadioEvents->TxDone_botHalf)
        RadioEvents->TxDone_botHalf();
}

void Radio::Init(const RadioEvents_t* e, unsigned spi_hz)
{
    uint64_t a, b;
    /* initialize payload length to zero in packet params */
    lora_pp_buf[3] = 0;
    gfsk_pp_buf[6] = 0;

    radio.txDone = txDoneBottom;
    radio.rxDone = rx_done;
    radio.timeout = timeout_callback;
    //radio.chipModeChange = chipModeChange;

    RadioEvents = e;
    lpt.start();

    spi.frequency(spi_hz);

    radio.dio9_topHalf = my_irq_handler;
    init_irq = false;
    radio.hw_reset();
    a = Kernel::get_ms_count();
    do {
        b = Kernel::get_ms_count();
        /* wait for radio interrupt at hardware reset, failure to start hf_xosc */
        if (init_irq) {
            radio.service();
            break;
        }
    } while ((b - a) < 30);

    radio.dio9_topHalf = dio9_top_half;

    initRfSwDIO();
}

void Radio::SetLoRaSymbolTimeout(uint16_t symbs)
{
    /*stat_t stat;*/
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    loraTimeoutSymbols = symbs;
    /*stat.word =*/ radio.xfer(OPCODE_SET_LORA_SYNC_TIMEOUT, 1, 0, &loraTimeoutSymbols);
}

void Radio::Rx(unsigned timeout)
{
    /*stat_t stat;*/
    uint8_t buf[3];
    unsigned rx_timeout;

    if (radio.getPacketType() == PACKET_TYPE_LORA) {
        /*stat.word =*/ radio.xfer(OPCODE_SET_LORA_SYNC_TIMEOUT, 1, 0, &loraTimeoutSymbols);
    }

    if (timeout == 0)
        rx_timeout = 0xffffff;  // receive until instructed otherwise
    else
        rx_timeout = timeout;

    radio.to_big_endian24(rx_timeout, buf);
    /*stat.word =*/ radio.xfer(OPCODE_SET_RX, 3, 0, buf);

}

#endif /* ..SX126x_H */
