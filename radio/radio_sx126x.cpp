#include "radio.h"
#ifdef SX126x_H 
#include "SPIu.h"

#ifdef DEVICE_LPTICKER
LowPowerTimer Radio::lpt;
#else
Timer Radio::lpt;
#endif

#if (MBED_MAJOR_VERSION < 6)
volatile us_timestamp_t Radio::irqAt;
#else
LowPowerClock::time_point Radio::irqAt;
#endif

bool Radio::paOff;

#ifdef TARGET_FF_ARDUINO
#ifdef TARGET_DISCO_L072CZ_LRWAN1
    /* murata type1SJ */
    SPIu spi(PB_15, PB_14, PB_13); // mosi, miso, sclk
                   //spi, nss, busy, dio1
    SX126x Radio::radio(spi, PB_12, PC_2, PB_0);

    DigitalOut antswPower(PA_15);
    void Radio::chipModeChange() { }
    const uint8_t chipType = CHIP_TYPE_SX1262;
#else
    /* sx126x arduino shield */
    SPIu spi(D11, D12, D13); // mosi, miso, sclk
                   //spi, nss, busy, dio1
    SX126x Radio::radio(spi, D7, D3, D5);

    DigitalOut antswPower(D8);
    AnalogIn xtalSel(A3);

    DigitalIn Radio::chipType(A2);

    #define PINNAME_NRST            A0

    #define LED_ON      1
    #define LED_OFF     0
    DigitalOut tx_led(A4);
    DigitalOut rx_led(A5);

    void Radio::chipModeChange()
    {
        if (radio.chipMode == CHIPMODE_NONE) {
            tx_led = LED_OFF;
            rx_led = LED_OFF;
        } else if (radio.chipMode == CHIPMODE_TX) {
            tx_led = LED_ON;
            rx_led = LED_OFF;
        } else if (radio.chipMode == CHIPMODE_RX) {
            tx_led = LED_OFF;
            rx_led = LED_ON;
        }
    }
#endif /* !TARGET_DISCO_L072CZ_LRWAN1 */
#endif /* TARGET_FF_ARDUINO */

const RadioEvents_t* RadioEvents;
PacketParams_t Radio::pp;
RadioModems_t Radio::_m_;
uint8_t Radio::loraTimeoutSymbols;

#if defined(TARGET_FF_MORPHO) && !defined(TARGET_DISCO_L072CZ_LRWAN1)
    DigitalOut pc3(PC_3);   // debug RX indication, for nucleo boards
    #define RX_INDICATION       pc3
#endif /* TARGET_FF_MORPHO */

void Radio::Rx(unsigned timeout)
{
    antswPower = 1;

    {
        uint8_t buf[8];
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.RxDone = 1;
        irqEnable.bits.Timeout = 1;

        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        radio.xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }

#ifdef RX_INDICATION
    RX_INDICATION = 1;
#endif
    if (timeout == 0) {
        uint8_t symbs = 0;
        if (radio.getPacketType() == PACKET_TYPE_LORA) // shut off timeout
            radio.xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &symbs);

        radio.start_rx(RX_TIMEOUT_CONTINUOUS);
    } else {
        if (radio.getPacketType() == PACKET_TYPE_LORA)
            radio.xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &loraTimeoutSymbols);

        radio.start_rx(timeout * RC_TICKS_PER_US);
    }
}

void Radio::Standby()
{
    radio.setStandby(STBY_RC);  // STBY_XOSC

    antswPower = 0;
}

void Radio::Sleep()
{
    radio.setSleep(true, false);

    antswPower = 0;
}

void Radio::set_tx_dbm(int8_t dbm)
{
    unsigned v = radio.readReg(REG_ADDR_ANACTRL16, 1);

    if (dbm == PA_OFF_DBM) {
        /* bench test: prevent overloading receiving station (very low tx power) */
        if ((v & 0x10) == 0) {
            v |= 0x10;
            radio.writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
        paOff = true;
    } else {
        radio.set_tx_dbm(chipType == CHIP_TYPE_SX1262, dbm);
        if (v & 0x10) {
            v &= ~0x10;
            radio.writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
        paOff = false;
    }
}

void Radio::SetTxContinuousWave(unsigned hz, int8_t dbm, unsigned timeout_us)
{
    SetChannel(hz);
    radio.set_tx_dbm(chipType == CHIP_TYPE_SX1262, dbm);
    radio.xfer(OPCODE_SET_TX_CARRIER, 0, 0, NULL);
}

uint32_t Radio::Random(void)
{
    uint32_t ret;

    radio.start_rx(RX_TIMEOUT_CONTINUOUS);

    ret = radio.readReg(REG_ADDR_RANDOM, 4);

    Standby();

    return ret;
}

bool Radio::CheckRfFrequency(unsigned hz)
{
    return true;
}

void Radio::SetChannel(unsigned hz)
{
    radio.setMHz(hz / 1000000.0);
}

float Radio::getFrfMHz()
{
    return radio.getMHz();
}

void Radio::LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    pp.lora.PreambleLengthHi = preambleLen >> 8;
    pp.lora.PreambleLengthLo = preambleLen;
    pp.lora.HeaderType = fixLen;
    pp.lora.CRCType = crcOn;
    pp.lora.InvertIQ = invIQ;

    radio.xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
}

void Radio::GFSKModemConfig(unsigned bps, unsigned bw_hz, unsigned fdev_hz)
{
    ModulationParams_t mp;
    uint32_t u32;

    if (radio.getPacketType() != PACKET_TYPE_GFSK)
        radio.setPacketType(PACKET_TYPE_GFSK);

    u32  = 32 * (XTAL_FREQ_HZ / bps);
    mp.gfsk.bitrateHi = u32 >> 16; // param1
    mp.gfsk.bitrateMid = u32 >> 8; // param2
    mp.gfsk.bitrateLo = u32;       // param3
    mp.gfsk.PulseShape = GFSK_SHAPE_BT1_0; // param4
    // param5:
    if (bw_hz < 5800)
        mp.gfsk.bandwidth = GFSK_RX_BW_4800;
    else if (bw_hz < 7300)
        mp.gfsk.bandwidth = GFSK_RX_BW_5800;
    else if (bw_hz < 9700)
        mp.gfsk.bandwidth = GFSK_RX_BW_7300;
    else if (bw_hz < 11700)
        mp.gfsk.bandwidth = GFSK_RX_BW_9700;
    else if (bw_hz < 14600)
        mp.gfsk.bandwidth = GFSK_RX_BW_11700;
    else if (bw_hz < 19500)
        mp.gfsk.bandwidth = GFSK_RX_BW_14600;
    else if (bw_hz < 23400)
        mp.gfsk.bandwidth = GFSK_RX_BW_19500;
    else if (bw_hz < 29300)
        mp.gfsk.bandwidth = GFSK_RX_BW_23400;
    else if (bw_hz < 39000)
        mp.gfsk.bandwidth = GFSK_RX_BW_29300;
    else if (bw_hz < 46900)
        mp.gfsk.bandwidth = GFSK_RX_BW_39000;
    else if (bw_hz < 58600)
        mp.gfsk.bandwidth = GFSK_RX_BW_46900;
    else if (bw_hz < 78200)
        mp.gfsk.bandwidth = GFSK_RX_BW_58600;
    else if (bw_hz < 93800)
        mp.gfsk.bandwidth = GFSK_RX_BW_78200;
    else if (bw_hz < 117300)
        mp.gfsk.bandwidth = GFSK_RX_BW_93800;
    else if (bw_hz < 156200)
        mp.gfsk.bandwidth = GFSK_RX_BW_117300;
    else if (bw_hz < 187200)
        mp.gfsk.bandwidth = GFSK_RX_BW_156200;
    else if (bw_hz < 234300)
        mp.gfsk.bandwidth = GFSK_RX_BW_187200;
    else if (bw_hz < 312000)
        mp.gfsk.bandwidth = GFSK_RX_BW_234300;
    else if (bw_hz < 373600)
        mp.gfsk.bandwidth = GFSK_RX_BW_312000;
    else if (bw_hz < 467000)
        mp.gfsk.bandwidth = GFSK_RX_BW_373600;
    else
        mp.gfsk.bandwidth = GFSK_RX_BW_467000;

    if (fdev_hz > 0) {
        u32 = fdev_hz / FREQ_STEP;
        mp.gfsk.fdevHi = u32 >> 16; // param6
        mp.gfsk.fdevMid = u32 >> 8;    // param7
        mp.gfsk.fdevLo = u32; // param8
    }

    radio.xfer(OPCODE_SET_MODULATION_PARAMS, 8, 0, mp.buf);
}

void Radio::GFSKPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn)
{
    if (radio.getPacketType() != PACKET_TYPE_GFSK)
        radio.setPacketType(PACKET_TYPE_GFSK);

    pp.gfsk.PreambleLengthHi = preambleLen >> 8;
    pp.gfsk.PreambleLengthLo = preambleLen;
    pp.gfsk.PreambleDetectorLength = GFSK_PREAMBLE_DETECTOR_LENGTH_16BITS;
    pp.gfsk.SyncWordLength = 24; // 0xC194C1
    pp.gfsk.AddrComp = 0;
    pp.gfsk.PacketType = fixLen;
    if (crcOn)
        pp.gfsk.CRCType = GFSK_CRC_2_BYTE;
    else
        pp.gfsk.CRCType = GFSK_CRC_OFF;

    //TODO pp.gfsk.PayloadLength = ;

    radio.xfer(OPCODE_SET_PACKET_PARAMS, 8, 0, pp.buf);
}

void Radio::LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t cr)
{
    ModulationParams_t mp;
    float khz, sp;

    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    if (bwKHz > 250) {
        mp.lora.bandwidth = LORA_BW_500;
        khz = 500;
    } else if (bwKHz > 125) {
        mp.lora.bandwidth = LORA_BW_250;
        khz = 250;
    } else if (bwKHz > 63) {
        mp.lora.bandwidth = LORA_BW_125;
        khz = 125;
    } else if (bwKHz > 42) {
        mp.lora.bandwidth = LORA_BW_62;
        khz = 62.5;
    } else if (bwKHz > 32) {
        mp.lora.bandwidth = LORA_BW_41;
        khz = 41.67;
    } else if (bwKHz > 21) {
        mp.lora.bandwidth = LORA_BW_31;
        khz = 31.25;
    } else if (bwKHz > 16) {
        mp.lora.bandwidth = LORA_BW_20;
        khz = 20.83;
    } else if (bwKHz > 11) {
        mp.lora.bandwidth = LORA_BW_15;
        khz = 15.625;
    } else if (bwKHz > 11) {
        mp.lora.bandwidth = LORA_BW_10;
        khz = 10.42;
    } else {
        mp.lora.bandwidth = LORA_BW_7;
        khz = 7.81;
    }

    mp.lora.spreadingFactor = sf;
    mp.lora.codingRate = cr;

    sp = (1 << mp.lora.spreadingFactor) / khz;
    /* TCXO dependent */
    if (sp > 16)
        mp.lora.LowDatarateOptimize = 1; // param4
    else
        mp.lora.LowDatarateOptimize = 0; // param4

    radio.xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mp.buf);

}

void Radio::SetLoRaSymbolTimeout(uint16_t symbs)
{
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    loraTimeoutSymbols = symbs;
    radio.xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &loraTimeoutSymbols);
}

float Radio::GetRssiInst()
{
    uint8_t buf[8];

    radio.xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
    return buf[1] / -2.0;
}

int Radio::Send(uint8_t size, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh)
{
    uint8_t buf[8];
    uint8_t pktType = radio.getPacketType();

    buf[0] = 0; // TX base address
    buf[1] = 0; // RX base address
    radio.xfer(OPCODE_SET_BUFFER_BASE_ADDR, 2, 0, buf);

    if (pktType == PACKET_TYPE_GFSK) {
        pp.gfsk.PayloadLength = size;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 8, 0, pp.buf);
    } else if (pktType == PACKET_TYPE_LORA) {
        pp.lora.PayloadLength = size;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
    }

    {
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.TxDone = 1;
        irqEnable.bits.Timeout = 1;

        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        radio.xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }

    antswPower = 1;

    if (maxListenTime > 0) {
        int rssi;
        us_timestamp_t startAt, chFreeAt, now;
        uint8_t symbs = 0;

        radio.xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &symbs);

        radio.start_rx(RX_TIMEOUT_CONTINUOUS);
    #if (MBED_MAJOR_VERSION < 6)
        startAt = lpt.read_us();
    #else
        startAt = LowPowerClock::now().time_since_epoch().count();
    #endif
Lstart:
        do {
    #if (MBED_MAJOR_VERSION < 6)
            now = lpt.read_us();
    #else
            now = LowPowerClock::now().time_since_epoch().count();
    #endif
            if ((now - startAt) > maxListenTime) {
                return -1;
            }
            radio.xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
            rssi = buf[1] / -2;
        } while (rssi > rssiThresh);
    #if (MBED_MAJOR_VERSION < 6)
        chFreeAt = lpt.read_us();
    #else
        chFreeAt = LowPowerClock::now().time_since_epoch().count();
    #endif
        do {
    #if (MBED_MAJOR_VERSION < 6)
            now = lpt.read_us();
    #else
            now = LowPowerClock::now().time_since_epoch().count();
    #endif
            radio.xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
            rssi = buf[1] / -2;
            if (rssi > rssiThresh) {
                goto Lstart;
            }
        } while ((now - chFreeAt) < channelFreeTime);
    } 

    if (paOff) {
        unsigned v = radio.readReg(REG_ADDR_ANACTRL16, 1);
        if ((v & 0x10) == 0) {
            v |= 0x10;
            radio.writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
    }
    radio.start_tx(size);

    return 0;
} // ..Send()

void Radio::SetRxMaxPayloadLength(uint8_t max)
{
    uint8_t pktType = radio.getPacketType();

    if (pktType == PACKET_TYPE_GFSK) {
        pp.gfsk.PayloadLength = max;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 8, 0, pp.buf);
    } else if (pktType == PACKET_TYPE_LORA) {
        pp.lora.PayloadLength = max;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
    }
}

void Radio::dio1_top_half()
{
#if (MBED_MAJOR_VERSION < 6)
    irqAt = lpt.read_us();
#else
    irqAt = LowPowerClock::now();
#endif

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
#ifdef RX_INDICATION
        RX_INDICATION = 0;
#endif
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

void to_big_endian24(uint32_t in, uint8_t *out)
{
    out[2] = in & 0xff;
    in >>= 8;
    out[1] = in & 0xff;
    in >>= 8;
    out[0] = in & 0xff;
}

void Radio::Init(const RadioEvents_t* e, unsigned spi_hz)
{
    radio.txDone = txDoneBottom;
    radio.rxDone = rx_done;
    radio.timeout = timeout_callback;
    radio.chipModeChange = chipModeChange;
    radio.dio1_topHalf = dio1_top_half;

    RadioEvents = e;
    lpt.start();

    spi.frequency(spi_hz);
    radio.SetDIO2AsRfSwitchCtrl(1);
#ifdef TARGET_DISCO_L072CZ_LRWAN1
    /* murata type1SJ */
    {
        unsigned tcxoDelayTicks = 3200;
        uint8_t buf[4];

        radio.hw_reset(PB_1);
        ThisThread::sleep_for(50);

        buf[0] = 1; // 1 = 1.7v
        to_big_endian24(tcxoDelayTicks, buf+1);

        radio.xfer(OPCODE_SET_DIO3_AS_TCXO_CTRL, 4, 0, buf);
    }
#endif
}

void Radio::service()
{
    radio.service();
}

void Radio::SetPublicNetwork(bool en)
{
    uint16_t ppg;

    if (en)
        ppg = 0x3444;
    else
        ppg = 0x1424;

    radio.writeReg(REG_ADDR_LORA_SYNC, ppg, 2);
}

uint32_t Radio::lora_toa_us( uint8_t pktLen )
{
    double bwKHz;
    unsigned preambleLen;
    ModulationParams_t mp;

    {
        loraConfig1_t conf1;
        conf1.octet = radio.readReg(REG_ADDR_LORA_CONFIG1, 1);
        mp.lora.LowDatarateOptimize = conf1.bits.ppm_offset;
        pp.lora.HeaderType = conf1.bits.implicit_header;
        pp.lora.InvertIQ = conf1.bits.rx_invert_iq;
        mp.lora.codingRate = conf1.bits.tx_coding_rate;
    }

    {
        loraConfig2_t conf2;
        conf2.octet = radio.readReg(REG_ADDR_LORA_CONFIG2, 1);
        pp.lora.CRCType = conf2.bits.tx_payload_crc16_en;
    }


    {
        uint32_t val;
        val = radio.readReg(REG_ADDR_LORA_PREAMBLE_SYMBNB, 2);
        pp.lora.PreambleLengthHi = val >> 8;
        pp.lora.PreambleLengthLo = val;
    }

    preambleLen = (pp.lora.PreambleLengthHi << 8) + pp.lora.PreambleLengthLo;

    {
        loraConfig0_t conf0;
        conf0.octet = radio.readReg(REG_ADDR_LORA_CONFIG0, 1);
        mp.lora.spreadingFactor = conf0.bits.modem_sf;
        mp.lora.bandwidth = conf0.bits.modem_bw;
    }

    switch (mp.lora.bandwidth) {
        case LORA_BW_7: bwKHz = 7.81; break;
        case LORA_BW_10: bwKHz = 10.42; break;
        case LORA_BW_15: bwKHz = 15.625; break;
        case LORA_BW_20: bwKHz = 20.83; break;
        case LORA_BW_31: bwKHz = 31.25; break;
        case LORA_BW_41: bwKHz = 41.67; break;
        case LORA_BW_62: bwKHz = 62.5; break;
        case LORA_BW_125: bwKHz = 125; break;
        case LORA_BW_250: bwKHz = 250; break;
        case LORA_BW_500: bwKHz = 500; break;
        default: bwKHz = 0; break;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bwKHz / ( 1 << mp.lora.spreadingFactor );
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = ( preambleLen + 4.25 ) * ts;
    // Symbol length of payload and time
    
    double tmp = ceil( ( 8 * pktLen - 4 * mp.lora.spreadingFactor +
                         28 + 16 * pp.lora.CRCType -
                         ( pp.lora.HeaderType ? 20 : 0 ) ) /
                         ( double )( 4 * ( mp.lora.spreadingFactor -
                         ( ( mp.lora.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                         ( mp.lora.codingRate + 4 );
    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return microseconds
    return floor( tOnAir * 1000 + 0.999 );
}

#if 0
void Radio::PrintStatus()
{
/*    uint8_t buf[4];
    status_t status;
    IrqFlags_t irqFlags;
    radio.xfer(OPCODE_GET_IRQ_STATUS, 0, 3, buf);
    irqFlags.word = buf[1] << 8;
    irqFlags.word |= buf[2];

    printf("dio1:%u  irqFlags:%04x\r\n", radio.getDIO1(), irqFlags.word);
    radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
    radio.PrintChipStatus(status);*/
    {
        loraConfig1_t conf1;
        conf1.octet = radio.readReg(REG_ADDR_LORA_CONFIG1, 1);
        printf("ldro%u %s %s cr%u\r\n",
            conf1.bits.ppm_offset,
            conf1.bits.implicit_header ? "fixed" : "var",
            conf1.bits.rx_invert_iq ? "inv" : "std",
            conf1.bits.tx_coding_rate
        );
    }

    {
        loraConfig2_t conf2;
        conf2.octet = radio.readReg(REG_ADDR_LORA_CONFIG2, 1);
        printf("crc16en:%u ", conf2.bits.tx_payload_crc16_en);
    }


    {
        uint32_t val;
        val = radio.readReg(REG_ADDR_LORA_PREAMBLE_SYMBNB, 2);
        printf("prelen %lu ", val);
    }

    {
        loraConfig0_t conf0;
        conf0.octet = radio.readReg(REG_ADDR_LORA_CONFIG0, 1);
        printf("sf%u, bw%u ", conf0.bits.modem_sf, conf0.bits.modem_bw);
    }

    printf("%.3fMHz\r\n", radio.getMHz());
}
#endif /* if 0 */

#endif /* ..SX126x_H */
