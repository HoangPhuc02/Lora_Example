#include "radio.h"
#ifdef SX127x_H 

#ifdef DEVICE_LPTICKER
LowPowerTimer Radio::lpt;
LowPowerTimeout TxTimeoutEvent;
#else
Timer Radio::lpt;
Timeout TxTimeoutEvent;
#endif

#if (MBED_MAJOR_VERSION < 6)
volatile us_timestamp_t Radio::irqAt;
#else
using namespace std::chrono;
LowPowerClock::time_point Radio::irqAt;
#endif

void Radio::Sleep()
{
    radio.set_opmode(RF_OPMODE_SLEEP);
}

void Radio::Standby()
{
    radio.set_opmode(RF_OPMODE_STANDBY);
}

bool Radio::CheckRfFrequency(unsigned hz)
{
    return true;
}

void Radio::SetChannel(unsigned hz)
{
    radio.set_frf_MHz(hz / 1000000.0);
}

float Radio::getFrfMHz()
{
    return radio.get_frf_MHz();
}

void SX1272OnTimeoutIrq( void )
{
    Radio::radio.set_opmode(RF_OPMODE_STANDBY);
}

void Radio::SetTxContinuousWave(unsigned hz, int8_t dbm, unsigned timeout_us)
{
    fsk.enable(true);
    fsk.RegPktConfig2.word = radio.read_u16(REG_FSK_PACKETCONFIG2);
    fsk.RegPktConfig2.bits.DataModePacket = 0; // continuous mode
    radio.write_u16(REG_FSK_PACKETCONFIG2, fsk.RegPktConfig2.word);
    fsk.set_tx_fdev_hz(0);  // unmodulated carrier, aka dead carrier
    SetChannel(hz);
    set_tx_dbm(dbm);

    if (timeout_us != 0) {
#if (MBED_MAJOR_VERSION < 6)
        TxTimeoutEvent.attach_us(SX1272OnTimeoutIrq, timeout_us);
#else
        TxTimeoutEvent.attach(SX1272OnTimeoutIrq, microseconds(timeout_us));
#endif
    }

    radio.set_opmode(RF_OPMODE_TRANSMITTER);
}

#define LORA_MAC_PRIVATE_SYNCWORD                   0x12
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34
void Radio::SetPublicNetwork(bool en)
{
    radio.write_reg(REG_LR_SYNC_BYTE, en ? LORA_MAC_PUBLIC_SYNCWORD : LORA_MAC_PRIVATE_SYNCWORD);
}

uint32_t Radio::Random(void)
{
    uint32_t ret = 0;
    unsigned i;

    radio.set_opmode(RF_OPMODE_RECEIVER);
    for (i = 0; i < 32; i++) {
        uint32_t r;
        wait_us(3000);
        r = radio.read_reg(REG_LR_WIDEBAND_RSSI);
        r <<= ((i & 7) << 2);
        ret ^= r;
    }

    return ret;
}

void Radio::LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    lora.RegPreamble = preambleLen;
    radio.write_u16(REG_LR_PREAMBLEMSB, lora.RegPreamble);

    if (radio.type == SX1276) {
        lora.RegModemConfig.sx1276bits.ImplicitHeaderModeOn = fixLen;
        lora.RegModemConfig2.sx1276bits.RxPayloadCrcOn = crcOn;
        radio.write_reg(REG_LR_MODEMCONFIG2, lora.RegModemConfig2.octet);
    } else if (radio.type == SX1272) {
        lora.RegModemConfig.sx1272bits.ImplicitHeaderModeOn = fixLen;
        lora.RegModemConfig.sx1272bits.RxPayloadCrcOn = crcOn;
    }

    radio.write_reg(REG_LR_MODEMCONFIG, lora.RegModemConfig.octet);

    lora.invert_tx(invIQ);
    lora.invert_rx(invIQ);
}

void Radio::GFSKModemConfig(unsigned bps, unsigned bw_hz, unsigned fdev_hz)
{
    if (radio.RegOpMode.bits.LongRangeMode)
        fsk.enable(false);

    fsk.set_bitrate(bps);

    fsk.set_rx_dcc_bw_hz(bw_hz, 0);
    fsk.set_rx_dcc_bw_hz(bw_hz * 1.5, 1);

    fsk.set_tx_fdev_hz(fdev_hz);
}


void Radio::GFSKPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn)
{
    if (radio.RegOpMode.bits.LongRangeMode)
        fsk.enable(false);

    radio.write_u16(REG_FSK_PREAMBLEMSB, preambleLen);

    fsk.RegPktConfig1.bits.PacketFormatVariable = fixLen ? 0 : 1;
    fsk.RegPktConfig1.bits.CrcOn = crcOn;
    radio.write_reg(REG_FSK_PACKETCONFIG1, fsk.RegPktConfig1.octet);
}

void Radio::SetLoRaSymbolTimeout(uint16_t symbs)
{
    if (!radio.RegOpMode.bits.LongRangeMode)
        lora.enable();

    radio.write_reg(REG_LR_SYMBTIMEOUTLSB, symbs & 0xff);
    symbs >>= 8;
    lora.RegModemConfig2.sx1272bits.SymbTimeoutMsb = symbs;
    radio.write_reg(REG_LR_MODEMCONFIG2, lora.RegModemConfig2.octet);
}

void Radio::LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t coderate)
{
    float sp;
    if (!radio.RegOpMode.bits.LongRangeMode)
        lora.enable();

    lora.RegModemConfig2.sx1276bits.SpreadingFactor = sf;
    radio.write_reg(REG_LR_MODEMCONFIG2, lora.RegModemConfig2.octet);

    lora.setBw_KHz(bwKHz);

    if (radio.type == SX1276) {
        lora.RegModemConfig.sx1276bits.CodingRate = coderate;

        sp = lora.get_symbol_period();
        if (sp > 16)
            lora.RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            lora.RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;

        radio.write_reg(REG_LR_MODEMCONFIG3, lora.RegModemConfig3.octet);
    } else if (radio.type == SX1272) {
        lora.RegModemConfig.sx1272bits.CodingRate = coderate;

        if (lora.get_symbol_period() > 16)
            lora.RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            lora.RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
    }
    radio.write_reg(REG_LR_MODEMCONFIG, lora.RegModemConfig.octet);
}

void Radio::SetRxMaxPayloadLength(uint8_t max)
{
    if (radio.RegOpMode.bits.LongRangeMode)
        radio.write_reg(REG_LR_RX_MAX_PAYLOADLENGTH, max);
    else
        radio.write_reg(REG_FSK_PAYLOADLENGTH, max);
}

void Radio::SetFixedPayloadLength(uint8_t len)
{
    lora.RegPayloadLength = len;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
}

const RadioEvents_t* RadioEvents;


volatile struct pe {
    uint8_t dio0 : 1;
    uint8_t dio1 : 1;
    uint8_t txing : 1;
} pinEvent;

void
Radio::dio0UserContext()
{
    service_action_e act = lora.service();

    if (!pinEvent.txing) {
        if (act == SERVICE_READ_FIFO && RadioEvents->RxDone) {
            int8_t rssi;
            float snr = lora.RegPktSnrValue / 4.0;
            
            rssi = lora.get_pkt_rssi();
            if (snr < 0)
                rssi += snr;
            RadioEvents->RxDone(lora.RegRxNbBytes, rssi, snr);
        } 
    } else if (act == SERVICE_TX_DONE) {
        if (RadioEvents->TxDone_botHalf)
            RadioEvents->TxDone_botHalf();
    }
}

void Radio::dio0isr()
{
#if (MBED_MAJOR_VERSION < 6)
    irqAt = lpt.read_us();
#else
    irqAt = LowPowerClock::now();
#endif

    if (RadioEvents->DioPin_top_half)
        RadioEvents->DioPin_top_half();
 
    if (pinEvent.txing) {
        /* TxDone handling requires low latency */
        if (RadioEvents->TxDone_topHalf)
            RadioEvents->TxDone_topHalf();    // TODO in callback read irqAt for timestamp of interrupt

    }

    pinEvent.dio0 = 1;
}

void Radio::dio1UserContext()
{
    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);

    if (RadioEvents->RxTimeout)
        RadioEvents->RxTimeout();

    radio.write_reg(REG_LR_IRQFLAGS, 0x80); // ensure RxTimeout is cleared
}

void Radio::dio1isr()
{
    pinEvent.dio1 = 1;

    if (RadioEvents->DioPin_top_half)
        RadioEvents->DioPin_top_half();
}

void Radio::Init(const RadioEvents_t* e, unsigned spi_hz)
{
    radio.m_spi.frequency(spi_hz);

    while (radio.dio0.read() || radio.dio1.read()) {
        radio.write_reg(REG_LR_IRQFLAGS, 0xff); // clear stagnant interrupt
    }
    dio0.rise(dio0isr);
    dio1.rise(dio1isr);

    radio.rf_switch = rfsw_callback;
    boardInit();

    RadioEvents = e;
    lpt.start();
}

float Radio::GetRssiInst()
{
    return lora.get_current_rssi();
}

int Radio::Send(uint8_t size, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh)
{
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_SLEEP) {
        radio.set_opmode(RF_OPMODE_STANDBY);
        wait_us(1000);
    }
    radio.write_reg(REG_LR_IRQFLAGS, 0x08); // ensure TxDone is cleared
    lora.RegPayloadLength = size;
    radio.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);

    if (maxListenTime > 0) {
        int rssi;
        us_timestamp_t startAt, chFreeAt, now;
        lora.start_rx(RF_OPMODE_RECEIVER);
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
            rssi = lora.get_current_rssi();
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
            rssi = lora.get_current_rssi();
            if (rssi > rssiThresh) {
                goto Lstart;
            }
        } while ((now - chFreeAt) < channelFreeTime);
    }

    lora.start_tx(size);
    pinEvent.txing = 1;

    return 0;
}

void Radio::Rx(unsigned timeout)
{
    if (timeout == 0) {
        lora.start_rx(RF_OPMODE_RECEIVER);
    } else {
        lora.start_rx(RF_OPMODE_RECEIVER_SINGLE);
    }

    pinEvent.txing = 0;
}


void Radio::ocp(uint8_t ma)
{
    if (ma < 130)
        radio.RegOcp.bits.OcpTrim = (ma - 45) / 5;
    else
        radio.RegOcp.bits.OcpTrim = (ma + 30) / 10;
    radio.write_reg(REG_OCP, radio.RegOcp.octet);
   
    radio.RegOcp.octet = radio.read_reg(REG_OCP);
    if (radio.RegOcp.bits.OcpTrim < 16)
        ma = 45 + (5 * radio.RegOcp.bits.OcpTrim);
    else if (radio.RegOcp.bits.OcpTrim < 28)
        ma = (10 * radio.RegOcp.bits.OcpTrim) - 30;
    else
        ma = 240;
}


#if 0
void Radio::PrintStatus()
{
#ifdef MAC_DEBUG
    radio.RegOpMode.octet = radio.read_reg(REG_OPMODE);
    switch (radio.RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: printf("SLEEP "); break;
        case RF_OPMODE_STANDBY: printf("STBY "); break;
        case RF_OPMODE_SYNTHESIZER_TX: printf("FSTX "); break;
        case RF_OPMODE_TRANSMITTER: printf("TX "); break;
        case RF_OPMODE_SYNTHESIZER_RX: printf("FSRX "); break;
        case RF_OPMODE_RECEIVER: printf("RXC "); break;
        case RF_OPMODE_RECEIVER_SINGLE: printf("RXS "); break;
        case RF_OPMODE_CAD: printf("CAD "); break;
    }

    printf("dio:%u:%u opmode:%02x %.2fMHz sf%ubw%u ", radio.dio0.read(), radio.dio1.read(), radio.RegOpMode.octet, radio.get_frf_MHz(), lora.getSf(), lora.getBw());
    lora.RegIrqFlags.octet = radio.read_reg(REG_LR_IRQFLAGS);
    printf("irqFlags:%02x\r\n", lora.RegIrqFlags.octet);
#endif /* MAC_DEBUG */
}
#endif /* if 0 */

#ifdef DUTY_ENABLE
us_timestamp_t
Radio::TimeOnAir(RadioModems_t m, uint8_t pktLen)
{
    uint32_t airTime = 0;

    switch (m)
    {
    case MODEM_FSK:
        {
            /* TODO
            airTime = round( ( 8 * ( SX1272.Settings.Fsk.PreambleLen +
                                     ( ( SX1272Read( REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( SX1272.Settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( SX1272Read( REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( SX1272.Settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     SX1272.Settings.Fsk.Datarate ) * 1e3 );
                                     */
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            uint8_t fixLen, bandwidth, LowDatarateOptimize, coderate, crcOn ;
            if (radio.type == SX1276) {
                coderate = lora.RegModemConfig.sx1276bits.CodingRate;
                LowDatarateOptimize = lora.RegModemConfig3.sx1276bits.LowDataRateOptimize;
                fixLen = lora.RegModemConfig.sx1276bits.ImplicitHeaderModeOn;
                bandwidth = lora.RegModemConfig.sx1276bits.Bw - 7;
                crcOn = lora.RegModemConfig2.sx1276bits.RxPayloadCrcOn;
            } else if (radio.type == SX1272) {
                coderate = lora.RegModemConfig.sx1272bits.CodingRate;
                LowDatarateOptimize = lora.RegModemConfig.sx1272bits.LowDataRateOptimize;
                fixLen = lora.RegModemConfig.sx1272bits.ImplicitHeaderModeOn;
                bandwidth = lora.RegModemConfig.sx1272bits.Bw;
                crcOn = lora.RegModemConfig.sx1272bits.RxPayloadCrcOn;
            } else
                return 0;

            switch( bandwidth )
            {
            case 0: // 125 kHz
                bw = 125;//ms: 125e3;
                break;
            case 1: // 250 kHz
                bw = 250;//ms:250e3;
                break;
            case 2: // 500 kHz
                bw = 500;//ms:500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << lora.RegModemConfig2.sx1276bits.SpreadingFactor );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble;
            tPreamble = (lora.RegPreamble + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * lora.RegModemConfig2.sx1276bits.SpreadingFactor +
                                 28 + 16 * crcOn -
                                 ( fixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * ( lora.RegModemConfig2.sx1276bits.SpreadingFactor -
                                 ( ( LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                                 ( coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return ms secs
            airTime = floor( tOnAir * 1e3 + 0.999 );
        }
        break;
    }
    return airTime;

}
#endif /* DUTY_ENABLE */

void Radio::service()
{
    if (pinEvent.dio0) {
        dio0UserContext();
        pinEvent.txing = 0;
        pinEvent.dio0 = 0;
    } else if (radio.dio0.read()) {
        /* fail: missed interrupt */
        dio0isr();
    }

    if (pinEvent.dio1) {
        dio1UserContext();
        pinEvent.dio1 = 0;
    }
}

uint32_t Radio::lora_toa_us( uint8_t pktLen )
{
    uint32_t airTime = 0;
    uint8_t chipBW = Radio::lora.getBw();
    double bwKHz;
    uint8_t LowDataRateOptimize;
    bool FixLen;
    uint8_t crcOn;
    uint16_t preambleLen = Radio::radio.read_u16(REG_LR_PREAMBLEMSB);

    Radio::lora.RegModemConfig2.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG2);
    Radio::lora.RegModemConfig.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG);

    if (Radio::radio.type == SX1276) {
        chipBW -= 7;
        Radio::lora.RegModemConfig3.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG3);
        LowDataRateOptimize = Radio::lora.RegModemConfig3.sx1276bits.LowDataRateOptimize;
        FixLen = Radio::lora.RegModemConfig.sx1276bits.ImplicitHeaderModeOn;
        crcOn = Radio::lora.RegModemConfig2.sx1276bits.RxPayloadCrcOn;
    } else if (Radio::radio.type == SX1272) {
        LowDataRateOptimize = Radio::lora.RegModemConfig.sx1272bits.LowDataRateOptimize;
        FixLen = Radio::lora.RegModemConfig.sx1272bits.ImplicitHeaderModeOn;
        crcOn = Radio::lora.RegModemConfig.sx1272bits.RxPayloadCrcOn;
    } else
        return 0;

    switch (chipBW) {
        case 0: bwKHz = 125; break;
        case 1: bwKHz = 250; break;
        case 2: bwKHz = 500; break;
        default: return 0;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bwKHz / ( 1 << Radio::lora.RegModemConfig2.sx1276bits.SpreadingFactor );
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = ( preambleLen + 4.25 ) * ts;
    // Symbol length of payload and time
    double tmp = ceil( ( 8 * pktLen - 4 * Radio::lora.RegModemConfig2.sx1276bits.SpreadingFactor +
                         28 + 16 * crcOn -
                         ( FixLen ? 20 : 0 ) ) /
                         ( double )( 4 * ( Radio::lora.RegModemConfig2.sx1276bits.SpreadingFactor -
                         ( ( LowDataRateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                         ( Radio::lora.getCodingRate(false) + 4 );
    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return microseconds
    airTime = floor( tOnAir * 1000 + 0.999 );

    return airTime;
}
#endif /* ..SX127x_H */

