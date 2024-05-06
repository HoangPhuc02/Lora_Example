#include "radio.h"
#ifdef SX128x_H 
#include "SPIu.h"
#include <float.h>

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

#ifdef TARGET_FF_ARDUINO    /* pins of SX126xDVK1xAS board */
    #define NRST_PIN        A0
    SPIu spi(D11, D12, D13); // mosi, miso, sclk
    //           spi, nss, busy, dio1
    SX128x Radio::radio(spi,  D7,   D3,   D5, NRST_PIN);

    #define LED_ON      1
    #define LED_OFF     0
    DigitalOut tx_led(A4);
    DigitalOut rx_led(A5);


    DigitalOut ant_sw(A3);
    DigitalOut cps(D6); // SE2436L

    bool fe_enable; // SE2436L

    void Radio::chipModeChange()
    {
        if (radio.chipMode == CHIPMODE_NONE) {
            cps = 0;
            tx_led = LED_OFF;
            rx_led = LED_OFF;
        } else if (radio.chipMode == CHIPMODE_TX) {
            cps = fe_enable;
            tx_led = LED_ON;
            rx_led = LED_OFF;
        } else if (radio.chipMode == CHIPMODE_RX) {
            cps = fe_enable;
            tx_led = LED_OFF;
            rx_led = LED_ON;
        }
    }
#endif /* TARGET_FF_ARDUINO */

#ifdef TARGET_FF_MORPHO
    DigitalOut pc3(PC_3);   // debug RX indication, for nucleo boards
    #define RX_INDICATION       pc3
#endif /* TARGET_FF_MORPHO */


PacketParams_t Radio::ppGFSK;
PacketParams_t Radio::ppLORA;
PacketParams_t Radio::ppFLRC;

ModulationParams_t Radio::mpBLE_GFSK;
ModulationParams_t Radio::mpFLRC;
ModulationParams_t Radio::mpLORA;

const RadioEvents_t* RadioEvents;

unsigned Radio::symbolPeriodUs;
unsigned Radio::nSymbs;
unsigned Radio::rxTimeoutMs;

void Radio::readChip()
{
    uint8_t reg8;

    reg8 = radio.readReg(REG_ADDR_PKTCTRL0, 1);
    ppGFSK.gfskFLRC.HeaderType = reg8 & 0x20;
    ppFLRC.gfskFLRC.HeaderType = reg8 & 0x20;

    reg8 = radio.readReg(REG_ADDR_PKTCTRL1, 1);
    ppGFSK.gfskFLRC.PreambleLength = reg8 & 0x70;
    ppFLRC.gfskFLRC.PreambleLength = reg8 & 0x70;
    ppGFSK.gfskFLRC.SyncWordLength = reg8 & 0x0e;
    ppFLRC.gfskFLRC.SyncWordLength = reg8 & 0x06;
    if (ppFLRC.gfskFLRC.SyncWordLength == 0x06)
        ppFLRC.gfskFLRC.SyncWordLength = FLRC_SYNC_WORD_LEN_P32S;

    reg8 = radio.readReg(REG_ADDR_PKT_SYNC_ADRS_CTRL, 1);
    ppGFSK.gfskFLRC.SyncWordMatch = reg8 & 0x70;
    ppFLRC.gfskFLRC.SyncWordMatch = reg8 & 0x70;

    reg8 = radio.readReg(REG_ADDR_PAYLOAD_LEN, 1);
    ppGFSK.gfskFLRC.PayloadLength = reg8;
    ppFLRC.gfskFLRC.PayloadLength = reg8;

    reg8 = radio.readReg(REG_ADDR_PKT_TX_HEADER, 1);    // TODO hi bit of payload length
    //ppBLE.ble.ConnectionState = reg8 & 0xe0;
    //ppBLE.ble.BleTestPayload = reg8 & 0x1c;

    reg8 = radio.readReg(REG_ADDR_PKT_BITSTREAM_CTRL, 1);
    //ppBLE.ble.CrcLength = reg8 & 0x30;
    //ppBLE.ble.Whitening = reg8 & 0x08;
    ppGFSK.gfskFLRC.CRCLength = reg8 & 0x30;
    ppFLRC.gfskFLRC.CRCLength = reg8 & 0x30;
    ppGFSK.gfskFLRC.Whitening = reg8 & 0x08;
    ppFLRC.gfskFLRC.Whitening = reg8 & 0x08;

    {
        LoRaPktPar0_t LoRaPktPar0;
        LoRaPktPar0.octet = radio.readReg(REG_ADDR_LORA_PKTPAR0, 1);
        switch (LoRaPktPar0.bits.modem_bw) {
            case 2: mpLORA.lora.bandwidth = LORA_BW_200; break;
            case 3: mpLORA.lora.bandwidth = LORA_BW_400; break;
            case 4: mpLORA.lora.bandwidth = LORA_BW_800; break;
            case 5: mpLORA.lora.bandwidth = LORA_BW_1600; break;
        }
        mpLORA.lora.spreadingFactor = LoRaPktPar0.bits.modem_sf << 4;
    }

    {
        LoRaPktPar1_t LoRaPktPar1;
        LoRaPktPar1.octet = radio.readReg(REG_ADDR_LORA_PKTPAR1, 1);
        mpLORA.lora.codingRate = LoRaPktPar1.bits.coding_rate;
        ppLORA.lora.InvertIQ = LoRaPktPar1.octet & LORA_IQ_STD; // LoRaPktPar1.bits.rxinvert_iq
        ppLORA.lora.HeaderType = LoRaPktPar1.bits.implicit_header ? IMPLICIT_HEADER : EXPLICIT_HEADER;
        // LoRaPktPar1.bits.ppm_offset
    }

    {
        LoRaPreambleReg_t LoRaPreambleReg;
        LoRaPreambleReg.octet = radio.readReg(REG_ADDR_LORA_PREAMBLE, 1);
        ppLORA.lora.PreambleLength = LoRaPreambleReg.bits.preamble_symb1_nb * (1 << LoRaPreambleReg.bits.preamble_symb_nb_exp);
    }
    ppLORA.lora.PayloadLength = radio.readReg(REG_ADDR_LORA_TX_PAYLOAD_LENGTH, 1);

    {
        LoRaLrCtl_t LoRaLrCtl;
        LoRaLrCtl.octet = radio.readReg(REG_ADDR_LORA_LRCTL, 1);
        ppLORA.lora.crc = LoRaLrCtl.octet & 0x20; // LoRaLrCtl.bits.crc_en
    }

    {
        RegRxBw_t RegRxBw;
        unsigned bps;
        FloraPreambleHi_t FloraPreambleHi;
        float mi, fdev_hz;
        unsigned freqDev;
        FskModDfH_t FskModDfH;
        FskModDfH.octet = radio.readReg(REG_ADDR_FSK_MODDFH, 1);
        freqDev = FskModDfH.bits.freqDev;
        freqDev <<= 8;
        freqDev |= radio.readReg(REG_ADDR_FSK_MODDFL, 1);
        fdev_hz = freqDev * PLL_STEP_HZ;

        FloraPreambleHi.octet = radio.readReg(REG_ADDR_FLORA_PREAMBLE_HI, 1);
        switch (FloraPreambleHi.bits.data_rate) {
            case 0:
                bps = 2.0e6;
                //mpFLRC.flrc.bitrateBandwidth = ??; // 2.6
                break;
            case 1:
                bps = 1.6e6;
                //mpFLRC.flrc.bitrateBandwidth = ??; // 2.08
                break;
            case 2:
                bps = 1.0e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_1_300_BW_1_2; // 1.3
                break;
            case 3:
                bps = 0.8e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_1_000_BW_1_2; // 1.04
                break;
            case 4:
                bps = 0.5e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_0_650_BW_0_6; // 0.65
                break;
            case 5:
                bps = 0.4e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_0_520_BW_0_6; // 0.52
                break;
            case 6:
                bps = 0.25e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_0_325_BW_0_3; // 0.325
                break;
            case 7:
                bps = 0.125e6;
                mpFLRC.flrc.bitrateBandwidth = FLRC_BR_0_260_BW_0_3; // 0.26
                break;
        }

        mi = (fdev_hz * 2.0) / bps;
        if (mi > 0.35) {
            mi -= 0.5;
            mi /= 0.25;
            mpBLE_GFSK.gfskBle.ModulationIndex = ((uint8_t)mi) + 1;
        } else
            mpBLE_GFSK.gfskBle.ModulationIndex = 0;

        RegRxBw.octet = radio.readReg(REG_ADDR_RXBW, 1);

        switch (RegRxBw.bits.bw) {
            case 0:
                if (FloraPreambleHi.bits.data_rate == 0)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_2_000_BW_2_4;
                if (FloraPreambleHi.bits.data_rate == 1)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_1_600_BW_2_4;
                if (FloraPreambleHi.bits.data_rate == 2)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_1_000_BW_2_4;
                if (FloraPreambleHi.bits.data_rate == 3)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_800_BW_2_4;
                break;
            case 1:
                if (FloraPreambleHi.bits.data_rate == 2)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_1_000_BW_1_2;
                if (FloraPreambleHi.bits.data_rate == 3)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_800_BW_1_2;
                if (FloraPreambleHi.bits.data_rate == 4)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_500_BW_1_2;
                if (FloraPreambleHi.bits.data_rate == 5)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_400_BW_1_2;
                break;
            case 2:
                if (FloraPreambleHi.bits.data_rate == 4)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_500_BW_0_6;
                if (FloraPreambleHi.bits.data_rate == 5)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_400_BW_0_6;
                if (FloraPreambleHi.bits.data_rate == 6)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_250_BW_0_6;
                break;
            case 3:
                if (FloraPreambleHi.bits.data_rate == 6)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_250_BW_0_3;
                if (FloraPreambleHi.bits.data_rate == 7)
                    mpBLE_GFSK.gfskBle.bitrateBandwidth = GFSK_BLE_BR_0_125_BW_0_3;
                break;
        }
        mpBLE_GFSK.gfskBle.bitrateBandwidth = reg8;
    }

    {
        FskCfg_t FskCfg;
        FskCfg.octet = radio.readReg(REG_ADDR_FSK_CFG, 1);
        mpBLE_GFSK.gfskBle.ModulationShaping = FskCfg.bits.gf_bt << 4;
        mpFLRC.flrc.ModulationShaping = mpBLE_GFSK.gfskBle.ModulationShaping;
    }

    {
        PktBitStreamCtrl_t PktBitStreamCtrl;
        PktBitStreamCtrl.octet = radio.readReg(REG_ADDR_PKT_BITSTREAM_CTRL, 1);
        mpFLRC.flrc.CodingRate = PktBitStreamCtrl.octet & 0x06; // PktBitStreamCtrl.bits.flora_coding_rate 
    }

}

void Radio:: diox_top_half()
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

void Radio::rxDone(uint8_t size, const pktStatus_t* pktStatus)
{
    float rssi, snr;

    if (pktStatus->ble_gfsk_flrc.sync.syncAddrsCode == 0) {
        int8_t s = pktStatus->lora.snr;
        rssi = -pktStatus->lora.rssiSync / 2.0;
        snr = s / 4.0;
    } else {
        rssi = -pktStatus->ble_gfsk_flrc.rssiSync / 2.0;
        snr = FLT_MIN;
    }

    RadioEvents->RxDone(size, rssi, snr);
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

void Radio::txDoneBottom()
{
    if (RadioEvents->TxDone_botHalf)
        RadioEvents->TxDone_botHalf();
}

void Radio::Init(const RadioEvents_t* e, unsigned spi_hz)
{
    uint64_t sa;

    radio.txDone = txDoneBottom;
    radio.rxDone = rxDone;
    radio.timeout = timeout_callback;
    radio.chipModeChange = chipModeChange;
    radio.diox_topHalf = diox_top_half;

    spi.frequency(spi_hz);
    readChip();

    radio.setRegulator(0);  // default to LDO

    sa = 0xc194c1;
    radio.setSyncAddr(1, sa);

    RadioEvents = e;
    lpt.start();

    fe_enable = true;

    radio.periodBase = 2;   // 1ms resolution
    nSymbs = 8;
}

float Radio::GetRssiInst()
{
    uint8_t buf[2];
    radio.xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
    return buf[1] / -2.0;
}

int Radio::Send(uint8_t size, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh)
{
    uint8_t buf[8];
    uint8_t pktType = radio.getPacketType();

    if (pktType == PACKET_TYPE_LORA) {
        ppLORA.lora.PayloadLength = size;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 5, 0, ppLORA.buf);
    } else if (pktType == PACKET_TYPE_GFSK) {
        ppGFSK.gfskFLRC.PayloadLength = size;
        radio.xfer(OPCODE_SET_PACKET_PARAMS, 7, 0, ppGFSK.buf);
    }

    if (maxListenTime > 0) {
        int rssi;
        us_timestamp_t startAt, chFreeAt, now;
        radio.start_rx(-1);
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

    radio.start_tx(size, 4000);

    return 0;
}

void Radio::service()
{
    radio.service();
}

bool Radio::CheckRfFrequency(unsigned hz)
{
    return true;
}

void Radio::Sleep()
{
    radio.setSleep(true);
}

void Radio::SetPublicNetwork(bool en)
{
/*    uint16_t ppg;

    if (en)
        ppg = 0x3444;
    else
        ppg = 0x1424;

    radio.writeReg(REG_ADDR_LORA_SYNC, ppg, 2);
*/
}

uint32_t Radio::lora_toa_us( uint8_t pktLen )
{
    double bwKHz;
    LoRaPktPar0_t LoRaPktPar0;
    LoRaLrCtl_t LoRaLrCtl;
    LoRaPktPar1_t LoRaPktPar1;
    uint8_t LowDatarateOptimize;

    {
        LoRaPktPar1.octet = radio.readReg(REG_ADDR_LORA_PKTPAR1, 1);
        LowDatarateOptimize = LoRaPktPar1.bits.ppm_offset ? 1 : 0;
        ppLORA.lora.HeaderType = LoRaPktPar1.bits.implicit_header ? IMPLICIT_HEADER : EXPLICIT_HEADER;
        ppLORA.lora.InvertIQ = LoRaPktPar1.octet & LORA_IQ_STD; // LoRaPktPar1.bits.rxinvert_iq
        mpLORA.lora.codingRate = LoRaPktPar1.bits.coding_rate;
    }

    {
        LoRaLrCtl.octet = radio.readReg(REG_ADDR_LORA_LRCTL, 1);
        ppLORA.lora.crc = LoRaLrCtl.octet & 0x20; // LoRaLrCtl.bits.crc_en
    }

    {
        LoRaPreambleReg_t LoRaPreambleReg;
        LoRaPreambleReg.octet = radio.readReg(REG_ADDR_LORA_PREAMBLE, 1);
        ppLORA.lora.PreambleLength = LoRaPreambleReg.bits.preamble_symb1_nb * (1 << LoRaPreambleReg.bits.preamble_symb_nb_exp);
    }

    {
        LoRaPktPar0.octet = radio.readReg(REG_ADDR_LORA_PKTPAR0, 1);
        switch (LoRaPktPar0.bits.modem_bw) {
            case 0: bwKHz = 50; break;
            case 1: bwKHz = 100; break;
            case 2: mpLORA.lora.bandwidth = LORA_BW_200; bwKHz = 200; break;
            case 3: mpLORA.lora.bandwidth = LORA_BW_400; bwKHz = 400; break;
            case 4: mpLORA.lora.bandwidth = LORA_BW_800; bwKHz = 800; break;
            case 5: mpLORA.lora.bandwidth = LORA_BW_1600; bwKHz = 1600; break;
            default: bwKHz = 0; break;
        }
        mpLORA.lora.spreadingFactor = LoRaPktPar0.bits.modem_sf << 4;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bwKHz / (1 << LoRaPktPar0.bits.modem_sf);
    double ts = 1 / rs;
    // time of preamble
    //
    double tPreamble = ( ppLORA.lora.PreambleLength + 4.25 ) * ts;
    // Symbol length of payload and time
    
    double tmp = ceil( ( 8 * pktLen - 4 * LoRaPktPar0.bits.modem_sf +
                         28 + 16 * LoRaLrCtl.bits.crc_en -
                         ( LoRaPktPar1.bits.implicit_header ? 20 : 0 ) ) /
                         ( double )( 4 * ( LoRaPktPar0.bits.modem_sf -
                         ( ( LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) ) *
                         ( LoRaPktPar1.bits.coding_rate + 4 );

    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return microseconds
    return floor( tOnAir * 1000 + 0.999 );
}

void Radio::GFSKModemConfig(unsigned bps, unsigned bw_hz, unsigned fdev_hz)
{
    uint8_t u8;
    float mi, Mbps = bps / 1000000.0;

    if (Mbps > 1.6) {
        /* 2.0Mbps */
        u8 = GFSK_BLE_BR_2_000_BW_2_4;
    } else if (Mbps > 1.0) {
        /* 1.6Mbps */
        u8 = GFSK_BLE_BR_1_600_BW_2_4;
    } else if (Mbps > 0.8) {
        /* 1.0Mbps */
        /*if (bwMHz > 1.2)
            u8 = GFSK_BLE_BR_1_000_BW_2_4;
        else*/
            u8 = GFSK_BLE_BR_1_000_BW_1_2;
    } else if (Mbps > 0.5) {
        /* 0.8Mbps */
        /*if (bwMHz > 1.2)
            u8 = GFSK_BLE_BR_0_800_BW_2_4;
        else*/
            u8 = GFSK_BLE_BR_0_800_BW_1_2;
    } else if (Mbps > 0.4) {
        /* 0.5Mbps */
        /*if (bwMHz > 0.6)
            u8 = GFSK_BLE_BR_0_500_BW_1_2;
        else*/
            u8 = GFSK_BLE_BR_0_500_BW_0_6;
    } else if (Mbps > 0.25) {
        /* 0.4Mbps */
        /*if (bwMHz > 0.6)
            u8 = GFSK_BLE_BR_0_400_BW_1_2;
        else*/
            u8 = GFSK_BLE_BR_0_400_BW_0_6;
    } else if (Mbps > 0.125) {
        /* 0.25Mbps */
        /*if (bwMHz > 0.3)
            u8 = GFSK_BLE_BR_0_250_BW_0_6;
        else*/
            u8 = GFSK_BLE_BR_0_250_BW_0_3;
    } else {
        /* 0.125Mbps */
        u8 = GFSK_BLE_BR_0_125_BW_0_3;
    }

    mpBLE_GFSK.gfskBle.bitrateBandwidth = u8;

    mpBLE_GFSK.gfskBle.ModulationShaping = BT_OFF;

    mi = (fdev_hz * 2.0) / bps;
    if (mi > 0.35) {
        mi -= 0.5;
        mi /= 0.25;
        mpBLE_GFSK.gfskBle.ModulationIndex = ((uint8_t)mi) + 1;
    } else
        mpBLE_GFSK.gfskBle.ModulationIndex = 0;

    radio.xfer(OPCODE_SET_MODULATION_PARAMS, 3, 0, mpBLE_GFSK.buf);
}

void Radio::GFSKPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn)
{
    ppGFSK.gfskFLRC.PreambleLength = (preambleLen - 4) / 4;
    ppGFSK.gfskFLRC.PreambleLength <<= 4;
    ppGFSK.gfskFLRC.SyncWordLength = (3 - 1) << 1;  // 3 byte 0xc194c1
    ppGFSK.gfskFLRC.HeaderType = fixLen ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
    ppGFSK.gfskFLRC.CRCLength = crcOn ? RADIO_CRC_2_BYTES : RADIO_CRC_OFF;

    // TODO ppGFSK.gfskFLRC.PayloadLength = ;

    radio.xfer(OPCODE_SET_PACKET_PARAMS, 7, 0, ppGFSK.buf);
}

void Radio::SetLoRaSymbolTimeout(uint16_t symbs)
{
    nSymbs = symbs;
    rxTimeoutMs = nSymbs * (symbolPeriodUs / 1000.0);
}

void Radio::LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t cr)
{
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    if (bwKHz > 800)
        mpLORA.lora.bandwidth = LORA_BW_1600;
    else if (bwKHz > 400)
        mpLORA.lora.bandwidth = LORA_BW_800;
    else if (bwKHz > 200)
        mpLORA.lora.bandwidth = LORA_BW_400;
    else if (bwKHz > 100)
        mpLORA.lora.bandwidth = LORA_BW_200;
    else if (bwKHz > 50)
        mpLORA.lora.bandwidth = LORA_BW_100;
    else
        mpLORA.lora.bandwidth = LORA_BW_50;

    mpLORA.lora.codingRate = cr;

    mpLORA.lora.spreadingFactor = sf << 4;

    radio.xfer(OPCODE_SET_MODULATION_PARAMS, 3, 0, mpLORA.buf);

    symbolPeriodUs = (1 << sf) / (bwKHz / 1000.0);   // bw in MHz gives microseconds
    rxTimeoutMs = nSymbs * (symbolPeriodUs / 1000.0);
}

void Radio::LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ)
{
    if (radio.getPacketType() != PACKET_TYPE_LORA)
        radio.setPacketType(PACKET_TYPE_LORA);

    ppLORA.lora.PreambleLength = preambleLen;
    ppLORA.lora.HeaderType = fixLen ? IMPLICIT_HEADER : EXPLICIT_HEADER;
    ppLORA.lora.crc = crcOn ? LORA_CRC_ENABLE : LORA_CRC_DISABLE;
    ppLORA.lora.InvertIQ = invIQ ? LORA_IQ_INVERTED : LORA_IQ_STD;

    radio.xfer(OPCODE_SET_PACKET_PARAMS, 5, 0, ppLORA.buf);
}

void Radio::SetChannel(unsigned hz)
{
    radio.setMHz(hz / 1000000.0);
}

uint32_t Radio::Random(void)
{
    uint8_t buf[2];
    uint32_t ret = 0;
    unsigned n;

    radio.start_rx(-1);

    for (n = 0; n < 8; n++) {
        uint32_t r, s;
        wait_us(5000);
        radio.xfer(OPCODE_GET_RSSIINST, 0, 2, buf);
        r = buf[1];
        s = n * 4;
        r <<= s;
        ret ^= r;
    }

    radio.setStandby(STDBY_RC);

    return ret;
}

void Radio::Rx(unsigned timeout)
{
#ifdef RX_INDICATION
    RX_INDICATION = 1;
#endif
    if (timeout == 0)
        radio.start_rx(0);  // continuous rx
    else {
        radio.start_rx(rxTimeoutMs);
    }
}

void Radio::Standby()
{
    radio.setStandby(STDBY_RC);
}

#define TX_PWR_OFFSET           18
void Radio::set_tx_dbm(int8_t dbm)
{
    if (dbm == PA_OFF_DBM) {
        /* TODO: shut off PA */
        radio.set_tx_dbm(0);
    } else {
        /* power range -18dBm to +13dBm */
        radio.set_tx_dbm(dbm + TX_PWR_OFFSET);
    }
}

void Radio::SetTxContinuousWave(unsigned hz, int8_t dbm, unsigned timeout_us)
{
    SetChannel(hz);
    radio.set_tx_dbm(dbm);
    radio.xfer(OPCODE_SET_TX_CARRIER, 0, 0, NULL);
}

void Radio::SetRxMaxPayloadLength(uint8_t max)
{
    uint8_t pktType = radio.getPacketType();

    if (pktType == PACKET_TYPE_GFSK)
        ppGFSK.gfskFLRC.PayloadLength = max;
    else if (pktType == PACKET_TYPE_LORA)
        ppLORA.lora.PayloadLength = max;
}

#endif /* ..SX126x_H */
