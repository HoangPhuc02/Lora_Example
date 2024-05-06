#include "lorawan.h"
#include "commands.h"
#include "SPIu.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#define PREAMBLE_SYMBS          8

const microseconds BEACON_PRELOAD_us(500000);
const seconds BEACON_INTERVAL_s(128);
HighResClock::time_point beaconAt;
Timeout beacon_timeout; /* beacon generator */

UnbufferedSerial pc( USBTX, USBRX );

char pcbuf[64];
int pcbuf_len;
uint8_t beacon_payload[4];

unsigned int skip_beacon_cnt;

float starting_bg_rssi;
volatile uint32_t usingChHz;

#define N_SAMPLES       64
void channel_scan()
{
    int min_ch, ch;
    uint32_t hz = LORAMAC_FIRST_CHANNEL;
    int acc[LORA_MAX_NB_CHANNELS];

    Radio::Standby( );
    ThisThread::sleep_for(20ms);
    
    for (ch = 0; ch < LORA_MAX_NB_CHANNELS; ch++) {
        int i;
        Radio::SetChannel(hz);
        Radio::Rx(0);
        acc[ch] = 0;
        for (i = 0; i < N_SAMPLES; i++) {
            int rssi = Radio::GetRssiInst();
            acc[ch] += rssi;
            ThisThread::sleep_for(10ms);
        }
        Radio::Standby( );
        pc_printf("ch%u: %d\r\n", ch, (int)(acc[ch] / (float)N_SAMPLES));
        hz += LORAMAC_STEPWIDTH_CHANNEL;
        ThisThread::sleep_for(20ms);
        Radio::SetChannel(hz);
    }

    int min = 0x7fffffff;
    min_ch = 0;
    for (ch = 0; ch < LORA_MAX_NB_CHANNELS; ch++) {
        if (acc[ch] < min) {
            min = acc[ch];
            min_ch = ch;
        }
    }
    hz = LORAMAC_FIRST_CHANNEL + (min_ch * LORAMAC_STEPWIDTH_CHANNEL);
    pc_printf("using ch%u, %luhz\r\n", min_ch, hz);
    Radio::SetChannel(hz);
    usingChHz = hz;
    
    starting_bg_rssi = acc[min_ch] / (float)N_SAMPLES;
}

void measure_ambient()
{
    int i, acc = 0;
    float bg_rssi;
    float diff;
    static unsigned cnt = 0;
    
    for (i = 0; i < N_SAMPLES; i++) {
        int rssi = Radio::GetRssiInst();
        acc += rssi;
        ThisThread::sleep_for(10ms);
    }
    bg_rssi = acc / (float)N_SAMPLES;
    diff = bg_rssi - starting_bg_rssi;
    //pc_printf("bg_rssi:%.1fdBm vs %1.fdBm, diff:%.1f, %d\r\n", bg_rssi, starting_bg_rssi, diff, cnt);
    pc_printf("bg_rssi:%ddBm vs %1ddBm, diff:%d, %d\r\n", (int)bg_rssi, (int)starting_bg_rssi, (int)diff, cnt);
    if (diff > 10) {
        if (++cnt > 3) {
            /* find better channel */
            channel_scan();
            Radio::Rx(0);
            cnt = 0;
        }
    } else
        cnt = 0;
}

void init_radio()
{
    Radio::Standby( );

#ifdef SX126x_H
    if (Radio::chipType == CHIP_TYPE_SX1262)
        Radio::set_tx_dbm(22);
    else
        Radio::set_tx_dbm(17);
#elif defined(SX127x_H)
    Radio::set_tx_dbm(17);
#elif defined(SX128x_H)
    Radio::set_tx_dbm(5);
#endif

    Radio::LoRaModemConfig(BANDWIDTH_KHZ, LoRaWan::Datarates[LORAMAC_DEFAULT_DATARATE], 1);
    pc_printf("using sf%u\r\n", LoRaWan::Datarates[LORAMAC_DEFAULT_DATARATE]);
    Radio::SetPublicNetwork(true);

    channel_scan();

    Radio::SetRxMaxPayloadLength(255);
}

#if defined SX127x_H
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

void decrypted_uplink(uint32_t dev_addr, uint8_t* buf, uint8_t buflen, uint8_t port)
{
    if (port == SENSOR_PORT) {
        uint8_t i = 0;
        uint16_t lum = buf[i++] << 8;
        lum += buf[i++];
        LoRaWan::filtered_printf(dev_addr, MAC, "SENSOR lum:%u", lum);

        while (i < buflen) {
            uint16_t seq, a_a, a_b, p;
            seq = buf[i++] << 8;
            seq += buf[i++];
            a_a = buf[i++] << 8;
            a_a += buf[i++];
            a_b = buf[i++] << 8;
            a_b += buf[i++];
            p = buf[i++];
            if (!LoRaWan::flags.show_mac)
                LoRaWan::filtered_printf(dev_addr, APP, "%x", dev_addr);

            LoRaWan::filtered_printf(dev_addr, APP, ", %u, %u, %u, %u, %u",
                seq,
                a_a,    /* analog */
                a_b,    /* analog */
                (p & 2) >> 1,    /* digital in */
                p & 1    /* digital out */
            );

            if (!LoRaWan::flags.show_mac && i < buflen)
                LoRaWan::filtered_printf(dev_addr, APP, "\r\n");
        }
    } else {
        int i;
        LoRaWan::filtered_printf(dev_addr, APP, "port%u: ", port);
        for (i = 0; i < buflen; i++)
            LoRaWan::filtered_printf(dev_addr, APP, "%02x ", buf[i]);
    }
}

//EventQueue queue;

void send_downlink()
{
    if (!LoRaWan::flags.do_downlink) {
        pc_printf("\e[41mno downlink to send\e[0m\r\n");
        return;
    }

    LoRaWan::flags.do_downlink = false;

    if (LoRaWan::flags.beacon_guard) {
        pc_printf("\e[33mdownlink during beacon_guard\e[0m\r\n");
        return;
    }

    Radio::Standby( );

    //                         preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, false, false, true);

    Radio::Send(LoRaWan::_tmp_payload_length, 0, 0, 0);
}

uint16_t tim_get_current_slot()
{
    int us_until = beaconAt.time_since_epoch().count() - HighResClock::now().time_since_epoch().count();
    return us_until / 30000;
}

static void OnRadioRxDone(uint8_t size, float rssi, float snr)
{
    LoRaWan::rx_slot = tim_get_current_slot();
    LoRaWan::rx_at = HighResClock::now();

    LoRaWan::parse_receive(size, rssi, snr);

    if (!LoRaWan::flags.do_downlink) {
        /* if not sending downlink, start receiver now */
        Radio::Rx(0);
    }
}

void
send_beacon()
{
    if (!LoRaWan::flags.beacon_loaded)
        return;

    LoRaWan::flags.beacon_loaded = false;
    Radio::Send(LoRaWan::_tmp_payload_length, 0, 0, 0);
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

void
_load_beacon()
{
    bool inv_iq = false;
    uint16_t crc;
    Radio::Standby( );

#if defined SX127x_H
    while(Radio::radio.dio0) {
        pc_printf("\e[41mmissed-interrupt\e[0m\r\n");
        Radio::service();
    }
#elif defined(SX128x_H)
    #error todo handle missed interrupt sx1280
#elif defined(SX126x_H)
    #error todo handle missed interrupt sx126x
#endif /* SX127x_H */

    if (skip_beacon_cnt > 0) {
        inv_iq = true;
        skip_beacon_cnt--;
    }
                       //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, true, false, inv_iq);

    LoRaWan::_tmp_payload_length = BEACON_SIZE;
    Radio::SetFixedPayloadLength(LoRaWan::_tmp_payload_length);

    Radio::radio.tx_buf[0] = beacon_payload[0];
    Radio::radio.tx_buf[1] = beacon_payload[1];
    Radio::radio.tx_buf[2] = beacon_payload[2];
    Radio::radio.tx_buf[3] = beacon_payload[3];
    beacon_payload[0] = CMD_NONE;

    crc = beacon_crc(Radio::radio.tx_buf, 4);
    Radio::radio.tx_buf[4] = crc & 0xff;
    Radio::radio.tx_buf[5] = crc >> 8;

    LoRaWan::flags.beacon_loaded = true;

    beacon_timeout.attach_absolute(send_beacon, beaconAt);
}

void load_beacon()
{
    LoRaWan::flags.beacon_guard = true;
    beacon_timeout.attach(_load_beacon, 200ms);
}

void get_time_till_beacon()
{
    uint16_t slots = tim_get_current_slot();
    pc_printf("slots:%u\r\n", slots);
}

void rx_isr()
{
    static uint8_t pcbuf_idx = 0;
    static uint8_t prev_len = 0;;
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

void cmd_skip_beacon(uint8_t idx)
{
    if (pcbuf[idx] >= '0' && pcbuf[idx] <= '9') {
        sscanf(pcbuf+idx, "%u", &skip_beacon_cnt);
    }
    pc_printf("skip_beacon_cnt:%u\r\n", skip_beacon_cnt);
}

void cmd_list_motes(uint8_t idx)
{
    int i;
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr != DEVADDR_NONE) {
            LoRaWan::print_octets_rev("", motes[i].dev_eui, LORA_EUI_LENGTH);
            pc_printf("    %" PRIx32 "\r\n", motes[i].dev_addr);
        }
    }
}

void
cmd_beacon_payload(uint8_t idx)
{
    uint32_t i;
    uint32_t* ptr;
    sscanf(pcbuf+idx, "%" PRIx32, &i);
    pc_printf("beacon_payload:%08" PRIx32 "\r\n", i);
    ptr = (uint32_t*)beacon_payload;
    *ptr = i;
}

void
cmd_send_downlink(uint8_t idx)
{
    ota_mote_t* mote = NULL;
    int i;
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x", &dev_addr);
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr == dev_addr) {
            break;
        }
    }
    if (i == N_MOTES) {
        pc_printf("mote %x not found\r\n", dev_addr);
        return;
    }
    mote = &motes[i];

    while (pcbuf[idx] != ' ') {
        if (pcbuf[++idx] == 0) {
            pc_printf("hit end\r\n");
            return;
        }
    }
    idx++;    // step past space

    mote->user_downlink_length = 0;
    while (pcbuf[idx] > ' ') {
        int o;
        sscanf(pcbuf+idx, "%02x", &o);
        LoRaWan::user_downlink[mote->user_downlink_length++] = o;
        idx += 2;
    }

    pc_printf("%u bytes scheduled for %" PRIx32 "\r\n", mote->user_downlink_length, mote->dev_addr);
}

#if 0
void cmd_rx_restart(uint8_t idx)
{
    /*radio.set_opmode(RF_OPMODE_STANDBY);
    pc_printf("standby\r\n");*/
    radio.set_opmode(RF_OPMODE_SLEEP);
    pc_printf("sleep\r\n");
    wait(0.05);
    radio.set_opmode(RF_OPMODE_RECEIVER);
    pc_printf("receive\r\n");
}
#endif /* if 0 */

void cmd_beacon_gpo(uint8_t idx)
{
    int gpo;
    sscanf(pcbuf+idx, "%d", &gpo);
    beacon_payload[0] = CMD_GPIO_OUT;
    beacon_payload[1] = gpo;   
    pc_printf("beacon gpo: %d\r\n", gpo);
}

void cmd_beacon_rgb(uint8_t idx)
{
    int r, g ,b;
    sscanf(pcbuf+idx, "%d %d %d", &r, &g, &b);
    beacon_payload[0] = CMD_LED_RGB;
    beacon_payload[1] = r;
    beacon_payload[2] = g;
    beacon_payload[3] = b;
    pc_printf("beacon rgb: %d %d %d\r\n", r, g, b);
}

void cmd_downlink_rgb(uint8_t idx)
{
    ota_mote_t* mote = NULL;
    int i, r, g ,b;
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x %d %d %d", &dev_addr, &r, &g, &b);
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr == dev_addr) {
            break;
        }
    }
    if (i == N_MOTES) {
        pc_printf("mote %x not found\r\n", dev_addr);
        return;
    }
    mote = &motes[i];

    mote->user_downlink_length = 0;
    LoRaWan::user_downlink[mote->user_downlink_length++] = CMD_LED_RGB;
    LoRaWan::user_downlink[mote->user_downlink_length++] = r;
    LoRaWan::user_downlink[mote->user_downlink_length++] = g;
    LoRaWan::user_downlink[mote->user_downlink_length++] = b;
    
    pc_printf("rgb %d %d %d to mote %" PRIx32 "\r\n", r, g, b, mote->dev_addr);
}

void cmd_downlink_gpo(uint8_t idx)
{
    ota_mote_t* mote = NULL;
    int i, gpo;
    unsigned int dev_addr;
    sscanf(pcbuf+idx, "%x %d", &dev_addr, &gpo);
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr == dev_addr) {
            break;
        }
    }
    if (i == N_MOTES) {
        pc_printf("mote %x not found\r\n", dev_addr);
        return;
    }
    mote = &motes[i];

    mote->user_downlink_length = 0;
    LoRaWan::user_downlink[mote->user_downlink_length++] = CMD_GPIO_OUT;
    LoRaWan::user_downlink[mote->user_downlink_length++] = gpo;
    
    pc_printf("gpo %d to mote %" PRIx32 "\r\n", gpo, mote->dev_addr);    
}

void cmd_status(uint8_t idx)
{
    pc_printf(" %uHz do_downlink:%u, ", usingChHz, LoRaWan::flags.do_downlink);
    pc_printf("rssi:%f\r\n", Radio::GetRssiInst());
    pc_printf("\r\nskip_beacon_cnt:%u, curSlot:%u\r\n", skip_beacon_cnt, tim_get_current_slot());
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
    pc_printf("loraSync:%04x\r\n", Radio::radio.readReg(REG_ADDR_LORA_SYNC, 2));
#elif defined SX126x_H
    status_t status;
    Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
    switch (status.bits.chipMode) {
        case 2: pc_printf("STBY_RC"); break;
        case 3: pc_printf("STBY_XOSC"); break;
        case 4: pc_printf("FS"); break;
        case 5: pc_printf("\e[32mRX\e[0m"); break;
        case 6: pc_printf("\e[31mTX\e[0m"); break;
        default: pc_printf("%u", status.bits.chipMode); break;
    }
    pc_printf(" ");
    switch (status.bits.cmdStatus) {
        case 1: pc_printf("rfu"); break;
        case 2: pc_printf("dataAvail"); break;
        case 3: pc_printf("timeout"); break;
        case 4: pc_printf("err"); break;
        case 5: pc_printf("fail"); break;
        case 6: pc_printf("txdone"); break;
        default: pc_printf("%u", status.bits.cmdStatus); break;
    }
    loraConfig0_t conf0;
    conf0.octet = Radio::radio.readReg(REG_ADDR_LORA_CONFIG0, 1);
    // bw7=125 bw8=250 b9=500
    pc_printf(" bw:%u sf%u\r\n", conf0.bits.modem_bw, conf0.bits.modem_sf);
    loraConfig1_t conf1;
    conf1.octet = Radio::radio.readReg(REG_ADDR_LORA_CONFIG1, 1);
    pc_printf("inviq:%u cr%u\r\n", conf1.bits.rx_invert_iq, conf1.bits.tx_coding_rate);
    pc_printf("loraSync:%04x\r\n", Radio::radio.readReg(REG_ADDR_LORA_SYNC, 2));
#elif defined SX127x_H
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
    pc_printf("\r\nskip_beacon_cnt:%u, currently:%u dio0map:%u\r\n", skip_beacon_cnt, tim_get_current_slot(), Radio::radio.RegDioMapping1.bits.Dio0Mapping);
    pc_printf("FIfoAddrPtr:%02x RxBase:%02x\r\n", Radio::radio.read_reg(REG_LR_FIFOADDRPTR), Radio::radio.read_reg(REG_LR_FIFORXBASEADDR));
#endif
}

void cmd_filter(uint8_t idx)
{
    if (sscanf(pcbuf+idx, "%lx", &LoRaWan::dev_addr_filter) != 1) {
        LoRaWan::dev_addr_filter = 0;
        pc_printf("filter off\r\n");
    } else
        pc_printf("filtering %lx\r\n", LoRaWan::dev_addr_filter);
}

void cmd_op(uint8_t idx)
{
    int dbm;
    if (sscanf(pcbuf+idx, "%d", &dbm) == 1) {
        Radio::set_tx_dbm(dbm);
        pc_printf("OutputPower:%ddBm\r\n", dbm);
    }
}

#ifdef TYPE_ABZ
void cmd_pa_select(uint8_t idx)
{
    radio.RegPaConfig.bits.PaSelect ^= 1;
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
    if (radio.RegPaConfig.bits.PaSelect)
        pc_printf("PA_BOOST\r\n");
    else
        pc_printf("RFO\r\n");
}
#endif /* TYPE_ABZ */

void cmd_set_time(uint8_t idx)
{
    set_time(0);
    pc_printf("time:%" PRIu32 "\r\n", time(NULL));
}


void cmd_pwm(uint8_t idx)
{
    ota_mote_t* mote;
    int i;
    unsigned dev_addr, p, d;

    if (sscanf(pcbuf+idx, "%x %u %u", &dev_addr, &p, &d) != 3) {
        pc_printf("parse fail\r\n");
        return;
    }
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr == dev_addr) {
            break;
        }
    }
    if (i == N_MOTES) {
        pc_printf("mote %x not found\r\n", dev_addr);
        return;
    }

    mote = &motes[i];

    mote->user_downlink_length = 0;
    LoRaWan::user_downlink[mote->user_downlink_length++] = CMD_PWM;
    LoRaWan::user_downlink[mote->user_downlink_length++] = p;
    LoRaWan::user_downlink[mote->user_downlink_length++] = d;
    
    pc_printf("period:%u duty:%u to mote %" PRIx32 "\r\n", p, d, mote->dev_addr);
}

void cmd_beacon_pwm(uint8_t idx)
{
    unsigned p, d;
    if (sscanf(pcbuf+idx, "%u %u", &p, &d) != 2) {
        pc_printf("parse fail\r\n");
        return;
    }

    beacon_payload[0] = CMD_PWM;
    beacon_payload[1] = p;
    beacon_payload[2] = d;
    
    pc_printf("period:%u duty:%u\r\n", p, d);
}

void cmd_endnode_txp(uint8_t idx)
{
    ota_mote_t* mote;
    int i;
    unsigned dev_addr, txi;

    if (sscanf(pcbuf+idx, "%x %u", &dev_addr, &txi) != 2) {
        pc_printf("parse fail\r\n");
        return;
    }
    for (i = 0; i < N_MOTES; i++) {
        if (motes[i].dev_addr == dev_addr) {
            break;
        }
    }
    if (i == N_MOTES) {
        pc_printf("mote %x not found\r\n", dev_addr);
        return;
    }

    mote = &motes[i];

    mote->user_downlink_length = 0;
    LoRaWan::user_downlink[mote->user_downlink_length++] = CMD_TX_POWER;
    LoRaWan::user_downlink[mote->user_downlink_length++] = txi;
    
    pc_printf("txp index %u to mote %" PRIx32 "\r\n", txi, mote->dev_addr);
}

void cmd_hide_mac(uint8_t idx)
{
    LoRaWan::flags.show_mac = 0;
    pc_printf("!show_mac\r\n");
}

void cmd_hide_payload(uint8_t idx)
{
    LoRaWan::flags.show_app = 0;
    pc_printf("!show_app\r\n");
}

void cmd_show_all(uint8_t idx)
{
    LoRaWan::flags.show_mac = 1;
    LoRaWan::flags.show_app = 1;
    pc_printf("show-all\r\n");
}

void cmd_beacon_endnode_txp(uint8_t idx)
{
    unsigned txi;
    if (sscanf(pcbuf+idx, "%u", &txi) != 1) {
        pc_printf("parse fail\r\n");
        return;
    }

    beacon_payload[0] = CMD_TX_POWER;
    beacon_payload[1] = txi;
    
    pc_printf("txp index:%u\r\n", txi);
}

void cmd_help(uint8_t);

typedef struct {
    const char* const cmd;
    void (*handler)(uint8_t args_at);
    const char* const arg_descr;
    const char* const description;
} menu_item_t;

const menu_item_t menu_items[] = 
{   /* after first character, command names must be [A-Za-z] */
    { "?", cmd_help, "","show available commands"}, 
    { ".", cmd_status, "","read status"}, 
    { "f", cmd_filter, "%x","set dev_addr print filter"}, 
    { "b", cmd_beacon_payload, "<%x>","set beacon payload"}, 
    { "sb", cmd_skip_beacon, "<%d>","skip beacons"}, 
    { "list", cmd_list_motes, "","list active motes"}, 
    { "dl", cmd_send_downlink, "[%x %s]","send downlink <mote-hex-dev-addr> <hex-payload>"}, 
    //{ "rxr", cmd_rx_restart, "", "restart RX"},
    { "brgb", cmd_beacon_rgb, "%u %u %u", "load RGB command into next beacon" },
    { "rgb", cmd_downlink_rgb, "%x %u %u %u", "load RGB command to mote"},
    { "bgpo", cmd_beacon_gpo, "%d", "load output pin command into next beacon"},
    { "gpo", cmd_downlink_gpo, "%x %d", "load output pin command to mote"},
    { "op", cmd_op, "<dBm>","(TX) get/set TX power"},  
#ifdef TYPE_ABZ
    { "pas", cmd_pa_select, "","(TX) toggle PaSelect"},  
#endif
    { "tz", cmd_set_time, "", "set seconds to zero"},   
    { "p", cmd_pwm, "%x %u %u", "send pwm period, duty to dev_addr"},
    { "bp", cmd_beacon_pwm, "%u %u", "send pwm period, duty on beacon"},
    { "ntxp", cmd_endnode_txp, "%x %u", "send txpower index to dev_addr"},
    { "bntxp", cmd_beacon_endnode_txp, "%u", "send txpower index on beacon"},
    { "hm", cmd_hide_mac, "", "hide mac layer printing "},
    { "hp", cmd_hide_payload, "", "hide payload printing"},
    { "sa", cmd_show_all, "", "show both mac and app layers"},
    { NULL, NULL, NULL, NULL }
};

void cmd_help(uint8_t args_at)
{
    int i;
    
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        pc_printf("%s%s\t%s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
    }
    
}

void
console()
{
    int i;
    uint8_t user_cmd_len;
    
    if (pcbuf_len < 0) {    // ctrl-C
        //pc_printf("abort\r\n");
        return;
    }
    if (pcbuf_len == 0)
        return;
        
    pc_printf("\r\n");
        
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

        if (menu_items[i].handler && user_cmd_len == mi_len && (strncmp(pcbuf, menu_items[i].cmd, mi_len) == 0)) {
            while (pcbuf[mi_len] == ' ')   // skip past spaces
                mi_len++;
            menu_items[i].handler(mi_len);
            break;
        }
    }
   
    pcbuf_len = 0;
    pc_printf("> ");
    fflush(stdout); 
}

static void OnRadioTxDone()
{
    if (LoRaWan::flags.beacon_test) {
        LoRaWan::flags.beacon_test = 0;
        return;
    }

           //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, false, true, false);
    
    Radio::Rx(0);

    if (LoRaWan::flags.beacon_guard) {   // beacon done transmitting
        measure_ambient();
        LoRaWan::flags.beacon_guard = false;

        beaconAt += BEACON_INTERVAL_s;
        beacon_timeout.attach_absolute(load_beacon, beaconAt - BEACON_PRELOAD_us);
    }
}

void tim_init()
{
    beaconAt = HighResClock::now() + BEACON_INTERVAL_s;
    beacon_timeout.attach_absolute(load_beacon, beaconAt - BEACON_PRELOAD_us);
}


const RadioEvents_t rev = {
    /* Dio0_top_half */     NULL,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    OnRadioTxDone,
    /* TxTimeout  */        NULL,
    /* RxDone  */           OnRadioRxDone,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

int main()
{
    Thread eventThread;
    pc.baud(115200);
    pc_printf("\r\nreset\r\n");
    set_time(0);

    Radio::Init(&rev);

    init_radio();

    {
        long txStartAt;
        _load_beacon();
        send_beacon();
        txStartAt = HighResClock::now().time_since_epoch().count();
        LoRaWan::flags.beacon_test = 1;
        while (LoRaWan::flags.beacon_test)
            Radio::service();

        LoRaWan::beaconDur = HighResClock::now().time_since_epoch().count() - txStartAt;
        pc_printf("beaconDur:%u, 0x%x\r\n", LoRaWan::beaconDur, LoRaWan::beaconDur);
    }

           //    preambleLen, fixLen, crcOn, invIQ
    Radio::LoRaPacketConfig(PREAMBLE_SYMBS, false, true, false);
    Radio::Rx(0);

    LoRaWan::init();

    tim_init();
    
    pc.attach(&rx_isr);

    /* default to printing both mac layer and application layer */
    LoRaWan::flags.show_mac = 1;
    LoRaWan::flags.show_app = 1;

    for (;;) {
        console();

        Radio::service();
    } // ..for(;;)
}
