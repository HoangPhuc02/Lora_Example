#include "sx127x_fsk.h"

/* SX127x driver
 * Copyright (c) 2013 Semtech
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

SX127x_fsk::SX127x_fsk(SX127x& r) : m_xcvr(r)
{
}

SX127x_fsk::~SX127x_fsk()
{
}

void SX127x_fsk::write_fifo(uint8_t len)
{
    int i;
    
    m_xcvr.m_cs = 0;
    m_xcvr.m_spi.write(REG_FIFO | 0x80); // bit7 is high for writing to radio
    
    if (!m_xcvr.RegOpMode.bits.LongRangeMode && RegPktConfig1.bits.PacketFormatVariable) {
        m_xcvr.m_spi.write(len);
    }
    
    for (i = 0; i < len; i++) {
        m_xcvr.m_spi.write(m_xcvr.tx_buf[i]);
    }
    m_xcvr.m_cs = 1;
}

void SX127x_fsk::enable(bool fast)
{
    m_xcvr.set_opmode(RF_OPMODE_SLEEP);
    
    m_xcvr.RegOpMode.bits.LongRangeMode = 0;
    m_xcvr.write_reg(REG_OPMODE, m_xcvr.RegOpMode.octet);
    if (fast)
        return;
    
    RegPktConfig1.octet = m_xcvr.read_reg(REG_FSK_PACKETCONFIG1);
    RegPktConfig2.word = m_xcvr.read_u16(REG_FSK_PACKETCONFIG2);
    RegRxConfig.octet = m_xcvr.read_reg(REG_FSK_RXCONFIG);
    RegPreambleDetect.octet = m_xcvr.read_reg(REG_FSK_PREAMBLEDETECT);
    RegSyncConfig.octet = m_xcvr.read_reg(REG_FSK_SYNCCONFIG);
    RegFifoThreshold.octet = m_xcvr.read_reg(REG_FSK_FIFOTHRESH);
    RegAfcFei.octet = m_xcvr.read_reg(REG_FSK_AFCFEI);
    
    if (!RegFifoThreshold.bits.TxStartCondition) {
        RegFifoThreshold.bits.TxStartCondition = 1; // start TX on fifoEmpty==0
        m_xcvr.write_reg(REG_FSK_FIFOTHRESH, RegFifoThreshold.octet);
    }
    
    if (RegSyncConfig.bits.AutoRestartRxMode != 1) {
        RegSyncConfig.bits.AutoRestartRxMode = 1;
        m_xcvr.write_reg(REG_FSK_SYNCCONFIG, RegSyncConfig.octet);
    }
    
    RegPreambleDetect.bits.PreambleDetectorOn = 1;
    RegPreambleDetect.bits.PreambleDetectorSize = 1;
    RegPreambleDetect.bits.PreambleDetectorTol = 10;
    m_xcvr.write_reg(REG_FSK_PREAMBLEDETECT, RegPreambleDetect.octet);      
    
    m_xcvr.set_opmode(RF_OPMODE_STANDBY);     
}

void SX127x_fsk::init()
{
    m_xcvr.set_opmode(RF_OPMODE_STANDBY);
    
    RegRxConfig.bits.RxTrigger = 6; // have RX restart (trigger) on preamble detection
    RegRxConfig.bits.AfcAutoOn = 1; // have AFC performed on RX restart (RX trigger)
    m_xcvr.write_reg(REG_FSK_RXCONFIG, RegRxConfig.octet);
    
    RegPreambleDetect.bits.PreambleDetectorOn = 1;  // enable preamble detector
    m_xcvr.write_reg(REG_FSK_PREAMBLEDETECT, RegPreambleDetect.octet);
    
    m_xcvr.write_reg(REG_FSK_SYNCVALUE1, 0x55);
    m_xcvr.write_reg(REG_FSK_SYNCVALUE2, 0x6f); 
    m_xcvr.write_reg(REG_FSK_SYNCVALUE3, 0x4e);
    RegSyncConfig.bits.SyncSize = 2;
    m_xcvr.write_reg(REG_FSK_SYNCCONFIG, RegSyncConfig.octet);
    
    // in case these were changed from default:
    set_bitrate(4800);
    set_tx_fdev_hz(5050);
    set_rx_dcc_bw_hz(10500, 0);    // rxbw
    set_rx_dcc_bw_hz(50000, 1);    // afcbw
}
    
uint32_t SX127x_fsk::get_bitrate()
{
    uint16_t br = m_xcvr.read_u16(REG_FSK_BITRATEMSB);

    if (br == 0)
        return 0;
    else {
        uint32_t bps = XTAL_FREQ / br;
        bit_period_us = bps / 1000;
        return bps;
    }
}

void SX127x_fsk::set_bitrate(uint32_t bps)
{
    uint16_t tmpBitrate = XTAL_FREQ / bps;
    bit_period_us = bps / 1000;
    //printf("tmpBitrate:%d = %d / %d\r\n", tmpBitrate, XTAL_FREQ, bps);
    m_xcvr.write_u16(REG_FSK_BITRATEMSB, tmpBitrate);
}

void SX127x_fsk::set_tx_fdev_hz(uint32_t hz)
{
    float tmpFdev = hz / FREQ_STEP_HZ;
    uint16_t v;
    //printf("tmpFdev:%f = %d / %f\r\n", tmpFdev, hz, FREQ_STEP_HZ);
    v = (uint16_t)tmpFdev;
    m_xcvr.write_u16(REG_FSK_FDEVMSB, v);
}
    
uint32_t SX127x_fsk::get_tx_fdev_hz(void)
{
    uint16_t fdev = m_xcvr.read_u16(REG_FSK_FDEVMSB);
    return fdev * FREQ_STEP_HZ;
}

uint32_t SX127x_fsk::ComputeRxBw( uint8_t mantisse, uint8_t exponent )
{
    // rxBw
    if (m_xcvr.RegOpMode.bits.ModulationType == 0)
        return XTAL_FREQ / (mantisse * (1 << (exponent+2)));
    else
        return XTAL_FREQ / (mantisse * (1 << (exponent+3)));    
}


void SX127x_fsk::ComputeRxBwMantExp( uint32_t rxBwValue, uint8_t* mantisse, uint8_t* exponent )
{
    uint8_t tmpExp, tmpMant;
    double tmpRxBw;
    double rxBwMin = 10e6;

    for( tmpExp = 0; tmpExp < 8; tmpExp++ ) {
        for( tmpMant = 16; tmpMant <= 24; tmpMant += 4 ) {
            tmpRxBw = ComputeRxBw(tmpMant, tmpExp);
            if( fabs( tmpRxBw - rxBwValue ) < rxBwMin ) {
                rxBwMin = fabs( tmpRxBw - rxBwValue );
                *mantisse = tmpMant;
                *exponent = tmpExp;
            }
        }
    }
}


uint32_t SX127x_fsk::get_rx_bw_hz(uint8_t addr)
{
    RegRxBw_t reg_bw;
    uint8_t mantissa;
    
    if (m_xcvr.RegOpMode.bits.LongRangeMode)
        return 0;

    reg_bw.octet = m_xcvr.read_reg(addr);
    switch (reg_bw.bits.Mantissa) {
        case 0: mantissa = 16; break;
        case 1: mantissa = 20; break;
        case 2: mantissa = 24; break;
        default: mantissa = 0; break;
    }

    if (addr == REG_FSK_RXBW)
        RegRxBw.octet = reg_bw.octet;
    else if (addr == REG_FSK_AFCBW)
        RegAfcBw.octet = reg_bw.octet;

    return ComputeRxBw(mantissa, reg_bw.bits.Exponent);    
}


void SX127x_fsk::set_rx_dcc_bw_hz(uint32_t bw_hz, char afc)
{
    uint8_t mantisse = 0;
    uint8_t exponent = 0;
    
    if (m_xcvr.RegOpMode.bits.LongRangeMode)
        return;    

    ComputeRxBwMantExp( bw_hz, &mantisse, &exponent );
    //printf("bw_hz:%d, mantisse:%d, exponent:%d\n", bw_hz, mantisse, exponent );
    switch( mantisse ) {
        case 16:
            RegRxBw.bits.Mantissa = 0;
            break;
        case 20:
            RegRxBw.bits.Mantissa = 1;
            break;
        case 24:
            RegRxBw.bits.Mantissa = 2;
            break;
        default:
            // Something went terribely wrong
            printf("maintisse:%d\n", mantisse);
            break;
    }
    RegRxBw.bits.Exponent = exponent;

    if (afc)
        m_xcvr.write_reg(REG_FSK_AFCBW, RegRxBw.octet);
    else
        m_xcvr.write_reg(REG_FSK_RXBW, RegRxBw.octet);
}


void SX127x_fsk::start_tx(uint16_t arg_len)
{
    uint16_t pkt_buf_len;
    RegIrqFlags2_t RegIrqFlags2;
    int maxlen = FSK_FIFO_SIZE-1;
    
    if (m_xcvr.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
        m_xcvr.set_opmode(RF_OPMODE_STANDBY);
    }

    if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 0) {
        m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0;
        m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    }

    RegPktConfig1.octet = m_xcvr.read_reg(REG_FSK_PACKETCONFIG1);
    if (RegPktConfig1.bits.PacketFormatVariable) {
        pkt_buf_len = arg_len;
    } else {
        pkt_buf_len = RegPktConfig2.bits.PayloadLength;
    }

    RegIrqFlags2.octet = m_xcvr.read_reg(REG_FSK_IRQFLAGS2);
    if (RegIrqFlags2.bits.FifoEmpty) {
        if (RegPktConfig1.bits.PacketFormatVariable) {
            --maxlen;   // space for length byte
            if (pkt_buf_len > maxlen) {
                /*setup_FifoLevel(FALLING_EDGE);
                radio_write_fifo_(pkt_buf, maxlen, pkt_buf_len);
                remaining_ = pkt_buf_len - maxlen;
                printf("var-oversized %d R=%d\r\n", pkt_buf_len, remaining_);*/
                printf("var-oversized %d\r\n", pkt_buf_len);
            } else {
                //setup_FifoLevel(NO_EDGE); // disable 
                write_fifo(pkt_buf_len);
                //remaining_ = 0; // all was sent
            }
        } else { // fixed-length pkt format...
            pkt_buf_len = RegPktConfig2.bits.PayloadLength;
            if (RegPktConfig2.bits.PayloadLength > maxlen) {
                /*setup_FifoLevel(FALLING_EDGE);
                radio_write_fifo_(pkt_buf, maxlen, -1);
                remaining_ = SX1272FSK->RegPktConfig2.bits.PayloadLength - maxlen;*/
                printf("todo: fsk large packet\r\n");
            } else {
                //setup_FifoLevel(NO_EDGE); // disable
                write_fifo(RegPktConfig2.bits.PayloadLength);
            }
        }
    } else
        printf("fifo not empty %02x\r\n", RegIrqFlags2.octet);

    m_xcvr.set_opmode(RF_OPMODE_TRANSMITTER);
    
}

void SX127x_fsk::config_dio0_for_pktmode_rx()
{
    if (RegPktConfig2.bits.DataModePacket) {
        if (RegPktConfig1.bits.CrcOn) {
            if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 1) {
                m_xcvr.RegDioMapping1.bits.Dio0Mapping = 1; // to CrcOk
                m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
            }
        } else { // Crc Off, use PayloadReady
            if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 0) {
                m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0; // to PayloadReady
                m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
            }
        }
    }
}

void SX127x_fsk::start_rx()
{
    if (m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
        
    if (m_xcvr.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
        // was already receiving, restart it
        m_xcvr.set_opmode(RF_OPMODE_STANDBY);
#if (MBED_MAJOR_VERSION < 6)
        ThisThread::sleep_for(10);
#else
        ThisThread::sleep_for(10ms);
#endif
    }
    
    config_dio0_for_pktmode_rx();
    
    m_xcvr.set_opmode(RF_OPMODE_RECEIVER);        
}

uint8_t SX127x_fsk::get_modulation_shaping()
{
    if (m_xcvr.type == SX1276) {
        RegPaRamp_t reg_paramp;
        reg_paramp.octet = m_xcvr.read_reg(REG_PARAMP);
        return reg_paramp.bits.ModulationShaping;
    } else if (m_xcvr.type == SX1272) {
        m_xcvr.RegOpMode.octet = m_xcvr.read_reg(REG_OPMODE);
        return m_xcvr.RegOpMode.bits.ModulationShaping;
    } else
        return 0xff;
}

void SX127x_fsk::set_modulation_shaping(uint8_t s)
{
    if (m_xcvr.type == SX1276) {
        RegPaRamp_t reg_paramp;
        reg_paramp.octet = m_xcvr.read_reg(REG_PARAMP);
        reg_paramp.bits.ModulationShaping = s;
        m_xcvr.write_reg(REG_PARAMP, reg_paramp.octet);
    } else if (m_xcvr.type == SX1272) {
        m_xcvr.RegOpMode.octet = m_xcvr.read_reg(REG_OPMODE);
        m_xcvr.RegOpMode.bits.ModulationShaping = s;
        m_xcvr.write_reg(REG_OPMODE, m_xcvr.RegOpMode.octet);
    } 
}

service_action_e SX127x_fsk::service()
{
    int i;
    
    if (m_xcvr.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {
        if (m_xcvr.dio0) {
            /* packetSent comes at start of last bit, wait for one bit period */
            wait_us(bit_period_us);
            if (tx_done_sleep)
                m_xcvr.set_opmode(RF_OPMODE_SLEEP);
            else
                m_xcvr.set_opmode(RF_OPMODE_STANDBY);
            return SERVICE_TX_DONE;
        }
    } else if (RegPktConfig2.bits.DataModePacket && m_xcvr.dio0) {
        if (RegRxConfig.bits.AfcAutoOn)
            RegAfcValue = m_xcvr.read_s16(REG_FSK_AFCMSB);
            
        if (RegPktConfig1.bits.PacketFormatVariable) {
            rx_buf_length = m_xcvr.read_reg(REG_FIFO);
        } else {
            rx_buf_length = RegPktConfig2.bits.PayloadLength;
        }
        
        m_xcvr.m_cs = 0;
        m_xcvr.m_spi.write(REG_FIFO); // bit7 is low for reading from radio
        for (i = 0; i < rx_buf_length; i++) {
            m_xcvr.rx_buf[i] = m_xcvr.m_spi.write(0);
        }
        m_xcvr.m_cs = 1;
        return SERVICE_READ_FIFO;
    }
    
    return SERVICE_NONE;    
}
