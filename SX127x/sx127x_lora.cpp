#include "sx127x_lora.h"

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

SX127x_lora::SX127x_lora(SX127x& r) : m_xcvr(r)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        enable();
        
    RegModemConfig.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG);
    RegModemConfig2.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG2);
    RegTest33.octet = m_xcvr.read_reg(REG_LR_TEST33);     // invert_i_q
    RegDriftInvert.octet = m_xcvr.read_reg(REG_LR_DRIFT_INVERT);
    RegGainDrift.octet = m_xcvr.read_reg(REG_LR_GAIN_DRIFT);
    
    if (m_xcvr.type == SX1276) {
        RegAutoDrift.octet = m_xcvr.read_reg(REG_LR_SX1276_AUTO_DRIFT);
    }
    

}

SX127x_lora::~SX127x_lora()
{
}
    
void SX127x_lora::write_fifo(uint8_t len)
{
    int i;
    
    m_xcvr.m_cs = 0;
    m_xcvr.m_spi.write(REG_FIFO | 0x80); // bit7 is high for writing to radio
    
    for (i = 0; i < len; i++) {
        m_xcvr.m_spi.write(m_xcvr.tx_buf[i]);
    }
    m_xcvr.m_cs = 1;
}

void SX127x_lora::read_fifo(uint8_t len)
{
    int i;
     
    m_xcvr.m_cs = 0;
    m_xcvr.m_spi.write(REG_FIFO); // bit7 is low for reading from radio
    for (i = 0; i < len; i++) {
        m_xcvr.rx_buf[i] = m_xcvr.m_spi.write(0);
    }
    m_xcvr.m_cs = 1;
}

void SX127x_lora::enable()
{
    m_xcvr.set_opmode(RF_OPMODE_SLEEP);
    
    m_xcvr.RegOpMode.bits.LongRangeMode = 1;
    m_xcvr.write_reg(REG_OPMODE, m_xcvr.RegOpMode.octet);
    
    m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
    m_xcvr.RegDioMapping1.bits.Dio1Mapping = 0;
    m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    
    RegTest31.octet = m_xcvr.read_reg(REG_LR_TEST31);    
    RegTest31.bits.if_freq_auto = 0;    // improved RX spurious rejection
    m_xcvr.write_reg(REG_LR_TEST31, RegTest31.octet);    
        
    m_xcvr.set_opmode(RF_OPMODE_STANDBY);            
}

uint8_t SX127x_lora::getCodingRate(bool from_rx)
{
    if (from_rx) {
        // expected RegModemStatus was read on RxDone interrupt
        return RegModemStatus.bits.RxCodingRate;    
    } else {    // transmitted coding rate...
        if (m_xcvr.type == SX1276)
            return RegModemConfig.sx1276bits.CodingRate;
        else if (m_xcvr.type == SX1272)
            return RegModemConfig.sx1272bits.CodingRate;
        else
            return 0;
    }
}



void SX127x_lora::setCodingRate(uint8_t cr)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
        
    if (m_xcvr.type == SX1276)
        RegModemConfig.sx1276bits.CodingRate = cr;
    else if (m_xcvr.type == SX1272)
        RegModemConfig.sx1272bits.CodingRate = cr;
    else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}



bool SX127x_lora::getHeaderMode(void)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1276bits.ImplicitHeaderModeOn;
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1272bits.ImplicitHeaderModeOn;
    } else
        return false;
}

void SX127x_lora::setHeaderMode(bool hm)
{
    if (m_xcvr.type == SX1276)
        RegModemConfig.sx1276bits.ImplicitHeaderModeOn = hm;
    else if (m_xcvr.type == SX1272)
        RegModemConfig.sx1272bits.ImplicitHeaderModeOn = hm;
    else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}


uint8_t SX127x_lora::getBw(void)
{
    if (m_xcvr.type == SX1276)
        return RegModemConfig.sx1276bits.Bw;
    else if (m_xcvr.type == SX1272)
        return RegModemConfig.sx1272bits.Bw;
    else
        return 0;
}

int SX127x_lora::get_freq_error_Hz()
{
    int freq_error;
    float f, khz = 0;
    freq_error = m_xcvr.read_reg(REG_LR_TEST28);
    freq_error <<= 8;
    freq_error += m_xcvr.read_reg(REG_LR_TEST29);
    freq_error <<= 8;
    freq_error += m_xcvr.read_reg(REG_LR_TEST2A);
    if (freq_error & 0x80000) {  // 20bit value is negative
        //signed 20bit to 32bit
        freq_error |= 0xfff00000;
    }   
    f = freq_error / (float)XTAL_FREQ;
    f *= (float)0x1000000; // 2^24
    if (m_xcvr.type == SX1272) {
        switch (RegModemConfig.sx1272bits.Bw) {
            case 0: khz = 125; break;
            case 1: khz = 250; break;
            case 2: khz = 500; break;                
        }
    } else if (m_xcvr.type == SX1276) {
        switch (RegModemConfig.sx1276bits.Bw) {
            case 0: khz = 7.8; break;
            case 1: khz = 10.4; break;
            case 2: khz = 15.6; break;
            case 3: khz = 20.8; break;
            case 4: khz = 31.25; break;
            case 5: khz = 41.7; break;
            case 6: khz = 62.5; break;
            case 7: khz = 125; break;
            case 8: khz = 250; break;
            case 9: khz = 500; break;            
        }
    }
    f *= khz / 500;
    return (int)f;
}

float SX127x_lora::get_symbol_period()
{
    float khz = 0;
    
    if (m_xcvr.type == SX1276) {
        switch (RegModemConfig.sx1276bits.Bw) {
            case 0: khz = 7.8; break;
            case 1: khz = 10.4; break;
            case 2: khz = 15.6; break;
            case 3: khz = 20.8; break;
            case 4: khz = 31.25; break;
            case 5: khz = 41.7; break;
            case 6: khz = 62.5; break;
            case 7: khz = 125; break;
            case 8: khz = 250; break;
            case 9: khz = 500; break;
        }
    } else if (m_xcvr.type == SX1272) {
        switch (RegModemConfig.sx1272bits.Bw) {
            case 0: khz = 125; break;
            case 1: khz = 250; break;
            case 2: khz = 500; break;            
        }
    }
    
    // return symbol duration in milliseconds
    return (1 << RegModemConfig2.sx1276bits.SpreadingFactor) / khz; 
}

void SX127x_lora::setBw_KHz(int khz)
{
    uint8_t bw = 0;
    
    if (m_xcvr.type == SX1276) {
        if (khz <= 8) bw = 0;
        else if (khz <= 11) bw = 1;
        else if (khz <= 16) bw = 2;
        else if (khz <= 21) bw = 3;
        else if (khz <= 32) bw = 4;
        else if (khz <= 42) bw = 5;
        else if (khz <= 63) bw = 6;
        else if (khz <= 125) bw = 7;
        else if (khz <= 250) bw = 8;
        else if (khz <= 500) bw = 9;
    } else if (m_xcvr.type == SX1272) {
        if (khz <= 125) bw = 0;
        else if (khz <= 250) bw = 1;
        else if (khz <= 500) bw = 2;
    }
    
    setBw(bw);
}

void SX127x_lora::setBw(uint8_t bw)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return;
        
    if (m_xcvr.type == SX1276) {        
        RegModemConfig.sx1276bits.Bw = bw;
        if (get_symbol_period() > 16)
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);        
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig.sx1272bits.Bw = bw;
        if (get_symbol_period() > 16)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
    } else
        return;
        
    m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
}



uint8_t SX127x_lora::getSf(void)
{
    // spreading factor same between sx127[26]
    return RegModemConfig2.sx1276bits.SpreadingFactor;
}

void SX127x_lora::set_nb_trig_peaks(int n)
{
    /* TODO: different requirements for RX_CONTINUOUS vs RX_SINGLE */
    RegTest31.bits.detect_trig_same_peaks_nb = n;
    m_xcvr.write_reg(REG_LR_TEST31, RegTest31.octet);
}


void SX127x_lora::setSf(uint8_t sf)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return; 

    // write register at 0x37 with value 0xc if at SF6
    if (sf < 7)
        m_xcvr.write_reg(REG_LR_DETECTION_THRESHOLD, 0x0c);
    else
        m_xcvr.write_reg(REG_LR_DETECTION_THRESHOLD, 0x0a);
    
    RegModemConfig2.sx1276bits.SpreadingFactor = sf; // spreading factor same between sx127[26]
    m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    
    if (m_xcvr.type == SX1272) {
        if (get_symbol_period() > 16)
            RegModemConfig.sx1272bits.LowDataRateOptimize = 1;
        else
            RegModemConfig.sx1272bits.LowDataRateOptimize = 0;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
    } else if (m_xcvr.type == SX1276) {
        if (get_symbol_period() > 16)
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 1;
        else
            RegModemConfig3.sx1276bits.LowDataRateOptimize = 0;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    }
}


        
bool SX127x_lora::getRxPayloadCrcOn(void)
{
    /* RxPayloadCrcOn enables CRC generation in transmitter */
    /* in implicit mode, this bit also enables CRC in receiver */
    if (m_xcvr.type == SX1276) {
        RegModemConfig2.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG2);
        return RegModemConfig2.sx1276bits.RxPayloadCrcOn;
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG);
        return RegModemConfig.sx1272bits.RxPayloadCrcOn;
    } else
        return 0;
}


void SX127x_lora::setRxPayloadCrcOn(bool on)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig2.sx1276bits.RxPayloadCrcOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig.sx1272bits.RxPayloadCrcOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG, RegModemConfig.octet);
    }   
}



bool SX127x_lora::getAgcAutoOn(void)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig3.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG3);
        return RegModemConfig3.sx1276bits.AgcAutoOn;
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig2.octet = m_xcvr.read_reg(REG_LR_MODEMCONFIG2);
        return RegModemConfig2.sx1272bits.AgcAutoOn;
    } else
        return 0;
}

void SX127x_lora::setAgcAutoOn(bool on)
{
    if (m_xcvr.type == SX1276) {
        RegModemConfig3.sx1276bits.AgcAutoOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG3, RegModemConfig3.octet);
    } else if (m_xcvr.type == SX1272) {
        RegModemConfig2.sx1272bits.AgcAutoOn = on;
        m_xcvr.write_reg(REG_LR_MODEMCONFIG2, RegModemConfig2.octet);
    }
    
}

void SX127x_lora::invert_tx(bool inv)
{
    RegTest33.bits.chirp_invert_tx = !inv;
    m_xcvr.write_reg(REG_LR_TEST33, RegTest33.octet);    
}

void SX127x_lora::invert_rx(bool inv)
{
    RegTest33.bits.invert_i_q = inv;
    m_xcvr.write_reg(REG_LR_TEST33, RegTest33.octet);
    /**/
    RegDriftInvert.bits.invert_timing_error_per_symbol = !RegTest33.bits.invert_i_q;    
    m_xcvr.write_reg(REG_LR_DRIFT_INVERT, RegDriftInvert.octet);
}

void SX127x_lora::start_tx(uint8_t len)
{                   
    // DIO0 to TxDone
    if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 1) {
        m_xcvr.RegDioMapping1.bits.Dio0Mapping = 1;
        m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    }
    
    // set FifoPtrAddr to FifoTxPtrBase
    m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFOTXBASEADDR));
    
    // write PayloadLength bytes to fifo
    write_fifo(len);
       
    m_xcvr.set_opmode(RF_OPMODE_TRANSMITTER);
}

void SX127x_lora::start_rx(chip_mode_e mode)
{
    if (!m_xcvr.RegOpMode.bits.LongRangeMode)
        return; // fsk mode
    if (m_xcvr.RegOpMode.sx1276LORAbits.AccessSharedReg)
        return; // fsk page
        
    if (m_xcvr.type == SX1276) {
        if (RegModemConfig.sx1276bits.Bw == 9) {  // if 500KHz bw: improved tolerance of reference frequency error
            if (RegAutoDrift.bits.freq_to_time_drift_auto) {
                RegAutoDrift.bits.freq_to_time_drift_auto = 0;
                m_xcvr.write_reg(REG_LR_SX1276_AUTO_DRIFT, RegAutoDrift.octet);
            }
            if (m_xcvr.HF) {
                // > 525MHz
                if (RegGainDrift.bits.freq_to_time_drift != 0x24) {
                    RegGainDrift.bits.freq_to_time_drift = 0x24;
                    m_xcvr.write_reg(REG_LR_GAIN_DRIFT, RegGainDrift.octet);                    
                }
            } else {
                // < 525MHz
                if (RegGainDrift.bits.freq_to_time_drift != 0x3f) {
                    RegGainDrift.bits.freq_to_time_drift = 0x3f;
                    m_xcvr.write_reg(REG_LR_GAIN_DRIFT, RegGainDrift.octet); 
                }
            }

        } else {
            if (!RegAutoDrift.bits.freq_to_time_drift_auto) {
                RegAutoDrift.bits.freq_to_time_drift_auto = 1;
                m_xcvr.write_reg(REG_LR_SX1276_AUTO_DRIFT, RegAutoDrift.octet);
            }
        }
    } // ... if (m_xcvr.type == SX1276)  
    
    // RX_CONTINUOUS: false detections vs missed detections tradeoff
    switch (RegModemConfig2.sx1276bits.SpreadingFactor) {
        case 6:
            set_nb_trig_peaks(3);
            break;
        case 7:
            set_nb_trig_peaks(4);
            break;
        default:
            set_nb_trig_peaks(5);
            break;
    }   
        
    m_xcvr.set_opmode(mode);

    if (m_xcvr.RegDioMapping1.bits.Dio0Mapping != 0) {
        m_xcvr.RegDioMapping1.bits.Dio0Mapping = 0;    // DIO0 to RxDone
        m_xcvr.write_reg(REG_DIOMAPPING1, m_xcvr.RegDioMapping1.octet);
    }
    
    m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFORXBASEADDR));
}

int SX127x_lora::get_pkt_rssi()
{
    if (m_xcvr.type == SX1276) {
        if (m_xcvr.HF)
            return RegPktRssiValue - 157;
        else
            return RegPktRssiValue - 164;
    } else
        return RegPktRssiValue - 139;
}

int SX127x_lora::get_current_rssi()
{
    uint8_t v = m_xcvr.read_reg(REG_LR_RSSIVALUE);
    if (m_xcvr.type == SX1276) {
        if (m_xcvr.HF)
            return v - 157;
        else
            return v - 164;
    } else
        return v - 139;
}

service_action_e SX127x_lora::service()
{
    if (m_xcvr.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER) {
        if (poll_vh) {
            RegIrqFlags.octet = m_xcvr.read_reg(REG_LR_IRQFLAGS);
            if (RegIrqFlags.bits.ValidHeader) {
                RegIrqFlags.octet = 0;
                RegIrqFlags.bits.ValidHeader = 1;
                m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);
                printf("VH\r\n");
            }
        }
    }
       
    if (m_xcvr.dio0 == 0)
        return SERVICE_NONE;
        
    switch (m_xcvr.RegDioMapping1.bits.Dio0Mapping) {
        case 0: // RxDone
            /* user checks for CRC error in IrqFlags */
            RegIrqFlags.octet = m_xcvr.read_reg(REG_LR_IRQFLAGS);  // save flags
            RegHopChannel.octet = m_xcvr.read_reg(REG_LR_HOPCHANNEL);
            //printf("[%02x]", RegIrqFlags.octet);
            m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet); // clear flags in radio
            
            /* any register of interest on received packet is read(saved) here */        
            RegModemStatus.octet = m_xcvr.read_reg(REG_LR_MODEMSTAT);          
            RegPktSnrValue = m_xcvr.read_reg(REG_LR_PKTSNRVALUE);
            RegPktRssiValue = m_xcvr.read_reg(REG_LR_PKTRSSIVALUE);
            RegRxNbBytes = m_xcvr.read_reg(REG_LR_RXNBBYTES);
    
            m_xcvr.write_reg(REG_LR_FIFOADDRPTR, m_xcvr.read_reg(REG_LR_FIFORXCURRENTADDR));
            read_fifo(RegRxNbBytes);
            return SERVICE_READ_FIFO;
        case 1: // TxDone
            RegIrqFlags.octet = 0;
            RegIrqFlags.bits.TxDone = 1;
            m_xcvr.write_reg(REG_LR_IRQFLAGS, RegIrqFlags.octet);                  
            return SERVICE_TX_DONE;        
    } // ...switch (RegDioMapping1.bits.Dio0Mapping)
    
    return SERVICE_ERROR;    
}

