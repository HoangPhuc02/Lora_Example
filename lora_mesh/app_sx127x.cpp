#include "sx12xx.h"
#ifdef SX127x_H 
#include "main.h"

void lora_print_dio()
{
    Radio::radio.RegDioMapping2.octet = Radio::radio.read_reg(REG_DIOMAPPING2);
    pc.printf("DIO5:");
    switch (Radio::radio.RegDioMapping2.bits.Dio5Mapping) {
        case 0: pc.printf("ModeReady"); break;
        case 1: pc.printf("ClkOut"); break;
        case 2: pc.printf("ClkOut"); break;
    }
    pc.printf(" DIO4:");
    switch (Radio::radio.RegDioMapping2.bits.Dio4Mapping) {
        case 0: pc.printf("CadDetected"); break;
        case 1: pc.printf("PllLock"); break;
        case 2: pc.printf("PllLock"); break;
    }    
    Radio::radio.RegDioMapping1.octet = Radio::radio.read_reg(REG_DIOMAPPING1);
    pc.printf(" DIO3:");
    switch (Radio::radio.RegDioMapping1.bits.Dio3Mapping) {
        case 0: pc.printf("CadDone"); break;
        case 1: pc.printf("ValidHeader"); break;
        case 2: pc.printf("PayloadCrcError"); break;
    }    
    pc.printf(" DIO2:");
    switch (Radio::radio.RegDioMapping1.bits.Dio2Mapping) {
        case 0:
        case 1:
        case 2:
            pc.printf("FhssChangeChannel");
            break;
    }    
    pc.printf(" DIO1:");
    switch (Radio::radio.RegDioMapping1.bits.Dio1Mapping) {
        case 0: pc.printf("RxTimeout"); break;
        case 1: pc.printf("FhssChangeChannel"); break;
        case 2: pc.printf("CadDetected"); break;
    }    
    pc.printf(" DIO0:");
    switch (Radio::radio.RegDioMapping1.bits.Dio0Mapping) {
        case 0: pc.printf("RxDone"); break;
        case 1: pc.printf("TxDone"); break;
        case 2: pc.printf("CadDone"); break;
    }    
    
    pc.printf("\r\n"); 
}

void lora_printAgcAutoOn()
{
    pc.printf("AgcAutoOn:%d", Radio::lora.getAgcAutoOn());
}

void lora_printBw()
{
    (void)Radio::lora.getBw();
    
    pc.printf("Bw:");
    if (Radio::radio.type == SX1276) {
        switch (Radio::lora.RegModemConfig.sx1276bits.Bw) {
            case 0: pc.printf("7.8KHz "); break;
            case 1: pc.printf("10.4KHz "); break;
            case 2: pc.printf("15.6KHz "); break;
            case 3: pc.printf("20.8KHz "); break;
            case 4: pc.printf("31.25KHz "); break;
            case 5: pc.printf("41.7KHz "); break;
            case 6: pc.printf("62.5KHz "); break;
            case 7: pc.printf("125KHz "); break;
            case 8: pc.printf("250KHz "); break;
            case 9: pc.printf("500KHz "); break;
            default: pc.printf("%x ", Radio::lora.RegModemConfig.sx1276bits.Bw); break;
        }
    } else if (Radio::radio.type == SX1272) {
        switch (Radio::lora.RegModemConfig.sx1272bits.Bw) {
            case 0: pc.printf("125KHz "); break;
            case 1: pc.printf("250KHz "); break;
            case 2: pc.printf("500KHz "); break;
            case 3: pc.printf("11b "); break;
        }
    }
}

void lora_printSf()
{
    // spreading factor same between sx127[26]
    pc.printf("sf:%d ", Radio::lora.getSf());
}

void lora_printTxContinuousMode()
{
    pc.printf("TxContinuousMode:%d ", Radio::lora.RegModemConfig2.sx1276bits.TxContinuousMode);    // same for sx1272 and sx1276
}

void radio_printLoraIrqs(bool clear)
{
    //RegIrqFlags_t RegIrqFlags;
 
    Radio::lora.RegIrqFlags.octet = Radio::radio.read_reg(REG_LR_IRQFLAGS);
    pc.printf("\r\nIrqFlags:");
    if (Radio::lora.RegIrqFlags.bits.CadDetected)
        pc.printf("CadDetected ");
    if (Radio::lora.RegIrqFlags.bits.FhssChangeChannel) {
        //radio.RegHopChannel.octet = Radio::radio.read_reg(REG_LR_HOPCHANNEL);
        pc.printf("FhssChangeChannel:%d ", Radio::lora.RegHopChannel.bits.FhssPresentChannel);
    }
    if (Radio::lora.RegIrqFlags.bits.CadDone)
        pc.printf("CadDone ");
    if (Radio::lora.RegIrqFlags.bits.TxDone)
        pc.printf("TxDone ");
    if (Radio::lora.RegIrqFlags.bits.ValidHeader)
        pc.printf("[42mValidHeader[0m ");
    if (Radio::lora.RegIrqFlags.bits.PayloadCrcError)
        pc.printf("[41mPayloadCrcError[0m ");
    if (Radio::lora.RegIrqFlags.bits.RxDone)
        pc.printf("[42mRxDone[0m ");  
    if (Radio::lora.RegIrqFlags.bits.RxTimeout)
        pc.printf("RxTimeout ");
 
    pc.printf("\r\n");
 
    if (clear)
        Radio::radio.write_reg(REG_LR_IRQFLAGS, Radio::lora.RegIrqFlags.octet);
 
}

void lora_printHeaderMode()
{
    if (Radio::lora.getHeaderMode())
        pc.printf("implicit ");
    else
        pc.printf("explicit ");
}

void lora_printRxPayloadCrcOn()
{
    bool on = Radio::lora.getRxPayloadCrcOn();
    pc.printf("RxPayloadCrcOn:%d = ", on);
    if (Radio::lora.getHeaderMode())
        pc.printf("Rx/");  // implicit mode
        
    if (on)
        pc.printf("Tx CRC Enabled\r\n");
    else
        pc.printf("Tx CRC disabled\r\n");
}

void lora_printCodingRate(bool from_rx)
{
    uint8_t d = Radio::lora.getCodingRate(from_rx);
    pc.printf("CodingRate:");
    switch (d) {
        case 1: pc.printf("4/5 "); break;
        case 2: pc.printf("4/6 "); break;
        case 3: pc.printf("4/7 "); break;
        case 4: pc.printf("4/8 "); break;
        default:
            pc.printf("%d ", d);
            break;
    }
}
 
void lora_print_status()
{    
    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    if (!Radio::radio.RegOpMode.bits.LongRangeMode) {
        pc.printf("FSK\r\n");
        return;
    }
    
    //lora_print_dio();
    pc.printf("LoRa ");
    
    // printing LoRa registers at 0x0d -> 0x3f
 
    Radio::lora.RegModemConfig.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG);
    Radio::lora.RegModemConfig2.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG2);
 
    lora_printCodingRate(false); // false: transmitted coding rate
    lora_printHeaderMode();
    lora_printBw();
    lora_printSf();
    lora_printRxPayloadCrcOn();
    // RegModemStat
    //pc.printf("ModemStat:0x%02x\r\n", Radio::radio.read_reg(REG_LR_MODEMSTAT));
 
    // fifo ptrs:
    Radio::lora.RegPayloadLength = Radio::radio.read_reg(REG_LR_PAYLOADLENGTH);
    Radio::lora.RegRxMaxPayloadLength = Radio::radio.read_reg(REG_LR_RX_MAX_PAYLOADLENGTH);
/*    pc.printf("fifoptr=0x%02x txbase=0x%02x rxbase=0x%02x payloadLength=0x%02x maxlen=0x%02x",
        Radio::radio.read_reg(REG_LR_FIFOADDRPTR),
        Radio::radio.read_reg(REG_LR_FIFOTXBASEADDR),
        Radio::radio.read_reg(REG_LR_FIFORXBASEADDR),
        Radio::lora.RegPayloadLength,
        Radio::lora.RegRxMaxPayloadLength
    );*/
 
    pc.printf("dio0pin:%u ", Radio::radio.dio0.read());
    Radio::lora.RegIrqFlags.octet = Radio::radio.read_reg(REG_LR_IRQFLAGS);
    radio_printLoraIrqs(false);
 
/*    Radio::lora.RegHopPeriod = Radio::radio.read_reg(REG_LR_HOPPERIOD);
    if (Radio::lora.RegHopPeriod != 0) {
        pc.printf("\r\nHopPeriod:0x%02x\r\n", Radio::lora.RegHopPeriod);
    }*/
 
    pc.printf("SymbTimeout:%d ", Radio::radio.read_u16(REG_LR_MODEMCONFIG2) & 0x3ff);
 
    Radio::lora.RegPreamble = Radio::radio.read_u16(REG_LR_PREAMBLEMSB);
    pc.printf("PreambleLength:%d ", Radio::lora.RegPreamble);
 
    if (Radio::radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || Radio::radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE) {
        pc.printf("rssi:%ddBm ", Radio::lora.get_current_rssi());
    }
 
    //lora_printTxContinuousMode();
 
    pc.printf("\r\n");
    //lora_printAgcAutoOn();
    if (Radio::radio.type == SX1272) {
        pc.printf(" LowDataRateOptimize:%d\r\n", Radio::lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    }
 
    /*pc.printf("\r\nHeaderCount:%d PacketCount:%d, ",
        Radio::radio.read_u16(REG_LR_RXHEADERCNTVALUE_MSB), Radio::radio.read_u16(REG_LR_RXPACKETCNTVALUE_MSB));
        */
 
    /*pc.printf("Lora detection threshold:%02x\r\n", Radio::radio.read_reg(REG_LR_DETECTION_THRESHOLD));
    Radio::lora.RegTest31.octet = Radio::radio.read_reg(REG_LR_TEST31);
    pc.printf("detect_trig_same_peaks_nb:%d\r\n", Radio::lora.RegTest31.bits.detect_trig_same_peaks_nb);*/
 
    if (Radio::radio.type == SX1272) {
        Radio::lora.RegModemConfig.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG);
        pc.printf("LowDataRateOptimize:%d ", Radio::lora.RegModemConfig.sx1272bits.LowDataRateOptimize);
    } else if (Radio::radio.type == SX1276) {
        Radio::lora.RegModemConfig3.octet = Radio::radio.read_reg(REG_LR_MODEMCONFIG3);
        pc.printf("LowDataRateOptimize:%d ", Radio::lora.RegModemConfig3.sx1276bits.LowDataRateOptimize);        
    }
    
    pc.printf(" invert: rx=%d tx=%d\r\n", Radio::lora.RegTest33.bits.invert_i_q, !Radio::lora.RegTest33.bits.chirp_invert_tx);
    
    pc.printf("\r\n");
}

void
printPa()
{
    Radio::radio.RegPaConfig.octet = Radio::radio.read_reg(REG_PACONFIG);
    if (Radio::radio.RegPaConfig.bits.PaSelect) {
        float output_dBm = 17 - (15-Radio::radio.RegPaConfig.bits.OutputPower);
        pc.printf(" PABOOST OutputPower=%.1fdBm", output_dBm);
    } else {
        float pmax = (0.6*Radio::radio.RegPaConfig.bits.MaxPower) + 10.8;
        float output_dBm = pmax - (15-Radio::radio.RegPaConfig.bits.OutputPower);
#ifdef TARGET_MTS_MDOT_F411RE
        pc.printf(" \x1b[31mRFO pmax=%.1fdBm OutputPower=%.1fdBm\x1b[0m", pmax, output_dBm);  // not connected
#else
        pc.printf(" RFO pmax=%.1fdBm OutputPower=%.1fdBm", pmax, output_dBm);
#endif
    }
}

bool isRadioRxing()
{
    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    return Radio::radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || Radio::radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE;
}

void radio_printOpMode()
{
    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    switch (Radio::radio.RegOpMode.bits.Mode) {
        case RF_OPMODE_SLEEP: pc.printf("[7msleep[0m"); break;
        case RF_OPMODE_STANDBY: pc.printf("[7mstby[0m"); break;
        case RF_OPMODE_SYNTHESIZER_TX: pc.printf("[33mfstx[0m"); break;
        case RF_OPMODE_TRANSMITTER: pc.printf("[31mtx[0m"); break;
        case RF_OPMODE_SYNTHESIZER_RX: pc.printf("[33mfsrx[0m"); break;
        case RF_OPMODE_RECEIVER: pc.printf("[32mrx[0m"); break;
        case 6:
            if (Radio::radio.RegOpMode.bits.LongRangeMode)
                pc.printf("[42mrxs[0m");
            else
                pc.printf("-6-");
            break;  // todo: different lora/fsk
        case 7:
            if (Radio::radio.RegOpMode.bits.LongRangeMode)
                pc.printf("[45mcad[0m");
            else
                pc.printf("-7-");
            break;  // todo: different lora/fsk
    }
}

void /* things always present, whether lora or fsk */
common_print_status()
{
    pc.printf("version:0x%02x %.3fMHz ", Radio::radio.read_reg(REG_VERSION), Radio::radio.get_frf_MHz());
    radio_printOpMode();
 
    printPa();
 
    Radio::radio.RegOcp.octet = Radio::radio.read_reg(REG_OCP);
    if (Radio::radio.RegOcp.bits.OcpOn) {
        int imax = 0;
        if (Radio::radio.RegOcp.bits.OcpTrim < 16)
            imax = 45 + (5 * Radio::radio.RegOcp.bits.OcpTrim);
        else if (Radio::radio.RegOcp.bits.OcpTrim < 28)
            imax = -30 + (10 * Radio::radio.RegOcp.bits.OcpTrim);
        else
            imax = 240;
        pc.printf(" OcpOn %dmA ", imax);
    } else
        pc.printf(" OcpOFF ");
 
    pc.printf("\r\n");
    
#if 0
    if (per_en) {
        if (cadper_enable) {
            pc.printf("cadper %" PRIu32 ", ", num_cads);
        }
        pc.printf("per_tx_delay:%f\r\n", per_tx_delay);
        pc.printf("PER device ID:%d\r\n", per_id);
    }    
    
    if (poll_irq_en) {
        pc.printf("poll_irq_en\r\n");
        if (!Radio::radio.RegOpMode.bits.LongRangeMode) {
            pc.printf("saved irqs: %02x %02x\r\n", fsk_RegIrqFlags1_prev.octet, fsk_RegIrqFlags2_prev.octet);
        }
    }
#endif /* if 0 */
 
}

void radio_print_status()
{
    if (Radio::radio.type == SX1276) {
#if defined(TARGET_MTS_MDOT_F411RE)
        pc.printf("\r\nSX1276 ");
#else
        /*if (shield_type == SHIELD_TYPE_LAS)
            pc.printf("\r\nSX1276LAS ");
        if (shield_type == SHIELD_TYPE_MAS)
            pc.printf("\r\nSX1276MAS ");*/
        pc.printf("\r\n");
#endif /* !TARGET_MTS_MDOT_F411RE */                       
    } else if (Radio::radio.type == SX1272)
        pc.printf("\r\nSX1272 ");
        
    Radio::radio.RegOpMode.octet = Radio::radio.read_reg(REG_OPMODE);
    if (Radio::radio.RegOpMode.bits.LongRangeMode)
        lora_print_status();
    /*else
        fsk_print_status();*/
    common_print_status();
}

void tx_dbm_print()
{
    int dbm;
    RegPdsTrim1_t pds_trim;
    uint8_t adr, pa_test_adr;

    if (Radio::radio.type == SX1276) {
        adr = REG_PDSTRIM1_SX1276;
        pa_test_adr = REG_PATEST_SX1276;
    } else {
        adr = REG_PDSTRIM1_SX1272;
        pa_test_adr = REG_PATEST_SX1272;
    }

    if (Radio::radio.read_reg(pa_test_adr) & 0x20) {
        pds_trim.octet = Radio::radio.read_reg(adr);

        Radio::radio.RegPaConfig.octet = Radio::radio.read_reg(REG_PACONFIG);
        if (Radio::radio.RegPaConfig.bits.PaSelect) {
            dbm = Radio::radio.RegPaConfig.bits.OutputPower + pds_trim.bits.prog_txdac - 2;
        } else {
            dbm = Radio::radio.RegPaConfig.bits.OutputPower - 1;
        }
    } else {
        dbm = PA_OFF_DBM;
    }
    pc.printf("%d", dbm);
}

bool tx_dbm_write(int i)
{
    uint8_t v, adr, pa_test_adr;
    RegPdsTrim1_t pds_trim;

    if (Radio::radio.type == SX1276) {
        adr = REG_PDSTRIM1_SX1276;
        pa_test_adr = REG_PATEST_SX1276;
    } else {
        adr = REG_PDSTRIM1_SX1272;
        pa_test_adr = REG_PATEST_SX1272;
    }

    v = Radio::radio.read_reg(pa_test_adr);

    if (i == PA_OFF_DBM) {
        /* for bench testing: prevent overloading receiving station (very low TX power) */
        v &= ~0x20; // turn off pu_regpa_n: disable PA
        Radio::radio.write_reg(pa_test_adr, v);
        return false;
    } else if ((v & 0x20) == 0) {
        v |= 0x20; // turn on pu_regpa_n: enable PA
        Radio::radio.write_reg(pa_test_adr, v);
    }

    pds_trim.octet = Radio::radio.read_reg(adr);

    if (Radio::radio.RegPaConfig.bits.PaSelect) {
        /* PABOOST used: +2dbm to +17, or +20 */
        if (i == 20) {
            pc.printf("+20dBm PADAC bias\r\n");
            i -= 3;
            pds_trim.bits.prog_txdac = 7;
            Radio::radio.write_reg(adr, pds_trim.octet);
        }
        if (i > 1)
                Radio::radio.RegPaConfig.bits.OutputPower = i - 2;
    } else {
        /* RFO used: -1 to +14dbm */
        if (i < 15)
            Radio::radio.RegPaConfig.bits.OutputPower = i + 1;
    }
    Radio::radio.write_reg(REG_PACONFIG, Radio::radio.RegPaConfig.octet);

    return false;
}

void cmd_op(uint8_t argsAt)
{
    int dbm;
    if (sscanf(pcbuf+argsAt, "%d", &dbm) == 1) {
        tx_dbm_write(dbm);
    }

    tx_dbm_print();
}
#endif /* ..SX127x_H */
