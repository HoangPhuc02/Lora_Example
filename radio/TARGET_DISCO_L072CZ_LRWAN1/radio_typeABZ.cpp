#include "radio.h"
#ifdef SX127x_H

#define SSA_BOARD   1

#include "SPIu.h"
SPIu spi(PA_7, PA_6, PB_3); // mosi, miso, sclk
//                  dio0, dio1,  nss,  spi,  rst
SX127x Radio::radio(PB_4, PB_1, PA_15, spi, PC_0); // sx1276 arduino shield
SX127x_lora Radio::lora(radio);
SX127x_fsk Radio::fsk(radio);

InterruptIn Radio::dio0(PB_4);
InterruptIn Radio::dio1(PB_1);

#define CRF1    PA_1
#define CRF2    PC_2
#define CRF3    PC_1
DigitalOut Vctl1(CRF1);
DigitalOut Vctl2(CRF2);
DigitalOut Vctl3(CRF3);

void Radio::rfsw_callback()
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

void
Radio::set_tx_dbm(int8_t dbm)
{
    RegPdsTrim1_t pds_trim;
    unsigned v = radio.read_reg(REG_PATEST_SX1276);

    if (dbm == PA_OFF_DBM) {
        /* for bench testing: prevent overloading receiving station (very low TX power) */
        v &= ~0x20; // turn off pu_regpa_n: disable PA
        radio.write_reg(REG_PATEST_SX1276, v);
        return;
    } else if ((v & 0x20) == 0) {
        v |= 0x20; // turn on pu_regpa_n: enable PA
        radio.write_reg(REG_PATEST_SX1276, v);
    }

    pds_trim.octet = radio.read_reg(REG_PDSTRIM1_SX1276);

    if (dbm > 13) {
        radio.RegPaConfig.bits.PaSelect = 1;
        if (dbm > 17) {
            pds_trim.bits.prog_txdac = 7;
            dbm -= 3;
            ocp(150);
        } else {
            pds_trim.bits.prog_txdac = 4;
            ocp(120);
        }

        radio.RegPaConfig.bits.OutputPower = dbm - 2;
    } else {
        pds_trim.bits.prog_txdac = 4;
        radio.RegPaConfig.bits.PaSelect = 0;
        radio.RegPaConfig.bits.OutputPower = dbm + 1;
        ocp(80);
    }

    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
    radio.write_reg(REG_PDSTRIM1_SX1276, pds_trim.octet);
}

#ifdef SSA_BOARD
DigitalOut pa12(PA_12); // tcxo enable
DigitalOut pa11(PA_11); // sw9v enable -> sw3v3
#endif
void Radio::boardInit()
{
#ifdef SSA_BOARD
    pa12 = 1; // tcxo
    pa11 = 1; // sw9v -> sw3v3
#endif    
}

#endif /* ..SX127x_H */
