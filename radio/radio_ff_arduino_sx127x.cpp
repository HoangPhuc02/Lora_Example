/* Only for NUCLEO boards: prevent compiling for MOTE_L152RC and typeABZ discovery */
#if defined(TARGET_FF_ARDUINO) && !defined(TARGET_DISCO_L072CZ_LRWAN1) && !defined(TARGET_MOTE_L152RC)
#include "radio.h"
#ifdef SX127x_H 
#include "SPIu.h"

SPIu spi(D11, D12, D13); // mosi, miso, sclk
//                  dio0, dio1, nss, spi, rst
SX127x Radio::radio(  D2,   D3, D10, spi, A0); // sx127[62] arduino shield
SX127x_lora Radio::lora(radio);
SX127x_fsk Radio::fsk(radio);

InterruptIn Radio::dio0(D2);
InterruptIn Radio::dio1(D3);

typedef enum {
    SHIELD_TYPE_NONE = 0,
    SHIELD_TYPE_LAS,
    SHIELD_TYPE_MAS,
} shield_type_e;
shield_type_e shield_type;

#ifdef TARGET_FF_MORPHO
DigitalOut pc3(PC_3);   // debug RX indication, for nucleo boards
#endif /* TARGET_FF_MORPHO */
DigitalInOut rfsw(A4);
void Radio::rfsw_callback()
{
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER)
        rfsw = 1;
    else
        rfsw = 0;

#ifdef TARGET_FF_MORPHO
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE)
        pc3 = 1;
    else
        pc3 = 0;
#endif /* TARGET_FF_MORPHO */
}

void
Radio::set_tx_dbm(int8_t dbm)
{
    RegPdsTrim1_t pds_trim;
    uint8_t v, adr, pa_test_adr;

    if (radio.type == SX1276) {
        adr = REG_PDSTRIM1_SX1276;
        pa_test_adr = REG_PATEST_SX1276;
    } else {
        adr = REG_PDSTRIM1_SX1272;
        pa_test_adr = REG_PATEST_SX1272;
    }
       
    v = radio.read_reg(pa_test_adr);
    if (dbm == PA_OFF_DBM) {
        /* for bench testing: prevent overloading receiving station (very low TX power) */
        v &= ~0x20; // turn off pu_regpa_n: disable PA
        radio.write_reg(pa_test_adr, v);
        return;
    } else if ((v & 0x20) == 0) {
        v |= 0x20; // turn on pu_regpa_n: enable PA
        radio.write_reg(pa_test_adr, v);
    }

    pds_trim.octet = radio.read_reg(adr);   

    if (shield_type == SHIELD_TYPE_LAS)
        radio.RegPaConfig.bits.PaSelect = 1;
    else
        radio.RegPaConfig.bits.PaSelect = 0;
                
    if (radio.RegPaConfig.bits.PaSelect) {
        /* PABOOST used: +2dbm to +17, or +20 */
        if (dbm > 17) {
            if (dbm > 20)
                dbm = 20;
            dbm -= 3;
            pds_trim.bits.prog_txdac = 7;
            radio.write_reg(adr, pds_trim.octet);
            ocp(150);
        } else
            ocp(120);

        if (dbm > 1)
                radio.RegPaConfig.bits.OutputPower = dbm - 2;
    } else {
        /* RFO used: -1 to +14dbm */
        ocp(80);
        if (dbm < 15)
            radio.RegPaConfig.bits.OutputPower = dbm + 1;
    }
    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);

    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);
    if (radio.RegPaConfig.bits.PaSelect) {
        dbm = radio.RegPaConfig.bits.OutputPower + pds_trim.bits.prog_txdac - 2;
    } else {
        dbm = radio.RegPaConfig.bits.OutputPower - 1;
    }
}

void Radio::boardInit()
{
    printf("boardInit sx127x-ff-arduino ");
    rfsw.input();
    if (rfsw.read()) {
        shield_type = SHIELD_TYPE_LAS;
        printf("LAS\r\n");
    } else {
        shield_type = SHIELD_TYPE_MAS;
        printf("MAS\r\n");
    }
    
    rfsw.output();
}

#endif /* ..SX127x_H */
#endif /* ...sx127x shield */

