#include "radio.h"

#define RADIO_RESET              PC_2 //NorAm_Mote Reset_sx
#define RADIO_MOSI               PB_15 //NorAm_Mote SPI2 Mosi
#define RADIO_MISO               PB_14 //NorAm_Mote SPI2 Miso
#define RADIO_SCLK               PB_13 //NorAm_Mote  SPI2 Clk
#define RADIO_NSS                PB_12 //NorAm_Mote SPI2 Nss
 
#define RADIO_DIO_0              PC_6 //NorAm_Mote DIO0 
#define RADIO_DIO_1              PC_10 //NorAm_Mote DIO1 
#define RADIO_DIO_2              PC_8 //NorAm_Mote DIO2 
#define RADIO_DIO_3              PB_4 //NorAm_Mote DIO3 
#define RADIO_DIO_4              PB_5 //NorAm_Mote DIO4 
#define RADIO_DIO_5              PB_6 //NorAm_Mote DIO5
 
#define RFSW1                    PC_4 //NorAm_Mote RFSwitch_CNTR_1
#define RFSW2                    PC_13 //NorAm_Mote RFSwitch_CNTR_2

//                    txpow:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14  15  16  17  18  19
const uint8_t PaBTable[20] = { 0, 0, 0, 0, 0, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15 };

//                    txpow:  20 21 22 23 24 25 26 27 28 29 30
const uint8_t RfoTable[11] = { 1, 1, 1, 2, 2, 3, 4, 5, 6, 8, 9 };
 
#include "SPIu.h"
SPIu spi(RADIO_MOSI, RADIO_MISO, RADIO_SCLK); // mosi, miso, sclk
//                   dio0, dio1, nss, spi, rst
SX127x Radio::radio(RADIO_DIO_0, RADIO_DIO_1, RADIO_NSS, spi, RADIO_RESET); // sx1276 arduino shield
SX127x_lora Radio::lora(radio);
SX127x_fsk Radio::fsk(radio);

DigitalOut rfsw1(RFSW1);
DigitalOut rfsw2(RFSW2);

InterruptIn Radio::dio0(RADIO_DIO_0);
InterruptIn Radio::dio1(RADIO_DIO_1);

DigitalOut red(LED1);
DigitalOut yellow(LED3);
#define LED_OFF     1
#define LED_ON      0

void Radio::rfsw_callback()
{
    if (radio.RegOpMode.bits.Mode == RF_OPMODE_TRANSMITTER) {  // start of transmission
        red = LED_ON;
        yellow = LED_OFF;
        if (radio.HF) {
            if (radio.RegPaConfig.bits.PaSelect) { // if PA_BOOST
                rfsw2 = 0;
                rfsw1 = 1;
            } else { // RFO to power amp
                rfsw2 = 1;
                rfsw1 = 0;            
            }
        } else {
            // todo: sx1276
        }
    } else if (radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER || radio.RegOpMode.bits.Mode == RF_OPMODE_RECEIVER_SINGLE || radio.RegOpMode.bits.Mode == RF_OPMODE_CAD) { // start of reception
        red = LED_OFF;
        yellow = LED_ON;
        if (radio.HF) {
            rfsw2 = 1;
            rfsw1 = 1;              
        } else {
            // todo: sx1276
        }
    } else { // RF switch shutdown
        yellow = LED_OFF;
        red = LED_OFF;
        rfsw2 = 0;
        rfsw1 = 0;     
    }
}

void
Radio::set_tx_dbm(int8_t dbm)
{
    //int i = dbm;
    RegPdsTrim1_t pds_trim;
    uint8_t adr;

    radio.RegPaConfig.octet = radio.read_reg(REG_PACONFIG);

    if (dbm < 20) {
        radio.RegPaConfig.bits.PaSelect = 1;
        if (dbm < 0)
            dbm = 0;
        radio.RegPaConfig.bits.OutputPower = PaBTable[dbm];
    } else {
        radio.RegPaConfig.bits.PaSelect = 0;
        if (dbm > 30)
            dbm = 30;
        radio.RegPaConfig.bits.OutputPower = RfoTable[dbm-20];
    }

    if (radio.type == SX1276)
        adr = REG_PDSTRIM1_SX1276;
    else
        adr = REG_PDSTRIM1_SX1272;
       
    pds_trim.octet = radio.read_reg(adr);   

    if (radio.RegPaConfig.bits.PaSelect) {
        /* PABOOST used: +2dbm to +17, or +20 */
        if (dbm == 20) {
            dbm -= 3;
            pds_trim.bits.prog_txdac = 7;
            radio.write_reg(adr, pds_trim.octet);
            ocp(150);
        } else if (dbm < 18) {
            pds_trim.bits.prog_txdac = 5;
            radio.write_reg(adr, pds_trim.octet);
            ocp(120);
        }
    } else
        ocp(80);

    radio.write_reg(REG_PACONFIG, radio.RegPaConfig.octet);
} // ..set_tx_dbm()

void Radio::boardInit()
{
    red = LED_OFF;
    yellow = LED_OFF;
}

