#include "sx12xx.h"
#ifdef SX126x_H 
#include "main.h"

uint8_t tx_param_buf[2];
uint8_t pa_config_buf[4];

void tx_dbm_print()
{
    PwrCtrl_t PwrCtrl;
    PaCtrl1b_t PaCtrl1b;
    unsigned v = Radio::radio.readReg(REG_ADDR_ANACTRL16, 1);

    if (v & 0x10) {
        pc.printf("%d", PA_OFF_DBM);
        return;
    }

    PwrCtrl.octet = Radio::radio.readReg(REG_ADDR_PWR_CTRL, 1);

    PaCtrl1b.octet = Radio::radio.readReg(REG_ADDR_PA_CTRL1B, 1);
    pa_config_buf[2] = PaCtrl1b.bits.tx_mode_bat;   // deviceSel

    if (PaCtrl1b.bits.tx_mode_bat)
        pc.printf("%ddBm ", PwrCtrl.bits.tx_pwr - 17);
    else
        pc.printf("%ddBm ", PwrCtrl.bits.tx_pwr - 9);
}

bool isRadioRxing()
{
    status_t status;
    Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
    return status.bits.chipMode == 5;
}

void radio_printOpMode()
{
    status_t status;
    Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);

    /* translate opmode_status_strs to opmode_select_strs */
    switch (status.bits.chipMode) {
        case 2: pc.printf("STBY_RC"); break;
        case 3: pc.printf("STBY_XOSC"); break;
        case 4: pc.printf("FS"); break;
        case 5: pc.printf("RX"); break;
        case 6: pc.printf("TX"); break;
        default: pc.printf("<%d>", status.bits.chipMode); break;
    }
}

void lora_printHeaderMode()
{
    loraConfig1_t conf1;
    conf1.octet = Radio::radio.readReg(REG_ADDR_LORA_CONFIG1, 1);
    if (conf1.bits.implicit_header)
        pc.printf("implicit ");
    else
        pc.printf("explicit ");
}

void radio_print_status()
{
    tx_dbm_print();
    pc.printf(" %.3fMHz ", Radio::radio.getMHz());
    radio_printOpMode();
    pc.printf("\r\n");
}

bool tx_dbm_write(int dbm)
{
    unsigned v = Radio::radio.readReg(REG_ADDR_ANACTRL16, 1);

    if (dbm == PA_OFF_DBM) {
        /* bench test: prevent overloading receiving station (very low tx power) */
        v |= 0x10;  // pa dac atb tst
        Radio::radio.writeReg(REG_ADDR_ANACTRL16, v, 1);
    } else {
        tx_param_buf[0] = dbm;
        Radio::radio.xfer(OPCODE_SET_TX_PARAMS, 2, 0, tx_param_buf);

        if (v & 0x10) {
            v &= ~0x10;
            Radio::radio.writeReg(REG_ADDR_ANACTRL16, v, 1);
        }
    }

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

#endif /* ..SX126x_H */

