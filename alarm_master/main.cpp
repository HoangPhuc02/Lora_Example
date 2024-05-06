//#include "user_platform.h"
#include "radio.h"

#if defined(SX127x_H) || defined(SX126x_H)
    #define BW_KHZ              500
    #define SPREADING_FACTOR    11
    #define CF_HZ               910800000
    #if defined(SX126x_H)
        #define TX_DBM              (Radio::chipType == CHIP_TYPE_SX1262 ? 20 : 14) 
    #else
        #define TX_DBM              17
    #endif
#elif defined(SX128x_H)
    #define BW_KHZ              200
    #define SPREADING_FACTOR    11
    #define CF_HZ               2487000000

    #define TX_DBM              5
#endif

#ifdef TARGET_DISCO_L072CZ_LRWAN1
    DigitalIn pinA(PB_12);
    DigitalIn pinB(PB_13);
    DigitalIn pinC(PB_14);
    DigitalIn pinD(PB_15);
#elif defined(TARGET_FF_MORPHO)
    DigitalIn pinA(PC_3);
    DigitalIn pinB(PC_2);
    DigitalIn pinC(PC_6);
    DigitalIn pinD(PC_8);
#endif 

volatile struct _f_ {
    uint8_t enable_pin_A : 1;
    uint8_t enable_pin_B : 1;
    uint8_t enable_pin_C : 1;
    uint8_t enable_pin_D : 1;
} flags;
 
Timer t;
#define CMD_PINA       0x02
#define CMD_PINB       0x03
#define CMD_PINC       0x06
#define CMD_PIND       0x08

volatile bool tx_done;

static uint16_t crc_ccitt( uint8_t *buffer, uint16_t length )
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

void transmit(unsigned target, uint8_t cmd)
{
    unsigned t_diff;
    uint16_t crc;

    Radio::radio.tx_buf[0] = cmd;
    t_diff = target - t.read_us();
    Radio::radio.tx_buf[1] = t_diff >> 24;
    Radio::radio.tx_buf[2] = t_diff >> 16;
    Radio::radio.tx_buf[3] = t_diff >> 8;
    Radio::radio.tx_buf[4] = t_diff & 0xff;
    crc = crc_ccitt(Radio::radio.tx_buf, 5);
    Radio::radio.tx_buf[5] = crc >> 8;
    Radio::radio.tx_buf[6] = crc & 0xff;

    Radio::Send(7, 0, 0, 0);

    for (tx_done = false; !tx_done; )
        Radio::service();

    printf("t_diff:%u crc:%04x\r\n", t_diff, crc);
}

#define TARGET_LATENCY      2000000
void send_alarm(uint8_t cmd)
{
    int i;
    unsigned target = t.read_us() + TARGET_LATENCY;
    printf("send_alarm() %u\n", target);

    for (i = 0; i < 5; i++) {
        transmit(target, cmd);
        wait(0.1);
    }
}

void debounce(DigitalIn* pin, uint8_t cmd)
{
    if (!pin->read()) {
        int i;
        for (i = 0; i < 5; i++) {
            wait(0.01);
            if (pin->read()) {
                printf("trans\r\n");
                break;
            }
        }
        if (i == 5)
            send_alarm(cmd);

        while (!pin->read())
            ;
    }
}

void txDoneCB()
{
    tx_done = true;
}

void rxDoneCB(uint8_t size, float Rssi, float Snr)
{
}

const RadioEvents_t rev = {
    /* Dio0_top_half */     NULL,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        NULL,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        NULL,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

int main()
{
    printf("\r\nreset-tx\r\n");

    pinA.mode(PullUp);
    pinB.mode(PullUp);
    pinC.mode(PullUp);
    pinD.mode(PullUp);

    wait(0.05);

    if (pinA.read() == 0) {
        printf("pinA-disabled\r\n");
        flags.enable_pin_A = 0;
    } else
        flags.enable_pin_A = 1;

    if (pinB.read() == 0) {
        printf("pinB-disabled\r\n");
        flags.enable_pin_B = 0;
    } else
        flags.enable_pin_B = 1;

    if (pinC.read() == 0) {
        printf("pinC-disabled\r\n");
        flags.enable_pin_C = 0;
    } else
        flags.enable_pin_C = 1;

    if (pinD.read() == 0) {
        printf("pinD-disabled\r\n");
        flags.enable_pin_D = 0;
    } else
        flags.enable_pin_D = 1;

    t.start();

    Radio::Init(&rev);

    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::LoRaPacketConfig(8, false, true, false);  // preambleLen, fixLen, crcOn, invIQ
    Radio::SetChannel(CF_HZ);

    Radio::set_tx_dbm(TX_DBM);
                
    for (;;) {       
        if (flags.enable_pin_A)
            debounce(&pinA, CMD_PINA);

        if (flags.enable_pin_B)
            debounce(&pinB, CMD_PINB);

        if (flags.enable_pin_C)
            debounce(&pinC, CMD_PINC);

        if (flags.enable_pin_D)
            debounce(&pinD, CMD_PIND);
    } // ..for (;;)
}
