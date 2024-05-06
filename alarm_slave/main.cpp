#include "radio.h"
 
DigitalOut myled(LED1);

#if defined(SX127x_H) || defined(SX126x_H)
    #define BW_KHZ              500
    #define SPREADING_FACTOR    11
    #define CF_HZ               910800000
#elif defined(SX128x_H)
    #define BW_KHZ              200
    #define SPREADING_FACTOR    11
    #define CF_HZ               2487000000
#endif

#ifdef TARGET_DISCO_L072CZ_LRWAN1
    DigitalOut pinA(PB_12);
    DigitalOut pinB(PB_13);
    DigitalOut pinC(PB_14);
    DigitalOut pinD(PB_15);
#else
    DigitalOut pinA(PC_3);
    DigitalOut pinB(PC_2);
    DigitalOut pinC(PC_6);
    DigitalOut pinD(PC_8);
#endif /* !TARGET_DISCO_L072CZ_LRWAN1 */

DigitalOut* pin; 
Timeout to;

#define PIN_ASSERT_us       500000

#define CMD_PINA       0x02
#define CMD_PINB       0x03
#define CMD_PINC       0x06
#define CMD_PIND       0x08

/**********************************************************************/

void alarm_pin_clr()
{
    pin->write(0);
}

void alarm_pin_set()
{
    pin->write(1);
    to.attach_us(&alarm_pin_clr, PIN_ASSERT_us);
}

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

void get_alarm()
{
    uint16_t rx_crc, crc = crc_ccitt(Radio::radio.rx_buf, 5);
    rx_crc = Radio::radio.rx_buf[5];
    rx_crc <<= 8;
    rx_crc += Radio::radio.rx_buf[6];
    //printf("%u) crc rx:%04x, calc:%04x\r\n", lora.RegRxNbBytes, rx_crc, crc);
    if (crc == rx_crc) {
        uint8_t c = Radio::radio.rx_buf[0];
        if (c == CMD_PINA || c == CMD_PINB || c == CMD_PINC || c == CMD_PIND) {
            unsigned delay;
            delay = Radio::radio.rx_buf[1];
            delay <<= 8;
            delay += Radio::radio.rx_buf[2];
            delay <<= 8;
            delay += Radio::radio.rx_buf[3];
            delay <<= 8;
            delay += Radio::radio.rx_buf[4];
            switch (c) {
                case CMD_PINA: pin = &pinA; break;
                case CMD_PINB: pin = &pinB; break;
                case CMD_PINC: pin = &pinC; break;
                case CMD_PIND: pin = &pinD; break;
            }
            to.attach_us(&alarm_pin_set, delay);
            printf("delay:%u\r\n", delay);
        } else
            printf("cmd? %02x\r\n", Radio::radio.rx_buf[0]);
    } else
        printf("crc fail %04x, %04x\r\n", rx_crc, crc);
}

void txDoneCB()
{
}

void rxDoneCB(uint8_t size, float Rssi, float Snr)
{
    get_alarm();
    printf("%.1fdBm  snr:%.1fdB ", Rssi, Snr);
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
    printf("\r\nreset-rx\r\n");


    Radio::Init(&rev);

    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::LoRaPacketConfig(8, false, true, false);  // preambleLen, fixLen, crcOn, invIQ
    Radio::SetChannel(CF_HZ);
    
    Radio::Rx(0);
    
    for (;;) {     
        Radio::service();
    }
}
