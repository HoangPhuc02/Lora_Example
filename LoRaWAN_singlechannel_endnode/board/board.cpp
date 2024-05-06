/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "mbed.h"
#include "board.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

UnbufferedSerial pc( USBTX, USBRX );

void app_printf(const char *fmt, ...)
{
    char n, str[64];
    va_list arg_ptr;

    pc.write("\e[35m", 5);

    va_start(arg_ptr, fmt);
    n = vsnprintf(str, sizeof(str), fmt, arg_ptr);
    va_end(arg_ptr);
    pc.write(str, n);

    pc.write("\e[0m", 4);
}

void mac_printf(const char *fmt, ...)
{
    char n, str[64];
    va_list arg_ptr;

    pc.write("\e[36m", 5);

    va_start(arg_ptr, fmt);
    n = vsnprintf(str, sizeof(str), fmt, arg_ptr);
    va_end(arg_ptr);
    pc.write(str, n);

    pc.write("\e[0m", 4);
}

void pc_printf(const char *fmt, ...)
{
    char n, str[64];
    va_list arg_ptr;

    va_start(arg_ptr, fmt);
    n = vsnprintf(str, sizeof(str), fmt, arg_ptr);
    va_end(arg_ptr);
    pc.write(str, n);
}

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInit( void )
{
/*  TODO: ban low-speed internal osc
    if (!LL_RCC_LSE_IsReady()) {
        for (;;) __NOP();
    }
*/
}

uint8_t BoardGetBatteryLevel( void ) 
{
    return 0xFE;
}

#ifdef TARGET_STM32L1   /* TARGET_NUCLEO_L152RE */
    /* 0x1ff80050: Cat1, Cat2 */
    #define ID1           ( 0x1ff800d0 ) /* Cat3, Cat4, Cat5 */
    #define ID2           ( ID1 + 0x04 )
    #define ID3           ( ID1 + 0x14 )
    DigitalOut rx_debug_pin(PC_3);
#elif defined(TARGET_STM32L0)   /* TARGET_NUCLEO_L073RZ */
    #define ID1           ( 0x1ff80050 )
    #define ID2           ( ID1 + 0x04 )
    #define ID3           ( ID1 + 0x14 )
    #ifdef TARGET_DISCO_L072CZ_LRWAN1
        DigitalOut rx_debug_pin(PA_0);
    #else
        DigitalOut rx_debug_pin(PC_3);
    #endif
#elif defined(TARGET_STM32L4)   /* TARGET_NUCLEO_L476RG */
    #define ID1           ( 0x1fff7590 )
    #define ID2           ( ID1 + 0x04 )
    #define ID3           ( ID1 + 0x08 )
    DigitalOut rx_debug_pin(PC_3);
#else
    #error "provide signature address for target"
#endif

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

