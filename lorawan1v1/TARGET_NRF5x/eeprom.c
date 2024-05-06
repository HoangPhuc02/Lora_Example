#include <stdint.h>
#include "eeprom.h"

/* TODO: use non-volatile memory */
uint16_t devNonce;
uint16_t rjCount1;
uint16_t joinNonce;

uint32_t fcntUp;
uint32_t nfcntDown;
uint32_t afcntDown;

uint32_t eeprom_read(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            return devNonce;
        case EEPROM_RJCOUNT1:
            return rjCount1;
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            return joinNonce;
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            return fcntUp;
        case EEPROM_NFCNTDWN:
            return nfcntDown;
        case EEPROM_AFCNTDWN:
            return afcntDown;
#endif /* !LORAWAN_JOIN_EUI */
        default: return 0;
    }
}

int eeprom_increment_value(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            devNonce++;
            return 0;
        case EEPROM_RJCOUNT1:
            rjCount1++;
            return 0;
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            joinNonce++;
            return 0;
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            fcntUp++;
            return 0;
        case EEPROM_NFCNTDWN:
            nfcntDown++;
            return 0;
        case EEPROM_AFCNTDWN:
            afcntDown++;
            return 0;
#endif /* !LORAWAN_JOIN_EUI */
        default:
            return -1;
    }
}

int eeprom_clear(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            devNonce++;
            return 0;
        case EEPROM_RJCOUNT1:
            rjCount1++;
            return 0;
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            joinNonce++;
            return 0;
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            fcntUp++;
            return 0;
        case EEPROM_NFCNTDWN:
            nfcntDown++;
            return 0;
        case EEPROM_AFCNTDWN:
            afcntDown++;
            return 0;
#endif /* !LORAWAN_JOIN_EUI */
        default: return 0;
    }
}

int eeprom_write_word(eeprom_value_e ev, uint32_t value)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
        case EEPROM_RJCOUNT1:
            return -1;
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            joinNonce = value;
            return 0;
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            fcntUp = value;
            return 0;
        case EEPROM_NFCNTDWN:
            nfcntDown = value;
            return 0;
        case EEPROM_AFCNTDWN:
            afcntDown = value;
            return 0;
#endif /* !LORAWAN_JOIN_EUI */
        default:
            return -1;
    }
}

