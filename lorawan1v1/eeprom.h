#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "Commissioning.h"
/* RJCount1 never wraps, RJcount0 resets on JoinAccept */
typedef enum {
#ifdef LORAWAN_JOIN_EUI /* OTA: */
    EEPROM_DEVNONCE,
    EEPROM_RJCOUNT1,
    #ifdef LORAWAN_ROOT_APPKEY
    EEPROM_JOINNONCE
    #endif /* LORAWAN_ROOT_APPKEY */
#else   /* ABP: */
    EEPROM_FCNTUP,
    EEPROM_NFCNTDWN,
    EEPROM_AFCNTDWN
#endif
} eeprom_value_e;

#if __cplusplus
extern "C" {
#endif
uint32_t eeprom_read(eeprom_value_e ev);

/* return -1 for failure */
int eeprom_increment_value(eeprom_value_e ev);
int eeprom_clear(eeprom_value_e ev);
int eeprom_write_word(eeprom_value_e ev, uint32_t value);
#if __cplusplus
};
#endif

#endif /* _EEPROM_H_ */
