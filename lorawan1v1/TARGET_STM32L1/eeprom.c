#include <stdint.h>
#include <stdbool.h>
#include "eeprom.h"
#include "Commissioning.h"

#include <stm32l1xx.h>
#define DEVNONCE_BASE           FLASH_EEPROM_BASE

#define DEVNONCE_END            (DEVNONCE_BASE + 16)  /* eight uint16_t values */
#define RJCOUNT1_BASE           DEVNONCE_END
#define RJCOUNT1_END            (RJCOUNT1_BASE + 16)
#define FCNTUP_BASE             RJCOUNT1_END
#define FCNTUP_END              (FCNTUP_BASE + 32)     /* eight uint32_t values */
#define NFCNTDWN_BASE           FCNTUP_END
#define NFCNTDWN_END            (NFCNTDWN_BASE + 32)     /* eight uint32_t values */
#define AFCNTDWN_BASE           NFCNTDWN_END
#define AFCNTDWN_END            (AFCNTDWN_BASE + 32)     /* eight uint32_t values */

#if defined(LORAWAN_JOIN_EUI) && defined(LORAWAN_ROOT_APPKEY)
    #define JOINNONCE_BASE          AFCNTDWN_END
    #define JOINNONCE_END           (JOINNONCE_BASE + 32)
#endif  // 1v1 OTA

#ifdef LORAWAN_JOIN_EUI
static uint16_t eeprom_read_halfword(uint32_t baseAddr, uint32_t endAddr)
{
    uint16_t* u16_ptr = (uint16_t*)baseAddr;
    uint16_t max_val = 0;
    while (u16_ptr < (uint16_t*)endAddr) {
        if (*u16_ptr > max_val)
            max_val = *u16_ptr;

        u16_ptr++;
    }
    return max_val;
}
#endif /* LORAWAN_JOIN_EUI */

#ifndef LORAWAN_JOIN_EUI
static uint32_t eeprom_read_word(uint32_t baseAddr, uint32_t endAddr)
{
    uint32_t* u32_ptr = (uint32_t*)baseAddr;
    uint32_t max_val = 0;
    while (u32_ptr < (uint32_t*)endAddr) {
        if (*u32_ptr > max_val)
            max_val = *u32_ptr;

        u32_ptr++;
    }
    return max_val;
}
#endif /* !LORAWAN_JOIN_EUI  */

uint32_t eeprom_read(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            return eeprom_read_halfword(DEVNONCE_BASE, DEVNONCE_END);
        case EEPROM_RJCOUNT1:
            return eeprom_read_halfword(RJCOUNT1_BASE, RJCOUNT1_END);
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            return eeprom_read_halfword(JOINNONCE_BASE, JOINNONCE_END);
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            return eeprom_read_word(FCNTUP_BASE, FCNTUP_END);
        case EEPROM_NFCNTDWN:
            return eeprom_read_word(NFCNTDWN_BASE, NFCNTDWN_END);
        case EEPROM_AFCNTDWN:
            return eeprom_read_word(AFCNTDWN_BASE, AFCNTDWN_END);
#endif /* !LORAWAN_JOIN_EUI */
        default: return 0;
    }
}

static void _getmax_halfword(uint32_t baseAddr, uint32_t endAddr, uint32_t* _max_val_at, uint32_t* max_val)
{
    uint16_t* u16_ptr = (uint16_t*)baseAddr;
    uint16_t* max_val_at = NULL;

    while (u16_ptr < (uint16_t*)endAddr) {
        if (*u16_ptr > *max_val) {
            *max_val = *u16_ptr;
            max_val_at = u16_ptr;
        }

        u16_ptr++;
    }

    if (max_val_at == NULL) {
        /* first time */
        max_val_at = (uint16_t*)baseAddr;
    } else {
        if (++max_val_at == (uint16_t*)endAddr)
            max_val_at = (uint16_t*)baseAddr;
    }

    *_max_val_at = (uint32_t)max_val_at;
}

static void _getmax_word(uint32_t baseAddr, uint32_t endAddr, uint32_t* max_val_at, uint32_t* max_val)
{
    uint32_t* u32_ptr = (uint32_t*)baseAddr;

    while (u32_ptr < (uint32_t*)endAddr) {
        if (*u32_ptr > *max_val) {
            *max_val = *u32_ptr;
            *max_val_at = (uint32_t)u32_ptr;
        }

        u32_ptr++;
    }

    if (*max_val_at == 0) {
        /* first time */
        *max_val_at = baseAddr;
    } else {
        *max_val_at += sizeof(uint32_t);
        if (*max_val_at == endAddr)
            *max_val_at = baseAddr;
    }
}

static int increment_eeprom(uint32_t erase_type, uint32_t baseAddr, uint32_t endAddr)
{
    bool erased;
    HAL_StatusTypeDef status;
    uint32_t prog_type;
    uint32_t max_val_at = 0;
    uint32_t max_val = 0;

    if (erase_type == FLASH_TYPEERASEDATA_HALFWORD) {
         _getmax_halfword(baseAddr, endAddr, &max_val_at, &max_val);
        prog_type = FLASH_TYPEPROGRAMDATA_HALFWORD;
    } else if (erase_type == FLASH_TYPEERASEDATA_WORD) {
         _getmax_word(baseAddr, endAddr, &max_val_at, &max_val);
        prog_type = FLASH_TYPEPROGRAMDATA_WORD;
    } else
        return -1;
    /* max_val_at now points to oldest */

    // 32bit FLASH_TYPEERASEDATA_WORD
    // 16bit FLASH_TYPEERASEDATA_HALFWORD
    // 8bit FLASH_TYPEERASEDATA_BYTE
    do {
        status = HAL_FLASHEx_DATAEEPROM_Erase(erase_type, max_val_at);
        if (status == HAL_OK)
            erased = true;
        else if (status == HAL_ERROR && HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP) {
            if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
                return -2;
        } else
            return -3;
    } while (!erased);

    if (HAL_FLASHEx_DATAEEPROM_Program(prog_type, max_val_at, max_val + 1) != HAL_OK) {
        HAL_FLASHEx_DATAEEPROM_Lock();
        return -4;
    }

    if (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK)
        return -5;

    return 0;
}

int eeprom_increment_value(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            return increment_eeprom(FLASH_TYPEERASEDATA_HALFWORD, DEVNONCE_BASE, DEVNONCE_END);
        case EEPROM_RJCOUNT1:
            return increment_eeprom(FLASH_TYPEERASEDATA_HALFWORD, RJCOUNT1_BASE, RJCOUNT1_END);
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            return increment_eeprom(FLASH_TYPEERASEDATA_WORD, JOINNONCE_BASE, JOINNONCE_END);
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            return increment_eeprom(FLASH_TYPEERASEDATA_WORD, FCNTUP_BASE, FCNTUP_END);
        case EEPROM_NFCNTDWN:
            return increment_eeprom(FLASH_TYPEERASEDATA_WORD, NFCNTDWN_BASE, NFCNTDWN_END);
        case EEPROM_AFCNTDWN:
            return increment_eeprom(FLASH_TYPEERASEDATA_WORD, AFCNTDWN_BASE, AFCNTDWN_END);
#endif /* !LORAWAN_JOIN_EUI */
        default:
            return -1;
    }
}

static int _eeprom_clear(uint32_t type, uint32_t baseAddr, uint32_t endAddr)
{
    int ret = 0;
    HAL_StatusTypeDef status;

    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK) {
        HAL_FLASHEx_DATAEEPROM_Lock();
        return -1;
    }

    if (type == FLASH_TYPEERASEDATA_HALFWORD) {
        uint16_t* u16_ptr = (uint16_t*)baseAddr;
        while (u16_ptr < (uint16_t*)endAddr) {
            if (*u16_ptr != 0) {
                status = HAL_FLASHEx_DATAEEPROM_Erase(type, (uint32_t)u16_ptr);
                if (status != HAL_OK) {
                    ret = -2;
                    break;
                }
            }
            u16_ptr++;
        }
    } else if (type == FLASH_TYPEERASEDATA_WORD) {
        uint32_t* u32_ptr = (uint32_t*)baseAddr;
        while (u32_ptr < (uint32_t*)endAddr) {
            if (*u32_ptr != 0) {
                status = HAL_FLASHEx_DATAEEPROM_Erase(type, (uint32_t)u32_ptr);
                if (status != HAL_OK) {
                    ret = -3;
                }
            }
            u32_ptr++;
        }
    } else
        ret = -1;

    HAL_FLASHEx_DATAEEPROM_Lock();

    return ret;
}

int eeprom_clear(eeprom_value_e ev)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
            return _eeprom_clear(FLASH_TYPEERASEDATA_HALFWORD, DEVNONCE_BASE, DEVNONCE_END);
        case EEPROM_RJCOUNT1:
            return _eeprom_clear(FLASH_TYPEERASEDATA_HALFWORD, RJCOUNT1_BASE, RJCOUNT1_END);
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            return _eeprom_clear(FLASH_TYPEERASEDATA_WORD, JOINNONCE_BASE, JOINNONCE_END);
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            return _eeprom_clear(FLASH_TYPEERASEDATA_WORD, FCNTUP_BASE, FCNTUP_END);
        case EEPROM_NFCNTDWN:
            return _eeprom_clear(FLASH_TYPEERASEDATA_WORD, NFCNTDWN_BASE, NFCNTDWN_END);
        case EEPROM_AFCNTDWN:
            return _eeprom_clear(FLASH_TYPEERASEDATA_WORD, AFCNTDWN_BASE, AFCNTDWN_END);
#endif /* !LORAWAN_JOIN_EUI */
        default: return 0;
    }
}

#if !defined(LORAWAN_JOIN_EUI) || (defined(LORAWAN_JOIN_EUI) && defined(LORAWAN_ROOT_APPKEY))
static int _write_word(uint32_t baseAddr, uint32_t endAddr, uint32_t value)
{
    bool erased;
    HAL_StatusTypeDef status;
    uint32_t max_val_at = 0;
    uint32_t max_val = 0;

    _getmax_word(baseAddr, endAddr, &max_val_at, &max_val);

    do {
        status = HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_WORD, max_val_at);
        if (status == HAL_OK)
            erased = true;
        else if (status == HAL_ERROR && HAL_FLASH_GetError() == HAL_FLASH_ERROR_WRP) {
            if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
                return -2;
        } else
            return -3;
    } while (!erased);

    if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, max_val_at, value) != HAL_OK) {
        HAL_FLASHEx_DATAEEPROM_Lock();
        return -4;
    }

    if (HAL_FLASHEx_DATAEEPROM_Lock() != HAL_OK)
        return -5;

    return 0;
}
#endif

int eeprom_write_word(eeprom_value_e ev, uint32_t value)
{
    switch (ev) {
#ifdef LORAWAN_JOIN_EUI
        case EEPROM_DEVNONCE:
        case EEPROM_RJCOUNT1:
            return -1;
    #ifdef LORAWAN_ROOT_APPKEY
        case EEPROM_JOINNONCE:
            return _write_word(JOINNONCE_BASE, JOINNONCE_END, value);
    #endif /* LORAWAN_ROOT_APPKEY */
#else
        case EEPROM_FCNTUP:
            return _write_word(FCNTUP_BASE, FCNTUP_END, value);
        case EEPROM_NFCNTDWN:
            return _write_word(NFCNTDWN_BASE, NFCNTDWN_END, value);
        case EEPROM_AFCNTDWN:
            return _write_word(AFCNTDWN_BASE, AFCNTDWN_END, value);
#endif /* !LORAWAN_JOIN_EUI */
        default:
            return -1;
    }
}
