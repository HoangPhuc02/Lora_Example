#include "mbed.h"
#ifdef MBED_DEBUG
//#error mbed_debug
#endif
#include "utilities.h"
#include "eeprom.h"
#include "Commissioning.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdlib.h>

#include "target.h"

#if defined( USE_BAND_868 )
    #define DUTY_ENABLE
    #include "duty.h"
#endif

#ifdef ENABLE_VT100
    #include "SerialDisplay.h"
#else
    extern RawSerial pc;
    #define APP_DEBUG
    //#define MAC_DEBUG
    //#define CRYPT_DEBUG
    //#define MIC_DEBUG_DOWN
    //#define DEBUG_MAC_CMD
#endif /* ENABLE_VT100 */
//#define MIC_DEBUG_UP
//#define MIC_DEBUG_DOWN

#ifdef APP_DEBUG
    #define APP_PRINTF(...) pc.printf("\e[33m" __VA_ARGS__); pc.printf("\e[0m")  /* yellow */
#else
    #define APP_PRINTF(...)    
#endif

#ifdef MAC_DEBUG
    #define DEBUG_MAC_BUF(w,x,y,z)    print_buf(w,x,y,z)
    #define MAC_PRINTF(...)           pc.printf("\e[36m" __VA_ARGS__); pc.printf("\e[0m")  /* cyan */
#else
    #define DEBUG_MAC_BUF(w,x,y,z)
    #define MAC_PRINTF(...)  
#endif

#ifdef CRYPT_DEBUG
    #define DEBUG_CRYPT_BUF(w,x,y,z)  print_buf(w,x,y,z)
    #define DEBUG_CRYPT(...)      pc.printf(__VA_ARGS__)
#else
    #define DEBUG_CRYPT_BUF(w,x,y,z)
    #define DEBUG_CRYPT(...)
#endif

#ifdef MIC_DEBUG_DOWN
    #ifdef ENABLE_VT100
        #define DEBUG_MIC_BUF_DOWN(w,x,y,z)    print_buf(w,x,y,z)
        #define DEBUG_MIC_DOWN(...)    vt.SetCursorPos(ROW_MIC, 1);      vt.printf(__VA_ARGS__)
    #else
        #define DEBUG_MIC_BUF_DOWN(w,x,y,z)    print_buf(w,x,y,z)
        #define DEBUG_MIC_DOWN(...)        pc.printf(__VA_ARGS__)
    #endif
#else
    #define DEBUG_MIC_BUF_DOWN(w,x,y,z)
    #define DEBUG_MIC_DOWN(...)
#endif

        
#ifdef MIC_DEBUG_UP
    #ifdef ENABLE_VT100
        #define DEBUG_MIC_BUF_UP(w,x,y,z)    print_buf(w,x,y,z)
        #define DEBUG_MIC_UP(...)      vt.SetCursorPos(ROW_MIC, 1);  vt.printf(__VA_ARGS__)
    #else
        #define DEBUG_MIC_BUF_UP(w,x,y,z)    print_buf(w,x,y,z)
        #define DEBUG_MIC_UP(...)        pc.printf(__VA_ARGS__)
    #endif
#else
    #define DEBUG_MIC_BUF_UP(w,x,y,z)    
    #define DEBUG_MIC_UP(...)
#endif

#ifdef DEBUG_MAC_CMD
    #define MACC_PRINTF(...)    pc.printf("\e[0m");
#else
    #define MACC_PRINTF(...)
#endif


uint8_t BoardGetBatteryLevel(void);
