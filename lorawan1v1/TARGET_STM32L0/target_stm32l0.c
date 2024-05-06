#include "target.h"
#include <string.h>

#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_rcc.h"

int targetCheckLSE()
{
    if (LL_RCC_LSE_IsReady())
        return 0;
    else
        return -1;
    /* LSI accuracy is inadequate for LoRaWAN */
}

void HardwareIDtoDevEUI(uint8_t* DevEui)
{
    uint32_t in;

    in = LL_GetUID_Word0();
    memcpy(DevEui, &in, sizeof(uint32_t));
    in = LL_GetUID_Word1() ^ LL_GetUID_Word2();
    memcpy(&DevEui[4], &in, sizeof(uint32_t));
}
