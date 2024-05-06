#include "target.h"
#include <string.h>
#include "nrf52.h"

int targetCheckLSE()
{
    /* does NRF52 have low-speed RC clock? */
    return 0;
}

void HardwareIDtoDevEUI(uint8_t* DevEui)
{
    //uint64_t result = *((uint64_t*) NRF_FICR->DEVICEADDR);
    memcpy(DevEui, NRF_FICR->DEVICEADDR, 8);
}
