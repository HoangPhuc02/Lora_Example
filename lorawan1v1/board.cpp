#include "lorawan_board.h"

#ifndef ENABLE_VT100
RawSerial pc(USBTX, USBRX);
#endif /* ENABLE_VT100 */

uint8_t BoardGetBatteryLevel()
{
    return 0;
}

