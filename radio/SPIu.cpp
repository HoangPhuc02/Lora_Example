#include "mbed.h"
#include "SPIu.h"

SPIu::SPIu(PinName mosi, PinName miso, PinName sclk, PinName ssel) : SPI(mosi, miso, sclk, ssel)
{
}

void SPIu::lock() {
}

void SPIu::unlock() {
}

