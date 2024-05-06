#include "main.h"
#ifndef GATEWAY
#include "app.h"

InterruptIn but(USER_BUTTON);

/* downlink handler */
void app_downlink(uint8_t len, const uint8_t* payload)
{
    unsigned n;
    pc.printf("downlink %u: ", len);
    for (n = 0; n < len; n++)
        pc.printf("%02x ", payload[n]);

    pc.printf("\r\n");
}

void button_check()
{
    static uint8_t cnt = 0;

    if (but.read() == 0) {
        uint8_t buf[2];
        buf[0] = BUTTON_PRESS;
        buf[1] = cnt++;
        pc.printf("appUp ");
        if (uplink(buf, 2))
            pc.printf("upFail\r\n");
    } else {
        pc.printf("appBounce\r\n");
        but.enable_irq();
    }
}

void button_pressed()
{
    queue.call_in(10, button_check);
    but.disable_irq();
}

void app_uplink_complete()
{
    but.enable_irq();
}

void app_init()
{
    /* nucleo: user button active low, inactive hi */
    but.fall(button_pressed);
}
#endif /* !GATEWAY */
