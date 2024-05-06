#include <stdint.h>

#if __cplusplus
extern "C" {
#endif
void HardwareIDtoDevEUI(uint8_t* DevEui);
int targetCheckLSE(void);
#if __cplusplus
};
#endif

