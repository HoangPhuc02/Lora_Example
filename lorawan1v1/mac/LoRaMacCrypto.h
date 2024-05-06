#include <stdint.h>

#define LORA_EUI_LENGTH        8
#define UP_LINK                0
#define DOWN_LINK              1

typedef struct {
    uint8_t AppSKey[16];
    uint8_t FNwkSIntKey[16];
    uint8_t SNwkSIntKey[16];
    uint8_t NwkSEncKey[16];
} skey_t;

typedef union {
    struct {
        uint8_t header;
        uint16_t confFCnt;
        uint8_t dr;
        uint8_t ch;
        uint8_t dir;
        uint32_t DevAddr;
        uint32_t FCnt;
        uint8_t zero8;
        uint8_t lenMsg;
    } __attribute__((packed)) b;
    uint8_t octets[16];
} block_t;

void LoRaMacEncrypt( uint8_t ctr, const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer );
//void LoRaMacComputeMic( bool OptNeg, const uint8_t *buffer, uint16_t size, skey_t* kptr, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint16_t ConfFCnt, uint8_t dr, uint8_t ch, uint32_t *mic );
void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer );
uint32_t LoRaMacComputeMic( const block_t* block, const uint8_t* pktPayload, const uint8_t* key);
#ifdef LORAWAN_JOIN_EUI
void LoRaMacGenerateJoinKey(uint8_t token, const uint8_t* root_key, const uint8_t* devEui, uint8_t* output);
int LoRaMacJoinComputeMic(bool verbose, const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic );
void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer );
void LoRaMacJoinComputeSKeys_1v0( const uint8_t *root_key, const uint8_t *ja_rx, uint16_t devNonce, skey_t* keys);
void LoRaMacJoinComputeSKeys_1v1( const uint8_t *nwk_root_key, const uint8_t *app_root_key, const uint8_t *joinNonce, const uint8_t *joinEUI, uint16_t devNonce, skey_t* keys);
#endif /* LORAWAN_JOIN_EUI */
void LoRaMacCryptoInit(void);
