/* */

#include <stdlib.h>
#include <stdint.h>
#include "utilities.h"

#include "cmac.h"
#include "aes.h"

#include "LoRaMacCrypto.h"

/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE                   16

/*!
 * MIC field computation initial data
 */
static uint8_t MicBlockB0[] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                              };

/*!
 * Contains the computed MIC field.
 *
 * \remark Only the 4 first bytes are used
 */
static uint8_t Mic[16];

/*!
 * Encryption aBlock and sBlock
 */
static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };

static mbedtls_cipher_context_t ctx;

/*!
 * \brief Computes the LoRaMAC frame MIC field  
 *
 * \param [IN]  buffer          Data buffer
 * \param [IN]  size            Data buffer size
 * \param [IN]  key             AES key to be used
 * \param [IN]  address         Frame address
 * \param [IN]  dir             Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter Frame sequence counter
 * \param [OUT] mic Computed MIC field
 */
int LoRaMacComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic )
{
    //tls_printf("LoRaMacComputeMic:\r\n");
    MicBlockB0[5] = dir;
    
    MicBlockB0[6] = ( address ) & 0xFF;
    MicBlockB0[7] = ( address >> 8 ) & 0xFF;
    MicBlockB0[8] = ( address >> 16 ) & 0xFF;
    MicBlockB0[9] = ( address >> 24 ) & 0xFF;

    MicBlockB0[10] = ( sequenceCounter ) & 0xFF;
    MicBlockB0[11] = ( sequenceCounter >> 8 ) & 0xFF;
    MicBlockB0[12] = ( sequenceCounter >> 16 ) & 0xFF;
    MicBlockB0[13] = ( sequenceCounter >> 24 ) & 0xFF;

    MicBlockB0[15] = size & 0xFF;

    if (mbedtls_cipher_cmac_starts(&ctx, key, 128))
        return -1;
 
    mbedtls_cipher_cmac_update(&ctx, MicBlockB0, LORAMAC_MIC_BLOCK_B0_SIZE);
 
    mbedtls_cipher_cmac_update(&ctx, buffer, size & 0xff);
 
    mbedtls_cipher_cmac_finish(&ctx, Mic);
    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
    return 0;
}

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
{
    mbedtls_aes_context actx;
    uint16_t i;
    uint8_t bufferIndex = 0;
    uint16_t ctr = 1;

    mbedtls_aes_init(&actx);
    mbedtls_aes_setkey_enc(&actx, key, 128);

    aBlock[5] = dir;

    aBlock[6] = ( address ) & 0xFF;
    aBlock[7] = ( address >> 8 ) & 0xFF;
    aBlock[8] = ( address >> 16 ) & 0xFF;
    aBlock[9] = ( address >> 24 ) & 0xFF;

    aBlock[10] = ( sequenceCounter ) & 0xFF;
    aBlock[11] = ( sequenceCounter >> 8 ) & 0xFF;
    aBlock[12] = ( sequenceCounter >> 16 ) & 0xFF;
    aBlock[13] = ( sequenceCounter >> 24 ) & 0xFF;

    while( size >= 16 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        ctr++;
        mbedtls_aes_encrypt(&actx, aBlock, sBlock);
        for( i = 0; i < 16; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if (size > 0)
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        mbedtls_aes_encrypt(&actx, aBlock, sBlock);
        for( i = 0; i < size; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }

    mbedtls_aes_free(&actx);
}

void LoRaMacPayloadDecrypt(
        const uint8_t *buffer,
        uint16_t size,
        const uint8_t *key,
        uint32_t address,
        uint8_t dir,
        uint32_t sequenceCounter,
        uint8_t *decBuffer )
{
    LoRaMacPayloadEncrypt( buffer, size, key, address, dir, sequenceCounter, decBuffer );
}

int LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic )
{
    int ret;
    uint8_t Mic[16];
    ret = mbedtls_cipher_cmac_starts(&ctx, key, 128);
    if (ret < 0)
        return ret;
    ret = mbedtls_cipher_cmac_update(&ctx, buffer, size & 0xff);
    if (ret < 0)
        return ret;
    ret = mbedtls_cipher_cmac_finish(&ctx, Mic);
    if (ret < 0)
        return ret;
    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
    return 0; 
}

int LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer )
{
    int ret;
    mbedtls_aes_context actx;

    mbedtls_aes_init(&actx);
    ret = mbedtls_aes_setkey_enc(&actx, key, 128);
    if (ret < 0)
        return -1;

    mbedtls_aes_encrypt(&actx, buffer, decBuffer);

    if (size >= 16) {
        mbedtls_aes_encrypt(&actx, buffer + 16, decBuffer + 16);
    }

    mbedtls_aes_free(&actx);

    return 0;
}

int LoRaMacJoinComputeSKeys( const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey )
{
    mbedtls_aes_context actx;
    uint8_t nonce[16];
    uint8_t *pDevNonce = ( uint8_t * )&devNonce;

    mbedtls_aes_init(&actx);
    if (mbedtls_aes_setkey_enc(&actx, key, 128) < 0)
        return -1;

    memset1( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x01;
    memcpy1( nonce + 1, appNonce, 6 );
    memcpy1( nonce + 7, pDevNonce, 2 );
    mbedtls_aes_encrypt(&actx, nonce, nwkSKey);

    memset1( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x02;
    memcpy1( nonce + 1, appNonce, 6 );
    memcpy1( nonce + 7, pDevNonce, 2 );
    mbedtls_aes_encrypt(&actx, nonce, appSKey);

    mbedtls_aes_free(&actx);

    return 0;
}

int LoRaMacCryptoInit()
{
    int ret;
    const mbedtls_cipher_info_t *cipher_info;
    mbedtls_cipher_init(&ctx);
    cipher_info = mbedtls_cipher_info_from_type( MBEDTLS_CIPHER_AES_128_ECB );
    if (cipher_info == NULL) {
        return -1;
    }
    ret = mbedtls_cipher_setup(&ctx, cipher_info);
    return ret;
}

