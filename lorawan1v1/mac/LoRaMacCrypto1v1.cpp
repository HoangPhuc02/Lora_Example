#include "lorawan_board.h"
#include "LoRaMacCrypto.h"
#include "cmac.h"
#include "aes.h"

#define LORAMAC_MIC_BLOCK_B0_SIZE                   16
mbedtls_cipher_context_t ctx;

uint32_t LoRaMacComputeMic(
    const block_t* block,
    const uint8_t* pktPayload,
    const uint8_t* key)
{
    uint8_t Mic[16];

    if (block->b.dir == DOWN_LINK) {
        DEBUG_MIC_DOWN("down ");
        DEBUG_MIC_BUF_DOWN(block->octets, LORAMAC_MIC_BLOCK_B0_SIZE, "b0", ROW_MIC+1);
        DEBUG_MIC_BUF_DOWN(key, 16, "b0-key", ROW_MIC+2);
    } else if (block->b.dir == UP_LINK) {
        DEBUG_MIC_UP("  up ");
        DEBUG_MIC_BUF_UP(block->octets, LORAMAC_MIC_BLOCK_B0_SIZE, "b0", ROW_MIC+1);
        DEBUG_MIC_BUF_UP(key, 16, "b0-key", ROW_MIC+2);
    }

    mbedtls_cipher_cmac_starts(&ctx, key, 128);

    mbedtls_cipher_cmac_update(&ctx, block->octets, LORAMAC_MIC_BLOCK_B0_SIZE);

    mbedtls_cipher_cmac_update(&ctx, pktPayload, block->b.lenMsg);

    mbedtls_cipher_cmac_finish(&ctx, Mic);

    return ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
}

void LoRaMacEncrypt( uint8_t ctr, const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
{
    uint8_t aBlock[16];
    uint8_t sBlock[16];
    uint16_t i;
    uint8_t bufferIndex = 0;
    mbedtls_aes_context actx;

    //memset( AesContext.ksch, '\0', 240 );
    mbedtls_aes_init(&actx);
    //aes_set_key( key, 16, &AesContext );
    mbedtls_aes_setkey_enc(&actx, key, 128);

    aBlock[0] = 0x01;

    aBlock[1] = 0x00;
    aBlock[2] = 0x00;
    aBlock[3] = 0x00;
    aBlock[4] = 0x00;

    aBlock[5] = dir;

    aBlock[6] = ( address ) & 0xFF;
    aBlock[7] = ( address >> 8 ) & 0xFF;
    aBlock[8] = ( address >> 16 ) & 0xFF;
    aBlock[9] = ( address >> 24 ) & 0xFF;

    aBlock[10] = ( sequenceCounter ) & 0xFF;
    aBlock[11] = ( sequenceCounter >> 8 ) & 0xFF;
    aBlock[12] = ( sequenceCounter >> 16 ) & 0xFF;
    aBlock[13] = ( sequenceCounter >> 24 ) & 0xFF;

    aBlock[14] = 0;

    while( size >= 16 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        ctr++;
        //aes_encrypt( aBlock, sBlock, &AesContext );
        mbedtls_aes_encrypt(&actx, aBlock, sBlock);
        for( i = 0; i < 16; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if( size > 0 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        //aes_encrypt( aBlock, sBlock, &AesContext );
        mbedtls_aes_encrypt(&actx, aBlock, sBlock);
        for( i = 0; i < size; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }

    mbedtls_aes_free(&actx);
}

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer )
{
    LoRaMacEncrypt(1, buffer, size, key, address, dir, sequenceCounter, decBuffer );
}

void
LoRaMacCryptoInit()
{
    //int ret;
    const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type( MBEDTLS_CIPHER_AES_128_ECB );
    if (cipher_info == NULL) {
        MAC_PRINTF("mbedtls_cipher_info_from_type() failed\n");
        return;
    }
    mbedtls_cipher_init(&ctx);
    /*ret = */mbedtls_cipher_setup(&ctx, cipher_info);
    //MAC_PRINTF("mbedtls_cipher_setup returned %d, type %d\r\n", ret, ctx.cipher_info->type);
    /* always using same ctx --- mbedtls_cipher_free(&ctx) */
}

#ifdef LORAWAN_JOIN_EUI
void LoRaMacJoinComputeSKeys_1v1( const uint8_t *nwk_root_key, const uint8_t *app_root_key, const uint8_t *joinNonce, const uint8_t *joinEUI, uint16_t devNonce, skey_t* keys)
{
    uint8_t buff[16];
    uint8_t *pDevNonce = ( uint8_t * )&devNonce;
    uint8_t* bufPtr;
    mbedtls_aes_context actx;

    mbedtls_aes_init(&actx);
    //memset( AesContext.ksch, '\0', 240 );
    DEBUG_CRYPT_BUF(app_root_key, 16, "AppSKey-root", 0);
    //aes_set_key( app_root_key, 16, &AesContext );
    mbedtls_aes_setkey_enc(&actx, app_root_key, 128);

    memset(buff, 0, sizeof(buff));
    bufPtr = buff + 1;
    memcpy(bufPtr, joinNonce, 3);
    bufPtr += 3;
    memcpyr(bufPtr, joinEUI, 8);
    bufPtr += 8;
    memcpy(bufPtr, pDevNonce, 2);
    bufPtr += 2;

    /* generate AppSKey */
    buff[0] = 0x02;
    DEBUG_CRYPT_BUF(buff, 16, "AppSKey-in", 0);
    //aes_encrypt(buff, keys->AppSKey, &AesContext );
    mbedtls_aes_encrypt(&actx, buff, keys->AppSKey);
    DEBUG_CRYPT_BUF(keys->AppSKey, 16, "AppSKey", 0);

    //memset( AesContext.ksch, '\0', 240 );
    //aes_set_key( nwk_root_key, 16, &AesContext );
    mbedtls_aes_setkey_enc(&actx, nwk_root_key, 128);

    /* generate FNwkSIntKey */
    buff[0] = 0x01;
    DEBUG_CRYPT_BUF(buff, sizeof(buff), "in-FNwkSIntKey", 0);
    //aes_encrypt(buff, keys->FNwkSIntKey, &AesContext );
    mbedtls_aes_encrypt(&actx, buff, keys->FNwkSIntKey);
    DEBUG_CRYPT_BUF(keys->FNwkSIntKey, 16, "FNwkSIntKey", 0);

    /* generate SNwkSIntKey */
    buff[0] = 0x03;
    //aes_encrypt(buff, keys->SNwkSIntKey, &AesContext );
    mbedtls_aes_encrypt(&actx, buff, keys->SNwkSIntKey);
    DEBUG_CRYPT_BUF(keys->SNwkSIntKey, 16, "SNwkSIntKey", 0);

    /* generate NwkSEncKey */
    buff[0] = 0x04;
    //aes_encrypt(buff, keys->NwkSEncKey, &AesContext );
    mbedtls_aes_encrypt(&actx, buff, keys->NwkSEncKey);
    DEBUG_CRYPT_BUF(keys->NwkSEncKey, 16, "NwkSEncKey", 0);

    mbedtls_aes_free(&actx);
}

void LoRaMacJoinComputeSKeys_1v0(const uint8_t *nwk_root_key, const uint8_t *ja_rx, uint16_t devNonce, skey_t* keys)
{
    /* ja_rx: joinNonce + NetID, 6bytes */
    uint8_t nonce[16];
    uint8_t *pDevNonce = ( uint8_t * )&devNonce;
    mbedtls_aes_context actx;

    mbedtls_aes_init(&actx);
    //memset( AesContext.ksch, '\0', 240 );
    //aes_set_key( nwk_root_key, 16, &AesContext );
    mbedtls_aes_setkey_enc(&actx, nwk_root_key, 128);

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x01;
    memcpy( nonce + 1, ja_rx, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    //aes_encrypt( nonce, keys->FNwkSIntKey, &AesContext );
    mbedtls_aes_encrypt(&actx, nonce, keys->FNwkSIntKey);

    memcpy(keys->SNwkSIntKey, keys->FNwkSIntKey, 16);
    memcpy(keys->NwkSEncKey, keys->FNwkSIntKey, 16);

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x02;
    memcpy( nonce + 1, ja_rx, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    //aes_encrypt( nonce, keys->AppSKey, &AesContext );
    mbedtls_aes_encrypt(&actx, nonce, keys->AppSKey);

    mbedtls_aes_free(&actx);
}

int LoRaMacJoinComputeMic(bool verbose, const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic )
{
    int ret;
    uint8_t Mic[LORAMAC_MIC_BLOCK_B0_SIZE];

#ifndef ENABLE_VT100
    if (verbose) {
        print_buf(key, 16, "mic-key");
        print_buf(buffer, size, "mic-in-buf");
    }
#endif /* ENABLE_VT100 */

    ret = mbedtls_cipher_cmac_starts(&ctx, key, 128);
    if (ret != 0) {
        MAC_PRINTF("0x%x = mbedtls_cipher_cmac_starts()\r\n", ret);
        return ret;
    }

    ret = mbedtls_cipher_cmac_update(&ctx, buffer, size & 0xff);
    if (ret != 0) {
        MAC_PRINTF("%d = mbedtls_cipher_cmac_update()\r\n", ret);
        return ret;
    }

    ret = mbedtls_cipher_cmac_finish(&ctx, Mic);
    if (ret != 0) {
        MAC_PRINTF("%d = mbedtls_cipher_cmac_finish()\r\n", ret);
        return ret;
    }

    *mic = ( uint32_t )( ( uint32_t )Mic[3] << 24 | ( uint32_t )Mic[2] << 16 | ( uint32_t )Mic[1] << 8 | ( uint32_t )Mic[0] );
    return 0;
}

void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer )
{
    mbedtls_aes_context actx;
    mbedtls_aes_init(&actx);
    if (mbedtls_aes_setkey_enc(&actx, key, 128) < 0) {
        MAC_PRINTF("%d = mbedtls_aes_setkey_enc()\r\n");
    }
    mbedtls_aes_encrypt(&actx, buffer, decBuffer);
    // Check if optional CFList is included
    if (size >= 16)
    {
        mbedtls_aes_encrypt(&actx, buffer + 16, decBuffer + 16);
    }

    mbedtls_aes_free(&actx);
}

void LoRaMacGenerateJoinKey(uint8_t token, const uint8_t* root_key, const uint8_t* devEui, uint8_t* output)
{
    int i;
    uint8_t input[16];
    uint8_t* ptr = input;
    mbedtls_aes_context actx;

    mbedtls_aes_init(&actx);

    memset(ptr, 0, sizeof(input));

    *ptr++ = token;

    /* EUI put into buffer in same order that it appears over-the-air */
    for (i = LORA_EUI_LENGTH - 1; i >= 0; i--)
        *ptr++ = devEui[i];

    DEBUG_CRYPT_BUF(root_key, 16, "generate-join-key-root_key", 0);
    DEBUG_CRYPT_BUF(input, 16, "generate-join-key-input", 0);

    mbedtls_aes_setkey_enc(&actx, root_key, 128);
    mbedtls_aes_encrypt(&actx, input, output);

    mbedtls_aes_free(&actx);
}
#endif /* LORAWAN_JOIN_EUI */
