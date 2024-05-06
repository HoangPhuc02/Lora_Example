
#include "main.h"
#ifdef GATEWAY
#include "app.h"


void cmd_downlink(uint8_t argsAt)
{
    uint32_t requested_id;
    const char* strPtr;
    uint8_t n, lenIdx;

/*    if (flags.CallTXRequest) {
        pc.printf("busy\r\n");
        return;
    }*/

    if (sscanf(pcbuf+argsAt, "%lx", &requested_id) != 1) {
        pc.printf("provide destination ID\r\n");
        return;
    }

    tx_dest_id = find_dest_id(requested_id);
    if (tx_dest_id == ID_NONE) {
        pc.printf("id %lx not found\r\n", requested_id);
        return;
    }
    pc.printf("downlink to %lx ", tx_dest_id);

    reqFlags.bits.currentOp = CMD_USER_PAYLOAD_DN_REQ;
    txBuf[txBuf_idx++] = CMD_USER_PAYLOAD_DN_REQ;
    putu32ToBuf(&txBuf[txBuf_idx], requested_id);
    txBuf_idx += 4;
    lenIdx = txBuf_idx++;

    strPtr = pcbuf + argsAt;
    strPtr = strchr(++strPtr, ' ');
    n = 0;
    while (strPtr) {
        unsigned octet;
        sscanf(strPtr, "%x", &octet);
        txBuf[txBuf_idx++] = octet;
        strPtr = strchr(++strPtr, ' ');
        n++;
    }
    txBuf[lenIdx] = n;

    flags.CallTXRequest = 1;    // transmit will occur in idle-rxsingle cycle

    pc.printf("\r\n");
}

void gateway_uplink(uint8_t len, uint32_t orig_src, const uint8_t* payload)
{
    unsigned n;
    pc.printf("userPayload %u from src:%lx: ", len, orig_src);
    /* take here for parsing*/
    for (n = 0; n < len; n++) {
        pc.printf("%02x ", payload[n]);
    }
    pc.printf("\r\n");
}

void app_init()
{
}
#endif /* GATEWAY */
