/* upstream: interface towards gateway 36m cyan */
#include "main.h"

#ifndef GATEWAY
struct {
    int preference;
    uint32_t id;
    uint8_t hfg;
} best_upstream;

#endif /* !GATEWAY */


/* packets received from upstream */
void upstream_req_rxDoneCB(float rssi, float snr, uint8_t* Idx, uint32_t sendingID, uint8_t cmd)
{
    if (cmd == CMD_REMOVE_DEVICE_REQ) {
        uint32_t remove_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
        *Idx += 4;
        if (!remove_directlyAttached_device(remove_id)) {
            /* wasnt found locally, check child devices */
            uint32_t removedFrom;
            remove_childDevice(remove_id, &removedFrom);
            if (removedFrom != ID_NONE) {
                /* forward this remove down */
                downRemove.destID = removedFrom;
                downRemove.removeID = remove_id;
                reqFlags.bits.txAns = ANSWER_OK;
            } else
                reqFlags.bits.txAns = ANSWER_BUSY;
        } else
            reqFlags.bits.txAns = ANSWER_OK;

    } else if (cmd == CMD_DISCOVERY_ANS) {
#ifdef GATEWAY
        *Idx += 6;
#else
        int my_sq, peer_sq;
        uint8_t idxCpy = *Idx;
        uint8_t hfg;
        float signal_preference;
        int hops_preference, preference;
        /* everything is upstream when we arent associated on network */

        // signal quality is negative: rss + snr
        peer_sq = Radio::radio.rx_buf[idxCpy++] - 0x100;
        my_sq = rssi + snr;
        signal_preference = (peer_sq + my_sq) / 2;   // round-trip signal quality
        hfg = Radio::radio.rx_buf[idxCpy++];
        if (hfg != HFG_UNATTACHED) {
            hops_preference = 1 - hfg; // lower number preferable
            preference = signal_preference + (hops_preference * 10);
            if (preference > best_upstream.preference) {
                best_upstream.preference = preference;
                best_upstream.id = sendingID;
                best_upstream.hfg = hfg;
            }
        }

        *Idx += 2;
#endif /* !GATEWAY */
    } else if (cmd == CMD_USER_PAYLOAD_DN_REQ) {    // rxDone callback, pkt from upstream
        uint8_t len;
#ifndef GATEWAY
        uint32_t final_dest_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
#endif /* !GATEWAY */
        *Idx += 4;
        len = Radio::radio.rx_buf[*Idx];
        (*Idx)++;
#ifdef GATEWAY
        reqFlags.bits.txAns = ANSWER_OK;
#else
        if (final_dest_id == my_id) {
            app_downlink(len, &Radio::radio.rx_buf[*Idx]);
            reqFlags.bits.txAns = ANSWER_OK;
        } else {
            if (attUp.id == ID_NONE) {
                reqFlags.bits.txAns = ANSWER_UNATTACHED;
            } else {
                if (fwd.len == -1) {
                    fwd.len = len;
                    fwd.A_id = final_dest_id;
                    fwd.tx_dest_id = find_dest_id(final_dest_id);
                    memcpy(fwd.buf, &Radio::radio.rx_buf[*Idx], len);
                    reqFlags.bits.currentOp = CMD_USER_PAYLOAD_DN_REQ;   // this is downlink
                    reqFlags.bits.txAns = ANSWER_OK;
                } else 
                    reqFlags.bits.txAns = ANSWER_BUSY;
            }
        }
#endif /* !GATEWAY */
        (*Idx) += len;
    }

} // ..upstream_req_rxDoneCB()

#ifndef GATEWAY

upstream_t attUp;
upstream_t prospect;
uint32_t id_newDeviceNotification;

void upstream_ans_rxDoneCB(float rssi, float snr, uint8_t* Idx, uint32_t sendingID, uint8_t cmd)
{
    if (reqFlags.bits.currentOp == CMD_ATTACH_REQ) {
        if (sendingID == best_upstream.id) {
            attUp.id = best_upstream.id;
            attUp.preference = best_upstream.preference;
            attUp.hfg = best_upstream.hfg;
            hops_from_gateway = best_upstream.hfg + 1;
            prospect.id = ID_NONE;
        }

        reqFlags.bits.currentOp = CMD_UNUSED;
    } else if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ) {
        app_uplink_complete();
        reqFlags.bits.currentOp = CMD_UNUSED;
    } else if (reqFlags.bits.currentOp == CMD_NEW_DEVICE_ATTACHED_REQ) {
        id_newDeviceNotification = ID_NONE;
        reqFlags.bits.currentOp = CMD_UNUSED;
    }

} // ..upstream_ans_rxDoneCB()

void upstream_print_status()
{
    pc.printf("attUp.id:%lx ", attUp.id);
    if (hops_from_gateway == HFG_UNATTACHED) {
        pc.printf("best_upstream %lx, %d  hfg:%u\r\n", best_upstream.id, best_upstream.preference, best_upstream.hfg);
    }
}

void upstream_new_device_notify()   /* called from txDone */
{
    txBuf[txBuf_idx++] = CMD_NEW_DEVICE_ATTACHED_REQ;
    putu32ToBuf(&txBuf[txBuf_idx], id_newDeviceNotification);
    txBuf_idx += 4;
    tx_dest_id = attUp.id;
    queue.call_in(500, txBuf_send, true);
    reqFlags.bits.currentOp = CMD_NEW_DEVICE_ATTACHED_REQ;
    //mdbg_printf("newDeviceNotify_tx ");
}

void discovery_rx_end()
{
    Radio::Sleep();
    //mdbg_printf("discoveryAnsRxEnd ");

    if (best_upstream.preference == INT_MIN) {
        /* nothing received, try discoverying again */
        queue.call_in(1000, txBuf_send, true);
        pc.printf("discoveredNothing\r\n");
    } else {
        /* best gateway heard, attach to it */
        txBuf_idx = 0; // squash discovery request, no longer needed
        txBuf[txBuf_idx++] = CMD_ATTACH_REQ;
        tx_dest_id = best_upstream.id;
        queue.call_in(500, txBuf_send, true);
        pc.printf("attach-to-%lx\r\n", best_upstream.id);
        reqFlags.bits.currentOp = CMD_ATTACH_REQ;
    }
} // ..discovery_rx_end()

void upstream_init()
{
    pc.printf("txDiscoverReq:%u ", txBuf_idx);
    txBuf[txBuf_idx++] = CMD_DISCOVERY_REQ;
    tx_dest_id = ANY_ID;
    txBuf_send(true);

    best_upstream.preference = INT_MIN;
    init_attached_upstream();
    hops_from_gateway = HFG_UNATTACHED;
    reqFlags.bits.currentOp = CMD_DISCOVERY_REQ;
}

int uplink(const uint8_t* userPayload, uint8_t userPayloadSize)
{
    unsigned n;
    //mdbg_printf("uplink_tx%u ", userPayloadSize);
    if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ) {
        //mdbg_printf("busy\r\n");
        return -1;
    }

    reqFlags.bits.currentOp = CMD_USER_PAYLOAD_UP_REQ;

    txBuf[txBuf_idx++] = CMD_USER_PAYLOAD_UP_REQ;

    // originating ID, to be preserved all the way up to gateway
    putu32ToBuf(&txBuf[txBuf_idx], my_id);
    txBuf_idx += 4;

    txBuf[txBuf_idx++] = userPayloadSize;
    for (n = 0; n < userPayloadSize; n++)
        txBuf[txBuf_idx++] = userPayload[n];

    tx_dest_id = attUp.id;
    flags.CallTXRequest = 1;    // transmit will occur in idle-rxsingle cycle

    return 0;
}

void upstream_attached_check(uint32_t sending_id)
{
    if (sending_id != attUp.id)
        return;

    cmd_e cmd = (cmd_e)Radio::radio.rx_buf[9];
    Mdbg_printf("upAttCmd:%s? ", cmdStrs[cmd]);
    if (cmd == CMD_ATTACH_REQ) {
        pc.printf("\e[31muppreattach\e[0m ");
        /* TODO appropriate action when upstream path changes */
    }
}

void init_attached_upstream()
{
    attUp.id = ID_NONE;
    attUp.cnt = 0;
    attUp.preference = INT_MIN;

    prospect.id = ID_NONE;
    prospect.cnt = 0;
    prospect.preference = INT_MIN;
}

void upstream_signal_check(float rssi, float snr, uint8_t rx_hfg, uint32_t rx_id)
{
    float signal_preference;
    int this_preference, my_sq, hops_preference = 1 - rx_hfg;
    my_sq = rssi + snr;
    signal_preference = my_sq;
    this_preference = signal_preference + (hops_preference * 10);
    if (rx_id == attUp.id) {
        /* how much has path to upstream changed ? */
        Rx_log_printf("attachedChange initial:%d vs this:%d ", attUp.preference, this_preference);
        if (attUp.preference > this_preference) {
            attUp.cnt++;
        } else
            attUp.cnt = 0;
    } else {
        /* check for better upstream */
        Rx_log_printf("attached:%d vs this:%d ", attUp.preference, this_preference);
        if ((this_preference-10) > attUp.preference) {  // -10: significantly better
            Rx_log_printf("better");
            if (prospect.id == ID_NONE) {
                prospect.cnt = 1;
                prospect.id = rx_id;
                Rx_log_printf("Init");
            } else {
                prospect.cnt++;
                Rx_log_printf("Inc%u", prospect.cnt);
                if (prospect.cnt > 3) {
                    init_attached_upstream();
                    hops_from_gateway = HFG_UNATTACHED;
                    queue.call_in(2000, upstream_init);
                    Rx_log_printf("\e[33mreattach\e[0m ");
                }
            }
        } else if (this_preference < attUp.preference) {
            Rx_log_printf("worse");
            if (prospect.id == rx_id) {
                prospect.id = ID_NONE;
                prospect.cnt = 0;
                Rx_log_printf("Clear");
            }
        }

        Rx_log_printf(" ");
    } // ..rx from other-than-attached
}

#endif /* !GATEWAY */
