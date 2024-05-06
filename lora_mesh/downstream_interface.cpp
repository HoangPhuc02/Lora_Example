
///* downstream: interface away from gateway 35m = magenta */;
#include "main.h"

lid_list_t* attachedDevices;
struct remove downRemove;

void request_remove_device()
{
    txBuf[txBuf_idx++] = CMD_REMOVE_DEVICE_REQ;
    putu32ToBuf(&txBuf[txBuf_idx], downRemove.removeID);
    txBuf_idx += 4;
    tx_dest_id = downRemove.destID;
    queue.call_in(500, txBuf_send, true);
    reqFlags.bits.currentOp = CMD_REMOVE_DEVICE_REQ;
}

/* return true: device is new */
static bool list_add_local_device(uint32_t newID)
{
    Rx_log_printf("newID:%lx ", newID);
    if (attachedDevices == NULL) {
        attachedDevices = (lid_list_t*)malloc(sizeof(lid_list_t));
        attachedDevices->id = newID;
        attachedDevices->attachedList = NULL;
        attachedDevices->next = NULL;
        return true;
    } else {
        lid_list_t *L, **mallocTarget;
        /* first check if this device was previously attached to a device attached to us */
        for (L = attachedDevices; L != NULL; L = L->next) {
            cid_list_t* children;
            for (children = L->attachedList;  children != NULL; children = children->next) {
                Rx_log_printf("checkChild:%lx_onLocal:%lx ", children->id, L->id);
                if (children->id == newID) {
                    Rx_log_printf("CLEARing_%lx_from_%lx ", children->id, L->id);
                    children->id = ID_NONE;
                }
            }
        }

        L = attachedDevices;
        mallocTarget = &attachedDevices->next;
        do {
            if (L->id == newID) {
                Rx_log_printf("alreadyHave:%lx ", L->id);
                //return false;
                return true; /* return true: notify upstream of moved device */
            } else if (L->id == ID_NONE) {
                Rx_log_printf("replaceCleared ");
                L->id = newID;
                return true; /* return true: notify upstream of moved device */
            }
            mallocTarget = &L->next;
            L = L->next;
        } while (L != NULL);
        *mallocTarget = (lid_list_t*)malloc(sizeof(lid_list_t));
        L = *mallocTarget;
        L->id = newID;
        L->attachedList = NULL;
        L->next = NULL;
        Rx_log_printf("addedToList:%lx ", L->id);
        return true;
    }
}

static void add_new_downstream_attached(uint32_t attachedTo, uint32_t newID)
{
    lid_list_t* L;
    /* first find if this device already exists anywhere, and remove/clear it */
    for (L = attachedDevices; L != NULL; L = L->next) {
        cid_list_t* children;
        if (L->id == newID) {
            L->id = ID_NONE;
        }
        if (L->attachedList == NULL)
            continue;
        for (children = L->attachedList;  children != NULL; children = children->next) {
            if (children->id == newID) {
#ifdef GATEWAY
                if (L->id != attachedTo) {
                    /* notify downstream */
                    downRemove.destID = L->id;
                    downRemove.removeID = newID;
                }
#endif /* GATEWAY */
                children->id = ID_NONE;
            }
        }
    }

    /* add */
    for (L = attachedDevices; L != NULL; L = L->next) {
        if (L->id == attachedTo) {
            cid_list_t* child;
            if (L->attachedList == NULL) { // first added child
                L->attachedList = (cid_list_t*)malloc(sizeof(cid_list_t));
                L->attachedList->id = newID;
                L->attachedList->next = NULL;
                return;
            }
            /* first check for vacated slot */
            for (child = L->attachedList; child != NULL; child = child->next) {
                if (child->id == ID_NONE) {
                    child->id = newID;
                    return;
                } else if (child->next == NULL)
                    break;  // next pointer available to malloc
            }
            child->next = (cid_list_t*)malloc(sizeof(cid_list_t));
            child = child->next;
            child->id = newID;
            child->next = NULL;
            return;
        }
    }
} // ..add_new_downstream_attached()


void downstream_ans_rxDoneCB(float rssi, float snr, uint8_t* Idx, uint32_t sendingID, uint8_t cmd)
{
    if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_DN_REQ) {
        reqFlags.bits.currentOp = CMD_UNUSED;
    } else if (reqFlags.bits.currentOp == CMD_REMOVE_DEVICE_REQ) {
        downRemove.destID = ID_NONE;    // removal request complete
        reqFlags.bits.currentOp = CMD_UNUSED;
    } else if (reqFlags.bits.currentOp == CMD_DOWNSTREAM_NOT_RESPONDING) {
        reqFlags.bits.currentOp = CMD_UNUSED;
    }
}

void discovery_tx_end()
{
    flags.discoverAnswering = 0;
    start_periodic_rxing(0x90);
}

#ifdef GATEWAY
void downstream_req_rxDoneCB(float rssi, float snr, uint8_t* Idx, uint32_t sendingID, uint8_t cmd, upInfo_t* up)
#else
void downstream_req_rxDoneCB(float rssi, float snr, uint8_t* Idx, uint32_t sendingID, uint8_t cmd)
#endif
{

    if (cmd == CMD_DISCOVERY_REQ) {
        unsigned toms, rnd;
        int8_t sq;

        Radio::Standby();
        reqFlags.bits.currentOp = CMD_DISCOVERY_ANS;
        
        txBuf[txBuf_idx++] = CMD_DISCOVERY_ANS;
        sq = rssi + snr;
        txBuf[txBuf_idx++] = sq;
        txBuf[txBuf_idx++] = hops_from_gateway;

        /* schedule transmit to occur randomly, twice */
        flags.firstDiscoverAns = 1;
        flags.discoverAnswering = 1;
        tx_dest_id = sendingID;     // discovery req -> ans
        setPreambleSize(false, 1); // sending discovery answer
        rnd = Radio::Random() % N_HALF_DISCOVERY_ANS;
        queue.call_in((discovery_ans_time_step_us * rnd) / 1000, txBuf_send, false);
        toms = discovery_ans_time_total_us / 1000;
        queue.call_in(toms, discovery_tx_end);
    } else if (cmd == CMD_ATTACH_REQ) {
        Radio::Standby();
        reqFlags.bits.txAns = ANSWER_OK;

#ifndef GATEWAY
        if (sendingID == attUp.id) {
            /* upstream device restarted or re-connected  */
            init_attached_upstream();
            hops_from_gateway = HFG_UNATTACHED;
        }
#endif /* GATEWAY */

        /* add sending_ID to list of attached devices */
        if (list_add_local_device(sendingID)) {
            /* device is new: if hfg > 0: notify upstream of new device upon txdone of CMD_ATTACH_ANS sent downstream */
#ifndef GATEWAY
            id_newDeviceNotification = sendingID;
#endif /* GATEWAY */
        }

    } else if (cmd == CMD_NEW_DEVICE_ATTACHED_REQ) { // rxDone callback
        uint32_t new_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
        *Idx += 4;
        /* new device not attached directly to me, but to a downstream device attached to me */
        add_new_downstream_attached(sendingID, new_id);

        /* ack sent first downstream, then (if we;re not a gateway) newDeviceNotification sent upstream */

#ifdef GATEWAY
        reqFlags.bits.txAns = ANSWER_OK;
#else
        if (id_newDeviceNotification == ID_NONE) {
            reqFlags.bits.txAns = ANSWER_OK;
            id_newDeviceNotification = new_id;
        } else
            reqFlags.bits.txAns = ANSWER_BUSY;
#endif /* !GATEWAY */
    } else if (cmd == CMD_USER_PAYLOAD_UP_REQ) { // rxDone callback, pkt from downstream
        uint8_t len;
        uint32_t originating_src_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
        *Idx += 4;
        len = Radio::radio.rx_buf[*Idx];
        (*Idx)++;
#ifdef GATEWAY
        //gateway_uplink(len, originating_src_id, &Radio::radio.rx_buf[*Idx]);
        up->rxBufIdx = *Idx;
        up->originating_src_id = originating_src_id;
        up->len = len;
        reqFlags.bits.txAns = ANSWER_OK;
#else
        if (fwd.len == -1) {
            fwd.len = len;
            fwd.B_id = originating_src_id;
            fwd.tx_dest_id = attUp.id;
            memcpy(fwd.buf, &Radio::radio.rx_buf[*Idx], len);
            reqFlags.bits.currentOp = CMD_USER_PAYLOAD_UP_REQ;   // this is uplink
            reqFlags.bits.txAns = ANSWER_OK;
            // txBuf_send() (for forwarding) will be called after ANS sent
        } else
            reqFlags.bits.txAns = ANSWER_BUSY;

#endif
        (*Idx) += len;
    } else if (cmd == CMD_DOWNSTREAM_NOT_RESPONDING) { // rxDone callback, pkt from downstream
#ifndef GATEWAY
        uint32_t reporting_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
#endif /* !reporting_id */
        *Idx += 4;
#ifndef GATEWAY
        uint32_t device_not_respoding_id = getu32FromBuf(&Radio::radio.rx_buf[*Idx]);
#endif /* !reporting_id */
        *Idx += 4;
#ifdef GATEWAY
        reqFlags.bits.txAns = ANSWER_OK;
#else
        if (notResponding.reporting_id == ID_NONE) {
            reqFlags.bits.currentOp = CMD_DOWNSTREAM_NOT_RESPONDING; 
            notResponding.reporting_id = reporting_id;
            notResponding.device_not_respoding_id = device_not_respoding_id;
            reqFlags.bits.txAns = ANSWER_OK;
        } else
            reqFlags.bits.txAns = ANSWER_BUSY;
#endif /* !GATEWAY */
    }

} // ..downstream_req_rxDoneCB()

