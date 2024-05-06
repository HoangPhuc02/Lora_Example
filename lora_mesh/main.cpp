#include "main.h"

#ifdef TARGET_STM32L0
#include "stm32l0xx_ll_utils.h"
#elif defined(TARGET_STM32L1)
#include "stm32l1xx_ll_utils.h"
#elif defined(TARGET_STM32L4)
#include "stm32l4xx_ll_utils.h"
#endif

#ifdef TARGET_DISCO_L072CZ_LRWAN1
DigitalOut rxdbg(PB_8);
#elif defined(TARGET_FF_ARDUINO)
DigitalOut rxdbg(PC_3);
#endif

RawSerial pc(USBTX, USBRX); 

EventQueue queue(32 * EVENTS_EVENT_SIZE);

char pcbuf[64]; /* local user terminal */
int pcbuf_len;

unsigned discovery_ans_time_step_us;
unsigned discovery_ans_time_total_us;

uint32_t my_id;

#ifdef GATEWAY
const uint8_t hops_from_gateway = 0;
#else
uint8_t hops_from_gateway;
#endif

volatile flags_t flags;

static uint8_t attemptCnt;
static int req_timeout_id;
reqflags_t reqFlags;
volatile unsigned channelVacantCount;

uint8_t txBuf[255];
uint8_t txBuf_idx;
uint32_t tx_dest_id;

bool remove_directlyAttached_device(uint32_t id)
{
    bool removed = false;
    lid_list_t* L;
    for (L = attachedDevices; L != NULL; L = L->next) {
        if (L->id == id) {
            /* remove/clear all child devices also */
            cid_list_t* children;
            for (children = L->attachedList;  children != NULL; children = children->next)
                children->id = ID_NONE;

            removed = true;
            L->id = ID_NONE;
        }
    }
    return removed;
}

void remove_childDevice(uint32_t id, uint32_t* attachedTo)
{
    lid_list_t* L;
    *attachedTo = ID_NONE;
    for (L = attachedDevices; L != NULL; L = L->next) {
        cid_list_t* children;
        for (children = L->attachedList;  children != NULL; children = children->next) {
            if (children->id == id) {
                *attachedTo = L->id;
                children->id = ID_NONE;
            }
        }
    }
}

void setPreambleSize(bool wakesize, uint8_t by)
{
    if (wakesize) {
        Radio::LoRaPacketConfig(N_PRE_SYMB, false, true, false);  // preambleLen, fixLen, crcOn, invIQ
        if (flags.discoverAnswering) {
            Mdbg_printf("\e[41mLPDA%02x\e[0m ", by);
        }
    } else {
        Radio::LoRaPacketConfig(8, false, true, false);  // preambleLen, fixLen, crcOn, invIQ
    }
}

static void rxSingle()
{
    if (flags.CallTXRequest) {
        txBuf_send(true);
        flags.CallTXRequest = 0;
    } else if (reqFlags.octet == 0 || flags.deferred_send) {
        flags.vacantCheck = 1;
        /* rx single: auto-timeout */
        Radio::Rx(999);
    } 
}

void start_periodic_rxing(uint8_t by)   // definition
{
    setPreambleSize(true, by | 3);
    Radio::SetLoRaSymbolTimeout(8);
    queue.call_in(WAKEUP_INTERVAL_MS, rxSingle);
}

const char* const cmdStrs[] = {
    "unused", /*  0  CMD_UNUSED */
    "Ans", /*  1  CMD_ANS, */
    "discoverReq", /*  2  CMD_DISCOVERY_REQ, */
    "discoverAns", /*  3  CMD_DISCOVERY_ANS, */
    "attachReq", /*  4  CMD_ATTACH_REQ, */
    "userPayReqUp", /*  5  CMD_USER_PAYLOAD_UP_REQ, */
    "userPayReqDn", /*  6  CMD_USER_PAYLOAD_DN_REQ, */
    "newDev", /*  7  CMD_NEW_DEVICE_ATTACHED_REQ, */
    "removeDev", /*  8  CMD_REMOVE_DEVICE_REQ, */
    "downstreamNotResponding", /*  9  CMD_DOWNSTREAM_NOT_RESPONDING, */
};


uint8_t tx_len;

static void _send_(void)
{
    if (flags.sending_req)
        setPreambleSize(true, 2); // sending request

    Radio::Send(tx_len, 0, 0, 0);   /* begin transmission */
    if (flags.discoverAnswering) {
        Mdbg_printf("\e[36m%u->txing%u_to:%lx_%s\e[0m", txBuf_idx, tx_len, tx_dest_id, cmdStrs[Radio::radio.tx_buf[9]]);
    } else {
        Mdbg_printf("%u->\e[31mtxing\e[0m%u_to:%lx_\e[7m%s\e[0m", txBuf_idx, tx_len, tx_dest_id, cmdStrs[Radio::radio.tx_buf[9]]);
    }
    Mdbg_printf(":");
#ifdef MESH_DEBUG
    /*{
        unsigned n;
        for (n = 0; n < tx_len; n++)
            pc.printf("%02x ", Radio::radio.tx_buf[n]);
    }*/
#endif /* MESH_DEBUG */

    if (flags.sending_req) {
        attemptCnt++;
        flags.getAns = 1;
    } else
        flags.getAns = 0;

    channelVacantCount = 0;
} // .._send_()

void txBuf_send(bool sendingReq)
{
    uint16_t crc;

    if (txBuf_idx == 0) {
        return;
    }

    Radio::Standby();

    if (sendingReq) {
        if (attemptCnt > RETRY_LIMIT) {
            /* give up trying */
            txBuf_idx = 0;
            attemptCnt = 0;
#ifdef GATEWAY
            reqFlags.octet = 0; // TODO dropping request
            start_periodic_rxing(0x80);  // retry give-up
            return;
#else
            if (tx_dest_id == attUp.id) {
                /* find new upstream device */
                queue.call_in(1000, upstream_init);
                if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ)
                    app_uplink_complete();  // TODO report failure to application layer
                return;
            } else {
                txBuf[txBuf_idx++] = CMD_DOWNSTREAM_NOT_RESPONDING;
                putu32ToBuf(&txBuf[txBuf_idx], my_id);   // ID of reporting device
                txBuf_idx += 4;
                putu32ToBuf(&txBuf[txBuf_idx], tx_dest_id);   // ID of failed downstream device
                txBuf_idx += 4;
                tx_dest_id = attUp.id;
                reqFlags.bits.currentOp = CMD_DOWNSTREAM_NOT_RESPONDING; 
            }
#endif /* !GATEWAY */
        }
    } // ..if (sendingReq)

    Radio::radio.tx_buf[0] = hops_from_gateway;
    putu32ToBuf(&Radio::radio.tx_buf[1], my_id);
    putu32ToBuf(&Radio::radio.tx_buf[5], tx_dest_id);

    tx_len = txBuf_idx;
    memcpy(&Radio::radio.tx_buf[9], txBuf, tx_len);
    tx_len += 9;
    crc = crc16(Radio::radio.tx_buf, tx_len);
    putu16ToBuf(&Radio::radio.tx_buf[tx_len], crc);
    tx_len += 2;

    flags.sending_req = sendingReq;
    if (sendingReq) {
        if (channelVacantCount > CHANNEL_VACANT_REQUIRED_COUNT)
            _send_();
        else {
            flags.deferred_send = 1;
            start_periodic_rxing(0x70);  // deferred send
        }
    } else
        _send_();

} // ..txBuf_send()

#ifdef MESH_DEBUG
bool rx_log_disable;
char rx_log[768];
volatile unsigned rx_log_buf_idx;
int _rx_log_printf(const char *format, ...)
    {
    va_list aptr;
    int ret = -1;

    va_start(aptr, format);
    if (!rx_log_disable) {
        ret = vsprintf(rx_log + rx_log_buf_idx , format, aptr);
        rx_log_buf_idx += ret;
        if (rx_log_buf_idx >= sizeof(rx_log)-1) {
            pc.printf("\e[31mrx_log_overrun\e[0m ");
            rx_log_disable = true;
        }
    }
    va_end(aptr);


    return ret;
}

void rx_log_print()
{
    rx_log[rx_log_buf_idx+1] = 0;
    pc.printf(rx_log);
    rx_log_buf_idx = 0;
}
#endif /* MESH_DEBUG */

static void txAns(unsigned sending_id)
{
    unsigned elapsedAlready;
    int pad_us;
    txBuf_idx = 0; // previously sent request no longer needed
    txBuf[txBuf_idx++] = CMD_ANS;
    txBuf[txBuf_idx++] = reqFlags.bits.txAns;
    setPreambleSize(false, 6); // sending answer
    tx_dest_id = sending_id;
    elapsedAlready = Radio::lpt.read_us() - Radio::irqAt;
    pad_us = (ANS_PAD_MS * 1000) - elapsedAlready;
    if (pad_us > 100) {
        wait_us(pad_us);    // short wait time more accurate in busy-loop
    }
    txBuf_send(false);
    if (pad_us <= 0) {
        pc.printf("\e[41mLATE:%d ", pad_us);
        pc.printf("\e[0m ");
    }

#ifdef MESH_DEBUG
    // printing of rx_log was deferred until this answer sent
    rx_log_print();
#endif /* MESH_DEBUG */
}

uint16_t getu16FromBuf(const uint8_t* in)
{
    uint16_t ret;

    ret = in[1];
    ret <<= 8;
    ret |= in[0];

    return ret;
}

void putu16ToBuf(uint8_t* out, uint16_t v)
{
    *out++ = v & 0xff;
    v >>= 8;
    *out = v & 0xff;
}

void putu32ToBuf(uint8_t* out, uint32_t v)
{
    /* most significant last */
    /* least significant first */
    *out++ = v & 0xff;
    v >>= 8;
    *out++ = v & 0xff;
    v >>= 8;
    *out++ = v & 0xff;
    v >>= 8;
    *out = v & 0xff;
}

uint32_t getu32FromBuf(const uint8_t* in)
{
    uint32_t ret;

    ret = in[3];
    ret <<= 8;
    ret |= in[2];
    ret <<= 8;
    ret |= in[1];
    ret <<= 8;
    ret |= in[0];

    return ret;
}



struct _fwd_ fwd;
struct _nr_ notResponding;

void txDoneCB()
{
    if (flags.getAns) {
        unsigned toms;
        setPreambleSize(false, 5);    // getting answer
        Radio::Rx(0);
#ifndef GATEWAY
        if (reqFlags.bits.currentOp == CMD_DISCOVERY_REQ) {
            /* discovering: listen for answers from any upstream devices */
            toms = discovery_ans_time_total_us / 1000;
            queue.call_in(toms, discovery_rx_end);
        } else
#endif /* !GATEWAY */
        {
            unsigned target_us;
            target_us = Radio::lora_toa_us(ANS_SIZE_BYTE) * 4;  // four packet length's worth
            toms = (ANS_PAD_MS + ANS_PAD_MS) + (target_us / 1000); // microseconds to milliseconds
            req_timeout_id = queue.call_in(toms, txBuf_send, true);
        }
    } else if (reqFlags.bits.txAns != NOT_ANSWERING) { // txDone callback answer-tx-complete
        /* we just sent answer: restore to idle condition waiting for wakeup packet */
        reqFlags.bits.txAns = NOT_ANSWERING;
        txBuf_idx = 0;

#ifndef GATEWAY
        if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ || reqFlags.bits.currentOp == CMD_USER_PAYLOAD_DN_REQ) {
            int n;
            if (fwd.len >= 0) {
                tx_dest_id = fwd.tx_dest_id;
                if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ) {
                    /* forward upstream */
                    txBuf[txBuf_idx++] = CMD_USER_PAYLOAD_UP_REQ;
                    putu32ToBuf(&txBuf[txBuf_idx], fwd.B_id);   // originating_src_id
                    txBuf_idx += 4;
                } else if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_DN_REQ) {
                    /* forward downstream */
                    txBuf[txBuf_idx++] = CMD_USER_PAYLOAD_DN_REQ;
                    putu32ToBuf(&txBuf[txBuf_idx], fwd.A_id);    // final_dest_id
                    txBuf_idx += 4;
                }
                txBuf[txBuf_idx++] = fwd.len;
                for (n = 0; n < fwd.len; n++)
                    txBuf[txBuf_idx++] = fwd.buf[n];

                queue.call_in(500, txBuf_send, true);
                fwd.len = -1;
            } // ..if (fwd.len >= 0)
            else {
                /* uplink/downlink not forwarding */
                if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ)
                    app_uplink_complete();
                reqFlags.bits.currentOp = CMD_UNUSED;
            }
        }

        if (attUp.id == ID_NONE) {
            /* disconnected from upstream, rediscover */
            queue.call_in(1000, upstream_init);
        } else if (id_newDeviceNotification != ID_NONE) {
            upstream_new_device_notify();
        } else if (reqFlags.bits.currentOp == CMD_DOWNSTREAM_NOT_RESPONDING) {
            tx_dest_id = attUp.id;
            txBuf[txBuf_idx++] = CMD_DOWNSTREAM_NOT_RESPONDING;
            putu32ToBuf(&txBuf[txBuf_idx], notResponding.reporting_id);
            txBuf_idx += 4;
            putu32ToBuf(&txBuf[txBuf_idx], notResponding.device_not_respoding_id);
            txBuf_idx += 4;
            queue.call_in(500, txBuf_send, true);
        } else
#endif /* !GATEWAY */
        if (downRemove.destID != ID_NONE) {
            request_remove_device();
        }
        start_periodic_rxing(0x60); // reqFlags.bits.txAns != NOT_ANSWERING
    } else if (reqFlags.bits.currentOp == CMD_DISCOVERY_ANS) {
        if (flags.firstDiscoverAns) {
            unsigned rnd = Radio::Random() % N_HALF_DISCOVERY_ANS;
            queue.call_in((discovery_ans_time_step_us * rnd) / 1000, txBuf_send, false);
            flags.firstDiscoverAns = 0;
        } else {
            reqFlags.bits.currentOp = CMD_UNUSED;
            txBuf_idx = 0;
            //start_periodic_rxing(0x50); // 2nd discoverAns
        }
    } else {
        /* ? wtf did we just transmit ? */
        pc.printf("\e[31mnoTxAns_or_STATE_GET_ANS\e[0m ");
    }
} // ..txDoneCB()


uint16_t crc16( uint8_t *buffer, uint16_t length )
{
    uint16_t i;
    // The CRC calculation follows CCITT
    const uint16_t polynom = 0x1021;
    // CRC initial value
    uint16_t crc = 0x0000;
 
    if( buffer == NULL )
    {
        return 0;
    }
 
    for( i = 0; i < length; ++i )
    {
        uint16_t j;
        crc ^= ( uint16_t ) buffer[i] << 8;
        for( j = 0; j < 8; ++j )
        {
            crc = ( crc & 0x8000 ) ? ( crc << 1 ) ^ polynom : ( crc << 1 );
        }
    }
 
    return crc;
}


void rxDoneCB(uint8_t size, float rssi, float snr)
{
    uint8_t rx_buf_idx;
    uint16_t calc, rxCrc;
    uint8_t rx_hfg = Radio::radio.rx_buf[0];
    uint32_t sending_id = getu32FromBuf(&Radio::radio.rx_buf[1]);
    uint32_t dest_id = getu32FromBuf(&Radio::radio.rx_buf[5]);
#ifdef GATEWAY
    upInfo_t up_info;
    up_info.originating_src_id = ID_NONE;
#endif /* GATEWAY */

#ifdef MESH_DEBUG
    bool print_log_here = true;
    rx_log_buf_idx = 0;
    rx_log[0] = 0;
    rx_log_disable = false;
#endif /* MESH_DEBUG */

    if (flags.vacantCheck) {
        channelVacantCount = 0;
        flags.vacantCheck = 0;
    }

    Rx_log_printf("\e[32mrxDone %ubytes %.1fdBm %.1fdB\e[0m ", size, rssi, snr);
    Rx_log_printf("from:%lx_to_%lx ", sending_id, dest_id);

    if (dest_id != my_id && dest_id != ANY_ID) {
#ifndef GATEWAY
        /* check if upstream device were attached to is re-attaching */
        upstream_attached_check(sending_id);
#endif /* !GATEWAY */
        if (!flags.discoverAnswering)
            start_periodic_rxing(0x40); // rxDone notForMe
        goto done;
    }

    calc = crc16(Radio::radio.rx_buf, size-2);
    rxCrc = getu16FromBuf(&Radio::radio.rx_buf[size-2]);
    if (calc != rxCrc) {
        Rx_log_printf("%04x != %04x\r\n", calc, rxCrc);
        if (!flags.discoverAnswering)
            start_periodic_rxing(0x30); // rxDone crcfail
        goto done;
    }

    size -= 2;  // take off trailing crc
    for (rx_buf_idx = 9; rx_buf_idx < size; ) {
        cmd_e cmd = (cmd_e)Radio::radio.rx_buf[rx_buf_idx++];

        Rx_log_printf(" curOp:%u_", reqFlags.bits.currentOp);
        Rx_log_printf(" \e[7mRxCmd:%s\e[0m", cmdStrs[cmd]);
        Rx_log_printf(" ");
            
        switch (cmd) {
            ans_e ans;
            case CMD_ANS:
                ans = (ans_e)Radio::radio.rx_buf[rx_buf_idx++];
                /* request as been answered successfully */
                Rx_log_printf("\e[35mrxAns\e[0m ");
                if (flags.getAns) {
                    if (ans == ANSWER_OK) {
                        txBuf_idx = 0;
                        queue.cancel(req_timeout_id);
                        attemptCnt = 0;
                        downstream_ans_rxDoneCB(rssi, snr, &rx_buf_idx, sending_id, cmd);
#ifndef GATEWAY
                        upstream_ans_rxDoneCB(rssi, snr, &rx_buf_idx, sending_id, cmd);
#endif /* !GATEWAY */

                        if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_UP_REQ) {
                            fwd.len = -1;
                            reqFlags.bits.currentOp = CMD_UNUSED;
                        }

                        if (reqFlags.bits.currentOp == CMD_USER_PAYLOAD_DN_REQ) {
                            fwd.len = -1;
                            reqFlags.bits.currentOp = CMD_UNUSED;
                        }
                    }
                }

                break;
            case CMD_UNUSED:
                break;
            case CMD_DISCOVERY_REQ:
            case CMD_ATTACH_REQ:
            case CMD_NEW_DEVICE_ATTACHED_REQ:
            case CMD_USER_PAYLOAD_UP_REQ:
            case CMD_DOWNSTREAM_NOT_RESPONDING:
#ifdef GATEWAY
                downstream_req_rxDoneCB(rssi, snr, &rx_buf_idx, sending_id, cmd, &up_info);
#else
                downstream_req_rxDoneCB(rssi, snr, &rx_buf_idx, sending_id, cmd);
#endif
                break;
            case CMD_REMOVE_DEVICE_REQ:
            case CMD_DISCOVERY_ANS:
            case CMD_USER_PAYLOAD_DN_REQ:
                upstream_req_rxDoneCB(rssi, snr, &rx_buf_idx, sending_id, cmd);
                break;
        } // ..switch (cmd)

    } // ..for (rx_buf_idx = 9; rx_buf_idx < size; )

    if (reqFlags.bits.currentOp != CMD_DISCOVERY_ANS && reqFlags.bits.currentOp != CMD_DISCOVERY_REQ) {
        if (reqFlags.bits.txAns != NOT_ANSWERING) {
            queue.call(txAns, sending_id);  // must return from this function prior to transmitting
#ifdef MESH_DEBUG
            print_log_here = false;
#endif /* MESH_DEBUG */
        } else {
            Radio::Sleep();
            start_periodic_rxing(0x20); // rxDone
        }
    }

done:
#ifdef GATEWAY
    if (rx_hfg == 0) {
        Rx_log_printf("\e[31mrx_hfg:%u\e[0m ", rx_hfg);   /* another gateway */
    } else {
        Rx_log_printf("rx_hfg:%u ", rx_hfg);
    }
#else
    Rx_log_printf("rx_hfg:%u ", rx_hfg);
    /* compare against attached upstream */
    if (hops_from_gateway != HFG_UNATTACHED)
        upstream_signal_check(rssi, snr, rx_hfg, sending_id);
#endif

#ifdef MESH_DEBUG
    if (print_log_here)
        queue.call_in(10, rx_log_print);
    // else txAns must be completed first, will be called from txAns
#endif /* MESH_DEBUG */

#ifdef GATEWAY
    if (up_info.originating_src_id != ID_NONE) {
        /* txAns takes priority over application layer */
        queue.call(gateway_uplink, up_info.len, up_info.originating_src_id, &Radio::radio.rx_buf[up_info.rxBufIdx]);
    }
#endif /* GATEWAY */
} // ..rxDoneCB()

void txTimeoutCB()
{
    pc.printf("\e[41mTxTimeout\e[0m\r\n");
}

void rxTimeoutCB()
{
    Radio::Sleep();
    queue.call_in(WAKEUP_INTERVAL_MS, rxSingle);

    if (flags.vacantCheck) {
        channelVacantCount++;
        if (flags.deferred_send) {
            uint8_t vc_thresh = CHANNEL_VACANT_REQUIRED_COUNT;
            if (attemptCnt > 1) {
                /* retry backoff */
                vc_thresh += Radio::Random() % CHANNEL_VACANT_REQUIRED_COUNT;
            }
            if (channelVacantCount > vc_thresh) {
                _send_();
                flags.deferred_send = 0;
            }
        }
        flags.vacantCheck = 0;
    }
}

uint32_t find_dest_id(uint32_t reqid)
{
    lid_list_t* L;
    for (L = attachedDevices; L != NULL; L = L->next) {
        if (L->id == reqid)
            return L->id;   // is locally attached
    }

    for (L = attachedDevices; L != NULL; L = L->next) {
        if (L->attachedList != NULL) {
            cid_list_t* children;
            for (children = L->attachedList;  children != NULL; children = children->next) {
                if (children->id == reqid)
                    return L->id;
            }
        }
    }
    return ID_NONE;
}

void cmd_tx(uint8_t argsAt)
{
    unsigned symbs;
    if (sscanf(pcbuf+argsAt, "%u", &symbs) == 1) {
        Radio::LoRaPacketConfig(symbs, false, true, false);  // preambleLen, fixLen, crcOn, invIQ
        pc.printf("txing %u symbols\r\n", symbs);
    }

    txBuf[txBuf_idx++] = CMD_UNUSED;
    tx_dest_id = ANY_ID;
    txBuf_send(false);
}

void cmd_list_devices(uint8_t argsAt)
{
    lid_list_t* L;
    pc.printf("my_id:%lx  hops_from_gateway:%u\r\n", my_id, hops_from_gateway);
    for (L = attachedDevices; L != NULL; L = L->next) {
        pc.printf("%lx", L->id);
        if (L->attachedList != NULL) {
            cid_list_t* children;
            pc.printf(": ");
            for (children = L->attachedList;  children != NULL; children = children->next)
                pc.printf("%lx ", children->id);
        }
        pc.printf("\r\n");
    }
}

void cmd_print_status(uint8_t idx)
{
    radio_print_status();

    pc.printf("my_id:%lx  hops_from_gateway:%u\r\n", my_id, hops_from_gateway);
    pc.printf("ClearChan%u reqFlags:%02x ", channelVacantCount, reqFlags.octet);
#ifndef GATEWAY
    upstream_print_status();
#endif /* GATEWAY */
    pc.printf("\r\n");
}

typedef struct {
    const char* const cmd;
    void (*handler)(uint8_t args_at);
    const char* const arg_descr;
    const char* const description;
} menu_item_t;

void cmd_help(uint8_t);

const menu_item_t menu_items[] =   {
    { ".", cmd_print_status, "","print status"},
    { "op", cmd_op, "%u","get/set tx power"},
#ifdef GATEWAY
    { "dl", cmd_downlink, "%x %x...","send downlink <destIDhex> <payload bytes hex>"},
#endif /* GATEWAY */
    { "ls", cmd_list_devices, "%u","list seen downstream devices"},
    { "tx", cmd_tx, "%u","tx test preamble length"},
    { "?", cmd_help, "","this list of commands"},
    { NULL, NULL, NULL, NULL }
};

void cmd_help(uint8_t args_at)
{
    int i;
    
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        printf("%s%s\t%s\r\n", menu_items[i].cmd, menu_items[i].arg_descr, menu_items[i].description);
    }
}

void console()
{
    uint8_t i, user_cmd_len;
 
    if (pcbuf_len == 0)
        return;
        
    printf("\r\n");
        
    /* get end of user-entered command */
    user_cmd_len = 1;   // first character can be any character
    for (i = 1; i <= pcbuf_len; i++) {
        if (pcbuf[i] < 'A' || (pcbuf[i] > 'Z' && pcbuf[i] < 'a') || pcbuf[i] > 'z') {
            user_cmd_len = i;
            break;
        }
    }
 
    for (i = 0; menu_items[i].cmd != NULL ; i++) {
        int mi_len = strlen(menu_items[i].cmd);
        if (menu_items[i].handler && user_cmd_len == mi_len && (strncmp(pcbuf, menu_items[i].cmd, mi_len) == 0)) {
            while (pcbuf[mi_len] == ' ')   // skip past spaces
                mi_len++;
            menu_items[i].handler(mi_len);
            break;
        }
    }
   
    pcbuf_len = 0;
    printf("> ");
    fflush(stdout); 
}

void radio_irq_topHalf()
{
    /* isr context -> main loop context */
    queue.call(Radio::service);
}

const RadioEvents_t rev = {
    /* DioPin_top_half */     radio_irq_topHalf,
    /* TxDone_topHalf */    NULL,
    /* TxDone_botHalf */    txDoneCB,
    /* TxTimeout  */        txTimeoutCB,
    /* RxDone  */           rxDoneCB,
    /* RxTimeout  */        rxTimeoutCB,
    /* RxError  */          NULL,
    /* FhssChangeChannel  */NULL,
    /* CadDone  */          NULL
};

void rx_callback()
{
    static uint8_t pcbuf_idx = 0;
    static uint8_t prev_len = 0;
    char c = pc.getc();
    if (c == 8) {
        if (pcbuf_idx > 0) {
            pc.putc(8);
            pc.putc(' ');
            pc.putc(8);
            pcbuf_idx--;
        }
    } else if (c == 3) {    // ctrl-C
        pcbuf_len = -1;
    } else if (c == '\r') {
        if (pcbuf_idx == 0) {
            pcbuf_len = prev_len;
        } else {
            pcbuf[pcbuf_idx] = 0;   // null terminate
            prev_len = pcbuf_idx;
            pcbuf_idx = 0;
            pcbuf_len = prev_len;
        }
        queue.call(console);
    } else if (pcbuf_idx < sizeof(pcbuf)) {
        pcbuf[pcbuf_idx++] = c;
        pc.putc(c);
    }
}

int main()
{
    pc.baud(115200);
    pc.printf("\r\nreset\r\n");
    pc.attach(rx_callback);

    {
        uint32_t u32;
#ifdef TARGET_FAMILY_STM32 
        u32 = LL_GetUID_Word0();
        u32 <<= 2;
        u32 ^= LL_GetUID_Word1();
        u32 ^= LL_GetUID_Word2();
#else
#error TODO_nSTM32
#endif
        my_id = u32;
        pc.printf("my_id %lx\r\n", my_id);
    }

    wait_ms(200);    // power stabilization from cold-reset
    Radio::Init(&rev);

    rxdbg = 0;
    Radio::Standby();
    Radio::LoRaModemConfig(BW_KHZ, SPREADING_FACTOR, 1);
    Radio::SetChannel(CF_MHZ * 1000000);
    Radio::set_tx_dbm(TX_DBM);
#ifdef SX126x_H 
    {
        status_t status;
        uint8_t stopOnPreamble = 1;

        Radio::radio.xfer(OPCODE_GET_STATUS, 0, 1, &status.octet);
        wait_ms(20);
        Radio::radio.xfer(OPCODE_STOP_TIMER_ON_PREAMBLE, 1, 0, &stopOnPreamble);
    }
#endif /* SX126x_H  */

    setPreambleSize(false, 4); //init
    {
        unsigned daDur = Radio::lora_toa_us(DISCOVERY_ANS_LENGTH);
        discovery_ans_time_step_us = daDur + (ANS_PAD_MS * 1000); // + padding for receiver handling
        discovery_ans_time_total_us = discovery_ans_time_step_us * N_DISCOVERY_ANS;
    }

#ifdef SX128x_H 
    /* C preprocess doesnt do floating point */
    if (N_PRE_SYMB > 255) {
        pc.printf("\e[41mlong preamble oversized %.1f\e[0m\r\n", N_PRE_SYMB);
    }
#endif /* ..SX128x_H */

#ifdef GATEWAY
    start_periodic_rxing(0x10);  // gateway startup
#else
    init_attached_upstream();
    hops_from_gateway = HFG_UNATTACHED;
    upstream_init();
#endif

    app_init();
    fwd.len = -1;

/*    if (!sleep_manager_can_deep_sleep()) {
        sleep_manager_unlock_deep_sleep();
        pc.printf("unLockDeepSleep\r\n");
    }*/

    queue.dispatch();
} // ..main()

