#if 0

#include "kermit.h"


#ifdef RADIO_FILE_XFER
typedef enum {
    XFER_STATE__NONE = 0,
    XFER_STATE_WAIT_S,  // 1
    XFER_STATE_S_ACKED, // 2
    XFER_STATE_WAIT_F, // 3
    XFER_STATE_F_ACKED, // 4
    XFER_STATE_WAIT_DATA_ACK,   // 5
    XFER_STATE_D_ACKED, // 6
    XFER_STATE_WAIT_Z,  // 7
    XFER_STATE_Z_ACKED,  // 8
    XFER_STATE_WAIT_B,  // 9
    XFER_STATE_B_ACKED  // 10
} xfer_state_e;

typedef struct {
    bool radio_initialized;
    bool do_tx;
    xfer_state_e state;
    int fail_length;
    float tx_sleep;
    char seq_from_rx;
    float data_tx_delay;
    char data_ack_char; // tmp debug
} radio_xfer_t;
radio_xfer_t radio_xfer;

#endif /* RADIO_FILE_XFER */

#ifdef TARGET_NUCLEO_F103RB
/* NUCLEO-F103RB UARTs:
 * #  TX       RX        use
 * 2  PA_2     PA_3     mbed default
 * 1  PA_9     PA_10          PA_9=D8=DIO4a   PA_10=D2=DIO0
 * 3  PB_10    PB_11     PB_10=D6=nothing   PB11=C26=4.7uF
 * 1  PB_6     PB_7     remap      PB_6=D10=SX1276_NSS   PB7=CN7-21
 * 3  PC_10    PC_11    partial remap
 * SX127x radio(D11,   D12, D13,    D10,  A0,   D2,   D3); 
 */
Serial pc_b(PB_10, PB_11);   //PB_10=D6=nothing   PB11=C26=4.7uF
CRC_HandleTypeDef   CrcHandle;
#endif /* TARGET_NUCLEO_F103RB */

#ifdef TARGET_LPC11U6X
/* U1_RXD: PIO0_13=A2, PIO1_2=P2-24
 * U1_TXD: PIO0_14=A1, PIO1_8=P2-50
 * U0_RXD: PIO0_18=mbed, PIO1_26=D5, PIO1_17=J4-4
 * U0_TXD: PIO0_19=mbed, PIO1_18=D2, PIO1_27=D6
 * U2_RXD: PIO0_20=P2-14, PIO1_6=P2-53
 * U2_TXD: PIO1_0=P2-13, PIO1_23=J4-1
 * PIO2_3:  U3_RXD=D9
 * PIO2_4:  U3_TXD=J8-3
 * PIO2_11: U4_RXD
*/
Serial pc_b(P1_8, P1_2);    // TX=PIO1_8=P2-50,    RX=PIO1_2=P2-24
#endif

#define MAX_LEN_FRAME       130 /* */

Kermit::Kermit(SX127x_lora& _lora) : lora(_lora)
{
    uart_rx_enabled = false;
}

Kermit::~Kermit()
{
}

uint8_t Kermit::tochar(uint8_t c) { return c + 32; }
uint8_t Kermit::unchar(uint8_t c) { return c - 32; }
uint8_t Kermit::ctl(uint8_t c) { return c ^ 64; }

void Kermit::rx_callback(uint8_t c)
{
    static uint8_t ctrl_c_cnt = 0;    
    
    if (c == 3) {
        if (++ctrl_c_cnt > 3) {
            uart_rx_enabled = false;
            end_cause = 3;
            end = true;
            return;
        }
    } else
        ctrl_c_cnt = 0;  
              
    switch (state) {
        case KERMIT_STATE_WAIT_SOH:
            if (c == SOH) {
                state = KERMIT_STATE_WAIT_LEN;
            }
            break;
        case KERMIT_STATE_WAIT_LEN:
            uart_rx_sum = c;
            uart_rx_length = unchar(c) - 2;
            state = KERMIT_STATE_WAIT_SEQ;
            break;
        case KERMIT_STATE_WAIT_SEQ:
            uart_rx_sum += c;
            uart_rx_seq = unchar(c);
            state = KERMIT_STATE_WAIT_TYPE;
            break;
        case KERMIT_STATE_WAIT_TYPE: 
            uart_rx_sum += c;
            uart_rx_type = c;
            state = KERMIT_STATE_DATA;
            uart_rx_data_idx = 0;
            break;
        case KERMIT_STATE_DATA:
            uart_rx_data[uart_rx_data_idx++] = c;
            if (uart_rx_data_idx == uart_rx_length) {
                char check = tochar((uart_rx_sum + ((uart_rx_sum & 192)/64)) & 63);
                if (check == c) {
                    if (parse_rx()) {
                        //kermit_state = KERMIT_STATE_GET_EOL;
                        end_cause = 4;
                        /////////////////////////
                        uart_tx_data_idx = 0;
                        uart_tx_data[uart_tx_data_idx++] = 'E';
                        uart_tx_data[uart_tx_data_idx++] = uart_rx_type;
                        uart_do_tx = true;  
                        end_after_tx = true;
                        /////////////////////////                           
                        break;
                    }
                }
                if (uart_rx_type == 'E') {
                    state = KERMIT_STATE_GET_EOL;
                    end_cause = 2;
                } else
                    state = KERMIT_STATE_WAIT_SOH;
            } else
                uart_rx_sum += c;
            break;
        case KERMIT_STATE_GET_EOL:   
            uart_rx_enabled = false;
            end = true;
            if (uart_rx_type == 'E')
                show_error = true;
            uart_rx_data[uart_rx_data_idx-1] = 0;  //null terminate, this is ascii text string          
            state = KERMIT_STATE_OFF;
            break;
    } // ..switch (state)    
}

#ifdef RADIO_FILE_XFER
void Kermit::radio_xfer_rx()
{
    radio_xfer.seq_from_rx = lora.m_xcvr.rx_buf[0];
    pc_b.printf("rfrx:%02x,%c\r\n", radio_xfer.seq_from_rx, lora.m_xcvr.rx_buf[1]);
    
    switch (radio_xfer.state) {
        case XFER_STATE_WAIT_S: // 'S' response
            end_cause = 0;
            filename[0] = 0;
            uart_tx_data_idx = 0;
            if (lora.m_xcvr.rx_buf[1] == 'S') {
                uart_tx_data[uart_tx_data_idx++] = 'Y';
                uart_tx_data[uart_tx_data_idx++] = tochar(94);   // MAXL
                //uart_tx_data[uart_tx_data_idx++] = tochar(kermit.time-1);//TIME
                uart_tx_data[uart_tx_data_idx++] = tochar(1);//TIME (when reply from other radio is bad)
                uart_tx_data[uart_tx_data_idx++] = tochar(0);    //NPAD
                uart_tx_data[uart_tx_data_idx++] = tochar(32);   //PADC
                uart_tx_data[uart_tx_data_idx++] = tochar('\r'); //EOL
                uart_tx_data[uart_tx_data_idx++] = '#';  //QCTL
                uart_tx_data[uart_tx_data_idx++] = 'Y'; //QBIN
                uart_tx_data[uart_tx_data_idx++] = '1'; //CHKT
                uart_tx_data[uart_tx_data_idx++] = '~'; //REPT                
            } else
                uart_tx_data[uart_tx_data_idx++] = 'E';
                
            uart_do_tx = true;  
            radio_xfer.state = XFER_STATE_S_ACKED;
            break;
        case XFER_STATE_WAIT_F: // 'F' response
            uart_tx_data_idx = 0;
            if (lora.m_xcvr.rx_buf[1] == 'F') {
                uart_tx_data[uart_tx_data_idx++] = 'Y';
            } else
                uart_tx_data[uart_tx_data_idx++] = 'E'; 
                
            uart_do_tx = true; 
            radio_xfer.state = XFER_STATE_F_ACKED;
            break;
        case XFER_STATE_WAIT_DATA_ACK:
            if (lora.m_xcvr.rx_buf[1] == 'E' || lora.m_xcvr.rx_buf[1] == 'Y' || lora.m_xcvr.rx_buf[1] == 'N') {
                uart_tx_data_idx = 0;
                radio_xfer.data_ack_char = lora.m_xcvr.rx_buf[1];
                uart_tx_data[uart_tx_data_idx++] = lora.m_xcvr.rx_buf[1];
                uart_do_tx = true;
                radio_xfer.state = XFER_STATE_D_ACKED;
            }
            break;
        case XFER_STATE_WAIT_Z:
            uart_tx_data_idx = 0;
            uart_tx_data[uart_tx_data_idx++] = lora.m_xcvr.rx_buf[1];
            uart_do_tx = true; 
            radio_xfer.state = XFER_STATE_Z_ACKED;        
            break;
        case XFER_STATE_WAIT_B:
            uart_tx_data_idx = 0;
            uart_tx_data[uart_tx_data_idx++] = lora.m_xcvr.rx_buf[1];
            uart_do_tx = true; 
            end_after_tx = true;
            radio_xfer.state = XFER_STATE_B_ACKED;        
            break;        
    } // ..switch (radio_xfer.state)
    //radio_xfer.ack_waiting = false;
}
#endif /* RADIO_FILE_XFER */  


#ifdef TARGET_STM
uint32_t Kermit::_HAL_CRC_Calculate(uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index = 0;

  /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&CrcHandle);
  __nop();
  __nop();
  __nop();
  __nop();
  __nop();

  /* Enter Data to the CRC calculator */
  for(index = 0; index < BufferLength; index++)
  {
    CrcHandle.Instance->DR = pBuffer[index];
    //printf("Calc %08x\r\n", CrcHandle.Instance->DR);
      __nop();
      __nop();
      __nop();    
  }

  /* Return the CRC computed value */
  return CrcHandle.Instance->DR;
}


 // 20000788 08003101 08007e35 08007e37 08007e39 08007e3b 08007e3d 00000000 00000000 00000000 00000000 08007e3f 08007e41 00000000 08007e43 08007e45 0000311b
 void Kermit::test_crc()
 {
     uint32_t tbuf[17] = { 0x20000788, 0x08003101, 0x08007e35, 0x08007e37, 0x08007e39, 0x08007e3b, 0x08007e3d, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x08007e3f, 0x08007e41, 0x00000000, 0x08007e43, 0x08007e45, 0x0000311b };
     uint32_t crc;
     int i;
     crc = _HAL_CRC_Calculate(tbuf, 17);
     printf("Crc:%08x\r\n", crc);
       __HAL_CRC_DR_RESET(&CrcHandle);
  __nop();
  __nop();
  __nop();
  __nop();
  __nop();
    for (i = 0; i < 17; i++) {
        CrcHandle.Instance->DR = tbuf[i];
        printf("%08x: %08x\r\n", tbuf[i], CrcHandle.Instance->DR);
      __nop();
      __nop();
      __nop();           
        
    }
     printf(": %08x\r\n", CrcHandle.Instance->DR);
     
}
#else // !STM...
const uint32_t CrcTable[16] = { // Nibble lookup table for 0x04C11DB7 polynomial
        0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
        0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD };
uint32_t Kermit::_HAL_CRC_Calculate(uint32_t u32_buf[], uint32_t Size)
{
    int i = 0;
    uint32_t Crc = 0xffffffff;

    while(Size--)
    {
        Crc = Crc ^ u32_buf[i++];

        // Process 32-bits, 4 at a time, or 8 rounds
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; // Assumes 32-bit reg, masking index to 4-bits
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; //  0x04C11DB7 Polynomial used in STM32
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
        Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    }

    return(Crc);
}
uint32_t g_tbuf[17] = { 0x20000788, 0x08003101, 0x08007e35, 0x08007e37, 0x08007e39, 0x08007e3b, 0x08007e3d, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x08007e3f, 0x08007e41, 0x00000000, 0x08007e43, 0x08007e45, 0x0000311b };

// 20000788 08003101 08007e35 08007e37 08007e39 08007e3b 08007e3d 00000000 00000000 00000000 00000000 08007e3f 08007e41 00000000 08007e43 08007e45 0000311b
void Kermit::test_crc()
{
    //uint32_t tbuf[17] = { 0x20000788, 0x08003101, 0x08007e35, 0x08007e37, 0x08007e39, 0x08007e3b, 0x08007e3d, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x08007e3f, 0x08007e41, 0x00000000, 0x08007e43, 0x08007e45, 0x0000311b };
    uint32_t crc;
    //uint32_t *u32_ptr;
    //u32_ptr = (uint32_t*)bin_data;
    
    printf("test_crc...%p\r\n", g_tbuf);
    //u32_ptr[0] = 0x20000788;
    //u32_ptr[1] = 0x08003101;
    
    //crc = _HAL_CRC_Calculate((uint32_t*)bin_data, 17);
    crc = _HAL_CRC_Calculate(g_tbuf, 17);
    printf("crc:%08x\r\n", crc);
}
#endif /* !TARGET_STM */


int Kermit::parse_rx()
{
    static char prev_uart_rx_seq;
    static uint32_t prev_bin_data_idx;
    
     //pc_b.printf("kermit_parse_rx %02x '%c'\r\n", uart_rx_seq, uartrx_type);
#ifdef RADIO_FILE_XFER    
    lora.m_xcvr.tx_buf[0] = uart_rx_seq;
    lora.m_xcvr.tx_buf[1] = uart_rx_type;
#endif /* RADIO_FILE_XFER */
            
    if (uart_rx_type == 'S') {
        got_send_init = true;

        if (uart_rx_data_idx > 0)
            maxl = unchar(uart_rx_data[0]);
        if (uart_rx_data_idx > 1)
            time = unchar(uart_rx_data[1]);    
        if (uart_rx_data_idx > 2)
            npad = unchar(uart_rx_data[2]);
        if (uart_rx_data_idx > 3)
            padc = unchar(uart_rx_data[3]);
        if (uart_rx_data_idx > 4)
            eol  = unchar(uart_rx_data[4]);
        if (uart_rx_data_idx > 5)
            qctl = uart_rx_data[5];   // verbatim
        if (uart_rx_data_idx > 6)
            qbin = uart_rx_data[6];   // verbatim   'Y'==agree-to-8bit, 'N'=no-8bit '&'==I need this char to do 8bit quoting
        if (uart_rx_data_idx > 7)
            chkt = uart_rx_data[7];   // verbatim
        if (uart_rx_data_idx > 8)
            rept = uart_rx_data[8];  

#ifdef RADIO_FILE_XFER
        lora.RegPayloadLength = 2;
        lora.m_xcvr.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        radio_xfer.do_tx = true;
        radio_xfer.state = XFER_STATE_WAIT_S;
#else
        uart_tx_data_idx = 0;
        uart_tx_data[uart_tx_data_idx++] = 'Y';
        uart_tx_data[uart_tx_data_idx++] = tochar(94);   // MAXL
        uart_tx_data[uart_tx_data_idx++] = tochar(kermit.time-1);//TIME
        uart_tx_data[uart_tx_data_idx++] = tochar(0);    //NPAD
        uart_tx_data[uart_tx_data_idx++] = tochar(32);   //PADC
        uart_tx_data[uart_tx_data_idx++] = tochar('\r'); //EOL
        uart_tx_data[uart_tx_data_idx++] = '#';  //QCTL
        uart_tx_data[uart_tx_data_idx++] = 'Y'; //QBIN
        uart_tx_data[uart_tx_data_idx++] = '1'; //CHKT
        uart_tx_data[uart_tx_data_idx++] = '~'; //REPT
        uart_do_tx = true;
#endif /* !RADIO_FILE_XFER */
        pc_b.printf("S\r\n");
    } else if (uart_rx_type == 'F') {
        /* keep filename if desired */
        memcpy(filename, uart_rx_data, uart_rx_data_idx-1);
        filename[uart_rx_data_idx-1] = 0;
        
        total_file_bytes = 0;
        prev_uart_rx_seq = uart_rx_seq;
        prev_bin_data_idx = 0;
#ifdef XXD_PRINT
        xxd_total_file_bytes_so_far = 0;
        xxd_remainder = 0;
#endif /* XXD_PRINT */        

#ifdef RADIO_FILE_XFER
        lora.RegPayloadLength = 2;
        lora.m_xcvr.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        radio_xfer.do_tx = true;
        radio_xfer.tx_sleep = radio_xfer.data_tx_delay;
        radio_xfer.state = XFER_STATE_WAIT_F;
#else
        uart_tx_data_idx = 0;
        uart_tx_data[uart_tx_data_idx++] = 'Y';
        /*memcpy(uart_tx_data+1, uart_rx_data, uart_rx_data_idx-1);
        uart_tx_data_idx += uart_rx_data_idx-1;*/
        uart_do_tx = true;
#endif /* !RADIO_FILE_XFER */   
        bin_data = (uint8_t*)bin_data_u32;
        pc_b.printf("F\r\n");
    } else if (uart_rx_type == 'D') {
        int i;
#ifdef RADIO_FILE_XFER
        uint8_t len_for_crc;
        //uint32_t* u32_ptr;
        uint32_t uwCRCValue;
#endif /* RADIO_FILE_XFER */   
        
        bin_data_idx = 0;

#ifdef KERMIT_DATA_PRINT
        pc_b.printf("%04x: ", kermit.total_file_bytes);
#endif /* */            
     
        uart_rx_data_idx--;   // cut off trailing sum byte
        for (i = 0; i < uart_rx_data_idx; i++) {
            if (uart_rx_data[i] == qctl) { // escaped..
                uint8_t in = uart_rx_data[++i];
#ifdef KERMIT_DATA_PRINT
                    pc_b.printf("#");
#endif /* */   
                if ((in & 0x7f) == rept || (in & 0x7f) == qctl) {
#ifdef KERMIT_DATA_PRINT
                    pc_b.printf(":%02x ", in);
#endif /* */                  
                    bin_data[bin_data_idx++] = in;
                } else {
#ifdef KERMIT_DATA_PRINT
                    pc_b.printf("ctl:%02x ", ctl(in));
#endif /* */                      
                    bin_data[bin_data_idx++] = ctl(in);
                }
            } else if (uart_rx_data[i] == rept) {   //repeat..
                uint8_t octet, cnt = unchar(uart_rx_data[++i]);
                i++;    // step past count
                if (uart_rx_data[i] == qctl) {
                    uint8_t raw = uart_rx_data[++i];
                    octet = ctl(raw);
                } else {
                    octet = uart_rx_data[i];
                }
                for (int n = 0; n < cnt; n++) {
#ifdef KERMIT_DATA_PRINT
                    pc_b.printf("rep%02x ", octet);
#endif /*  */            
                    bin_data[bin_data_idx++] = octet;
                }
            } else {
#ifdef KERMIT_DATA_PRINT
                    pc_b.printf("%02x ", uart_rx_data[i]);
#endif /*  */    
                bin_data[bin_data_idx++] = uart_rx_data[i];
            }
        } // ..for()
#ifdef KERMIT_DATA_PRINT
        pc_b.printf("\r\n");
#endif /*  */                   
        
        if (prev_uart_rx_seq == uart_rx_seq) {
            // resend of previous 'D' packet
            total_file_bytes -= prev_bin_data_idx;
        } else {
#ifdef XXD_PRINT
            xxd_print(0);
#endif /* XXD_PRINT */            
        }

        total_file_bytes += bin_data_idx;
        prev_bin_data_idx = bin_data_idx;
        prev_uart_rx_seq = uart_rx_seq;        
        
#ifdef RADIO_FILE_XFER               
        // zero-pad to 4byte size alignment
        len_for_crc = bin_data_idx;
        while (len_for_crc & 3) {
            bin_data[len_for_crc++] = 0;
        }
        uwCRCValue = _HAL_CRC_Calculate((uint32_t *)bin_data, len_for_crc >> 2);
        //kermit.crc32 = uwCRCValue;
        //memcpy(crc_buf, bin_data, len_for_crc);
        //kermit.len_for_crc = len_for_crc;
    
        
        if (bin_data_idx > MAX_LEN_FRAME) {  // oversized
            radio_xfer.fail_length = bin_data_idx;
            return 1;   // fail
        }
        
        //u32_ptr = (uint32_t *)&lora.m_xcvr.tx_buf[2];
        //pc_b.printf("D-hhh %p\r\n", u32_ptr);
        //*u32_ptr = uwCRCValue;
        memcpy(&lora.m_xcvr.tx_buf[2], &uwCRCValue, 4);
/*        
        if (prev_rx_seq == kermit.rx_seq) {
            // resend of previous 'D' packet
            radio_xfer.total_file_bytes -= prev_bin_data_idx;
        }
        radio_xfer.total_file_bytes += bin_data_idx;
        prev_bin_data_idx = bin_data_idx;
        prev_rx_seq = kermit.rx_seq;
*/
        
        memcpy(lora.m_xcvr.tx_buf+6, bin_data, bin_data_idx);
        lora.RegPayloadLength = bin_data_idx+6;
        lora.m_xcvr.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        radio_xfer.do_tx = true;
        radio_xfer.state = XFER_STATE_WAIT_DATA_ACK;
        radio_xfer.tx_sleep = radio_xfer.data_tx_delay;
#else 
        /* send ACK.. */
        while (uart_do_tx);
        uart_tx_data_idx = 0;
        uart_tx_data[uart_tx_data_idx++] = 'Y';
        uart_do_tx = true;        
#endif /* !RADIO_FILE_XFER */
        return 0;
    } else if (uart_rx_type == 'Z') {
#ifdef RADIO_FILE_XFER  
        lora.RegPayloadLength = 2;
        lora.m_xcvr.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        radio_xfer.do_tx = true;
        radio_xfer.tx_sleep = radio_xfer.data_tx_delay;
        radio_xfer.state = XFER_STATE_WAIT_Z;
#else
        /* send ACK.. */
        while (uart_do_tx);
        uart_tx_data_idx = 0;
        uart_tx_data[uart_tx_data_idx++] = 'Y';
        uart_do_tx = true;               
        uart_tx_sleep = 0.1;
#endif /* !RADIO_FILE_XFER */
        pc_b.printf("Z\r\n");
        return 0;
    } else if (uart_rx_type == 'B') {
#ifdef RADIO_FILE_XFER  
        lora.RegPayloadLength = 2;
        lora.m_xcvr.write_reg(REG_LR_PAYLOADLENGTH, lora.RegPayloadLength);
        radio_xfer.do_tx = true;
        radio_xfer.tx_sleep = radio_xfer.data_tx_delay;
        radio_xfer.state = XFER_STATE_WAIT_B;
#else
        uart_tx_data_idx = 0;
        uart_tx_data[uart_tx_data_idx++] = 'Y';
        uart_do_tx = true;               
        uart_tx_sleep = 0.1;
#endif /* RADIO_FILE_XFER */
#ifdef XXD_PRINT
        xxd_print(1);
#endif /* XXD_PRINT */        
        pc_b.printf("B\r\n");
        return 0;
    } else {
        // unknown packet, prevent further packets from overwriting
        return 1;
    }
    
    return 0;
}

void Kermit::kermit_uart_tx()
{
    uint8_t buf[128];
    uint8_t idx = 0;
    int i;
    uint32_t sum;
           
    buf[idx++] = SOH;   // MARK
    buf[idx++] = 0;   // length to be inserted later
#ifdef RADIO_FILE_XFER
    buf[idx++] = tochar(radio_xfer.seq_from_rx); // SEQ
#else
    buf[idx++] = tochar(uart_rx_seq);   // SEQ
#endif
    // TYPE is uart_tx_data[0]
    for (i = 0; i < uart_tx_data_idx; i++) {
        buf[idx++] = uart_tx_data[i];
    }
    
    buf[1] = tochar(idx - 1); // LEN  (-1 because block check hasnt been included in idx yet)
    
    sum = 0;
    for (i = 1; i < idx; i++) {
        sum += buf[i];
    }
    buf[idx++] = tochar((sum + ((sum & 192)/64)) & 63);
    buf[idx++] = eol;
    
    for (i = 0; i < idx; i++)
        putc(buf[i], stdout);
       //pc.putc(buf[i]);

}

void Kermit::service()
{
    if (end) {
        if (show_error) {
            printf("kermit error:\"%s\"\r\n", uart_rx_data);
            show_error = false;
        }
        printf("kermit_end\r\n");
        printf("total_file_bytes:%d\r\n", total_file_bytes);
        end = false;
    }
    
    if (uart_do_tx) {
        if (uart_tx_sleep > 0.001) {
            wait(uart_tx_sleep);
            uart_tx_sleep = 0;
        }
        kermit_uart_tx();
        uart_do_tx = false;
        if (end_after_tx) {
            //know this cause -- end_cause = 1;
            uart_rx_enabled = false;
            end = true;   
            state = KERMIT_STATE_OFF;            
            end_after_tx = false;
        }
    } // ...if (uart_do_tx)
    
#ifdef RADIO_FILE_XFER
    if (!radio_xfer.radio_initialized) {
        lora.m_xcvr.set_opmode(RF_OPMODE_STANDBY);   
        lora.m_xcvr.write_reg(REG_LR_SYNC_BYTE, 0x34);
        lora.setBw_KHz(500);
        lora.setSf(7); 
        lora.m_xcvr.set_frf_MHz(915.0);
        lora.invert_tx(true);
        radio_xfer.fail_length = -1;
        radio_xfer.radio_initialized = true;
    }

    if (radio_xfer.do_tx) {
        pc_b.printf("rfTX:%02x\r\n", lora.m_xcvr.tx_buf[0]);
        if (radio_xfer.tx_sleep > 0.001) {
            wait(radio_xfer.tx_sleep);
            radio_xfer.tx_sleep = 0;
        }
        lora.start_tx(lora.RegPayloadLength);
        radio_xfer.do_tx = false;
    }
#endif /* RADIO_FILE_XFER */    
}

void Kermit::uart_rx_enable()
{
    got_send_init = false;
    uart_rx_enabled = true;
    state = KERMIT_STATE_WAIT_LEN;
#ifdef RADIO_FILE_XFER    
    radio_xfer.radio_initialized = false;   // causes radio initialization for transfer
#endif /* RADIO_FILE_XFER */    
}
#endif /* #if 0 */