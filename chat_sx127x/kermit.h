#if 0
#include "mbed.h"

#define RADIO_FILE_XFER

#ifdef RADIO_FILE_XFER
#include "sx127x_lora.h"
#endif /* RADIO_FILE_XFER */


#define SOH     0x01
#define NAK     0x15
#define CAN     0x18

typedef enum {
    KERMIT_STATE_OFF = 0,
    KERMIT_STATE_WAIT_SOH,
    KERMIT_STATE_WAIT_LEN,
    KERMIT_STATE_WAIT_SEQ,
    KERMIT_STATE_WAIT_TYPE,
    KERMIT_STATE_DATA,
    KERMIT_STATE_GET_EOL,
} kermit_state_e;


class Kermit
{
    public:
        Kermit(SX127x_lora& _lora);
        ~Kermit();
        
        bool uart_rx_enabled;
        bool got_send_init;
        bool end;
        uint8_t end_cause;
        uint32_t total_file_bytes;
        SX127x_lora& lora;
        char filename[128];
        
        uint32_t bin_data_u32[128];
        uint8_t* bin_data;
        uint32_t bin_data_idx;        
        
        uint8_t maxl;
        uint8_t time;
        uint8_t npad;
        char padc;
        char eol;
        char qctl;  // verbatim quote char
        char qbin;
        char chkt;
        char rept;        
        
        void rx_callback(uint8_t);
        void uart_rx_enable(void);
        void service(void);
        uint32_t _HAL_CRC_Calculate(uint32_t u32_buf[], uint32_t Size);
        void test_crc(void);
        
    protected:
        bool show_error;
        bool end_after_tx;
        kermit_state_e state;
        void kermit_uart_tx(void);    
        int parse_rx(void);
        void radio_xfer_rx(void);
        
        uint8_t tochar(uint8_t c);
        uint8_t unchar(uint8_t c);
        uint8_t ctl(uint8_t c);
        
        uint32_t uart_rx_sum;        
        uint8_t uart_rx_data[128];
        uint8_t uart_rx_data_idx;
        uint8_t uart_rx_length; 
        char uart_rx_seq;
        char uart_rx_type;
        
        float uart_tx_sleep;
        uint8_t uart_tx_data[128];
        uint8_t uart_tx_data_idx;
        bool uart_do_tx;
};
#endif /* #if 0 */