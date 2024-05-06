#include "radio.h"

#define USE_BAND_915
//#define USE_BAND_433

#ifdef SX128x_H

    #define LORAMAC_DEFAULT_DATARATE        5       // 5=sf7
    #define BANDWIDTH_KHZ                   200
    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)2486.9e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)300e3 )
    #define LORA_MAX_NB_CHANNELS            8

#elif defined(USE_BAND_915)
    /*!
     * LoRaMac datarates definition
     */
    #define DR_8                                        8  // SF12 - BW500 |
    #define DR_9                                        9  // SF11 - BW500 |
    #define DR_10                                       10 // SF10 - BW500 |
    #define DR_11                                       11 // SF9  - BW500 |
    #define DR_12                                       12 // SF8  - BW500 +-> Down link
    #define DR_13                                       13 // SF7  - BW500 |
    #define DR_14                                       14 // RFU          |
    #define DR_15                                       15 // RFU          |

    //#define LORAMAC_DEFAULT_DATARATE                    DR_8   // sf12
    //#define LORAMAC_DEFAULT_DATARATE                    DR_9   // sf11
    //#define LORAMAC_DEFAULT_DATARATE                    DR_10   // sf10
    //#define LORAMAC_DEFAULT_DATARATE                    DR_11   // sf9
    //#define LORAMAC_DEFAULT_DATARATE                    DR_12   // sf8
    #define LORAMAC_DEFAULT_DATARATE                    DR_13   // sf7

    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)910.0e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)800e3 )
    #define LORA_MAX_NB_CHANNELS            8

    #define BANDWIDTH_KHZ       500

    /* end USE_BAND_915 */
#elif defined(USE_BAND_433)

    #define DR_0                             0  // SF12 - BW125
    #define DR_1                             1  // SF11 - BW125
    #define DR_2                             2  // SF10 - BW125
    #define DR_3                             3  // SF9  - BW125
    #define DR_4                             4  // SF8  - BW125
    #define DR_5                             5  // SF7  - BW125
    //#define DR_6                             6  // SF7  - BW250
    //#define DR_7                             7  // FSK

    //#define LORAMAC_DEFAULT_DATARATE            DR_0   // sf12
    //#define LORAMAC_DEFAULT_DATARATE            DR_1   // sf11
    //#define LORAMAC_DEFAULT_DATARATE            DR_2   // sf10
    //#define LORAMAC_DEFAULT_DATARATE            DR_3   // sf9
    //#define LORAMAC_DEFAULT_DATARATE            DR_4   // sf8
    #define LORAMAC_DEFAULT_DATARATE            DR_5   // sf7

    #define LORAMAC_FIRST_CHANNEL           ( (uint32_t)433.32e6 )
    #define LORAMAC_STEPWIDTH_CHANNEL       ( (uint32_t)200e3 )
    #define LORA_MAX_NB_CHANNELS            7 /* last channel 434.52 */

    #define BANDWIDTH_KHZ       125
    /* end USE_BAND_433 */
#endif

#define DEVADDR_NONE        0xffffffff

#define BEACON_SIZE             6

#define LORA_EUI_LENGTH                 8
#define LORA_CYPHERKEYBYTES             16

using namespace std::chrono;

//extern Timer timer;
//extern EventQueue queue;

extern UnbufferedSerial pc;
void pc_printf(const char *fmt, ...);

#define MAC_CMD_QUEUE_SIZE      6
#define MAC_CMD_SIZE            8
typedef struct {
    uint8_t dev_eui[LORA_EUI_LENGTH];
    uint8_t app_eui[LORA_EUI_LENGTH];
    uint8_t app_key[LORA_CYPHERKEYBYTES];

    uint8_t app_session_key[LORA_CYPHERKEYBYTES];
    uint8_t network_session_key[LORA_CYPHERKEYBYTES];
    uint32_t dev_addr;

    uint16_t tx_slot_offset;

    uint8_t macCmd_queue[MAC_CMD_QUEUE_SIZE][MAC_CMD_SIZE];
    uint8_t macCmd_queue_in_idx, macCmd_queue_out_idx;

    uint8_t user_downlink_length;
    uint16_t FCntDown;
} ota_mote_t;
#define N_MOTES     8
extern ota_mote_t motes[N_MOTES];   /* from Comissioning.h */

typedef struct {
    uint8_t show_mac      : 1; // 1
    uint8_t show_app      : 1; // 2
    uint8_t do_downlink   : 1; // 3
    uint8_t beacon_guard  : 1; // 4
    uint8_t beacon_loaded : 1; // 5
    uint8_t beacon_test   : 1; // 6
} flags_t;

typedef enum
{
    MAC,
    APP,
    BOTH
} layer_e;

class LoRaWan {
    private:
        static void SendJoinComplete(uint16_t deviceNonce, uint8_t firstReceiveWindowDataRateoffset, ota_mote_t* mote);
        static void parse_mac_command(ota_mote_t* mote, uint8_t* rx_cmd_buf, uint8_t rx_cmd_buf_len);
        static void parse_uplink(ota_mote_t* mote, uint8_t rxSize);
        static void parse_join_req(ota_mote_t* mote, uint8_t rxSize);
        static void classA_downlink(ota_mote_t* mote);

    public:
        static int parse_receive(uint8_t rxSize, float rssi, float snr);
        static int init(void);
        static void print_octets_rev(char const* label, uint8_t const* buf, uint8_t buf_len);

        static volatile uint16_t rx_slot;
        static volatile uint32_t beaconDur;
        static uint8_t user_downlink[];
        static const uint8_t Datarates[];
        static HighResClock::time_point rx_at;  //static volatile uint32_t rx_ms;
        static uint32_t dev_addr_filter;
        static void filtered_printf(uint32_t dev_addr, layer_e, const char* format, ...);

        static volatile flags_t flags;
        static uint8_t _tmp_payload_length;
};


// from main.cpp:
void send_downlink(void);
void decrypted_uplink(uint32_t dev_addr, uint8_t* buf, uint8_t buflen, uint8_t port);
