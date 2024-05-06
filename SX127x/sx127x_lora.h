#include "sx12xx.h"

// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0d
#define REG_LR_FIFOTXBASEADDR                       0x0e
#define REG_LR_FIFORXBASEADDR                       0x0f
#define REG_LR_FIFORXCURRENTADDR /*REG_LR_RXDATAADDR*/  0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_RXNBBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUE_MSB                 0x14
#define REG_LR_RXHEADERCNTVALUE_LSB                 0x15
#define REG_LR_RXPACKETCNTVALUE_MSB                 0x16
#define REG_LR_RXPACKETCNTVALUE_LSB                 0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1a
#define REG_LR_RSSIVALUE                            0x1b
#define REG_LR_HOPCHANNEL                           0x1c
#define REG_LR_MODEMCONFIG                          0x1d
#define REG_LR_MODEMCONFIG2                         0x1e
#define REG_LR_SYMBTIMEOUTLSB                       0x1f
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22 // and RX length for implicit
#define REG_LR_RX_MAX_PAYLOADLENGTH                 0x23 // length limit for explicit mode
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_RXBYTEADDR /*REG_LR_RXDATAADDR*/     0x25
#define REG_LR_MODEMCONFIG3                         0x26    // sx1272 REG_LR_PPM_CORRECTION_MSB
#define REG_LR_PPM_CORRECTION_LSB                   0x27
#define REG_LR_TEST28                               0x28  // est_freq_error
#define REG_LR_TEST29                               0x29    // est_freq_error
#define REG_LR_TEST2A                               0x2a    // est_freq_error
#define REG_LR_TEST2B                               0x2b    // 
#define REG_LR_WIDEBAND_RSSI                        0x2c 
#define REG_LR_AGCH_TH                              0x2d    // agc_upper_th
#define REG_LR_AGCL_TH                              0x2e    // agc_lower_th
#define REG_LR_IFFRQH                               0x2f    // if_freq(12:8)
#define REG_LR_IFFRQL                               0x30    // if_freq(7:0)
#define REG_LR_TEST31                               0x31    // if_freq_auto, ...
#define REG_LR_TEST32                               0x32    // 
#define REG_LR_TEST33                               0x33    // invert IQ
#define REG_LR_CAD_PEAK_TO_NOISE_RATIO              0x34
#define REG_LR_CAD_MIN_PEAK                         0x35
#define REG_LR_SX1276_AUTO_DRIFT                    0x36
#define REG_LR_DETECTION_THRESHOLD                  0x37
#define REG_LR_SYNC_BYTE                            0x39    // default 0x12 (value of 0x21 will isolate network)
#define REG_LR_GAIN_DRIFT                           0x3a
#define REG_LR_DRIFT_INVERT                         0x3b  

typedef union {
    struct {    // sx127x register 0x12
        uint8_t CadDetected         : 1;    // 0
        uint8_t FhssChangeChannel   : 1;    // 1
        uint8_t CadDone             : 1;    // 2
        uint8_t TxDone              : 1;    // 3
        uint8_t ValidHeader         : 1;    // 4
        uint8_t PayloadCrcError     : 1;    // 5
        uint8_t RxDone              : 1;    // 6
        uint8_t RxTimeout           : 1;    // 7
    } bits;
    uint8_t octet;
} RegIrqFlags_t;

typedef union {
    struct {    // sx127x register 0x18
        uint8_t detect          : 1;    // 0
        uint8_t sync            : 1;    // 1
        uint8_t rx_ongoing      : 1;    // 2
        uint8_t header_valid    : 1;    // 3
        uint8_t clear           : 1;    // 4
        uint8_t RxCodingRate    : 3;    // 5,6,7
    } bits;
    uint8_t octet;
} RegModemStatus_t;

typedef union {
    struct {    // sx127x register 0x1c
        uint8_t FhssPresentChannel  : 6;    // 0,1,2,3,4,5
        uint8_t RxPayloadCrcOn      : 1;    // 6
        uint8_t PllTimeout          : 1;    // 7
    } bits;
    uint8_t octet;
} RegHopChannel_t;

typedef union {
    struct {    // sx1276 register 0x1d
        uint8_t ImplicitHeaderModeOn    : 1;    // 0
        uint8_t CodingRate              : 3;    // 1,2,3
        uint8_t Bw                      : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1d
        uint8_t LowDataRateOptimize     : 1;    // 0  ppm_offset: number of cyclic shifts possible to encode to symbol
        uint8_t RxPayloadCrcOn          : 1;    // 1
        uint8_t ImplicitHeaderModeOn    : 1;    // 2
        uint8_t CodingRate              : 3;    // 3,4,5
        uint8_t Bw                      : 2;    // 6,7
    } sx1272bits;
    uint8_t octet;
} RegModemConfig_t;

typedef union {
    struct {    // sx1276 register 0x1e
        uint8_t SymbTimeoutMsb          : 2;    // 0,1
        uint8_t RxPayloadCrcOn          : 1;    // 2
        uint8_t TxContinuousMode        : 1;    // 3
        uint8_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1276bits;
    struct {    // sx1272 register 0x1e
        uint8_t SymbTimeoutMsb          : 2;    // 0,1
        uint8_t AgcAutoOn               : 1;    // 2
        uint8_t TxContinuousMode        : 1;    // 3
        uint8_t SpreadingFactor         : 4;    // 4,5,6,7
    } sx1272bits;
    uint8_t octet;
} RegModemConfig2_t;

typedef union {
    struct {    // sx127x register 0x26
        uint8_t reserved    : 2;    // 0,1
        uint8_t AgcAutoOn   : 1;    // 2
        uint8_t LowDataRateOptimize  : 1;    // 3   ppm_offset, use when symbol duration exceeds 16ms
        uint8_t unused      : 4;    // 4,5,6,7 
    } sx1276bits;
    uint8_t octet;
    uint8_t sx1272_ppm_correction_msb;
} RegModemConfig3_t;


typedef union {
    struct {    // sx127x register 0x31
        uint8_t detect_trig_same_peaks_nb  : 3;    // 0,1,2
        uint8_t disable_pll_timeout        : 1;    // 3
        uint8_t tracking_intergral         : 2;    // 4,5
        uint8_t frame_sync_gain            : 1;    // 6
        uint8_t if_freq_auto               : 1;    // 7
    } bits;
    uint8_t octet;
} RegTest31_t;

typedef union {
    struct {    // sx127x register 0x33
        uint8_t chirp_invert_tx    : 1;    // 0  invert TX spreading sequence  (default=1)
        uint8_t chirp_invert_rx    : 1;    // 1  invert chip direction in RX mode  (default=1)
        uint8_t sync_detect_th     : 1;    // 2  require 6dB despread SNR during preamble
        uint8_t invert_coef_phase  : 1;    // 3  
        uint8_t invert_coef_amp    : 1;    // 4
        uint8_t quad_correction_en : 1;    // 5  enable IQ compensation
        uint8_t invert_i_q         : 1;    // 6  RX invert (default=0)
        uint8_t start_rambist      : 1;    // 7
    } bits;
    uint8_t octet;
} RegTest33_t;

typedef union {
    struct {    // sx1276 register 0x36
        uint8_t freq_to_time_drift_auto         : 1;    // 0  manual control of 0x3a register (1=auto)
        uint8_t sd_max_freq_deviation_auto      : 1;    // 1  manual control of 0x3b[3:1] (1=auto)
        uint8_t reserved                        : 6;    // 
    } bits;
    uint8_t octet;
} RegAutoDrift_t;

typedef union {
    struct {    // sx127x register 0x3a
        uint8_t freq_to_time_drift         : 6;    // 0,1,2,3,4,5  manual control of 0x3a register
        uint8_t preamble_timing_gain       : 1;    // 6,7
    } bits;
    uint8_t octet;
} RegGainDrift_t;

typedef union {
    struct {    // sx127x register 0x3b
        uint8_t coarse_sync                     : 1;    // 0  must be set to 1
        uint8_t fine_sync                       : 1;    // 1  must be clr to 0
        uint8_t invert_timing_error_per_symbol  : 1;    // 2  set to !invert_i_q
        uint8_t invert_freq_error               : 1;    // 3  
        uint8_t invert_delta_sampling           : 1;    // 4    must be set to 1
        uint8_t reserved                        : 1;    // 5    must be clr to 0
        uint8_t invert_fast_timing              : 1;    // 6 
        uint8_t invert_carry_in                 : 1;    // 7
    } bits;
    uint8_t octet;
} RegDriftInvert_t;


//class SX127x_lora : public SX127x
class SX127x_lora {
    public:
        //SX127x_lora(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName rst, PinName dio_0, PinName dio_1, PinName fem_ctx, PinName fem_cps);
        SX127x_lora(SX127x& r);
        
        ~SX127x_lora();
        
        /** changes from FSK mode to LoRa mdoe */
        void enable(void);
        
        /** fills radio FIFO with payload contents, prior to transmission
         * @param len count of bytes to put into FIFO
         * @note tx_buf[] should contain desired payload (to send) prior to calling
         */
        void write_fifo(uint8_t len);     
        
        /** transmit a packet
         * @param len size of packet
         * @note Limited to (lora fifo size 256)
         */
        void start_tx(uint8_t len);
        
        /** start continuous receive mode
         * @param mode RF_OPMODE_RECEIVER or RF_OPMODE_RECEIVER_SINGLE
         * @note the variable service_action needs to be monitored to indicate read_fifo() needs to be called to pull packet from FIFO.
         */
        void start_rx(chip_mode_e mode);
        
        /** Called by main program when indicated by service_action variable, to pull recevied packet from radio FIFO.
         * @returns count of bytes received
         * @note received packet in rx_buf[]
         */
        void read_fifo(uint8_t len);
        
        /** CodingRate: how much FEC to encoding onto packet */
        uint8_t getCodingRate(bool from_rx);    // false:transmitted, true:last recevied packet
        void setCodingRate(uint8_t cr);
        
        /** HeaderMode: explicit mode sents CodingRate and payload length, implicit mode requires assumption by receiver */
        bool getHeaderMode(void);
        void setHeaderMode(bool hm);

        /** bandwidth: SX1272 has three bandwidths. SX1276 adds more narrower bandwidths. */
        uint8_t getBw(void);
        void setBw(uint8_t bw);
        
        /** Set bandwidth in KHz *
         * @param khz lora bandwidth in KHz
         */        
        void setBw_KHz(int khz);
        
        /** spreading factor: trade-off between data rate and processing gain (link budget) */
        uint8_t getSf(void);
        void setSf(uint8_t sf);        
        
        /** enable CRC in transmitted packet */
        bool getRxPayloadCrcOn(void);
        void setRxPayloadCrcOn(bool);
        
        bool getAgcAutoOn(void);
        void setAgcAutoOn(bool);
        
        int get_pkt_rssi(void);
        int get_current_rssi(void);
        
        /** retrieve symbol duration from bandwidth and spreading factor
         * @returns symbol duration in milliseconds
         */        
        float get_symbol_period(void);
        
        service_action_e service(void); // (SLIH) ISR bottom half 
        
        bool poll_vh;
        
        void set_nb_trig_peaks(int);
        
        /** get receiver difference in RF frequency 
          * @returns hertz
          * @note if receiver is lower in frequency than transmitter, a negative Hz will be returned
          */
        int get_freq_error_Hz(void);
        
        
        /** invert transmitted spectrum */
        void invert_tx(bool);
        
        /** invert spectrum on receiver */
        void invert_rx(bool);
        
        RegIrqFlags_t       RegIrqFlags;            // 0x12
        uint8_t             RegRxNbBytes;           // 0x13
        RegModemStatus_t    RegModemStatus;         // 0x18
        int8_t              RegPktSnrValue;         // 0x19  signed, s/n can be negative
        uint8_t             RegPktRssiValue;        // 0x1a
        RegHopChannel_t     RegHopChannel;          // 0x1c
        RegModemConfig_t    RegModemConfig;         // 0x1d
        RegModemConfig2_t   RegModemConfig2;        // 0x1e
        uint16_t            RegPreamble;            // 0x20->0x21
        uint8_t             RegPayloadLength;       // 0x22
        uint8_t             RegRxMaxPayloadLength;  // 0x23
        uint8_t             RegHopPeriod;           // 0x24
        RegModemConfig3_t   RegModemConfig3;        // 0x26
        RegTest31_t         RegTest31;              // 0x31
        RegTest33_t         RegTest33;              // 0x33
        RegAutoDrift_t      RegAutoDrift;           // 0x36  sx1276 only
        RegGainDrift_t      RegGainDrift;           // 0x3a
        RegDriftInvert_t    RegDriftInvert;         // 0x3b
        
        
        SX127x& m_xcvr;

    private:
                                                                 
};
