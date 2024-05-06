#include "sx12xx.h"

#define REG_FSK_BITRATEMSB                            0x02
#define REG_FSK_BITRATELSB                            0x03
#define REG_FSK_FDEVMSB                              0x04 
#define REG_FSK_FDEVLSB                              0x05

#define REG_FSK_RXCONFIG                                0x0D
#define REG_FSK_RSSICONFIG                            0x0E
#define REG_FSK_RSSICOLLISION                          0x0F // rssi delta threshold (interferer)
#define REG_FSK_RSSITHRESH                            0x10  // trigger level for rssi interrupt
#define REG_FSK_RSSIVALUE                              0x11
#define REG_FSK_RXBW                                    0x12 
#define REG_FSK_AFCBW                                  0x13
#define REG_FSK_OOKPEAK                              0x14   // bitsync config
#define REG_FSK_OOKFIX                                0x15  // threshold dB
#define REG_FSK_OOKAVG                                0x16  
#define REG_FSK_RES17                                  0x17 // barker test
#define REG_FSK_RES18                                  0x18 // barker test
#define REG_FSK_RES19                                  0x19 // barker test
#define REG_FSK_AFCFEI                                0x1A
#define REG_FSK_AFCMSB                                0x1B
#define REG_FSK_AFCLSB                                0x1C
#define REG_FSK_FEIMSB                                0x1D
#define REG_FSK_FEILSB                                0x1E
#define REG_FSK_PREAMBLEDETECT                        0x1F
#define REG_FSK_RXTIMEOUT1                            0x20  // rssi timeout
#define REG_FSK_RXTIMEOUT2                            0x21  // preamble detect timeout
#define REG_FSK_RXTIMEOUT3                            0x22  // sync detect timeout
#define REG_FSK_RXDELAY                              0x23   // RX restart delay
// Oscillator settings
#define REG_FSK_OSC                                  0x24   // clkout output divider
// Packet handler settings
#define REG_FSK_PREAMBLEMSB                          0x25   // preamble length
#define REG_FSK_PREAMBLELSB                          0x26   // preamble length
#define REG_FSK_SYNCCONFIG                            0x27
#define REG_FSK_SYNCVALUE1                            0x28
#define REG_FSK_SYNCVALUE2                            0x29
#define REG_FSK_SYNCVALUE3                            0x2A
#define REG_FSK_SYNCVALUE4                            0x2B
#define REG_FSK_SYNCVALUE5                            0x2C
#define REG_FSK_SYNCVALUE6                            0x2D
#define REG_FSK_SYNCVALUE7                            0x2E
#define REG_FSK_SYNCVALUE8                            0x2F
#define REG_FSK_PACKETCONFIG1                          0x30
#define REG_FSK_PACKETCONFIG2                          0x31
#define REG_FSK_PAYLOADLENGTH                          0x32
#define REG_FSK_NODEADRS                                0x33
#define REG_FSK_BROADCASTADRS                          0x34
#define REG_FSK_FIFOTHRESH                            0x35
// SM settings
#define REG_FSK_SEQCONFIG1                            0x36
#define REG_FSK_SEQCONFIG2                            0x37
#define REG_FSK_TIMERRESOL                            0x38
#define REG_FSK_TIMER1COEF                            0x39  // period of timer1 interrupt
#define REG_FSK_TIMER2COEF                            0x3A  // period of timer2 interrupt
// Service settings
#define REG_FSK_IMAGECAL                                0x3B
#define REG_FSK_TEMP                                    0x3C
#define REG_FSK_LOWBAT                                0x3D  // EOL "end of life"
// Status
#define REG_FSK_IRQFLAGS1                              0x3E
#define REG_FSK_IRQFLAGS2                              0x3F // packet flags

/******************************************************************************/

#define FSK_FIFO_SIZE       64
#define FSK_FIFO_SIZE_HALF  (FSK_FIFO_SIZE>>1)

typedef union {
    struct {    // sx1272 register 0x0d
        //uint8_t wait_rssi_irq        : 1; // 0 wait for signal strength before entering RX
        //uint8_t wait_irq_0x55        : 1; // 1 wait for preamble before entering RX
        //uint8_t agc_on_irq_0x55        : 1;   // 2 1=LNA gain adj done until irq_0x55 asserted
        uint8_t RxTrigger              : 3; // 0,1,2: 0=none 1=rssiInt 6=preambleDet 7=both
        uint8_t AgcAutoOn              : 1; // 3
        uint8_t AfcAutoOn              : 1; // 4
        uint8_t RestartRxWithPllLock    : 1;    // 5        restart from FSRX mode
        uint8_t RestartRxWithoutPllLock : 1;    // 6
        uint8_t RestartRxOnCollision    : 1;    // 7
    } bits;
    uint8_t octet;
} RegRxConfig_t;


typedef union {
    struct {    // sx1272 register 0x1a
        uint8_t AfcAutoClearOn  : 1;    // 0
        uint8_t AfcClear        : 1;    // 1    manual clear
        uint8_t unused1      : 1;   // 2
        uint8_t fei_range      : 1; // 3    FEI range limited by: 0=rxbw    1=fs/2
        uint8_t AgcStart        : 1;    // 4    manual trigger AGC
        uint8_t unused        : 3;  // 5,6,7 
    } bits;
    uint8_t octet;
} RegAfcFei_t;


typedef union {
    struct {    // sx1272 register 0x0e
        uint8_t RssiSmoothing   : 3;    // 0,1,2
        uint8_t RssiOffset      : 5;    // 3,4,5,6,7
    } bits;
    uint8_t octet;
} RegRssiConfig_t;

typedef union {
    struct {    // sx1272 register 0x12
        //uint8_t RxBw          : 5;    // 0,1,2,3,4        (0,1,2=exp   3,4=mant)
        uint8_t Exponent        : 3;    // 0,1,2
        uint8_t Mantissa        : 2;    // 3,4
        uint8_t dcc_force      : 1; // 5 force dcc on all rxbw (otherwise put only if > 167KHz)
        uint8_t dcc_fast_init   : 1;    // 6 
        uint8_t reserved        : 1;    // 7 
    } bits;
    uint8_t octet;
} RegRxBw_t;

typedef union {
    struct {    // sx1272 register 0x14
        uint8_t OokPeakThreshStep   : 3;    // 0,1,2
        uint8_t OokThreshType      : 2; // 3,4
        uint8_t BitSyncOn          : 1; // 5
        uint8_t barker_en          : 1; // 6
        uint8_t bsync_opt          : 1; // 7    not used
    } bits;
    uint8_t octet;
} RegOokPeak_t; // DEMOD1 0x14

typedef union {
    struct {    // sx1272 register 0x16
        uint8_t OokAverageThreshFilt : 2; // 0,1
        uint8_t OokAverageOffset     : 2; // 2,3
        uint8_t reserved             : 1; // 4
        uint8_t OokPeakThreshDec     : 3; // 5,6,7
    } bits;
    uint8_t octet;
} RegOokAvg_t; // at 0x16

typedef union {
    struct {    // sx1272 register 0x1f
        uint8_t PreambleDetectorTol  : 5;   // 0,1,2,3,4    allowed chip errors
        uint8_t PreambleDetectorSize    : 2;    // 5,6    00b=1bytes... 11b=4bytes
        uint8_t PreambleDetectorOn    : 1;  // 7
    } bits;
    uint8_t octet;
} RegPreambleDetect_t;

typedef union {
    struct {    // sx127x register 0x24
        uint8_t ClkOut         : 3;   // 0,1,2
        uint8_t rc_cal_trigger : 1;    // 3
        uint8_t unused         : 4;  // 4,5,6,7
    } bits;
    uint8_t octet;
} RegOsc_t;

typedef union {
    struct {    // sx1232 register 0x27
        uint8_t SyncSize            : 3;    // 0,1,2
        uint8_t FifoFillCondition   : 1;    // 3    rx fifo fill starting 0=start-on-sync
        uint8_t SyncOn            : 1;  // 4    enable pattern recognition
        uint8_t PreamblePolarity    : 1;    // 5    0=0xaa 1=0x55
        uint8_t AutoRestartRxMode   : 2;    // 6,7  00b=do not restart 10b=wait-for-pll
    } bits;
    uint8_t octet;
} RegSyncConfig_t;

typedef union {
    struct {    // sx1232 register 0x30
        uint8_t CrCWhiteningType : 1;   // 0       1=IBM-crc   0=ccitt-crc
        uint8_t AddressFiltering : 2;   // 1,2   11b = two-byte nodeadrs at 0x2c->0x2f
        uint8_t CrcAutoClearOff  : 1;   // 3
        uint8_t CrcOn           : 1;    // 4
        uint8_t DcFree         : 2; // 5,6 
        uint8_t PacketFormatVariable : 1;   // 7       1=variable length, 0=fixed
    } bits;
    uint8_t octet;
} RegPktConfig1_t;

typedef union {
    struct {    // sx1272 register 0x31 and 0x32
        uint16_t PayloadLength      : 11;   // 0->10
        uint16_t BeaconOn           : 1;    // 11 
        uint16_t IoHomePowerFrame   : 1;    // 12   CRC LFSR init: 0=0x1d0f, 1=0x0000=powerlink
        uint16_t IoHomeOn           : 1;    // 13
        uint16_t DataModePacket     : 1;    // 14   1=packet mode, 0=continuous mode
        uint16_t unused             : 1;    // 15 
    } bits;
    uint16_t word;
} RegPktConfig2_t;

typedef union {
    struct {    // sx1272 register 0x35
        uint8_t FifoThreshold      : 6; // 0,1,2,3,4,5
        uint8_t unused            : 1;  // 6 
        uint8_t TxStartCondition    : 1;    // 7        0=fifoThresh 1=fifoNotEmpty
    } bits;
    uint8_t octet;
} RegFifoThreshold_t;

typedef union {
    struct {    // sx1272 register 0x36
        uint8_t FromTransmit        : 1;    // 0
        uint8_t FromIdle            : 1;    // 1
        uint8_t LowPowerSelection   : 1;    // 2
        uint8_t FromStart          : 2; // 3,4
        uint8_t IdleMode            : 1;    // 5 
        uint8_t SequencerStop      : 1; // 6 
        uint8_t SequencerStart    : 1;  // 7
    } bits;
    uint8_t octet;
} RegSeqConfig1_t;   // @0x36

typedef union {
    struct {    // sx1272 register 0x37
        uint8_t FromPacketReceived  : 3;    // 0,1,2
        uint8_t FromRxTimeout      : 2; // 3,4
        uint8_t FromReceive      : 3;   // 5,6,7
    } bits;
    uint8_t octet;
} RegSeqConfig2_t;   // @0x37

typedef union {
    struct {    // sx1272 register 0x38
        uint8_t timer2_resol   : 2; // 0,1
        uint8_t timer1_resol   : 2; // 2,3
        uint8_t force_hlm_irq  : 1; // 4 
        uint8_t hlm_started : 1;    // 5 
        uint8_t unused       : 2;   // 6,7
    } bits;
    uint8_t octet;
} RegTimerResol_t;   // HL42 @0x38

typedef union {
    struct {    // sx1272 register 0x3b
        uint8_t TempMonitorOff  : 1;    // 0
        uint8_t TempThreshold   : 2;    // 1,2
        uint8_t TempChange      : 1;    // 3    read-only
        uint8_t unused          : 1;    // 4 
        uint8_t ImageCalRunning : 1;    // 5    read-only
        uint8_t ImageCalStart   : 1;    // 6    write-only
        uint8_t AutoImageCalOn  : 1;    // 7
    } bits;
    uint8_t octet;
} RegImageCal_t;   // 

typedef union {
    struct {    // sx1232 register 0x3e
        uint8_t SyncAddressMatch    : 1;    // 0 
        uint8_t PreambleDetect    : 1;  // 1 
        uint8_t Timeout          : 1;   // 2    rx-timeout
        uint8_t Rssi                : 1;    // 3 
        uint8_t PllLock          : 1;   // 4 
        uint8_t TxReady          : 1;   // 5 
        uint8_t RxReady          : 1;   // 6 
        uint8_t ModeReady          : 1; // 7 
    } bits;
    uint8_t octet;
} RegIrqFlags1_t;   // STAT0

typedef union {
    struct {    // sx1232 register 0x3f
        uint8_t LowBat        : 1;  // 0    "eol"
        uint8_t CrcOk          : 1; // 1 
        uint8_t PayloadReady    : 1;    // 2 
        uint8_t PacketSent    : 1;  // 3 
        uint8_t FifoOverrun  : 1;   // 4 
        uint8_t FifoLevel      : 1; // 5 
        uint8_t FifoEmpty      : 1; // 6 
        uint8_t FifoFull        : 1;    // 7 
    } bits;
    uint8_t octet;
} RegIrqFlags2_t;   // STAT1 @0x3f

//class SX127x_fsk : public SX127x
class SX127x_fsk {
    public:
        //SX127x_fsk(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName rst, PinName dio_0, PinName dio_1, PinName fem_ctx, PinName fem_cps);
        SX127x_fsk(SX127x& r);
        
        ~SX127x_fsk();
        
        /** switches from LoRa mode to FSK mdoe
         * before SX127x_fsk can be used, eanble() must be called.  LoRa mode is unavailable while FSK is in use.
         * @param fast true=bypass reading FSK registers after mode switch */
        void enable(bool fast);
        
        /** put FSK modem to some functioning default */
        void init(void);
        
        /** fills radio FIFO with payload contents, prior to transmission
         * @param len count of bytes to put into FIFO
         * @note tx_buf[] should contain desired payload (to send) prior to calling
         */
        void write_fifo(uint8_t len);        
        
        void start_tx(uint16_t len);
        
        void start_rx(void);
        uint8_t rx_buf_length;
        void config_dio0_for_pktmode_rx(void);
        
        uint32_t get_rx_bw_hz(uint8_t addr);
        
        /** bw_hz: single side (ssb) */
        void set_rx_dcc_bw_hz(uint32_t bw_hz, char afc);
        
        uint32_t get_bitrate(void);
        void set_bitrate(uint32_t);

        uint32_t get_tx_fdev_hz(void);
        void set_tx_fdev_hz(uint32_t);
        
        uint8_t get_modulation_shaping(void);
        void set_modulation_shaping(uint8_t);
        
        service_action_e service(void); // (SLIH) ISR bottom half 
        
        RegRxConfig_t        RegRxConfig;         // 0x0d
        RegRssiConfig_t      RegRssiConfig;       // 0x0e
        uint8_t              RegRssiThresh;       // 0x10
        RegRxBw_t            RegRxBw;             // 0x12
        RegRxBw_t            RegAfcBw;            // 0x13
        RegOokPeak_t         RegOokPeak;          // 0x14
        RegAfcFei_t          RegAfcFei;           // 0x1a
        int16_t              RegAfcValue;         // 0x1c
        RegPreambleDetect_t  RegPreambleDetect;   // 0x1f
        RegSyncConfig_t      RegSyncConfig;       // 0x27
        RegPktConfig1_t      RegPktConfig1;       // 0x30
        RegPktConfig2_t      RegPktConfig2;       // 0x31 -> 0x32
        RegFifoThreshold_t   RegFifoThreshold;    // 0x35
        RegSeqConfig1_t      RegSeqConfig1;       // 0x36
        RegSeqConfig2_t      RegSeqConfig2;       // 0x37
        RegTimerResol_t      RegTimerResol;       // 0x38
        RegImageCal_t        RegImageCal;         // 0x3b
        
        bool tx_done_sleep; // false:go to standby after tx done, true:sleep
        
        SX127x& m_xcvr;
        
    private:
        unsigned int bit_period_us;
        uint32_t ComputeRxBw( uint8_t mantisse, uint8_t exponent );
        void ComputeRxBwMantExp( uint32_t rxBwValue, uint8_t* mantisse, uint8_t* exponent );     
                   
};

