#include "sx12xx.h"
#ifdef SX127x_H 
#include "sx127x_lora.h"
#include "sx127x_fsk.h"
#endif /* SX127x_H */

#define PA_OFF_DBM      -127

#define RADIO_OSC_STARTUP_us                           1000 // [ms]
#define RADIO_SLEEP_TO_RX_us                           2000 // [ms]
#define RADIO_WAKEUP_TIME_us                           ( RADIO_OSC_STARTUP_us + RADIO_SLEEP_TO_RX_us )

typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
    MODEM_FLRC,
} RadioModems_t;

/*!
 * \brief Radio driver callback functions
 */
typedef struct
{
    void (*DioPin_top_half)(void);
    /*!
     * \brief  Tx Done callback prototype.
     */
    void    (*TxDone_topHalf)(void);    // read irqAt for timestamp of interrupt
    void    (*TxDone_botHalf)(void);    // read irqAt for timestamp of interrupt
    /*!
     * \brief  Tx Timeout callback prototype.
     */
    void    ( *TxTimeout )( void );
    /*!
     * \brief Rx Done callback prototype.
     *
     * \param [IN] payload Received buffer pointer
     * \param [IN] size    Received buffer size
     * \param [IN] rssi    RSSI value computed while receiving the frame [dBm]
     * \param [IN] snr     Raw SNR value given by the radio hardware
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     * \param [IN] curTime captured time at RxDone event occurance
     */
    //void    ( *RxDone )(uint16_t size, int16_t rssi, int8_t snr);
    void    ( *RxDone )(uint8_t size, float rssi, float snr);    // read radio.rx_buf for payload, irqAt for timestamp of interrupt
    /*!
     * \brief  Rx Timeout callback prototype.
     */
    void    ( *RxTimeout )( void );
    /*!
     * \brief Rx Error callback prototype.
     */
    void    ( *RxError )( void );
    /*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param [IN] currentChannel   Index number of the current channel
     */
    void ( *FhssChangeChannel )( uint8_t currentChannel );

    /*!
     * \brief CAD Done callback prototype.
     *
     * \param [IN] channelDetected    Channel Activity detected during the CAD
     */
    void ( *CadDone ) ( bool channelActivityDetected );

} RadioEvents_t;

class Radio {
    public:
        static void SetTxContinuousWave(unsigned hz, int8_t txPower, unsigned timeout);
        static uint32_t Random(void);
        static void SetPublicNetwork(bool);
        static void Sleep(void);
        static void SetChannel(unsigned hz);
        static float getFrfMHz(void);
        static void SetRxMaxPayloadLength(uint8_t);
        static void SetFixedPayloadLength(uint8_t); // implicit mode
        static void Rx(unsigned timeout); // timeout 0 for continuous rx
        static void Standby(void);
        static bool CheckRfFrequency(unsigned hz);
        static float GetRssiInst(void);
        static void Init(const RadioEvents_t*, unsigned spiHz=1000000);
        static int Send(uint8_t size, timestamp_t maxListenTime, timestamp_t channelFreeTime, int rssiThresh);
        static void service(void);
		static uint32_t lora_toa_us(uint8_t pktLen);
    #if (MBED_MAJOR_VERSION < 6)
        static volatile us_timestamp_t irqAt;
    #else
        static LowPowerClock::time_point irqAt;
    #endif
#ifdef DUTY_ENABLE
        static us_timestamp_t TimeOnAir(RadioModems_t, uint8_t);
#endif /* DUTY_ENABLE */

        static void LoRaModemConfig(unsigned bwKHz, uint8_t sf, uint8_t cr);
        static void LoRaPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn, bool invIQ);
        static void SetLoRaSymbolTimeout(uint16_t symbs);

        static void GFSKModemConfig(unsigned bps, unsigned bwKHz, unsigned fdev_hz);
        static void GFSKPacketConfig(unsigned preambleLen, bool fixLen, bool crcOn);

        static void set_tx_dbm(int8_t dbm);

#ifdef DEVICE_LPTICKER
        static LowPowerTimer lpt;
#else
        static Timer lpt;
#endif

#ifdef SX127x_H 
        static SX127x radio;
        static SX127x_lora lora;
        static SX127x_fsk fsk;
#elif defined(SX126x_H)
    #define SetFixedPayloadLength      SetRxMaxPayloadLength
        static SX126x radio;
    #ifdef TARGET_FF_ARDUINO
        #define CHIP_TYPE_SX1262        0
        #define CHIP_TYPE_SX1261        1
        #ifndef TARGET_DISCO_L072CZ_LRWAN1
            static DigitalIn chipType;
        #endif
    #endif /* TARGET_FF_ARDUINO */
#elif defined(SX128x_H)
    #define SetFixedPayloadLength      SetRxMaxPayloadLength
        static unsigned symbolPeriodUs;
        static unsigned nSymbs;
        static unsigned rxTimeoutMs;
        static SX128x radio;
#elif defined(SX1265_H)
        static SX1265 radio;
        static void dio9_top_half(void);
        static void timeout_callback(bool);
        static void rx_done(uint8_t, float, float);
        static void txDoneBottom(void);
        static uint8_t loraTimeoutSymbols;
#else
        #error import radio driver library
#endif

    private:
        static void print_buf(const uint8_t* buf, uint8_t size, const char* txt);
        static void boardInit(void);
#ifdef SX126x_H 
        static RadioModems_t _m_;
        static void rx_done(uint8_t, float, float);
        static void txDoneBottom(void);
        static void timeout_callback(bool);
        static void dio1_top_half(void);
        static PacketParams_t pp;
        static bool paOff;
        static uint8_t loraTimeoutSymbols;
#endif /* SX126x_H */
#ifdef SX128x_H 
        static void readChip(void);
        static PacketParams_t ppGFSK, ppLORA, ppFLRC;
        static ModulationParams_t mpFLRC, mpBLE_GFSK, mpLORA;
        static void diox_top_half(void);
        static void rxDone(uint8_t size, const pktStatus_t*);
        static void txDoneBottom(void);
        static void timeout_callback(bool);
#endif /* SX128x_H */
#if defined(SX1265_H)
        static void my_irq_handler(void);
        static bool init_irq;
#endif /* SX1265_H */
        static void chipModeChange(void);
        static void rfsw_callback(void);
        static void ocp(uint8_t ma);

        static InterruptIn dio0;
        static InterruptIn dio1;
        static void dio0isr(void);
        static void dio1isr(void);
        static void dio0UserContext(void);
        static void dio1UserContext(void);
};


