/* SX127x driver
 * Copyright (c) 2013 Semtech
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
  
#ifndef SX127x_H
#define SX127x_H

#include "mbed.h"

#define XTAL_FREQ   32000000

#define FREQ_STEP_MHZ     61.03515625e-6    // 32 / (2^19)
#define FREQ_STEP_KHZ     61.03515625e-3    // 32e3 / (2^19)
#define FREQ_STEP_HZ      61.03515625       // 32e6 / (2^19)

#define MHZ_TO_FRF(m)   (m / FREQ_STEP_MHZ)

/******************************************************************************/
/*!
 * SX127x Internal registers Address
 */
#define REG_FIFO                                    0x00
#define REG_OPMODE                                  0x01
#define REG_FRFMSB                                  0x06
#define REG_FRFMID                                  0x07
#define REG_FRFLSB                                  0x08
// Tx settings
#define REG_PACONFIG                                0x09
#define REG_PARAMP                                  0x0A
#define REG_OCP                                     0x0B 
// Rx settings
#define REG_LNA                                     0x0C


/***** registers above 0x40 are same as FSK/OOK page */

#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
#define REG_VERSION                                 0x42

#define REG_PATEST_SX1276                           0x44
#define REG_PATEST_SX1272                           0x4b
#define REG_PDSTRIM1_SX1276                         0x4d
#define REG_PDSTRIM1_SX1272                         0x5a
#define REG_PLL_SX1272                              0x5c    // RX PLL bandwidth
#define REG_PLL_LOWPN_SX1272                        0x5e
#define REG_PLL_SX1276                              0x70 
#define REG_BSYNCTST2                               0x67
/******************************************************************************/


typedef enum {
    RF_OPMODE_SLEEP = 0,
    RF_OPMODE_STANDBY,          // 1
    RF_OPMODE_SYNTHESIZER_TX,   // 2
    RF_OPMODE_TRANSMITTER,      // 3
    RF_OPMODE_SYNTHESIZER_RX,   // 4
    RF_OPMODE_RECEIVER,         // 5
    RF_OPMODE_RECEIVER_SINGLE,  // 6
    RF_OPMODE_CAD               // 7
} chip_mode_e;

typedef enum {
    SX_NONE = 0,
    SX1272,
    SX1276
} type_e;

typedef enum {
    SERVICE_NONE = 0,
    SERVICE_ERROR,
    //! request to call read_fifo()
    SERVICE_READ_FIFO,
    //! notification to application of transmit complete
    SERVICE_TX_DONE
} service_action_e;

/******************************************************************************/

typedef union {
    struct {    // sx1272 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t ModulationShaping   : 2;    // 3,4  FSK/OOK
        uint8_t ModulationType      : 2;    // 5,6  FSK/OOK
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } bits;
    struct {    // sx1276 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t LowFrequencyModeOn  : 1;    // 3    1=access to LF test registers (0=HF regs)
        uint8_t reserved            : 1;    // 4
        uint8_t ModulationType      : 2;    // 5,6  FSK/OOK
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } sx1276FSKbits;
    struct {    // sx1276 register 0x01
        uint8_t Mode                : 3;    // 0,1,2
        uint8_t LowFrequencyModeOn  : 1;    // 3    1=access to LF test registers (0=HF regs)
        uint8_t reserved            : 2;    // 4,5
        uint8_t AccessSharedReg     : 1;    // 6    1=FSK registers while in LoRa mode
        uint8_t LongRangeMode       : 1;    // 7    change this bit only in sleep mode
    } sx1276LORAbits;
    uint8_t octet;
} RegOpMode_t;

typedef union {
    struct {    // sx12xx register 0x09
        uint8_t OutputPower : 4;    // 0,1,2,3
        uint8_t MaxPower    : 3;    // 4,5,6
        uint8_t PaSelect    : 1;    // 7        1=PA_BOOST
    } bits;
    uint8_t octet;
} RegPaConfig_t;

typedef union {
    struct {    // sx12xx register 0x0b
        uint8_t OcpTrim : 5;    // 0,1,2,3,4
        uint8_t OcpOn   : 1;    // 5
        uint8_t unused  : 2;    // 6,7
    } bits;
    uint8_t octet;
} RegOcp_t;

typedef union {
    struct {    // sx12xx register 0x0c
        uint8_t LnaBoostHF           : 2;    // 0,1
        uint8_t reserved             : 1;    // 2
        uint8_t LnaBoostLF           : 2;    // 3,4
        uint8_t LnaGain              : 3;    // 5,6,7
    } bits;
    uint8_t octet;
} RegLna_t; // RXFE

typedef union {
    struct {    // sx127x register 0x0a
        uint8_t PaRamp             : 4;    // 0,1,2,3
        uint8_t LowPnTxPllOff      : 1;    // 4        sx1272 only
        uint8_t ModulationShaping  : 2;    // 5,6      sx1276 only
        uint8_t unused             : 1;    // 7
    } bits;
    uint8_t octet;
} RegPaRamp_t; //

/*********************** ****************************/

typedef union {
    struct {    // sx12xx register 0x40
        uint8_t Dio3Mapping     : 2;    // 0,1
        uint8_t Dio2Mapping     : 2;    // 2,3
        uint8_t Dio1Mapping     : 2;    // 4,5
        uint8_t Dio0Mapping     : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping1_t;

typedef union {
    struct {    // sx12xx register 0x41
        uint8_t MapPreambleDetect : 1;    // 0      //DIO4 assign: 1b=preambleDet 0b=rssiThresh
        uint8_t io_mode           : 3;    // 1,2,3  //0=normal,1=debug,2=fpga,3=pll_tx,4=pll_rx,5=analog
        uint8_t Dio5Mapping       : 2;    // 4,5
        uint8_t Dio4Mapping       : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegDioMapping2_t;

/***************************************************/

typedef union {
    struct {    // sx1272 register 0x5a (sx1276 0x4d)
        uint8_t prog_txdac             : 3;    // 0,1,2     BGR ref current to PA DAC
        uint8_t pds_analog_test        : 1;    // 3      
        uint8_t pds_pa_test            : 2;    // 4,5
        uint8_t pds_ptat               : 2;    // 6,7     leave at 2 (5uA)
    } bits;
    uint8_t octet;
} RegPdsTrim1_t;

typedef union {
    struct {    // sx1272 register 0x5c
        uint8_t reserved           : 6;    // 0->5
        uint8_t PllBandwidth       : 2;    // 6,7 
    } bits;
    uint8_t octet;
} RegPll_t;

typedef union {
    struct {    // sx1272 register 0x67
        uint8_t bsync_mode              : 3;    // 0,1,2
        uint8_t reserved                : 1;    // 3
        uint8_t bsync_thresh_validity   : 1;    // 4
        uint8_t unused                  : 3;    // 5,6,7 
    } bits;
    uint8_t octet;
} RegBsyncTest2_t;

/** FSK/LoRa radio transceiver.
 * see http://en.wikipedia.org/wiki/Chirp_spread_spectrum
 */

class SX127x {
    public:
            /** Create SX127x instance
         * @param mosi SPI master-out pin
         * @param miso SPI master-in pin
         * @param sclk SPI clock pin
         * @param cs SPI chip-select pin
         * @param rst radio hardware reset pin
         * @param dio_0 interrupt pin from radio
         * @param fem_ctx rx-tx switch for HF bands (800/900)
         * @param fem_cps rx-tx switch for LF bands (vhf/433)
         */
         
        SX127x(PinName dio0, PinName dio_1, PinName cs, SPI&, PinName rst);
        
        ~SX127x();
        
        /** set center operating frequency
         * @param MHz operating frequency in MHz
         */
        void set_frf_MHz( float MHz );
        
        /** get center operating frequency
         * @returns operating frequency in MHz
         */
        float get_frf_MHz(void);

        void set_opmode(chip_mode_e mode);
        
        /** reset radio using pin
         */
        void hw_reset(void);
        /** initialise SX1232 class to radio
         * @note this is called from class instantiation, but must also be manually called after hardware reset
         */
        void init(void);
        void get_type(void); // identify radio chip
        
        /** read register from radio
         * @param addr register address
         * @returns the value read from the register
         */
        uint8_t read_reg(uint8_t addr);
        uint16_t read_u16(uint8_t addr);
        int16_t read_s16(uint8_t addr);
        
        /** read register from radio. from an arbitrary amount of registers following the first
         * @param addr register address
         * @param buffer the read values will be placed here
         * @param size how many registers to read
         */        
        void ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
        
        /** write register to radio
         * @param addr register address
         * @param data byte to write
         */
        void write_reg(uint8_t addr, uint8_t data);
        void write_u16(uint8_t addr, uint16_t data);
        void write_u24(uint8_t addr, uint32_t data);
        
        /** write register(s) to radio, to an arbitrary amount of registers following first
         * @param addr register address
         * @param buffer byte(s) to write
         * @param size count of registers to write to
         */
        void WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
        
        /* *switch between FSK or LoRa modes */
        //void SetLoRaOn(bool);
        
        /*****************************************************/
        
        //! RF transmit packet buffer
        uint8_t tx_buf[256];    // lora fifo size
        
        //! RF receive packet buffer
        uint8_t rx_buf[256];    // lora fifo size
       
        //! radio chip type plugged in
        type_e type;
        
        //! operating mode
        RegOpMode_t RegOpMode;
        
        //! transmitter power configuration
        RegPaConfig_t RegPaConfig;
        
        RegOcp_t RegOcp;            // 0x0b
        
        // receiver front-end
        RegLna_t RegLna;            // 0x0c
        
        //! pin assignments
        RegDioMapping1_t RegDioMapping1;
        
        //! pin assignments
        RegDioMapping2_t RegDioMapping2;
               
        DigitalIn dio0;
        DigitalIn dio1;
        DigitalOut m_cs;
        SPI& m_spi;
        bool HF;    // sx1272 is always HF   

        /*! board-specific RF switch callback, called whenever operating mode is changed
         * This function should also set RegPaConfig.bits.PaSelect to use PA_BOOST or RFO during TX.
         * examples:
         *      PE4259-63: controlled directly by radio chip, no software function needed
         *      SKY13350-385LF: two separate control lines, requires two DigitalOut pins
         */
        Callback<void()> rf_switch;
         
    private:    
        DigitalInOut reset_pin;        
        
    protected:
        
};

#endif /* SX127x_H */
