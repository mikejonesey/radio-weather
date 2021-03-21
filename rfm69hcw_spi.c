//
// Created by mike on 06/03/2021.
//

#include <hardware/gpio.h>
#include <pico/printf.h>
#include <string.h>
#include "hardware/spi.h"
#include "rfm69hcw_spi.h"
#include "common.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_DIO0 22
#define PIN_RESET 21

uint32_t now(){
    absolute_time_t t = get_absolute_time();
    uint32_t nowret = to_ms_since_boot(t);
    return nowret;
}

static inline void cs_select()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void RFM69_writeReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg | 0x80;
    buf[1] = data;
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
    sleep_ms(1);
}

static uint8_t RFM69_readReg(uint8_t addr)
{
    uint8_t buf[1];
    //addr &= 0x7F;
    cs_select();
    spi_write_blocking(SPI_PORT, &addr, 1);
    sleep_ms(1);
    spi_read_blocking(SPI_PORT, 0, buf, 1);
    cs_deselect();
    sleep_ms(1);
    return buf[0];
}

static int read_registers(uint8_t reg, uint8_t *readBuff, uint16_t len) {
    int PIC_READ, PIC_WRITE;
    // Poke Registry to Read
    cs_select();
    PIC_WRITE = spi_write_blocking(SPI_PORT, &reg, 1);
    if(PIC_WRITE == 0){
        // no write, bad address or device not present
        report_error();
        return 1;
    }
    sleep_ms(10);
    // READ
    PIC_READ = spi_read_blocking(SPI_PORT,0, readBuff, len);
    if(PIC_READ != len || PIC_READ == PICO_ERROR_GENERIC) {
        // no data returned, or bad address, or device not present
        report_error();
        return 1;
    }
    cs_deselect();
    sleep_ms(10);
    return 0;
}

//////////////////////////////////////////////////
// IRQ 1 Management
//////////////////////////////////////////////////
bool getModeReady(){
    return (0b10000000 & RFM69_readReg(RFM69_REG_IRQFLAGS1)) >>7;
}
bool getRxReady(){
    return (0b01000000 & RFM69_readReg(RFM69_REG_IRQFLAGS1)) >>6;
}
bool getTxReady(){
    return (0b00100000 & RFM69_readReg(RFM69_REG_IRQFLAGS1)) >>5;
}

void waitReadyIrq1(bool waitMode, bool waitRx, bool waitTx){
    uint16_t timeout;

    // Wait on ModeReady
    if(waitMode){
        timeout = now()+500;
        while (getModeReady() != 1){
            if(now() > timeout){
                printf("timeout waiting on ready mode...\n");
                break;
            }
            sleep_ms(100);
        } // wait for ModeReady
    }

    // Wait on RxReady
    if(waitRx){
        timeout = now()+500;
        while (getRxReady() != 1){
            if(now() > timeout){
                printf("timeout waiting on RxReady...\n");
                break;
            }
            sleep_ms(100);
        } // wait for RxReady
    }

    // Wait on TxReady
    if(waitTx){
        timeout = now()+500;
        while (getTxReady() != 1){
            if(now() > timeout){
                printf("timeout waiting on TxReady...\n");
                break;
            }
            sleep_ms(100);
        } // wait for TxReady
    }
}

//////////////////////////////////////////////////
// IRQ 2 Management
//////////////////////////////////////////////////
bool getFifoFull(){
    return (0b10000000 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>7;
}
bool getFifoNotEmpty(){
    return (0b01000000 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>6;
}
bool getFifoLevel(){
    return (0b00100000 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>5;
}
bool getFifoOverrun(){
    return (0b00010000 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>4;
}
void setFifoOverrun(){
    RFM69_writeReg(RFM69_REG_IRQFLAGS2, 0b00010000);
}
bool getPacketSent(){
    return (0b00001000 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>3;
}
bool getPayloadReady(){
    return (0b00000100 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>2;
}
bool getCrcOk(){
    return (0b00000010 & RFM69_readReg(RFM69_REG_IRQFLAGS2)) >>1;
}

void waitReadyIrq2(bool waitPacketSent){
    uint16_t timeout;

    // Wait on ModeReady
    if(waitPacketSent){
        timeout = now()+500;
        while (getPacketSent() != 1){
            if(now() > timeout){
                printf("timeout waiting on ready mode...\n");
                break;
            }
            sleep_ms(100);
        } // wait for ModeReady
    }
}

//////////////////////////////////////////////////
// Power Control
//////////////////////////////////////////////////
void setOverCurrentProtection(bool OcpOn, uint8_t OcpTrim){
    // An over current protection block is built-in the module. It helps preventing surge currents required when the transmitter is
    // used at its highest power levels, thus protecting the battery that may power the application. The current clamping value is
    // controlled by OcpTrim bits in RegOcp, and is calculated with the following formula:

    // Imax = 45 + 5 - OcpTrim mA

    // Imax sets a limit on the current drain of the Power Amplifier only, hence the maximum current drain of the
    // RFM69HCW is equal to Imax + I FS

    // Value for High Power: 0x0F (0b01111 = off)
    // Value for Rx or PA0: 0x1x (0x1a-0x1f = enabled, with trim)

    // Enables overload current protection (OCP) for the PA:
    uint8_t newOcpOn = 0; //  0 → OCP disabled
    if(OcpOn)
        newOcpOn = 1; // 1 → OCP enabled

    // Trimming of OCP current:
    // 95 mA OCP by default
    uint8_t newOcpTrim;
    if(OcpTrim>15)
        newOcpTrim = 15;
    else
        newOcpTrim = OcpTrim;

    uint8_t newOcp = newOcpOn << 4 | newOcpTrim;
    RFM69_writeReg(RFM69_REG_OCP, newOcp);
}

void setSensitivityBoost(bool SensitivityBoostOn){
    if(SensitivityBoostOn && radio_settings.use_rx_boost){
        RFM69_writeReg(RFM69_REG_TESTLNA, 0x2D); // 0x2D → High sensitivity mode
    }else{
        RFM69_writeReg(RFM69_REG_TESTLNA, 0x1B); // 0x1B → Normal mode
    }
}

void setRssiThreshold(uint8_t RssiThreshold){
    // Received Signal Strength Indicator Threshold

    // The receiver stays in WAIT mode, until RssiValue exceeds RssiThreshold for two consecutive samples. Its power
    // consumption is the receiver power consumption.

    // For correct operation of the AGC, RssiThreshold in RegRssiThresh must be set to the sensitivity of the receiver. The
    // receiver will remain in WAIT mode until RssiThreshold is exceeded.

    // RSSI trigger level for Rssi interrupt :
    // Default = 0xE4 / 228 (-114dBm)
    // Max = -115 dB+ Dynamic Range RSSI

    uint8_t newRssiThreshold;
    if(RssiThreshold>230)
        newRssiThreshold = 230;
    else
        newRssiThreshold = RssiThreshold;

    RFM69_writeReg(RFM69_REG_RSSITHRESH, newRssiThreshold); // - (RssiThreshold / 2) [dBm]
}

void setPowerLevel(bool Pa0On, bool Pa1On, bool Pa2On, uint8_t OutputPower, bool PaBoost){
    bool newPaBoost = PaBoost;
    if(!radio_settings.use_tx_boost)
        newPaBoost = false;
    // The RFM69HCW has a high power +20 dBm capability on PA_BOOST pin

    // When PA1 and PA2 are combined to deliver +20 dBm to the antenna, a specific impedance matching / harmonic filtering
    // design is required to ensure impedance transformation and regulatory compliance.

    // p0=1 p1=0 p2=0 : PR: -18 to +13 dBm Formula: -18 dBm + OutputPower
    // p0=0 p1=1 p2=0 : PR: -2 to +13 dBm  Formula: -18 dBm + OutputPower
    // p0=0 p1=1 p2=1 : PR: +2 to +17 dBm  Formula: -14 dBm + OutputPower
    // p0=0 p1=1 p2=1 + HIGH POWER : PR: +5 to +20 dBm Formula: -11 dBm + OutputPower

    // Enables PA0, connected to RFIO and LNA
    uint8_t newPa0On = 0;
    if(Pa0On && !Pa1On && !Pa2On){
        newPa0On = 1;
        // High Power settings MUST be turned off when using PA0, and in Receive mode
        newPaBoost = false;
    }

    // Enables PA1, on PA_BOOST pin
    uint8_t newPa1On = 0;
    if(Pa1On || Pa2On)
        newPa1On = 1;

    // Enables PA2, on PA_BOOST pin
    uint8_t newPa2On = 0;
    if(Pa2On)
        newPa2On = 1;

    // Output power setting, with 1 dB steps
    // Pout = -18 + OutputPower [dBm] , with PA0
    // Pout = -18 + OutputPower [dBm] , with PA1**
    // Pout = -14+ OutputPower [dBm] , with PA1 and PA2**
    // Pout = -11 + OutputPower [dBm] , with PA1 and PA2, and high Power PA settings (refer to section 3.3.7)**
    uint8_t newOutputPower;
    if((Pa1On||Pa2On) && OutputPower < 16){
        // ** Only the 16 upper values of OutputPower are accessible
        newOutputPower = 16;
    }else if(OutputPower>31){
        newOutputPower=31;
    }else{
        newOutputPower=OutputPower;
    }

    // BOOST Limits
    if(newPaBoost){
        newPa0On = 0;
        newPa1On = 1;
        newPa2On = 1;
        if(newOutputPower<16)
            newOutputPower = 16;
    }

    // Set Power Level
    uint8_t newPaLevel = newPa0On << 7 | newPa1On << 6 | newPa2On << 5 | newOutputPower;
    RFM69_writeReg(RFM69_REG_PALEVEL, newPaLevel);

    // Set Overcurrent Protection and High Power
    if(newPaBoost){
        // First disable OCP
        setOverCurrentProtection(false, 15);
        // now enable high power
        RFM69_writeReg(RFM69_REG_TESTPA1, 0x5D); // 0x5D → +20 dBm mode
        RFM69_writeReg(RFM69_REG_TESTPA2, 0x7C); // 0x7C → +20 dBm mode
    }else{
        // First disable High Power
        RFM69_writeReg(RFM69_REG_TESTPA1, 0x55); // 0x55 → Normal mode and Rx mode
        RFM69_writeReg(RFM69_REG_TESTPA2, 0x70); // 0x70 → Normal mode and Rx mode
        // now enable OCP
        setOverCurrentProtection(true, 10);
    }
}

void setPaRamp(uint8_t PaRamp){
    // Rise/Fall time of ramp up/down in FSK

    // 0  0000 → 3.4 ms
    // 1  0001 → 2 ms
    // 2  0010 → 1 ms
    // 3  0011 → 500 us
    // 4  0100 → 250 us
    // 5  0101 → 125 us
    // 6  0110 → 100 us
    // 7  0111 → 62 us
    // 8  1000 → 50 us
    // 9  1001 → 40 us << DEFAULT
    // 10 1010 → 31 us
    // 11 1011 → 25 us
    // 12 1100 → 20 us
    // 13 1101 → 15 us
    // 14 1110 → 12 us
    // 15 1111 → 10 us

    uint8_t newPaRamp = PaRamp;
    if(PaRamp>15){
        newPaRamp=15;
    }

    RFM69_writeReg(RFM69_REG_PARAMP, newPaRamp);
}

void setLowNoiseAmplifier(uint8_t LnaZin, uint8_t LnaGainSelect){
    // In the specific case where the LNA gain is manually set by the user, the receiver will not be able to properly handle
    // FSK signals with a modulation index smaller than 2 at an input power greater than the 1dB compression point,
    // tabulated in section 3.4.3.

    // LNA’s input impedance
    uint8_t newLnaZin = 0; // 0 → 50 ohms
    if(LnaZin > 0)
        newLnaZin = 1; // 1 → 200 ohms

    // LNA gain setting:
    // 000 → gain set by the internal AGC loop
    // 001 → G1 = highest gain
    // 010 → G2 = highest gain – 6 dB
    // 011 → G3 = highest gain – 12 dB
    // 100 → G4 = highest gain – 24 dB
    // 101 → G5 = highest gain – 36 dB
    // 110 → G6 = highest gain – 48 dB
    // 111 → reserved
    uint8_t newLnaGainSelect = LnaGainSelect;
    if(newLnaGainSelect > 6)
        newLnaGainSelect = 6;

    uint8_t newLna = newLnaZin<<7 | newLnaGainSelect;
    RFM69_writeReg(RFM69_REG_LNA, newLna);
}

void setOpMode(bool SequencerOff, bool ListenOn, bool ListenAbort, uint8_t Mode){
    //Controls the automatic Sequencer (see section 4.2 ):
    //0 → Operating mode as selected with Mode bits in
    //RegOpMode is automatically reached with the Sequencer
    //1 → Mode is forced by the user
    uint8_t setSequencerOff = 0;
    if(SequencerOff){
        setSequencerOff = 1;
    }

    // Enables Listen mode, should be enabled whilst in
    //Standby mode:
    //0 → Off (see section 4.3)
    //1 → On
    uint8_t setListenOn = 0;
    if(ListenOn){
        setListenOn = 1;
    }

    // Aborts Listen mode when set together with ListenOn=0
    //See section 4.3.4 for details
    //Always reads 0.
    uint8_t setListenAbort = 0;
    if(ListenAbort){
        setListenAbort = 1;
    }

    // Transceiver’s operating modes:
    uint8_t setMode = 0; // Sleep mode (SLEEP)
    if(Mode == 1){
        setMode = 0b001; // Standby mode (STDBY)
    } else if (Mode == 2) {
        setMode = 0b010; // Frequency Synthesizer mode (FS)
    } else if (Mode == 3) {
        setMode = 0b011; // Transmitter mode (TX)
        setListenAbort = 1;
    } else if (Mode == 4) {
        setMode = 0b100; // Receiver mode (RX)
        setListenAbort = 1;
    } // others → reserved; Reads the value corresponding to the current module mode

    //printf("Seq: %i, List: %i, LAbor: %i, Mode: %i\n",setSequencerOff,setListenOn,setListenAbort,setMode);
    uint8_t newOpMode = setSequencerOff << 7 | setListenOn << 6 | setListenAbort << 5 | setMode << 2;

    RFM69_writeReg(RFM69_REG_OPMODE, newOpMode);

    // Mode was changed so we wait for the mode ready...
    //Set when the operation mode requested in Mode, is ready
    //- Sleep: Entering Sleep mode
    //- Standby: XO is running
    //- FS: PLL is locked
    //- Rx: RSSI sampling starts
    //- Tx: PA ramp-up completed
    //Cleared when changing operating mode.
    uint16_t timeout = now()+500;
    while (getModeReady() != 1){
        if(now() > timeout){
            printf("timeout waiting on ready mode...\n");
            break;
        }
        sleep_ms(100);
    } // wait for ModeReady

    // Extra Actions Based on Mode change...
    if(radio_settings.mode != Mode){
        switch (Mode) {
            case RFM69_MODE_SLEEP:
                setSensitivityBoost(false);
                setPowerLevel(true, false, false, 10, false);
                break;
            case RFM69_MODE_STDBY:
                setSensitivityBoost(false);
                setPowerLevel(true, false, false, 10, false);
                break;
            case RFM69_MODE_TX:
                setSensitivityBoost(false);
                //RFM69_writeReg(RFM69_REG_LNA, 0x0);
                setLowNoiseAmplifier(0, 0);
                setPowerLevel(false, true, true, radio_settings.powerLevel, true);
                break;
            case RFM69_MODE_RX:
                setSensitivityBoost(true);
                //RFM69_writeReg(RFM69_REG_LNA, 0b10001001);
                setLowNoiseAmplifier(1, 2);
                setPowerLevel(true, false, false, 10, false);
                break;
            default:
                return;
        } // End switch
    }
    radio_settings.SequencerOff = SequencerOff;
    radio_settings.ListenOn = ListenOn;
    radio_settings.mode = Mode;
}

void setSequencerOff(bool SequencerOff){
    if (SequencerOff == radio_settings.SequencerOff)
        return;
    setOpMode(SequencerOff,radio_settings.ListenOn,false, radio_settings.mode);
}

void setListenOn(bool ListenOn){
    if(ListenOn == radio_settings.ListenOn)
        return;
    setOpMode(radio_settings.SequencerOff,ListenOn,false, radio_settings.mode);
}

void setMode(uint8_t Mode)
{
    /*if (Mode == radio_settings.mode){
        printf("Abort mode change, already in correct mode...\n");
        return;
    }*/
    setOpMode(radio_settings.SequencerOff,radio_settings.ListenOn,false, Mode);
}

void setDataModul(uint8_t DataMode, uint8_t ModulationType, uint8_t ModulationShaping){
    // 7 - unused

    // 6-5 DataMode
    uint8_t newDataMode;
    if(DataMode == 0){
        newDataMode = 0b00; // 00 → Packet mode
    } else if (DataMode == 1) {
        newDataMode = 0b01; //01 → reserved
    } else if (DataMode == 2) {
        newDataMode = 0b10; //10 → Continuous mode with bit synchronizer
    } else if (DataMode == 3) {
        newDataMode = 0b11; //11 → Continuous mode without bit synchronizer
    } else {
        return;
    }

    // 4-3 ModulationType
    uint8_t newModulationType;
    if (ModulationType == 0){
        newModulationType = 0b00; // Modulation scheme: 00 → FSK
    } else if(ModulationType == 1){
        newModulationType = 0b01; // Modulation scheme: 01 → OOK
    } else{
        // 10 - 11 → reserved
        return;
    }

    // 2 - unused

    // 1-0 ModulationShaping
    uint8_t newModulationShaping;
    if(ModulationShaping == 0){
        // in FSK: 00 → no shaping
        // in OOK: 00 → no shaping
        newModulationShaping = 0b00;
    } else if (ModulationShaping == 1) {
        // in FSK: 01 → Gaussian filter, BT = 1.0
        // in OOK: 01 → filtering with f cutoff = BR
        newModulationShaping = 0b01;
    } else if (ModulationShaping == 2) {
        // in FSK: 10 → Gaussian filter, BT = 0.5
        // in OOK: 10 → filtering with f cutoff = 2*BR
        newModulationShaping = 0b10;
    } else if (ModulationShaping == 3) {
        // in FSK: 11 → Gaussian filter, BT = 0.3
        // in OOK: 11 → reserved
        if(ModulationType == 1){
            // 11 is reserved in ook, can't set this.
            return;
        }
        newModulationShaping = 0b11;
    } else {
        return;
    }

    uint8_t newDataModul = newDataMode << 5 | newModulationType << 3 | newModulationShaping;

    RFM69_writeReg(RFM69_REG_DATAMODUL, newDataModul);
}

void setBitrate(uint16_t bitrate){
    // BitRate = FXO SC / BitRate(15,0)
    // Default value: 4.8 kb/s
    // 0 - 65,535

    // 0x68 0x2b = 26667 = 1.2kbps = 32000/26667 = 1.19998500019
    // 0x34 0x15 = 13333 = 2.4kbsp = 32000/13333 = 2.4000600015
    // 0x1A 0x0B = 6667 = 4.8kbsp = 32000/6667 = 4.799760012
    // 0x0D 0x05 = 3333 = 9.6kbps = 32000/3333 = 9.600960096
    // 0x06 0x83 = 1667 = 19.2 kbps = 32000/1667 = 19.1961607678
    // 0x03 0x41 = 833 = 38.4 kbps = 32000/833 = 38.4153661465
    // 0x01 0xA1 = 417 = 76.8 kbps = 32000/417 = 76.7386091127
    // 0x00 0xD0 = 208 = 153.6 kbps = 32000/208 = 153.846153846
    // ...
    // 0x02 0x80 = 640 = 50 kbps = 32000/640 = 50
    // 0x01 0x40 = 320 = 100 kbps = 32000/320 = 100
    // 0x00 0x6B = 107 = 300 kbps = 32000/107 = 299.065420561
    // ...
    // 0x03 0xD1 = 977 = 32.768 kbps = 32000/977 = 32.75332651
    uint8_t newBitrateMsb = bitrate >> 8; // MSB of Bit Rate (Chip Rate when Manchester encoding is enabled)
    uint8_t newBitrateLsb = bitrate; // LSB of Bit Rate (Chip Rate if Manchester encoding is enabled)
    RFM69_writeReg(RFM69_REG_BITRATEMSB, newBitrateMsb);
    RFM69_writeReg(RFM69_REG_BITRATELSB, newBitrateLsb);
}

void setRxBw(uint8_t DccFreq, uint8_t RxBwMant, uint8_t RxBwExp){
    //////////////////////////////////////////////////
    // Info:
    //////////////////////////////////////////////////
    // FXOSC = Crystal oscillator frequency (32MHz)

    // DccFreq (010) [7-5] // Cut-off frequency of the DC offset canceller (DCC):
    // ~4% of the RxBw by default
    // fc = 4*RxBw/2pi*2^DccFreq+2
    if(DccFreq>7)
        return;

    // RxBwMant (10) [4-3] // Channel filter bandwidth control:
    // 00 → RxBwMant = 16
    // 01 → RxBwMant = 20
    // 10 → RxBwMant = 24
    if(RxBwMant>2)
        return;

    // RxBwExp (101) [2-0] // Channel filter bandwidth control:
    // FSK Mode: RxBw  = FXOSC / RxBwMant ∗2^(RxBwExp + 2)
    // OOK Mode: RxBw  = FXOSC / RxBwMant ∗2^(RxBwExp + 3)
    // Max value = 7+3 in OOK
    if(RxBwExp>7)
        return;

    //////////////////////////////////////////////////
    // Notes / math
    //////////////////////////////////////////////////

    // so the calc of default (FSK) is...
    // 32mhz / 24∗2^(5 + 2) = 32mhz/24*2^7 = 32/24*128 = 32/3072 = 32*khz/3072 = 10.4166666667
    // this matches with the table which shows;
    // mant: 24, Exp: 5 = 10.4kHz...

    // lowpowerlab had...
    // 32000/16*2^(2+2) = 32000/16*2^4 = 32000/16*16 = 32000/256 = 125kHz ??? i calc wrong?

    uint8_t newRxBw = DccFreq<<5 | RxBwMant<<3 | RxBwExp;
    RFM69_writeReg(RFM69_REG_RXBW, newRxBw);
}

void printRxBw(uint8_t RxBwMant, uint8_t RxBwExp){
    if(RxBwMant>2)
        return;

    if(RxBwExp>7)
        return;

    uint32_t tempCalcMant;
    if(RxBwMant==0)
        tempCalcMant=16;
    else if(RxBwMant==1)
        tempCalcMant=20;
    else if(RxBwMant==2)
        tempCalcMant=24;

    RxBwExp+=2;
    uint32_t powRes=1;
    while (RxBwExp!=0){
        powRes*=2;
        RxBwExp--;
    }
    uint32_t khzResult = 32000/(tempCalcMant*powRes);
    printf("new FSK RxBw: %i kHz (default 10.417kHz)\n",khzResult);
}

void setFrequencyDeviation(uint16_t frequencyDeviation){
    // frequency deviation Default value: 5 kHz
    // 0 - 16383
    // Fdev = Fstep ⋅ Fdev(15,0)
    uint16_t newfrequencyDeviation = 0;
    if(frequencyDeviation>16383){
        newfrequencyDeviation = 16383;
    }else{
        newfrequencyDeviation = frequencyDeviation;
    }
    uint8_t newFDMsb = newfrequencyDeviation >> 8; // MSB of Bit Rate (Chip Rate when Manchester encoding is enabled)
    uint8_t newFDLsb = newfrequencyDeviation; // LSB of Bit Rate (Chip Rate if Manchester encoding is enabled)
    RFM69_writeReg(RFM69_REG_FDEVMSB, newFDMsb);
    RFM69_writeReg(RFM69_REG_FDEVLSB, newFDLsb);
}

bool calibrateOscillatorDone(){
    return ((0b10000000 & RFM69_readReg(RFM69_REG_OSC1))>>7);
}

void calibrateOscillator(){
    // Triggers the calibration of the RC oscillator when set.
    // Always reads 0. RC calibration must be triggered in
    // Standby mode.

    uint8_t modeSave = radio_settings.mode;
    setMode(RFM69_MODE_STDBY);

    uint8_t RcCalStart = 1;
    uint8_t newRegOsc1 = RcCalStart<<7;
    RFM69_writeReg(RFM69_REG_OSC1, newRegOsc1);

    for(uint8_t i = 0; i<100; i++){
        if(calibrateOscillatorDone()){
            break;
        }
        sleep_ms(100);
    }
    setMode(modeSave);
}

void setAfcLowBetaOn(bool AfcLowBetaOn){
    // Improved AFC routine for signals with modulation index
    //lower than 2. Refer to section 3.4.16 for details

    //0 → Standard AFC routine
    //1 → Improved AFC routine
    uint8_t newAfcLowBetaOn = 0;
    if(AfcLowBetaOn){
        newAfcLowBetaOn = 1;
    }
    RFM69_writeReg(RFM69_REG_OSC1, (newAfcLowBetaOn<<5));
}

void setListenSettings(uint8_t ListenResolIdle, uint8_t ListenResolRx, uint8_t ListenCriteria, uint8_t ListenEnd, uint8_t ListenCoefIdle, uint8_t ListenCoefRx){

    // Resolution of Listen mode Idle time (calibrated RC osc):
    uint8_t newListenResolIdle;
    if(ListenResolIdle == 1){
        newListenResolIdle = 0b01; // 01 → 64 us
    } else if(ListenResolIdle == 2){
        newListenResolIdle = 0b10; // 10 → 4.1 ms = Default
    } else if(ListenResolIdle == 3){
        newListenResolIdle = 0b01; // 11 → 262 ms
    } else {
        return;
    }

    // Resolution of Listen mode Rx time (calibrated RC osc):
    uint8_t newListenResolRx;
    if(ListenResolRx == 1){
        newListenResolRx = 0b01; // 01 → 64 us
    } else if(ListenResolRx == 2){
        newListenResolRx = 0b10; // 10 → 4.1 ms = Default
    } else if(ListenResolRx == 3){
        newListenResolRx = 0b01; // 11 → 262 ms
    } else {
        return;
    }

    // Criteria for packet acceptance in Listen mode:
    uint8_t newListenCriteria;
    if (ListenCriteria==0){
        newListenCriteria = 0; // 0 → signal strength is above RssiThreshold
    } else if (ListenCriteria==1){
        newListenCriteria = 1; // 1 → signal strength is above RssiThreshold and SyncAddress matched
    } else {
        return;
    }

    // ListenEnd
    // Action taken after acceptance of a packet in Listen mode:
    uint8_t newListenEnd;
    if(ListenEnd == 0){
        newListenEnd = 0b00; //00 → chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3).
    } else if(ListenEnd == 1){
        newListenEnd = 0b01; //01 → chip stays in Rx mode until PayloadReady or Timeout interrupt occurs. It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3).
    } else if(ListenEnd == 2){
        newListenEnd = 0b10; //10 → chip stays in Rx mode until PayloadReady or Timeout interrupt occurs. Listen mode then resumes in Idle state. FIFO content is lost at next Rx wakeup.
    } else {
        return;
    }

    uint8_t newListen1 = newListenResolIdle << 6 | newListenResolRx << 4 | newListenCriteria << 3 | newListenEnd <<1;
    RFM69_writeReg(RFM69_REG_LISTEN1, newListen1);
    // Duration of the Idle phase in Listen mode. tListenIdle = ListenCoefIdle ∗ ListenResolIdle
    RFM69_writeReg(RFM69_REG_LISTEN2, ListenCoefIdle);
    // Duration of the Rx phase in Listen mode (startup time included, see section 4.2.3) t ListenRx = ListenCoefRx ∗ ListenResolRx
    RFM69_writeReg(RFM69_REG_LISTEN3, ListenCoefRx);
}

void setPacketConfig(bool pktLenVariable, bool ManchesterEnc, bool WhiteningEnc, bool crcOn, bool CrcAutoClearOff, bool filterAddress, bool filterBroadcast){

    // Defines the packet format used:
    uint8_t newPacketFormat = 0; // 0 → Fixed length
    if(pktLenVariable){
        newPacketFormat = 1; // 1 → Variable length
    }

    // Defines DC-free encoding/decoding performed:
    uint8_t newDcFree = 0;
    if(!ManchesterEnc && !WhiteningEnc)
        newDcFree = 0; // 00 → None (Off)
    else if(ManchesterEnc)
        newDcFree = 1; // 01 → Manchester
    else if(WhiteningEnc)
        newDcFree = 2; // 10 → Whitening

    // Enables CRC calculation/check (Tx/Rx):
    uint8_t newCrcOn = 0;
    if(crcOn)
        newCrcOn = 1;

    // Defines the behavior of the packet handler when CRC check fails:
    uint8_t newCrcAutoClearOff = 0; // 0 → Clear FIFO and restart new packet reception. No PayloadReady interrupt issued.
    if(CrcAutoClearOff)
        newCrcAutoClearOff = 1; // 1 → Do not clear FIFO. PayloadReady interrupt issued.

    // Defines address based filtering in Rx:
    uint8_t newAddressFiltering;
    if(!filterAddress)
        newAddressFiltering = 0; // 00 → None (Off)
    else if(filterAddress && !filterBroadcast)
        newAddressFiltering = 1; // 01 → Address field must match NodeAddress
    else if(filterAddress)
        newAddressFiltering = 2; // 10 → Address field must match NodeAddress or BroadcastAddress

    uint8_t  newPacketConfig1 = newPacketFormat<<7 | newDcFree<<5 | newCrcOn<<4 | newCrcAutoClearOff<<3 | newAddressFiltering<<1;
    RFM69_writeReg(RFM69_REG_PACKETCONFIG1, newPacketConfig1);
}

void setPacketConfig2(uint8_t InterPacketRxDelay, bool RestartRx, bool AutoRxRestartOn, bool AesOn){
    // [7-4] InterPacketRxDelay
    // 3 - unused
    // 2 RestartRx
    // 1 AutoRxRestartOn
    // 0 AesOn
    uint8_t newInterPacketRxDelay = InterPacketRxDelay;
    if(newInterPacketRxDelay>=12)
        newInterPacketRxDelay = 0;

    uint8_t newRestartRx = 0;
    if(RestartRx)
        newRestartRx = 1;

    uint8_t newAutoRxRestartOn = 0;
    if(AutoRxRestartOn)
        newAutoRxRestartOn = 1;

    uint8_t newAesOn = 0;
    if(AesOn)
        newAesOn = 1;

    uint8_t newPacketConfig2 = newInterPacketRxDelay<<4 | newRestartRx<<2 | newAutoRxRestartOn<<1 | newAesOn;
    RFM69_writeReg(RFM69_REG_PACKETCONFIG2, newPacketConfig2);
}

void setEncrypt(uint8_t *aesKey) {
    setMode(RFM69_MODE_STDBY);
    uint8_t validKey = aesKey != 0 && strlen(aesKey)!=0;
    if (validKey)
    {
        uint8_t buf[2];
        buf[0] = RFM69_REG_AESKEY1 | 0x80;
        cs_select();
        spi_write_blocking(SPI_PORT, buf, 1);
        for (uint8_t i = 0; i < 16; i++){
            buf[0] = aesKey[i];
            spi_write_blocking(SPI_PORT, buf, 1);
        }
        cs_deselect();
        sleep_ms(1);

    }
    //uint8_t validKey = key != 0 && strlen(key)!=0;
    RFM69_writeReg(RFM69_REG_PACKETCONFIG2, (RFM69_readReg(RFM69_REG_PACKETCONFIG2) & 0xFE) | (validKey ? 1 : 0));
}

// pow function
uint32_t power(uint32_t start, uint32_t power){
    uint32_t powResult=1;
    for(;power!=0; power--){
        powResult*=start;
    }
    return powResult;
}

// Returns FSTEP in Hz (61.035156)
double getFstepHz(){
    // FSTEP = FXOSC/2^19
    uint32_t fxosc = 32000000;
    uint32_t powercalc = power(2, 19);
    return (double)fxosc/powercalc;
}

// Returns FSTEP in Hz (61.035156)
double getFstepKHz(){
    // FSTEP = FXOSC/2^19
    uint32_t fxosc = 32000;
    uint32_t powercalc = power(2, 19);
    return (double)fxosc/powercalc;
}

// Returns FSTEP in Hz (61.035156)
double getFstepMHz(){
    // FSTEP = FXOSC/2^19
    uint32_t fxosc = 32;
    uint32_t powercalc = power(2, 19);
    return (double)fxosc/powercalc;
}

// return the frequency (in Hz)
uint32_t RFM69_getFrequency()
{
    // The carrier frequency is programmed through RegFrf, split across addresses 0x07 to 0x09:
    // FRF = FSTEP - Frf(23,0);
    // fxosc = 32000
    // 2^19 = 524288
    // 32000 / 524288 = 0.06103515625
    uint32_t theFreq = (((uint32_t) RFM69_readReg(RFM69_REG_FRFMSB) << 16) + ((uint16_t) RFM69_readReg(RFM69_REG_FRFMID) << 8) + RFM69_readReg(RFM69_REG_FRFLSB));
    theFreq*=getFstepHz();
    return theFreq;
}

void rmf69_SetFrequency(double Frequency)
{
    // Frf = Fstep - Frf 23:0

    // Optimized Setup for Low Modulation Index Systems
    // For wide band systems, where AFC is usually not required (XTAL inaccuracies do not typically impact the sensitivity), it
    // is recommended to offset the LO frequency of the receiver to avoid desensitization. This can be simply done by
    // modifying Frf in RegFrfLsb. A good rule of thumb is to offset the receiver’s LO by 10% of the expected transmitter
    // frequency deviation.
    // For narrow band systems, it is recommended to perform AFC. The RFM69HCW has a dedicated AFC, enabled
    // when
    // AfcLowBetaOn in RegAfcCtrl is set to 1. A frequency offset, programmable through LowBetaAfcOffset in RegTestAfc, is
    // added and is calculated as follows:
    // Offset = LowBetaAfcOffset x 488 Hz

    uint32_t newFrf = Frequency / getFstepMHz();
    RFM69_writeReg(RFM69_REG_FRFMSB, (newFrf >> 16) & 0xFF); // MSB of the RF carrier frequency
    RFM69_writeReg(RFM69_REG_FRFMID, (newFrf >> 8) & 0xFF);  // Middle byte of the RF carrier frequency
    RFM69_writeReg(RFM69_REG_FRFLSB, newFrf & 0xFF); // LSB of the RF carrier frequency
}

void setPreambleSize(uint16_t newSize){
    // Preamble 0 to 65535 bytes (Frame identification)
    // Default: 3
    uint8_t msb = newSize >> 8; // 0x00; (default) (shift bits 0b1111111100000000 to 0b11111111)
    uint8_t lsb = newSize; // 0x03; (default) (drops bits 0bxxxxxxxx00000000 to 0b00000000)

    // In Packet mode, the RFM69HCW will automatically modulate the RF signal with preamble bytes as soon as TxReady or
    // ModeReady happen. The actual packet transmission (starting with the number of preambles specified in PreambleSize)
    // will start when the TxStartCondition is fulfilled.

    // Packet mode (recommended): user only provides/retrieves payload bytes to/from the FIFO. The packet is automatically
    // built with preamble, Sync word, and optional AES, CRC, and DC-free encoding schemes The reverse operation is
    // performed in reception. The uC processing overhead is hence significantly reduced compared to Continuous mode.
    // Depending on the optional features activated (CRC, AES, etc) the maximum payload length is limited to FIFO size, 255
    // bytes or unlimited.

    // PreambleMsb 7-0 PreambleSize(15:8)
    // PreambleLsb 7-0 PreambleSize(7:0)
    //Size of the preamble to be sent (from TxStartCondition fulfilled)
    RFM69_writeReg(RFM69_REG_PREAMBLEMSB, msb);
    RFM69_writeReg(RFM69_REG_PREAMBLELSB, lsb);
}

void setSyncConfig(uint8_t SyncOn, uint8_t FifoFillCondition, uint8_t SyncSize, uint8_t SyncTol){

    uint8_t shiftedSyncOn = (SyncOn<<7); // Enables the Sync word generation and detection: 0 → Off 1 → On
    uint8_t shiftedFifoFillCondition = (FifoFillCondition<<6); // FIFO filling condition: 0 → if SyncAddress interrupt occurs 1 → as long as FifoFillCondition is set
    uint8_t shiftedSyncSize = ((SyncSize-1)<<3); // Size of the Sync word: (SyncSize + 1) bytes (max value is 0b111 which is 7, but 7+1 = 8)
    // (0) SyncTol Number of tolerated bit errors in Sync word

    RFM69_writeReg(RFM69_REG_SYNCCONFIG, shiftedSyncOn | shiftedFifoFillCondition | shiftedSyncSize | SyncTol);
}

void setSyncWords(uint8_t sync_word1, uint8_t sync_word2, uint8_t sync_word3, uint8_t sync_word4, uint8_t sync_word5, uint8_t sync_word6, uint8_t sync_word7, uint8_t sync_word8){
    // Syncwords 0-8 bytes (known seequence identifying the start of a frame)

    uint8_t sync_msb_1 = sync_word1; // 0x01; // 7-0 SyncValue(63:56)
    uint8_t sync_lsb_1 = sync_word2; // 0x01; // 7-0 SyncValue(55:48)
    uint8_t sync_lsb_2 = sync_word3; // 0x01; // 7-0 SyncValue(47:40)
    uint8_t sync_lsb_3 = sync_word4; // 0x01; // 7-0 SyncValue(39:32)
    uint8_t sync_lsb_4 = sync_word5; // 0x01; // 7-0 SyncValue(31:24)
    uint8_t sync_lsb_5 = sync_word6; // 0x01; // 7-0 SyncValue(23:16)
    uint8_t sync_lsb_6 = sync_word7; // 0x01; // 7-0 SyncValue(15:8)
    uint8_t sync_lsb_7 = sync_word8; // 0x01; // 7-0 SyncValue(7:0)

    RFM69_writeReg(RFM69_REG_SYNCVALUE1, sync_msb_1); // 1 st byte of Sync word. (MSB byte) Used if SyncOn is set.
    RFM69_writeReg(RFM69_REG_SYNCVALUE2, sync_lsb_1); // 2 nd byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 2.
    RFM69_writeReg(RFM69_REG_SYNCVALUE3, sync_lsb_2); // 3 rd byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 3.
    RFM69_writeReg(RFM69_REG_SYNCVALUE4, sync_lsb_3); // 4 th byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 4.
    RFM69_writeReg(RFM69_REG_SYNCVALUE5, sync_lsb_4); // 5 th byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 5.
    RFM69_writeReg(RFM69_REG_SYNCVALUE6, sync_lsb_5); // 6 th byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 6.
    RFM69_writeReg(RFM69_REG_SYNCVALUE7, sync_lsb_6); // 7 th byte of Sync word. Used if SyncOn is set and (SyncSize +1) >= 7.
    RFM69_writeReg(RFM69_REG_SYNCVALUE8, sync_lsb_7); // 8 th byte of Sync word. Used if SyncOn is set and (SyncSize +1) = 8.
}

void setNodeAddress(){
    RFM69_writeReg(RFM69_REG_NODEADRS, radio_settings.nodeID);
    RFM69_writeReg(RFM69_REG_BROADCASTADRS, 255);
}

void setMaxPayloadLen(uint8_t maxPayloadSize){
    RFM69_writeReg(RFM69_REG_PAYLOADLENGTH, maxPayloadSize);
}

void setFifoThresh(bool TxWaitThreshold, uint8_t ThresholdBytes){
    uint8_t newTxWaitThreshold = 0;
    if(TxWaitThreshold)
        newTxWaitThreshold = 1;

    uint8_t newThresholdBytes = 1;
    if(ThresholdBytes<=127)
        newThresholdBytes = ThresholdBytes;

    uint8_t newFifoThresh = newTxWaitThreshold<<7 | newThresholdBytes;
    RFM69_writeReg(RFM69_REG_FIFOTHRESH, newFifoThresh);
}

void setTestDagc(uint8_t ContinuousDagc){
    // Fading Margin Improvement, refer to 3.4.4
    // 0x00 → Normal mode
    // 0x20 → Improved margin, use if AfcLowBetaOn=1
    // 0x30 → Improved margin, use if AfcLowBetaOn=0
    if(ContinuousDagc!=0x00 && ContinuousDagc!=0x20 && ContinuousDagc!=0x30)
        return;

    RFM69_writeReg(RFM69_REG_FIFOTHRESH, ContinuousDagc);
}

int rmf69_init(uint8_t net_id, uint8_t node_id, float Frequency){
    //////////////////////////////////////////////////
    // SETTINGS
    //////////////////////////////////////////////////

    // ID:
    radio_settings.networkID = net_id;
    radio_settings.nodeID = node_id;

    // POWER:
    radio_settings.powerLevel = 31;
    radio_settings.use_tx_boost = true;
    radio_settings.use_rx_boost = true;

    // Filters:
    radio_settings.filter_address = true;
    radio_settings.filter_broadcast = false;
    radio_settings.crc_check = true;
    radio_settings.rssi_threshold = 130; // -65db start (auto increases if we can't find a signal)
    radio_settings.max_payload_size = 18; // TTTT-HHHH-PPPP\0 + headers = 18

    // Mode / Function:
    radio_settings.mode = 255; // NO MODE YET
    radio_settings.SequencerOff = false;
    radio_settings.ListenOn = false;

    // Bandwidth:
    radio_settings.bitrate = 3333; // 9.6kbps
    radio_settings.rxbw_maint = 2; // 20kHz
    radio_settings.rxbw_exp = 4; // 20kHz

    //////////////////////////////////////////////////
    // SETUP PINS
    //////////////////////////////////////////////////
    // datasheet doesn't mention a low range for the clock;
    //spi_init(SPI_PORT, 8000*1000); // lowpowerlab has 8000000 hz (8000 khz | 8mhz)...
    //spi_init(SPI_PORT, 2000*1000); // other recommendations online suggest 2mhz lowest (others reported issues at 500khz)
    spi_init(SPI_PORT, 500*1000); // 500hkz seems to be working for my usage...
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // RESET...
    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);
    gpio_put(PIN_RESET, 1);
    sleep_ms(100);
    gpio_put(PIN_RESET, 0);
    sleep_ms(100);

    // CS is output, chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // G0/DIO0 is input
    gpio_set_dir(PIN_DIO0, GPIO_IN);

    //////////////////////////////////////////////////
    // Activate Settings
    //////////////////////////////////////////////////

    // Dio PIN Mappings
    RFM69_writeReg(RFM69_REG_DIOMAPPING1, 0b01000000); // dio pins 0y,1n,2n,3n
    RFM69_writeReg(RFM69_REG_DIOMAPPING2, 0b00000111); // dio pins 4n,5n + clkout n

    // General Settings
    setOpMode(radio_settings.SequencerOff, radio_settings.ListenOn, false, RFM69_MODE_TX);
    setDataModul(0,0,2);
    setTestDagc(0x30);
    setBitrate(radio_settings.bitrate);
    setRxBw(2, radio_settings.rxbw_maint, radio_settings.rxbw_exp);
    setFrequencyDeviation(82);
    rmf69_SetFrequency(Frequency);
    setPreambleSize(10);
    setSyncWords('-', radio_settings.networkID, 'W', 'S', 'T', 'A', 'T', 'I');
    setSyncConfig(1, 0, 8, 0);
    setPacketConfig(true, false, true, radio_settings.crc_check, false, radio_settings.filter_address, radio_settings.filter_broadcast);
    setPacketConfig2(2, false, true, false);
    if(radio_settings.filter_address){
        setNodeAddress();
    }

    // Recieve Settings
    //setListenSettings(2, 1, 0, 1, 245, 32);
    setListenSettings(2, 1, 1, 1, 245, 32);
    setRssiThreshold(radio_settings.rssi_threshold);
    setMaxPayloadLen(radio_settings.max_payload_size);

    // Transmit Settings
    setPaRamp(8);
    //setFifoThresh(false, 15);
    setFifoThresh(true, 18);

    // Prepare
    calibrateOscillator();
    setFifoOverrun(); // fifo buffers reset
    setEncrypt(0);

    //////////////////////////////////////////////////
    // PRINT ID / SETTINGS
    //////////////////////////////////////////////////
    // Get Version of the chip...
    // Version code of the chip. Bits 7-4 give the full revision
    // number; bits 3-0 give the metal mask revision number.
    uint8_t rfmChipVersion = RFM69_readReg(RFM69_REG_VERSION);
    printf("\n----------\n");
    int full_revision = (0xf & rfmChipVersion >> 4);
    int metal_mask_revision = 0xf0 & rfmChipVersion;
    printf("Returned RFM69HCW version: 0x%x (Revision: %i, Metal Mask Revision: %i) we wanted version 0x%x\n", rfmChipVersion, full_revision, metal_mask_revision, RMF69HCW_VERSION);
    printf("----------\n");
    if(rfmChipVersion != RMF69HCW_VERSION){
        return 1;
    }
    printf("Bitrate: %.2f kbsp (default 4.8kbps)\n", (float)32000/radio_settings.bitrate);
    printRxBw(radio_settings.rxbw_maint, radio_settings.rxbw_exp);

    // Get current frequency
    uint32_t currentFreq = RFM69_getFrequency();
    printf("Carrier Frequency: %.3f \n", (double)currentFreq/1000);

    //////////////////////////////////////////////////
    // SET STANDBY AND FINISH INIT
    //////////////////////////////////////////////////
    setMode(RFM69_MODE_STDBY);

    return 0;
}

uint8_t rfm69_sendFrame(uint16_t toAddress, const uint8_t *data, uint8_t len, bool requestACK, bool sendACK){

    // todo... check max packet size;
    // In Fixed length mode the Message part of the payload that can be encrypted/decrypted can be 64 bytes long. If the
    //address filtering is enabled, the length of the payload should be at max 65 bytes in this case.
    //In Variable length mode the Max message size that can be encrypted/decrypted is also 64 bytes when address filtering is
    //disabled, else it is 48 bytes. Thus, including length byte, the length of the payload is max 65 or 50 bytes (the latter when
    //address filtering is enabled).
    //If the address filtering is expected then AddressFiltering must be enabled on the transmitter side as well to prevent address
    //byte to be encrypted.

    //RFM69_writeReg( RFM69_REG_IRQFLAGS1, 0x08); // Status register: PLL Lock state, Timeout, RSSI > Threshold...
    //RFM69_writeReg( RFM69_REG_IRQFLAGS2, 0x08); // Status register: FIFO handling flags...

    setMode(RFM69_MODE_STDBY);
    //while ((RFM69_readReg(RFM69_REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    cs_select();
    uint8_t buf[1];

    // FIFO access: if the address byte corresponds to the address of the FIFO, then succeeding data byte will address the
    // FIFO. The address is not automatically incremented but is memorized and does not need to be sent between each data
    // byte. The NSS pin goes low at the beginning of the frame and stay low between each byte. It goes high only after the
    // last byte transfer.

    // Preamble (1010...) // This is auto-prepended by the chip... (len can be configured)
    // Sync word (Network ID) // This is configured and auto prepended
    // >> Length byte // Must be added here for variable len packets
    // >> Optional Address byte (Node ID) // must be added here if address filtering is on
    // >> Message data // the message added here
    // >> Optional 2-bytes CRC checksum // auto-appended by chip

    // Write to FIFO
    buf[0] = RFM69_REG_FIFO | 0x80;
    spi_write_blocking(SPI_PORT, buf, 1);

    // PREABLE AND SYNCWORD PREPENDED BY CHIP

    // Send Size
    buf[0] = len + 2; // Send size + Header size...
    spi_write_blocking(SPI_PORT, buf, 1);

    // 2x header (TO,FROM)
    buf[0] = toAddress; // Device we send to...
    spi_write_blocking(SPI_PORT, buf, 1);
    buf[0] = radio_settings.nodeID; // who we are...
    spi_write_blocking(SPI_PORT, buf, 1);

    // Actual Payload
    spi_write_blocking(SPI_PORT, data, len); // Message data

    cs_deselect();

    // go into transmit mode
    setMode(RFM69_MODE_TX);

    int maxcount=1000;
    while (((0b00001000 & (RFM69_readReg(RFM69_REG_IRQFLAGS2))) >>3) != 0x1){
        //printf("debug... irq PacketSent not yet... 0x%02X !!!\n",RFM69_readReg(RFM69_REG_IRQFLAGS2));
        sleep_ms(50);
        maxcount--;
        if(maxcount==0){
            printf("Something is wrong, failed sending...\n");
            setMode(RFM69_MODE_STDBY);
            return 1;
        }
    }

    waitReadyIrq2(true);
    printf("Packet was sent... !!!\n");
    setMode(RFM69_MODE_STDBY);

    return 0;
}

uint8_t rfm69_read(uint8_t *recv_from, uint8_t *recv_data){
    uint8_t sendBuff[2] = {0};
    uint8_t readBuff[255] = {0};
    uint8_t payloadlen, rxHeaderTo;
    // try to read from raido...

    setMode(RFM69_MODE_RX);
    //waitReady(false, true, false);

    //printf("START READ TEST...\n");
    cs_select();
    sendBuff[0] = RFM69_REG_FIFO & 0x7F; // Send the start address with the write mask off
    spi_write_blocking(SPI_PORT, sendBuff, 1);
    // Read data
    spi_read_blocking(SPI_PORT,0, readBuff,254);

    // Check for read timeout
    // Timeout interrupt is generated TimeoutRxStart x 16x Tbit after switching to RX mode if RssiThreshold flag does not
    // raise within this time frame

    //////////////////////////////////////////////////
    // Get a Payload
    //////////////////////////////////////////////////
    for(int max_try=100; max_try>0 && readBuff[0]==0; max_try--){
        // try to grab some data...
        spi_read_blocking(SPI_PORT,0, readBuff,254);
        sleep_ms(200);
    }
    payloadlen = readBuff[0];

    if(payloadlen==0){
        // Failed to get a packet with a payload
        printf("No packets with a payload found, aborting read.\n");
        cs_deselect();
        setOpMode(false,false,false,RFM69_MODE_SLEEP);
        if(radio_settings.rssi_threshold<230){
            // Permit weaker signals, as we're not getting anything...
            // We start from 190 (-95dBm), if this is too restricting, we allow weaker signals
            radio_settings.rssi_threshold+=10;
            setRssiThreshold(radio_settings.rssi_threshold);
            printf("Rssi Threshold increased to -%idBm\n\n",radio_settings.rssi_threshold/2);
        }
        return payloadlen;
    }

    //////////////////////////////////////////////////
    // Ensure packet is for us... (if chip is not configured)
    //////////////////////////////////////////////////
    rxHeaderTo = readBuff[1];
    if(!radio_settings.filter_address){
        // We need to filter the address ourselves...
        for(int max_try=100; max_try>0 && (readBuff[1]!=radio_settings.nodeID || readBuff[1]!=255); max_try--){
            // try to grab some data... addressed to us...
            spi_read_blocking(SPI_PORT,0, readBuff,254);
            sleep_ms(200);
        }

        payloadlen = readBuff[0];
        rxHeaderTo = readBuff[1];
        if (rxHeaderTo != radio_settings.nodeID && rxHeaderTo != 255){
            // Failed to get a packet addressed to us...
            printf("No packets addressed to us found, aborting read.\n");
            cs_deselect();
            setOpMode(false,false,false,RFM69_MODE_SLEEP);
            return payloadlen;
        }
    }

    //////////////////////////////////////////////////
    // Process Payload
    //////////////////////////////////////////////////
    if (payloadlen> 0 && (rxHeaderTo == radio_settings.nodeID || rxHeaderTo == 255)){
        printf("rxHeaderTo: %i\n", rxHeaderTo);

        *recv_from = readBuff[2];
        for(int i = 0; i<payloadlen; i++)
            recv_data[i] = readBuff[i+3];
        recv_data[payloadlen-3] = '\0';
    } else if (payloadlen>0) {
        printf("Packet is NOT for us... something went wrong... size: %i for: %i\n",payloadlen,rxHeaderTo);
        printf("DEBUG: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i\n",readBuff[0],readBuff[1],readBuff[2],readBuff[3],readBuff[4],readBuff[5],readBuff[6],readBuff[7],readBuff[8],readBuff[9],readBuff[10],readBuff[11],readBuff[12],readBuff[13],readBuff[14]);
        printf("DEBUG: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",readBuff[0],readBuff[1],readBuff[2],readBuff[3],readBuff[4],readBuff[5],readBuff[6],readBuff[7],readBuff[8],readBuff[9],readBuff[10],readBuff[11],readBuff[12],readBuff[13],readBuff[14]);
        printf("DEBUG: %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c\n\n",readBuff[0],readBuff[1],readBuff[2],readBuff[3],readBuff[4],readBuff[5],readBuff[6],readBuff[7],readBuff[8],readBuff[9],readBuff[10],readBuff[11],readBuff[12],readBuff[13],readBuff[14]);
        cs_deselect();
        //RFM69_writeReg(RFM69_REG_IRQFLAGS2, 0b00010000);
        sleep_ms(500);
        return 0;
    }

    cs_deselect();
    // Clean FIFO, we got our packet...
    //RFM69_writeReg(RFM69_REG_IRQFLAGS2, 0b00010000);
    sleep_ms(500);
    setOpMode(false,false,false,RFM69_MODE_SLEEP);
    return payloadlen;
}
