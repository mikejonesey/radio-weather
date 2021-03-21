//
// Created by mike on 06/03/2021.
//

#ifndef RADIO_WEATHER_RFM69HCW_SPI_H
#define RADIO_WEATHER_RFM69HCW_SPI_H

#define RMF69HCW_VERSION 0x24

struct RFM69_SETTINGS
{
    uint8_t mode;
    uint8_t networkID;
    uint8_t nodeID;
    uint8_t powerLevel;
    bool SequencerOff;
    bool ListenOn;
    bool use_tx_boost;
    bool use_rx_boost;
    uint8_t rssi_threshold;
    bool filter_address;
    bool filter_broadcast;
    bool crc_check;
    uint16_t bitrate;
	uint8_t rxbw_maint;
	uint8_t rxbw_exp;
	uint8_t max_payload_size;
};

struct RFM69_SETTINGS radio_settings;

enum rfm69_register {
    RFM69_REG_FIFO = 0x00, // FIFO read/write access
    RFM69_REG_OPMODE = 0x01, // Operating modes of the transceiver
    RFM69_REG_DATAMODUL = 0x02, // Data operation mode and Modulation settings
    RFM69_REG_BITRATEMSB = 0x03, // Bit Rate setting, Most Significant Bits
    RFM69_REG_BITRATELSB = 0x04, // Bit Rate setting, Least Significant Bits
    RFM69_REG_FDEVMSB = 0x05, // Frequency Deviation setting, Most Significant Bits
    RFM69_REG_FDEVLSB = 0x06, // Frequency Deviation setting, Least Significant Bits
    RFM69_REG_FRFMSB = 0x07, // RF Carrier Frequency, Most Significant Bits
    RFM69_REG_FRFMID = 0x08, // RF Carrier Frequency, Intermediate Bits
    RFM69_REG_FRFLSB = 0x09, // RF Carrier Frequency, Least Significant Bits
    RFM69_REG_OSC1 = 0x0A, // RC Oscillators Settings
    RFM69_REG_AFCCTRL = 0x0B, // AFC control in low modulation index situations
    RFM69_REG_Reserved0C = 0x0C, // -
    RFM69_REG_LISTEN1 = 0x0D, // Listen Mode settings
    RFM69_REG_LISTEN2 = 0x0E, // Listen Mode Idle duration
    RFM69_REG_LISTEN3 = 0x0F, // Listen Mode Rx duration
    RFM69_REG_VERSION = 0x10, //
    RFM69_REG_PALEVEL = 0x11, // PA selection and Output Power control
    RFM69_REG_PARAMP = 0x12, // Control of the PA ramp time in FSK mode
    RFM69_REG_OCP = 0x13, // Over Current Protection control
    RFM69_REG_Reserved14 = 0x14,  // -
    RFM69_REG_Reserved15 = 0x15,  // -
    RFM69_REG_Reserved16 = 0x16,  // -
    RFM69_REG_Reserved17 = 0x17,  // -
    RFM69_REG_LNA = 0x18, // LNA settings
	RFM69_REG_RXBW = 0x19, // Channel Filter BW Control
	RFM69_REG_AFCBW = 0x1A, // Channel Filter BW control during the AFC routine
	RFM69_REG_OOKPEAK = 0x1B, // OOK demodulator selection and control in peak mode
	RFM69_REG_OOKAVG = 0x1C, // Average threshold control of the OOK demodulator
	RFM69_REG_OOKFIX = 0x1D, // Fixed threshold control of the OOK demodulator
	RFM69_REG_AFCFEI = 0x1E, // AFC and FEI control and status
	RFM69_REG_AFCMSB = 0x1F, // MSB of the frequency correction of the AFC
	RFM69_REG_AFCLSB = 0x20, // LSB of the frequency correction of the AFC
	RFM69_REG_FEIMSB = 0x21, // MSB of the calculated frequency error
	RFM69_REG_FEILSB = 0x22, // LSB of the calculated frequency error
	RFM69_REG_RSSICONFIG = 0x23, // RSSI-related settings
	RFM69_REG_RSSIVALUE = 0x24, // RSSI value in dBm
	RFM69_REG_DIOMAPPING1 = 0x25, // Mapping of pins DIO0 to DIO3
	RFM69_REG_DIOMAPPING2 = 0x26, // Mapping of pins DIO4 and DIO5, ClkOut frequency
	RFM69_REG_IRQFLAGS1 = 0x27, // Status register: PLL Lock state, Timeout, RSSI > Threshold...
	RFM69_REG_IRQFLAGS2 = 0x28, // Status register: FIFO handling flags...
	RFM69_REG_RSSITHRESH = 0x29, // RSSI Threshold control
	RFM69_REG_RXTIMEOUT1 = 0x2A, // Timeout duration between Rx request and RSSI detection
	RFM69_REG_RXTIMEOUT2 = 0x2B, // Timeout duration between RSSI detection and PayloadReady
	RFM69_REG_PREAMBLEMSB = 0x2C, // Preamble length, MSB
	RFM69_REG_PREAMBLELSB = 0x2D, // Preamble length, LSB
	RFM69_REG_SYNCCONFIG = 0x2E, // Sync Word Recognition control
	RFM69_REG_SYNCVALUE1 = 0x2F, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE2 = 0x30, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE3 = 0x31, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE4 = 0x32, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE5 = 0x33, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE6 = 0x34, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE7 = 0x35, // Sync Word bytes, 1 through 8
	RFM69_REG_SYNCVALUE8 = 0x36, // Sync Word bytes, 1 through 8
	RFM69_REG_PACKETCONFIG1 = 0x37, // Packet mode settings
	RFM69_REG_PAYLOADLENGTH = 0x38, // Payload length setting
	RFM69_REG_NODEADRS = 0x39, // Node address
	RFM69_REG_BROADCASTADRS = 0x3A, // Broadcast address
	RFM69_REG_AUTOMODES = 0x3B, // Auto modes settings
	RFM69_REG_FIFOTHRESH = 0x3C, // Fifo threshold, Tx start condition
	RFM69_REG_PACKETCONFIG2 = 0x3D, // Packet mode settings
	RFM69_REG_AESKEY1 = 0x3E, // 16 bytes of the cypher key
	RFM69_REG_AESKEY2 = 0x3F, // 16 bytes of the cypher key
	RFM69_REG_AESKEY3 = 0x40, // 16 bytes of the cypher key
	RFM69_REG_AESKEY4 = 0x41, // 16 bytes of the cypher key
	RFM69_REG_AESKEY5 = 0x42, // 16 bytes of the cypher key
	RFM69_REG_AESKEY6 = 0x43, // 16 bytes of the cypher key
	RFM69_REG_AESKEY7 = 0x44, // 16 bytes of the cypher key
	RFM69_REG_AESKEY8 = 0x45, // 16 bytes of the cypher key
	RFM69_REG_AESKEY9 = 0x46, // 16 bytes of the cypher key
	RFM69_REG_AESKEY10 = 0x47, // 16 bytes of the cypher key
	RFM69_REG_AESKEY11 = 0x48, // 16 bytes of the cypher key
	RFM69_REG_AESKEY12 = 0x49, // 16 bytes of the cypher key
	RFM69_REG_AESKEY13 = 0x4A, // 16 bytes of the cypher key
	RFM69_REG_AESKEY14 = 0x4B, // 16 bytes of the cypher key
	RFM69_REG_AESKEY15 = 0x4C, // 16 bytes of the cypher key
	RFM69_REG_AESKEY16 = 0x4D, // 16 bytes of the cypher key
	RFM69_REG_TEMP1 = 0x4E, // Temperature Sensor control
	RFM69_REG_TEMP2 = 0x4F, // Temperature readout
	RFM69_REG_TESTLNA = 0x58, // Sensitivity boost
	RFM69_REG_TESTPA1 = 0x5A,  // High Power PA settings
	RFM69_REG_TESTPA2 = 0x5C,  // High Power PA settings
	RFM69_REG_TESTDAGC = 0x6F, // Fading Margin Improvement
	RFM69_REG_TestAfc = 0x71, // AFC offset for low modulation index AFC
	RFM69_REG_Test = 0x50 // Internal test registers
};

enum RFM69_MODE {
    RFM69_MODE_SLEEP = 0b000,
    RFM69_MODE_STDBY = 0b001,
    RFM69_MODE_FS = 0b010,
    RFM69_MODE_TX = 0b011,
    RFM69_MODE_RX = 0b100,
};

int rmf69_init(uint8_t net_id, uint8_t node_id, float Frequency);
uint8_t rfm69_sendFrame(uint16_t toAddress, const uint8_t *data, uint8_t len, bool requestACK, bool sendACK);
uint8_t rfm69_read(uint8_t *recv_from, uint8_t *recv_data);

#endif //RADIO_WEATHER_RFM69HCW_SPI_H
