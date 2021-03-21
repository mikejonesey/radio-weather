//
// Created by mike on 28/02/2021.
//

#ifndef RADIO_WEATHER_BME280_I2C_H

#define RADIO_WEATHER_BME280_I2C_H

// register address and register info grabbed from:
// https://github.com/pimoroni/bme280-python/blob/master/library/bme280/__init__.py
// https://community.bosch-sensortec.com/t5/Knowledge-base/BME280-Sensor-Data-Interpretation/ta-p/13912
// https://download.mikroe.com/documents/datasheets/BME280.pdf

enum bme280_register {
    BME280_REG_CHIP_ID = 0xD0, // The “id” register contains the chip identification numberchip_id[7:0], which is 0x60. This number can be read as soon as the device finished the power-on-reset.
    BME280_REG_RESET = 0xE0, // The “reset” register contains the soft reset word reset[7:0]. If the value 0xB6 is writtento the register, the device is reset using the complete power-on-reset procedure. Writing other values than 0xB6 has no effect. The readout value is always 0x00.
    BME280_REG_CTRL_HUM = 0xF2, // The “ctrl_hum” register sets the humidity data acquisition options of the device.Changes to this register only become effective after a write operation to “ctrl_meas”.
    BME280_REG_STATUS = 0xF3, // The “status” register contains two bits which indicate the status of the device.
    BME280_REG_CTRL_MEAS = 0xF4, // The “ctrl_meas” register sets the pressure and temperature data acquisition options of the device. The register needs to be written after changing “ctrl_hum”for the changes to become effective.
    BME280_REG_CONFIG = 0xF5, // The “config” register sets therate, filter and interface options of the device. Writes to the “config” register in normal mode may be ignored.In sleep mode writes are not ignored.
    BME280_REG_PRESS_MSB = 0xF7, // The “press” register contains the raw pressure measurement output data up[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_PRESS_LSB = 0xF8, // The “press” register contains the raw pressure measurement output data up[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_PRESS_XLSB = 0xF9, // The “press” register contains the raw pressure measurement output data up[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_TEMP_MSB = 0xFA, // The “temp” register contains the raw temperature measurement output data ut[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_TEMP_LSB = 0xFB, // The “temp” register contains the raw temperature measurement output data ut[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_TEMP_XLSB = 0xFC, // The “temp” register contains the raw temperature measurement output data ut[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_HUM_MSB = 0xFD, // The “hum” register contains the raw humidity measurement output data ut[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_HUM_LSB = 0xFE, // The “hum” register contains the raw humidity measurement output data ut[19:0]. For details on how to read out the pressure and temperature information from the device, please consult chapter 4.
    BME280_REG_CALIBRATION = 0x88, // 0x88...0xA1 = dig_T & dig_P
    BME280_REG_CALIBRATION2 = 0xE1 // 0xE1...0xE7 = dig_H
};

enum bme280_oversampling {
    BME280_OVERSAMPLING_NONE = 0b000,
    BME280_OVERSAMPLING_X1 = 0b001,
    BME280_OVERSAMPLING_X2 = 0b010,
    BME280_OVERSAMPLING_X4 = 0b011,
    BME280_OVERSAMPLING_X8 = 0b100,
    BME280_OVERSAMPLING_X16 = 0b101
};

enum BME280_SPI {
    BME280_SPI_NO = 0b00,
    BME280_SPI_YES = 0b01,
};

enum BME280_FILTER {
    BME280_FILTER_OFF = 0b000,
    BME280_FILTER_2 = 0b001,
    BME280_FILTER_4 = 0b010,
    BME280_FILTER_8 = 0b011,
    BME280_FILTER_16 = 0b100,
};

// 1.8 μA @ 1 Hz humidity and temperature
// 2.8 μA @ 1 Hz pressure and temperature
// 3.6 μA @ 1 Hz humidity, pressure and temperature
// 0.1 μA in sleep mode
enum BME280_MODE {
    BME280_MODE_NORMAL = 0b11,
    BME280_MODE_FORCED = 0b01,
    BME280_MODE_SLEEP = 0b00
};

enum bme280_standby {
    BME280_STANDBY_0_5 = 0b000,
    BME280_STANDBY_62_5 = 0b001,
    BME280_STANDBY_125 = 0b010,
    BME280_STANDBY_250 = 0b011,
    BME280_STANDBY_500 = 0b100,
    BME280_STANDBY_1000 = 0b101,
    BME280_STANDBY_10 = 0b110,
    BME280_STANDBY_20 = 0b111
};

uint8_t bme280_init();
bool isMeasuring(void);
int bme280_read(int32_t *temp, int32_t *humd, int32_t *pres);
uint8_t bme280_gosleep();
uint8_t bme280_wakeup();
uint8_t bme280_forced();

#endif //RADIO_WEATHER_BME280_I2C_H
