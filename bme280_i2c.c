//
// Created by mike on 28/02/2021.
//

#include <pico/printf.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bme280_i2c.h"
#include "common.h"

#define I2C_PORT i2c1
#define PIN_SDA   2
#define PIN_SCL   3
//#define PIN_NA    4

//////////////////////////////////////////////////
// BME280 Settings
//////////////////////////////////////////////////

// https://community.bosch-sensortec.com/t5/Knowledge-base/BME280-Sensor-Data-Interpretation/ta-p/13912
// Oversampling can be enabled during the measurement, which can reduce noise but also consumes more power. Different sensors have different OSRs (oversampling rate).
enum bme280_oversampling bme280_temperature_oversampling = BME280_OVERSAMPLING_X1;
enum bme280_oversampling bme280_pressure_oversampling = BME280_OVERSAMPLING_X1;
enum bme280_oversampling bme280_humidity_oversampling = BME280_OVERSAMPLING_X1;

// Mode (normal, forced or sleep)
enum BME280_MODE bme280Mode = BME280_MODE_NORMAL;

// Sleep time whilst in normal mode
enum bme280_standby bme280Standby = BME280_STANDBY_1000;

//////////////////////////////////////////////////
// Global Vars
//////////////////////////////////////////////////

// pimoroni breakout i2c bme280 (addr: 0x76 or 0x77)
uint8_t I2C_BME280_ADDR = 0x76; // default address

// bme280 calib
int32_t t_fine;
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

//////////////////////////////////////////////////
// Functions begin...
//////////////////////////////////////////////////

static int read_registers(uint8_t reg, uint8_t *readBuff, uint16_t len) {
    int PIC_READ, PIC_WRITE;
    // Poke Registry to Read
    PIC_WRITE = i2c_write_blocking(I2C_PORT, I2C_BME280_ADDR, &reg, 1, true);
    if(PIC_WRITE != 1 || PIC_WRITE == PICO_ERROR_GENERIC){
        // no write, bad address or device not present
        report_error();
        return 1;
    }
    sleep_ms(10);
    // READ
    PIC_READ = i2c_read_blocking(I2C_PORT, I2C_BME280_ADDR, readBuff, len, false);
    if(PIC_READ != len || PIC_READ == PICO_ERROR_GENERIC) {
        // no data returned, or bad address, or device not present
        report_error();
        return 1;
    }
    sleep_ms(10);
    return 0;
}

static int write_registers(uint8_t reg, uint8_t data) {
    uint8_t writeBuff[2] = {reg, data};
    int PIC_WRITE;
    PIC_WRITE = i2c_write_blocking(I2C_PORT, I2C_BME280_ADDR, writeBuff, 2, true);
    if(PIC_WRITE != 2 || PIC_WRITE == PICO_ERROR_GENERIC){
        // no write, bad address or device not present
        report_error();
        return 1;
    }
    sleep_ms(10);
    return 0;
}

static int write_byte_registers(uint8_t reg) {
    uint8_t writeBuff[1] = {reg};
    int PIC_WRITE;
    PIC_WRITE = i2c_write_blocking(I2C_PORT, I2C_BME280_ADDR, writeBuff, 1, true);
    if(PIC_WRITE != 1 || PIC_WRITE == PICO_ERROR_GENERIC){
        // no write, bad address or device not present
        report_error();
        return 1;
    }
    sleep_ms(10);
    return 0;
}

bool isMeasuring(void) {
    int rc;
    uint8_t readBuff[32];
    // BitField('measuring', 0b00001000),  # 1 when conversion is running
    // BitField('im_update', 0b00000001),  # 1 when NVM data is being copied
    rc = read_registers(BME280_REG_STATUS, readBuff, 1);
    if(rc != 0) {
        report_error();
        return 1; // not status but say busy when err
    }
    // Bit 3 : measuring[0] : Automatically set to ‘1’ whenever a conversion is
    //running and back to ‘0’ when the results have been
    //transferred to the data registers.
    // Bit 0 : im_update[0] : Automatically set to ‘1’ when the NVM data are being
    //copied to image registers and back to ‘0’ when the
    //copying is done. The data are copied at power-on-
    //reset and before every conversion.
    bool checkMeasureBit = ((0x8 & readBuff[0])>>3) != 0;
    bool checkUpdateBit = (0x1 & readBuff[0]) != 0;
    if (checkMeasureBit || checkUpdateBit){
        return 1;
    }else {
        return 0;
    }
    //return (readBuff[0] & (1 << 0)) != 0;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) dig_P6);
    var2 = var2 + ((var1 * ((int32_t) dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t) dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t) dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t) (((int32_t) 1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t) var1);
    else
        p = (p / (uint32_t) var1) * 2;

    var1 = (((int32_t) dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t) (p >> 2)) * ((int32_t) dig_P8)) >> 13;
    p = (uint32_t) ((int32_t) p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
            >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t) 76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t) dig_H4) << 20) - (((int32_t) dig_H5) * v_x1_u32r)) +
                   ((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) dig_H6)) >> 10) * (((v_x1_u32r *
                                                                                                  ((int32_t) dig_H3))
            >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
                                                 ((int32_t) dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t) (v_x1_u32r >> 12);
}

int bme280_read(int32_t *temp, int32_t *humd, int32_t *pres){
    int rc;
    uint8_t readBuff[32];

    //////////////////////////////////////////////////
    // Read Data
    //////////////////////////////////////////////////

    // To read out data after a conversion, it is strongly recommended
    // to use a burst read and not address every register individually.
    // This will prevent a possible mix-up of bytes belonging to
    // different measurements and reduce interface traffic.
    // Note that in I²C mode, even when pressure was not measured,
    // reading the unused registers is faster than reading
    // temperature and humidity data separately.
    //
    // Data readout is done by starting a burst read from 0xF7 to 0xFC
    // (temperature and pressure) or from 0xF7 to 0xFE (temperature, pressure and humidity).
    // The data are read out in an unsigned 20-bit format both for
    // pressure and for temperature and in an unsigned 16-bit
    // format for humidity.

    // Burst read 8 bits starting from 0xF7
    rc = read_registers(BME280_REG_PRESS_MSB, readBuff, 8);
    if(rc != 0) {
        report_error();
        return 1;
    }

    // Raw BitField to uint32
    // press msb 7:0 (19:12) | press lsb 7:0 (11:4) | press xlsb 7:4 (3:0) - always 20 bit when using filter.
    // temp msb 7:0 (19:12) | temp lsb 7:0 (11:4) | temp xlsb 7:4 (3:0) - always 20 bit when using filter.
    // hum msb 7:0 (15:8) | hum lsb 7:0 (7:0) - always 16 bit
    *pres = ((uint32_t) readBuff[0] << 12) | ((uint32_t) readBuff[1] << 4) | (readBuff[2] >> 4);
    *temp = ((uint32_t) readBuff[3] << 12) | ((uint32_t) readBuff[4] << 4) | (readBuff[5] >> 4);
    *humd = (uint32_t) readBuff[6] << 8 | readBuff[7];

    // Raw uint32 to interpreted (compensated) uint32
    *pres = compensate_pressure(*pres);
    *temp = compensate_temp(*temp);
    *humd = compensate_humidity(*humd);

    //////////////////////////////////////////////////
    // END
    //////////////////////////////////////////////////

    return 0;
}

void read_compensation_parameters() {
    uint8_t buffer[26];

    read_registers(BME280_REG_CALIBRATION, buffer, 24);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    read_registers(BME280_REG_CALIBRATION2, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t) buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t) buffer[7];
}

uint8_t bme280_gosleep(){
    uint8_t rc;
    uint8_t data = ((bme280_temperature_oversampling << 5) | (bme280_pressure_oversampling << 2) | BME280_MODE_SLEEP);
    rc = write_registers(BME280_REG_CTRL_MEAS, data);
    if (rc != 0){
        report_error();
        return 1;
    }
    sleep_ms(100);
    return 0;
}

uint8_t bme280_forced(){
    uint8_t rc;
    uint8_t data = ((bme280_temperature_oversampling << 5) | (bme280_pressure_oversampling << 2) | BME280_MODE_FORCED);
    rc = write_registers(BME280_REG_CTRL_MEAS, data);
    if (rc != 0){
        report_error();
        return 1;
    }
    sleep_ms(100);
    return 0;
}

uint8_t bme280_wakeup(){
    uint8_t rc;
    uint8_t data = ((bme280_temperature_oversampling << 5) | (bme280_pressure_oversampling << 2) | bme280Mode);
    rc = write_registers(BME280_REG_CTRL_MEAS, data);
    if (rc != 0){
        report_error();
        return 1;
    }
    sleep_ms(100);
    return 0;
}

uint8_t bme280_init(){
    int rc;
    uint8_t writeBuff[8];
    uint8_t readBuff[32];

    //////////////////////////////////////////////////
    // bme280 Setup (2.6v,SDA,SCL,X,GND) 200 0.2MHz
    // bme default:  100KHz, documentation up to to 400KHz
    // Baudrate in Hz (e.g. 100kHz is 100000)
    //////////////////////////////////////////////////
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C); // SDA
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    sleep_ms(100);

    //////////////////////////////////////////////////
    // Get Chip ID
    //////////////////////////////////////////////////
    static uint8_t bme280_chip = 0x60;
    read_registers(BME280_REG_CHIP_ID, readBuff, 1);
    printf("\n\n----------\n");
    printf("Returned chip: 0x%x we wanted chip 0x%x\n",readBuff[0], bme280_chip);
    printf("----------\n\n");
    if(rc != 0 || readBuff[0] != bme280_chip){
        report_error();
        return 1;
    }

    //////////////////////////////////////////////////
    // Perform Reset...
    //////////////////////////////////////////////////
    rc = write_byte_registers(BME280_REG_RESET);
    if (rc != 0){
        report_error();
        return 1;
    }
    sleep_ms(200); // Sleep after reset

    //////////////////////////////////////////////////
    // Set Humidity Oversampling...
    //////////////////////////////////////////////////
    rc = write_registers(BME280_REG_CTRL_HUM, bme280_humidity_oversampling);
    if (rc != 0){
        report_error();
        return 1;
    }

    //////////////////////////////////////////////////
    // Set Mode... and temp & pressure oversampling
    //////////////////////////////////////////////////
    // https://download.mikroe.com/documents/datasheets/BME280.pdf
    // Bit 7, 6, 5 = osrs_t[2:0] = Controls oversampling of temperature data.
    // Bit 4, 3, 2 = osrs_p[2:0] = Controls oversampling of pressure data.
    // Bit 1, 0 = mode[1:0] = Controls the sensormode of the device. (00sleep 01 and 10 forced, 11 normal)
    uint8_t data = ((bme280_temperature_oversampling << 5) | (bme280_pressure_oversampling << 2) | bme280Mode);
    rc = write_registers(BME280_REG_CTRL_MEAS, data);
    if (rc != 0){
        report_error();
        return 1;
    }

    //////////////////////////////////////////////////
    // Set Config
    //////////////////////////////////////////////////
    // https://download.mikroe.com/documents/datasheets/BME280.pdf
    // Bit 7, 6, 5 = t_sb[2:0] = Controls inactive duration tstandbyin normal mode.
    // Bit 4, 3, 2 = filter[2:0] = Controls the time constant of the IIR filter.
    // Bit 0 = spi3w_en[0] = Enables 3-wire SPI interface when set to ‘1’.
    data = ((bme280Standby << 5) | (BME280_FILTER_2 << 2) | BME280_SPI_NO);
    rc = write_registers(BME280_REG_CONFIG, data);
    if (rc != 0){
        report_error();
        return 1;
    }
    //////////////////////////////////////////////////
    // Get the calibration settings
    //////////////////////////////////////////////////

    read_compensation_parameters();

    //////////////////////////////////////////////////
    // END BME init
    //////////////////////////////////////////////////
    return 0;
}