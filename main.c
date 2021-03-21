#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "main.h"
#include "common.h"
// reboot
#include "pico/bootrom.h"
// button check...
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
// BME280 i2c
#include "bme280_i2c.h"
// LoRa Radio
#include "rfm69hcw_spi.h"
#include "lora_spi.h"

struct SENSINFO BMEINFO;

#define PIN_LED   25

static void reboot(){
    uint32_t usb_activity_gpio_pin_mask = 0;
    uint32_t disable_interface_mask = 0;
    reset_usb_boot(usb_activity_gpio_pin_mask,disable_interface_mask);
}

static bool __no_inline_not_in_flash_func(check_button)() {
    const uint CS_PIN_INDEX = 1;

    // Must disable interrupts, as interrupt handlers may be in flash, and we
    // are about to temporarily disable flash access!
    uint32_t flags = save_and_disable_interrupts();

    // Set chip select to Hi-Z
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    // Note we can't call into any sleep functions in flash right now
    for (volatile int i = 0; i < 1000; ++i);

    // The HI GPIO registers in SIO can observe and control the 6 QSPI pins.
    // Note the button pulls the pin *low* when pressed.
    bool button_state = !(sio_hw->gpio_hi_in & (1u << CS_PIN_INDEX));

    // Need to restore the state of chip select, else we are going to have a
    // bad time when we return to code in flash!
    hw_write_masked(&ioqspi_hw->io[CS_PIN_INDEX].ctrl,
                    GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(flags);
    if(button_state)
        printf("button pressed...\n");
    return button_state;
}

void getSensorString(uint32_t bme280_temp, uint32_t bme280_humd, uint32_t bme280_pres, uint8_t *sensorString){
    // 32 bit ints into 8bit ints for transmission...

    // TEMP
    sensorString[0] = (uint8_t) (bme280_temp >> 24); // 0b 11111111 00000000 00000000 00000000
    sensorString[1] = (uint8_t) (bme280_temp >> 16); // 0b 00000000 11111111 00000000 00000000
    sensorString[2] = (uint8_t) (bme280_temp >> 8); // 0b 00000000 00000000 11111111 00000000
    sensorString[3] = (uint8_t) (bme280_temp); // 0b 00000000 00000000 00000000 11111111
    sensorString[4] = ',';

    // HUM
    sensorString[5] = (uint8_t) (bme280_humd >> 24); // 0b 11111111 00000000 00000000 00000000
    sensorString[6] = (uint8_t) (bme280_humd >> 16); // 0b 00000000 11111111 00000000 00000000
    sensorString[7] = (uint8_t) (bme280_humd >> 8); // 0b 00000000 00000000 11111111 00000000
    sensorString[8] = (uint8_t) (bme280_humd); // 0b 00000000 00000000 00000000 11111111
    sensorString[9] = ',';

    // PRES
    sensorString[10] = (uint8_t) (bme280_pres >> 24); // 0b 11111111 00000000 00000000 00000000
    sensorString[11] = (uint8_t) (bme280_pres >> 16); // 0b 00000000 11111111 00000000 00000000
    sensorString[12] = (uint8_t) (bme280_pres >> 8); // 0b 00000000 00000000 11111111 00000000
    sensorString[13] = (uint8_t) (bme280_pres); // 0b 00000000 00000000 00000000 11111111
    sensorString[14] = '\0';
}

void formatRecvData(uint8_t *recv_data, uint32_t *bme280_temp, uint32_t *bme280_humd, uint32_t *bme280_pres){
    // return int32 full values
    *bme280_temp = (uint32_t) recv_data[0] <<24 | (uint32_t) recv_data[1] <<16 | (uint32_t) recv_data[2] <<8 | (uint32_t) recv_data[3];
    *bme280_humd = (uint32_t) recv_data[5] <<24 | (uint32_t) recv_data[6] <<16 | (uint32_t) recv_data[7] <<8 | (uint32_t) recv_data[8];
    *bme280_pres = (uint32_t) recv_data[10] <<24 | (uint32_t) recv_data[11] <<16 | (uint32_t) recv_data[12] <<8 | (uint32_t) recv_data[13];
}

int main() {
    radio_weather.mode = BROADCAST_MODE;
    radio_weather.debug = true;
    radio_weather.networkID = 1;
    //radio_weather.nodeID = 1;
    radio_weather.frequency = 433.100;
    radio_weather.radio_module = RFM69;

    if(radio_weather.mode==BROADCAST_MODE)
        radio_weather.nodeID = 2;
    else if(radio_weather.mode==RECEIVE_MODE)
        radio_weather.nodeID = 1;

    // testing sleep for battery support...
    //sleep_ms(2000);
    // Wait for debug terminal via minicom
    //sleep_ms(5000);

    stdio_init_all();

    // Setup LED for notification pico is working
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Setup Error LED for DEBUG
    if (radio_weather.debug){
        debug_led_init();
    }

    int32_t rc, bme280_temp, bme280_humd, bme280_pres;

    //////////////////////////////////////////////////
    // bme280 init
    //////////////////////////////////////////////////
    if(radio_weather.mode == BROADCAST_MODE){
        rc = bme280_init();
        if(rc != 0){
            // ERROR CONST
            gpio_put(PIN_LED, 1);
            report_error();
            return 1;
        }
    }

    //////////////////////////////////////////////////
    // RADIO MODULE INIT
    //////////////////////////////////////////////////
    if(radio_weather.radio_module == RFM69){
        // RMF69HCW INIT
        rc = rmf69_init(radio_weather.networkID, radio_weather.nodeID, radio_weather.frequency);
        if(rc != 0){
            // ERROR CONST
            gpio_put(PIN_LED, 1);
            report_error();
            reboot();
            return 1;
        }
    } else if(radio_weather.radio_module == RFM96){
        // LoRa init
        setup_lora(radio_weather.frequency, 1, "MNKWEATHER");
    }

    //////////////////////////////////////////////////
    // Start main loop
    //////////////////////////////////////////////////
    while (true) {
        if(radio_weather.mode == BROADCAST_MODE){
            // Sleep and Bleep when busy
            while (isMeasuring()){
                report_error();
                sleep_ms(100);
            }

            // Get Values from BME280
            rc = bme280_read(&bme280_temp, &bme280_humd, &bme280_pres);
            if(rc != 0){
                // ERROR LED
                gpio_put(PIN_LED, 1);
                sleep_ms(5000);
                gpio_put(PIN_LED, 0);
                sleep_ms(5000);
                continue;
            }

            if(radio_weather.debug){
                // Send Values Over USB
                radio_weather.counter++;
                printf("\nReading... %i\n", radio_weather.counter);
                printf("Pressure = %dPa\n", (int)bme280_pres);
                printf("Temp. = %.2fC\n", bme280_temp / 100.0);
                printf("Humidity = %.2f%%\n", bme280_humd / 1024.0);
            }

            // Send Values via RADIO
            BMEINFO.bme280_pres = bme280_pres;
            BMEINFO.bme280_humd = bme280_humd;
            BMEINFO.bme280_temp = bme280_temp;
            if(radio_weather.radio_module == RFM69){
                //uint8_t* send_data = "HelloWo123456789012345678901234567890123456789012345678901234567890";
                uint8_t send_data[15];
                getSensorString(bme280_temp, bme280_humd, bme280_pres, send_data);
                rfm69_sendFrame(1, send_data, 15, false, false);
            } else if(radio_weather.radio_module == RFM96){
                check_lora(&BMEINFO);
            }
        } else if(radio_weather.mode == RECEIVE_MODE){
            if(radio_weather.radio_module == RFM69){
                uint8_t recv_from;
                uint8_t recv_data[255];
                uint8_t recv_data_size = rfm69_read(&recv_from,recv_data);
                if (recv_data_size > 0){
                    formatRecvData(recv_data, &bme280_temp, &bme280_humd, &bme280_pres);
                    printf("Remote ID = %i\n", recv_from);
                    printf("Remote Pressure = %dPa\n", (int)bme280_pres);
                    printf("Remote Temp. = %.2fC\n", bme280_temp / 100.0);
                    printf("Remote Humidity = %.2f%%\n\n", bme280_humd / 1024.0);
                }
            } else if(radio_weather.radio_module == RFM96){
                check_lora(&BMEINFO);
            }

        } // END radio_weather.mode CHECK

        // loop finish bleep LED once on/off
        gpio_put(PIN_LED, 1);
        sleep_ms(200);
        gpio_put(PIN_LED, 0);

        // final sleep before starting loop again
        if(radio_weather.mode == BROADCAST_MODE){
            //sleep_ms(29000);
            sleep_ms(4000);
        }else if(radio_weather.mode == RECEIVE_MODE){
            //sleep_ms(10000);
            sleep_ms(4000);
        }

        // check button for reboot...
        if(check_button()) break;

        // break out after 5 loops...
        //if (counter > 14) break;
    }
    // reboot for next boot image...
    reboot();
}
