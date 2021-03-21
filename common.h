//
// Created by mike on 07/03/2021.
//

#ifndef RADIO_WEATHER_COMMON_H
#define RADIO_WEATHER_COMMON_H

#define PIN_DEBUG_LED   15

void report_error();
void debug_led_init();

struct SENSINFO
{
    int32_t bme280_temp;
    int32_t bme280_humd;
    int32_t bme280_pres;
};

#endif //RADIO_WEATHER_COMMON_H
