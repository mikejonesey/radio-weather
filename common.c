//
// Created by mike on 07/03/2021.
//

#include "pico/stdlib.h"
#include <hardware/gpio.h>
#include "common.h"
#include "main.h"

void report_error(){
    if(radio_weather.debug){
        // temp leaving in state on, so i can see when device has power quick
        gpio_put(PIN_DEBUG_LED, 0);
        gpio_put(PIN_DEBUG_LED, 1);
        sleep_ms(500);
        gpio_put(PIN_DEBUG_LED, 0);
        sleep_ms(500);
        gpio_put(PIN_DEBUG_LED, 1);
    }
}

void debug_led_init(){
    gpio_init(PIN_DEBUG_LED);
    gpio_set_dir(PIN_DEBUG_LED, GPIO_OUT);

    // Check debug LED is working... 4x quick bleep
    for(int i=0; i<10; i++){
        report_error();
    }
}