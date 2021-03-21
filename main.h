//
// Created by mike on 27/02/2021.
//

#ifndef RADIO_WEATHER_MAIN_H

#define RADIO_WEATHER_MAIN_H

enum radio_module {
    RFM69 = 1, // rfm69hwc packet radio = These modules are cheaper, don't transmit as far
    RFM96 = 2 // rfm9x/5/6/7/8 LoRa radio module = These modules transmit further (code working from daveake/pico-tracker)
};

enum radio_mode {
    BROADCAST_MODE = 1, // Broadcast Measurements (send)
    RECEIVE_MODE = 2 // Receive Measurements (receive)
};

struct RW_SETTINGS
{
    uint8_t mode;
    bool debug;
    uint8_t radio_module;
    uint8_t networkID;
    uint8_t nodeID;
    float frequency;
    int counter;
} radio_weather;

#endif //RADIO_WEATHER_MAIN_H
