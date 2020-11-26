//
// Created by matej on 08/11/2020.
//

#ifndef HEATINGSYSTEMV5_INDICATOR_H
#define HEATINGSYSTEMV5_INDICATOR_H


#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "Hardware.h"
#define NUM_LEDS 3

#define SLOW_ON 500
#define SLOW_OFF 1000
#define FAST_ON 100
#define FAST_OFF 100

#define SINGLE_PRE 400
#define SINGLE_ON 1500
#define SINGLE_ON_SHORT 150
#define SINGLE_POST 50

#define MODE_OFF 0
#define MODE_SLOW 1
#define MODE_FAST 2
#define MODE_SOLID 3
#define MODE_SINGLE 4

#define MAX_HANDLER_dT (FAST_ON + FAST_OFF) / 2

class Indicator {

public:

    Indicator(Hardware *obj);

    void led_handler(bool reset); //call this at least once every 10ms

    void slow_blink(uint8_t led);
    void fast_blink(uint8_t led);
    void solid_on(uint8_t led);
    void solid_off(uint8_t led);
    void stop_blink(); //calls solid_off for all leds
    void single(uint8_t led, bool blink, bool shrt); //one shot event with blinking and short blink flag
    bool is_single_done(uint8_t led);



private:

    void set_led(uint8_t led, bool state);

    uint8_t ledModes[NUM_LEDS] = {0};
    bool singleDone[NUM_LEDS] = {true, true, true};
    bool singleBlinking[NUM_LEDS] = {0};
    bool singleShort[NUM_LEDS] = {0};

    uint32_t slowRamp = 0;
    uint32_t fastRamp = 0;

    Hardware *hw_driver;




};


#endif //HEATINGSYSTEMV5_INDICATOR_H
