//
// Created by matej on 11/10/2020.
//

#ifndef HEATINGSYSTEMV5_HARDWARE_H
#define HEATINGSYSTEMV5_HARDWARE_H


#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "adc.h"

#define ADC_MAX_VAL 4095
#define ADC_REF 2500
#define MCU_SPLY 2500
#define ADC_VBAT_KOEF 2

#define HTR_RESISTANCE 2800 //mOhm
//ADC_buffer array indexes
#define ADC_CC1 0
#define ADC_CC2 1
#define ADC_VBAT 2

#define V5V3A_LCC_MAX 100 //max voltage at CC pin that should be unconnected (when detecting 5V 3A adapter)
#define V5V3A_HCC_MIN 1515 //min voltage at CC pin that should be pulled up by source (when detecting 5V 3A adapter)
#define V5V3A_HCC_MAX 1818 //max voltage at CC pin that should be pulled up by source (when detecting 5V 3A adapter)

//bat SOC definitions
#define SOC_0to10 0
#define SOC_10to40 1
#define SOC_40to70 2
#define SOC_70to100 3
#define SOC_DEAD 4

#define BAT_RINT 50 //internal resistance in miliohms



class Hardware {

public:
    Hardware();
    void init(); //initializes hardware


    void debug_print(const char *format, ...);

    void handler(); //call this every 10ms

    void start_ADC(); //starts ADC conversions.
    void stop_ADC(); //stops ADC conversions
    uint8_t chrg_stat(); //gets charger status (see status defines)
    bool is_htr_connected(); //checks if heater cable is connected
    bool set_heating(uint16_t value); //enables heating/sets heating power. returns false if heating is not possible
    bool is_charging(); //checks if battery is charging
    uint8_t get_SOC(); //returns estimated battery state. Valid values only: 0,33,66,99
    bool is_sply_5V3A(); //checks if 5V 3A type-C compatible power supply - charger is connected


    //low level HW functions (used mostly by higher level functions in this class)
    void set_charging(bool state); //turns charging ON or OFF
    void set_pwr_mosfet(bool state); //heating mosfet low level controll
    void set_button_sply(bool state); //turns on/off button supply voltage (set to ON an forget - only HW rev 1.0)
    uint16_t get_CC1_volt(); //gets CC1 pin voltage
    uint16_t get_CC2_volt(); //gets CC2 pin voltage
    uint16_t get_vbat(); //gets battery voltage
    void set_vbat_sply(bool state); //turns on/off vbat resistor divider supply (GND)
    void led_ctrl(uint8_t led, bool state); //turns specified led ON or OFF
    bool chrg_pgd(); //gets input power good status from charger (true if ~5V present on VBUS)
    bool is_pwr_mosfet_on(); //check if power mosfet is connecting battery voltage to VBUS
    void set_htr_det(bool state); //enable or disable heater detect pullup resistor (NOT USED)
    bool get_button_state();
    void set_default_input_cur(); //sets default VBUS input current limit ~1A
    void set_max_input_cur(); //sets maximum VBUS input current limit ~3A

    uint32_t handler_counter = 0; //increments every time handler executes


    uint32_t ADC_buffer[3] = {0}; //ADC buffer (filled by DMA)










private:

    const uint16_t soc_thr [5] = {3390, 3750, 3910, 2800, 3100}; //thresholds for 10%, 40%, 70%, LOW and LOW_RELEASE (NO LOAD)
    bool htr_det_state = 0; //keeps state of heater detect functionality


};


#endif //HEATINGSYSTEMV5_HARDWARE_H
