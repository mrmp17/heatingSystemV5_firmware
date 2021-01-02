//
// Created by matej on 11/10/2020.
//

#ifndef HEATINGSYSTEMV5_HARDWARE_H
#define HEATINGSYSTEMV5_HARDWARE_H


#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "adc.h"
#include "gpio.h"
#include "rtc.h"

#define STATE_TRACE true
#define ENABLE_SLEEP true

#define ADC_NUM_CH 5

#define HANDLER_PERIOD 20

#define ADC_MAX_VAL 4095
#define ADC_REF 2500
#define MCU_SPLY 2500
#define VREF_INT 1224
#define VREF_INT_RAW 2005 //adc should return this raw value when measuring internal reference voltage, if Vcc is 2500mV
#define ADC_VBAT_KOEF 2

#define HTR_RESISTANCE 2900 //mOhm
//ADC_buffer array indexes
#define ADC_CC1 0
#define ADC_CC2 1
#define ADC_VBAT 2
#define ADC_UDP 3
#define ADC_VREF 4

#define V5V3A_LCC_MAX 100 //max voltage at CC pin that should be unconnected (when detecting 5V 3A adapter)
#define V5V3A_HCC_MIN 1515 //min voltage at CC pin that should be pulled up by source (when detecting 5V 3A adapter)
#define V5V3A_HCC_MAX 1818 //max voltage at CC pin that should be pulled up by source (when detecting 5V 3A adapter)
#define PORT_EMPTY_CCMAX 50 //alow max 50mV on CC pins to detect empty connetor
#define UDP_HTR_MIN 1188 //USB data+ pin range min for heater detection
#define UDP_HTR_MAX 1313 //USB data+ pin range max for heater detection

//bat SOC definitions
#define SOC_0to10 0
#define SOC_10to40 1
#define SOC_40to70 2
#define SOC_70to100 3
#define SOC_DEAD 4
#define SOC_HYST 25 //mV
#define BAT_DIV_TCONST 60 //RC filter on battery divider takes 60ms to settle
#define SOC_MAX_INTERVAL 10000 //SOC handler requests SOC measurement conditions after this many ms
#define SOC_RTC_NUM 10 //allow some idle time for SOC measurement every __ RTC events


#define BAT_RINT 90 //internal resistance in milliohms (todo: set to correct value, this includes test cables)

#define WAKE_SOURCE_BTN 0
#define WAKE_SOURCE_RTC 1
#define RTC_TICKS_PER_S 2313
#define RTC_WAKE_TIME 4000

#define CHRG_STAT_IDLE 0
#define CHRG_STAT_CHARGING 1
#define CHRG_STAT_ERROR 2

//heating power defines
#define HEAT_LOW 1350 //mW
#define HEAT_MED 1900 //mW
#define HEAT_HIGH 2400 //mW
#define HEAT_MAX 3500 //mW use only to preheat. not possible at low battery voltages

//cycles correspond to time between handler calls
#define BUTTON_DBOUNCE_CYCLES 2
#define BUTTON_SHORPTESS_CYCLES 0
#define BUTTON_LONGPRESS_CYCLES 80
#define BUTTON_SUPERLONG_CYCLES 350

#define WAIT_LONGPRESS (HANDLER_PERIOD*BUTTON_LONGPRESS_CYCLES)+50

#define WAIT_HEATER_TIMEOUT 200
#define PORT_NOT_EMPTY_WAIT 100 //wait to allow connected heater CC resistor divider settle down before recognising it as non-heater device



class Hardware {

public:
    explicit Hardware(bool *btn_interrupt_flag_pointer);


    void init(); //initializes hardware


    void debug_print(const char *format, ...);

    //reset parameter in handlers resets state machines and timing variables. Call handlers with reset=1 right after wake-up from sleep
    //TODO: make separate reset functions
    void main_handler(); //call this every x ms
    void soft_pwm_handler(bool reset); //call this in handler
    void chrg_stat_handler(bool reset);
    void button_handler(bool reset);
    void SOC_handler(bool reset);
    void analog_handler(bool reset);

    void start_ADC(); //starts ADC conversions.
    void stop_ADC(); //stops ADC conversions
    uint8_t chrg_stat(); //gets charger status (see status defines)
    bool is_htr_connected(); //checks if heater cable is connected
    void set_heating(uint16_t value); //enables heating/sets heating power. this is ABSOLUTE HEATING POWER - PWM DUTY
    bool is_charging(); //checks if battery is charging
    bool is_SOC_request_meas();
    void confirm_SOC_request_meas();
    uint32_t get_confirm_SOC_request_meas_time();
    uint8_t get_SOC();
    bool is_sply_5V3A(); //checks if 5V 3A type-C compatible power supply - charger is connected
    bool is_port_empty(); //returns true if nothing is connected to USB-C connector
    bool is_button_longpress(); //returns longpress flag and clears it
    bool is_button_shortpress(); //returns longpress flag and clears it
    bool is_button_superlongpress();
    void clear_button_flags();

    void sleep(); //prepare and enter sleep, configures after sleep
    uint8_t wake_source(); //returns wake-up source


    //low level HW functions (used mostly by higher level functions in this class)
    void set_charging(bool state); //turns charging ON or OFF
    void set_pwr_mosfet(bool state); //heating mosfet low level controll
    void set_button_sply(bool state); //turns on/off button supply voltage (set to ON an forget - only HW rev 1.0)
    uint16_t get_CC1_volt(); //gets CC1 pin voltage
    uint16_t get_CC2_volt(); //gets CC2 pin voltage
    uint16_t get_vbat(); //gets battery voltage
    uint16_t get_vref();
    uint16_t get_real_ADC_ref(); //reutrns real ADC reference, reverse measured via internal voltage reference
    uint16_t get_UDP_volt(); //gets USB data positive voltage
    void set_vbat_sply(bool state); //turns on/off vbat resistor divider supply (GND)
    bool is_vbat_sply_on();
    void led_ctrl(uint8_t led, bool state); //turns specified led ON or OFF
    bool chrg_pgd(); //gets input power good status from charger (true if ~5V present on VBUS)
    bool is_pwr_mosfet_on(); //check if power mosfet is connecting battery voltage to VBUS
    void set_htr_det(bool state); //enable or disable heater detect pullup resistor (NOT USED)
    bool is_htr_det_on();
    bool get_button_state();
    bool get_button_dbncd_state(); //returns debouned state of button
    void set_default_input_cur(); //sets default VBUS input current limit ~1A
    void set_max_input_cur(); //sets maximum VBUS input current limit ~3A
    uint16_t rel_htr_pwr(uint16_t power_mw); //returns relative heater power in %, required to get requested power ouptut
    void trace(uint16_t state_num);

    void config_clk_wake();
    void config_gpio_sleep(); //not used currently //todo: not updated
    void config_gpio_wake(); //not used currently //todo: not updated

    uint32_t handler_counter = 0; //increments every time handler executes


    uint32_t ADC_buffer[ADC_NUM_CH] = {0}; //ADC buffer (filled by DMA)

    uint32_t led_flip_time = 0; // time of last LED flip









private:

    //const uint16_t soc_thr [5] = {3390, 3750, 3910, 2800, 3000}; //thresholds for 10%, 40%, 70%, LOW and LOW_RELEASE (NO LOAD)
    const uint16_t soc_thr [5] = {3256, 3612, 3886, 2800, 3000}; //thresholds for 10%, 40%, 70%, LOW and LOW_RELEASE (NO LOAD) (new calibration from data)
    bool htr_det_state = false; //keeps state of heater detect functionality
    uint32_t last_pwrmos_flip = 0; //keeps the time of last power mosfet switch (used for battery voltage measurements)
    uint32_t last_divider_enable = 0; //keeps the time of last batter voltage divider turn-on event
    uint32_t request_SOC_meas_confirm_time = 0; //time at which code granted SOC measurement conditions
    uint8_t SOC_val = SOC_10to40; //keeps battery SOC state
    bool request_SOC_meas = false; //soc handler requests SOC measurement conditions (listed in SOC handler) by seting this flag
    uint16_t current_htr_pwr = 0;
    uint8_t chrg_stat_ = 0;
    uint8_t wakeup_src = 0;
    bool pwr_mos_state = false; //keeps the state of power mosfet

    uint16_t valid_raw_ADC[ADC_NUM_CH] = {0};

    uint8_t soc_loop_ctrl = 1;
    uint32_t lastSOCcalc = 0;

    // heater
    uint8_t heater_soft_pwm_cnt = 0;
    bool heater_wait_SOC = false;

    // charge state
    uint32_t charge_pin_last_edge_time = 0;
    bool charge_last_pin_state = false;
    const uint32_t charge_error_interval = 1000;
    uint32_t charge_blink_counter = 0; //counts valid on-after-another blinks

    // button
    // TODO: button could be a separate class - more reusable
    uint8_t button_loop_ctrl = 0;
    uint32_t button_counter = 0;
    bool shortPressFlag = false;
    bool longPressFlag = false;
    bool superLongPressFlag = false;
    bool buttonDebouncedState = false;
    bool *btn_int_flag_pointer;

};


#endif //HEATINGSYSTEMV5_HARDWARE_H
