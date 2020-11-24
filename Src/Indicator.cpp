//
// Created by matej on 08/11/2020.
//

#include "Indicator.h"

Indicator::Indicator(Hardware *obj) {
  hw_driver = obj;
}

void Indicator::led_handler(bool reset) {
  static uint8_t loopCtrl[NUM_LEDS] = {0};
  static uint32_t ledTiming[NUM_LEDS] = {0};
  static uint32_t lastHandlerTime = 0;

  uint32_t timeNow = HAL_GetTick();

  if(reset){
    for(uint8_t n=0 ; n<NUM_LEDS ; n++){
      ledModes[n] = 0;
      slowRamp = 0;
      fastRamp = 0;
      lastHandlerTime = timeNow;
    }
  }
  //ramp generator
  if(timeNow-lastHandlerTime >= MAX_HANDLER_dT){
    lastHandlerTime = timeNow-MAX_HANDLER_dT;
    //something's f'd if this happens
  }
  //slow ramp
  if(slowRamp < SLOW_ON + SLOW_OFF){
    if(timeNow - lastHandlerTime > SLOW_ON + SLOW_OFF - slowRamp){
      slowRamp = (timeNow-lastHandlerTime) - (SLOW_ON + SLOW_OFF - slowRamp);
    }
    else{
      slowRamp += timeNow-lastHandlerTime;
    }
  }
  else{
    slowRamp = timeNow-lastHandlerTime;
  }
  //fast ramp
  if(fastRamp < FAST_ON + FAST_OFF){
    if(timeNow - lastHandlerTime > FAST_ON + FAST_OFF - fastRamp){
      fastRamp = (timeNow-lastHandlerTime) - (FAST_ON + FAST_OFF - fastRamp);
    }
    else{
      fastRamp += timeNow-lastHandlerTime;
    }
  }
  else{
    fastRamp = timeNow-lastHandlerTime;
  }

  lastHandlerTime = timeNow;


  //state machine run once for every led
  for(uint8_t n=0 ; n<NUM_LEDS ; n++){ //turn off directly if set to off
    if(ledModes[n] == MODE_OFF){
      set_led(n, false);
    }
    switch (loopCtrl[n]){

      case 0: //off
        singleDone[n] = true;
        if(ledModes[n] == MODE_SLOW || ledModes[n] == MODE_FAST){
          loopCtrl[n] = 2;
        }
        else if(ledModes[n] == MODE_SOLID){
          set_led(n, true);
          loopCtrl[n] = 1;
        }
        else if(ledModes[n] == MODE_SINGLE){
          ledTiming[n] = timeNow;
          singleDone[n] = false;
          set_led(n, false);
          loopCtrl[n] = 3;
        }
        break;

      case 1: //ful on
        singleDone[n] = true;
        //flow control
        if(ledModes[n] == MODE_OFF){
          set_led(n, false);
          loopCtrl[n] = 0;
        }
        else if(ledModes[n] == MODE_SLOW || ledModes[n] == MODE_FAST){
          set_led(n, false);
          loopCtrl[n] = 2;
        }
        else if(ledModes[n] == MODE_SINGLE){
          ledTiming[n] = timeNow;
          singleDone[n] = false;
          set_led(n, false);
          loopCtrl[n] = 3;
        }
        break;


      case 2: //blinking
        singleDone[n] = true;

        if(ledModes[n] == MODE_SLOW){
          set_led(n, slowRamp<=SLOW_ON);
        }
        else if(ledModes[n] == MODE_FAST){
          set_led(n, fastRamp<=FAST_ON);
        }

        //flow control
        if(ledModes[n] == MODE_OFF){
          set_led(n, false);
          loopCtrl[n] = 0;
        }
        else if(ledModes[n] == MODE_SOLID){
          set_led(n, true);
          loopCtrl[n] = 1;
        }
        else if(ledModes[n] == MODE_SINGLE){
          ledTiming[n] = timeNow;
          singleDone[n] = false;
          set_led(n, false);
          loopCtrl[n] = 3;
        }
        break;

      case 3: //single pre off
        if(timeNow - ledTiming[n] > SINGLE_PRE){
          ledTiming[n] = timeNow;
          if(!singleBlinking[n]){
            set_led(n, true);
          }
          loopCtrl[n] = 4;
        }
        else if(ledModes[n] == MODE_OFF){
          loopCtrl[n] = 0;
          ledTiming[n] = 0;
          set_led(n, false);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SOLID){
          loopCtrl[n] = 1;
          ledTiming[n] = 0;
          set_led(n, true);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SLOW || ledModes[n] == MODE_FAST){
          loopCtrl[n] = 2;
          ledTiming[n] = 0;
          set_led(n, false);
          singleDone[n] = true;
        }
        break;

      case 4: //single on

        if(singleBlinking[n]){
          set_led(n, fastRamp<=FAST_ON);
        }


        if(timeNow - ledTiming[n] > SINGLE_ON){
          ledTiming[n] = timeNow;
          loopCtrl[n] = 5;
          set_led(n, false);
        }
        else if(ledModes[n] == MODE_OFF){
          loopCtrl[n] = 0;
          ledTiming[n] = 0;
          set_led(n, false);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SOLID){
          loopCtrl[n] = 1;
          ledTiming[n] = 0;
          set_led(n, true);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SLOW || ledModes[n] == MODE_FAST){
          loopCtrl[n] = 2;
          ledTiming[n] = 0;
          singleDone[n] = true;
        }
        break;

      case 5: //single post off
        if(timeNow - ledTiming[n] > SINGLE_POST){ //single is done, reset mode to off
          ledTiming[n] = 0;
          singleDone[n] = true;
          ledModes[n] = MODE_OFF;
          loopCtrl[n] = 0;
          set_led(n, false);
        }
        else if(ledModes[n] == MODE_OFF){
          loopCtrl[n] = 0;
          ledTiming[n] = 0;
          set_led(n, false);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SOLID){
          loopCtrl[n] = 1;
          ledTiming[n] = 0;
          set_led(n, true);
          singleDone[n] = true;
        }
        else if(ledModes[n] == MODE_SLOW || ledModes[n] == MODE_FAST){
          loopCtrl[n] = 2;
          ledTiming[n] = 0;
          set_led(n, false);
          singleDone[n] = true;
        }
        break;



    }
  }
}








void Indicator::slow_blink(uint8_t led) {
  ledModes[led] = MODE_SLOW;
}

void Indicator::fast_blink(uint8_t led) {
  ledModes[led] = MODE_FAST;
}

void Indicator::solid_on(uint8_t led) {
  ledModes[led] = MODE_SOLID;
}

void Indicator::solid_off(uint8_t led) {
  ledModes[led] = MODE_OFF;
}

void Indicator::stop_blink() {
  for(uint8_t n = 0 ; n<NUM_LEDS ; n++){
    ledModes[n] = MODE_OFF;
  }
  led_handler(false);
}

void Indicator::single(uint8_t led, bool blink) {
  singleDone[led] = false;
  singleBlinking[led] = blink;
  ledModes[led] = MODE_SINGLE;
}

bool Indicator::is_single_done(uint8_t led) {
  return singleDone[led];
}

void Indicator::set_led(uint8_t led, bool state) {
  hw_driver->led_ctrl(led, state);
}

