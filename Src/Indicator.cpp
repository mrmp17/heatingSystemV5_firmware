//
// Created by matej on 08/11/2020.
//

#include "Indicator.h"

Indicator::Indicator(Hardware *obj) {
  hw_driver = obj;
}

void Indicator::led_handler(bool reset) {

  uint32_t timeNow = HAL_GetTick();

  if(reset){
    for(uint8_t n=0 ; n<NUM_LEDS ; n++){
      ledModes[n] = 0;
      slowRampCnt = 0;
      fastRampCnt = 0;
      lastHandlerTime = timeNow;
    }
  }

  uint32_t dt = timeNow - lastHandlerTime;

  //ramp generator
  if(dt >= MAX_HANDLER_dT){
    dt = MAX_HANDLER_dT;
    //something's f'd if this happens
  }
  //slow ramp
  if(slowRampCnt < SLOW_ON + SLOW_OFF){
    if(dt > SLOW_ON + SLOW_OFF - slowRampCnt){
      slowRampCnt = dt - (SLOW_ON + SLOW_OFF - slowRampCnt);
    }
    else{
      slowRampCnt += dt;
    }
  }
  else{
    slowRampCnt = dt;
  }
  //fast ramp
  if(fastRampCnt < FAST_ON + FAST_OFF){
    if(dt > FAST_ON + FAST_OFF - fastRampCnt){
      fastRampCnt = dt - (FAST_ON + FAST_OFF - fastRampCnt);
    }
    else{
      fastRampCnt += dt;
    }
  }
  else{
    fastRampCnt = dt;
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

      case 1: //full on
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
          set_led(n, slowRampCnt<=SLOW_ON);
        }
        else if(ledModes[n] == MODE_FAST){
          set_led(n, fastRampCnt<=FAST_ON);
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
          set_led(n, fastRampCnt<=FAST_ON);
        }

        if((timeNow - ledTiming[n] > SINGLE_ON_SHORT) && singleShort[n]){ //single time for short enabled
          ledTiming[n] = timeNow;
          loopCtrl[n] = 5;
          set_led(n, false);
        }

        else if(timeNow - ledTiming[n] > SINGLE_ON){ //single time for short disabled
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
          //TODO implement switching to prev mode instead of off if desired
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
  if (led > NUM_LEDS-1)
    return;
  ledModes[led] = MODE_SLOW;
}

void Indicator::fast_blink(uint8_t led) {
  if (led > NUM_LEDS-1)
    return;
  ledModes[led] = MODE_FAST;
}

void Indicator::solid_on(uint8_t led) {
  if (led > NUM_LEDS-1)
    return;
  ledModes[led] = MODE_SOLID;
}

void Indicator::solid_off(uint8_t led) {
  if (led > NUM_LEDS-1)
    return;
  ledModes[led] = MODE_OFF;
}

void Indicator::stop_blink() {
  for(uint8_t n = 0 ; n<NUM_LEDS ; n++){
    ledModes[n] = MODE_OFF;
  }
  led_handler(false);
}

void Indicator::single(uint8_t led, bool blink, bool shrt) {
  if (led > NUM_LEDS-1)
    return;
  singleDone[led] = false;
  singleBlinking[led] = blink;
  singleShort[led] = shrt;
  ledModes[led] = MODE_SINGLE;
}

bool Indicator::is_single_done(uint8_t led) {
  if (led > NUM_LEDS-1)
    return false;
  return singleDone[led];
}

void Indicator::set_led(uint8_t led, bool state) {
  if (led > NUM_LEDS-1)
    return;
  if(state != prevLedState[led]){
    hw_driver->led_ctrl(led, state);
    hw_driver->led_flip_time = HAL_GetTick();
  }
  prevLedState[led] = state;

}

