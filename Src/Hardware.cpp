//
// Created by matej on 11/10/2020.
//

#include <usart.h>
#include "Hardware.h"


Hardware::Hardware(bool *btn_interrupt_flag_pointer) {
  btn_int_flag_pointer = btn_interrupt_flag_pointer;
}


void Hardware::debug_print(const char *format, ...){
  char formatedString[128] = {0};
  va_list arglist;
  va_start(arglist, format);
  int len = vsnprintf(formatedString, sizeof(formatedString), format, arglist);
  uint16_t n = 0;
  //comment out this block if carriage return not needed (change serial_01.write len!!!)
  while(formatedString[n] != 0) n++;
  formatedString[n] = 0xD;
  //####
  va_end(arglist);
  HAL_UART_Transmit(&huart1, (uint8_t *)(formatedString), len+1, 100);
}


void Hardware::init() {

  set_vbat_sply(true); //enable resistor divider supply. (enabled all the time, except when sleeping)
  set_charging(false); //disable charging
  set_button_sply(true); //enable button supply (can remain on all the time - no vampire drain)
  set_pwr_mosfet(false); //disable pwr mosfet just in case
  start_ADC(); //start ADC sampling
}

void Hardware::set_pwr_mosfet(bool state) {
  if(state != pwr_mos_state){
    HAL_GPIO_WritePin(HTR_SW_CTRL_GPIO_Port, HTR_SW_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pwr_mos_state = state;
    last_pwrmos_flip = HAL_GetTick();
  }

}

bool Hardware::is_pwr_mosfet_on() {
  return HAL_GPIO_ReadPin(HTR_SW_CTRL_GPIO_Port, HTR_SW_CTRL_Pin);
}


void Hardware::start_ADC() {
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED); //calibrate ADC //todo: how long does this take?
  HAL_ADC_Start_DMA(&hadc, ADC_buffer, ADC_NUM_CH);
}

void Hardware::stop_ADC() {
  HAL_ADC_Stop_DMA(&hadc);
}

uint16_t Hardware::get_real_ADC_ref() {
  return (uint16_t)(((float)VREF_INT_RAW/valid_raw_ADC[ADC_VREF])*ADC_REF);
}

uint16_t Hardware::get_vref() {
  return ((valid_raw_ADC[ADC_VREF]*ADC_REF)/ADC_MAX_VAL); //return vref voltage in millivolts
}

uint16_t Hardware::get_vbat() {
  return ((valid_raw_ADC[ADC_VBAT]*get_real_ADC_ref())/ADC_MAX_VAL)* ADC_VBAT_KOEF; //return battery voltage in millivolts
  //return ((ADC_buffer[ADC_VBAT]*ADC_REF)/ADC_MAX_VAL)* ADC_VBAT_KOEF; //return battery voltage in millivolts
}

uint16_t Hardware::get_UDP_volt() {
  return ((valid_raw_ADC[ADC_UDP]*ADC_REF)/ADC_MAX_VAL); //return usb D+ voltage in millivolts
}

uint16_t Hardware::get_CC1_volt() {
  return ((valid_raw_ADC[ADC_CC1]*ADC_REF)/ADC_MAX_VAL); //return CC1 voltage in millivolts
}

uint16_t Hardware::get_CC2_volt() {
  return ((valid_raw_ADC[ADC_CC2]*ADC_REF)/ADC_MAX_VAL); //return CC2 voltage in millivolts
}


void Hardware::set_button_sply(bool state) {
  HAL_GPIO_WritePin(FORCE_SNS_PEN_GPIO_Port, FORCE_SNS_PEN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Hardware::set_vbat_sply(bool state) {
  HAL_GPIO_WritePin(VBAT_SNS_CTRL_GPIO_Port, VBAT_SNS_CTRL_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}


void Hardware::led_ctrl(uint8_t led, bool state) {
  if(led == 0){
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if(led == 1){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if(led == 2){
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

bool Hardware::chrg_pgd() {
  return HAL_GPIO_ReadPin(CH_PG_GPIO_Port, CH_PG_Pin) ? false : true; //invert output: active low output on charger IC
}

bool Hardware::get_button_state() {
  return HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
}

bool Hardware::get_button_dbncd_state() {
  return buttonDebouncedState;
}

void Hardware::set_charging(bool state) {
  HAL_GPIO_WritePin(CH_CE_GPIO_Port, CH_CE_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Hardware::set_htr_det(bool state) {
  //this should switch  TEST_GPIO pin between HIGH an HIGH_Z (bodge connecting TEST_GPIO and CC2
  //is_htr_connected() checks if this causes voltage on CC1 pin and measures resistance CC2-CC1
  if(state){
    HAL_GPIO_WritePin(HTR_DET_GPIO_Port, HTR_DET_Pin, GPIO_PIN_SET); //set to high
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = HTR_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HTR_DET_GPIO_Port, &GPIO_InitStruct); //enable output mode
  }
  else{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = HTR_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HTR_DET_GPIO_Port, &GPIO_InitStruct); //enable analog mode (high Z)
  }
  htr_det_state = state; //save current state

  //TODO
}

void Hardware::set_default_input_cur() {
  HAL_GPIO_WritePin(CH_ILIM_CTRL_GPIO_Port, CH_ILIM_CTRL_Pin, GPIO_PIN_SET); //FLOATING (open drain pin)
}

void Hardware::set_max_input_cur() {
  HAL_GPIO_WritePin(CH_ILIM_CTRL_GPIO_Port, CH_ILIM_CTRL_Pin, GPIO_PIN_RESET); //LOW (open drain pin)
}

bool Hardware::is_sply_5V3A() {
  //source should be sourcing 330uA +- 8% on one CC pin. other CC pin should be near ground
  //330uA +- 8%  on 5k1 Rd resistor is 1.515V to  1.818V
  uint16_t CC1 = get_CC1_volt();
  uint16_t CC2 = get_CC2_volt();
  //return if CC pins are at correct voltage levels for 5V 3A supply
  return (CC1 < V5V3A_LCC_MAX && (CC2 > V5V3A_HCC_MIN && CC2 < V5V3A_HCC_MAX)) || (CC2 < V5V3A_LCC_MAX && (CC1 > V5V3A_HCC_MIN && CC1 < V5V3A_HCC_MAX));
}

bool Hardware::is_port_empty() {
  if(is_htr_det_on()){
    if(get_CC1_volt() < PORT_EMPTY_CCMAX && get_CC2_volt() < PORT_EMPTY_CCMAX && !is_htr_connected()){
      return true;
    }
    else return false;
  }

  else{
    if(get_CC1_volt() < PORT_EMPTY_CCMAX && get_CC2_volt() < PORT_EMPTY_CCMAX){
      return true;
    }
    else return false;
  }

}

bool Hardware::is_charging() {
  return chrg_stat_ == CHRG_STAT_CHARGING;
}

uint8_t Hardware::get_SOC() {
  return SOC_val;
}

uint8_t Hardware::calculate_SOC() {
  static uint8_t loopCtrl = 1;
  static uint8_t soc = 0;
  uint16_t voltage = get_vbat();

  vbat_compensated = voltage; //no load voltage


  //return SOC_0to10;


  switch (loopCtrl){

    case 0: //SOC_DEAD
      soc = SOC_DEAD;
      if(voltage > soc_thr[4]){
        loopCtrl = 1;
        soc = SOC_0to10;
      }
      break;

    case 1: //SOC_0to10
    soc = SOC_0to10;
    if(voltage < soc_thr[3]){
      loopCtrl = 0;
      soc = SOC_DEAD;
    }
    else if(voltage > soc_thr[0]+SOC_HYST){
      loopCtrl = 2;
      soc = SOC_10to40;
    }
      break;

    case 2: //SOC_10to40
      soc = SOC_10to40;
      if(voltage < soc_thr[0]){
        loopCtrl = 1;
        soc = SOC_0to10;
      }
      else if(voltage > soc_thr[1]+SOC_HYST){
        loopCtrl = 3;
        soc = SOC_40to70;
      }
      break;

    case 3: //SOC_40to70
      soc = SOC_40to70;
      if(voltage < soc_thr[1]){
        loopCtrl = 2;
        soc = SOC_10to40;
      }
      else if(voltage > soc_thr[2]+SOC_HYST){
        loopCtrl = 4;
        soc = SOC_70to100;
      }
      break;
    case 4: //SOC_70to100
      soc = SOC_70to100;
      if(voltage < soc_thr[2]){
        loopCtrl = 3;
        soc = SOC_40to70;
      }
      break;
  }

  return soc;
}

bool Hardware::is_htr_connected() {
  if(!htr_det_state){
    return false; //return false if detection circuit is not ON
    //can not detect heater without detection circuit
  }
  //USB data+ is pulled down with 10k in heater connector
  //when HTR_DET is high it pulls D+ high 10k
  //D+ voltage sould be VCC/2 = 1250mV +-8%: 1188mV to 1313mV
  //both cc pins should be floaty
  uint16_t dpVolt = get_UDP_volt();
  return (get_CC1_volt() < PORT_EMPTY_CCMAX && get_CC2_volt() < PORT_EMPTY_CCMAX && (dpVolt > UDP_HTR_MIN && dpVolt < UDP_HTR_MAX));
}

uint16_t Hardware::rel_htr_pwr(uint16_t power_mw) {
  float vbat = get_vbat();
  float maxPower = (vbat*vbat)/HTR_RESISTANCE; //U^2 / R
  float k = power_mw/maxPower;
  if(k >= 1){ //set to 100% if requested power is higher than max power.
    //debug_print("cant provide requested power.\n");
    return 100;
  }
  else return (uint16_t)(k*100);
}

void Hardware::set_heating(uint16_t pwr_percent) {
  current_htr_pwr = pwr_percent;
}

void Hardware::soft_pwm_handler(bool reset) {
  static uint8_t cnt = 0;
  static bool waitSOC = false;

  if(reset){ //reset cunter ramp
    cnt = 0;
  }

  if(current_htr_pwr == 0){
    set_pwr_mosfet(false);
    cnt = 0; //reset counter so it starts at 0 at next turn-on
  }

  else if(is_SOC_request_meas()){
    if(!waitSOC){
      set_pwr_mosfet(false);
      waitSOC = true;
    }
  }

  else{
    waitSOC = false;

    if(cnt < current_htr_pwr){
      set_pwr_mosfet(true);
      //led_ctrl(3, true);
    }
    else{
      set_pwr_mosfet(false);
      //led_ctrl(3, false);
    }

    //ramp generator
    if(cnt < 100){
      cnt++;
    }
    else{
      cnt = 0;
    }
  }
}

void Hardware::sleep() {
  if(!ENABLE_SLEEP){
    HAL_Delay(RTC_WAKE_TIME);
    return;
  }
  //disable all power hungry peripherals and enter sleep
  //TODO: check if other stuff needs to be turned off
  //just in case, set heating to OFF and call handler to make sure heating gets disabled
  set_heating(0);
  soft_pwm_handler(false);

  stop_ADC(); //stop ADC/DMA
  set_vbat_sply(false); //disable vbat divider supply

  set_default_input_cur();
  set_charging(true); //enable charging to prevent CE pulldown drain

  set_htr_det(false);

  led_ctrl(0, false);
  led_ctrl(1, false);
  led_ctrl(2, false);

  //enable RTC timer
  uint32_t sleepTime = ((uint32_t)RTC_WAKE_TIME*RTC_TICKS_PER_S)/1000; //2313 cycles per second at RCCCLK/16
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepTime, RTC_WAKEUPCLOCK_RTCCLK_DIV16); //set rtc interrupt for RTC_WAKE_TIME ms
  *btn_int_flag_pointer = false; //this flag could be set if button was pressed while not sleeping
  HAL_SuspendTick(); //suspend tick to prevent tick interrupts
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWR_DisableSleepOnExit(); //disable sleeping after exiting interrupt that caused wake-up
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  //sleeping
  //wake up


  config_clk_wake();
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_ResumeTick();
  set_vbat_sply(true); //enable vbat divider supply
  confirm_SOC_request_meas();
  last_divider_enable = HAL_GetTick();
  start_ADC();
  if(*btn_int_flag_pointer){
    wakeup_src = WAKE_SOURCE_BTN;
  }
  else{
    wakeup_src = WAKE_SOURCE_RTC;
  }
  //HAL_Delay(10);
  //*btn_int_flag_pointer = false; //do this before sleep, not after
  //set_charging(false);  //TODO: is this ok?
  button_handler(true); //call handler with reset param
  chrg_stat_handler(true);

}

void Hardware::config_clk_wake() {
  //configure clocks
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  //end clock configuration
}

void Hardware::config_gpio_slp() {
  //set pins to analog mode to reduce power consumption (digital input stage disabled in this mode)
  //todo: what to do with usart and swd pins?

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //port C
  GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = CH_ILIM_CTRL_Pin | CH_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //port A
  GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = USB_CC1_ANALOG_Pin | USB_CC2_ANALOG_Pin | VBAT_SNS_Pin | VBAT_SNS_CTRL_Pin | HTR_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //port B
  GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = CH_STAT_Pin | CH_PG_Pin | HTR_SW_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Hardware::config_gpio_wake() {
  //set gpios back to normal state
  MX_GPIO_Init();
}

void Hardware::chrg_stat_handler(bool reset) {
  //todo: test
  //detects if charger reports:
  // - charging complete / sleep: pin HIGH
  // - charging in progress: pin LOW
  // - charging suspended: pin BLINKING 1Hz
  static uint32_t lastEdgeTime = 0;
  static bool lastState = false;
  static const uint32_t errorInterval = 1000;
  static uint32_t blinkCounter = 0; //counts valid on-after-another blinks

  if(reset){ //resets timing and blink counter
    lastEdgeTime = 0;
    lastState = false;
    blinkCounter = 0;
  }

  bool state = HAL_GPIO_ReadPin(CH_STAT_GPIO_Port, CH_STAT_Pin) ? true : false;
  if(state != lastState){ // edge event
    uint32_t time = HAL_GetTick();
    uint32_t timeFromLast = time - lastEdgeTime;
    if(timeFromLast > errorInterval-50 && timeFromLast < errorInterval+50){ //1 second from last edge
      blinkCounter++;
    }
    else{
      blinkCounter = 0; //last edge was not 1 second ago, reset blinkCounter
    }
    if(blinkCounter > 2){
      chrg_stat_ = CHRG_STAT_ERROR;
    }
    else if(state == true){
      chrg_stat_ = CHRG_STAT_IDLE;
    }
    else if(state == false){
      chrg_stat_ = CHRG_STAT_CHARGING;
    }
    lastEdgeTime = time; //save time
    lastState = state; //save state
  }
  else if(HAL_GetTick()-lastEdgeTime > 1500){
    if(state == true){
      chrg_stat_ = CHRG_STAT_IDLE;
    }
    else if(state == false){
      chrg_stat_ = CHRG_STAT_CHARGING;
    }
    lastEdgeTime = HAL_GetTick();
    lastState = state;
  }
}

uint8_t Hardware::chrg_stat() {
  return chrg_stat_;
}

void Hardware::button_handler(bool reset) {
  static uint8_t loopCtrl = 0;
  static uint32_t counter = 0;

  if(reset){
    counter = 0;
    //TODO
    //if button is being held down before sleep, we don't want long press flag after longpress duration after sleep every time MCU wakes up
    loopCtrl = 0;
    //reset flags if called with reset parameter
    shortPressFlag = false;
    longPressFlag = false;
    superLongPressFlag = false;
  }

  switch(loopCtrl){
    case 0: //waiting for button press
      if(get_button_state()){ //is button pressed?
        counter = 0;
        loopCtrl = 1;
      }
      break;
    case 1: //waiting for debounce period
      if(counter >= BUTTON_DBOUNCE_CYCLES){
        if(get_button_state()){ //button still pressed after debounce time
          buttonDebouncedState = true;
          loopCtrl = 2; //go wait for short press time
        }
        else{ //button is not pressed anymore
          buttonDebouncedState = false;
          loopCtrl = 0; //go back to state 0
        }
      }
      break;
    case 2: //button is pressed, waiting for short and long press
      if(!get_button_state()){ //button released
        if(counter >= BUTTON_SHORPTESS_CYCLES){ //button was pressed for correct time to trigger short press event
          shortPressFlag = true;
        }
        buttonDebouncedState = false;
        loopCtrl = 0; //go back to state 0
      }
      else if(counter >= BUTTON_LONGPRESS_CYCLES){
        longPressFlag = true;
        loopCtrl = 3;
      }
      break;

    case 3: //waiting for button release after long press duration
      if(!get_button_state()){ //go to state 0 when button released
        buttonDebouncedState = false;
        loopCtrl = 0;
      }
      else if(counter >= BUTTON_SUPERLONG_CYCLES){
        buttonDebouncedState = false;
        superLongPressFlag = true;
        counter = 0;
      }
      break;
  }
  counter++;
}

bool Hardware::is_button_shortpress() {
  if(shortPressFlag){
    shortPressFlag = false;
    return true;
  }
  else return false;
}

bool Hardware::is_button_longpress() {
  if(longPressFlag){
    longPressFlag = false;
    return true;
  }
  else return false;
}

bool Hardware::is_button_superlongpress() {
  if(superLongPressFlag){
    superLongPressFlag = false;
    return true;
  }
  else return false;
}

uint8_t Hardware::wake_source() {
  return wakeup_src;
}

void Hardware::trace(uint16_t state_num) {
  if(STATE_TRACE){
    debug_print("TRC: %d\n", state_num);
  }
}

bool Hardware::is_htr_det_on() {
  return htr_det_state;
}

void Hardware::clear_button_flags() {
  //clear both flags
  is_button_shortpress();
  is_button_longpress();
  is_button_superlongpress();
}

bool Hardware::is_vbat_sply_on() {
  return !HAL_GPIO_ReadPin(VBAT_SNS_CTRL_GPIO_Port, VBAT_SNS_CTRL_Pin);
}

bool Hardware::is_SOC_request_meas() {
  return request_SOC_meas;
}

void Hardware::confirm_SOC_request_meas() {
  request_SOC_meas_confirm_time = HAL_GetTick();
}

uint32_t Hardware::get_confirm_SOC_request_meas_time() {
  return request_SOC_meas_confirm_time;
}


void Hardware::SOC_handler(bool reset) {
  //check if battery is unloaded and divider is ON and settled and run SOC state machine. request unloading if SOC data is old.
  //things to check: pwrmos OFF ; not charging ; divider ON ; enough time from divider ON ; enough time from pwrmos OFF ; BAT_DIV_TCONST from last request
  static uint32_t lastSOCcalc = 0;

  uint32_t timeNow = HAL_GetTick();

  if(timeNow - lastSOCcalc > SOC_MAX_INTERVAL){
    request_SOC_meas = true;
  }
  else{
    request_SOC_meas = false;
  }

  if( !is_pwr_mosfet_on()
      && !is_charging()
      && is_vbat_sply_on()
      && timeNow - last_divider_enable > BAT_DIV_TCONST
      && timeNow - last_pwrmos_flip > BAT_DIV_TCONST
      && timeNow - request_SOC_meas_confirm_time > BAT_DIV_TCONST){

    //run soc state machine
    SOC_val = calculate_SOC();
    lastSOCcalc = timeNow;
  }
}

void Hardware::analog_handler(bool reset) {
  if(HAL_GetTick() - led_flip_time > BAT_DIV_TCONST){
    for(uint8_t n = 0 ; n<ADC_NUM_CH ; n++){
      valid_raw_ADC[n] = ADC_buffer[n];
    }
  }
}


