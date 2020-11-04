//
// Created by matej on 11/10/2020.
//

#include <usart.h>
#include "Hardware.h"


Hardware::Hardware() {}


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
  HAL_GPIO_WritePin(HTR_SW_CTRL_GPIO_Port, HTR_SW_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Hardware::is_pwr_mosfet_on() {
  return HAL_GPIO_ReadPin(HTR_SW_CTRL_GPIO_Port, HTR_SW_CTRL_Pin);
}


void Hardware::start_ADC() {
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED); //calibrate ADC //todo: how long does this take?
  HAL_ADC_Start_DMA(&hadc, ADC_buffer, 3);
}

void Hardware::stop_ADC() {
  HAL_ADC_Stop_DMA(&hadc);
}



uint16_t Hardware::get_vbat() {
  return ((ADC_buffer[ADC_VBAT]*ADC_REF)/ADC_MAX_VAL)* ADC_VBAT_KOEF; //return battery voltage in millivolts
}

uint16_t Hardware::get_CC1_volt() {
  return ((ADC_buffer[ADC_CC1]*ADC_REF)/ADC_MAX_VAL); //return CC1 voltage in millivolts
}

uint16_t Hardware::get_CC2_volt() {
  return ((ADC_buffer[ADC_CC2]*ADC_REF)/ADC_MAX_VAL); //return CC2 voltage in millivolts
}


void Hardware::set_button_sply(bool state) {
  HAL_GPIO_WritePin(FORCE_SNS_PEN_GPIO_Port, FORCE_SNS_PEN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Hardware::set_vbat_sply(bool state) {
  HAL_GPIO_WritePin(VBAT_SNS_CTRL_GPIO_Port, VBAT_SNS_CTRL_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}


void Hardware::led_ctrl(uint8_t led, bool state) {
  if(led == 1){
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if(led == 2){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  else if(led == 3){
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

bool Hardware::chrg_pgd() {
  return HAL_GPIO_ReadPin(CH_PG_GPIO_Port, CH_PG_Pin) ? false : true; //invert output: active low output on charger IC
}

bool Hardware::get_button_state() {
  return HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
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

bool Hardware::is_charging() {
  return false;
  //todo: implement
}

uint8_t Hardware::get_SOC() {
  static uint8_t soc = SOC_40to70; //persistent SOC variable
  uint16_t voltage = get_vbat();

  if(voltage <= soc_thr[3]){
    soc = SOC_DEAD; //if battery falls below 2.8V, report battery dead
  }

  else{
    if(soc == SOC_DEAD && voltage < soc_thr[4]) soc = SOC_DEAD; //return SOC_DEAD until battery reaches SOC_DEAD release voltage

    else if(is_charging()){ //battery is charging
      //shit. current (internal resistance voltage drop) not known :( don't report state while charging, this is very imprecise
      if(voltage <  soc_thr[0]) soc = SOC_0to10;
      else if(voltage <  soc_thr[1]) soc = SOC_10to40;
      else if(voltage <  soc_thr[2]) soc = SOC_40to70;
      else soc = SOC_70to100;
    }
    else{ //battery is not charging
      if(is_pwr_mosfet_on()){ //battery is loaded by the heater, compensate for internal resistance drop
        uint16_t irdrop = (((float)voltage / HTR_RESISTANCE)*BAT_RINT); //drop on battery internal resistance in mV
        voltage = voltage + irdrop; // compensate for internal resistance drop
        if(voltage <  soc_thr[0]) soc = SOC_0to10;
        else if(voltage <  soc_thr[1]) soc = SOC_10to40;
        else if(voltage <  soc_thr[2]) soc = SOC_40to70;
        else soc = SOC_70to100;

      }
      else{ //battery is unloaded
        if(voltage <  soc_thr[0]) soc = SOC_0to10;
        else if(voltage <  soc_thr[1]) soc = SOC_10to40;
        else if(voltage <  soc_thr[2]) soc = SOC_40to70;
        else soc = SOC_70to100;
      }
    }
  }

  return soc;
}

bool Hardware::is_htr_connected() {
  if(!htr_det_state){
    return false; //return false if detection circuit is not ON
    //can not detect heater without tetection circuit
  }
  //CC2 pin is hard pulled up. should be around 2500mV
  //CC1 is mid of divider |2V5 -- 10K -- CC1 -- 5K1 -- GND| if heater is connected
  //voltage should be around 844mV if connected, around 0 if not. (added +-8% tolerance)
  uint16_t cc1 = get_CC1_volt();
  uint16_t cc2 = get_CC2_volt();
  return (cc2 > 2400 && (cc1 > 780 && cc1 < 910)); //todo: test if range is to small
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

  if(reset){ //reset cunter ramp
    cnt = 0;
  }

  if(current_htr_pwr == 0){
    set_pwr_mosfet(false);
    cnt = 0; //reset counter so it starts at 0 at next turn-on
  }
  else{
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
  //disable all power hungry peripherals and enter sleep
  //TODO: check if other stuff needs to be turned off
  //just in case, set heating to OFF and call handler to make sure heating gets disabled
  set_heating(0);
  soft_pwm_handler(false);

  stop_ADC(); //stop ADC/DMA
  set_vbat_sply(false); //disable vbat divider supply
  set_charging(true); //enable charging to prevent CE pulldown drain
  //enable RTC timer
  uint32_t sleepTime = ((uint32_t)RTC_WAKE_TIME*RTC_TICKS_PER_S)/1000; //2313 cycles per second at RCCCLK/16
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepTime, RTC_WAKEUPCLOCK_RTCCLK_DIV16); //set rtc interrupt for RTC_WAKE_TIME ms
  HAL_SuspendTick(); //suspend tick to prevent tick interrupts
  HAL_PWREx_EnableUltraLowPower();
  HAL_PWR_DisableSleepOnExit(); //disable sleeping after exiting interrupt that caused wake-up
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  //sleeping
  //wake up


  config_clk_wake();
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  set_vbat_sply(true); //enable vbat divider supply
  start_ADC();
  set_charging(false);  //TODO: is this ok?
  button_handler(true); //call handler with reset param

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
        loopCtrl = 0;
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


