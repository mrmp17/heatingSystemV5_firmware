/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Hardware.h"
#include "Indicator.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool pcintFlag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == BUTTON_Pin){
    pcintFlag = true;
  }
}

Hardware hardware(&pcintFlag);
Indicator leds(&hardware);

void blinkBattery(){
  switch(hardware.get_SOC()){
    case SOC_DEAD:
      leds.solid_off(0);
      leds.solid_off(1);
      leds.single(2, false, true);
      break;
    case SOC_0to10:
      leds.solid_off(0);
      leds.solid_off(1);
      leds.single(2, true, false);
      break;
    case SOC_10to40:
      leds.solid_off(0);
      leds.solid_off(1);
      leds.single(2, false, false);
      break;
    case SOC_40to70:
      leds.solid_off(0);
      leds.single(2, false, false);
      leds.single(1, false, false);
      break;
    case SOC_70to100:
      leds.single(2, false, false);
      leds.single(1, false, false);
      leds.single(0, false, false);
      break;
  }
}

void stateMachine(){
  static uint16_t loopCtrl = 0;
  static uint32_t stateTransitionTime = 0;
  static uint32_t RTC_wake_counter = 1;

  switch(loopCtrl){
    case 0: //main inactive/sleep state
      // state actions:       #####
      hardware.sleep();
      leds.stop_blink();
      leds.led_handler(true); //call handler with reset

      // state flowControl    #####
      if(hardware.wake_source() == WAKE_SOURCE_RTC){
        RTC_wake_counter++;
        loopCtrl = 1;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.wake_source() == WAKE_SOURCE_BTN){
        RTC_wake_counter = 0; //reset counter
        loopCtrl = 2;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

    case 1: //wakeup from RTC state
      // state actions:       #####

      // state flowControl    #####
      if(hardware.chrg_pgd()){ //is charger connected?
        hardware.set_charging(true);
        loopCtrl = 11;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(RTC_wake_counter % SOC_RTC_NUM == 0 && HAL_GetTick() - stateTransitionTime < 3*BAT_DIV_TCONST){
        //nothing to do, just wait to let SOC handler measure battery
      }
      else if(hardware.get_SOC() == SOC_DEAD){
        loopCtrl = 3; //to low bat sleep
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else{
        loopCtrl = 0; //back to normal sleep
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

    case 2: //wakeup from button state
      // state actions:       #####

      // state flowControl    #####
      if(hardware.chrg_pgd()){
        hardware.set_charging(true);
        loopCtrl = 11; //go charging
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.get_SOC() == SOC_DEAD){
        blinkBattery();
        loopCtrl = 6;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(HAL_GetTick() - stateTransitionTime > WAIT_LONGPRESS){ //longpress should have happened by now, going back to sleep
        loopCtrl = 0;
        //hardware.debug_print("now %d, strs: %d\n", HAL_GetTick(), stateTransitionTime);
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_shortpress()){
        blinkBattery();
        loopCtrl = 10; //display battery SOC and go to sleep if short press
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_longpress()){
        //blinkBattery(); //not needed
        loopCtrl = 4; //blink SOC and go to heating
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

    case 3: //battery dead sleep state
      // state actions:       #####
      hardware.sleep();
      leds.stop_blink();
      leds.led_handler(true);
      //todo: further reduce power consumption?

      // state flowControl    #####
      if(hardware.wake_source() == WAKE_SOURCE_RTC){
        RTC_wake_counter++;
        loopCtrl = 1;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.wake_source() == WAKE_SOURCE_BTN){
        RTC_wake_counter = 0; //reset RTC counter
        loopCtrl = 2;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

    case 4: //blink battery and go to heating stuff
      // state actions:       #####

      // state flowControl    #####
      if(leds.is_single_done(0) && leds.is_single_done(1) && leds.is_single_done(2)){
        loopCtrl = 13;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

    case 5: //wait for valid heater connection
      // state actions:       #####
      leds.stop_blink();

      // state flowControl    #####
      if(hardware.chrg_pgd()){ //power supply connected
        blinkBattery();
        loopCtrl = 10;
        hardware.set_htr_det(false);
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_htr_connected()){
        loopCtrl = 8; //go to medium heating
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(HAL_GetTick() - stateTransitionTime > WAIT_HEATER_TIMEOUT){
        blinkBattery();
        loopCtrl = 10;
        hardware.set_htr_det(false);
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

    case 6:
      // state actions:       #####
      // state flowControl    #####
      if(leds.is_single_done(0) && leds.is_single_done(1) && leds.is_single_done(2) && !hardware.get_button_dbncd_state()){ //button must not be pressed to allow sleep
        loopCtrl = 3;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

    case 7: //low heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_LOW));

      if(hardware.get_SOC() == SOC_0to10){
        leds.solid_off(0);
        leds.solid_off(1);
        leds.slow_blink(2);
      }
      else{
        leds.solid_on(2);
        leds.solid_off(0);
        leds.solid_off(1);
      }

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
        blinkBattery();
        loopCtrl = 6;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(!hardware.is_htr_connected()){
        hardware.set_heating(0);
        loopCtrl = 5;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_shortpress()){
        loopCtrl = 8;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_longpress()){
        hardware.set_heating(0);
        hardware.set_htr_det(false);
        leds.stop_blink();
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;


    case 8: //medium heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_MED));
      if(hardware.get_SOC() == SOC_0to10){
        leds.slow_blink(2);
        leds.slow_blink(1);
        leds.solid_off(0);
      }
      else{
        leds.solid_on(1);
        leds.solid_on(2);
        leds.solid_off(0);
      }

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
        blinkBattery();
        loopCtrl = 6;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(!hardware.is_htr_connected()){
        hardware.set_heating(0);
        loopCtrl = 5;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_shortpress()){
        loopCtrl = 9;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_longpress()){
        hardware.set_heating(0);
        hardware.set_htr_det(false);
        leds.stop_blink();
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

    case 9: //high heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_HIGH));
      if(hardware.get_SOC() == SOC_0to10){
        leds.slow_blink(0);
        leds.slow_blink(1);
        leds.slow_blink(2);
      }
      else{
        leds.solid_on(0);
        leds.solid_on(1);
        leds.solid_on(2);
      }

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
        hardware.set_htr_det(false);
        leds.stop_blink();
        blinkBattery();
        loopCtrl = 6;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(!hardware.is_htr_connected()){
        hardware.set_heating(0);
        loopCtrl = 5;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_shortpress()){
        loopCtrl = 7;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_longpress()){
        hardware.set_heating(0);
        hardware.set_htr_det(false);
        leds.stop_blink();
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }

      break;

    case 10: //blink bat and go to sleep
      // state actions:       #####

      // state flowControl    #####
      if(leds.is_single_done(0) && leds.is_single_done(1) && leds.is_single_done(2) && !hardware.get_button_dbncd_state()){
        loopCtrl = 0;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;
    case 11:
      // state actions:       #####
      switch(hardware.get_SOC()){
        case SOC_DEAD:
          leds.solid_off(0);
          leds.solid_off(1);
          leds.fast_blink(2);
          break;
        case SOC_0to10:
          leds.solid_off(0);
          leds.solid_off(1);
          leds.fast_blink(2);
          break;
        case SOC_10to40:
          leds.solid_off(0);
          leds.solid_off(1);
          leds.slow_blink(2);
          break;
        case SOC_40to70:
          leds.solid_off(0);
          leds.slow_blink(1);
          leds.slow_blink(2);
          break;
        case SOC_70to100:
          leds.slow_blink(0);
          leds.slow_blink(1);
          leds.slow_blink(2);
          break;
      }

      if(hardware.is_sply_5V3A()){
        hardware.set_max_input_cur();
      }
      else{
        hardware.set_default_input_cur();
      }

      // state flowControl    #####
      if(hardware.is_SOC_request_meas()){ //cant go from this state if request is active
        hardware.set_charging(false);
        hardware.confirm_SOC_request_meas();
        loopCtrl = 14;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(!hardware.chrg_pgd()){ //charger disconnected
        hardware.set_default_input_cur();
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.chrg_stat() == CHRG_STAT_IDLE && HAL_GetTick()){ //charging finished
        hardware.set_default_input_cur();
        loopCtrl = 12;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

      case 12:
      // state actions:       #####
      leds.solid_on(0);
      leds.solid_on(1);
      leds.solid_on(2);

      // state flowControl    #####
      if(!hardware.chrg_pgd()){ //charger disconnected
        hardware.set_charging(false); //disable again, just in case
        hardware.set_default_input_cur();
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.chrg_stat() == CHRG_STAT_CHARGING){ //charging again
        loopCtrl = 11;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }

      break;

    case 13: //check if port empty or htr connected and turn on heater detection
      // state actions:       #####

      // state flowControl    #####
      if(hardware.is_port_empty() || hardware.is_htr_connected()){
        hardware.set_htr_det(true);
        loopCtrl = 5; // go to wait for heater connection
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else{ //something (charger, unknown) is connected, blink SOC and go back to sleep for safety!
        blinkBattery();
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

    case 14:
      if(!hardware.is_SOC_request_meas()){
        hardware.set_charging(true);
        if(hardware.chrg_stat() == CHRG_STAT_CHARGING || HAL_GetTick() - stateTransitionTime > 500){ //todo: change this to get_confirm_SOC_request_meas_time??
          loopCtrl = 11;
          stateTransitionTime = HAL_GetTick();
          hardware.trace(loopCtrl);
        }
      }
      else if(HAL_GetTick() - stateTransitionTime > 500){ //timeout
        loopCtrl = 11;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);

  hardware.init();
  HAL_Delay(100);
  hardware.debug_print("###### FeatHeet by Planinsek industries. ######\n");
  hardware.debug_print("MCU: STM32L051K8U6, Vcc=2.5V\n");
  hardware.debug_print("Booting firmware version ");
  hardware.debug_print(FW_version);
  hardware.debug_print(" ........\n");


  hardware.led_ctrl(0, true);
  HAL_Delay(50);
  hardware.led_ctrl(0, false);
  HAL_Delay(100);
  hardware.led_ctrl(0, true);
  HAL_Delay(50);
  hardware.led_ctrl(0, false);

  HAL_Delay(200);
  hardware.debug_print("OK\n");
  hardware.debug_print("Battery voltage: %d mV\n", hardware.get_vbat());
  hardware.debug_print("Starting normal operation. Your feet shall never be cold again.\n");
  hardware.debug_print("###############\n\n");

  HAL_Delay(100);
//  hardware.led_ctrl(2, true);
//  HAL_Delay(10);
//  hardware.led_ctrl(2, false);
//  HAL_Delay(1000);
//  //hardware.debug_print("sleep :)\n");
//  hardware.sleep();
//  hardware.led_ctrl(3, true);
//  HAL_Delay(2000);
//  hardware.led_ctrl(3, false);
//  hardware.led_ctrl(2, true);
//  HAL_Delay(10);
//  hardware.led_ctrl(2, false);
//  hardware.sleep();
//  hardware.led_ctrl(3, true);

  //hardware.debug_print("wakeup :(\n");

  //leds.solid_on(0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t timing = 0;
    static uint32_t state_machine_timing = 0;
    static uint32_t cnt = 0;

    //start testing

    //end testing

    if(HAL_GetTick() - timing >= HANDLER_PERIOD && true){ //20ms event
      timing = HAL_GetTick();
      cnt++;
      hardware.analog_handler(false);
      leds.led_handler(false);
      hardware.button_handler(false);
      hardware.soft_pwm_handler(false);
      hardware.chrg_stat_handler(false);
      hardware.SOC_handler(false);

      //hardware.debug_print("%d\n", hardware.get_vref());
      //hardware.debug_print("%d\n", hardware.get_vbat());
      //hardware.debug_print("%d\n", hardware.get_real_ADC_ref());
      //hardware.debug_print("%d\n", hardware.vbat_compensated);

      if(hardware.is_button_superlongpress()){ //system resets if very long pres is detected. useful if state machine gets stuck
        hardware.debug_print("resetting...\n");
        hardware.led_ctrl(0, false);
        hardware.led_ctrl(1, false);
        hardware.led_ctrl(2, false);
        hardware.led_ctrl(0, true);
        HAL_Delay(50);
        hardware.led_ctrl(0, false);
        HAL_Delay(100);
        hardware.led_ctrl(0, true);
        HAL_Delay(50);
        hardware.led_ctrl(0, false);
        HAL_NVIC_SystemReset();
      }
    }
    if(cnt%300 == 0 && true){
      //hardware.debug_print("B: %d mV\n", hardware.get_vbat());
      //hardware.debug_print("SOC: %d\n", hardware.get_SOC());
//      hardware.debug_print("V:%d S:%d\n", hardware.vbat_compensated, hardware.get_SOC());
      cnt++;
    }

//    leds.slow_blink(0);
//    leds.slow_blink(1);
//    leds.slow_blink(2);



//    if(hardware.get_SOC() == SOC_0to10){
//      leds.slow_blink(0);
//      leds.slow_blink(1);
//      leds.slow_blink(2);
//    }
//    else{
//      leds.solid_on(0);
//      leds.solid_on(1);
//      leds.solid_on(2);
//    }

    if(HAL_GetTick() - state_machine_timing >= STATE_MACHINE_PERIOD){
      state_machine_timing = HAL_GetTick();
      stateMachine(); //
    }





    //hardware.set_max_input_cur();
    //hardware.set_default_input_cur();

    //hardware.set_charging(true);

    //hardware.led_ctrl(1, hardware.get_button_state());
    //hardware.debug_print("vbat volt: %d\n", hardware.get_vbat());
//    hardware.debug_print("cc1 volt: %d\n", hardware.get_CC1_volt());
//    hardware.debug_print("cc2 volt: %d\n", hardware.get_CC2_volt());
    //hardware.debug_print("VBUS ok: %d\n", hardware.chrg_pgd());
//    hardware.debug_print("5V3A det: %d\n", hardware.is_sply_5V3A());
//    hardware.debug_print("SOC: %d\n", hardware.get_SOC());
//    hardware.debug_print("HTR present: %d\n", hardware.is_htr_connected());
//    hardware.debug_print("rel htr pwr: %d\n", hardware.rel_htr_pwr(HEAT_MAX));
    //hardware.debug_print("\n");
    //HAL_Delay(100);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
