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


void stateMachine(){
  static uint16_t loopCtrl = 0;
  static uint32_t stateTransitionTime = 0;

  switch(loopCtrl){
    case 0: //main inactive/sleep state
      // state actions:       #####
      hardware.sleep();

      // state flowControl    #####
      if(hardware.wake_source() == WAKE_SOURCE_RTC){
        loopCtrl = 1;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.wake_source() == WAKE_SOURCE_BTN){
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
        loopCtrl = 11; //go charging
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.get_SOC() == SOC_DEAD){
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
        loopCtrl = 10; //display battery SOC and go to sleep if short press
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_button_longpress()){
        loopCtrl = 4; //blink SOC and go to heating
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;


    case 3: //battery dead sleep state
      // state actions:       #####
      hardware.sleep();
      //todo: further reduce power consumption

      // state flowControl    #####
      if(hardware.wake_source() == WAKE_SOURCE_RTC){
        loopCtrl = 1;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      else if(hardware.wake_source() == WAKE_SOURCE_BTN){
        loopCtrl = 2;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;

      //TODO: indicator functions needed
    case 4: //blink battery and go to heating stuff
      // state actions:       #####

      // state flowControl    #####
      loopCtrl = 13;
      stateTransitionTime = HAL_GetTick();
      hardware.clear_button_flags();
      hardware.trace(loopCtrl);

      break;
    case 5: //wait for valid heater connection todo: alow few ms for voltage settling when connecting
      static uint32_t portNotEmptyTime = 0; //warning, this is accessible in all switch cases!
      // state actions:       #####

      // state flowControl    #####
      if(hardware.chrg_pgd()){ //power supply connected
        portNotEmptyTime = 0;
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_htr_connected()){
        portNotEmptyTime = 0;
        loopCtrl = 8; //go to medium heating
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.is_port_empty()){
        portNotEmptyTime = 0;
        if(HAL_GetTick() - stateTransitionTime > WAIT_HEATER_TIMEOUT){
          loopCtrl = 10;
          hardware.set_htr_det(false);
          stateTransitionTime = HAL_GetTick();
          hardware.clear_button_flags();
          hardware.trace(loopCtrl);
        }
        //else nothing to do, just wait
      }
      else if(portNotEmptyTime == 0){
        portNotEmptyTime = HAL_GetTick();
      }
      else if(HAL_GetTick() - portNotEmptyTime > PORT_NOT_EMPTY_WAIT){
        //something that is not heater is connected even after waiting for PORT_NOT_EMPTY_WAIT time, ABORT
        //hardware.debug_print("abort\n");
        portNotEmptyTime = 0;
        hardware.set_htr_det(false);
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      break;

      //TODO: indicator functions needed
    case 6:
      // state actions:       #####

      // state flowControl    #####
      loopCtrl = 3;
      stateTransitionTime = HAL_GetTick();
      hardware.clear_button_flags();
      hardware.trace(loopCtrl);

      break;


    case 7: //low heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_LOW));
      //todo: update indicator;

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
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
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }
      break;


    case 8: //medium heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_MED));
      //todo: update indicator;

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
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
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }

      break;


    case 9: //high heating
      // state actions:       #####
      hardware.set_heating(hardware.rel_htr_pwr(HEAT_HIGH));
      //todo: update indicator;

      // state flowControl    #####
      if(hardware.get_SOC() == SOC_DEAD){
        hardware.set_heating(0);
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
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.trace(loopCtrl);
      }

      break;

      //TODO: indicator functions needed
    case 10: //blink bat and go to sleep
      // state actions:       #####

      // state flowControl    #####
      loopCtrl = 0;
      stateTransitionTime = HAL_GetTick();
      hardware.clear_button_flags();
      hardware.trace(loopCtrl);

      break;
    case 11:
      // state actions:       #####
      //todo: update indicator
      //todo: optimize charging current

      // state flowControl    #####
      if(!hardware.chrg_pgd()){ //charger disconnected
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }
      else if(hardware.chrg_stat() == CHRG_STAT_IDLE){ //charging finished
        loopCtrl = 12;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
        hardware.trace(loopCtrl);
      }


      break;
    case 12:
      // state actions:       #####
      //todo: update indicator

      // state flowControl    #####
      if(!hardware.chrg_pgd()){ //charger disconnected
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
        loopCtrl = 10;
        stateTransitionTime = HAL_GetTick();
        hardware.clear_button_flags();
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

  HAL_Delay(5000);

  hardware.init();

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


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //TODO: check clock config at next cubemx update
    static uint32_t timing = 0;
    static uint32_t cnt = 0;

    if(HAL_GetTick() - timing >= HANDLER_PERIOD && true){ //20ms event
      timing = HAL_GetTick();
      cnt++;
      hardware.button_handler(false);
      hardware.soft_pwm_handler(false);
      hardware.chrg_stat_handler(false);
    }

    stateMachine();




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
