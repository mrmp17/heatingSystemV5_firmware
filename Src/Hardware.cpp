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
  HAL_ADC_Start_DMA(&hadc, ADC_buffer, 4);
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
  HAL_GPIO_WritePin(HTR_DETECT_GPIO_Port, HTR_DETECT_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Hardware::set_default_input_cur() {
  HAL_GPIO_WritePin(CH_ILIM_CTRL_GPIO_Port, CH_ILIM_CTRL_Pin, GPIO_PIN_SET); //FLOATING (open drain pin)
}

void Hardware::set_max_input_cur() {
  HAL_GPIO_WritePin(CH_ILIM_CTRL_GPIO_Port, CH_ILIM_CTRL_Pin, GPIO_PIN_RESET); //LOW (open drain pin)
}


