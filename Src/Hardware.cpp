//
// Created by matej on 11/10/2020.
//

#include "Hardware.h"


Hardware::Hardware() {}




void Hardware::start_ADC() {
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED); //calibrate ADC //todo: how long does this take?
  HAL_ADC_Start_DMA(&hadc, ADC_buffer, 4);
}

void Hardware::stop_ADC() {
  HAL_ADC_Stop_DMA(&hadc);
}

void Hardware::COMP_start() {
  HAL_COMP_Start(&hcomp1);
}

void Hardware::COMP_stop() {
  HAL_COMP_Stop(&hcomp1);
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

uint16_t Hardware::get_fsns_votl() {
  return ((ADC_buffer[ADC_FSNS]*ADC_REF)/ADC_MAX_VAL); //return force sensor voltage in millivolts
}

void Hardware::set_fsns_sply(bool state) {
  HAL_GPIO_WritePin(FORCE_SNS_PEN_GPIO_Port, FORCE_SNS_PEN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Hardware::set_vbat_sply(bool state) {
  HAL_GPIO_WritePin(VBAT_SNS_CTRL_GPIO_Port, VBAT_SNS_CTRL_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Hardware::get_fsns_state() {
  return HAL_COMP_GetOutputLevel(&hcomp1);
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

bool Hardware::chr_pgd() {
  return HAL_GPIO_ReadPin(CH_PG_GPIO_Port, CH_PG_Pin) ? false : true; //invert output: active low output on charger IC
}

bool Hardware::htr_connected() {
  //heater detect pin (pullup on VBUS) should be enabled at all times when heater detection is needed.
  if(HAL_GPIO_ReadPin(HTR_DETECT_GPIO_Port, HTR_DETECT_Pin) == GPIO_PIN_RESET) return false;
  //different calculations: VBUS at VBAT || VBUS pullup 10k to MCU_SPLY
  if(is_power_mosfet_on()){ //VBUS is at VBAT voltage

  }
  else{ //VBUS is pulled up (10k) to 2.5V MCU_SPLY

  }
}


