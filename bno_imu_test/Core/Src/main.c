/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Jordan Howard
  * @brief          : Main program body for the BNO08x IMU.
  *
  * This firmware initializes the STM32F1 microcontroller, configures the BNO08x
  * IMU over I2C, and periodically reads orientation/gyro data for use in
  * robotics experiments and sensor fusion research.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Institute of Electrical Engineers Student Branch Chapter
  * at The University of Georgia. 
  * All rights reserved.
  *
  * This work is free and accessible to all interested parties.
  * Redistribution and use are permitted provided that proper credit is given.
  *
  ******************************************************************************
*/

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "hardware_config.h" 
#include "bno08x.h"

// -----------------------------------------------------------------------------
// Macros
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Data Structures 
// -----------------------------------------------------------------------------
typedef struct {
    int16_t Gyro_Z_RAW;
    float dps; // Units: deg/sec = Raw Data / LSB Sensitivity (Reg: 0x1B)
    float Yaw; // Units: degrees
} MPU6050_t;

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
MPU6050_t MPU6050;
char uart_buffer[200];
uint8_t MPU6050_ADDR;

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  while (1)
  {
    return 1;
  }
}

// -----------------------------------------------------------------------------
// Error Handler
// -----------------------------------------------------------------------------
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void error_handler(void)
{
  /* USER CODE BEGIN error_handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

}
