/**
  ******************************************************************************
  * @file           : hardware_config.h
  * @brief          : Header for hardware_config.c file.
  *                   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Institute of Electrical Engineers Student Branch Chapter
  * at The University of Georgia. All rights reserved.
  *
  * This work is free and accessible to all interested parties.
  * Redistribution and use are permitted provided that proper credit is given.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// We include main.h here because it usually contains the "stm32f1xx_hal.h" 
// include, which defines types like I2C_HandleTypeDef.
#include "main.h" // Gives us access to Error_Handler() and HAL types

/* Exported variables --------------------------------------------------------*/
// The 'extern' keyword tells the compiler: "These variables exist, but 
// they are defined (memory allocated) in a different file (hardware_config.c)."
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1; // 1. Share the Handles

/* Exported functions ------------------------------------------------------- */
// These are the prototypes for the functions you moved.
// NOTICE: The 'static' keyword is removed so other files can call them.


// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
void SystemClock_Config(void); // 2. Share the Function names
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARE_CONFIG_H */