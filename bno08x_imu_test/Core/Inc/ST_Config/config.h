/**
  ******************************************************************************
  * @file           : config.h
  * @author         : Jordan Howard
  * @brief          : Header for config.c file.
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
#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include "main.h" 


// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1; 


// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
void SystemClock_Config(void); 
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */