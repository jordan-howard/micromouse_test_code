/**
  ******************************************************************************
  * @file           : bno08x.h
  * @brief          : Header for bno08x.c file.
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
#ifndef BNO08X_H
#define BNO08X_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


// -----------------------------------------------------------------------------
// Macros
// -----------------------------------------------------------------------------
#define BNO08X_I2C_ADDR_7BIT   0x4A   // or 0x4B depending on DI/SA0 pin
#define BNO08X_I2C_ADDR        (BNO08X_I2C_ADDR_7BIT << 1) // HAL expects 8-bit

// SHTP channels (datasheet)
#define BNO08X_CH_COMMAND      0     // SHTP command channel
#define BNO08X_CH_EXECUTABLE   1
#define BNO08X_CH_CONTROL      2     // sensor hub control (Set Feature, etc.)
#define BNO08X_CH_REPORTS      3     // input sensor reports (non-wake)
#define BNO08X_CH_GYRO_RV      5     // gyro-rotation-vector channel (not used here)

// SH-2 report IDs
#define BNO08X_REPORTID_GYRO_CALIBRATED   0x02
#define BNO08X_REPORTID_SET_FEATURE       0xFD

#define BNO08X_MAX_PACKET_LEN   64


// -----------------------------------------------------------------------------
// Data Structures 
// -----------------------------------------------------------------------------
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t seq[6];                 // sequence number per channel
    uint8_t rxBuf[BNO08X_MAX_PACKET_LEN];
} BNO08x_Handle_t;


// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
bool BNO08x_Init(BNO08x_Handle_t *dev, I2C_HandleTypeDef *hi2c);
bool BNO08x_EnableGyro(BNO08x_Handle_t *dev, uint32_t reportInterval_us);
bool BNO08x_ReadGyro(BNO08x_Handle_t *dev,  float *gz);

#ifdef __cplusplus
}
#endif

#endif /* BNO08X_H */
