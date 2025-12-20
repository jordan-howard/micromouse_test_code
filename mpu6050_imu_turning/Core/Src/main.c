/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "hardware_config.h" 

// -----------------------------------------------------------------------------
// Macros
// -----------------------------------------------------------------------------
// MPU6050 Registers
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define INT_ENABLE_REG 0x38
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define FIFO_EN_REG 0x23
#define USER_CTRL_REG 0x6A
#define FIFO_COUNTH_REG 0x72
#define FIFO_R_W_REG 0x74

// Configuration values
#define GYRO_SCALE_250 0x00

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
// Function Declarations
// -----------------------------------------------------------------------------
uint8_t MPU6050_Init(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_All(void);
void MPU6050_Calculate_Angles(void);
uint8_t MPU6050_Read_Register(uint8_t reg);
void MPU6050_Write_Register(uint8_t reg, uint8_t value);

// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------
/**
  * @brief  Executes in case of error occurrence
  * @return None
  */
void error_handler(void)
{
  __disable_irq(); // Disables all interrupts
}
// -----------------------------------------------------------------------------
uint8_t MPU6050_init(void) {
    uint8_t check;
    uint8_t data;
    
    // Try both possible I2C addresses
    uint8_t addresses[] = {0xD0, 0xD2};
    uint8_t found = 0;
    
    for (int i = 0; i < 2; i++) {
        MPU6050_ADDR = addresses[i];
        
        // Check if device responds
        if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2, 100) == HAL_OK) {
            // Read WHO_AM_I register
            HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 
              WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
            
            if (check == 0x68) {  // Valid WHO_AM_I value
                found = 1;
                break;
            }
        }
    }
    
    if (!found) {
        return 1;  // Initialization failed
    }
    
    // Wake up MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 
      PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(100);  // Wait for sensor to stabilize
    
    // Set sample rate to 1kHz
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 
      SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);

    // Configure gyroscope (±250 deg/s)
    data = GYRO_SCALE_250;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 
      GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
    
    // Set digital low-pass filter
    data = 0x03;  // ~44Hz bandwidth
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 
      CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
  
    return 0;
}
// -----------------------------------------------------------------------------
/**
  * @brief  Read gyroscope data Reg: 0x43 and
  * @retval None
  */
void MPU6050_Read_Gyro(void) {
    uint8_t data[6];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 
      GYRO_XOUT_H_REG, 1, data, 6, HAL_MAX_DELAY);
    

    MPU6050.Gyro_Z_RAW = (int16_t)(data[4] << 8 | data[5]);
    
    // Convert to deg/s (±250 deg/s range -> 131 LSB/deg/s)
    MPU6050.dps = MPU6050.Gyro_Z_RAW / 131.0f;
}
// -----------------------------------------------------------------------------
/**
  * @brief  Calibrates the Z-Axis of Gyro
  * @return None
  */
void MPU6050_CalibrateZ(MPU6050_t *imu, int samples)
{
    int32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        // read raw Z here from sensor into imu->Gyro_Z_RAW
        // e.g. MPU6050_ReadGyroZ(&imu->Gyro_Z_RAW);

        sum += imu->Gyro_Z_RAW;
        HAL_Delay(2);  // or your sample period
    }

    //float avg_raw = (float)sum / (float)samples;
    //gyro_z_offset = avg_raw / gyro_sens;  // in deg/s
}






// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}



