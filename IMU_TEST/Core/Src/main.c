/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - MPU-6050 Full Implementation
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;
    
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;
    
    int16_t Temp_RAW;
    float Temperature;
    
    float Roll;
    float Pitch;
} MPU6050_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// MPU6050 I2C Address
#define MPU6050_ADDR 0xD0  // AD0 pin low (0x68 << 1)

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
#define ACCEL_SCALE_2G 0x00
#define ACCEL_SCALE_4G 0x08
#define ACCEL_SCALE_8G 0x10
#define ACCEL_SCALE_16G 0x18

#define GYRO_SCALE_250 0x00
#define GYRO_SCALE_500 0x08
#define GYRO_SCALE_1000 0x10
#define GYRO_SCALE_2000 0x18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
char uart_buffer[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);
void MPU6050_Read_All(void);
void MPU6050_Calculate_Angles(void);
uint8_t MPU6050_Read_Register(uint8_t reg);
void MPU6050_Write_Register(uint8_t reg, uint8_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Write to MPU6050 register
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval None
  */
void MPU6050_Write_Register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, data, 2, HAL_MAX_DELAY);
}

/**
  * @brief  Read from MPU6050 register
  * @param  reg: Register address
  * @retval Register value
  */
uint8_t MPU6050_Read_Register(uint8_t reg) {
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

/**
  * @brief  Initialize MPU6050
  * @retval 0: Success, 1: Failure
  */
uint8_t MPU6050_Init(void) {
    uint8_t check;
    uint8_t data;
    
    // Check device ID WHO_AM_I
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
    
    if (check == 0x68) {  // 0x68 is the default WHO_AM_I value
        // Wake up MPU6050
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
        
        // Set sample rate to 1kHz
        data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, HAL_MAX_DELAY);
        
        // Configure accelerometer (±2g)
        data = ACCEL_SCALE_2G;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
        
        // Configure gyroscope (±250 deg/s)
        data = GYRO_SCALE_250;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
        
        // Set digital low-pass filter
        data = 0x03;  // ~44Hz bandwidth
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
        
        return 0;
    }
    return 1;
}

/**
  * @brief  Read accelerometer data
  * @retval None
  */
void MPU6050_Read_Accel(void) {
    uint8_t data[6];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, HAL_MAX_DELAY);
    
    MPU6050.Accel_X_RAW = (int16_t)(data[0] << 8 | data[1]);
    MPU6050.Accel_Y_RAW = (int16_t)(data[2] << 8 | data[3]);
    MPU6050.Accel_Z_RAW = (int16_t)(data[4] << 8 | data[5]);
    
    // Convert to g (±2g range -> 16384 LSB/g)
    MPU6050.Ax = MPU6050.Accel_X_RAW / 16384.0f;
    MPU6050.Ay = MPU6050.Accel_Y_RAW / 16384.0f;
    MPU6050.Az = MPU6050.Accel_Z_RAW / 16384.0f;
}

/**
  * @brief  Read gyroscope data
  * @retval None
  */
void MPU6050_Read_Gyro(void) {
    uint8_t data[6];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, HAL_MAX_DELAY);
    
    MPU6050.Gyro_X_RAW = (int16_t)(data[0] << 8 | data[1]);
    MPU6050.Gyro_Y_RAW = (int16_t)(data[2] << 8 | data[3]);
    MPU6050.Gyro_Z_RAW = (int16_t)(data[4] << 8 | data[5]);
    
    // Convert to deg/s (±250 deg/s range -> 131 LSB/deg/s)
    MPU6050.Gx = MPU6050.Gyro_X_RAW / 131.0f;
    MPU6050.Gy = MPU6050.Gyro_Y_RAW / 131.0f;
    MPU6050.Gz = MPU6050.Gyro_Z_RAW / 131.0f;
}

/**
  * @brief  Read temperature data
  * @retval None
  */
void MPU6050_Read_Temp(void) {
    uint8_t data[2];
    
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, data, 2, HAL_MAX_DELAY);
    
    MPU6050.Temp_RAW = (int16_t)(data[0] << 8 | data[1]);
    
    // Convert to Celsius: Temp = (TEMP_OUT / 340) + 36.53
    MPU6050.Temperature = (MPU6050.Temp_RAW / 340.0f) + 36.53f;
}

/**
  * @brief  Calculate roll and pitch angles
  * @retval None
  */
void MPU6050_Calculate_Angles(void) {
    // Calculate roll and pitch from accelerometer
    MPU6050.Roll = atan2(MPU6050.Ay, MPU6050.Az) * 180.0f / M_PI;
    MPU6050.Pitch = atan2(-MPU6050.Ax, sqrt(MPU6050.Ay * MPU6050.Ay + MPU6050.Az * MPU6050.Az)) * 180.0f / M_PI;
}

/**
  * @brief  Read all sensor data
  * @retval None
  */
void MPU6050_Read_All(void) {
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    MPU6050_Read_Temp();
    MPU6050_Calculate_Angles();
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Initialize MPU6050
  if (MPU6050_Init() == 0) {
      sprintf(uart_buffer, "MPU6050 Initialized Successfully!\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
  } else {
      sprintf(uart_buffer, "MPU6050 Initialization Failed!\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
      while(1);  // Stop if initialization failed
  }
  
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Read all sensor data
    MPU6050_Read_All();
    
    // Print accelerometer data
    sprintf(uart_buffer, "Accel: X=%.2f g, Y=%.2f g, Z=%.2f g\r\n", 
            MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Print gyroscope data
    sprintf(uart_buffer, "Gyro:  X=%.2f °/s, Y=%.2f °/s, Z=%.2f °/s\r\n", 
            MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Print temperature
    sprintf(uart_buffer, "Temp:  %.2f °C\r\n", MPU6050.Temperature);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    // Print calculated angles
    sprintf(uart_buffer, "Angles: Roll=%.2f°, Pitch=%.2f°\r\n", 
            MPU6050.Roll, MPU6050.Pitch);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    sprintf(uart_buffer, "-----------------------------------\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    
    HAL_Delay(500);  // Read every 500ms
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */