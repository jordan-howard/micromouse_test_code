/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE BEGIN Includes */
#include "core_cm3.h"   // SCB/DWT/CoreDebug for Cortex-M3
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT GPIOA
#define LED_PIN  GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Benchmark results (watch these in the debugger) */
volatile uint32_t sink_flash = 0, sink_ram = 0;
volatile uint32_t cycles_flash = 0, cycles_ram = 0;

/* Benchmark controls */
volatile uint32_t seed = 0x12345678u;
const uint32_t iters = 20000u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);



/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void delay_cycles(volatile uint32_t n);
static inline uint32_t lfsr_step(uint32_t x);

__attribute__((noinline))
uint32_t branch_bench_flash(uint32_t iters, volatile uint32_t seed);

__attribute__((section(".RamFunc"), noinline))
uint32_t branch_bench_ram(uint32_t iters, volatile uint32_t seed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void DWT_Init(void)
{
  /* Enable DWT cycle counter */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_cycles(volatile uint32_t n)
{
  while (n--) { __NOP(); }
}

static inline uint32_t lfsr_step(uint32_t x)
{
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

/* Flash version (lives in .text, executes from Flash) */
__attribute__((noinline))
uint32_t branch_bench_flash(uint32_t iters_local, volatile uint32_t seed_local)
{
  uint32_t x = (uint32_t)seed_local ^ 0xA5A5A5A5u;

  while (iters_local--)
  {
    x = lfsr_step(x);

    if (x & (1u << 0))  x -= 3;        else x += 1;
    if (x & (1u << 1))  x += 7;        else x ^= 0x11;
    if (x & (1u << 2))  x ^= (x >> 1); else x += (x << 2);
    if (x & (1u << 3))  x += (x >> 3); else x -= (x >> 2);
    if (x & (1u << 4))  x ^= 0x1234;   else x += 0x55;
    if (x & (1u << 5))  x -= 0x33;     else x ^= (x << 1);
    if (x & (1u << 6))  x += 0x123;    else x -= 0x77;
    if (x & (1u << 7))  x ^= (x >> 5); else x += (x << 1);
  }

  return x;
}

/* SRAM version (placed in .RamFunc; Cube startup typically copies it to RAM) */
__attribute__((section(".RamFunc"), noinline))
uint32_t branch_bench_ram(uint32_t iters_local, volatile uint32_t seed_local)
{
  uint32_t x = (uint32_t)seed_local ^ 0xA5A5A5A5u;

  while (iters_local--)
  {
    x = lfsr_step(x);

    if (x & (1u << 0))  x -= 3;        else x += 1;
    if (x & (1u << 1))  x += 7;        else x ^= 0x11;
    if (x & (1u << 2))  x ^= (x >> 1); else x += (x << 2);
    if (x & (1u << 3))  x += (x >> 3); else x -= (x >> 2);
    if (x & (1u << 4))  x ^= 0x1234;   else x += 0x55;
    if (x & (1u << 5))  x -= 0x33;     else x ^= (x << 1);
    if (x & (1u << 6))  x += 0x123;    else x -= 0x77;
    if (x & (1u << 7))  x ^= (x >> 5); else x += (x << 1);
  }

  return x;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  //SCB->VTOR = 0x20000000;
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Enable AFIO + disable JTAG so PA15 is usable as GPIO (SWD still works) */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  volatile uint32_t sysclk = 0;
  sysclk = HAL_RCC_GetSysClockFreq();   // should be 72000000

  /* Init DWT cycle counter */
  DWT_Init();

  /* Infinite loop */
  while (1)
  {
    sink_flash = branch_bench_ram(iters, seed);
    seed ^= sink_flash;
    GPIOA->ODR ^= (1U<<15);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 1) Enable HSE and PLL: 8 MHz * 9 = 72 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                 // optional
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2) Bus clocks: HCLK=72, PCLK1=36, PCLK2=72 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_HCLK   |
                                RCC_CLOCKTYPE_PCLK1  |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // HCLK = 72 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // PCLK1 = 36 MHz (max)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // PCLK2 = 72 MHz

  /* 3) Flash latency: 2 wait states @ 72 MHz (also enables prefetch) */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  SystemCoreClockUpdate();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
