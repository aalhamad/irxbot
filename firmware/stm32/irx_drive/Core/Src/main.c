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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno085.h"
#include "robot_drive.h"
#include "vl53l1x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t last_imu_tick     = 0;
static uint32_t last_odo_tick     = 0;
static uint32_t last_err_tick[4]  = {0};
static uint8_t  cmd_buf[32];
static uint8_t  cmd_len           = 0;
static int32_t  cmd_left          = 0;
static int32_t  cmd_right         = 0;
static uint32_t last_cmd_tick     = 0;
static uint8_t  bumper_last       = 0xFF; /* force send on first loop */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, (uint8_t*)"BOOT\r\n", 6, 100);

  /* CAN: accept-all filter then start */
  {
      CAN_FilterTypeDef f = {0};
      f.FilterBank           = 0;
      f.FilterMode           = CAN_FILTERMODE_IDMASK;
      f.FilterScale          = CAN_FILTERSCALE_32BIT;
      f.FilterFIFOAssignment = CAN_RX_FIFO0;
      f.FilterActivation     = ENABLE;
      if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK ||
          HAL_CAN_Start(&hcan1) != HAL_OK)
          HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:CAN\r\n", 9, 100);
      else
          HAL_UART_Transmit(&huart2, (uint8_t*)"CAN_OK\r\n", 8, 100);
  }

  /* IMU */
  if (BNO085_Init(&hi2c1) != HAL_OK)
      HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:IMU\r\n", 9, 100);
  else
      HAL_UART_Transmit(&huart2, (uint8_t*)"IMU_OK\r\n", 8, 100);

  /* ToF */
  {
      uint8_t n = ToF_InitAll(&hi2c1);
      char buf[32];
      int  len = snprintf(buf, sizeof(buf), "TOF_OK %u/4\r\n", n);
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();

      /* ── IMU ── */
      BNO085_Process();

      /* ── ToF ── */
      ToF_Process();

      /* ── UART RX: accumulate CMD <left> <right> ── */
      while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
      {
          uint8_t b = (uint8_t)(huart2.Instance->DR & 0xFF);
          if (b == '\n' || b == '\r')
          {
              if (cmd_len == 0) continue;   /* ignore empty / duplicate CR+LF */
              cmd_buf[cmd_len] = '\0';
              if (cmd_len >= 7 &&
                  cmd_buf[0]=='C' && cmd_buf[1]=='M' &&
                  cmd_buf[2]=='D' && cmd_buf[3]==' ')
              {
                  char *p   = (char*)cmd_buf + 4;
                  int32_t l = strtol(p, &p, 10);
                  int32_t r = (*p == ' ') ? strtol(p + 1, NULL, 10) : 0;

                  char ack[32];
                  int an = snprintf(ack, sizeof(ack), "CMD_OK %ld %ld\r\n", l, r);
                  HAL_UART_Transmit(&huart2, (uint8_t*)ack, an, 50);

                  cmd_left  = l;
                  cmd_right = r;
                  last_cmd_tick = now;
                  robot_set_rpm(l, r);

                  /* Report CAN peripheral error state after the send attempt */
                  HAL_Delay(5);
                  vesc_can_dump_status();
              }
              else
              {
                  HAL_UART_Transmit(&huart2, (uint8_t*)"ERR:CMD\r\n", 9, 50);
              }
              cmd_len = 0;
          }
          else if (cmd_len < (uint8_t)(sizeof(cmd_buf) - 1))
          {
              cmd_buf[cmd_len++] = b;
          }
      }

      /* ── CAN RX: update cached VESC feedback ── */
      vesc_process_rx();

      /* ── Periodic CMD resend @ 200 ms — keeps VESCs alive within their 1s timeout ── */
      if (last_cmd_tick > 0 && (now - last_cmd_tick) >= 200)
      {
          last_cmd_tick = now;
          robot_set_rpm(cmd_left, cmd_right);
      }

      /* ── IMU report @ 10 s (debug rate) ── */
      if ((now - last_imu_tick) >= 5000)
      {
          last_imu_tick = now;
          BNO085_Data_t *imu = BNO085_GetData();
          if (imu->valid)
          {
              char buf[128];
              int n = snprintf(buf, sizeof(buf),
                  "IMU %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
                  imu->qw, imu->qx, imu->qy, imu->qz,
                  imu->ax, imu->ay, imu->az,
                  imu->gx, imu->gy, imu->gz);
              HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 10);
          }
      }

      /* ── Bumpers: send BMP on any change ── */
      {
          uint8_t bmp = 0;
          if (HAL_GPIO_ReadPin(BUMPER_FL_GPIO_Port, BUMPER_FL_Pin) == GPIO_PIN_SET) bmp |= 1;
          if (HAL_GPIO_ReadPin(BUMPER_FR_GPIO_Port, BUMPER_FR_Pin) == GPIO_PIN_SET) bmp |= 2;
          if (HAL_GPIO_ReadPin(BUMPER_RL_GPIO_Port, BUMPER_RL_Pin) == GPIO_PIN_SET) bmp |= 4;
          if (HAL_GPIO_ReadPin(BUMPER_RR_GPIO_Port, BUMPER_RR_Pin) == GPIO_PIN_SET) bmp |= 8;
          if (bmp != bumper_last)
          {
              bumper_last = bmp;
              char buf[24];
              int n = snprintf(buf, sizeof(buf), "BMP %u %u %u %u\r\n",
                  (bmp >> 0) & 1,
                  (bmp >> 1) & 1,
                  (bmp >> 2) & 1,
                  (bmp >> 3) & 1);
              HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 10);
          }
      }

      /* ── ODO report @ 10 s (debug rate) ── */
      if ((now - last_odo_tick) >= 10000)
      {
          last_odo_tick = now;

          /* FL = left reference, FR = right (negated — commanded inverse) */
          int32_t l_rpm = 0, r_rpm = 0;
          vesc_get_rpm(VESC_FL, &l_rpm);
          vesc_get_rpm(VESC_FR, &r_rpm);
          r_rpm = -r_rpm;

          char buf[32];
          int n = snprintf(buf, sizeof(buf), "ODO %ld %ld\r\n", l_rpm, r_rpm);
          HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 10);

          /* Report any VESC that has stopped responding (rate-limited to 1 per 2 s) */
          for (uint8_t id = 1; id <= 4; id++)
          {
              uint32_t last = vesc_last_seen(id);
              if (last == 0) continue;  /* never heard — VESC may not be powered */
              if ((now - last) > 1000 && (now - last_err_tick[id-1]) > 2000)
              {
                  last_err_tick[id-1] = now;
                  char err[16];
                  int en = snprintf(err, sizeof(err), "ERR:ODO%d\r\n", id);
                  HAL_UART_Transmit(&huart2, (uint8_t*)err, en, 50);
              }
          }
      }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TOF_XSHUT_REAR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOF_XSHUT_FRONT_Pin|TOF_XSHUT_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOF_XSHUT_LEFT_GPIO_Port, TOF_XSHUT_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUMPER_FL_Pin BUMPER_FR_Pin BUMPER_RL_Pin BUMPER_RR_Pin */
  GPIO_InitStruct.Pin = BUMPER_FL_Pin|BUMPER_FR_Pin|BUMPER_RL_Pin|BUMPER_RR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TOF_XSHUT_REAR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TOF_XSHUT_REAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_XSHUT_FRONT_Pin TOF_XSHUT_RIGHT_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_FRONT_Pin|TOF_XSHUT_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_XSHUT_LEFT_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_XSHUT_LEFT_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
