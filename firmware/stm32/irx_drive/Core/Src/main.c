#include "main.h"
#include "bno085.h"

#include "robot_drive.h"
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

static uint8_t uart_rx_char;
static char uart_rx_buf[64];
static uint8_t uart_rx_idx = 0;

static int32_t cmd_left_rpm = 0;
static int32_t cmd_right_rpm = 0;
static uint32_t last_send_tick = 0;

static uint32_t last_imu_tick = 0;

I2C_HandleTypeDef hi2c1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  HAL_UART_Transmit(&huart2, (uint8_t*)"BOOT\r\n", strlen("BOOT\r\n"), 100);

  MX_CAN1_Init();

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    char buf[64];
    sprintf(buf, "CAN_FAIL ERR=%lu\r\n", HAL_CAN_GetError(&hcan1));
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
    Error_Handler();
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)"CAN_OK\r\n", strlen("CAN_OK\r\n"), 100);
  HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);

  MX_I2C1_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"I2C_OK\r\n", 8, 100);

  if (BNO085_Init(&hi2c1) != HAL_OK)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"IMU_FAIL\r\n", 10, 100);
  }
  else
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"IMU_OK\r\n", 8, 100);
  }




  while (1)
  {
      uint32_t now = HAL_GetTick();  /* must be first */

      BNO085_Process();

      if ((now - last_imu_tick) >= 20)
      {
          last_imu_tick = now;
          BNO085_Data_t *imu = BNO085_GetData();
          char buf[128];
          int len = snprintf(buf, sizeof(buf),
              "IMU %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\r\n",
              imu->qw, imu->qx, imu->qy, imu->qz,
              imu->ax, imu->ay, imu->az,
              imu->gx, imu->gy, imu->gz);
          HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 10);
      }

      if ((now - last_send_tick) >= 50)
      {
          last_send_tick = now;
          robot_set_rpm(cmd_left_rpm, cmd_right_rpm);
      }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    char c = (char)uart_rx_char;

    if (c == '\r' || c == '\n')
    {
      uart_rx_buf[uart_rx_idx] = '\0';

      int32_t l = 0;
      int32_t r = 0;

      if (sscanf(uart_rx_buf, "CMD %ld %ld", &l, &r) == 2)
      {
        cmd_left_rpm = l;
        cmd_right_rpm = r;

        char buf[64];
        sprintf(buf, "OK L=%ld R=%ld\r\n", l, r);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
      else if (uart_rx_idx > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)"BAD_CMD\r\n", strlen("BAD_CMD\r\n"), 100);
      }

      uart_rx_idx = 0;
    }
    else
    {
      if (uart_rx_idx < sizeof(uart_rx_buf) - 1)
      {
        uart_rx_buf[uart_rx_idx++] = c;
      }
      else
      {
        uart_rx_idx = 0;
        HAL_UART_Transmit(&huart2, (uint8_t*)"BUF_FULL\r\n", strlen("BUF_FULL\r\n"), 100);
      }
    }

    HAL_UART_Receive_IT(&huart2, &uart_rx_char, 1);
  }
}

static void MX_CAN1_Init(void)
{
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
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
  }
}
