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
#include "bootloader.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void printmsg(char* format, ...);
void BL_JUMP_USERAPP();
void BL_UART_READ();
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
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
	  BL_UART_READ();
  } else {
	  BL_JUMP_USERAPP();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void printmsg(char* format, ...) {
#ifdef BL_DEBUG_MSG
	char str[80];
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(&huart3, (uint8_t*)str, sizeof(str), HAL_MAX_DELAY);
	va_end(args);
#endif
}

void BL_SendACK(uint8_t follow_len) {
	uint8_t buf[2];
	buf[0] = BL_ACK;
	buf[1] = follow_len;
	HAL_UART_Transmit(&huart2, buf, 2, HAL_MAX_DELAY);
}

void BL_SendNACK() {
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(&huart2, &nack, 1, HAL_MAX_DELAY);
}

uint8_t BL_VerifyCRC(uint8_t *data, uint32_t len, uint32_t received_crc) {
	uint32_t CRCVal = 0xFF;
	for (uint32_t i = 0; i < len; i++) {
		uint32_t i_data = data[i];
		CRCVal = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}
	if (CRCVal == received_crc) {
		return CRC_OK;
	}
	return CRC_ERR;
}

uint8_t BL_VerifyAddr(uint32_t addr) {
	if (addr >= SRAM1_BASE && addr <= SRAM2_BASE) return 1;
	else if (addr > SRAM2_BASE && addr <= 0x2001FFFF ) return 1;
	else if (addr >= 0x08000000 && addr <= 0x080FFFFF) return 1;
	return 0;
}


uint8_t BL_ExecFlashErase(uint8_t sector, uint8_t sector_num) {
	if (sector_num > 11) {
		return 0;
	}
	uint8_t status;
	uint32_t sector_error;
	FLASH_EraseInitTypeDef flash_erase = {0};
	if (sector_num == 0xFF || sector_num == 11) {
		flash_erase.TypeErase = FLASH_TYPEERASE_MASSERASE;
	} else {
		uint8_t sectors_remaining = 8 - sector;
		if (sector_num > sectors_remaining) {
			sector_num = sectors_remaining;
		}
		flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
		flash_erase.Sector = sector;
		flash_erase.NbSectors = sector_num;
	}
	flash_erase.Banks = FLASH_BANK_1;
	HAL_FLASH_Unlock();
	flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	status = (uint8_t) HAL_FLASHEx_Erase(&flash_erase, &sector_error);
	HAL_FLASH_Lock();
	return status;
}

uint8_t BL_FlashWrite(uint8_t* data_buffer, uint32_t addr, uint32_t len) {
	uint8_t status;
	HAL_FLASH_Unlock();
	for (uint32_t i = 0; i < len; i++) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, data_buffer[i]);
	}
	HAL_FLASH_Lock();
	return status;
}

uint8_t BL_FlashRWProtection(uint16_t sectors, uint8_t protection_mode, uint8_t EnOrDi) {
	volatile uint32_t* pFLASH_OPTCR = ((uint32_t*)(0x40023C00 + 0x18));
	if (!EnOrDi) {
		HAL_FLASH_OB_Unlock();
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		*pFLASH_OPTCR &= ~(sectors << 16);
		*pFLASH_OPTCR |= (1 << 1);
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
		return 0;
	}
	if (protection_mode == 1){
		HAL_FLASH_OB_Unlock();
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		*pFLASH_OPTCR |= (0x7FF << 16);
		*pFLASH_OPTCR |= (1 << 1);
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
		return 0;
	} else {
		// TODO RW protection
		return 0;
	}
	return 1;

}

void BL_GetVer(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		uint8_t version = BL_VER;
		BL_SendACK(1);
		HAL_UART_Transmit(&huart2, &version, 1, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_GetHelp(uint8_t* buff) {
	uint8_t cmds[11] = {0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61};
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		HAL_UART_Transmit(&huart2, cmds, sizeof(cmds), HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_GetCID(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint32_t* dID = (uint32_t*) DBGMCU_ID;
		uint32_t id = (*dID) & 0x7FF;
		HAL_UART_Transmit(&huart2, (uint8_t*)&id, 4, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}

void BL_GetRDP(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint8_t rdp_status;
		volatile uint32_t* OB_Addr = (uint32_t*) 0x1FFFC000;
		rdp_status = (uint8_t) (*OB_Addr >> 8);
		HAL_UART_Transmit(&huart2, &rdp_status, 1, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_GotoAddr(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint32_t req_addr = *((uint32_t*) &buff[2]);
		if (BL_VerifyAddr(req_addr)) {
			uint8_t addr_valid = 0;
			HAL_UART_Transmit(&huart2, &addr_valid, 1, HAL_MAX_DELAY);
			req_addr += 1; // t bit must be 1
			void (*jump_dst) (void) = (void*) req_addr;
			jump_dst();
		} else {
			uint8_t addr_valid = 1;
			HAL_UART_Transmit(&huart2, &addr_valid, 1, HAL_MAX_DELAY);
		}
	} else {
		BL_SendNACK();
	}
}
void BL_FlashErase(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint8_t status = BL_ExecFlashErase(buff[2], buff[3]);
		HAL_UART_Transmit(&huart2, &status, 4, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_MemWrite(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint32_t addr = *((uint32_t*) (&buff[2]));
		uint8_t status = BL_FlashWrite(&buff[7], addr, buff[6]);
		HAL_UART_Transmit(&huart2, &status, 1, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_MemRead(uint8_t* buff) {
 // TODO
}
void BL_ReadSectorStatus(uint8_t* buff) {
 // TODO
}
void BL_OTPRead(uint8_t* buff) {
 // TODO
}
void BL_DisRWProtect(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint16_t sectors = (buff[2] << 8) | (buff[3]);
		uint8_t status = BL_FlashRWProtection(sectors, buff[4], DISABLE);
		HAL_UART_Transmit(&huart2, &status, 1, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}
void BL_EnRWPProtect(uint8_t* buff) {
	uint32_t cmd_len = buff[0] + 1;
	uint32_t rcv_crc = *((uint32_t*) (buff + cmd_len - 4));
	if (BL_VerifyCRC(&buff[0], cmd_len - 4, rcv_crc)) {
		BL_SendACK(1);
		uint16_t sectors = (buff[2] << 8) | (buff[3]);
		uint8_t status = BL_FlashRWProtection(sectors, buff[4], ENABLE);
		HAL_UART_Transmit(&huart2, &status, 1, HAL_MAX_DELAY);
	} else {
		BL_SendNACK();
	}
}


void BL_JUMP_USERAPP() {
	void (*UserApp_ResetHandler) (void);
	uint32_t msp_value = *(uint32_t*) 0x08008000U;
	__set_MSP(msp_value);
	uint32_t ResetHandler_Addr = *(volatile uint32_t*) (0x08008000U + 4);
	UserApp_ResetHandler = (void*) ResetHandler_Addr;
	UserApp_ResetHandler();
}
void BL_UART_READ() {
	uint8_t rcv_len = 0;
	uint8_t RX_Buffer[200] = {0};
	for (;;) {
		HAL_UART_Receive(&huart2, RX_Buffer, 1, HAL_MAX_DELAY);
		rcv_len = RX_Buffer[0];
		HAL_UART_Receive(&huart2, &RX_Buffer[1], rcv_len, HAL_MAX_DELAY);
		uint8_t cmd = RX_Buffer[1];
		switch (cmd) {
			case BL_GET_VER:
				BL_GetVer(RX_Buffer);
				break;
			case BL_GET_HELP:
				BL_GetHelp(RX_Buffer);
				break;
			case BL_GET_CID:
				BL_GetCID(RX_Buffer);
				break;
			case BL_GET_RDP_STATUS:
				BL_GetRDP(RX_Buffer);
				break;
			case BL_FLASH_ERASE:
				BL_FlashErase(RX_Buffer);
				break;
			case BL_MEM_WRITE:
				BL_MemWrite(RX_Buffer);
				break;
			case BL_MEM_READ:
				BL_MemRead(RX_Buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				BL_ReadSectorStatus(RX_Buffer);
				break;
			case BL_OTP_READ:
				BL_OTPRead(RX_Buffer);
				break;
			case BL_DIS_RW_PROTECT:
				BL_DisRWProtect(RX_Buffer);
				break;
			case BL_EN_RW_PROTECT:
				BL_EnRWPProtect(RX_Buffer);
				break;
			default:
		}
	}
}
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
