/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

//This command is used to read the bootloader version from the MCU
#define GET_VER				0x51
#define GET_HELP				0x52
#define GET_CID				0x53
#define GET_RDP_STATUS		0x54
#define GO_TO_ADDR			0x55
#define FLASH_ERASE          0x56
#define MEM_WRITE			0x57
#define EN_RW_PROTECT		0x58
#define MEM_READ				0x59
#define READ_SECTOR_P_STATUS	0x5A
#define OTP_READ				0x5B



#define ACK		0xA5
#define NACK	0x7F

#define CRC_OK		0x01
#define CRC_ERROR	0x00


uint8_t supported_commands[] ={
		GET_VER,
		GET_HELP,
		GET_CID,
		GET_RDP_STATUS,
		GO_TO_ADDR,
		FLASH_ERASE,
		MEM_WRITE,
		READ_SECTOR_P_STATUS
};
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_Button_Pin GPIO_PIN_0
#define User_Button_GPIO_Port GPIOA
#define User_Button_EXTI_IRQn EXTI0_IRQn
#define LED_Red_Pin GPIO_PIN_9
#define LED_Red_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
