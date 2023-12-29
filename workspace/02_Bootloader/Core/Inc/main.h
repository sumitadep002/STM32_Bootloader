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


void  Read_Host(void);
void Jump_To_App(void);
void  Get_Version_cmd(uint8_t *bl_rx_buffer);
void  Get_Help_cmd(uint8_t *pBuffer);
void  Get_Chip_ID_cmd(uint8_t *pBuffer);
void  Get_Flash_Read_Protection_Level_cmd(uint8_t *pBuffer);
void  Go_cmd(uint8_t *pBuffer);
void  flash_erase_cmd(uint8_t *pBuffer);
void  mem_write_cmd(uint8_t *pBuffer);
void  en_rw_protect(uint8_t *pBuffer);
void  mem_read (uint8_t *pBuffer);
void  read_sector_protection_status(uint8_t *pBuffer);
void  read_otp(uint8_t *pBuffer);
void  dis_rw_protect(uint8_t *pBuffer);

void Send_Ack(uint8_t command_code, uint8_t follow_len);
void Send_Nack(void);

uint8_t Check_CRC (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t Get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);

uint16_t Get_MCU_Chip_ID(void);
uint8_t Get_Flash_RDP_Level(void);
uint8_t Verify_Address(uint32_t go_address);
uint8_t Execute_Flash_Erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t Execute_Mem_Write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);

uint8_t Set_Flash_Sector_RW_Protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t Read_OB_RW_Protection_Status(void);

//version 1.0
#define BL_VERSION 0x02

// our bootloader commands

#define GET_VER					0x51
#define GET_HELP				0x52
#define GET_CID					0x53
#define GET_RDP_STATUS			0x54
#define GO_TO_ADDR				0x55
#define FLASH_ERASE         	0x56
#define MEM_WRITE				0x57
#define EN_RW_PROTECT			0x58
#define MEM_READ				0x59
#define READ_SECTOR_PROTECTION_STATUS	0x5A
#define OTP_READ				0x5B
#define DISABLE_R_W_PROTECT			0x5C

/* ACK and NACK bytes*/
#define ACK   0XA5
#define NACK  0X7F

/*CRC*/
#define CRC_FAIL    1
#define CRC_SUCCESS 0

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define INVALID_SECTOR 0x04

/*Some Start and End addresses of different memories of STM32F446xx MCU */
/*Change this according to your MCU */


#define CCRAM_BASE	0x10000000
#define RAM_BASE	0x20000000
#define CCRAM_SIZE	4*1024     // STM32F446RE has 112KB of SRAM1
#define CCRAM_END   (CCRAM_BASE + CCRAM_SIZE)
#define RAM_SIZE    16*1024     // STM32F446RE has 16KB of SRAM2
#define RAM_END     (RAM_BASE + RAM_SIZE)
#define FLASH_SIZE  64*1024     // STM32F446RE has 512KB of SRAM2
#define FLASH_END	(FLASH_BASE + FLASH_SIZE)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_Button_Pin GPIO_PIN_0
#define User_Button_GPIO_Port GPIOA
#define User_Button_EXTI_IRQn EXTI0_IRQn
#define LED_Red_Pin GPIO_PIN_6
#define LED_Red_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
