/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BL_RX_LEN  100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Boot = 0;
uint8_t supported_commands[] = {
                               GET_VER ,
                               GET_HELP,
                               GET_CID,
                               GET_RDP_STATUS,
                               GO_TO_ADDR,
                               FLASH_ERASE,
                               MEM_WRITE,
							   READ_SECTOR_PROTECTION_STATUS} ;
char somedata[] = "Hello from Bootloader\r\n";
uint8_t bl_rx_buffer[BL_RX_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

void Jump_To_App();

static void printmsg(char *format,...);






/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void flash_testing(void)
{

	uint8_t protection_mode =2 ;
	uint8_t sector_details = 0x80;

	//Flash option control register (OPTCR)
	volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	  			//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//here wer are setting read and write protection for the sectors
			//set the 31st bit
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR |= (1 << 31);

			//put write protection on sectors
	    *pOPTCR &= ~(0xff << 16);
			*pOPTCR |= (sector_details << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();
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
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


	  for(int i=0;i<10;i++)
	  {
		  HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
		  HAL_Delay(250);
	  }

	  if(Boot == 1)
	  {
		  printmsg("Bootloader: Entering into bootloader Mode\r\n");
		  Read_Host();
	  }
	  else
		  Jump_To_App();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Red_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Red_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == User_Button_Pin)
	{
		Boot = 1;
	}
}

void Jump_To_App()
{
	void (*app_reset_handler)(void);
	uint32_t msp_value = *(volatile uint32_t *) (0x08008000);
	__set_MSP(msp_value);
	uint32_t reset_handler_address = *(volatile uint32_t *) (0x08008000 + 4);
	app_reset_handler = (void*) (reset_handler_address);
	app_reset_handler();

}

void printmsg(char *format,...)
{
	char str[80];

		/*Extract the the argument list using VA apis */
		va_list args;
		va_start(args, format);
		vsprintf(str, format,args);
		HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str),HAL_MAX_DELAY);
		va_end(args);
}

void  Read_Host(void)
{
    uint8_t rcv_len=0;

	while(1)
	{
		memset(bl_rx_buffer,0,100);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
    HAL_UART_Receive(&huart1,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];
		HAL_UART_Receive(&huart1,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1])
		{
            case GET_VER:
            	Get_Version_cmd(bl_rx_buffer);
                break;
            case GET_HELP:
            	Get_Help_cmd(bl_rx_buffer);
                break;
            case GET_CID:
            	Get_Chip_ID_cmd(bl_rx_buffer);
                break;
            case GET_RDP_STATUS:
            	Get_Flash_Read_Protection_Level_cmd(bl_rx_buffer);
                break;
            case GO_TO_ADDR:
                 Go_cmd(bl_rx_buffer);
                break;
            case FLASH_ERASE:
                 flash_erase_cmd(bl_rx_buffer);
                break;
            case MEM_WRITE:
                 mem_write_cmd(bl_rx_buffer);
                break;
            case EN_RW_PROTECT:
                 en_rw_protect(bl_rx_buffer);
                break;
            case MEM_READ:
                 mem_read(bl_rx_buffer);
                break;
            case READ_SECTOR_PROTECTION_STATUS:
                 read_sector_protection_status(bl_rx_buffer);
                break;
            case OTP_READ:
                 read_otp(bl_rx_buffer);
                break;
			case DISABLE_R_W_PROTECT:
                 dis_rw_protect(bl_rx_buffer);
                break;
             default:
                printmsg("Bootloader:Invalid command code received from host \r\n");
                break;


		}

	}

}
void  Get_Version_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) verify the checksum
      printmsg("Bootloader: getver_cmd\n");

	 //Total length of the command packet
	  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	  //extract the CRC32 sent by the Host
	  uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

    if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
        printmsg("Bootloader:checksum success !!\n");
        // checksum is correct..
        Send_Ack(bl_rx_buffer[0],1);
        bl_version=Get_bootloader_version();
        printmsg("Bootloader:BL_VER : %d %#x\n",bl_version,bl_version);
        bootloader_uart_write_data(&bl_version,1);

    }else
    {
        printmsg("Bootloader:checksum fail !!\n");
        //checksum is wrong send nack
        Send_Nack();
    }


}

/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void  Get_Help_cmd(uint8_t *pBuffer)
{
    printmsg("Bootloader: Get_Help_cmd_cmd\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],sizeof(supported_commands));
        bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}

}

/*Helper function to handle BL_GET_CID command */
void  Get_Chip_ID_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	printmsg("Bootloader: Get_Chip_ID_cmd\r\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],2);
        bl_cid_num = Get_MCU_Chip_ID();
        printmsg("Bootloader:MCU id : %d %#x !!\n",bl_cid_num, bl_cid_num);
        bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}


}

/*Helper function to handle BL_GET_RDP_STATUS command */
void  Get_Flash_Read_Protection_Level_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    printmsg("Bootloader: Get_Flash_Read_Protection_Level_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],1);
        rdp_level = Get_Flash_RDP_Level();
        printmsg("Bootloader:RDP level: %d %#x\n",rdp_level,rdp_level);
        bootloader_uart_write_data(&rdp_level,1);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}


}

/*Helper function to handle BL_GO_TO_ADDR command */
void  Go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    printmsg("Bootloader: Go_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");

        Send_Ack(pBuffer[0],1);

        //extract the go address
        go_address = *((uint32_t *)&pBuffer[2] );
        printmsg("Bootloader:GO addr: %#x\n",go_address);

        if( Verify_Address(go_address) == ADDR_VALID )
        {
            //tell host that address is fine
            bootloader_uart_write_data(&addr_valid,1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

            go_address+=1; //make T bit =1

            void (*lets_jump)(void) = (void *)go_address;

            printmsg("Bootloader: jumping to go address! \n");

            lets_jump();

		}else
		{
            printmsg("Bootloader:GO addr invalid ! \n");
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}


}

/*Helper function to handle BL_FLASH_ERASE command */
void  flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    printmsg("Bootloader: flash_erase_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],1);
        printmsg("Bootloader:initial_sector : %d  no_ofsectors: %d\n",pBuffer[2],pBuffer[3]);

        HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);
        erase_status = Execute_Flash_Erase(pBuffer[2] , pBuffer[3]);
        HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);

        printmsg("Bootloader: flash erase status: %#x\n",erase_status);

        bootloader_uart_write_data(&erase_status,1);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}
}

/*Helper function to handle BL_MEM_WRITE command */
void  mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];

    printmsg("Bootloader: mem_write_cmd\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");

        Send_Ack(pBuffer[0],1);

        printmsg("Bootloader: mem write address : %#x\n",mem_address);

		if( Verify_Address(mem_address) == ADDR_VALID )
		{

            printmsg("Bootloader: valid mem write address\n");

            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);

            //execute mem write
            write_status = Execute_Mem_Write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}else
		{
            printmsg("Bootloader: invalid mem write address\n");
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}

}

/*Helper function to handle BL_EN_RW_PROTECT  command */
void  en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg("Bootloader: endis_rw_protect\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],1);

        status = Set_Flash_Sector_RW_Protection(pBuffer[2] , pBuffer[3],0);

        printmsg("Bootloader: flash erase status: %#x\n",status);

        bootloader_uart_write_data(&status,1);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}


}


/*Helper function to handle BL_EN_RW_PROTECT  command */
void  dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg("Bootloader: dis_rw_protect\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],1);

        status = Set_Flash_Sector_RW_Protection(0,0,1);

        printmsg("Bootloader: flash erase status: %#x\n",status);

        bootloader_uart_write_data(&status,1);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}


}

/*Helper function to handle BL_MEM_READ command */
void  mem_read (uint8_t *pBuffer)
{


}

/*Helper function to handle READ_SECTOR_PROTECTION_STATUS command */
void  read_sector_protection_status(uint8_t *pBuffer)
{
	 uint16_t status;
	printmsg("Bootloader: read_sector_protection_status\n");

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! Check_CRC(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg("Bootloader:checksum success !!\n");
        Send_Ack(pBuffer[0],2);
        status=Read_OB_RW_Protection_Status();
        printmsg("Bootloader: nWRP status: %#x\n",status);
        bootloader_uart_write_data((uint8_t*)&status,2);

	}else
	{
        printmsg("Bootloader:checksum fail !!\n");
        Send_Nack();
	}

}

/*Helper function to handle BL_OTP_READ command */
void  read_otp(uint8_t *pBuffer)
{


}

/*This function sends ACK if CRC matches along with "len to follow"*/
void Send_Ack(uint8_t command_code, uint8_t follow_len)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(&huart1,ack_buf,2,HAL_MAX_DELAY);

}

/*This function sends NACK */
void Send_Nack(void)
{
	uint8_t nack = NACK;
	HAL_UART_Transmit(&huart1,&nack,1,HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t Check_CRC (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return CRC_SUCCESS;
	}

	return CRC_FAIL;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(&huart1,pBuffer,len,HAL_MAX_DELAY);

}


//Just returns the macro value .
uint8_t Get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

//Read the chip identifier or device Identifier
uint16_t Get_MCU_Chip_ID(void)
{
/*
	The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;

}


/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 9. Description of the option bytes" in stm32f446xx RM
 */
uint8_t Get_Flash_RDP_Level(void)
{

	uint8_t rdp_status=0;
#if 0
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	 rdp_status =  (uint8_t)(*pOB_addr >> 8) ;
#endif

	return rdp_status;

}

//verify the address sent by the host .
uint8_t Verify_Address(uint32_t go_address)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.

//incomplete -poorly written .. optimize it
	if ( go_address >= CCRAM_BASE && go_address <= CCRAM_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= RAM_BASE && go_address <= RAM_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
	return ADDR_INVALID;
}

 uint8_t Execute_Flash_Erase(uint8_t sector_number , uint8_t number_of_sector)
{

    //we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors

	//FLASH_EraseInitTypeDef flashErase_handle;
	//uint32_t sectorError;
	//HAL_StatusTypeDef status;


	//if( number_of_sector > 8 )
	//	return INVALID_SECTOR;

//	if( (sector_number == 0xff ) || (sector_number <= 7) )
//	{
//		if(sector_number == (uint8_t) 0xff)
//		{
//			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
//		}else
//		{
		    /*Here we are just calculating how many sectors needs to erased */
//			uint8_t remanining_sector = 8 - sector_number;
//            if( number_of_sector > remanining_sector)
//            {
//            	number_of_sector = remanining_sector;
//            }
//			flashErase_handle.TypeErase = TYPEERASE_SECTORS;
//			flashErase_handle.PageAddress = sector_number; // this is the initial sector
//			flashErase_handle.NbPages = number_of_sector;
//		}
//		flashErase_handle.Banks = FLASH_BANK_1;
//
//		/*Get access to touch the flash registers */
//		HAL_FLASH_Unlock();
//		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
//		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
//		HAL_FLASH_Lock();
//
//		return status;
//	}
//
//
	printmsg("Bootloader: Under Development Phase\r\n");
	return INVALID_SECTOR;
}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t Execute_Mem_Write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,mem_address+i,pBuffer[i] );
    }

    HAL_FLASH_Lock();

    return status;
}


/*
Modifying user option bytes
To modify the user option value, follow the sequence below:
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2. Write the desired option value in the FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
4. Wait for the BSY bit to be cleared.
*/
uint8_t Set_Flash_Sector_RW_Protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    //First configure the protection mode
    //protection_mode =1 , means write protect of the user flash sectors
    //protection_mode =2, means read/write protect of the user flash sectors
    //According to RM of stm32f446xx TABLE 9, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)

	 //Flash option control register (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	  if(disable)
		{

			//disable all r/w protection on sectors

			//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//clear the 31st bit (default state)
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR &= ~(1 << 31);

			//clear the protection : make all bits belonging to sectors as 1
			*pOPTCR |= (0xFF << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();

			return 0;

		}

	   if(protection_mode == (uint8_t) 1)
    {
           //we are putting write protection on the sectors encoded in sector_details argument

			//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//here we are setting just write protection for the sectors
			//clear the 31st bit
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR &= ~(1 << 31);

			//put write protection on sectors
			*pOPTCR &= ~ (sector_details << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();
		}

		else if (protection_mode == (uint8_t) 2)
    {
	  	//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//here wer are setting read and write protection for the sectors
			//set the 31st bit
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR |= (1 << 31);

			//put read and write protection on sectors
            *pOPTCR &= ~(0xff << 16);
			*pOPTCR |= (sector_details << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();
    }

		return 0;
}

uint16_t Read_OB_RW_Protection_Status(void)
{
    //This structure is given by ST Flash driver to hold the OB(Option Byte) contents .
	FLASH_OBProgramInitTypeDef OBInit;

	//First unlock the OB(Option Byte) memory access
	HAL_FLASH_OB_Unlock();
	//get the OB configuration details
	HAL_FLASHEx_OBGetConfig(&OBInit);
	//Lock back .
	HAL_FLASH_Lock();

	//We are just interested in r/w protection status of the sectors.
	return (uint16_t)OBInit.WRPPage;

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

#ifdef  USE_FULL_ASSERT
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
