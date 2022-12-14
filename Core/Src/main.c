/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "usbd_cdc_if.h"
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
gb_metadata_t gb_metadata = {0};

uint8_t usb_rx_buffer[APP_RX_DATA_SIZE];

uint8_t gb_rom_buff[ROM_BUFFER_SIZE];


uint8_t tetris_header[]  = {
	0x00, 0xC3, 0x50, 0x01, 0xCE, 0xED, 0x66, 0x66,
	0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83,
	0x00, 0x0C, 0x00, 0x0D, 0x00, 0x08, 0x11, 0x1F,
	0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6,
	0xDD, 0xDD, 0xD9, 0x99, 0xBB, 0xBB, 0x67, 0x63,
	0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F,
	0xBB, 0xB9, 0x33, 0x3E, 0x54, 0x45, 0x54, 0x52,
	0x49, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x0A, 0x16, 0xBF
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
//void init_test_gb_metadata(void)
//{
//	for(uint8_t i = 0; i < ROM_TITLE_LEN; ++i)
//	{
//		gb_metadata.rom_title[i] = i+65;
//	}
//
//	gb_metadata.manufacturers_code = 0x69;
//	gb_metadata.cgb_flag           = CGB_SUPPORT;
//	gb_metadata.new_licensee_code  = CAPCOM;
//	gb_metadata.sgb_flag           = SGB_NO;
//	gb_metadata.cart_type          = MBC1_RAM_BATT;
//	gb_metadata.rom_size           = ROM_512KB;
//	gb_metadata.ram_size           = RAM_8KB;
//	gb_metadata.dest_code          = NO_JAP;
//	gb_metadata.old_licensee_code  = 0x00;
//	gb_metadata.mask_rom_ver_num   = 0x01;
//	gb_metadata.checksum           = 0x03;
//}


void unpack_metadata(gb_metadata_t* md, uint8_t* buff)
{
	memcpy(&md->entry_point,        buff + HEADER_ADDR(CART_ENTRY_POINT),  ENTRY_POINT_LEN);
	memcpy(&md->rom_title,          buff + HEADER_ADDR(CART_TITLE),        ROM_TITLE_LEN);
	memcpy(&md->rom_title[11],      buff + HEADER_ADDR(CART_MAUFACTURER),  MAN_CODE_LEN);
	memcpy(&md->rom_title[15],      buff + HEADER_ADDR(CART_CGB_FLAG),     1);
	memcpy(&md->new_licensee_code,  buff + HEADER_ADDR(CART_NEW_LIC_CODE), 2);
	memcpy(&md->sgb_flag,           buff + HEADER_ADDR(CART_SGB_FLAG),     1);
	memcpy(&md->cart_type,          buff + HEADER_ADDR(CART_TYPE),         1);
	memcpy(&md->rom_size,           buff + HEADER_ADDR(CART_ROM_SIZE),     1);
	memcpy(&md->ram_size,           buff + HEADER_ADDR(CART_RAM_SIZE),     1);
	memcpy(&md->dest_code,          buff + HEADER_ADDR(CART_DEST_CODE),    1);
	memcpy(&md->old_licensee_code,  buff + HEADER_ADDR(CART_OLD_LIC_CODE), 1);
	memcpy(&md->mask_rom_ver_num,   buff + HEADER_ADDR(CART_MASK_ROM_VER), 1);
	memcpy(&md->checksum,           buff + HEADER_ADDR(CART_HEADER_CHECK), 1);
}


void address_bus_init();

//port b clock already started. no need to call this
void data_bus_init()
{
	// input mode & floating input are reset state so no need to set
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
}

inline uint8_t data_bus_read(void)
{
	// TODO: figure out what these need to be set to
//	HAL_GPIO_WritePin(data_bus_dir_GPIO_Port, data_bus_dir_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(gb_cs_GPIO_Port, gb_cs_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(gb_read_GPIO_Port, gb_read_Pin, GPIO_PIN_RESET);
	return DATA_BUS_PORT->IDR & !DATA_BUS_BITMASK;
}
void gb_set_read(void);
void gb_set_address(uint16_t addr)
{
	// TODO: figure out what these need to be set to
	gb_set_read();
	HAL_GPIO_WritePin(sr_latch_GPIO_Port, sr_latch_Pin, GPIO_PIN_RESET);

	for(uint8_t i = 0; i < ADDR_BUS_WIDTH; ++i)
	{
		HAL_GPIO_WritePin(sr_clk_GPIO_Port, sr_clk_Pin, GPIO_PIN_RESET);

		if(0x01 && (addr >> i))
		{
			HAL_GPIO_WritePin(sr_data_GPIO_Port, sr_data_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(sr_data_GPIO_Port, sr_data_Pin, GPIO_PIN_RESET);
		}


		HAL_GPIO_WritePin(sr_clk_GPIO_Port, sr_clk_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(sr_latch_GPIO_Port, sr_latch_Pin, GPIO_PIN_SET);
}

void gb_set_read(void)
{
//		HAL_GPIO_WritePin(addr_bus_dir_GPIO_Port, addr_bus_dir_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_cs_GPIO_Port, gb_cs_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_read_GPIO_Port, gb_read_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_write_GPIO_Port, gb_write_Pin, GPIO_PIN_SET);
}

void gb_set_write(void)
{
//		HAL_GPIO_WritePin(addr_bus_dir_GPIO_Port, addr_bus_dir_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_cs_GPIO_Port, gb_cs_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_read_GPIO_Port, gb_read_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(gb_write_GPIO_Port, gb_write_Pin, GPIO_PIN_SET);
}

uint8_t gb_fetch_data(uint16_t addr)
{
	gb_set_address(addr);

	return data_bus_read();
}

inline uint8_t gb_header_checksum(gb_metadata_t *metadata)
{
	uint8_t checksum = 0;
	uint8_t *addr = (uint8_t*)&metadata->rom_title[0];

	while(addr < &metadata->checksum)
	{
		checksum -= (*addr++) - 1;
	}

	return checksum;
}

bool client_send_metadata(gb_metadata_t *metadata)
{
	client_instruction_e instruction = START_SEND_METADATA;

	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&start_delimiter, sizeof(uint8_t)));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&instruction,     sizeof(uint8_t)));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)metadata,         30));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&end_delimiter,   sizeof(uint8_t)));

	return true;
}

void client_listen(void)
{

}

void client_send_chunk(uint8_t* data, uint8_t len);



//void set_test_metadata(gb_metadata_t *metadata)
//{
//	char title[ROM_TITLE_LEN] = "Testicles";
//
//	memcpy(gb_metadata.rom_title, title, ROM_TITLE_LEN);
//
//	gb_metadata.manufacturers_code = 0x69;
//	gb_metadata.cgb_flag           = CGB_SUPPORT;
//	gb_metadata.new_licensee_code  = CAPCOM;
//	gb_metadata.sgb_flag           = SGB_SUPPORT;
//	gb_metadata.cart_type          = POCKET_CAMERA;
//	gb_metadata.rom_size           = ROM_512KB;
//	gb_metadata.ram_size           = RAM_8KB;
//	gb_metadata.dest_code          = NO_JAP;
//	gb_metadata.old_licensee_code  = 0x00;
//	gb_metadata.mask_rom_ver_num   = 0x01;
//	gb_metadata.checksum           = 0x03;
//}

void send_test_metadata(gb_metadata_t *metadata)
{
	unpack_metadata(metadata, tetris_header);
	client_send_metadata(metadata);
}


void send_test_rom(gb_metadata_t *metadata, uint8_t num_pages)
{
	set_test_metadata(metadata);

	client_instruction_e instruction = START_SEND_ROM;
	const uint16_t size = sizeof(uint16_t) + sizeof(gb_metadata_t) + (ROM_BUFFER_SIZE * num_pages);

	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&start_delimiter,        sizeof(uint8_t)));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&instruction,            sizeof(uint8_t)));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&size,                   sizeof(uint8_t)));
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)(&size)+sizeof(uint8_t), sizeof(uint8_t))); // size >> 8. compiler yelled
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)metadata,                sizeof(gb_metadata_t)));

	// generate buffer
	for(uint8_t page = 0; page < num_pages; ++page)
	{
		for(uint16_t i = 0; i < ROM_BUFFER_SIZE; i+=2)
		{
			gb_rom_buff[i] = 'A' + page;

			if(i < ROM_BUFFER_SIZE - 1)
			{
				gb_rom_buff[i + 1] = i;
			}
		}
		while(USBD_BUSY == CDC_Transmit_FS(gb_rom_buff, sizeof(ROM_BUFFER_SIZE)));
	}
	// tx checksum
	while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)&end_delimiter,   sizeof(uint8_t)));
}


void dump_rom()
{
	for(uint16_t i = 0; i < 64; ++i)
	{

	}
}


void dump_ram();


bool get_metadata(gb_metadata_t *gb_metadata);
void get_title();


void write_oled();

//uint8_t gb_calc_checksum(uint8_t* rom);
//for usb com port stuff :  https://www.youtube.com/watch?v=92A98iEFmaA


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
//	uint8_t usb_tx_buff[USB_TX_BUFF_LEN];

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
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
//  init_test_gb_metadata();
//  set_test_metadata(&gb_metadata);
  	 unpack_metadata(&gb_metadata, tetris_header);


//  char* msg = "GB rom dumper usb serial";


//  memcpy(&gb_metadata, buf, sizeof(gb_msetadata_t));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(Onboard_LED_GPIO_Port, Onboard_LED_Pin);
//	  while(USBD_BUSY == CDC_Transmit_FS((uint8_t*)msg, strlen(msg)));
//	  while(USBD_BUSY == CDC_Transmit_FS(buf, 10));
	  client_send_metadata(&gb_metadata);
	  HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Onboard_LED_GPIO_Port, Onboard_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, sr_latch_Pin|sr_clk_Pin|sr_data_Pin|data_bus_dir_Pin
                          |gb_write_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, signal_bus_dir_Pin|addr_bus_dir_Pin|gb_read_Pin|gb_cs_Pin
                          |gb_rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Onboard_LED_Pin */
  GPIO_InitStruct.Pin = Onboard_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Onboard_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : sr_latch_Pin sr_clk_Pin sr_data_Pin data_bus_dir_Pin
                           gb_write_Pin */
  GPIO_InitStruct.Pin = sr_latch_Pin|sr_clk_Pin|sr_data_Pin|data_bus_dir_Pin
                          |gb_write_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : signal_bus_dir_Pin addr_bus_dir_Pin gb_read_Pin gb_cs_Pin
                           gb_rst_Pin */
  GPIO_InitStruct.Pin = signal_bus_dir_Pin|addr_bus_dir_Pin|gb_read_Pin|gb_cs_Pin
                          |gb_rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

