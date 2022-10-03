/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


#define ADDR_BUS_UPPER_PORT    GPIOA
#define ADDR_BUS_UPPER_BITMASK 0x07FE // Port A (1 - 10)

#define ADDR_BUS_LOWER_PORT    GPIOB
#define ADDR_BUS_LOWER_BITMASK 0x0004 // Port B (0 - 4)

#define ADDR_BUS_WIDTH         (uint8_t)15

#define DATA_BUS_PORT          GPIOB
#define DATA_BUS_BITMASK       0xFF00 // Port B (8 - 15)
#define DATA_BUS_WIDTH         (uint8_t)8

#define ROM_BUFFER_SIZE 1024


typedef enum
{
	IDLE = 0x00,
	READY_TO_TRANSMIT_METADATA,
	TRANSMITTING_METADATA,
	READY_TO_TRANSMIT_ROM,
	TRANSMITTING_ROM,
	VERIFYING
}state_e;


typedef enum
{
	START_SEND_METADATA = 0x00,
	START_SEND_ROM,
	END_SEND_METADATA,
	END_SEND_ROM,
	CART_RESET
}client_instruction_e;


typedef enum
{
	METADATA_SEND_FAILED = 0x00,
	ROM_REND_FAILED
}client_status_e;


typedef enum
{
	DMG = 0,
	COLOR,
	GBA
}gb_type_e;


#define ENTRY_POINT_LEN     (uint8_t) 4
#define ROM_TITLE_LEN       (uint8_t) 16
#define MAN_CODE_LEN        (uint8_t) 4
#define METADATA_SIZE_BYTES (uint8_t) 31

//uint8_t nintendo_logo[48] = {
//	0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
//	0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
//	0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E };


/**
  GAMEBOY METADATA STUFF
 */

typedef enum
{
	CART_ENTRY_POINT  = 0x0100,
	CART_LOGO         = 0x0104,
	CART_TITLE        = 0x0134,
	CART_MAUFACTURER  = 0x013F,
	CART_CGB_FLAG     = 0x0143,
	CART_NEW_LIC_CODE = 0x0144,
	CART_SGB_FLAG     = 0x0146,
	CART_TYPE         = 0x0147,
	CART_ROM_SIZE     = 0x0148,
	CART_RAM_SIZE     = 0x0149,
	CART_DEST_CODE    = 0x014A,
	CART_OLD_LIC_CODE = 0x014B,
	CART_MASK_ROM_VER = 0x014C,
	CART_HEADER_CHECK = 0x014D,
	CART_GLOBAL_CHECK = 0x014E,
	CART_HEADER_END   = 0x014F
}cartridge_header_address_e;

// removes offset so enum values can be used for pointer arithmetic
#define HEADER_ADDR(addr) (addr - CART_ENTRY_POINT)


typedef enum
{
	CGB_SUPPORT       = (uint8_t)0x80,
	CGB_ONLY          = 0xC0
}cgb_flag_e;


typedef enum
{
	NONE              = (uint16_t)0x0000,
	NINTENDO_RND      = (uint16_t)0x0001,
	CAPCOM            = (uint16_t)0x0008
	// TODO: more of these
}new_licensee_code_e;


typedef enum
{
	SGB_NO            = (uint8_t)0x00,
	SGB_SUPPORT       = 0x03
}sgb_flag_e;


typedef enum
{
	ROM_ONLY          = (uint8_t)0x00,
	MBC1              = 0x01,
	MBC1_RAM          = 0x02,
	MBC1_RAM_BATT     = 0x03,
	MBC2              = 0x05,
	MBC2_BATT         = 0x06,
	ROM_RAM           = 0x08,
	ROM_RAM_BATT      = 0x09,
	MM01              = 0x0B,
	MM01_RAM          = 0x0C,
	MM01_RAM_BATT     = 0x0D,
	MBC3_TIM_BATT     = 0x0F,
	MBC3_TIM_RAM_BATT = 0x10,
	MBC3              = 0x11,
	MBC3_RAM          = 0x12,
	MBC3_RAM_BATT     = 0x13,
	MBC5              = 0x19,
	MBC5_RAM          = 0x1A,
	MBC5_RAM_BATT     = 0x1B,
	MBC5_RUMBLE       = 0x1C,
	MBC5_RUM_RAM      = 0x1D,
	MBC5_RUM_RAM_BATT = 0x1E,
	MBC6              = 0x20,
	MBC7_SEN_RU_RA_BA = 0x22,   // MBC7+SENSOR+RUMBLE+RAM+BATTERY
	POCKET_CAMERA     = 0xFC,
	BANDAI_TAMA5      = 0xFD,
	HUC3              = 0xFE,
	HUC1_RAM_BATT     = 0xFF
}cart_type_e;


typedef enum
{
	ROM_32KB          = (uint8_t)0x00,  // no rom banks
	ROM_64KB          = 0x01,  // 4 banks
	ROM_128KB         = 0x02,  // 8 banks
	ROM_256KB         = 0x03,  // 16 banks
	ROM_512KB         = 0x04,  // 32 banks
	ROM_1MB           = 0x05,  // 64 banks - only 63 banks used by MBC1
	ROM_2MB           = 0x06,  // 128 banks - only 125 banks used by MBC1
	ROM_4MB           = 0x07,  // 256 banks
	ROM_8MB           = 0x08,  // 512 banks
	ROM_1_1MB         = 0x52,  // 72 banks
	ROM_1_2MB         = 0x53,  // 80 banks
	ROM_1_5MB         = 0x54   // 96 banks
}rom_size_e;


typedef enum
{
	RAM_NONE          = (uint8_t)0x00,
	RAM_2KB           = 0x01,
	RAM_8KB           = 0x02,
	RAM_32KB          = 0x03,  // 4 banks of 8kB
	RAM_128KB         = 0x04,  // 16 banks of 8kB
	RAM_64KB          = 0x05   // 8 banks of 8kB
}ram_size_e;


typedef enum
{
	JAP    = (uint8_t)0x00,
	NO_JAP = 0x01
}dest_code_e;

typedef union
{

}title_u;


typedef struct
{
	uint8_t             entry_point[ENTRY_POINT_LEN];
	char                rom_title[ROM_TITLE_LEN];
//	char                manufacturers_code[MAN_CODE_LEN];
//	cgb_flag_e          cgb_flag;
	new_licensee_code_e new_licensee_code;
	sgb_flag_e          sgb_flag;
	cart_type_e         cart_type;
	rom_size_e          rom_size;
	ram_size_e          ram_size;
	dest_code_e         dest_code;
	uint8_t             old_licensee_code;
	uint8_t             mask_rom_ver_num;
	uint8_t             checksum;
}__attribute__((packed, aligned(1))) gb_metadata_t;


static const uint8_t start_delimiter = 0xBE;
static const uint8_t end_delimiter   = 0xEF;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define USB_TX_BUFF_LEN (uint8_t)1024
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Onboard_LED_Pin GPIO_PIN_13
#define Onboard_LED_GPIO_Port GPIOC
#define button_Pin GPIO_PIN_2
#define button_GPIO_Port GPIOA
#define sr_latch_Pin GPIO_PIN_4
#define sr_latch_GPIO_Port GPIOA
#define sr_clk_Pin GPIO_PIN_5
#define sr_clk_GPIO_Port GPIOA
#define sr_data_Pin GPIO_PIN_6
#define sr_data_GPIO_Port GPIOA
#define data_bus_dir_Pin GPIO_PIN_7
#define data_bus_dir_GPIO_Port GPIOA
#define signal_bus_dir_Pin GPIO_PIN_0
#define signal_bus_dir_GPIO_Port GPIOB
#define addr_bus_dir_Pin GPIO_PIN_1
#define addr_bus_dir_GPIO_Port GPIOB
#define gb_write_Pin GPIO_PIN_15
#define gb_write_GPIO_Port GPIOA
#define gb_read_Pin GPIO_PIN_3
#define gb_read_GPIO_Port GPIOB
#define gb_cs_Pin GPIO_PIN_4
#define gb_cs_GPIO_Port GPIOB
#define gb_rst_Pin GPIO_PIN_5
#define gb_rst_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
