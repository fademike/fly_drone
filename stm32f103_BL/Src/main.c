/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ModemControl.h"
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
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#pragma pack(push, 1)

typedef struct {
	uint8_t st;
	uint8_t len;
} rf_pack_header;
typedef struct {
	uint8_t status;
	uint16_t current_block;
	uint16_t total_block;
} rf_pack_program_header;
typedef struct {
	uint16_t crc;
} rf_pack_crc;
typedef struct {
	rf_pack_header header;
	rf_pack_program_header p_head;
	rf_pack_crc crc;
	uint8_t end;
} rf_program_answer;
typedef struct {
	rf_pack_header header;
	rf_pack_program_header p_head;
	uint8_t payload[50];
	rf_pack_crc crc;
	uint8_t end;
} rf_program_data;
#pragma pack(pop)


void Printf(const char * pmt);
void Printf_i(int i);
int pack_getLen(uint8_t * pack);
uint16_t pack_returnCRC(uint8_t * pack, int len);
void pack_setCRC(uint8_t * pack);
bool pack_checkCRC(uint8_t * pack);
int flash_data_finish (void);
int flash_data (uint8_t data);
int flash_pack (uint8_t *data, int len);
int read_pack(uint8_t * pack);
void jumpToApplication(uint32_t address);
int readFLASH(uint32_t address, int *d);
int writeFLASH(uint32_t address, int * d);
void Write_Bootloader_Key(uint32_t key);
uint32_t Read_Bootloader_Key(void);

// Bootloader key configuration
#define BOOTLOADER_KEY_START_ADDRESS                             (uint32_t)0x08002C00
#define BOOTLOADER_KEY_VALUE                                     0xAAAA5555
// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS                               (uint32_t)0x08003000
//#define FLASH_PAGE_SIZE                                          1024

int program_state = 0;

uint8_t buf[FLASH_PAGE_SIZE];	// buffer for write flash memory
int addr = 0;			// current addr to write FLASH_PAGE_SIZE to flash
int flash_data_cnt = 0;	// counter for load buf untill size FLASH_PAGE_SIZE

void test_set(int set){
	if (set !=0) HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);
}

void Printf(const char * pmt){
	HAL_UART_Transmit(&huart1, pmt, strlen(pmt), 500);
}
void Printf_i(int i){
	//char num[10] = {0,0,0,0,0,0,0,0,0,0};
	//itoa(i, num, 10);
	//Printf(num);
	if (i>=100) Printf("100");
	else if (i<=0) Printf("0");
	else {
		int d1=i/10;
		int d2=i%10;
		char c[3];
		c[0]=d1+0x30;
		c[1]=d2+0x30;
		c[2] = '\0';
		Printf(c);
	}
}

volatile uint32_t bootloader_timeout = 0;	//used by modemcontrol !

void SYS_myTick(void)
{
	bootloader_timeout++;
}

uint32_t GetTime_ms(void){
	return bootloader_timeout;
}

enum {	// for future, that run app together
	CMD_DATA = 0,
	CMD_REPLY_OK = 1,
	CMD_REPLY_FAIL = 2,
};

int pack_getLen(uint8_t * pack){
	int len = pack[1];
	if (len >= 60) return 0;
	return len;
}
uint16_t pack_returnCRC(uint8_t * pack, int len){
	int i;
	uint16_t crc = 0;
	for (i=0;i<(len+sizeof(rf_pack_header));i++) crc+= pack[i]*87;	// calculate crc
	return crc;
}
// return total pack len
void pack_setCRC(uint8_t * pack){
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	pack_crc->crc = crc;
}
bool pack_checkCRC(uint8_t * pack){
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	if (pack_crc->crc == crc) return true;
	return false;
}

int flash_data_finish (void){ // to load the latest data
	if (flash_data_cnt == 0) return 0;
	writeFLASH(MAIN_PROGRAM_START_ADDRESS + addr, (int *)buf);
	return 0;
}
int flash_data (uint8_t data){
	buf[flash_data_cnt++] = data;
	if (flash_data_cnt >= FLASH_PAGE_SIZE) {
		writeFLASH(MAIN_PROGRAM_START_ADDRESS + addr, (int *)buf); 	// write buf page to flash
		addr += FLASH_PAGE_SIZE;
		flash_data_cnt = 0;
	}
	return 0;
}
int flash_pack (uint8_t *data, int len){
	int i=0;
	for (i=0;i<len;i++) flash_data(data[i]);
	return 0;
}

int read_pack(uint8_t * pack){
	if (!pack_checkCRC(pack)) return -1;

	rf_program_data * data = (rf_program_data *)pack;
	rf_program_answer * answer = (rf_program_data *)pack;

	uint16_t rx_block = data->p_head.current_block;
	uint16_t rx_total = data->p_head.total_block;

	static uint16_t curr_block = 0;

	static bool clear_key = true;
	if ((clear_key) && (rx_block == 0)){
		Write_Bootloader_Key(0);
		clear_key = 0;
	}

	program_state = rx_block;
	if (rx_block == 0) {	// preparing for a new firmware
		addr = 0;			// addr current memory
		flash_data_cnt = 0;	// counter FLASH_PAGE_SIZE rx byte for write to flash
		curr_block = 0;
	}
	if (rx_block == curr_block) {
		Printf("LOAD ");
		Printf_i(rx_block*100/rx_total);
		if (rx_block == 0) Printf("%  \r");
		else Printf("%\r");
		flash_pack(data->payload, data->header.len-sizeof(rf_pack_program_header));
		curr_block++;
	}
	if (curr_block > rx_total) {	// run the app at the end of load firmware
		Printf("LOAD OK  \n\r");
		flash_data_finish();			// load the latest data to flash
		Write_Bootloader_Key(BOOTLOADER_KEY_VALUE);	// set key
		jumpToApplication (MAIN_PROGRAM_START_ADDRESS);		// run new application
	}

	//answer->header.st = '*';
	answer->header.len = sizeof(rf_pack_program_header);
	answer->p_head.status = CMD_REPLY_OK;
	//answer->p_head.current_block = rx_block;
	//answer->p_head.total_block = rx_total;
	answer->end = '#';
	pack_setCRC(pack);
	ModemControl_SendPacket(pack, sizeof(rf_program_answer));
	return 0;
}

int read_raw_byte(uint8_t data){	// parse rx raw bytes
	static int pos = 0;
	static uint8_t len = 0;
	static uint8_t clen = 0;
	static uint8_t rx_pack[64];

	rx_pack[pos + clen] = data;

	if ((pos == 0) && (data == '*')) {pos++;}
	else if (pos == 1) {len = data; if (len >= 60)len = 0; clen = 0; pos++;}
	else if ((pos == 2) && (len > clen)) {clen++;}	// payload
	else if ((pos == 2) && (len == clen)) {pos++;}	//crc1
	else if ((pos == 3)) pos++;						//crc2
	else if ((pos == 4) && (data == '#')) {read_pack(rx_pack); pos=0; clen=0;}
	else pos = 0;

	return 0;
}
void read_raw_buffer(uint8_t * data, int len){	// parse rx buffer
	int i;
	for (i=0;i<len; i++) read_raw_byte(data[i]);
}

void jumpToApplication(uint32_t address) {	// run app
//	return;
	typedef void (*pFunction)(void);
	pFunction Jump_To_Application;
	uint32_t JumpAddress;

	__disable_irq();
	JumpAddress = *(uint32_t*) (address + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	// Initialize user application's Stack Pointer
	__set_MSP(*(__IO uint32_t*) address);
	Jump_To_Application();
}

int readFLASH(uint32_t address, int *d){

	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	if (result != HAL_OK) {return result;}
	//Printf("Read HAL_FLASH_Unlock\n\r");
	int i=0;
	for (i=0;i<(FLASH_PAGE_SIZE/4);i++) d[i] = *(int *)(address+ i*4);
	result = HAL_FLASH_Lock();
	//Printf("Read HAL_FLASH_Lock\n\r");
	if (result != HAL_OK) {return result;}

	return result;
}

int writeFLASH(uint32_t address, int * d){

	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	if (result != HAL_OK) {return result;}

	FLASH_EraseInitTypeDef pEraseInit;
	pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	pEraseInit.PageAddress = address;
	pEraseInit.NbPages = 1;

	uint32_t PageError = 0;

	HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	if (result != HAL_OK) {return result;}

	int i=0;
	for (i=0;i<(FLASH_PAGE_SIZE/4);i++){
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+i*4, *(int *)&d[i]);
		if (result != HAL_OK) { return result;}
	}

	result = HAL_FLASH_Lock();
	if (result != HAL_OK) {return result;}

	return result;
}

void Write_Bootloader_Key(uint32_t key){
	if (key) Printf("SET");
	else Printf("CLEAR");
	Printf(" KEY\n\r");
	*(uint32_t *)buf = key;
	writeFLASH(BOOTLOADER_KEY_START_ADDRESS, (int *)buf);
}
uint32_t Read_Bootloader_Key(void){
	readFLASH(BOOTLOADER_KEY_START_ADDRESS, (int *)buf);
	return *(uint32_t * )buf;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


  HAL_IWDG_Init(&hiwdg);
  __HAL_IWDG_START(&hiwdg);
  HAL_IWDG_Refresh(&hiwdg);

  Printf("\n\rBL\n\r");

  int MC_init = ModemControl_init();
  if (MC_init < 0) {
	  Printf("MC FAIL\n\r");
	  HAL_Delay(5000);	// wait wdg...
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	int ret = 0;
	HAL_IWDG_Refresh(&hiwdg);

	ret = ModemControl_Loop();
	if (ret == 1){	// if rx data pack
		uint8_t buff_pack[64];
		int32_t rx_len = ModemControl_GetPacket(buff_pack);
		if (rx_len > 0) read_raw_buffer(buff_pack, rx_len);
	}

	static uint32_t l_timer = 0;
	if ((bootloader_timeout - l_timer) >= 1000) {	// every 1s try start...
		l_timer = bootloader_timeout;

		const int ps = 0;
		if (program_state != ps) {	// if programming stop, will try start after 2s
			program_state = ps;
			continue;
		}
		uint32_t key = Read_Bootloader_Key();
		if (key == BOOTLOADER_KEY_VALUE) {	// if key set => run app
			  Printf("RUN APP\n\r");
			  jumpToApplication (MAIN_PROGRAM_START_ADDRESS);
		}
	}

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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST3_GPIO_Port, PIN_TEST3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDCARD_SS_Pin|SPI1_NSS_Pin|SI_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin PC14 */
  GPIO_InitStruct.Pin = BTN1_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST2_Pin */
  GPIO_InitStruct.Pin = PIN_TEST2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST3_Pin */
  GPIO_InitStruct.Pin = PIN_TEST3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST_Pin */
  GPIO_InitStruct.Pin = PIN_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCARD_SS_Pin SPI1_NSS_Pin SI_SDN_Pin */
  GPIO_InitStruct.Pin = SDCARD_SS_Pin|SPI1_NSS_Pin|SI_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
