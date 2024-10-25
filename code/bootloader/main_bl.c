/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>		// for strlen
#include <stdio.h>		// for sprintf
#include <math.h>		  // for abs and other

#include "radio_config_Si4463.h"

// #include "debug.h"

#include "params.h"
#include "system.h"
#include "mavlink_handler.h"

#include "imu.h"

#include "ModemControl.h"
#include "MotorControl.h"

#include "task.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
// osThreadId_t defaultTaskHandle;
// const osThreadAttr_t defaultTask_attributes = {
//   .name = "defaultTask",
//   .stack_size = 512 * 4,
//   .priority = (osPriority_t) osPriorityNormal,
// };
// /* Definitions for myTask02 */
// osThreadId_t myTask02Handle;
// const osThreadAttr_t myTask02_attributes = {
//   .name = "myTask02",
//   .stack_size = 256 * 4,
//   .priority = (osPriority_t) osPriorityHigh,
// };
// /* Definitions for myTask03 */
// osThreadId_t myTask03Handle;
// const osThreadAttr_t myTask03_attributes = {
//   .name = "myTask03",
//   .stack_size = 256 * 4,
//   .priority = (osPriority_t) osPriorityLow,
// };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

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


void Printf(const char *fmt, ...);
void Printf_i(int i);
int pack_getLen(uint8_t * pack);
uint16_t pack_returnCRC(uint8_t * pack, int len);
void pack_setCRC(uint8_t * pack);
char pack_checkCRC(uint8_t * pack);
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

void Printf(const char *fmt, ...){
	HAL_UART_Transmit(&huart1, fmt, strlen(fmt), 500);
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
char pack_checkCRC(uint8_t * pack){
	int len = pack_getLen(pack);
	uint16_t crc = pack_returnCRC(pack, len);
	rf_pack_crc * pack_crc = (rf_pack_crc *)&pack[sizeof(rf_pack_header) + len];
	if (pack_crc->crc == crc) return 1;
	return 0;
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

	static char clear_key = 1;
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

//HAL_DeInit();
  RCC->APB1RSTR = 0xFFFFFFFFU;
  RCC->APB1RSTR = 0x00;
  RCC->APB2RSTR = 0xFFFFFFFFU;
  RCC->APB2RSTR = 0x00;
 
//SysTick DeInit
  SysTick->CTRL=0;
  SysTick->VAL=0;
  SysTick->LOAD=0;
 
  __disable_irq();
 
  //NVIC DeInit
  __set_BASEPRI(0);
  __set_CONTROL(0);
  NVIC->ICER[0]=0xFFFFFFFF;
  NVIC->ICPR[0]=0xFFFFFFFF;
  NVIC->ICER[1]=0xFFFFFFFF;
  NVIC->ICPR[1]=0xFFFFFFFF;
 
  __enable_irq();
 
  // /* Change the main and local  stack pointer. */
  // __set_MSP(*(volatile uint32_t*)address);
  //   SCB->VTOR=*(volatile uint32_t*)address;


	// __disable_irq();
	JumpAddress = *(uint32_t*) (address + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	// Initialize user application's Stack Pointer
	__set_MSP(*(__IO uint32_t*) address);
  SCB->VTOR=*(volatile uint32_t*)address;
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

///////////////////////

// void test_set(int set){
//   if (set !=0) HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);
//   else HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);

// }

// volatile uint32_t tick_ms = 0;

// uint32_t Get_tick(void){
//   return tick_ms;
// }

// void HAL_IncTick(void)
// {
//   uwTick += uwTickFreq;
//   tick_ms++;
//   if (htim1.Instance != 0)__HAL_TIM_SET_COUNTER(&htim1, 0); // reset for timer counter us
// }

// void SYS_myTick(void)  // IRQ 1 ms
// {
//   tick_ms++;
//   // if ((tick_ms%1000) == 0) Printf("round\n\r");
//   //reset timer for calculate us
//   if (htim1.Instance != 0)__HAL_TIM_SET_COUNTER(&htim1, 0); // reset for timer counter us
// }

// void Printf(const char *fmt, ...){
//  HAL_UART_Transmit(&huart1, (unsigned char *)fmt, strlen(fmt), 500);
// }
// #include <stdarg.h>
// void Printf(const char *fmt, ...)
// {
//   char buf[256];
//   va_list lst;
//   va_start(lst, fmt);
//   vsprintf(buf, fmt, lst);
//   va_end(lst);
//   HAL_UART_Transmit(&huart1, (unsigned char *)buf, strlen(buf), 500);
//   // if (ModemControl_getStatus()>=0) mavlink_send_statustext(buf);
// }

// for printf. the size is larger on 2k
// int _write(int fd, char* ptr, int len) {
//     HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
//   ptr[len] = '\0';
//   if (ModemControl_getStatus()>=0) mavlink_send_statustext(ptr);
//     return len;
// }

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
  // MX_ADC1_Init();
  // MX_I2C2_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  // MX_TIM2_Init();
  // MX_TIM3_Init();
  MX_USART1_UART_Init();
  // MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

//   Printf("...\n\r");

//   __enable_irq();
//   HAL_IWDG_Init(&hiwdg);
//   __HAL_IWDG_START(&hiwdg);
//   HAL_IWDG_Refresh(&hiwdg);

//   HAL_TIM_Base_Start(&htim1); // timer for us calculate
//   HAL_TIM_Base_Start(&htim2); // reserv for additional motors
//   HAL_TIM_Base_Start(&htim3); // timer for motor pwm

//   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // for additional motors
//   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // for additional motors

//   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // motor 1
//   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // motor 2
//   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // motor 3
//   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // motor 4

// //  Printf("zz");
// //  HAL_Delay(10);
// //  Printf("z\n\r");

//   // params_restore();

//   HAL_IWDG_Refresh(&hiwdg);

//   MotorControl_init();

//   if (imu_init() < 0) system_reboot();
//   if (ModemControl_init() < 0) {Printf("MC_init false\n\r"); system_reboot();};

//   Printf("Started...BL..\n\r");

//   /* USER CODE END 2 */

//   /* Init scheduler */
//   osKernelInitialize();

//   /* USER CODE BEGIN RTOS_MUTEX */
//   /* add mutexes, ... */
//   /* USER CODE END RTOS_MUTEX */

//   /* USER CODE BEGIN RTOS_SEMAPHORES */
//   /* add semaphores, ... */
//   /* USER CODE END RTOS_SEMAPHORES */

//   /* USER CODE BEGIN RTOS_TIMERS */
//   /* start timers, add new ones, ... */
//   /* USER CODE END RTOS_TIMERS */

//   /* USER CODE BEGIN RTOS_QUEUES */
//   /* add queues, ... */
//   /* USER CODE END RTOS_QUEUES */

//   /* Create the thread(s) */
//   /* creation of defaultTask */
//   defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

//   /* creation of myTask02 */
//   myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

//   /* creation of myTask03 */
//   myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

//   /* USER CODE BEGIN RTOS_THREADS */
//   /* add threads, ... */
//   /* USER CODE END RTOS_THREADS */

//   /* USER CODE BEGIN RTOS_EVENTS */
//   /* add events, ... */
//   /* USER CODE END RTOS_EVENTS */

//   /* Start scheduler */
//   osKernelStart();

//   /* We should never get here as control is now taken by the scheduler */



  HAL_IWDG_Init(&hiwdg);
  __HAL_IWDG_START(&hiwdg);
  HAL_IWDG_Refresh(&hiwdg);

  Printf("\n\rBL\n\r");

  int MC_init = ModemControl_init();
  if (MC_init < 0) {
	  Printf("MC FAIL\n\r");
	  HAL_Delay(5000);	// wait wdg...
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    int ret = 0;
    HAL_IWDG_Refresh(&hiwdg);

    static int t = 0;

    if ((t+1000)<(bootloader_timeout)){
      t=bootloader_timeout;
      // Printf("tick\n\r");

    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST2_GPIO_Port, PIN_TEST2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_NSS_Pin|SI_SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_TEST2_Pin */
  GPIO_InitStruct.Pin = PIN_TEST2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_TEST_Pin */
  GPIO_InitStruct.Pin = PIN_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_NSS_Pin SI_SDN_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin|SI_SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

// /* USER CODE BEGIN 4 */
// int timer_tick = 0;
// /* USER CODE END 4 */

// /* USER CODE BEGIN Header_StartDefaultTask */
// /**
//   * @brief  Function implementing the defaultTask thread.
//   * @param  argument: Not used
//   * @retval None
//   */
// /* USER CODE END Header_StartDefaultTask */
// void StartDefaultTask(void *argument)
// {
//   /* USER CODE BEGIN 5 */


//   // portTickType xLastWakeTime;
//   // xLastWakeTime = xTaskGetTickCount();

//   /* Infinite loop */
//   for(;;)
//   {
//     // HAL_IWDG_Refresh(&hiwdg);

//     // vTaskDelayUntil( &xLastWakeTime, ( 1 / portTICK_RATE_MS ) );
    
//     // vTaskDelay( ( 1 / portTICK_RATE_MS ) );

//     // Printf("mt");

//     imu_loop();
//     // Printf("1");

//     mavlink_loop();
//     // Printf("2");
//     if (ModemControl_Loop() == 1){  // if rx data pack
//       uint8_t buff_pack[64];
//       int32_t rx_len = ModemControl_GetPacket(buff_pack);
//       mavlink_receive_pack(buff_pack, rx_len);  // send packet to parse
//     }
//     // Printf("3. ");
//     osDelay(1);

//     static int d=0;
//     if (d++>100){
//       d=0;
//       mavlink_send_attitude();
//       mavlink_send_heartbeat();
//       mavlink_send_status();
//       HAL_GPIO_TogglePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin);
//       Battery_Read();
//       // Printf("Task. %d.\n\r", timer_tick);
//     }
    
//     HAL_IWDG_Refresh(&hiwdg);

//     taskYIELD();

//   }
//   /* USER CODE END 5 */
// }

// /* USER CODE BEGIN Header_StartTask02 */
// /**
// * @brief Function implementing the myTask02 thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_StartTask02 */
// void StartTask02(void *argument)
// {
//   /* USER CODE BEGIN StartTask02 */
//   /* Infinite loop */

//   // portTickType xLastWakeTime;
//   // xLastWakeTime = xTaskGetTickCount();

//   for(;;)
//   {

//     // vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_RATE_MS ) );
    
//     // vTaskDelay(200);

//     static int i=0;
//     Printf("Task. %d.\n\r", i++);

//     timer_tick++;
    

//     // int cnt_rd = cnt;
//     // Printf("Task. %d. %d\n\r", i++, cnt);
//     // cnt = 0;
//     // cnt_rst = 1;

//     // HAL_GPIO_TogglePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin);
//     // mavlink_send_attitude();
//     // mavlink_send_heartbeat();
//     // mavlink_send_status();
//     // Battery_Read();

//     osDelay(100);
//     // taskYIELD();

//   }
//   /* USER CODE END StartTask02 */
// }

// /* USER CODE BEGIN Header_StartTask03 */
// /**
// * @brief Function implementing the myTask03 thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_StartTask03 */
// void StartTask03(void *argument)
// {
//   /* USER CODE BEGIN StartTask03 */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END StartTask03 */
// }

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
    bootloader_timeout++;
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
