
#include "system.h"

#include "stm32f1xx.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

// My threads
struct ThreadFlyStruct {			// This is time settings for some processes
	int enabled;		// enabled = 1 => on; else off
	int t_interval;		// interval to run
	int t_previous_run;	// previous launch time
} 		ThreadFly[THREAD_ALL] = {	
	[THREAD_IMU_LOOP] = 			{1, 		2, 		0},
	[THREAD_MAV_SEND_ATTITUDE] = 	{1, 		100, 	0},
	[THREAD_MAV_SEND_STATUS] = 		{1, 		1000, 	0},
	[THREAD_ADC] = 					{1, 		10, 	0},	// to read battery voltage
	[THREAD_MODEMCONTROL] = 		{1, 		1, 		0},	// ModemControl loop
	[THREAD_TEST] = 				{1, 		500, 	200},
};


void system_changeThread(int name, int param, int value){
	int * ptrToStruct = (int *)&ThreadFly[name];
	ptrToStruct[param] = value;
}

uint64_t system_getTime_us(void){
	uint64_t ms;
	uint64_t local_time_us = 0;
	// return system_getTime_ms()*1000;
	do{
		ms = HAL_GetTick();
		local_time_us = ms*1000;
		local_time_us += (uint32_t) __HAL_TIM_GET_COUNTER(&htim1);
	} while (ms != HAL_GetTick());
	return local_time_us;
}

uint32_t system_getTime_ms(void){
	return HAL_GetTick();
}

void system_Delay_us(unsigned int us){
	uint64_t stime = system_getTime_us(), t=0;
	do {
		t = system_getTime_us() - stime;
	}while(t <  us);
}

void system_Delay_ms(unsigned int  ms){
	uint32_t stime = system_getTime_ms(), t=0;
	do {
		t = system_getTime_ms() - stime;
	}while(t <  ms);
}

void system_reboot(void){
	NVIC_SystemReset();
}

int Thread_Cycle(void)
{
	uint64_t c_time = (uint64_t)system_getTime_ms();
	static uint64_t t_launch = 0;	// launch time

	if (t_launch == 0) t_launch = c_time;	// if it's the first launch, then set the launch time
	c_time -= t_launch;

	static int i=0;
	for (; i<THREAD_ALL; i++){
		if ((ThreadFly[i].enabled == 1) &&
					((c_time - ThreadFly[i].t_previous_run) > ThreadFly[i].t_interval)){
			ThreadFly[i].t_previous_run = c_time;
			return i;
		}
	}
	i = 0;
	return -1;
}


extern ADC_HandleTypeDef hadc1;
/* ADC init function */
static void ADC_Change_Channel(int Channel)
{
	  ADC_ChannelConfTypeDef sConfig;

	    /**Common config
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

	  sConfig.Channel = Channel;//ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
		    Error_Handler();
	  }

}



int Read_ADC_Channel(int channel)
{
	//static int ch = -1;
	//if (ch != channel)
	int adcResult=0, i=0,Count = 10;	// if Count = 1 => Time read = 50us; if Count = 10 => Time read = 63.6us
	  HAL_ADC_DeInit(&hadc1);
	  ADC_Change_Channel(channel);


	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  for (i=0; i<Count; i++)
		  {
		  adcResult += HAL_ADC_GetValue(&hadc1);
		  }
	  HAL_ADC_Stop(&hadc1);
	  return (adcResult/Count);
}

static int voltage = 0;
static int batPercent = 0;


int Battery_getVoltage(void){
	return voltage;
}
int Battery_getBatPercent(void){
	return batPercent;
}

#define UNDERVOLTAGE 3000

void Battery_Read(void){
	//static float f_voltage = 0;
	const float multiplier = 12.2;//11.0;	//res divider

	int adc = Read_ADC_Channel(0);
	int calc_mV=((adc*3000)/0xFFF) * multiplier;
	//if (voltage == 0) voltage = calc_mV;
	//else
		voltage += (calc_mV - voltage)*(0.2f);	//LPF
	unsigned int batPercent_tmp;
	batPercent_tmp = (voltage-UNDERVOLTAGE) * 100 / (4200-UNDERVOLTAGE);
	if (batPercent_tmp > 100) batPercent_tmp = 100;
	batPercent = batPercent_tmp;
}



