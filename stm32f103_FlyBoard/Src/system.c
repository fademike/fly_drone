
#include "system.h"

#include "stm32f1xx.h"


#define StartUP 2000


extern TIM_HandleTypeDef htim1;


// My threads

struct ThreadFlyStruct {			// This is time settings for some processes
	int name;		//enabled = 1 => on; else off
	int enabled;		//enabled = 1 => on; else off
	int t_startup;		//time to run
	int t_interval;		//interval to run
	int t_previous_run;		//interval to run

	int status;			//status thread // if (status<0) => error; else pointer in process
	int priority;		// 1..10; 10 high 1 - low
} 		ThreadFly[] = {	{thread_IMU_loop,			1, 	StartUP, 		2, 		0,	0, 		1},
						{thread_MAV_send_attitude,	1, 	StartUP, 		100, 	0,	0, 		1},
						{thread_MAV_send_status,	1, 	StartUP, 		1000, 	0,	0, 		1},
						{thread_ADC,				1, 	0, 				10, 	0,	0, 		1},
						{thread_ModemControl,		1, 	StartUP, 		1, 		0,	0, 		1},
						{thread_test,				0, 	StartUP, 		1000, 	0,	0, 		1},
};



uint64_t system_getTime_us(void){
	//uint64_t local_time_us = 0;
	uint64_t local_time_us = HAL_GetTick()*1000;
	int addition_us = __HAL_TIM_GET_COUNTER(&htim1);
	local_time_us += addition_us;
	return local_time_us;
}

uint32_t system_getTime_ms(void){
	return HAL_GetTick();
}

void system_Delay_us(unsigned int us){
	uint64_t stime = system_getTime_us(), t=0;
	do {
		t = system_getTime_us() - stime;
	}while((0 <= t) && (t <  us));
}

void system_Delay_ms(unsigned  ms){
	uint32_t stime = system_getTime_ms(), t=0;
	do {
		t = system_getTime_ms() - stime;
	}while((0 <= t) && (t <  ms));
}

//poll all threads
//execute the stream with the highest priority and exit
int Thread_Cycle(void)
{//Printf("Cycle %d, %d, %d, %d\n\r", ThreadFly[9].priority, ThreadFly[9].enabled, ThreadFly[9].t_counter, ThreadFly[9].status);HAL_Delay(1000); return;
	int thread_all=sizeof(ThreadFly)/sizeof(ThreadFly[0]);
	int priority=10;
	static char priority_found = 0;
	static char priorityVar[11] = {0,};	// stories priority options
	static char needComplete_priority = -1;
	static char needComplete_thread = -1;

	//Look at all the priorities
	if (priority_found == 0){
		for (priority=10;priority>0; priority--){	// looking for what priority options exist.
			int cnt = 0;
			priorityVar[priority]=0; 	// priority does not exist
			for (cnt=0; cnt<thread_all; cnt++){
				if (ThreadFly[cnt].priority == priority)
				{
					priorityVar[priority]=1; // priority exist
					continue;
				}
			}
		}
		priority_found = 1;
	}

	uint32_t c_time = system_getTime_ms();
	//Look at all exist the priorities
//	if (needComplete_priority > 0)priority = needComplete_priority;
//	else
		priority=10;
	for (;priority>0; priority--){
		if (priorityVar[priority] != 1) continue;	// priority does not exist
		int cnt = 0;
//		if (needComplete_thread >= 0) cnt = needComplete_thread;
		//int fl_runned = 0;
		for (; cnt<thread_all; cnt++){
			if ((ThreadFly[cnt].priority == priority) && (ThreadFly[cnt].enabled == 1) &&
						(ThreadFly[cnt].t_startup < c_time) &&
						((c_time - ThreadFly[cnt].t_previous_run) > ThreadFly[cnt].t_interval) &&
						(ThreadFly[cnt].status >= 0)){
				ThreadFly[cnt].t_previous_run = c_time;
				//RunThread(&ThreadFly[cnt]);	// Run thread
				//fl_runned = 1;
//				needComplete_priority = priority;
//				needComplete_thread = cnt++;
				return ThreadFly[cnt].name;
			}
		}
		//if (fl_runned == 0)return;
	}

	needComplete_priority = -1;
	needComplete_thread = -1;
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


void Battery_Read(void){
	//static float f_voltage = 0;
	const int undervoltage = 3000;
	const float multiplier = 12.2;//11.0;	//res divider

	int adc = Read_ADC_Channel(0);
	int calc_mV=((adc*3000)/0xFFF) * multiplier;
	//if (voltage == 0) voltage = calc_mV;
	//else
		voltage += (calc_mV - voltage)*(0.5f);
	batPercent = (voltage-undervoltage) * 100 / (4200-undervoltage);
	if (batPercent > 100) batPercent = 100;
}



