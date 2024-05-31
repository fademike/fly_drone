#pragma once
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdint.h>

typedef enum  {
	thread_IMU_loop 			= 0,
	thread_MAV_send_attitude	= 1,
	thread_MAV_send_status		= 2,
	thread_ADC					= 3,
	thread_ModemControl			= 4,
	thread_test					= 5,
}thread_name;							// names of my threads

typedef enum  {
	THREAD_NAME 			= 0,
	THREAD_ENABLE			= 1,
	THREAD_T_STARTUP		= 2,
	THREAD_T_INTERVAL		= 3,
	THREAD_T_PRERUN			= 4,
	THREAD_STATUS			= 5,
	THREAD_PRIORITY			= 6,
} thread_variables;

void system_changeThread(int name, int param, int value);

uint64_t system_getTime_us(void);
uint32_t system_getTime_ms(void);
void system_Delay_us(unsigned int us);
void system_Delay_ms(unsigned  ms);
void system_reboot(void);
int Thread_Cycle(void);

int Battery_getVoltage(void);
int Battery_getBatPercent(void);
void Battery_Read(void);

#endif
