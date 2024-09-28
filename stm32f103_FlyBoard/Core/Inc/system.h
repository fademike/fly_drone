#pragma once
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdint.h>

typedef enum  {
	THREAD_IMU_LOOP 			= 0,
	THREAD_MAV_SEND_ATTITUDE	= 1,
	THREAD_MAV_SEND_STATUS		= 2,
	THREAD_ADC					= 3,
	THREAD_MODEMCONTROL			= 4,
	THREAD_TEST					= 5,
	THREAD_ALL,
} threads;							// names of my threads

typedef enum  {
	THREAD_ENABLE			= 0,
	THREAD_T_INTERVAL		= 1,
	THREAD_T_PRERUN			= 2,
} thread_variables;

void system_changeThread(int name, int param, int value);

uint64_t system_getTime_us(void);
uint32_t system_getTime_ms(void);
void system_Delay_us(unsigned int us);
void system_Delay_ms(unsigned int ms);
void system_reboot(void);
int Thread_Cycle(void);

int Battery_getVoltage(void);
int Battery_getBatPercent(void);
void Battery_Read(void);

#endif
