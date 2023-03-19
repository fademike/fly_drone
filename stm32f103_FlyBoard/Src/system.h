

#include <stdint.h>

typedef enum  {
	thread_IMU_loop 			= 0,
	thread_MAV_send_attitude	= 1,
	thread_MAV_send_status		= 2,
	thread_ADC					= 3,
	thread_ModemControl			= 4,
	thread_test					= 5,
}thread_name;							// names of my threads





uint64_t system_getTime_us(void);
uint32_t system_getTime_ms(void);
void system_Delay_us(unsigned int us);
void system_Delay_ms(unsigned  ms);
int Thread_Cycle(void);

int Battery_getVoltage(void);
int Battery_getBatPercent(void);
void Battery_Read(void);

