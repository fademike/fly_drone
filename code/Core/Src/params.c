
#include "stm32f1xx_hal.h"
#include "params.h"
#include "imu.h"
#include <mavlink.h>
#include "main.h"


#define FLASH_ADDRESS_MYDATA (0x8000000+0x400*63)
#define BOOTLOADER_KEY_START_ADDRESS                             (uint32_t)0x08002C00
//#define FLASH_PAGE_SIZE                                          1024

int readFLASH(uint32_t address, int *d);
int writeFLASH(uint32_t address, int * d);

struct param_struct t_param[PARAM_ALL] = {	
	[PARAM_FLASH] = {"flash_params", {.FLOAT=0}, MAV_PARAM_TYPE_REAL32},
	[PARAM_P_KP] = {"p_kp", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_ORIENTATION] = {"p_orientation", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_PID_T_P] = {"pid_t_p", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_T_D] = {"pid_t_d", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_T_I] = {"pid_t_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_T_MUX_CHAN] = {"chan_t_mux", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_T_OFFSET] = {"chan_t_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_PID_P_P] = {"pid_p_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_P_D] = {"pid_p_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_P_I] = {"pid_p_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_P_MUX_CHAN] = {"chan_p_mux", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_P_OFFSET] = {"chan_p_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_PID_R_P] = {"pid_r_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_R_D] = {"pid_r_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_R_I] = {"pid_r_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_R_MUX_CHAN] = {"chan_r_mux", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_R_OFFSET] = {"chan_r_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_PID_Y_P] = {"pid_y_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_D] = {"pid_y_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_I] = {"pid_y_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_MUX_CHAN] = {"chan_y_mux", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_PID_Y_OFFSET] = {"chan_y_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_P_KP_ARM] = {"p_kp_arm", {.FLOAT=0.5f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_P_KI] = {"p_ki", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_M1_ACTION] = {"m1_action", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M1_MIN] = {"m1_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M1_MAX] = {"m1_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M1_OFFSET] = {"m1_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_M2_ACTION] = {"m2_action", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M2_MIN] = {"m2_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M2_MAX] = {"m2_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M2_OFFSET] = {"m2_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_M3_ACTION] = {"m3_action", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M3_MIN] = {"m3_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M3_MAX] = {"m3_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M3_OFFSET] = {"m3_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_M4_ACTION] = {"m4_action", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M4_MIN] = {"m4_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M4_MAX] = {"m4_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_M4_OFFSET] = {"m4_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},

	[PARAM_ALT_MAX] = {"p_alt_max", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
	[PARAM_P_MODE] = {"p_mode", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},
};

void params_save(void){
	int d[FLASH_PAGE_SIZE/4];
	for (int i=0;i<PARAM_ALL;i++) *(float *)&d[i] = t_param[i].param_value.FLOAT;
	imu_AccOffset_get((float *)&d[MEM_CLB_AX], (float *)&d[MEM_CLB_AY], (float *)&d[MEM_CLB_AZ]);
	// Printf("acc save %d, %d, %d\n\r", (int)((*(float*)&d[ADDR_ACC_X])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Y])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Z])*1000.0f));
	writeFLASH(FLASH_ADDRESS_MYDATA, d);
}

void params_restore(void){
	int d[FLASH_PAGE_SIZE/4];
	readFLASH(FLASH_ADDRESS_MYDATA, d);
	if (*(float *)&d[0] == 0) {Printf("params default!\n\r"); return;}// if ndef params
	
	for (int i=0;i<PARAM_ALL;i++) t_param[i].param_value.FLOAT = *(float *)&d[i];
	// Printf("acc read %d, %d, %d\n\r", (int)((*(float*)&d[ADDR_ACC_X])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Y])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Z])*1000.0f));
	imu_AccOffset_set(*(float*)&d[MEM_CLB_AX], *(float*)&d[MEM_CLB_AY], *(float*)&d[MEM_CLB_AZ]);
}

float params_GetParamValue(unsigned int n){
	if (n >= PARAM_ALL) return 0.0f;
	return t_param[n].param_value.FLOAT;
}

int params_getSize(void){
	return PARAM_ALL;
}
int params_setValue(unsigned int n, union param_value value, int type){
	if (n >= PARAM_ALL) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) t_param[n].param_value.FLOAT = value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) t_param[n].param_value.INT = (int)value.INT;
	else t_param[n].param_value.FLOAT = value.FLOAT;

	if (n == 0) params_save();
	return HAL_OK;
}
int params_getValue(unsigned int n, union param_value * value, int type){
	if (n >= PARAM_ALL) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) value->FLOAT = t_param[n].param_value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) value->FLOAT = t_param[n].param_value.INT;
	else  value->FLOAT = t_param[n].param_value.FLOAT;
	return HAL_OK;
}
int params_getParam(unsigned int n, struct param_struct * param){
	if (n >= PARAM_ALL) return HAL_ERROR;
	*param = t_param[n];
	return HAL_OK;
}
int params_getIndexById(char * id){
	int i=-1;// flag of existence param
	for (i=0;i<PARAM_ALL; i++){									// find rx param in param base
		if (strcmp(id, t_param[i].param_id)==0){				// if rx param == param base
			return i;
		}
	}
	return -1;
}

int readFLASH(uint32_t address, int *d){
	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	//if (result != HAL_OK) {return result;}
	for (int i=0;i<(FLASH_PAGE_SIZE/4);i++) d[i] = *(int *)(address+ i*4);
	result = HAL_FLASH_Lock();

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

	for (int i=0;(i<(FLASH_PAGE_SIZE/4) || (result == HAL_OK));i++){
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+i*4, *(int *)&d[i]);
	}
	result = HAL_FLASH_Lock();

	return result;
}

void Clear_Bootloader_Key(void){
	uint32_t buf[FLASH_PAGE_SIZE/4];
	buf[0] = 0;
	writeFLASH(BOOTLOADER_KEY_START_ADDRESS, (int *)buf);
}
