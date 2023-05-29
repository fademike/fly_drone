
#include "stm32f1xx_hal.h"
#include "params.h"

#include "main.h" // for Printf
#include "imu.h"

//struct param_struct{
//	char name[16];
//};

#include <mavlink.h>

int readFLASH(int *d);
int writeFLASH(int * d);

#define PARAM_ALL 12

struct param_struct t_param[PARAM_ALL] = {	{"flash_params", {.FLOAT=0}, MAV_PARAM_TYPE_REAL32},	//20		//MAV_PARAM_TYPE_INT8},
											{"f_kp", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},		//10
											{"f_ki", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},			//0
											{"pid_pr_p", {.FLOAT=60.0f}, MAV_PARAM_TYPE_REAL32},		//60
											{"pid_pr_d", {.FLOAT=2000.0f}, MAV_PARAM_TYPE_REAL32},		//2000
											{"pid_pr_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//0
											{"mux_pr_chan", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},	//50
											{"mux_y_chan", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},	//1
											{"pid_y_p", {.FLOAT=-60.0f}, MAV_PARAM_TYPE_REAL32},		//-60
											{"pid_y_d", {.FLOAT=-4000.0f}, MAV_PARAM_TYPE_REAL32},		//-4000
											{"pid_y_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//0
											{"f_kp_arm", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32} };	//0


void params_save(void){
	int d[24];
	int i=0;
	for (i=0;i<PARAM_ALL;i++) *(float *)&d[i] = t_param[i].param_value.FLOAT;
	imu_AccOffset_get((float *)&d[21], (float *)&d[22], (float *)&d[23]);
	Printf("acc save %d, %d, %d\n\r", (int)((*(float*)&d[21])*1000.0f), (int)((*(float*)&d[22])*1000.0f), (int)((*(float*)&d[23])*1000.0f));
	writeFLASH(d);
}

void params_restore(void){
	int d[24];
	readFLASH(d);
	int i=0;
	if (*(float *)&d[0] != 0) // if ndef params
		for (i=0;i<PARAM_ALL;i++) t_param[i].param_value.FLOAT = *(float *)&d[i];
	Printf("acc read %d, %d, %d\n\r", (int)((*(float*)&d[21])*1000.0f), (int)((*(float*)&d[22])*1000.0f), (int)((*(float*)&d[23])*1000.0f));
	imu_AccOffset_set(*(float*)&d[21], *(float*)&d[22], *(float*)&d[23]);
}


float params_GetMemValue(int n){
	//if ((1 <= n) && (n < PARAM_ALL))
		return t_param[n].param_value.FLOAT;
	//return 0.0f;
}


int params_getSize(void){
	return PARAM_ALL;
}
int params_setValue(int n, union param_value value, int type){
	int t = n-1;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) t_param[t].param_value.FLOAT = value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) t_param[t].param_value.INT = (int)value.INT;
	else t_param[t].param_value.FLOAT = value.FLOAT;

	//Printf("param set %d, %d\n\r", n, (int) value.FLOAT);
	if (n == 1) params_save();
	return HAL_OK;
}
int params_getValue(int n, union param_value * value, int type){
	int t = n-1;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	if (type == MAV_PARAM_TYPE_REAL32) value->FLOAT = t_param[t].param_value.FLOAT;
	else if (type == MAV_PARAM_TYPE_INT8) value->FLOAT = t_param[t].param_value.INT;
	else  value->FLOAT = t_param[t].param_value.FLOAT;
	return HAL_OK;
}
int params_getParam(int n, struct param_struct * param){
	int t = n-1;
	if ((t<0) || (t>=PARAM_ALL)) return HAL_ERROR;
	*param = t_param[t];
	return HAL_OK;
}
int params_getIndexById(char * id){
	int i=0;// flag of existence param
	for (i=0;i<PARAM_ALL; i++){									// find rx param in param base
		if (strcmp(id, t_param[i].param_id)==0){				// if rx param == param base
			Printf("strcmp %s,  %d \n\r", t_param[i].param_id, i);
			return i+1;
		}
	}
	return 0;
}


#define FLASH_ADDRESS_MYDATA (0x8000000+0x400*63)

int readFLASH(int *d){

	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	//if (result != HAL_OK) {return result;}
	Printf("Read HAL_FLASH_Unlock\n\r");
	int i=0;
	for (i=0;i<24;i++) d[i] = *(int *)(FLASH_ADDRESS_MYDATA+ i*4);
	result = HAL_FLASH_Lock();
	Printf("Read HAL_FLASH_Lock\n\r");
	if (result != HAL_OK) {return result;}

	return result;
}

int writeFLASH(int * d){

	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	if (result != HAL_OK) {return result;}

	FLASH_EraseInitTypeDef pEraseInit;
	pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	pEraseInit.PageAddress = FLASH_ADDRESS_MYDATA;
	pEraseInit.NbPages = 1;

	uint32_t PageError = 0;

	HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	if (result != HAL_OK) {return result;}

	int i=0;

	for (i=0;i<24;i++){
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDRESS_MYDATA+i*4, *(int *)&d[i]);
		if (result != HAL_OK) { return result;}
	}

	result = HAL_FLASH_Lock();
	if (result != HAL_OK) {return result;}

	return result;
}

