
#include "stm32f1xx_hal.h"
#include "params.h"

#include "main.h" // for Printf
#include "imu.h"

//struct param_struct{
//	char name[16];
//};

#include <mavlink.h>


#define FLASH_ADDRESS_MYDATA (0x8000000+0x400*63)
#define BOOTLOADER_KEY_START_ADDRESS                             (uint32_t)0x08002C00
//#define FLASH_PAGE_SIZE                                          1024

int readFLASH(uint32_t address, int *d);
int writeFLASH(uint32_t address, int * d);

#define PARAM_ALL (45)
#define ADDR_ACC_X (PARAM_ALL+0)
#define ADDR_ACC_Y (PARAM_ALL+1)
#define ADDR_ACC_Z (PARAM_ALL+2)

struct param_struct t_param[PARAM_ALL] = {	{"flash_params", {.FLOAT=0}, MAV_PARAM_TYPE_REAL32},	//0		//MAV_PARAM_TYPE_INT8},
											{"f_kp", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},		//1
											{"p_orientation", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//2

											{"pid_t_p", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},		//3
											{"pid_t_d", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//4
											{"pid_t_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//5
											{"mux_t_chan", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},	//6

											{"pid_p_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},		//7
											{"pid_p_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},	//8
											{"pid_p_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//9
											{"mux_p_chan", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},	//10

											{"pid_r_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},		//11
											{"pid_r_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},	//12
											{"pid_r_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//13
											{"mux_r_chan", {.FLOAT=50.0f}, MAV_PARAM_TYPE_REAL32},	//14

											{"pid_y_p", {.FLOAT=10.0f}, MAV_PARAM_TYPE_REAL32},	//15
											{"pid_y_d", {.FLOAT=500.0f}, MAV_PARAM_TYPE_REAL32},	//16
											{"pid_y_i", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},		//17
											{"mux_y_chan", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},	//18

											{"f_kp_arm", {.FLOAT=0.5f}, MAV_PARAM_TYPE_REAL32},		//19
											{"f_ki", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},			//20

//											{"motor1_action", {.FLOAT=1.0f}, MAV_PARAM_TYPE_REAL32},//21
											{"motor1_action", {.FLOAT=149.0f}, MAV_PARAM_TYPE_REAL32},//21
											{"motor1_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//22
											{"motor1_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},//23
											{"motor1_mux", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//24
											{"motor1_init", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//25
											{"motor1_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//26

											{"motor2_action", {.FLOAT=89.0f}, MAV_PARAM_TYPE_REAL32},//27
											{"motor2_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//28
											{"motor2_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},//29
											{"motor2_mux", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//30
											{"motor2_init", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//31
											{"motor2_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//32

											{"motor3_action", {.FLOAT=169.0f}, MAV_PARAM_TYPE_REAL32},//33
											{"motor3_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//34
											{"motor3_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},//35
											{"motor3_mux", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//36
											{"motor3_init", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//37
											{"motor3_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//38

											{"motor4_action", {.FLOAT=101.0f}, MAV_PARAM_TYPE_REAL32},//39
											{"motor4_min", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//40
											{"motor4_max", {.FLOAT=1000.0f}, MAV_PARAM_TYPE_REAL32},//41
											{"motor4_mux", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//42
											{"motor4_init", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},	//43
											{"motor4_offset", {.FLOAT=0.0f}, MAV_PARAM_TYPE_REAL32},//44

										};


void params_save(void){
	int d[FLASH_PAGE_SIZE/4];
	int i=0;
	for (i=0;i<PARAM_ALL;i++) *(float *)&d[i] = t_param[i].param_value.FLOAT;
	imu_AccOffset_get((float *)&d[ADDR_ACC_X], (float *)&d[ADDR_ACC_Y], (float *)&d[ADDR_ACC_Z]);
	Printf("acc save %d, %d, %d\n\r", (int)((*(float*)&d[ADDR_ACC_X])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Y])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Z])*1000.0f));
	writeFLASH(FLASH_ADDRESS_MYDATA, d);
}

void params_restore(void){
	int d[FLASH_PAGE_SIZE/4];
	readFLASH(FLASH_ADDRESS_MYDATA, d);
	int i=0;
	if (*(float *)&d[0] == 0) {Printf("params default!\n\r"); return;}// if ndef params
	else
		for (i=0;i<PARAM_ALL;i++) t_param[i].param_value.FLOAT = *(float *)&d[i];
	Printf("acc read %d, %d, %d\n\r", (int)((*(float*)&d[ADDR_ACC_X])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Y])*1000.0f), (int)((*(float*)&d[ADDR_ACC_Z])*1000.0f));
	imu_AccOffset_set(*(float*)&d[ADDR_ACC_X], *(float*)&d[ADDR_ACC_Y], *(float*)&d[ADDR_ACC_Z]);
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
			//Printf("strcmp %s,  %d \n\r", t_param[i].param_id, i);
			return i+1;
		}
	}
	return 0;
}



int readFLASH(uint32_t address, int *d){

	int result = HAL_OK;

	result = HAL_FLASH_Unlock();
	//if (result != HAL_OK) {return result;}
	Printf("Read HAL_FLASH_Unlock\n\r");
	int i=0;
	for (i=0;i<(FLASH_PAGE_SIZE/4);i++) d[i] = *(int *)(address+ i*4);
	result = HAL_FLASH_Lock();
	Printf("Read HAL_FLASH_Lock\n\r");
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


void Clear_Bootloader_Key(void){
	uint32_t buf[FLASH_PAGE_SIZE/4];
	buf[0] = 0;
	writeFLASH(BOOTLOADER_KEY_START_ADDRESS, (int *)buf);
}
