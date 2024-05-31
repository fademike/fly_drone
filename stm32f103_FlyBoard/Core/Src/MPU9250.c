/*
 * MPU9250.c
 *
 *  Created on: 08 2018
 *      Author: fademike
 */

#include "MPU9250.h"
#include "main.h"
#include "stm32f1xx_hal.h"


//#define SPI
#define I2C

#ifdef SPI
extern SPI_HandleTypeDef hspi1;
#endif
#ifdef I2C
#define hi2cN hi2c2
extern I2C_HandleTypeDef hi2cN;

// if i2c interface not disabled, then read/write uSD card by SPI will be not correct
#define EN_I2C {}//if(CONFLICT)SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) 		//disable I2c
#define DIS_I2C {}//if(CONFLICT)CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN) 	//disable I2c
#endif


#define DEVID (0x68<<1)
#define MAGID (0x0C<<1)
#define BMPID (0x76<<1)

#define WHO_AM_I 0x75
#define READWRITE_CMD 0x80

static int MPU_Read_Regs(uint8_t cmd, uint8_t * buf, uint8_t num){
#ifdef SPI
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	uint8_t data = cmd | READWRITE_CMD;
	if (HAL_SPI_Transmit(&hspi1, &data, 1, 100) != HAL_OK) return -1;
	if (HAL_SPI_Receive(&hspi1, buf, num, 100) != HAL_OK) return -1;

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return HAL_OK;
#endif
#ifdef I2C
	EN_I2C;
	int res = HAL_I2C_Mem_Read(&hi2cN, (DEVID+1), cmd, 1, buf, num, 50);
	DIS_I2C;
	if (res != HAL_OK)return -1;
	return HAL_OK;
#endif
}
static int MPU_Write_Regs(uint8_t cmd, uint8_t * buf, uint8_t num){
#ifdef SPI
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	uint8_t data = cmd;
	if (HAL_SPI_Transmit(&hspi1, &data, 1, 100) != HAL_OK) return -1;
	if (HAL_SPI_Transmit(&hspi1, buf, num, 100) != HAL_OK) return -1;
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return HAL_OK;
#endif
#ifdef I2C

	EN_I2C;
	int res = HAL_I2C_Mem_Write(&hi2cN, (DEVID+1), cmd, 1, buf, num, 50);
	DIS_I2C;
	if (res != HAL_OK)return -1;
	return HAL_OK;
#endif
}

uint8_t mpu_regs[4] = {-1, -1, -1, -1};

int MPU6050_Init(void);
int MPU9250_Init(void);

int MPU_Init(void)
{
#if MPU_CHIP == MPU9250
	return MPU9250_Init();
#endif
#if MPU_CHIP == MPU6050
	return MPU6050_Init();
#endif
}

int MPU6050_Init(void)
{
	uint8_t buf[1];

	uint8_t mpu_reg_1A;
	uint8_t mpu_reg_1B;
	uint8_t mpu_reg_1C;
//	uint8_t mpu_reg_1D;
	if (MPU_Read_Regs(WHO_AM_I, buf, 1) != HAL_OK) return -1;

	if (buf[0] != 0x68) {Printf("Iam is 0x%x\n\r", buf[0]); return -2;}
	//Printf("Iam is 0x%x\n\r", buf[0]);


	uint8_t mpu_reg_6B = 0;
	if (MPU_Write_Regs(0x6B, &mpu_reg_6B, 1) != HAL_OK) return -3;	// clk setup

	mpu_reg_1A = 0x0<<0; // DLPF_CFG : 0 - 256Hz, 1 - 188Hz, 2 - 98Hz, 3 - 42Hz, 4 - 20Hz, 5 - 10Hz, 6 - 5Hz

	mpu_reg_1B = 0 | (0x03<<3);	//GYRO_FS_SEL[1:0] -  (0- +-250dps, 1-500dps, 2-1000, 3-2000dps)

	mpu_reg_1C = 0x03<<3;	//ACCEL_FS_SEL[1:0]	// 2g (00), 4g (01), 8g (10), 16g (11)


	mpu_regs[0] = mpu_reg_1A;
	mpu_regs[1] = mpu_reg_1B;
	mpu_regs[2] = mpu_reg_1C;

	if (MPU_Write_Regs(0x1A, mpu_regs, 3) != HAL_OK) return -3;

	if (MPU_Read_Regs(0x1A, mpu_regs, 3) != HAL_OK) return -3;

	if (mpu_regs[0] != mpu_reg_1A) return -4;
	if (mpu_regs[1] != mpu_reg_1B) return -4;
	if (mpu_regs[2] != mpu_reg_1C) return -4;


	Printf("MPU6050_init ok\n\r");

	return 0;
}
int MPU9250_Init(void)
{
	uint8_t buf[1];


//	//Set magnit
//	buf[0] = 0x00;
//	if (MPU_Write_Regs(0x6B, buf, 1) != HAL_OK) return -1;
//	if (MPU_Write_Regs(0x6A, buf, 1) != HAL_OK) return -1;
//	buf[0] = 0x02;
//	if (MPU_Write_Regs(0x37, buf, 1) != HAL_OK) return -1;

	uint8_t mpu_reg_1A;
	uint8_t mpu_reg_1B;
	uint8_t mpu_reg_1C;
	uint8_t mpu_reg_1D;


	mpu_reg_1A = 0x0<<0; // if (FCHOICE==3) 0 - 250Hz, 1 - 184Hz, 2 - 92Hz, 3 - 41Hz, 4 - 20Hz, 5 - 10Hz, 6 - 5Hz, 7 - 3600Hz
	//if (MPU_Write_Regs(0x1A, &mpu_reg_1A, 1) != HAL_OK) return -1;	//DLPF_CFG

	mpu_reg_1B = 0
				  | (0x03<<3)	//GYRO_FS_SEL[1:0] -  (0- +-250dps, 1-500dps, 2-1000, 3-2000dps)
				  | (0<<0);	//Fchoice_b[1:0]	// =2b00 - Using DLPF
	//if (MPU_Write_Regs(0x1B, &mpu_reg_1B, 1) != HAL_OK) return -1;	//GYRO_FS_SEL

	mpu_reg_1C = 0x03<<3;	//ACCEL_FS_SEL[1:0]	// 2g (00), 4g (01), 8g (10), 16g (11)
	//if (MPU_Write_Regs(0x1C, &mpu_reg_1C, 1) != HAL_OK) return -1;	//ACCEL_FS_SEL

	mpu_reg_1D = 0 //ACCEL_CONFIG 2 -
				   | 0x00<<3 	//(ACCEL_FCHOICE_B<<2) if (ACCEL_FCHOICE_B == 0) => enable LPF
				   | 0x00<<0;	// (A_DLPF_CFG<<0)	(X-1130;0-460;1-184;2-92;3-41;4-20;5-10;6-5;7-460Hz)
	//if (MPU_Write_Regs(0x1D, &mpu_reg_1D, 1) != HAL_OK) return -1; //ACCEL_FS_SEL

	mpu_regs[0] = mpu_reg_1A;
	mpu_regs[1] = mpu_reg_1B;
	mpu_regs[2] = mpu_reg_1C;
	mpu_regs[3] = mpu_reg_1D;

	if (MPU_Read_Regs(WHO_AM_I, buf, 1) != HAL_OK) return -1;

	if ((buf[0] != 0x71) && (buf[0] != 0x73)) {Printf("I am is 0x%x\n\r",  buf[0]); return -2;}


	if (MPU_Write_Regs(0x1A, mpu_regs, 4) != HAL_OK) return -3;

//	mpu_regs[0] = mpu_reg_1A;
//	mpu_regs[1] = mpu_reg_1B;
//	mpu_regs[2] = mpu_reg_1C;
//	mpu_regs[3] = mpu_reg_1D;

	Printf("MPU_init ok\n\r");
	//aTxBuffer[0] = 100;//0x05<<0;
	//if (HAL_I2C_Mem_Write(&hi2c1, (DEVID+0), 0x19, 1, aTxBuffer, 1, 1000) != HAL_OK)return -1;	//Sample Rate Divider

	return 0;
}

int MPU_check(void){

	unsigned char r[14];
	if (MPU_Read_Regs(0x1A, r, 4) != HAL_OK) return -1;

	if ((r[0] == mpu_regs[0]) &&
		(r[1] == mpu_regs[1]) &&
		(r[2] == mpu_regs[2]) &&
		(r[3] == mpu_regs[3]) ){return 0;}
	return -2;

}

int MPU_GetData(pos_struct * Est_A, pos_struct * Est_G, short * t){
//	if (Get_acc(EstA) != HAL_OK) return -1;
//	if (Get_gyro(EstG) != HAL_OK) return -1;

	unsigned char r[14];
	if (MPU_Read_Regs(ACCEL_XOUT_H, r, 14) != HAL_OK) return -1;

	Est_A->x = (r[0]<<8) | r[1];
	Est_A->y = (r[2]<<8) | r[3];
	Est_A->z = (r[4]<<8) | r[5];

	Est_G->x = (r[8]<<8) | r[9];
	Est_G->y = (r[10]<<8) | r[11];
	Est_G->z = (r[12]<<8) | r[13];

	*t= (r[6]<<8) | r[7];

	return 0;
}



int MPU_Get_acc(pos_struct * Est_A)
{
	unsigned char r[6];

	if (MPU_Read_Regs(ACCEL_XOUT_H, r, 6) != HAL_OK) return -1;

	Est_A->x = (r[0]<<8) | r[1];
	Est_A->y = (r[2]<<8) | r[3];
	Est_A->z = (r[4]<<8) | r[5];

	return 0;
}


int MPU_Get_gyro(pos_struct * Est_G)
{
	unsigned char r[6];

	if (MPU_Read_Regs(GYRO_XOUT_H, r, 6) != HAL_OK) return -1;

	Est_G->x = (r[0]<<8) | r[1];
	Est_G->y = (r[2]<<8) | r[3];
	Est_G->z = (r[4]<<8) | r[5];

	return 0;
}

int  MPU_Get_temp(short temp)
{
	unsigned char d[2];

	if (MPU_Read_Regs(TEMP_OUT_H, &d[1], 6) != HAL_OK) return -1;
	if (MPU_Read_Regs(TEMP_OUT_L, &d[0], 6) != HAL_OK) return -1;

	short tem = *(short *)&d[0];

	temp = tem;

	return 0;
}



