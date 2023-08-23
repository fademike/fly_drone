



#include "stm32f1xx.h"
#include "main.h"
#include "MotorControl.h"
#include "mavlink_handler.h"
#include "params.h"
#include "imu.h"

//extern TIM_HandleTypeDef htim3;

void UpdateFromParam(void);




typedef struct
{
	unsigned char Reset;
	float lastResult;
	float lastInput;
	float I;
	float P;
	float D;
	float MaxLimit;
} t_pid;

t_pid mPID[4] = {{1,0,0,0,0,0,1000.0f},
				{1,0,0,0,0,0,1000.0f},
				{1,0,0,0,0,0,1000.0f},
				{1,0,0,0,0,0,1000.0f}};

unsigned int motor_action[4] = {0,0,0,0};
float motor_min[4] = {0,0,0,0};
float motor_max[4] = {1000,1000,1000,1000};
//float motor_mux[4] = {1,1,1,1};
float motor_offset[4] = {0,0,0,0};
unsigned short motor_init[4] = {1000,1000,1000,1000};

float SetPID(volatile t_pid *PID, float input, float position);

static int armed = 0;
static int status = -1;

unsigned int motor[4];

void MotorControl_setArm(int a){
	armed = a;
}
int MotorControl_isArmed(void){
	return armed;
}
int MotorControl_getState(void){
	return status;
}
int MotorControl_getMotorValue(unsigned int * m){
	for (int i=0; i<4;i++) m[i] = motor[i];
	return 0;
}


int SetMotorValues(float motor, float pitch, float roll, float yaw, unsigned int * m){
	for (int i=0;i<4;i++){
		float set_motor = 0;
		if (motor_action[i]&(0x1<<0)) set_motor +=motor;	//THROTTLE
		if (motor_action[i]&(0x1<<1)) set_motor -=motor;	//THROTTLE
		if (motor_action[i]&(0x1<<2)) set_motor +=pitch;
		if (motor_action[i]&(0x1<<3)) set_motor -=pitch;

		if (motor_action[i]&(0x1<<4)) set_motor +=roll;
		if (motor_action[i]&(0x1<<5)) set_motor -=roll;
		if (motor_action[i]&(0x1<<6)) set_motor +=yaw;
		if (motor_action[i]&(0x1<<7)) set_motor -=yaw;

		//set_motor *= motor_mux[i];
		set_motor += motor_offset[i];

		if (set_motor > motor_max[i]) set_motor = motor_max[i];
		if (set_motor < motor_min[i]) set_motor = motor_min[i];

		m[i] = (unsigned int)set_motor;
	}
	return 0;
}

int ApplyMotorValues(unsigned int * m, int reset){

	for (int i=0;i<4;i++){
		char brashed = (motor_action[i]&(0x1<<8)) ? 1 : 0;

		unsigned int offsetMotor = 1000;
		if (brashed) {offsetMotor = 0; m[i]*=(MOTOR_LIMIT_BRUSHED/MOTOR_LIMIT_BRUSHLESS);}	//zero offset for brashed motor

		unsigned int newValue = 0;
		if (reset){
			newValue = offsetMotor + motor_init[i];
			m[i]=0;
		}
		else {
			float limit = MOTOR_LIMIT_BRUSHLESS;
			if (brashed) limit = MOTOR_LIMIT_BRUSHED;
			if (m[i]>limit) {m[i] = limit;}
			newValue = offsetMotor + m[i];
		}
		if (i==0)		TIM3->CCR1 = newValue;
		else if (i==1)	TIM3->CCR2 = newValue;
		else if (i==2)	TIM3->CCR3 = newValue;
		else if (i==3)	TIM3->CCR4 = newValue;
	}

	if (reset){
		status = MOTOR_STATUS_READY;

		mPID[THROTTLE].Reset = 1;	// reset i parameter when no running
		mPID[PITCH].Reset = 1;	// reset i parameter when no running
		mPID[ROLL].Reset = 1;	// reset i parameter when no running
		mPID[YAW].Reset = 1;	// reset i parameter when no running
	}
	else {
		status = MOTOR_STATUS_LAUNCHED;
	}
	return 0;
}

void MotorControl_loop(void){

	if (armed  != 1) {MotorControl_init(); return;}

	UpdateFromParam();



	float pitch = imu_getPitch();
	float roll = imu_getRoll();
	float yaw = imu_getYaw();

	uint32_t delay_update = mavlink_getChanUpdateTimer();

	float Throll_Chan = ((float)(mavlink_getChan(0)-1000))/1000.0f;
	float Yaw_Chan = ((float)(mavlink_getChan(1)-1500))/1000.0f;
	float Pitch_Chan = ((float)(mavlink_getChan(2)-1500))/1000.0f;
	float Roll_Chan = ((float)(mavlink_getChan(3)-1500))/1000.0f;

	float force[4];

	float mux_t = params_GetMemValue(PARAM_MUX_T_CHAN);//1.0f;//((float)SET_StabKoeffCh2)/10.0;	//20;
	float mux_p = params_GetMemValue(PARAM_MUX_P_CHAN);//1.0f;//((float)SET_StabKoeffCh2)/10.0;	//20;
	float mux_r = params_GetMemValue(PARAM_MUX_R_CHAN);//1.0f;//((float)SET_StabKoeffCh2)/10.0;	//20;
	float mux_y = params_GetMemValue(PARAM_MUX_Y_CHAN);//1.0f;//((float)SET_StabKoeffCh2)/10.0;	//20;

	static int ind_update = 0;
	if (delay_update > 2000){	// if you haven't received any more
		Throll_Chan = 0;
		Yaw_Chan = 0;
		Pitch_Chan = 0;
		Roll_Chan = 0;
		if (ind_update == 1) Printf("ALARM STOP\n\r");
		ind_update = 0;
		MotorControl_init();
		return;
	}
	else {
		ind_update = 1;
	}

	float Throll_Chan_muxed = Throll_Chan * mux_t;
	float Pitch_Chan_muxed = Pitch_Chan * mux_p;
	float Roll_Chan_muxed = Roll_Chan * mux_r;
	static float Yaw_Chan_muxed = 0;
#define IMU_FREQ 100.0f
	Yaw_Chan_muxed +=Yaw_Chan * mux_y/IMU_FREQ;
	if (Yaw_Chan_muxed > 180)Yaw_Chan_muxed -= 360.0f;
	if (Yaw_Chan_muxed < -180)Yaw_Chan_muxed += 360.0f;

	force[THROTTLE] = SetPID(&mPID[THROTTLE], 0 + Throll_Chan_muxed, 0);
	force[PITCH] = SetPID(&mPID[PITCH], pitch + Pitch_Chan_muxed, 0);
	force[ROLL] = SetPID(&mPID[ROLL], roll + Roll_Chan_muxed, 0);
	force[YAW] = SetPID(&mPID[YAW], yaw + Yaw_Chan_muxed, 0);

//	float setMotor = Throll_Chan*1.0f*(float)MOTOR_LIMIT;//((SETmainMotorValue*1000)/0xFF);
//	if (Throll_Chan <= 0) {
//		mPID[THROTTLE].Reset = 1;
//		mPID[PITCH].Reset = 1;
//		mPID[ROLL].Reset = 1;
//		mPID[YAW].Reset = 1;
//	}

	//if (MOTOR_TYPE == MOTOR_BRUSHLESS) mux = ((float)SET_StabKoeffCh2)/200.0;

//#if MOTOR_TYPE == MOTOR_BRUSHLESS
		//for rc36
		motor[0] = 0 + force[THROTTLE] + (force[PITCH]*1.0f) + (force[ROLL]*1.0f) - (force[YAW]*1.0f);// -trim_pitch-trim_roll-trim_yaw;
		motor[1] = 0 + force[THROTTLE] - (force[PITCH]*1.0f) + (force[ROLL]*1.0f) + (force[YAW]*1.0f);// +trim_pitch-trim_roll+trim_yaw;		//for "X" 		// for mobicaro new
		motor[2] = 0 + force[THROTTLE] - (force[PITCH]*1.0f) - (force[ROLL]*1.0f) - (force[YAW]*1.0f);// +trim_pitch+trim_roll-trim_yaw;
		motor[3] = 0 + force[THROTTLE] + (force[PITCH]*1.0f) - (force[ROLL]*1.0f) + (force[YAW]*1.0f);// -trim_pitch+trim_roll+trim_yaw;



//		SetMotorValues(setMotor, force[PITCH], force[ROLL], force[YAW], &motor);
		SetMotorValues(force[THROTTLE], force[PITCH], force[ROLL], force[YAW], motor);

		if (Throll_Chan == 0) ApplyMotorValues(motor, 1);
		else ApplyMotorValues(motor, 0);
}


void UpdateFromParam(void){
	mPID[THROTTLE].P = params_GetMemValue(PARAM_PID_T_P);
	mPID[THROTTLE].I = params_GetMemValue(PARAM_PID_T_I);
	mPID[THROTTLE].D = params_GetMemValue(PARAM_PID_T_D);

	mPID[PITCH].P = params_GetMemValue(PARAM_PID_P_P);
	mPID[PITCH].I = params_GetMemValue(PARAM_PID_P_I);
	mPID[PITCH].D = params_GetMemValue(PARAM_PID_P_D);

	mPID[ROLL].P = params_GetMemValue(PARAM_PID_R_P);
	mPID[ROLL].I = params_GetMemValue(PARAM_PID_R_I);
	mPID[ROLL].D = params_GetMemValue(PARAM_PID_R_D);

	mPID[YAW].P = params_GetMemValue(PARAM_PID_Y_P);
	mPID[YAW].I = params_GetMemValue(PARAM_PID_Y_I);
	mPID[YAW].D = params_GetMemValue(PARAM_PID_Y_D);

	int i=0;
	for (i=0;i<4;i++){
		float get_data_action = 0;
		float get_data_min = 0;
		float get_data_max = 0;
//		float get_data_mux = 0;
		float get_data_init = 0;
		float get_data_offset = 0;
		if (i==0) {
			get_data_action = params_GetMemValue(PARAM_M1_ACTION);
			get_data_min = params_GetMemValue(PARAM_M1_MIN);
			get_data_max = params_GetMemValue(PARAM_M1_MAX);
//			get_data_mux = params_GetMemValue(PARAM_M1_MUX);
			get_data_init = params_GetMemValue(PARAM_M1_INIT);
			get_data_offset = params_GetMemValue(PARAM_M1_OFFSET);
		}
		else if (i==1) {
			get_data_action = params_GetMemValue(PARAM_M2_ACTION);
			get_data_min = params_GetMemValue(PARAM_M2_MIN);
			get_data_max = params_GetMemValue(PARAM_M2_MAX);
//			get_data_mux = params_GetMemValue(PARAM_M2_MUX);
			get_data_init = params_GetMemValue(PARAM_M2_INIT);
			get_data_offset = params_GetMemValue(PARAM_M2_OFFSET);
		}
		else if (i==2) {
			get_data_action = params_GetMemValue(PARAM_M3_ACTION);
			get_data_min = params_GetMemValue(PARAM_M3_MIN);
			get_data_max = params_GetMemValue(PARAM_M3_MAX);
//			get_data_mux = params_GetMemValue(PARAM_M3_MUX);
			get_data_init = params_GetMemValue(PARAM_M3_INIT);
			get_data_offset = params_GetMemValue(PARAM_M3_OFFSET);
		}
		else if (i==3) {
			get_data_action = params_GetMemValue(PARAM_M4_ACTION);
			get_data_min = params_GetMemValue(PARAM_M4_MIN);
			get_data_max = params_GetMemValue(PARAM_M4_MAX);
//			get_data_mux = params_GetMemValue(PARAM_M4_MUX);
			get_data_init = params_GetMemValue(PARAM_M4_INIT);
			get_data_offset = params_GetMemValue(PARAM_M4_OFFSET);
		}
		if ((0 <= get_data_action) || (get_data_action<=1023)) {
			motor_action[i] = (unsigned short)get_data_action;
		}

		motor_min[i] = get_data_min;
		motor_max[i] = get_data_max;
//		motor_mux[i] = get_data_mux;
		motor_offset[i] = get_data_offset;

		if ((1000 <= get_data_init) || (get_data_init<=2000)) {
			motor_init[i] = (unsigned short) get_data_init;
		}
	}


}

void MotorControl_init(void){

	  UpdateFromParam();	//update local pid

	  ApplyMotorValues(motor, 1);

//#if MOTOR_TYPE == MOTOR_BRUSHED
//		  TIM3->CCR1 = 0;	//main motor
//		  TIM3->CCR3 = 0;	//servo2
//		  TIM3->CCR4 = 0;	//servo1
//		  TIM3->CCR2 = 0;	//second motor
//		  TIM2->CCR2 = 0;	//motor3
//		  TIM2->CCR4 = 0;	//motor4
//#else
//		  TIM3->CCR1 = 1000+motor_init[0];//1000;	//main motor
//		  TIM3->CCR2 = 1000+motor_init[1];//1000;	//second motor
//		  TIM3->CCR3 = 1000+motor_init[2];//1000;	//servo2
//		  TIM3->CCR4 = 1000+motor_init[3];//1000;	//servo1
//
//		  TIM2->CCR2 = 1000;	//motor3
//		  TIM2->CCR4 = 1000;	//motor4
//#endif
//
//
//	  mPID[PITCH].Reset = 1;	// reset i parameter when no running
//	  mPID[ROLL].Reset = 1;	// reset i parameter when no running
//	  mPID[YAW].Reset = 1;	// reset i parameter when no running

	  status = MOTOR_STATUS_OK;
}



float SetPID(volatile t_pid *PID, float input, float position)
{
	static float result;
	static float Ierror;
	//
	if (PID->Reset)
	{
		PID->Reset = 0;
		PID->lastResult = -position;
		PID->lastInput = position;
	}
	Ierror = PID->I * input;
	if (Ierror > PID->MaxLimit) Ierror = PID->MaxLimit;
	else if (Ierror < -PID->MaxLimit) Ierror = -PID->MaxLimit;
	PID->lastResult += Ierror;
	result = PID->P * input +
			 PID->lastResult +
			 PID->D * (input - PID->lastInput);
	PID->lastInput = input;
	return result;
}
