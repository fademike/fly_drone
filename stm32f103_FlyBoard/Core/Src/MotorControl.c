



#include "stm32f1xx.h"
#include "main.h"
#include "MotorControl.h"
#include "mavlink_handler.h"
#include "params.h"
#include "imu.h"

//extern TIM_HandleTypeDef htim3;

void UpdateFromParam(void);



typedef struct {
	unsigned int throttle_p	:1;
	unsigned int throttle_m	:1;
	unsigned int pitch_p	:1;
	unsigned int pitch_m	:1;
	unsigned int roll_p		:1;
	unsigned int roll_m		:1;
	unsigned int yaw_p		:1;
	unsigned int yaw_m		:1;
	unsigned int brashed	:1;
} motor_action_struct;

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

float motor[4];

void MotorControl_setArm(int a){
	armed = a;
}
int MotorControl_isArmed(void){
	return armed;
}
int MotorControl_getState(void){
	return status;
}
float MotorControl_getMotorValue(float * m){
	for (int i=0; i<4;i++) m[i] = motor[i];
	return 0;
}


int SetMotorValues(float throttle, float pitch, float roll, float yaw, float * m){
	for (int i=0;i<4;i++){
		float set_motor = 0;

		motor_action_struct * mAction = (motor_action_struct *) &motor_action[i];

		if (mAction->throttle_p) set_motor +=throttle;
		if (mAction->throttle_m) set_motor -=throttle;
		if (mAction->pitch_p) set_motor +=pitch;
		if (mAction->pitch_m) set_motor -=pitch;
		if (mAction->roll_p) set_motor +=roll;
		if (mAction->roll_m) set_motor -=roll;
		if (mAction->yaw_p) set_motor +=yaw;
		if (mAction->yaw_m) set_motor -=yaw;

		//set_motor *= motor_mux[i];
		set_motor += motor_offset[i];

		if (set_motor > motor_max[i]) set_motor = motor_max[i];
		if (set_motor < motor_min[i]) set_motor = motor_min[i];

		m[i] = set_motor;
	}
	return 0;
}

int ApplyMotorValues(float * m, int reset){
	int brashed_cnt = 0;
	for (int i=0;i<4;i++){
		char brashed = (motor_action[i]&(0x1<<8)) ? 1 : 0;	// 1 - brashed; 0 - brashless motor
		if (brashed) brashed_cnt++;

		float offsetMotor = 1000;
		if (brashed) {offsetMotor = 0; m[i]*=(float)(MOTOR_LIMIT_BRUSHED/MOTOR_LIMIT_BRUSHLESS);}	//zero offset for brashed motor

		float newValue = 0;
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
		*(uint32_t *)(&TIM3->CCR1 + (i)) = (uint32_t) newValue;	//FIXME
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
		if (brashed_cnt >= 4) TIM3->PSC = TIM_PRESCALER/8;	// increasing the frequency if all motor is brashed // FIXME
		else TIM3->PSC = TIM_PRESCALER;			// normal frequency for brashless motor
	}
	return 0;
}

static float LPFilter(float * f, float data){
	*f += (data-*f)*0.05f;
	return *f;
}


void MotorControl_loop(void){

	if (armed  != 1) {MotorControl_init(); return;}

	UpdateFromParam();



	float pitch = imu_getPitch();
	float roll = imu_getRoll();
	float yaw = imu_getYaw();
	int alt = 0;//imu_getAlt();

	int alt_max = (int)params_GetMemValue(ALT_MAX);
	//if ((alt_max < 0) || (alt_max > 2000)) alt_max = 0;
	if ((alt_max > 0) || (alt_max < 2000)) alt = imu_getAlt();
	else alt_max=0;

	uint32_t delay_update = mavlink_getChanUpdateTimer();

	float Throll_Chan = ((float)(mavlink_getChan(0)-1000))/1000.0f;
	float Yaw_Chan = ((float)(mavlink_getChan(1)-1500))/1000.0f;
	float Pitch_Chan = ((float)(mavlink_getChan(2)-1500))/1000.0f;
	float Roll_Chan = ((float)(mavlink_getChan(3)-1500))/1000.0f;

	float force[4];

	float mux_t = params_GetMemValue(PARAM_MUX_T_CHAN);
	float mux_p = params_GetMemValue(PARAM_MUX_P_CHAN);
	float mux_r = params_GetMemValue(PARAM_MUX_R_CHAN);
	float mux_y = params_GetMemValue(PARAM_MUX_Y_CHAN);

	static int ind_update = 0;
	if (delay_update > 2000){	// if you haven't received chain any more
		if (ind_update == 1) Printf("ALARM STOP\n\r");
		ind_update = 0;			// so that I don't send it every time
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

	float set_alt = 0;
	if ((alt_max != 0) && (alt >= 0)){
		set_alt = -alt;
		Throll_Chan_muxed*=(float)alt_max/1000.0f;	// change raw value to percent alt_max

		if (Throll_Chan_muxed <= alt) {
			mPID[THROTTLE].MaxLimit = params_GetMemValue(PARAM_T_STEP)/10.0f;	//for a slow change
		} else {
			mPID[THROTTLE].MaxLimit = params_GetMemValue(PARAM_T_STEP);
		}
		// static int dd=0;	// for debug
		// if (dd++>=100){
		// 	dd=0;
		// 	Printf("diff = %d, chan=%d, alt=%d\n\r", abs((int)Throll_Chan_muxed - (int)alt), (int)Throll_Chan_muxed, (int)alt);
		// }
	}
	else {
		mPID[THROTTLE].I=0;
		mPID[THROTTLE].D=0;
	}

	force[THROTTLE] = SetPID(&mPID[THROTTLE], Throll_Chan_muxed + set_alt, 0);
	force[PITCH] = SetPID(&mPID[PITCH], pitch + Pitch_Chan_muxed, 0);
	force[ROLL] = SetPID(&mPID[ROLL], roll + Roll_Chan_muxed, 0);
	force[YAW] = SetPID(&mPID[YAW], yaw + Yaw_Chan_muxed, 0);

	if (mPID[THROTTLE].lastResult<0) mPID[THROTTLE].lastResult = 0;

	static float f = 0.0f;

	SetMotorValues(LPFilter(&f, force[THROTTLE]), force[PITCH], force[ROLL], force[YAW], motor);

	if (Throll_Chan <= 0) ApplyMotorValues(motor, 1);	// reset motor
	else ApplyMotorValues(motor, 0);
}


void UpdateFromParam(void){
	mPID[THROTTLE].P = params_GetMemValue(PARAM_PID_T_P);
	mPID[THROTTLE].I = params_GetMemValue(PARAM_PID_T_I);
	mPID[THROTTLE].D = params_GetMemValue(PARAM_PID_T_D);

	mPID[THROTTLE].MaxLimit = params_GetMemValue(PARAM_T_STEP);

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

			get_data_action = params_GetMemValue(PARAM_M1_ACTION + (PARAM_MOTOR_LEN) *i);
			get_data_min = params_GetMemValue(PARAM_M1_MIN + (PARAM_MOTOR_LEN) *i);
			get_data_max = params_GetMemValue(PARAM_M1_MAX + (PARAM_MOTOR_LEN) *i);
//			get_data_mux = params_GetMemValue(PARAM_M1_MUX);
			get_data_init = params_GetMemValue(PARAM_M1_INIT + (PARAM_MOTOR_LEN) *i);
			get_data_offset = params_GetMemValue(PARAM_M1_OFFSET + (PARAM_MOTOR_LEN) *i);

		if ((0 <= get_data_action) || (get_data_action<=1023)) {
			motor_action[i] = (unsigned int)get_data_action;
		}

		motor_min[i] = get_data_min;
		motor_max[i] = get_data_max;
//		motor_mux[i] = get_data_mux;
		motor_offset[i] = get_data_offset;

//		if ((1000 <= get_data_init) && (get_data_init<=2000)) {
			motor_init[i] = (unsigned short) get_data_init;
//		}
	}


}

void MotorControl_init(void){

	  UpdateFromParam();	//update local pid
	  ApplyMotorValues(motor, 1);
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
