



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
		mPID[THROTTLE].Reset = 1;	// reset i parameter when no running
		mPID[PITCH].Reset = 1;
		mPID[ROLL].Reset = 1;
		mPID[YAW].Reset = 1;
		status = MOTOR_STATUS_READY;
	}
	else status = MOTOR_STATUS_LAUNCHED;
	
	if (brashed_cnt >= 4) TIM3->PSC = TIM_PRESCALER/8;	// increasing the frequency if all motor is brashed // FIXME
	else TIM3->PSC = TIM_PRESCALER;			// normal frequency for brashless motor

	return 0;
}

void MotorControl_loop(void){

	if (armed  != 1) {MotorControl_init(); return;}

	UpdateFromParam();

	float pitch = imu_getPitch();
	float roll = imu_getRoll();
	float yaw = imu_getYaw();
	int alt = imu_getAlt();

	int alt_max = (int)params_GetParamValue(ALT_MAX);

	uint32_t delay_update = mavlink_getChanUpdateTimer();

	float Throll_Chan = ((float)(mavlink_getChan(THROTTLE)-1000))/1000.0f;
	float Yaw_Chan = ((float)(mavlink_getChan(YAW)-1500))/1000.0f;
	float Pitch_Chan = ((float)(mavlink_getChan(PITCH)-1500))/1000.0f;
	float Roll_Chan = ((float)(mavlink_getChan(ROLL)-1500))/1000.0f;


	static int ind_update = 0;
	if (delay_update > 2000){	// if you haven't received chain any more
		if (ind_update == 1) Printf("ALARM STOP\n\r");
		ind_update = 0;			// so that I don't send it every time
		MotorControl_init();
		return;
	}
	ind_update = 1;

	float Throll_Chan_muxed = Throll_Chan * params_GetParamValue(PARAM_PID_T_MUX_CHAN);
	float Pitch_Chan_muxed = Pitch_Chan * params_GetParamValue(PARAM_PID_P_MUX_CHAN);
	float Roll_Chan_muxed = Roll_Chan * params_GetParamValue(PARAM_PID_R_MUX_CHAN);
	static float Yaw_Chan_muxed = 0;
#define YAW_UPDATED_FREQ 10.0f // Hz 
	Yaw_Chan_muxed +=Yaw_Chan * params_GetParamValue(PARAM_PID_Y_MUX_CHAN)/YAW_UPDATED_FREQ;
	if (Yaw_Chan_muxed > 180)Yaw_Chan_muxed -= 360.0f;
	if (Yaw_Chan_muxed < -180)Yaw_Chan_muxed += 360.0f;

	if ((alt_max > 0) && (alt >= 0)){
		Throll_Chan_muxed*=(float)alt_max/1000.0f;	// change raw value to percent alt_max

		#define SLOW_WINDOW 0.0f

		// if (((Throll_Chan_muxed*(1.0-SLOW_WINDOW)) <= alt) && (alt <= (Throll_Chan_muxed*(1.0f+2*SLOW_WINDOW))) ) {
		if (alt > (Throll_Chan_muxed*(1.0-SLOW_WINDOW))) {
			mPID[THROTTLE].MaxLimit = params_GetParamValue(PARAM_PID_T_I_LIMIT)/10.0f;	//for a slow change
		// }
		// else if (imu_getAltCanged()>(Throll_Chan_muxed/mPID[THROTTLE].D)){
		// 	mPID[THROTTLE].MaxLimit = params_GetParamValue(PARAM_PID_T_I_LIMIT)/10.0f;
		} else {
			mPID[THROTTLE].MaxLimit = params_GetParamValue(PARAM_PID_T_I_LIMIT);
		}
		// static float f=0;
		// f += ((float)imu_getAltCanged() - f)*0.01;
		// alt += f*mPID[THROTTLE].D;
		// mPID[THROTTLE].D=0;
	}
	// else {
	// 	mPID[THROTTLE].P=1;
	// 	mPID[THROTTLE].I=0;
	// 	mPID[THROTTLE].D=0;
	// }

	static float yaw_free = 0;
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) {yaw_free = yaw; Yaw_Chan_muxed = 0;}
	yaw = yaw - yaw_free;
	while (yaw <-180) yaw += 360.0f;
	while (yaw>180) yaw -= 360.0f;

	if (params_GetParamValue(PARAM_P_MODE) == MODE_CAR){
		pitch = 0;
		roll = 0;
		yaw = 0;
		alt = 0;
	}

	float force[4];
	force[THROTTLE] = SetPID(&mPID[THROTTLE], Throll_Chan_muxed - alt, 0);
	force[PITCH] = SetPID(&mPID[PITCH], pitch + Pitch_Chan_muxed, 0);
	force[ROLL] = SetPID(&mPID[ROLL], roll + Roll_Chan_muxed, 0);
	force[YAW] = SetPID(&mPID[YAW], yaw + Yaw_Chan_muxed, 0);

	float t_min = params_GetParamValue(PARAM_PID_T_I_MIN);
	if (mPID[THROTTLE].lastResult<t_min) mPID[THROTTLE].lastResult = t_min;		// set min value!

	// static int d=0;
	// if (d++>20){d=0;
	// 	Printf("%d,%d,%d,%d", (int)force[THROTTLE], (int)Throll_Chan_muxed, (int)set_alt, (int)alt);
	// }

	SetMotorValues(force[THROTTLE], force[PITCH], force[ROLL], force[YAW], motor);	//LPFilter(&f, force[THROTTLE], 0.05f)

	if ((params_GetParamValue(PARAM_P_MODE) != MODE_CAR) &&
		 (Throll_Chan <= 0)) ApplyMotorValues(motor, 1);	// reset motor
	else ApplyMotorValues(motor, 0);
}


void UpdateFromParam(void){

	for (int i=0;i<4;i++){
		mPID[i].P = params_GetParamValue(PARAM_PID_T_P + (PARAM_PID_LEN) * i);
		mPID[i].I = params_GetParamValue(PARAM_PID_T_I + (PARAM_PID_LEN) * i);
		mPID[i].D = params_GetParamValue(PARAM_PID_T_D + (PARAM_PID_LEN) * i);
	}

	mPID[THROTTLE].MaxLimit = params_GetParamValue(PARAM_PID_T_I_LIMIT);

	for (int i=0;i<4;i++){
		motor_action[i] = (unsigned int)params_GetParamValue(PARAM_M1_ACTION + (PARAM_MOTOR_LEN) *i);
		motor_min[i] = params_GetParamValue(PARAM_M1_MIN + (PARAM_MOTOR_LEN) *i);
		motor_max[i] = params_GetParamValue(PARAM_M1_MAX + (PARAM_MOTOR_LEN) *i);
//		motor_mux[i] = params_GetParamValue(PARAM_M1_MUX);
		motor_offset[i] = params_GetParamValue(PARAM_M1_OFFSET + (PARAM_MOTOR_LEN) *i);
		motor_init[i] = (unsigned short) params_GetParamValue(PARAM_M1_INIT + (PARAM_MOTOR_LEN) *i);
	}
}

void MotorControl_init(void){
	  UpdateFromParam();	//update local pid
	  ApplyMotorValues(motor, 1);
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
