
/*

MotorControl_loop():
Example for PITCH Chan to M1:
/--------------------\  /---------------------\  /----------------------\
| Chan[PITCH]        |->| -PARAM_PID_P_OFFSET |->|*PARAM_PID_P_MUX_CHAN |
| from mavlink       |  | (-chan_p_offset)    |  | (*chan_p_mux)        |
|                    |  | /1000               |  |                      |
| Values:1000-2000   |  | OUT: -0.5...0.5 *   |  |                      |
\--------------------/  \---------------------/  \----------------------/
/---PWM------\  /---MOTOR-SETTINGS-----------------\              |
|            |  | PARAM_M1_ACTION  (m1_action)     |  /-----\     |   /----------\
|            |  | (m1_action set to +force[PITCH]) |  | PID |<----+--| IMU pitch  |
|            |<-| +PARAM_M1_OFFSET (m1_offset)     |  |     |         \----------/
|            |  | PARAM_M1_MIN     (m1_min)        |  \-----/         
|-ApplyMotor-|  | PARAM_M1_MAX     (m1_max)        |     |
\-Values()---/  \---SetMotorValues()---------------/   force[PITCH]

*- for throttle out values: 0..1

PARAM_P_MODE: 	bit 0		// Do not use angels from imu if set (pitch=0, rol=0...)
				bit 1		// If not set - reset motors, pids to default, when throttle equal 0 

*/


#include "stm32f1xx.h"
#include "main.h"
#include "MotorControl.h"
#include "mavlink_handler.h"
#include "params.h"
#include "imu.h"

void UpdateFromParam(void);

#define DECREASE_ANGLE(angel) {while (angel <-180) angel += 360.0f; while (angel>180) angel -= 360.0f;}

typedef struct {
	unsigned int throttle_p	:1;	// + throttle
	unsigned int throttle_m	:1; // - throttle
	unsigned int pitch_p	:1;
	unsigned int pitch_m	:1;

	unsigned int roll_p		:1;
	unsigned int roll_m		:1;
	unsigned int yaw_p		:1;
	unsigned int yaw_m		:1;
	
	unsigned int brashed	:1;	// set 1 when used all brashed motor. it is set PWM 0..MAX
	                            // set 0 when used servo or esc with 1..2ms impulse

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

unsigned int motor_action[4];// = {0,0,0,0};
float motor_min[4];// = {0,0,0,0};
float motor_max[4];// = {1000,1000,1000,1000};
float motor_offset[4];// = {0,0,0,0};

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

		float offsetMotor = 1000;	// imp 1ms for brashless motor
		// if brashed motor: set zero offset, recalculation for another limit
		if (brashed) {offsetMotor = 0; m[i]*=(float)(MOTOR_LIMIT_BRUSHED/MOTOR_LIMIT_BRUSHLESS);}

		float newValue = 0;
		if (reset){
			newValue = offsetMotor + motor_offset[i];
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

float filter_out = 0;

void MotorControl_loop(void){

	if (armed  != 1) {MotorControl_init(); return;}

	UpdateFromParam();	// reading settings from param

	float pitch = imu_getPitch();
	float roll = imu_getRoll();
	float yaw = imu_getYaw();
	int alt = imu_getAlt();

	uint32_t delay_update = mavlink_getChanUpdateTimer();

	float Throll_Chan = ((float)(mavlink_getChan(THROTTLE)-1000) - params_GetParamValue(PARAM_PID_T_OFFSET))/1000.0f;	// 0...1
	float Yaw_Chan = ((float)(mavlink_getChan(YAW)-1500) - params_GetParamValue(PARAM_PID_Y_OFFSET))/1000.0f;			// -0.5...0.5
	float Pitch_Chan = ((float)(mavlink_getChan(PITCH)-1500) - params_GetParamValue(PARAM_PID_P_OFFSET))/1000.0f;
	float Roll_Chan = ((float)(mavlink_getChan(ROLL)-1500) - params_GetParamValue(PARAM_PID_R_OFFSET))/1000.0f;

	if (delay_update > 2000){	// if you haven't received chain any more
		if ( MotorControl_getState() == MOTOR_STATUS_LAUNCHED) Printf("ALARM STOP\n\r");
		MotorControl_init();	// reset motor
		return;
	}

	float Throll_Chan_muxed = Throll_Chan * params_GetParamValue(PARAM_PID_T_MUX_CHAN);
	float Pitch_Chan_muxed = Pitch_Chan * params_GetParamValue(PARAM_PID_P_MUX_CHAN);
	float Roll_Chan_muxed = Roll_Chan * params_GetParamValue(PARAM_PID_R_MUX_CHAN);
	static float Yaw_Chan_muxed = 0;
#define YAW_UPDATED_FREQ 10.0f // Hz 
	Yaw_Chan_muxed += Yaw_Chan * params_GetParamValue(PARAM_PID_Y_MUX_CHAN)/YAW_UPDATED_FREQ;
	DECREASE_ANGLE(Yaw_Chan_muxed);

	static float yaw_base = 0;
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) {yaw_base = yaw; Yaw_Chan_muxed = 0;}
	yaw = yaw - yaw_base;
	DECREASE_ANGLE(yaw);

	if (IS_SET_BIT(P_BIT_IMU_NOT_USE, (int)params_GetParamValue(PARAM_P_MODE))){	// if do not use imu
		pitch = roll = yaw = alt = 0;
	}

	float force[4];
	force[THROTTLE] = SetPID(&mPID[THROTTLE], Throll_Chan_muxed - alt, 0);
	force[PITCH] = SetPID(&mPID[PITCH], pitch + Pitch_Chan_muxed, 0);
	force[ROLL] = SetPID(&mPID[ROLL], roll + Roll_Chan_muxed, 0);
	force[YAW] = SetPID(&mPID[YAW], yaw + Yaw_Chan_muxed, 0);	// FIXME!!! problem of crossing the circle

	SetMotorValues(force[THROTTLE], force[PITCH], force[ROLL], force[YAW], motor);

	ApplyMotorValues(motor, ((int)(Throll_Chan*1000.0f) == 0) && 
							!IS_SET_BIT(P_BIT_WITHOUT_RESET, (int)params_GetParamValue(PARAM_P_MODE)));
}


void UpdateFromParam(void){

	for (int i=0;i<4;i++){
		mPID[i].P = params_GetParamValue(PARAM_PID_T_P + (PARAM_PID_LEN) * i);
		mPID[i].I = params_GetParamValue(PARAM_PID_T_I + (PARAM_PID_LEN) * i);
		mPID[i].D = params_GetParamValue(PARAM_PID_T_D + (PARAM_PID_LEN) * i);
	}

	for (int i=0;i<4;i++){
		motor_action[i] = (unsigned int)params_GetParamValue(PARAM_M1_ACTION + (PARAM_MOTOR_LEN) *i);
		motor_min[i] = params_GetParamValue(PARAM_M1_MIN + (PARAM_MOTOR_LEN) *i);
		motor_max[i] = params_GetParamValue(PARAM_M1_MAX + (PARAM_MOTOR_LEN) *i);
		motor_offset[i] = params_GetParamValue(PARAM_M1_OFFSET + (PARAM_MOTOR_LEN) *i);
	}
}

// resetting motors to initial state
void MotorControl_init(void){
	  UpdateFromParam();	//update local pid, settings
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
