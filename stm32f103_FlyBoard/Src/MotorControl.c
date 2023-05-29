



#include "stm32f1xx.h"
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

t_pid mPID[4] = {{1,0,0,0,0,0,1000.0f},{1,0,0,0,0,0,1000.0f},{1,0,0,0,0,0,1000.0f},{1,0,0,0,0,0,1000.0f}};


float SetPID(volatile t_pid *PID, float input, float position);

static int armed = 0;
static int status = -1;


void MotorControl_setArm(int a){
	armed = a;
}
int MotorControl_isArmed(void){
	return armed;
}
int MotorControl_getState(void){
	return status;
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

	float mux_pr = params_GetMemValue(PARAM_MUX_PR_CHAN);//1.0f;//((float)SET_StabKoeffCh2)/10.0;	//20;
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

	float Pitch_Chan_muxed = Pitch_Chan * mux_pr;
	float Roll_Chan_muxed = Roll_Chan * mux_pr;
	static float Yaw_Chan_muxed = 0;
#define IMU_FREQ 100.0f
	Yaw_Chan_muxed +=Yaw_Chan * mux_y/IMU_FREQ;
	if (Yaw_Chan_muxed > 180)Yaw_Chan_muxed -= 360.0f;
	if (Yaw_Chan_muxed < -180)Yaw_Chan_muxed += 360.0f;

	force[PITCH] = SetPID(&mPID[PITCH], pitch + Pitch_Chan_muxed, 0);
	force[ROLL] = SetPID(&mPID[ROLL], roll + Roll_Chan_muxed, 0);
	force[YAW] = SetPID(&mPID[YAW], yaw + Yaw_Chan_muxed, 0);

	unsigned int motor1, motor2, motor3, motor4;

#define MOTOR_LIMIT 1000

	// TODO 0.5 for debug only for tests
	float setMotor = Throll_Chan*0.5*(float)MOTOR_LIMIT;//((SETmainMotorValue*1000)/0xFF);
	if (Throll_Chan <= 0) {mPID[PITCH].Reset = 1;mPID[ROLL].Reset = 1;mPID[YAW].Reset = 1;}

	//if (MOTOR_TYPE == MOTOR_BRUSHLESS) mux = ((float)SET_StabKoeffCh2)/200.0;

#if MOTOR_TYPE == MOTOR_BRUSHLESS
		//for rc36
		motor1 = 0 + setMotor + (force[PITCH]*1.0f) + (force[ROLL]*1.0f) - (force[YAW]*1.0f);// -trim_pitch-trim_roll-trim_yaw;
		motor2 = 0 + setMotor - (force[PITCH]*1.0f) + (force[ROLL]*1.0f) + (force[YAW]*1.0f);// +trim_pitch-trim_roll+trim_yaw;		//for "X" 		// for mobicaro new
		motor3 = 0 + setMotor - (force[PITCH]*1.0f) - (force[ROLL]*1.0f) - (force[YAW]*1.0f);// +trim_pitch+trim_roll-trim_yaw;
		motor4 = 0 + setMotor + (force[PITCH]*1.0f) - (force[ROLL]*1.0f) + (force[YAW]*1.0f);// -trim_pitch+trim_roll+trim_yaw;


		if (motor1>MOTOR_LIMIT) {motor1 = MOTOR_LIMIT;}
		if (motor2>MOTOR_LIMIT) {motor2 = MOTOR_LIMIT;}
		if (motor3>MOTOR_LIMIT) {motor3 = MOTOR_LIMIT;}
		if (motor4>MOTOR_LIMIT) {motor4 = MOTOR_LIMIT;}

		int MOTOR_DIV = 1;
		//if (MOTOR_TYPE == MOTOR_BRUSHED) MOTOR_DIV = 10;

		if (setMotor == 0){
		  TIM3->CCR1 = 1000;
		  TIM3->CCR2 = 1000;
		  TIM3->CCR3 = 1000;
		  TIM3->CCR4 = 1000;
		  status = MOTOR_STATUS_READY;

		  mPID[PITCH].Reset = 1;	// reset i parameter when no running
		  mPID[ROLL].Reset = 1;	// reset i parameter when no running
		  mPID[YAW].Reset = 1;	// reset i parameter when no running
		}
		else {
		  TIM3->CCR1 = 1000+motor1;//(motor1/MOTOR_DIV);	//10000;	//main motor			//J13
		  TIM3->CCR2 = 1000+motor2;//(motor2/MOTOR_DIV);	//13000;	//second motor			//J15
		  TIM3->CCR3 = 1000+motor3;//(motor3/MOTOR_DIV);	//11000;	//servo2				//J17
		  TIM3->CCR4 = 1000+motor4;//(motor4/MOTOR_DIV);	//12000;	//servo1				//J19
		  status = MOTOR_STATUS_LAUNCHED;
		}
		  static int ind = 0;
		  if (++ind>10){ind=0;
		  	  Printf("%d, %d, %d, %d\n\r", motor1/10, motor2/10, motor3/10, motor4/10);
		  	  //Printf("f %d %d %d\n\r", (int)(force[PITCH]*1.0f), (int)(force[ROLL]*1.0f),(int)(force[YAW]*1.0f));
		  	  //Printf("f %d %d\n\r",  (int)(yaw*1.0f),(int)(force[YAW]*1.0f));
		  }

#endif

#if MOTOR_TYPE == MOTOR_BRUSHED
		//for rc36
		motor1 = 0 + setMotor - (force[PITCH]*mux) - (force[ROLL]*mux) - (force[YAW]*mux);// -trim_pitch-trim_roll-trim_yaw;		//for "X" 		// for mobicaro new
		motor3 = 0 + setMotor + (force[PITCH]*mux) + (force[ROLL]*mux) - (force[YAW]*mux);// +trim_pitch+trim_roll-trim_yaw;
		motor4 = 0 + setMotor - (force[PITCH]*mux) + (force[ROLL]*mux) + (force[YAW]*mux);// -trim_pitch+trim_roll+trim_yaw;
		motor2 = 0 + setMotor + (force[PITCH]*mux) - (force[ROLL]*mux) + (force[YAW]*mux);// +trim_pitch-trim_roll+trim_yaw;

#define MOTOR_LIMIT 2000

		if (motor1>MOTOR_LIMIT) {motor1 = MOTOR_LIMIT;}
		if (motor2>MOTOR_LIMIT) {motor2 = MOTOR_LIMIT;}
		if (motor3>MOTOR_LIMIT) {motor3 = MOTOR_LIMIT;}
		if (motor4>MOTOR_LIMIT) {motor4 = MOTOR_LIMIT;}

		int MOTOR_DIV = 1;
		//if (MOTOR_TYPE == MOTOR_BRUSHED) MOTOR_DIV = 10;

		  TIM3->CCR1 = motor1;//(motor1/MOTOR_DIV);	//10000;	//main motor			//J13
		  TIM3->CCR2 = motor2;//(motor2/MOTOR_DIV);	//13000;	//second motor			//J15
		  TIM3->CCR3 = motor3;//(motor3/MOTOR_DIV);	//11000;	//servo2				//J17
		  TIM3->CCR4 = motor4;//(motor4/MOTOR_DIV);	//12000;	//servo1				//J19

		  static int ind = 0;
		  if (++ind>100){ind=0;
		  	  Printf("m %d %d %d %d\n\r", motor1, motor2, motor3, motor4);
		  }
#endif

}


void UpdateFromParam(void){
	mPID[PITCH].P = params_GetMemValue(PARAM_PID_PR_P);
	mPID[ROLL].P = params_GetMemValue(PARAM_PID_PR_P);
	mPID[YAW].P = params_GetMemValue(PARAM_PID_Y_P);

	mPID[PITCH].I = params_GetMemValue(PARAM_PID_PR_I);
	mPID[ROLL].I = params_GetMemValue(PARAM_PID_PR_I);
	mPID[YAW].I = 0;//params_GetMemValue(PARAM_PID_Y_I);

	mPID[PITCH].D = params_GetMemValue(PARAM_PID_PR_D);
	mPID[ROLL].D = params_GetMemValue(PARAM_PID_PR_D);
	mPID[YAW].D = 0;//params_GetMemValue(PARAM_PID_Y_D);


}

void MotorControl_init(void){
#if MOTOR_BRUSHED
		  TIM3->CCR1 = 0;	//main motor
		  TIM3->CCR3 = 0;	//servo2
		  TIM3->CCR4 = 0;	//servo1
		  TIM3->CCR2 = 0;	//second motor
		  TIM2->CCR2 = 0;	//motor3
		  TIM2->CCR4 = 0;	//motor4
#else
		  TIM3->CCR1 = 1000;	//main motor
		  TIM3->CCR3 = 1000;	//servo2
		  TIM3->CCR4 = 1000;	//servo1
		  TIM3->CCR2 = 1000;	//second motor

		  TIM2->CCR2 = 1000;	//motor3
		  TIM2->CCR4 = 1000;	//motor4
#endif

	  UpdateFromParam();

	  mPID[PITCH].Reset = 1;	// reset i parameter when no running
	  mPID[ROLL].Reset = 1;	// reset i parameter when no running
	  mPID[YAW].Reset = 1;	// reset i parameter when no running

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
