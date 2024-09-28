

#include "stm32f1xx.h"

#include <math.h>
#include "MPU9250.h"
#include "MahonyAHRS.h"
#include "system.h"
#include "params.h"
#include "MotorControl.h"
#include "main.h"

#include "imu.h"

#include "VL53L0X.h"

//statInfo_t_VL53L0X distanceStr;


void imu_autoCalibrateByNoize(int stop);
void imu_accCalibrate(void);


struct imuAngle_struct {
	float pitch;
	float roll;
	float yaw;
};
struct axis_struct {
	float x;
	float y;
	float z;
};

typedef union {
	struct axis_struct axis;
	struct imuAngle_struct angle;
	float value[3];
} vector;

vector vector_setConst(vector axes, float Value);
vector vector_muxConst(vector axes, const float Value);
vector vector_divConst(vector axes, const float Value);
vector vector_muxVector(vector axes, const vector axes2);
vector vector_addVector(vector axes, vector axes2);
vector vector_removeVector(vector axes, vector axes2);
vector vector_rearranging(vector axes, int * val);

vector imuAngle = {.angle.pitch=0,.angle.roll=0,.angle.yaw=0};

float imu_getPitch(void){return imuAngle.angle.pitch;}
float imu_getRoll(void){return imuAngle.angle.roll;}
float imu_getYaw(void){return imuAngle.angle.yaw;}
int imu_getAlt(void){
	int alt = 0;//get_altitude();
	int alt_max = (int)params_GetParamValue(ALT_MAX);
	if (alt_max > 0) {
		alt = get_altitude();
		alt *= 1000.0f/(float)alt_max;	//convert 
	}
	return alt;
}

vector gyro = {.axis.x=0,.axis.y=0,.axis.z=0};
vector gyro_offs = {.axis.x=0,.axis.y=0,.axis.z=0};

vector acc = {.axis.x=0,.axis.y=0,.axis.z=0};
vector acc_offs = {.axis.x=0,.axis.y=0,.axis.z=0};

static int status =-1;
static int statusCalibrateGyro = -1;
static int statusCalibrateAcc = 0;

int imu_getStatus(void){	// status < 0 - fail imu init; 0 - ok
	return status;
}
int imu_GyroCalibrate_getStatus(void){	// -2 fail, -1 process calib, 0 ok (in the process of recalibration)
	return statusCalibrateGyro;
}
void imu_GyroCalibrate_run(void){
	statusCalibrateGyro = -2;
}
int imu_AccCalibrate_getStatus(void){	// -1 need to calibrate, 0 finish of calibration
	return statusCalibrateAcc;
}
void imu_AccCalibrate_run(void){
	statusCalibrateAcc = -1;
}
void imu_AccOffset_get(float * ox, float * oy, float * oz){
	*ox = acc_offs.axis.x;
	*oy = acc_offs.axis.y;
	*oz = acc_offs.axis.z;
}
void imu_AccOffset_set(float ox, float oy, float oz){
	Printf("acc set %d, %d, %d\n\r", (int)(ox*1000.0f), (int)(oy*1000.0f), (int)(oz*1000.0f));
	acc_offs.axis.x = ox;
	acc_offs.axis.y = oy;
	acc_offs.axis.z = oz;
}
int imu_init(void){
	status = MPU_Init();
	Printf("imu init %d\n\r", status);
	return status;
}


void UpdateOrientation(int * dat_r, float * dat_a_sig, float * dat_g_sig){
	
	static float orientation = 0;
	float param_orientation = params_GetParamValue(PARAM_ORIENTATION);

	// 0, 24 - norm
	// 60 - up
	// dataset: 6 bit: sig_third,sig_second,sig_first,second_axis,[first_axis:2]
	// first_axis <= x=0, y=1, z=2;
	// second_axis = the remaining axes: if first_axit = x(0) => y=0,z=1; if first_axit = z(2) => x=0,y=1...
	// sig_first, sig_second, sig_third = sig of choised axis; 0- normal; 1 - sig minus
	if (orientation != param_orientation){
		orientation = param_orientation;
		if ((0 <= orientation) && (orientation <= 0xFFFF)){
			int i_o = (int)param_orientation;
			int v1 = i_o&0x3;
			if (v1>=AXIS_Z) v1 = AXIS_Z;
			dat_r[0] = v1;	// set first axis
			int v2 = (i_o>>2)&0x1;
			if ((v1 == AXIS_X) && (v2 == AXIS_Y)) v2++;
			if (v1 == v2) v2++;
			int v3 = 0;
			if ((v1 == v3) || (v2 == v3)) v3++;
			if ((v1 == v3) || (v2 == v3)) v3++;

			dat_r[0] = v1;
			dat_r[1] = v2;
			dat_r[2] = v3;

			int sig = (i_o>>4)&0x7;
			int s=0;
			for (s=0;s<3;s++) dat_a_sig[s] = 1.0;
			for (s=0;s<3;s++) if (sig&(0x1<<s)) dat_a_sig[s] = -1.0;
			sig = (i_o>>8)&0x7;
			for (s=0;s<3;s++) dat_g_sig[s] = 1.0;
			for (s=0;s<3;s++) if (sig&(0x1<<s)) dat_g_sig[s] = -1.0;

			Printf("set new (%d). %d, %d, %d sig a %d %d, %d sig g %d %d, %d\n\r", i_o, v1,v2,v3,
					(int)dat_a_sig[0],(int)dat_a_sig[1],(int)dat_a_sig[2], (int)dat_g_sig[0],(int)dat_g_sig[1],(int)dat_g_sig[2]);
		}
	}
}

void imu_loop(void){
	static uint64_t l_time_us = 0;

	if (status != 0 ){imu_init(); return ;}

	
	vector EstA, EstG;
	short t;
	if (MPU_GetDataFloat(EstA.value, EstG.value, &t) != HAL_OK ) {imu_init(); return;}

	uint64_t c_time_us = system_getTime_us();
	if (l_time_us  == 0 ) {l_time_us = c_time_us; return;} //first cycle. when haven't l_time.

	// Preparing gyro data..
	gyro = vector_addVector(vector_muxConst(EstG, lsb2dps_gyro), gyro_offs);
	imu_autoCalibrateByNoize(MotorControl_getState() == MOTOR_STATUS_LAUNCHED);	// calibrate gyro, if motor not launched, if necessary

	// Preparing acc data..	 convert "raw" data to "g" data and add offset
	acc = vector_addVector(vector_muxConst(EstA, lsb2g_acc), acc_offs);
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) imu_accCalibrate(); 	// calibrate acc, if motor not launched, if necessary

	// TODO heavy while launched motors
	// #define sq(x) ((x)*(x))
	// 
	// if (fabs(sqrt(sq(acc.axis.x)+sq(acc.axis.y)+sq(acc.axis.z))-1.0f)>0.1){	 //if heavy (more than 1g +-0.1g)
	// 	HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);
	// 	params_GetParamValue(PARAM_FL_KP_ARM);
	// }
	// else {
	// 	HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);
	// 	params_GetParamValue(PARAM_FL_KP);
	// }

	float dt = (c_time_us - l_time_us);
	float ki = params_GetParamValue(PARAM_P_KI);
	float kp;

	l_time_us = c_time_us;
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) kp = params_GetParamValue(PARAM_P_KP);
	else kp = params_GetParamValue(PARAM_P_KP_ARM);


	MahonyUpdateVariables(dt*1e-6f, kp, ki);

	// updating params for imu orientation
	static int dat_r[3] = {AXIS_X,AXIS_Y,AXIS_Z};	// orientation imu for rearranging
	static vector dat_a_sig = {.axis.x=1.0f,.axis.y=1.0f,.axis.z=1.0f};
	static vector dat_g_sig = {.axis.x=1.0f,.axis.y=1.0f,.axis.z=1.0f};
	UpdateOrientation(dat_r, dat_a_sig.value, dat_g_sig.value);

	// convert deg to rad, change sig and rearranging values from orientation
	vector g = vector_rearranging(vector_muxVector(vector_muxConst(gyro, M_PI/180.0f), dat_g_sig), dat_r);
	// change sig and rearranging values from orientation
	vector a = vector_rearranging(vector_muxVector(acc, dat_a_sig), dat_r);

	MahonyAHRSupdateIMU(g.axis.x, g.axis.y, g.axis.z, a.axis.x, a.axis.y, a.axis.z);
	MahonyGetAngles(&imuAngle.angle.pitch, &imuAngle.angle.roll, &imuAngle.angle.yaw);

	imuAngle = vector_muxConst(imuAngle, 180.0/M_PI); // convert rad to deg

	MotorControl_loop();
}

#define LIMIT 3000
#define DEF_UPDATE_MIN(a, b) {if (a.x>b.x) a.x = b.x; if (a.y>b.y) a.y = b.y; if (a.z>b.z) a.z = b.z;}
#define DEF_UPDATE_MAX(a, b) {if (a.x<b.x) a.x = b.x; if (a.y<b.y) a.y = b.y; if (a.z<b.z) a.z = b.z;}

void imu_autoCalibrateByNoize(int stop){
	static uint32_t l_time = 0;
	uint32_t time = system_getTime_ms();
	static vector average; 		//for get average data
	static vector min,max;
	static vector diff_last = {.axis.x=2*LIMIT,.axis.y=2*LIMIT,.axis.z=2*LIMIT};	// save the best result is in terms of noise
	//
	static int nStop = 1; 	// not to count the first time and set default values
	static uint32_t cnt = 0;	// count new data

	if (statusCalibrateGyro < -1) {	// reset calibrate
		statusCalibrateGyro=-1;
		diff_last = vector_setConst(diff_last, 2*LIMIT);
		l_time = time;
	}

	uint32_t timeWait = time-l_time;
	if (timeWait < 1000)//(GyroClbCnt < (1*gyro_freq))//1000)
	{
		//for (i=0;i<3;i++)GyroClbAcc_axis[i] += sVel_axis[i];
		DEF_UPDATE_MIN(min.axis, gyro.axis);
		DEF_UPDATE_MAX(max.axis, gyro.axis);

		average = vector_addVector(average, gyro);
		cnt++;
		nStop += stop;	// if calibration has been stopped
	}
	else
	{
		vector diff_res = {max.axis.x-min.axis.x, max.axis.y-min.axis.y, max.axis.z-min.axis.z};
		if ((cnt >=10) && 	// minimum count
				(nStop == 0) &&		// if there was no calibration stop
				((diff_res.axis.x<diff_last.axis.x) && (diff_res.axis.y<diff_last.axis.y) && (diff_res.axis.z<diff_last.axis.z))	// if the best result is in terms of noise
				){
			
			// gyro_offs -= average gyro data
			gyro_offs = vector_removeVector(gyro_offs, vector_divConst(average, (float)cnt));
			diff_last = diff_res;
			statusCalibrateGyro = 0;
		}
		// reset values:
		nStop = 0;
		cnt = 0;
		average = vector_setConst(average, 0);
		min = vector_setConst(min, LIMIT);
		max = vector_setConst(max, -LIMIT);
		l_time = time;
	}
}


void imu_accCalibrate(void){
	static uint32_t l_time = 0;
	static int start = 1;
	static vector average = {.axis.x=0,.axis.y=0,.axis.z=0};
	static uint32_t cnt = 0;

	// if no needs calibrate
	if (statusCalibrateAcc >= 0) {return;}

	uint32_t time = system_getTime_ms();
	if (start != 0){
		start=0;
		l_time = time;
		cnt = 0;
		average = vector_setConst(average, 0);
		acc_offs = vector_setConst(acc_offs, 0);	// reseting offset to get data without offset
		return;
	}

	uint32_t dt = time-l_time;
	if (time < l_time){dt = 0; l_time = time;}	//when the counter overflows
	if (dt < 1000)
	{
		average = vector_addVector(average, acc);
		cnt++;
	}
	else
	{
		if (cnt == 0) {statusCalibrateAcc = -1; start = 1; return;}	// for avoid div zerro error

		average = vector_divConst(average, (float)(cnt*1.0f));	// get the average value 
		vector offset_tmp = average;	// get the average value 

		float A_abs[3];
		for (int i=0;i<AXES_ALL;i++) A_abs[i] = fabs(offset_tmp.value[i]);

		int max_axis = AXIS_X;	// seeking main axis: (and remove 1g from the main axis)
		if (A_abs[AXIS_Y] > A_abs[max_axis]) max_axis = AXIS_Y;
		if (A_abs[AXIS_Z] > A_abs[max_axis]) max_axis = AXIS_Z;
		if (offset_tmp.value[max_axis]<0) offset_tmp.value[max_axis] += 1.0f;	// remove 1g
		else offset_tmp.value[max_axis] -= 1.0f;	// remove 1g

		acc_offs = vector_removeVector(acc_offs, offset_tmp);	// change result offset, by new data

		Printf("average c: %d, %d, %d\n\r", (int)(average.value[0]*1000), (int)(average.value[1]*1000), (int)(average.value[2]*1000));
		Printf("acc c: %d, %d, %d\n\r", (int)(acc_offs.axis.x*1000), (int)(acc_offs.axis.y*1000), (int)(acc_offs.axis.z*1000));
		statusCalibrateAcc = 0;
		start = 1;
		cnt=0;
	}
}


vector vector_setConst(vector axes, float Value){
	for (int i=0;i<AXES_ALL;i++) axes.value[i]=Value;
	return axes;
}
// vector vector_addConst(vector axes, float Value){
// 	for (int i=0;i<AXES_ALL;i++) axes.value[i]+=Value;
// 	return axes;
// }
vector vector_muxConst(vector axes, const float Value){
	for (int i=0;i<AXES_ALL;i++) axes.value[i]*=Value;
	return axes;
}
vector vector_divConst(vector axes, const float Value){
	for (int i=0;i<AXES_ALL;i++) axes.value[i]/=Value;
	return axes;
}
// vector vector_removeConst(vector axes, const float Value){
// 	for (int i=0;i<AXES_ALL;i++) axes.value[i]-=Value;
// 	return axes;
// }
vector vector_muxVector(vector axes, const vector axes2){
	for (int i=0;i<AXES_ALL;i++) axes.value[i] *= axes2.value[i];
	return axes;
}
vector vector_addVector(vector axes, vector axes2){
	for (int i=0;i<AXES_ALL;i++) axes.value[i]+=axes2.value[i];
	return axes;
}
vector vector_removeVector(vector axes, vector axes2){
	for (int i=0;i<AXES_ALL;i++) axes.value[i]-=axes2.value[i];
	return axes;
}
vector vector_rearranging(vector axes, int * val){
	vector res;
	for (int i=0;i<AXES_ALL;i++) if (val[i]<AXES_ALL)res.value[val[i]] = axes.value[i];
	return res;
}
