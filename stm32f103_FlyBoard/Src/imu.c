

#include "stm32f1xx.h"

#include <math.h>
#include "MPU9250.h"
#include "MahonyAHRS.h"
#include "system.h"
#include "params.h"
#include "MotorControl.h"

void imu_autoCalibrateByNoize(int reset);
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

struct imuAngle_struct imuAngle = {0,0,0};

float imu_getPitch(void){return imuAngle.pitch;}
float imu_getRoll(void){return imuAngle.roll;}
float imu_getYaw(void){return imuAngle.yaw;}

float atan2_approx(float y, float x);

pos_struct EstA, EstG;
short t;
//int MPU_GetData(pos_struct * EstA, pos_struct * EstG, short * t);


struct axis_struct gyro = {0,0,0};
struct axis_struct gyro_ang = {0,0,0};
struct axis_struct gyro_offs = {0,0,0};

struct axis_struct acc = {0,0,0};
struct axis_struct acc_offs = {0,0,0};

static int status =-1;
static int statusCalibrateGyro = -1;
static int statusCalibrateAcc = 0;

int imu_getStatus(void){	// fail, calib, ok
	return status;
}
int imu_GyroCalibrate_getStatus(void){	// fail, calib, ok
	return statusCalibrateGyro;
}
int imu_AccCalibrate_getStatus(void){	// fail, calib, ok
	return statusCalibrateAcc;
}
void imu_AccCalibrate_run(void){	// fail, calib, ok
	statusCalibrateAcc = -1;
}
//void imu_AccOffset_get(int GYROorACC, float * ox, float * oy, float * oz){
void imu_AccOffset_get(float * ox, float * oy, float * oz){
	*ox = acc_offs.x;
	*oy = acc_offs.y;
	*oz = acc_offs.z;
}
//void imu_AccOffset_set(int GYROorACC, float ox, float oy, float oz){
void imu_AccOffset_set(float ox, float oy, float oz){
	Printf("acc set %d, %d, %d\n\r", (int)(ox*1000.0f), (int)(oy*1000.0f), (int)(oz*1000.0f));
	acc_offs.x = ox;
	acc_offs.y = oy;
	acc_offs.z = oz;
}
int imu_init(void){
	status = MPU_Init();
	Printf("imu init %d\n\r", status);
	return status;
}

#include "fast_atan.h"
#include "main.h"
void imu_loop(void){
	if (status != 0 ){imu_init(); return ;}
	int ret = MPU_GetData(&EstA, &EstG, &t);
	if (ret<0) {imu_init(); return ;}
	//int ch = MPU_check();

	static uint64_t l_time_us = 0;
	uint64_t c_time_us = system_getTime_us();

	float dt = (c_time_us - l_time_us);
	float kp = params_GetMemValue(PARAM_FL_KP);//params_Get_f_kp();
	float ki = params_GetMemValue(PARAM_FL_KI);//params_Get_f_ki();
	float q[4];	//quat



	if (l_time_us  == 0 ) {l_time_us = c_time_us; return;} //first cycle. when haven't l_time.
	l_time_us = c_time_us;



	gyro.x = ((float)EstG.x)*lsb2dps_gyro + gyro_offs.x;//lsb2dps_gyro;//*M_PI/180.0f;
	gyro.y = ((float)EstG.y)*lsb2dps_gyro + gyro_offs.y;//lsb2dps_gyro;//*M_PI/180.0f;
	gyro.z = ((float)EstG.z)*lsb2dps_gyro + gyro_offs.z;//lsb2dps_gyro;//*M_PI/180.0f;


	imu_autoCalibrateByNoize(MotorControl_getState() == MOTOR_STATUS_LAUNCHED);	// if drone is relax..
	//statusCalibrateGyro=0;

//	acc_offs.x=0;
//	acc_offs.y=0;
//	acc_offs.z=0;

	acc.x = ((float)EstA.x)*1.0f*lsb2g_acc + acc_offs.x;//*16.0f/32767.0f;//*lsb2g_acc;
	acc.y = ((float)EstA.y)*1.0f*lsb2g_acc + acc_offs.y;//*16.0f/32767.0f;//*lsb2g_acc;
	acc.z = ((float)EstA.z)*1.0f*lsb2g_acc + acc_offs.z;//*16.0f/32767.0f;//*lsb2g_acc;

	//if (MotorControl_isArmed() == 0) imu_accCalibrate();
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) imu_accCalibrate();

	if (statusCalibrateGyro < 0) { // TODO need test.. maybe
		imuAngle.pitch = 0;
		imuAngle.roll = 0;
		imuAngle.yaw = 0;
		MotorControl_loop();
		return;
	}

	//static int divi = 0;
	//if (++divi>=50){divi=0;
	//Printf("%d,%d,%d\n\r", (int)(acc_offs.x*100.0f), (int)(acc_offs.y*100.0f), (int)(acc_offs.z*100.0f));
	//Printf("%d,%d,%d\n\r", (int)(acc.x*100.0f), (int)(acc.y*100.0f), (int)(acc.z*100.0f));
	//Printf("%d,%d,%d\n\r", (int)(EstA.x/1.0f), (int)(EstA.y/1.0f), (int)(EstA.z/1.0f));
	//}
	//HAL_GPIO_WritePin(PIN_TEST3_GPIO_Port, PIN_TEST3_Pin, GPIO_PIN_SET);

	//if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) kp *= 10.0f;
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) kp = params_GetMemValue(PARAM_FL_KP);
	else kp = params_GetMemValue(PARAM_FL_KP_ARM);
	//#define sq(x) ((x)*(x))
	// * sq(1.0f/256);
	static int recoveryTime = 0;
	if (fabs(fabs(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z)-1.0f)>0.2){
	//if (fabs(sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z)-1.0f)>0.2){
	//if (fabs(sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z)-1.0f)>0.2){
		HAL_GPIO_WritePin(PIN_TEST3_GPIO_Port, PIN_TEST3_Pin, GPIO_PIN_SET);
		//recoveryTime = 10;
		kp=0;
	}
	else {
		HAL_GPIO_WritePin(PIN_TEST3_GPIO_Port, PIN_TEST3_Pin, GPIO_PIN_RESET);
		//recoveryTime = (recoveryTime>0)?recoveryTime-1:0;
	}
	//if (recoveryTime > 0) kp=0;

	MahonyUpdateVariables(dt*1e-6f, kp, ki);
	MahonyAHRSupdateIMU(gyro.x*M_PI/180.0f,gyro.y*M_PI/180.0f,gyro.z*M_PI/180.0f,acc.x,acc.y,acc.z);
	MahonyGetQuat(q);

	const float px = 2 * (q[1]*q[3] - q[0]*q[2]);
	const float py = 2 * (q[0]*q[1] + q[2]*q[3]);
	const float pz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	//simple | without sqrt|
//#define ATAN atan2approx	//199	|	149
//#define ATAN atan2_approx	//201	|	160
//#define ATAN atan2PI_4	//186	|	137
//#define ATAN atan2_approx
//#define ATAN atan2LUT

	float yaw = atan2approx(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
	float roll = atan2approx(px, sqrt(py*py + pz*pz))  * 180/M_PI;			// sqrt(py*py + pz*pz))
	float pitch = atan2approx(py, sqrt(px*px + pz*pz))  * 180/M_PI;			// FIXME MAYBE DEL SQRT

	imuAngle.pitch = pitch;
	imuAngle.roll = -roll;
	//imuAngle.yaw = yaw;
	static float yaw_free = 0;
	if (MotorControl_getState() != MOTOR_STATUS_LAUNCHED) {yaw_free = yaw;}
	imuAngle.yaw = yaw - yaw_free;
	if (imuAngle.yaw<-180)imuAngle.yaw += 360.0f;
	else if (imuAngle.yaw>180)imuAngle.yaw -= 360.0f;

	MotorControl_loop();
}

#define def_update_min(a, b) if (1){if (a.x>b.x) a.x = b.x; if (a.y>b.y) a.y = b.y; if (a.z>b.z) a.z = b.z;}
#define def_update_max(a, b) if (1){if (a.x<b.x) a.x = b.x; if (a.y<b.y) a.y = b.y; if (a.z<b.z) a.z = b.z;}

void imu_autoCalibrateByNoize(int reset){
	static uint32_t l_time = 0;
	uint32_t time = system_getTime_ms();
	static struct axis_struct GyroClbAcc_axis = {0,0,0};
	static struct axis_struct min = {3000,3000,3000}, max = {-3000,-3000,-3000};
	static struct axis_struct diff_last = {6000,6000,6000};
	//
	static int nReset = 0;

	static uint32_t GyroClbCnt = 0;
	uint32_t timeWait = time-l_time;
	if ((0 <= timeWait) && (timeWait < 1000))//(GyroClbCnt < (1*gyro_freq))//1000)
	{
		//for (i=0;i<3;i++)GyroClbAcc_axis[i] += sVel_axis[i];
		def_update_min(min, gyro);
		def_update_max(max, gyro);

		GyroClbAcc_axis.x +=gyro.x;
		GyroClbAcc_axis.y +=gyro.y;
		GyroClbAcc_axis.z +=gyro.z;
		GyroClbCnt++;
		nReset += reset;
	}
	else
	{
		struct axis_struct diff_res = {max.x-min.x, max.y-min.y, max.z-min.z};
		if ((GyroClbCnt >=10) && (nReset == 0) &&
				((diff_res.x<diff_last.x) && (diff_res.y<diff_last.y) && (diff_res.z<diff_last.z))){
			gyro_offs.x -= GyroClbAcc_axis.x/(float)GyroClbCnt;
			gyro_offs.y -= GyroClbAcc_axis.y/(float)GyroClbCnt;
			gyro_offs.z -= GyroClbAcc_axis.z/(float)GyroClbCnt;

			diff_last.x = diff_res.x;
			diff_last.y = diff_res.y;
			diff_last.z = diff_res.z;

			gyro_ang.x=0;
			gyro_ang.y=0;
			gyro_ang.z=0;

			statusCalibrateGyro = 0;
		}
		nReset = 0;

		GyroClbCnt = 0;
		GyroClbAcc_axis.x = 0;
		GyroClbAcc_axis.y = 0;
		GyroClbAcc_axis.z = 0;

		min.x = 3000;
		min.y = 3000;
		min.z = 3000;
		max.x = -3000;
		max.y = -3000;
		max.z = -3000;

		l_time = time;
	}


}


void imu_accCalibrate(void){
	static uint32_t l_time = 0;

	static int start = 1;
	uint32_t time = system_getTime_ms();
	static struct axis_struct AccClbAcc_axis = {0,0,0};
	//
	static uint32_t GyroClbCnt = 0;

	// if no needs calibrate
	if (statusCalibrateAcc >= 0) {l_time = time; start = 1; return;}

	if (start != 0){
		start=0;
		GyroClbCnt = 0;
		AccClbAcc_axis.x = 0;
		AccClbAcc_axis.y = 0;
		AccClbAcc_axis.z = 0;
	}

	uint32_t timeWait = time-l_time;
	if ((0 <= timeWait) && (timeWait < 1000))//(GyroClbCnt < (1*gyro_freq))//1000)
	{

		AccClbAcc_axis.x +=acc.x;
		AccClbAcc_axis.y +=acc.y;
		AccClbAcc_axis.z +=acc.z;
		GyroClbCnt++;
	}
	else
	{
		acc_offs.x -= AccClbAcc_axis.x/(float)GyroClbCnt;
		acc_offs.y -= AccClbAcc_axis.y/(float)GyroClbCnt;
		acc_offs.z -= (AccClbAcc_axis.z/(float)GyroClbCnt) - 1.0f;

		statusCalibrateAcc = 0;

		GyroClbCnt = 0;
		AccClbAcc_axis.x = 0;
		AccClbAcc_axis.y = 0;
		AccClbAcc_axis.z = 0;

		l_time = time;
	}


}

#define M_PIf       3.14159265358979323846f

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}


