
enum {
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,
	AXES_ALL = 3,
};
enum {
	THROTTLE = 0,
	PITCH = 1,
	ROLL = 2,
	YAW = 3
};
//enum {
//	ANGEL_PITCH = 0,
//	ANGEL_ROLL = 1,
//	ANGEL_YAW = 2
//};

int imu_getStatus(void);
int imu_GyroCalibrate_getStatus(void);
void imu_GyroCalibrate_run(void);

int imu_AccCalibrate_getStatus(void);
void imu_AccCalibrate_run(void);

void imu_AccOffset_get(float * ox, float * oy, float * oz);
void imu_AccOffset_set(float ox, float oy, float oz);

int imu_init(void);
void imu_loop(void);
int imu_getStatus(void);

float imu_getPitch(void);
float imu_getRoll(void);
float imu_getYaw(void);
int imu_getAlt(void);

