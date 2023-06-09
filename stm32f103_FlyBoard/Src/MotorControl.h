
#define MOTOR_BRUSHLESS 0
#define MOTOR_BRUSHED 1

#define MOTOR_TYPE MOTOR_BRUSHLESS//MOTOR_BRUSHLESS


enum {
	MOTOR_STATUS_FAIL = -1,
	MOTOR_STATUS_OK = 0,
	MOTOR_STATUS_READY = 1,
	MOTOR_STATUS_LAUNCHED = 2
};

void MotorControl_setArm(int a);
int MotorControl_isArmed(void);
int MotorControl_getState(void);

void MotorControl_init(void);
void MotorControl_loop(void);

