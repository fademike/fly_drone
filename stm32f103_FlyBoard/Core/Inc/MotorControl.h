

#define TIM_PERIOD 10000
#define TIM_PRESCALER 63 //0

#define MOTOR_LIMIT_BRUSHLESS 1000
#define MOTOR_LIMIT_BRUSHED 10000

enum {
	MOTOR_STATUS_FAIL = -1,
	MOTOR_STATUS_OK = 0,
	MOTOR_STATUS_READY = 1,
	MOTOR_STATUS_LAUNCHED = 2
};

void MotorControl_setArm(int a);
int MotorControl_isArmed(void);
int MotorControl_getState(void);
float MotorControl_getMotorValue(float * m);

void MotorControl_init(void);
void MotorControl_loop(void);

