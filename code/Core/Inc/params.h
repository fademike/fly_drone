
// define for motor get params...
#define PARAM_MOTOR_LEN 4	//ACTION,MIN,MAX,..
#define PARAM_PID_LEN 5		//P,I,D,CHAN, OFFSET

// to access the parameters
enum {
	PARAM_FLASH = 		0,
	PARAM_P_KP,
	PARAM_ORIENTATION,
	
	PARAM_PID_T_P,
	PARAM_PID_T_D,
	PARAM_PID_T_I,
	PARAM_PID_T_MUX_CHAN,
	PARAM_PID_T_OFFSET,
	
	PARAM_PID_P_P,
	PARAM_PID_P_D,
	PARAM_PID_P_I,
	PARAM_PID_P_MUX_CHAN,
	PARAM_PID_P_OFFSET,

	PARAM_PID_R_P,
	PARAM_PID_R_D,
	PARAM_PID_R_I,
	PARAM_PID_R_MUX_CHAN,
	PARAM_PID_R_OFFSET,

	PARAM_PID_Y_P,
	PARAM_PID_Y_D,
	PARAM_PID_Y_I,
	PARAM_PID_Y_MUX_CHAN,
	PARAM_PID_Y_OFFSET,

	PARAM_P_KP_ARM,
	PARAM_P_KI,

	PARAM_M1_ACTION,
	PARAM_M1_MIN,
	PARAM_M1_MAX,
	// PARAM_M1_MUX,
	// PARAM_M1_INIT,
	PARAM_M1_OFFSET,

	PARAM_M2_ACTION,
	PARAM_M2_MIN,
	PARAM_M2_MAX,
	// PARAM_M2_MUX,
	// PARAM_M2_INIT,
	PARAM_M2_OFFSET,

	PARAM_M3_ACTION,
	PARAM_M3_MIN,
	PARAM_M3_MAX,
	// PARAM_M3_MUX,
	// PARAM_M3_INIT,
	PARAM_M3_OFFSET,

	PARAM_M4_ACTION,
	PARAM_M4_MIN,
	PARAM_M4_MAX,
	// PARAM_M4_MUX,
	// PARAM_M4_INIT,
	PARAM_M4_OFFSET,

	PARAM_ALT_MAX,

	PARAM_P_MODE,

	PARAM_ALL,
};

// calibrated data in flash
enum {
	MEM_CLB_AX = 200,
	MEM_CLB_AY,
	MEM_CLB_AZ,
};

#define IS_SET_BIT(x, i) (((i)&(0x1<<x)) != 0)
#define P_BIT_IMU_NOT_USE 		0		// Do not use angels from imu if set
#define P_BIT_WITHOUT_RESET 	1		// If not set - reset motors, pids to default, when throttle equal 0 

union param_value{
	float FLOAT;
	int INT;
};

struct param_struct {
	char param_id[16];
	union param_value param_value;
	char param_type;
};

void params_save(void);
void params_restore(void);

int params_getSize(void);
int params_setValue(unsigned int n, union param_value value, int type);
int params_getValue(unsigned int n, union param_value * value, int type);
int params_getParam(unsigned int n, struct param_struct * param);
int params_getIndexById(char * id);

float params_GetParamValue(unsigned int n);
int params_SetParamValue(unsigned int n, float value);

//int readFLASH(int *d);
//int writeFLASH(int * d);
void Clear_Bootloader_Key(void);

