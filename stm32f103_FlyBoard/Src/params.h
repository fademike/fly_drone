

enum {
	PARAM_FLASH = 		0,
	PARAM_FL_KP = 		1,
	PARAM_ORIENTATION = 2,
	PARAM_PID_T_P = 	3,
	PARAM_PID_T_D = 	4,
	PARAM_PID_T_I = 	5,
	PARAM_MUX_T_CHAN = 	6,
	PARAM_PID_P_P = 	7,
	PARAM_PID_P_D = 	8,
	PARAM_PID_P_I = 	9,
	PARAM_MUX_P_CHAN = 	10,
	PARAM_PID_R_P = 	11,
	PARAM_PID_R_D = 	12,
	PARAM_PID_R_I = 	13,
	PARAM_MUX_R_CHAN = 	14,
	PARAM_PID_Y_P = 	15,
	PARAM_PID_Y_D = 	16,
	PARAM_PID_Y_I = 	17,
	PARAM_MUX_Y_CHAN = 	18,
	PARAM_FL_KP_ARM = 	19,
	PARAM_FL_KI = 		20,
	PARAM_M1_ACTION = 	21,
	PARAM_M1_MIN = 		22,
	PARAM_M1_MAX = 		23,
	PARAM_M1_MUX = 		24,
	PARAM_M1_INIT = 	25,
	PARAM_M1_OFFSET = 	26,
	PARAM_M2_ACTION = 	27,
	PARAM_M2_MIN = 		28,
	PARAM_M2_MAX = 		29,
	PARAM_M2_MUX = 		30,
	PARAM_M2_INIT = 	31,
	PARAM_M2_OFFSET = 	32,
	PARAM_M3_ACTION = 	33,
	PARAM_M3_MIN = 		34,
	PARAM_M3_MAX = 		35,
	PARAM_M3_MUX = 		36,
	PARAM_M3_INIT = 	37,
	PARAM_M3_OFFSET = 	38,
	PARAM_M4_ACTION = 	39,
	PARAM_M4_MIN = 		40,
	PARAM_M4_MAX = 		41,
	PARAM_M4_MUX = 		42,
	PARAM_M4_INIT = 	43,
	PARAM_M4_OFFSET = 	44,
};

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
int params_setValue(int n, union param_value value, int type);
int params_getValue(int n, union param_value * value, int type);
int params_getParam(int n, struct param_struct * param);
int params_getIndexById(char * id);

float params_GetMemValue(int n);

//int readFLASH(int *d);
//int writeFLASH(int * d);

