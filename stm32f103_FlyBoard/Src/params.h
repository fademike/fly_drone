

enum {
	PARAM_FLASH = 0,
	PARAM_FL_KP = 1,
	PARAM_FL_KI = 2,
	PARAM_PID_PR_P = 3,
	PARAM_PID_PR_D = 4,
	PARAM_PID_PR_I = 5,
	PARAM_MUX_PR_CHAN = 6,
	PARAM_MUX_Y_CHAN = 7,
	PARAM_PID_Y_P = 8,
	PARAM_PID_Y_D = 9,
	PARAM_PID_Y_I = 10,
	PARAM_FL_KP_ARM = 11
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

