

#include "mavlink.h"
#include "main.h"	//for Printf
#include "params.h"
#include "imu.h"
#include "ModemControl.h"
#include "system.h"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

#define MAV_DEBUG 0

uint8_t target_system;
uint8_t target_component;

uint16_t chan[10];

uint32_t chan_UpdateTime_ms = 0;

uint16_t mavlink_getChan(int n){
	return chan[n];
}
uint32_t mavlink_getChanUpdateTimer(void){	// Get delay after the last reception of Chan.
	uint32_t delay = system_getTime_ms() - chan_UpdateTime_ms;
	return delay;
}

void mavlink_send_msg(mavlink_message_t * msg){
	int i;
	uint8_t buf[BUFFER_LENGTH];
	int len = mavlink_msg_to_send_buffer(buf, msg);

	if (ModemControl_getStatus < 0) return;
	for (i=0; i<len;i++) ModemControl_SendSymbol(buf[i]);
	//Printf("send mav\n\r");
}


void mavlink_send_param(int n){
	struct param_struct p;
	int find_fl = 0;
	if (params_getParam(n, &p) == 0) find_fl = 1;	// if param exist

	mavlink_message_t msg;
	// if param exist - set param to msg
	if (find_fl) {
		float fvalue = 0;
		if ((p.param_type == MAV_PARAM_TYPE_REAL32) || p.param_type == MAV_PARAM_TYPE_REAL64)
			fvalue = p.param_value.FLOAT;
		else fvalue = (float)p.param_value.INT;

		mavlink_msg_param_value_pack(1, 200, &msg, p.param_id, fvalue, p.param_type, params_getSize(), n);}
	// if param none - set none param
	else mavlink_msg_param_value_pack(1, 200, &msg, NULL, 0, MAV_PARAM_TYPE_REAL32, params_getSize(), -1);

	mavlink_send_msg(&msg);

	// for debug
	if (find_fl == 1) {if (MAV_DEBUG) Printf("param send ans %d: %d\n\r", n, (int)p.param_value.FLOAT);}
	else if (MAV_DEBUG) Printf("param send ans -1\n\r");
}

void mavlink_send_heartbeat(void){
	mavlink_message_t msg;

	uint8_t system_status = MAV_STATE_BOOT;
	if (imu_getStatus >= 0) {
		system_status = MAV_STATE_CALIBRATING;
		if (imu_GyroCalibrate_getStatus() >= 0) {
			system_status = MAV_STATE_STANDBY;
			if (MotorControl_isArmed()) system_status = MAV_STATE_ACTIVE;
		}
	}
	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_INVALID, base_mode, 0, system_status);
	mavlink_send_msg(&msg);
}

void mavlink_send_status(void){
	mavlink_message_t msg;
	int voltage = Battery_getVoltage();
	int bat = Battery_getBatPercent();
	mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, voltage, -1, bat, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	mavlink_send_msg(&msg);
}


void mavlink_send_attitude(void){
	mavlink_message_t msg;
	int timer_us = system_getTime_ms()*1000;

	float pitch = imu_getPitch()*M_PI/180.0f;
	float roll = imu_getRoll()*M_PI/180.0f;
	float yaw = imu_getYaw()*M_PI/180.0f;

	mavlink_msg_attitude_pack(1, 200, &msg, timer_us, roll, pitch, yaw, 0, 0, 0);
	mavlink_send_msg(&msg);
}

void mavlink_send_battery_status(void){
	mavlink_message_t msg;
	uint16_t voltages[1] = {4200};
	mavlink_msg_battery_status_pack(1, 200, &msg, 0, 0, 0, 0, voltages, 0, 0, 0, 45, 0, 0, 0, 0, 0);
	mavlink_send_msg(&msg);
}

void mavlink_send_time(void){
	mavlink_message_t msg;
    static uint64_t time_us=1675674266;
    static uint32_t time_ms=0;
    time_ms+=1000; time_us+=1;
    mavlink_msg_system_time_pack(1, 200, &msg, time_us, time_ms);
	mavlink_send_msg(&msg);
}

// result MAV_RESULT_ACCEPTED
void mavlink_send_cmd_ack(uint16_t command, uint8_t result, uint8_t progress){
	mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, 200, &msg, command, result, progress, 0, target_system, target_component);
	mavlink_send_msg(&msg);
}

void mavlink_receive(char rxdata){

	mavlink_message_t msg;

	float t_param[10];
	uint16_t t_cmd;

	mavlink_status_t status;

	if (mavlink_parse_char(MAVLINK_COMM_0, rxdata, &msg, &status))
	{
		// Packet received
		switch (msg.msgid){
			case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
				chan[0] = mavlink_msg_rc_channels_override_get_chan1_raw(&msg);
				chan[1] = mavlink_msg_rc_channels_override_get_chan2_raw(&msg);
				chan[2] = mavlink_msg_rc_channels_override_get_chan3_raw(&msg);
				chan[3] = mavlink_msg_rc_channels_override_get_chan4_raw(&msg);
				chan[4] = mavlink_msg_rc_channels_override_get_chan5_raw(&msg);

				chan_UpdateTime_ms = system_getTime_ms();
				//if (MAV_DEBUG) Printf("ch: %d, %d, %d, %d, %d, \n\r", chan[0], chan[1], chan[2], chan[3], chan[4]);
				break;
			case MAVLINK_MSG_ID_PARAM_SET:
			{
				mavlink_param_set_t param_set;
				char this_param_id[16];

				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);
				uint16_t param_id_len = mavlink_msg_param_set_get_param_id(&msg, this_param_id);	// name of rx param
				this_param_id[strlen(this_param_id)-1] = '\0';										// correction name
				float this_param_value = mavlink_msg_param_set_get_param_value(&msg);
				uint8_t param_type = 0;
				// if (msg.magic != MAVLINK_STX_MAVLINK1)
				param_type = _MAV_RETURN_uint8_t(&msg,  msg.len-1);

				if (param_type == MAV_PARAM_TYPE_REAL32){
					if (MAV_DEBUG) Printf("set param REAL32: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
				}
				else {
					if (MAV_DEBUG) Printf("set param nan: %s, %d, %f\n\r", this_param_id, param_type, this_param_value);
				}

				int pos = params_getIndexById(this_param_id);
				union param_value pv = {.FLOAT=this_param_value};	//FIXME!!!!
				if (pos != 0) params_setValue(pos, pv, param_type);
				mavlink_send_param(pos);

				if (MAV_DEBUG) Printf("param set %d: %d\n\r", pos, (int)this_param_value);
				break;
			}
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			{
				mavlink_param_set_t param_set;
				char this_param_id[16];


				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);
				uint16_t param_id_len = mavlink_msg_param_request_read_get_param_id(&msg, this_param_id);	// name of rx param
				this_param_id[strlen(this_param_id)-1] = '\0';										// correction name
				int16_t this_index = mavlink_msg_param_request_read_get_param_index(&msg);

				int pos = params_getIndexById(this_param_id);
				if (pos != 0) mavlink_send_param(pos);
				else  mavlink_send_param(this_index);

				if (MAV_DEBUG)Printf("reuest read \n\r");
				break;
			}
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
			{
				if (MAV_DEBUG) Printf("reuest list\n\r");

				target_system = mavlink_msg_param_set_get_target_system(&msg);
				target_component = mavlink_msg_param_set_get_target_component(&msg);

				int p;
				for (p=1;p<=params_getSize(); p++){
					mavlink_send_param(p);
					if (MAV_DEBUG) Printf("reuest read ans %d\n\r", p);
				}
				break;
			}
			case MAVLINK_MSG_ID_HEARTBEAT:
			{
				mavlink_send_heartbeat();
				if (MAV_DEBUG) Printf("rx HEARTBEAT\n\r");
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_LONG:
			{
				uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
				switch(cmd){
					case(MAV_CMD_COMPONENT_ARM_DISARM):
						t_param[1] = mavlink_msg_command_long_get_param1(&msg);
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						if (MAV_DEBUG) Printf("rx cmd ARM_DISARM %d\n\r", (int)t_param[1]);
						MotorControl_setArm((int)t_param[1]);
						break;
					case(MAV_CMD_PREFLIGHT_CALIBRATION):
						t_param[1] = mavlink_msg_command_long_get_param1(&msg);
						t_param[5] = mavlink_msg_command_long_get_param5(&msg);
						mavlink_send_cmd_ack(cmd, MAV_RESULT_ACCEPTED, 100);
						if (MAV_DEBUG) Printf("rx calibrate\n\r");
						if (t_param[1] != 0)if (MAV_DEBUG) Printf("rx calibrate gyro %d\n\r", (int)t_param[1]);
						if (t_param[5] != 0)if (MAV_DEBUG) Printf("rx calibrate acc %d\n\r", (int)t_param[5]);

						if (t_param[5] != 0) imu_AccCalibrate_run();

						break;
				}
				break;
			}
			default:
				if (MAV_DEBUG) Printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
		}
	}

}
