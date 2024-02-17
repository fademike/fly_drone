

uint16_t mavlink_getChan(int n);
uint32_t mavlink_getChanUpdateTimer(void);

void mavlink_send_statustext(char * text);

void mavlink_loop(void);

void mavlink_receive(char rxdata);


void mavlink_send_status(void);
void mavlink_send_attitude(void);
void mavlink_send_battery_status(void);
void mavlink_send_time(void);

