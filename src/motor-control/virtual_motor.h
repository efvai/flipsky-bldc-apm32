#ifndef VIRTUAL_MOTOR_H_
#define VIRTUAL_MOTOR_H_

#include "datatypes.h"

void virtual_motor_init(volatile mc_configuration *conf);
void virtual_motor_set_configuration(volatile mc_configuration *conf);
void virtual_motor_int_handler(float v_alpha, float v_beta);
void connect_virtual_motor(float ml, float J, float Vbus);
void disconnect_virtual_motor(void);
bool virtual_motor_is_connected(void);
float virtual_motor_get_angle_deg(void);
#endif /* VIRTUAL_MOTOR_H_ */