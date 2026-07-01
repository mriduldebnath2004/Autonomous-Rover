#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H

void yawrate_init(void);
void yawrate_set_target(float target_rad_s);
void yawrate_update(float forward_mps);

#endif