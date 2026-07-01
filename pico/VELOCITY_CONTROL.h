#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

void velocity_init(void);
void velocity_set_target(float left_mps, float right_mps);
void velocity_update(void);
float velocity_get_left(void);
float velocity_get_right(void);
float velocity_difference(void);

#endif