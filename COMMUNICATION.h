#ifndef COMMUNICATION_H
#define COMMUNICATION_H

void communication_init(void);

void communication_receive(void);
void communication_transmit(void);

float communication_get_forward_mps(void);
float communication_get_yaw_rate_radps(void);

#endif