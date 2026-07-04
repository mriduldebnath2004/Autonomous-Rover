#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <stdbool.h>

void communication_init(void);

void communication_receive(void);
void communication_transmit(void);

bool communication_is_connected(void);

float communication_get_forward_mps(void);
float communication_get_yaw_rate_radps(void);

#endif