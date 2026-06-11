#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);

int32_t encoder_get_count(uint8_t motor);

void encoder_reset_counts(void);

#endif