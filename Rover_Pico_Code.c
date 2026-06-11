#include <stdio.h>
#include "pico/stdlib.h"
#include "PINS.h"
#include "ENCODER.h"
#include "MOTOR.h"

#define MIN_PWM 82
#define count_to_m 0.0000620149f


int main() {
    stdio_init_all();

    motor_init();
    encoder_init();

    while (true) {
        sleep_ms(5000);
        
        motor_set_left(true,85);
        motor_set_right(true,85);
        for (int i = 0; i < 50; i++) {
            printf(
                "Encoder left side count: %.1f  Encoder right side count: %.1f\n",
                (encoder_get_count(0) + encoder_get_count(2)) * 0.5f,
                (encoder_get_count(1) + encoder_get_count(3)) * 0.5f
            );
            sleep_ms(100);
        }
        motor_stop_all();
        encoder_reset_counts();
    }
}
