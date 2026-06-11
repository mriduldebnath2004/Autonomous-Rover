#include "MOTOR.h"
#include "hardware/pwm.h"
#include "pins.h"
#include "pico/stdlib.h"

static void setup_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

void motor_init(void) {
    gpio_init(DIR1);
    gpio_set_dir(DIR1, GPIO_OUT);

    gpio_init(DIR2);
    gpio_set_dir(DIR2, GPIO_OUT);

    setup_pwm(PWM1);
    setup_pwm(PWM2);
}

static void set_motor(uint dir_pin, uint pwm_pin, bool forward, uint8_t speed) {
    gpio_put(dir_pin, forward ? 1 : 0);

    uint slice = pwm_gpio_to_slice_num(pwm_pin);
    uint channel = pwm_gpio_to_channel(pwm_pin);

    pwm_set_chan_level(slice, channel, speed);
}

void motor_set_left(bool forward, uint8_t speed) {
    set_motor(DIR1, PWM1, forward, speed);
}

void motor_set_right(bool forward, uint8_t speed) {
    set_motor(DIR2, PWM2, forward, speed);
}

void motor_stop_all(void) {
    set_motor(DIR1, PWM1, true, 0);
    set_motor(DIR2, PWM2, true, 0);
}

