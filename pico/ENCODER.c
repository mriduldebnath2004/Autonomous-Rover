#include "ENCODER.h"
#include "PINS.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"

#define COUNT_TO_M 0.0000620149f //experimentally found this value testing all 4 motors over over many cycles and averaging

static volatile int32_t motor_counts[4] = {0};
static volatile uint8_t prev_state[4] = {0};

// left side motors(1,2) forward and right side's forward are opposite signs so reversing them back here
static const int8_t encoder_sign[4] = {
    +1,  // M1
    -1,  // M2
    +1,  // M3
    -1   // M4
};


static const int8_t enc_lookup[4][4] = { //Quadrature Encoder look up table to determine direction while adding/subtracting counts
    { 0, +1, -1,  0},
    {-1,  0,  0, +1},
    {+1,  0,  0, -1},
    { 0, -1, +1,  0}
};

static void update_encoder(uint8_t motor, uint pinA, uint pinB) { //updating each encoder count
    uint8_t A = gpio_get(pinA);
    uint8_t B = gpio_get(pinB);

    uint8_t current = (A << 1) | B;

    int8_t delta = enc_lookup[prev_state[motor]][current];

     motor_counts[motor] += encoder_sign[motor] * delta;

    prev_state[motor] = current;
}

static void encoder_interrupt(uint gpio, uint32_t events) { //updating individual motor counts depending on pin interrupt
    (void)events;
    if (gpio == ENC1A || gpio == ENC1B) {
        update_encoder(0, ENC1A, ENC1B);
    }
    else if (gpio == ENC2A || gpio == ENC2B) {
        update_encoder(1, ENC2A, ENC2B);
    }
    else if (gpio == ENC3A || gpio == ENC3B) {
        update_encoder(2, ENC3A, ENC3B);
    }
    else if (gpio == ENC4A || gpio == ENC4B) {
        update_encoder(3, ENC4A, ENC4B);
    }
}

static void init_encoder_state(uint8_t motor, uint pinA, uint pinB) { //find starting states of each motor 
    uint8_t A = gpio_get(pinA);
    uint8_t B = gpio_get(pinB);

    prev_state[motor] = (A << 1) | B;
}


void encoder_init(void) {

    gpio_init(ENC1A);
    gpio_set_dir(ENC1A, GPIO_IN);

    gpio_init(ENC1B);
    gpio_set_dir(ENC1B, GPIO_IN);

    gpio_init(ENC2A);
    gpio_set_dir(ENC2A, GPIO_IN);

    gpio_init(ENC2B);
    gpio_set_dir(ENC2B, GPIO_IN);

    gpio_init(ENC3A);
    gpio_set_dir(ENC3A, GPIO_IN);

    gpio_init(ENC3B);
    gpio_set_dir(ENC3B, GPIO_IN);

    gpio_init(ENC4A);
    gpio_set_dir(ENC4A, GPIO_IN);

    gpio_init(ENC4B);
    gpio_set_dir(ENC4B, GPIO_IN);

    // find the starting state of the encoders
    init_encoder_state(0, ENC1A, ENC1B);
    init_encoder_state(1, ENC2A, ENC2B);
    init_encoder_state(2, ENC3A, ENC3B);
    init_encoder_state(3, ENC4A, ENC4B);

    gpio_set_irq_enabled_with_callback( //enable high rise interrupt 
        ENC1A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_interrupt
    );

    gpio_set_irq_enabled(ENC1B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC2A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC2B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC3A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC3B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC4A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC4B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);




}

int32_t encoder_get_count(uint8_t motor) {
    if (motor > 3) return 0;

    uint32_t save = save_and_disable_interrupts();
    int32_t count = motor_counts[motor];
    restore_interrupts(save);

    return count;
}

void encoder_reset_counts(void) {
    uint32_t save = save_and_disable_interrupts();

    for (int i = 0; i < 4; i++) {
        motor_counts[i] = 0;
    }

    restore_interrupts(save);
}