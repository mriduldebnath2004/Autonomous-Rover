#include "COMMUNICATION.h"
#include "PINS.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "ENCODER.h"
#include "IMU.h"

#include <stdio.h>
#include <stdint.h>

#define BAUD_RATE 115200

#define COMM_TX_PERIOD_MS 20   // 50 Hz
#define COMM_TIMEOUT_MS 500 

static absolute_time_t tx_next_time;
static absolute_time_t last_rx_time;

#define UART_ID uart0
#define RX_BUFFER_SIZE 64

static char rx_buffer[RX_BUFFER_SIZE];
static int rx_index = 0;

static float forward_mps = 0.0f;
static float yaw_rate_radps = 0.0f;

void communication_init(void)
{
    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    rx_index = 0;
    forward_mps = 0.0f;
    yaw_rate_radps = 0.0f;

    tx_next_time = delayed_by_ms(get_absolute_time(), COMM_TX_PERIOD_MS);
    last_rx_time = get_absolute_time();
}

void communication_receive(void)
{
    while (uart_is_readable(UART_ID))
    {
        char c = uart_getc(UART_ID);

        if (c == '\n')
        {
            rx_buffer[rx_index] = '\0';

            float v = 0.0f;
            float w = 0.0f;

            if (sscanf(rx_buffer, "V %f %f", &v, &w) == 2)
            {
                forward_mps = v;
                yaw_rate_radps = w;
                last_rx_time = get_absolute_time();
            }

            rx_index = 0;
        }
        else
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = c;
            }
            else
            {
                rx_index = 0; // overflow protection
            }
        }
    }

    if (absolute_time_diff_us(last_rx_time, get_absolute_time()) > COMM_TIMEOUT_MS * 1000)
    {
    forward_mps = 0.0f;
    yaw_rate_radps = 0.0f;
    }
}


void communication_transmit(void)
{
    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, tx_next_time) > 0)
        return;

    tx_next_time = delayed_by_ms(tx_next_time, COMM_TX_PERIOD_MS); //transmits at 50hz


    float imu[6];
    read_imu(imu);

    int32_t e0 = encoder_get_count(0);
    int32_t e1 = encoder_get_count(1);
    int32_t e2 = encoder_get_count(2);
    int32_t e3 = encoder_get_count(3);

    char buffer[128];

    snprintf(buffer, sizeof(buffer),
            "T %d %d %d %d %.3f %.3f %.3f %.3f %.3f %.3f\n",
            e0, e1, e2, e3,
            imu[0], imu[1], imu[2],
            imu[3], imu[4], imu[5]);

    uart_puts(UART_ID, buffer);
}


float communication_get_forward_mps(void)
{
    return forward_mps;
}

float communication_get_yaw_rate_radps(void)
{
    return yaw_rate_radps;
}