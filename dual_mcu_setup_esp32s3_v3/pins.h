#pragma once

#define BUZZER GPIO_NUM_47
#define COMM_UART_RX 18
#define COMM_UART_TX 17

#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9

#define BATTERY_PIN GPIO_NUM_4
#define BATTERY_FULL_VOLT 12.6

#define DIST_SEN_NUMB 4

#define IO_PIN_1 1
#define IO_PIN_0 0
// motor_2 rotation direction logic
#define IO_PIN_3 3
#define IO_PIN_2 2

// motor_3 rotation direction logic
#define IO_PIN_7 7
#define IO_PIN_6 6
// motor_4 rotation direction logic
#define IO_PIN_5 5
#define IO_PIN_4 4

const uint8_t XSHUT_PINS[DIST_SEN_NUMB] = {IO_PIN_4,IO_PIN_5,IO_PIN_6,IO_PIN_7};
