#pragma once

#define BUZZER GPIO_NUM_47
#define COMM_UART_RX 18
#define COMM_UART_TX 17

#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9


#define NUM_SENSORS 4

const uint8_t XSHUT_PINS[NUM_SENSORS] = {15,16,17,18};
const uint8_t SENSOR_ADDRESSES[NUM_SENSORS] = {0X30, 0X31, 0X32, 0X33};

