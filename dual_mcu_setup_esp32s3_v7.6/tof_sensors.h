#include "PCF8574.h"
#include "wire.h"
#include <Adafruit_VL53L0X.h>
// #include <Adafruit_PCF8574.h>


#define UPDATE_INTERVAL_MS 40


const uint8_t SENSOR_ADDRESSES[DIST_SEN_NUMB] = {0X30, 0X31, 0X32, 0X33};

PCF8574 ioExpander(0x38);
Adafruit_VL53L0X tof_sensors[DIST_SEN_NUMB];