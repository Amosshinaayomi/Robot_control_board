#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define NUM_SENSORS 4
#define SDA_PIN GPIO_NUM_8
#define SCL_PIN GPIO_NUM_9
#define UPDATE_INTERVAL_MS 40

const uint8_t XSHUT_PINS[NUM_SENSORS] = {15,16,17,18};
const uint8_t SENSOR_ADDRESSES[NUM_SENSORS] = {0X30, 0X31, 0X32, 0X33};

Adafruit_VL53L0X tof_sensors[NUM_SENSORS];



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial)
  {
    delay(100);
  }
  Serial.println("Hello World");
  Wire.begin();
  Wire.setClock(1000000);
  // STEP 1: Shutdown all sensors first
  Serial.println("\nStep 1: Shutting down all sensors...");
  for(int i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);  // Hold in reset/shutdown
  }
  for(int i = 0; i < NUM_SENSORS-1; i++)
  {
    Serial.print("\nInitializing sensor ");
    Serial.println(i);

    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(5);

    if(!tof_sensors[i].begin(SENSOR_ADDRESSES[i]))
    {
      Serial.println(" FAILED!");
      Serial.print("  Could not initialize sensor at address 0x");
      Serial.println(SENSOR_ADDRESSES[i], HEX);
    }
  
    Serial.println("Success");
    tof_sensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    // Start continuous ranging mode
    tof_sensors[i].startRangeContinuous(UPDATE_INTERVAL_MS);
  }
 
}

void loop() {
  // put your main code here, to run repeatedly:
    // Record how long polling takes
    static uint32_t lastPollTime = 0;
    uint32_t pollStart = micros();
    
    // Poll each sensor in sequence
    for (int i = 0; i < NUM_SENSORS-1; i++) {
        // Check if measurement is complete (non-blocking)
        if (tof_sensors[i].isRangeComplete()) {
            // Read the result
            Serial.printf("sensor %i reading is %i\n", i, tof_sensors[i].readRangeResult()); 
            Serial.printf("sensor %i range is %i\n", i, tof_sensors[i].readRangeStatus());
            
            // Check for sensor errors
            if (tof_sensors[i].Status != VL53L0X_ERROR_NONE) {
                handle_sensor_error(i, tof_sensors[i].Status);
            }
        } else {
            // Measurement not ready
            Serial.println("no distance data was gotten");
        }

    }

    delay(UPDATE_INTERVAL_MS);
}


void handle_sensor_error(uint8_t sensor_id, uint8_t error_code) {
    Serial.print("ERROR: Sensor ");
    Serial.print(sensor_id);
    Serial.print(" reported error ");
    Serial.print(error_code);
    Serial.print(" (");
    
    // Common error codes (from vl53l0x_def.h)
    switch(error_code) {
        case 0: Serial.println("No error)"); break;
        case 1: Serial.println("CALIBRATION_WARNING)"); break;
        case 2: Serial.println("RANGING_COMPLETE_WITH_WRAP_AROUND)"); break;
        case 7: Serial.println("SIGNAL_FAIL)"); break;
        case 8: Serial.println("SIGMA_FAIL)"); break;
        case 13: Serial.println("OUT_OF_BOUNDS_FAIL)"); break;
        case 255: Serial.println("Not ready)"); break;
        default: Serial.println("Unknown error)"); break;
    }
}
