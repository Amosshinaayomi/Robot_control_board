#include <Wire.h>
#include <PCF8574.h>
#include <Adafruit_VL53L0X.h>

// ----------------- PCF8574 Setup -----------------
PCF8574 pcf8574(0x27);
int xshutPins[4] = {7, 6, 3, 2};
uint8_t sensorAddresses[4] = {0x30, 0x31, 0x32, 0x33};
Adafruit_VL53L0X lox[4];
bool sensorReady[4] = {false, false, false, false};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(5);
  Wire.begin();

  Serial.print("Initializing PCF8574... ");
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
    while (1);
  }

  // 1. Disable all sensors (hold in reset)
  for (int i = 0; i < 4; i++) {
    pcf8574.write(xshutPins[i], LOW);
  }
  delay(100);

  // 2. Enable and initialise each sensor one by one
  for (int i = 0; i < 4; i++) {
    pcf8574.write(xshutPins[i], HIGH);   // bring out of reset
    delay(50);                            // let it power up

    Serial.print("Init sensor ");
    Serial.print(i);
    Serial.print(" at 0x");
    Serial.print(sensorAddresses[i], HEX);
    Serial.print(" ... ");

    if (lox[i].begin(sensorAddresses[i])) {
      Serial.println("OK");
      sensorReady[i] = true;
      // Use HIGH_SPEED for faster readings (or HIGH_ACCURACY if you prefer)
      lox[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    } else {
      Serial.println("FAILED");
      sensorReady[i] = false;
    }
    delay(10);
  }

  Serial.println("All sensors initialised. Starting readings...\n");
}

void loop() {
  // Read each sensor sequentially
  int startmillis = millis();
  for (int i = 0; i < 4; i++) {
    if (!sensorReady[i]) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" not ready – skipping.");
      continue;
    }

    VL53L0X_RangingMeasurementData_t measure;
    // Perform a single ranging measurement (blocking)
    lox[i].rangingTest(&measure, false);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" | status=");
    Serial.print(measure.RangeStatus);
    Serial.print(" | distance=");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  }
  Serial.printf("time taken for all four sensors in millis is %i\n", millis() - startmillis);
  Serial.println("--------------------");
  delay(500);   // wait half a second before next cycle
}