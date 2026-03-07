// #pragma once

// #include <Arduino.h>
// #include <pins.h>
// #include "BMI160.h"
// #include "QMC5883.h"

// class AHRS {
//     public:
//         AHRS(uint8_t sdaPin = SDA, uint8_t sclPin = SCL, uint32_t i2cSpeed = 400000UL);

//             // Initialize sensors, calibrate, set up filter – call in setup()
//         bool begin();

//         // Must be called as often as possible (e.g., every loop iteration)
//         void update();

//         // Getters for orientation (angles in degrees)
//         float getRoll() const;     // degrees
//         float getPitch() const;    // degrees
//         float getYaw() const;      // degrees (0‑360, heading)
//         float getYawRate() const;  // degrees per second

//         // Linear acceleration (body frame, g's)
//         void getLinearAcceleration(float &x, float &y, float &z) const;

//         // World‑frame linear acceleration (useful for dead reckoning)
//         void getWorldLinearAcceleration(float &x, float &y, float &z) const;

//         // Temperature in °C
//         float getTemperature() const;

//         // Set magnetic declination (in degrees)
//         void setMagneticDeclination(float declinationDeg);

//         // (Optional) Manually set calibration biases – useful after EEPROM restore
//         void setGyroBias(float x, float y, float z);
//         void setAccelBias(float x, float y, float z);

//     private:
//         // Sensor objects
//         BMI160Wrapper imu;
//         QMC5883LWrapper mag;

//         // Fusion library objects
//         FusionOffset offsetFilter;      // gyro offset correction
//         FusionAhrs ahrsFilter;          // main AHRS filter

//         // Calibration data
//         float gyroBias[3];
//         float accelBias[3];
//         float originalGyroBias[3];      // for temperature compensation
//         float currentTemperature;
//         float referenceTemperature;
//         bool temperatureCompEnabled;

//         // Last update time for delta calculation
//         unsigned long lastUpdateMicros;

//         // Cached outputs
//         mutable float roll, pitch, yaw;
//         float yawRate;
//         float linearAccBody[3];

//         // Magnetic declination (radians)
//         float declinationRad;

//         // I2C settings
//         uint8_t sda, scl;
//         uint32_t i2cSpeed;

//         // Private helpers
//         void calibrateSensors();
//         void readTemperature();
//         void applyTemperatureCompensation(float gyroScale);
//         FusionVector rotateVectorByQuaternion(const FusionQuaternion &q, const FusionVector &v);
// }