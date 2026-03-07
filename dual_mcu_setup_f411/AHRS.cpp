// #include "AHRS.h"
// #include <math.h>

// AHRS::AHRS(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cSpeed)
//     : imu(sdaPin, sclPin, i2cSpeed),
//       mag(sdaPin, sclPin, i2cSpeed),
//       currentTemperature(25.0f),
//       referenceTemperature(25.0f),
//       temperatureCompEnabled(false),
//       lastUpdateMicros(0),
//       roll(0), pitch(0), yaw(0), yawRate(0),
//       declinationRad(0) {
//     for (int i = 0; i < 3; i++) {
//         gyroBias[i] = accelBias[i] = originalGyroBias[i] = 0.0f;
//         linearAccBody[i] = 0.0f;
//     }
// }


// bool AHRS::begin() {
//     // 1. Initialize IMU
//     if (!imu.begin()) {
//         Serial.println("BMI160 init failed");
//         return false;
//     }

//     // Configure IMU ranges and rates
//     imu.setAccelRange(4);       // ±4g
//     imu.setGyroRange(500);      // ±500 dps
//     delay(100);
//     imu.setAccelRate(0x0B);     // 800 Hz
//     imu.setGyroRate(0x0A);      // 400 Hz

//     // Check gyro mode for temperature compensation
//     if (imu.isGyroNormalMode()) {
//         temperatureCompEnabled = true;
//     } else {
//         Serial.println("Gyro not in normal mode – temp comp disabled");
//     }
//     delay(500);

//     // 2. Calibrate IMU
//     calibrateSensors();

//     // 3. Initialize Magnetometer
//     if (!mag.begin()) {
//         Serial.println("QMC5883L init failed – proceeding without magnetometer");
//     } else {
//         mag.selfTest();          // optional
//         mag.setOrientation(QMC5883::ORIENTATION_ROTATE_270); // adjust as needed
//     }

//     // 4. Set default magnetic declination (you can change later via setter)
//     setMagneticDeclination(7.4);  // example for your location

//     // 5. Initialize Fusion filters
//     //    Gyro offset filter: sample rate = gyro ODR (400 Hz)
//     FusionOffsetInitialise(&offsetFilter, 400);
//     //    AHRS filter: default settings
//     FusionAhrsInitialise(&ahrsFilter);
//     // Optional: adjust gain – FusionAhrsSetSettings(&ahrsFilter, &settings);

//     // 6. Read initial temperature
//     readTemperature();

//     lastUpdateMicros = micros();
//     return true;
// }
