#pragma once

  if (!imu.begin(SDA_PIN, SCL_PIN, 4000000UL)) {
        Serial.println("Failed to connect to BMI160!");
        while (true); // Halt
    }
    Serial.println("BMI160 Connected.");

    // ---- A. CONFIGURE SENSOR RANGES AND RATES ----
    // Set ranges (affects resolution and dynamic range)
    imu.setAccelRange(4); // ±4g - Good for ground robot dynamics
    imu.setGyroRange(500); // ±500 degrees/sec - Adequate for turning
    
    // Wait for gyro to enter normal mode (should happen in configureSensor())
    delay(100); // Give time for power mode transition

    // Set Output Data Rates (ODR)
    imu.setAccelRate(0x0B); // 800Hz - High rate for responsive correction
    imu.setGyroRate(0x0A);  // 400Hz - Can be slightly lower than accelerometer

    // Check if gyro is in normal mode (required for temp sensor at 100Hz)
    if (imu.isGyroInNormalMode()) {
        Serial.println("Gyro in normal mode - Temperature sensor ready.");
        temperatureCompEnabled = true;
    } else {
        Serial.println("WARNING: Gyro not in normal mode - temperature sensor limited.");
        temperatureCompEnabled = false;
    }
    delay(500); // let it settle
    // ---- B. PERFORM STARTUP CALIBRATION ----
    Serial.println("Calibrating Gyroscope (keep robot still)...");
    imu.calibrateGyro(500, &gyroBias[0], &gyroBias[1], &gyroBias[2]);
    
    // Save original biases for temperature compensation
    originalGyroBias[0] = gyroBias[0];
    originalGyroBias[1] = gyroBias[1];
    originalGyroBias[2] = gyroBias[2];

    Serial.println("Calibrating Accelerometer (keep robot level)...");
    imu.calibrateAccel(300, &accelBias[0], &accelBias[1], &accelBias[2]);

    // Try to read initial temperature
    float temp;
    int attempts = 0;
    while (attempts < 10) {
        if (imu.getTemperature(&temp)) {
            referenceTemperature = temp;
            Serial.print("Reference temperature: ");
            Serial.print(referenceTemperature, 1);
            Serial.println(" °C");
            break;
        }
        attempts++;
        delay(50);
    }
    
    if (attempts >= 10) {
        Serial.println("Could not read temperature sensor. Using default 25°C.");
        referenceTemperature = 25.0f;
        currentTemperature = 25.0f;
    } else {
        currentTemperature = referenceTemperature;
    }
  // IMU initialization
  if(mag.begin(SDA_PIN, SCL_PIN)) {
    Serial.println("QMC5883 initialized successfully");
    Serial.print("Chip ID: 0x");
    Serial.println(mag.getChipID(), HEX);
    
    // Perform self-test
    if (mag.selfTest()) {
        Serial.println("Self-test PASSED");
    } else {
        Serial.println("Self-test FAILED");
    }
  } else {
      Serial.println("Failed to initialize QMC5883");
  }

  // Set your local magnetic declination
  mag.setDeclination(7.4); 

  // Set orientation (adjust based on your sensor mounting)
  mag.setOrientation(QMC5883::ORIENTATION_ROTATE_270);  
  bool magSetupStatus = setupMag();
  if(magSetupStatus)
  {
    Serial.println("Mag setup successful");
  } else {
    Serial.println("Mag setup failed");
  }

