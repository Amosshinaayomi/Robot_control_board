#include "AHRS.h"
#include <math.h>

AHRS::AHRS(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cSpeed)
    : imu(0x69),
      mag(0x2c),
      currentTemperature(25.0f),
      referenceTemperature(25.0f),
      temperatureCompEnabled(false),
      lastUpdateMicros(0),
      lastTempReadMillis(0),
      roll(0), pitch(0), yaw(0), yawRate(0),
      declinationRad(0),
      sda(sdaPin), scl(sclPin), i2cSpeed(i2cSpeed)
{
        for (int i = 0; i < 3; i++) {
            gyroBias[i] = accelBias[i] = originalGyroBias[i] = 0.0f;
            linearAccBody[i] = 0.0f;
        }
}

bool AHRS::begin() {
    // Initialize IMU
    if (!imu.begin(SDA_PIN, SCL_PIN, 4000000UL)) {
        Serial.println("Failed to connect to BMI160!");
        return false;
    }
    Serial.println("BMI160 Connected.");

    // CONFIGURE SENSOR RANGES AND RATES ----
    imu.setAccelRange(4); // ±4g
    imu.setGyroRange(500); // ±500 degrees/sec
    
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
    calibrateSensors();


    // // Try to read initial temperature
    readTemperature();
    // Mag initialization
    if(!mag.begin(SDA_PIN, SCL_PIN, 4000000UL)) {
        Serial.println("Failed to connect to Mag");
        return false;
    } else {
        Serial.println("QMC5883 initialized successfully");
        // Set your local magnetic declination
        mag.setDeclination(7.4); 

        // Set orientation (adjust based on your sensor mounting)
        mag.setOrientation(QMC5883::ORIENTATION_ROTATE_270);  
    }




    // ---- C. CONFIGURE FUSION AHRS SETTINGS ----
    FusionAhrsSettings settings;
    settings.gain = 0.5f;               // Slightly higher than default 0.5 for better drift correction on a robot
    settings.accelerationRejection = 8.0f; // Degrees. Lower threshold for a robot that may accelerate smoothly.
    settings.gyroscopeRange = 500.0f;   // MUST match the dps setting above for correct over-range detection
    settings.recoveryTriggerPeriod = 5 * 800; // 5 seconds in samples (using 800Hz accel ODR)

    // Choose your coordinate convention:
    // - NWU: X=North, Y=West, Z=Up (Common for robotics)
    // - ENU: X=East, Y=North, Z=Up (Common for aviation)
    // - NED: X=North, Y=East, Z=Down (Common for drones)
    settings.convention = convention;

    FusionAhrsSetSettings(&ahrsFilter, &settings);

    // Initialize the AHRS filter. This starts the 3-second initialisation period.
    FusionAhrsInitialise(&ahrsFilter);

    // ---- D. INITIALIZE GYROSCOPE OFFSET CORRECTION ----
    // This runs alongside the AHRS to auto-calibrate the gyro bias during operation.
    FusionOffsetInitialise(&offsetFilter, 400); // Sample rate must match your gyro ODR (400Hz).

    lastUpdateMicros = micros();

    return true;
}

void AHRS::update() 
{
     // ---- A. TIMING ----
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastUpdateMicros) / 1000000.0f; // Convert to seconds
    lastUpdateMicros = currentMicros;
    // Safety check
    if (deltaTime > 0.1f) {
        return;
    }
    
    // === TEMPERATURE READING (every 500ms) ===
    static unsigned long lastTempRead = 0;
    if (millis() - lastTempRead > 500) {
        if (temperatureCompEnabled) {
            float newTemp;
            if (imu.getTemperature(&newTemp)) {
                static float tempHistory[3] = {25.0f, 25.0f, 25.0f};
                static int tempIndex = 0;
                
                tempHistory[tempIndex] = newTemp;
                tempIndex = (tempIndex + 1) % 3;
                currentTemperature = (tempHistory[0] + tempHistory[1] + tempHistory[2]) / 3.0f;
                
                const float gyroScale = imu.getGyroScaleDps();
                applyTemperatureCompensation(currentTemperature, gyroScale);
            }
        }
        lastTempRead = millis();
    }
    
    // ---- B. READ RAW SENSOR DATA ----
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (!imu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw)) {
        return;
    }

    // ---- C. APPLY SENSOR CALIBRATION ----
    const float accelScale = imu.getAccelScaleG();
    const float gyroScale  = imu.getGyroScaleDps();

    float ax_cal = (float)axRaw - accelBias[0];
    float ay_cal = (float)ayRaw - accelBias[1];
    float az_cal = (float)azRaw - accelBias[2];

    float gx_cal = (float)gxRaw - gyroBias[0];
    float gy_cal = (float)gyRaw - gyroBias[1];
    float gz_cal = (float)gzRaw - gyroBias[2];


    // Convert to physical units
    float ax_g = ax_cal * accelScale;
    float ay_g = ay_cal * accelScale;
    float az_g = az_cal * accelScale;

    float gx_dps = gx_cal * gyroScale;
    float gy_dps = gy_cal * gyroScale;
    float gz_dps = gz_cal * gyroScale;


    // Create Fusion vectors
    FusionVector accelerometer = {ax_g, ay_g, az_g};
    FusionVector gyroscope = {gx_dps, gy_dps, gz_dps};

    // ---- D. UPDATE GYROSCOPE OFFSET CORRECTION ----
    gyroscope = FusionOffsetUpdate(&offsetFilter, gyroscope);

    // ---- E. UPDATE THE AHRS FILTER ----
    FusionAhrsUpdateNoMagnetometer(&ahrsFilter, gyroscope, accelerometer, deltaTime);

    // ---- F. GET FILTER OUTPUTS ----
    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrsFilter);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    roll = -(euler.angle.roll);
    pitch = euler.angle.pitch;
    yaw = -(euler.angle.yaw);

    gx_dps = -gx_dps;
    gz_dps = -gz_dps;
    // Get linear acceleration in sensor frame
    FusionVector linearAcc = FusionAhrsGetLinearAcceleration(&ahrsFilter);
    // linearAccBody[0] = linearAcc.axis.x;
    // linearAccBody[1] = linearAcc.axis.y;
    // linearAccBody[2] = linearAcc.axis.z;

    // 1. Rotate linear acceleration to world frame
    // FusionVector worldLinearAcc = rotateVectorByQuaternion(quat, linearAcc);
    
    // Monitor filter health
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    if (states.accelerometerIgnored) {
        Serial.println("[Warning] High Acceleration - Accel data ignored.");
    }
  

    // QMC5883::Data data;
    // float magnetic_heading, true_heading;
}

// --- Getters (convert to degrees) ---
float AHRS::getRoll() const {
    return roll; 
}
float AHRS::getPitch() const {
    return pitch;
}
float AHRS::getYaw() const {
    return yaw;
}
float AHRS::getYawRate() const {
    return yawRate;  // already in degrees/sec
}
void AHRS::getLinearAcceleration(float &x, float &y, float &z) const {
    x = linearAccBody[0];
    y = linearAccBody[1];
    z = linearAccBody[2];
}
float AHRS::getTemperature() const {
    return currentTemperature;
}


void AHRS::calibrateSensors() {
    Serial.println("Calibrating Gyroscope (keep robot still)...");
    imu.calibrateGyro(500, &gyroBias[0], &gyroBias[1], &gyroBias[2]);
    for (int i = 0; i < 3; i++)
    { 
        originalGyroBias[i] = gyroBias[i];
    }
    Serial.println("Calibrating Accelerometer (keep robot level)...");
    imu.calibrateAccel(300, &accelBias[0], &accelBias[1], &accelBias[2]);
}

void AHRS::readTemperature() {
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
}



void AHRS::applyTemperatureCompensation(float currentTemp, float gyroScale) {
    if(!temperatureCompEnabled) return;
    float deltaTemp = currentTemp - referenceTemperature;

    // Gyro bias temperature compensation (most critical)
    // From datasheet: Zero-rate offset change over temperature = 0.05 °/s/K
    // Convert to LSB/K based on current gyro range
    float gyroTempCoeffLSB = 0.05f / gyroScale; // gyroScale is dps/LSB


    // Apply compensation to each axis
    for (int i = 0; i < 3; i++) {
        gyroBias[i] = originalGyroBias[i] + (gyroTempCoeffLSB * deltaTemp);
    }
    
    // Optional: Scale factor compensation (less critical)
    // From datasheet: Sensitivity change over temperature = ±0.02 %/K
    // We'll handle this in the main loop when converting to dps
}
