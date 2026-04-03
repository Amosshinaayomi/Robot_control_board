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
      sda(sdaPin), scl(sclPin), i2cSpeed(i2cSpeed),
      dataMutex(NULL)   // initialize mutex handle to NULL

{
        for (int i = 0; i < 3; i++) {
            gyroBias[i] = accelBias[i] = originalGyroBias[i] = 0.0f;
            linearAccBody[i] = 0.0f;
            accelFiltered[i] = 0.0f;
            gyroFiltered[i] = 0.0f;
        }
}

bool AHRS::begin() {
    // Initialize IMU
    if (!imu.begin(SDA_PIN, SCL_PIN, 4000000UL)) {
        Serial.println("Failed to connect to BMI160!");
        return false;
    }
    Serial.println("BMI160 Connected.");

    // Create a recursive mutex (or standard mutex) for thread safety
    dataMutex = xSemaphoreCreateMutex();
    if(dataMutex == NULL) {
        Serial.println("Failed to create AHRS mutex");
        return false;
    }
    // CONFIGURE SENSOR RANGES AND RATES ----
    imu.setAccelRange(4); // ±4g
    imu.setGyroRange(500); // ±500 degrees/sec
    
    // Wait for gyro to enter normal mode (should happen in configureSensor())
    delay(50); // Give time for power mode transition

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
    delay(50);
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

    // --- CRITICAL SECTION: update shared members ---
    
    bool accelIgnored = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        roll = -(euler.angle.roll);
        pitch = euler.angle.pitch;
        yaw = -(euler.angle.yaw);


        gx_dps = gx_dps;
        gz_dps = gz_dps;

        yawRate = gz_dps;   // filtered yaw rate        
        accelFiltered[0] = ax_g;
        accelFiltered[1] = ay_g;
        accelFiltered[2] = az_g;

        gyroFiltered[0] = gx_dps;
        gyroFiltered[1] = gy_dps;
        gyroFiltered[2] = gz_dps;

        // Get linear acceleration in sensor frame
        FusionVector linearAcc = FusionAhrsGetLinearAcceleration(&ahrsFilter);
        // linearAccBody[0] = linearAcc.axis.x;
        // linearAccBody[1] = linearAcc.axis.y;
        // linearAccBody[2] = linearAcc.axis.z;

        // 1. Rotate linear acceleration to world frame
        // FusionVector worldLinearAcc = rotateVectorByQuaternion(quat, linearAcc);
        
        // Monitor filter health
        FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
        if (states.accelerometerIgnored) accelIgnored = true;

        // QMC5883::Data data;
        // float magnetic_heading, true_heading;

        xSemaphoreGive(dataMutex);
    }
        if (accelIgnored) Serial.println("[Warning] High Acceleration - Accel data ignored.");
}


// --- Getters 


void AHRS::getAccel(float accelData[3]) const {
    // --- CRITICAL SECTION: update shared members ---
    
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        accelData[0] = accelFiltered[0];
        accelData[1] = accelFiltered[1];
        accelData[2] = accelFiltered[2];
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to get data from  accel.");
        accelData[0] = accelData[1] = accelData[2] = 0;
    }
}

void AHRS::getGyro(float gyroData[3]) const {
    // --- CRITICAL SECTION: update shared members ---
    
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        
        
        gyroData[0] = gyroFiltered[0];
        gyroData[1] = gyroFiltered[1];
        gyroData[2] = gyroFiltered[2];
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to get data from  gyro.");
        gyroData[0] = gyroData[1] = gyroData[2] = 0;
    }
}
float AHRS::getRoll() const {
    float val;
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        
        val = roll;
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to fetch data.");
        val = 0.0f; // fallback, should not happen
    }
    return val;
}

float AHRS::getPitch() const {
    float val;
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        
        val = pitch;
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to fetch data.");
        val = 0.0f;
    }
    return val;
}

float AHRS::getYaw() const {
    float val;
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        
        val = yaw;
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to fetch data.");
        val = 0.0f;
    }
    return val;
}

float AHRS::getYawRate() const {
    float val;
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        val = yawRate;
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to fetch data.");
        val = 0.0f;
    }
    return val;
}

void AHRS::getLinearAcceleration(float &x, float &y, float &z) const {
    // bool dataNotfetched = true;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // dataNotfetched = false;
        
        x = linearAccBody[0];
        y = linearAccBody[1];
        z = linearAccBody[2];
        xSemaphoreGive(dataMutex);
    } else {
        // Serial.println("wasn't able to fetch data.");
        x = y = z = 0.0f;
    }
}

float AHRS::getTemperature() const {
    return currentTemperature;
}

void AHRS::setMagneticDeclination(float declDeg) {
    declinationRad = declDeg * M_PI / 180.0f;
}

// Calibration helpers (unchanged)
void AHRS::calibrateSensors() {
    Serial.println("Calibrating Gyroscope (keep robot still)...");
    imu.calibrateGyro(500, &gyroBias[0], &gyroBias[1], &gyroBias[2]);
    for (int i = 0; i < 3; i++) originalGyroBias[i] = gyroBias[i];
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
    if (!temperatureCompEnabled) return;
    float deltaTemp = currentTemp - referenceTemperature;
    float gyroTempCoeffLSB = 0.05f / gyroScale;
    for (int i = 0; i < 3; i++) {
        gyroBias[i] = originalGyroBias[i] + (gyroTempCoeffLSB * deltaTemp);
    }
}







void printAHRSPacket(ahrsPacket_t data)
{
    Serial.print("Time stamp: "); Serial.println(data.timestamp_ms);
    Serial.print("Gyro (dps) X,Y,Z: ");
    Serial.print(data.gyro_dps[0], 3); Serial.print(", ");
    Serial.print(data.gyro_dps[1], 3); Serial.print(", ");
    Serial.println(data.gyro_dps[2], 3);
    Serial.print("Accel(g) X,Y,Z: ");
    Serial.print(data.accel_g[0], 3); Serial.print(", ");
    Serial.print(data.accel_g[1], 3); Serial.print(", ");
    Serial.println(data.accel_g[2], 3);

    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(", ROLL:"); Serial.print(data.roll);
    Serial.print(", YAW:"); Serial.println(data.yaw); 

    Serial.print("YawRate: "); Serial.println(data.yawRate);
    Serial.printf("Front Left side encoder tick is %i\nFront right side encoder tick is %i\nBack Left side encoder tick is %i\nBack right side encoder tick is %i\n",data.encoder_ticks[0], data.encoder_ticks[1], data.encoder_ticks[2], data.encoder_ticks[3]);  
    Serial.println();    
}
