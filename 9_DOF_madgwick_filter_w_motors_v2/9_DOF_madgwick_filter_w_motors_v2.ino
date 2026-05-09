// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include <Wire.h>
#include "STMPWMTimer.h"
#include "BMI160.h" // Your driver header
#include "QMC5883.h"  
#include "Fusion.h" // The x-io Fusion library header
#include <EEPROM.h>


typedef struct  {
    float gyroBias[3];      // from AHRS (deg/s)
    float accelBias[3];     // from AHRS (m/s^2 or g)
    float hardIronOffset[3];    // hard iron offsets (uT)
    int16_t magXMin, magXMax;  // raw counts
    int16_t magYMin, magYMax;
    int16_t magZMin, magZMax;
    float softIronmatrix[3][3];   // soft iron matrix (3x3)
    uint16_t magic;         // e.g., 0x5A5A to indicate valid data
    uint16_t checksum;      // simple XOR of all previous bytes
} CalibrationData_t;

// ------------------------------------------------------------------
// EEPROM storage address
// ------------------------------------------------------------------
#define CALIB_EEPROM_ADDR 0
#define CALIB_MAGIC 0x5A5A

#define SDA_PIN PB7
#define SCL_PIN PB6

uint8_t motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB0, PB13};
uint8_t motorDcontrolpins[3] = {PA9, PB1, PB10};

#define MOTOR_STBY_PIN PB12

// Create motor objects
STMPWMTimer motorA(motorAcontrolpins[0], 25000);   // TIM1_CH3
STMPWMTimer motorB(motorBcontrolpins[0], 25000);   // TIM1_CH2 (same timer, different channel!)
STMPWMTimer motorC(motorCcontrolpins[0], 25000);   // TIM1_CH1 (same timer, different channel!)
STMPWMTimer motorD(motorDcontrolpins[0], 25000);   // TIM2_CH2 (different timer)



// Sensor and Fusion Objects
BMI160 imu; // Using default I2C address 0x69
QMC5883 mag;
FusionAhrs ahrsFilter;
FusionOffset offsetFilter; // For gyroscope run-time offset correction


typedef struct {
    unsigned long timestamp_us;  // Microsecond timestamp
    float accel_g[3];
    float gyro_dps[3];
    // Orientation (Degrees)
    float roll;
    float pitch;
    float yaw;
    // Derived Data
    float linear_accel_body[3]; // Linear accel in body frame (g)
    float temperature_c;
} ImuDataPacket_t;


// Timestamp for delta-time calculation
unsigned long previousMicros = 0;
// Biases and temperature compensation
float accelBias[3] = {0, 0, 0}; // Store biases for X, Y, Z
float gyroBias[3] = {0, 0, 0};
float originalGyroBias[3] = {0, 0, 0};

// Online hard‑iron tracking
float magRawMin[3] = {30000, 30000, 30000};
float magRawMax[3] = {-30000, -30000, -30000};
unsigned long lastDynamicUpdate = 0;
const unsigned long DYNAMIC_UPDATE_PERIOD = 100; // 60 seconds

// Temperature variables
float referenceTemperature = 25.0f;
float currentTemperature = 25.0f;
float temperatureAlpha = 0.1f; //Low-pass filter coefficient for temperature
bool temperatureCompEnabled = true;
unsigned long lastTempUpdate = 0;
const unsigned long TEMP_UPDATE_INTERVAL = 500000; // 100ms in microseconds


// Forward declarations
bool loadCalibrationFromEEPROM(CalibrationData_t &data);
void saveCalibrationToEEPROM(const CalibrationData_t &data);
uint16_t calculateChecksum(const CalibrationData_t &data);
void calibrateMagnetometer2D();
void storeAllCalibration();
void updateDynamicHardIron(float rawX, float rawY, float rawZ);
void applyDynamicHardIron();


// Coordinate convention
static FusionConvention convention = FusionConventionNwu;
FusionVector CustomAxesSwapPXNY(const FusionVector sensor)
{
    FusionVector result;
    result.axis.x = sensor.axis.x;
    result.axis.y = -sensor.axis.y;
    result.axis.z = sensor.axis.z;
    return result;
}


void applyTemperatureCompensation(float currentTemp, float gyroScale) {
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

// Add this function to rotate vectors by quaternion
FusionVector rotateVectorByQuaternion(const FusionQuaternion& q, const FusionVector& v) {
    // q * v * q_conjugate
    FusionQuaternion v_quat = {0, v.axis.x, v.axis.y, v.axis.z};
    FusionQuaternion q_conj = {q.element.w, -q.element.x, -q.element.y, -q.element.z};
    
    FusionQuaternion temp = FusionQuaternionMultiply(q, v_quat);
    FusionQuaternion result = FusionQuaternionMultiply(temp, q_conj);
    
    return {result.element.x, result.element.y, result.element.z};
}



bool initMotorDrivers();
void analog_turn_l(int dutyCycle);
void analog_turn_r(int dutyCycle);





// ==== 2. INITIALIZATION FUNCTION ====
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    // while(!Serial) {delay(50);}
    bool initMotors = initMotorDrivers();
    if(!initMotors){
        Serial.println("Motor  driver initialization failed");
        while(true);
    }
    // Initialize BMI160
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


    
    // Initialize QMC5883P magnetometer
    if (!mag.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("Failed to connect to QMC5883P!");
        while (1);
    }
    Serial.println("QMC5883P Connected.");

    // Set orientation to match your IMU's coordinate convention (NWU)
    // This depends on physical mounting; see Section 2.
    mag.setOutputDataRate(QMC5883::ODR_100Hz);
    Serial.println("Reading at magnetic north");
    // QMC5883::Data raw;
    // mag.read(raw, QMC5883::RNG_8G);
    // Serial.print("Raw X:"); Serial.print(raw.x);
    // Serial.print(" Y:"); Serial.print(raw.y);
    // Serial.print(" Z:"); Serial.println(raw.z);
    // delay(3000);
    mag.setCustomOrientation(
    1,    // BMI160 X from Raw Y
    0,    // BMI160 Y from Raw X
    2,    // BMI160 Z from Raw Z
    false, // Do not invert X (Raw Y already negative)
    true,  // Invert Y (to get negative when facing East)
    true   // Invert Z (to make downward field negative)
); // due to magnetometer orientation relative to the IMU

    // ---------- 3. Load saved calibration or run fresh ----------
    CalibrationData_t savedCal;
    if (loadCalibrationFromEEPROM(savedCal)) {
        // Restore BMI160 biases
        gyroBias[0] = savedCal.gyroBias[0];
        gyroBias[1] = savedCal.gyroBias[1];
        gyroBias[2] = savedCal.gyroBias[2];
        accelBias[0] = savedCal.accelBias[0];
        accelBias[1] = savedCal.accelBias[1];
        accelBias[2] = savedCal.accelBias[2];

        // Restore magnetometer calibrate
        QMC5883::CalibrationData magCal;
        magCal.x_min = savedCal.magXMin;
        magCal.x_max = savedCal.magXMax;
        magCal.y_min = savedCal.magYMin;
        magCal.y_max = savedCal.magYMax;
        magCal.z_min = savedCal.magZMin;
        magCal.z_max = savedCal.magZMax;
        magCal.calibrated = true;
        memcpy(magCal.soft_iron, savedCal.softIronmatrix, sizeof(magCal.soft_iron));
        mag.setCalibrationData(magCal);

        Serial.println("Calibration loaded from EEPROM.");
    } else {
        // ---- B. PERFORM STARTUP CALIBRATION ----
        Serial.println("Calibrating Gyroscope (keep robot still)...");
        imu.calibrateGyro(500, &gyroBias[0], &gyroBias[1], &gyroBias[2]);
        
        // Save original biases for temperature compensation
        originalGyroBias[0] = gyroBias[0];
        originalGyroBias[1] = gyroBias[1];
        originalGyroBias[2] = gyroBias[2];    

        Serial.println("Calibrating Accelerometer (keep robot level)...");
        imu.calibrateAccel(300, &accelBias[0], &accelBias[1], &accelBias[2]);
        calibrateMagnetometer2D(12000);     

        // Save everything
        storeAllCalibration();
        Serial.println("Fresh calibration stored.");       
    }





    // Check if gyro is in normal mode (required for temp sensor at 100Hz)
    if (imu.isGyroInNormalMode()) {
        Serial.println("Gyro in normal mode - Temperature sensor ready.");
        temperatureCompEnabled = true;
    } else {
        Serial.println("WARNING: Gyro not in normal mode - temperature sensor limited.");
        temperatureCompEnabled = false;
    }
    delay(100); // let it settle

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
    // Optional: Set magnetic declination for your location (e.g., 10.5° East)
    // mag.setDeclination(10.5f);

    if (attempts >= 10) {
        // Serial.println("Could not read temperature sensor. Using default 25°C.");
        referenceTemperature = 25.0f;
        currentTemperature = 25.0f;
    } else {
        currentTemperature = referenceTemperature;
    }
    
    // ---- C. CONFIGURE FUSION AHRS SETTINGS ----
    FusionAhrsSettings settings;
    settings.gain = 0.5f;               // Slightly higher than default 0.5 for better drift correction on a robot
    settings.accelerationRejection = 20.0f; // Degrees. Lower threshold for a robot that may accelerate smoothly.
    settings.gyroscopeRange = 500.0f;   // MUST match the dps setting above for correct over-range detection
    settings.recoveryTriggerPeriod = 1 * 200; // 1 seconds in samples (using 400hz update rate)
    settings.magneticRejection = 30.0f;
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

    previousMicros = micros();
    Serial.println("IMU and Fusion Library Initialized.");
}

long last_print_millis = millis();
float currentVelX, currentVelY, currentVelZ, prevVelX, prevVelY, prevVelZ, posX, posY, posZ;

// ==== 3. MAIN PROCESSING LOOP ====
void loop() {
    // ---- A. TIMING ----
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - previousMicros) / 1000000.0f; // Convert to seconds
    previousMicros = currentMicros;

    // Safety check
    if (deltaTime > 0.1f) {
        return;
    }
    
    // === TEMPERATURE READING (every 500ms) ===
    // static unsigned long lastTempRead = 0;
    // if (millis() - lastTempRead > 500) {
    //     if (temperatureCompEnabled) {
    //         float newTemp;
    //         if (imu.getTemperature(&newTemp)) {
    //             static float tempHistory[3] = {25.0f, 25.0f, 25.0f};
    //             static int tempIndex = 0;
                
    //             tempHistory[tempIndex] = newTemp;
    //             tempIndex = (tempIndex + 1) % 3;
    //             currentTemperature = (tempHistory[0] + tempHistory[1] + tempHistory[2]) / 3.0f;
                
    //             const float gyroScale = imu.getGyroScaleDps();
    //             applyTemperatureCompensation(currentTemperature, gyroScale);
    //         }
    //     }
    //     lastTempRead = millis();
    // }
    

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
    // ----- Read magnetometer (raw + calibrated) -----
    QMC5883::Data rawMag, calMag;
    bool magValid = mag.readCalibrated(calMag, QMC5883::RNG_8G);
    FusionVector magnetometer;


        // Raw (oriented but uncalibrated) for online hard-iron tracking
    if (mag.read(rawMag, QMC5883::RNG_8G)) {
        updateDynamicHardIron(rawMag.x, rawMag.y, rawMag.z);
    }

    // Serial.print("Raw X:"); Serial.print(calMag.x);
    // Serial.print(" Y:"); Serial.print(calMag.y);
    // Serial.print(" Z:"); Serial.println(calMag.z);
    if (magValid) {
        // Convert Gauss to µT (1 Gauss = 100 µT)
        magnetometer.axis.x = calMag.x * 100.0f;
        magnetometer.axis.y = calMag.y * 100.0f;
        magnetometer.axis.z = calMag.z * 100.0f;
    }
    // ---- E. UPDATE THE AHRS FILTER ----
    if (magValid) {
        FusionAhrsUpdate(&ahrsFilter, gyroscope, accelerometer, magnetometer, deltaTime);
        // Serial.println("Using full 9DOF");
    } else {
        // Fallback to no-magnetometer update if magnetometer read fails
        FusionAhrsUpdateNoMagnetometer(&ahrsFilter, gyroscope, accelerometer, deltaTime);
        // Serial.println("Using 6DOF");
        }
// FusionAhrsUpdateNoMagnetometer(&ahrsFilter, gyroscope, accelerometer, deltaTime);

    // ---- F. GET FILTER OUTPUTS ----
    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrsFilter);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    float roll = -(euler.angle.roll);
    float pitch = euler.angle.pitch;
    float yaw = -(euler.angle.yaw);
    gx_dps = -gx_dps;
    gz_dps = -gz_dps;
    // Get linear acceleration in sensor frame
    FusionVector linearAcc = FusionAhrsGetLinearAcceleration(&ahrsFilter);
    
    // 1. Rotate linear acceleration to world frame
    FusionVector worldLinearAcc = rotateVectorByQuaternion(quat, linearAcc);
    

    ImuDataPacket_t imuData;
    imuData.timestamp_us = currentMicros;
    imuData.accel_g[0] = ax_g; imuData.accel_g[1] = ay_g; imuData.accel_g[2] = az_g;
    imuData.gyro_dps[0] = gx_dps; imuData.gyro_dps[1] = gy_dps; imuData.gyro_dps[2] = gz_dps;
    imuData.roll = roll;
    imuData.pitch = pitch;
    imuData.yaw = yaw;
    
    // Linear acceleration (body frame)
    imuData.linear_accel_body[0] = linearAcc.axis.x;
    imuData.linear_accel_body[1] = linearAcc.axis.y;
    imuData.linear_accel_body[2] = linearAcc.axis.z;
    
    // Temperature
    imuData.temperature_c = currentTemperature;  


    // ---- H. PERIODIC OUTPUT ----
    if(millis() - last_print_millis >= 20) {
        // Invert gyro axes for display (if needed)
        // printIMUPacket(imuData);
        sendVisualizationData(imuData);
        last_print_millis = millis();
    }
    
    // Monitor filter health
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    if (states.accelerometerIgnored) {
        Serial.println("[Warning] High Acceleration - Accel data ignored.");
    }
    if(states.magnetometerIgnored) {
        Serial.println("mag cel data ignored.");
    }

   // ----- Apply dynamic hard-iron updates (low‑pass filtered) -----
    // applyDynamicHardIron();

    // Control loop rate
    delay(2);
}


uint16_t calculateChecksum(const CalibrationData_t &data) {
    const uint8_t *bytes = (const uint8_t*)&data;
    uint16_t chk = 0;
    for (size_t i = 0; i < offsetof(CalibrationData_t, checksum); i++) {
        chk ^= bytes[i];
    }
    return chk;
}


bool loadCalibrationFromEEPROM(CalibrationData_t &data) {
    uint8_t *bytes = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(data); i++) {
        bytes[i] = EEPROM.read(CALIB_EEPROM_ADDR + i);
    }
    if (data.magic != CALIB_MAGIC) return false;
    if (data.checksum != calculateChecksum(data)) return false;
    return true;
}

void saveCalibrationToEEPROM(const CalibrationData_t &data) {
    CalibrationData_t toSave = data;
    toSave.magic = CALIB_MAGIC;
    toSave.checksum = calculateChecksum(toSave);
    const uint8_t *bytes = (const uint8_t*)&toSave;
    for (size_t i = 0; i < sizeof(toSave); i++) {
        EEPROM.write(CALIB_EEPROM_ADDR + i, bytes[i]);
    }
}

void storeAllCalibration() {
    CalibrationData_t data;

    // BMI160 biases (already stored globally after calibration)
    data.gyroBias[0] = gyroBias[0];
    data.gyroBias[1] = gyroBias[1];
    data.gyroBias[2] = gyroBias[2];
    data.accelBias[0] = accelBias[0];
    data.accelBias[1] = accelBias[1];
    data.accelBias[2] = accelBias[2];

    // Magnetometer calibration
    
    const QMC5883::CalibrationData& magCal = mag.getCalibrationData();
    // if (magCal.calibrated) {
    //     data.magXMin = magCal.x_min;
    //     data.magXMax = magCal.x_max;
    //     data.magYMin = magCal.y_min;
    //     data.magYMax = magCal.y_max;
    //     data.magZMin = magCal.z_min;
    //     data.magZMax = magCal.z_max;
    //     memcpy(data.softIronmatrix, magCal.soft_iron, sizeof(data.softIronmatrix));

    //     // Hard iron offset in µT (for reference)
    //     float sens = QMC5883::getSensitivity(QMC5883::RNG_8G);
    //     data.hardIronOffset[0] = ((magCal.x_min + magCal.x_max) / 2.0f) / sens * 100.0f;
    //     data.hardIronOffset[1] = ((magCal.y_min + magCal.y_max) / 2.0f) / sens * 100.0f;
    //     data.hardIronOffset[2] = ((magCal.z_min + magCal.z_max) / 2.0f) / sens * 100.0f;
    // } else {
    //     // fallback: zeros and identity
    //     data.magXMin = data.magXMax = 0;
    //     data.magYMin = data.magYMax = 0;
    //     data.magZMin = data.magZMax = 0;
    //     memset(data.softIronmatrix, 0, sizeof(data.softIronmatrix));
    //     data.softIronmatrix[0][0] = data.softIronmatrix[1][1] = data.softIronmatrix[2][2] = 1.0f;
    //     data.hardIronOffset[0] = data.hardIronOffset[1] = data.hardIronOffset[2] = 0.0f;
    // }
    if (magCal.calibrated) {
        data.magXMin = magCal.x_min;
        data.magXMax = magCal.x_max;
        data.magYMin = magCal.y_min;
        data.magYMax = magCal.y_max;
        data.magZMin = magCal.z_min;
        data.magZMax = magCal.z_max;
        memcpy(data.softIronmatrix, magCal.soft_iron, sizeof(data.softIronmatrix));
    } else {
        // fallback identity
        memset(data.softIronmatrix, 0, sizeof(data.softIronmatrix));
        data.softIronmatrix[0][0] = data.softIronmatrix[1][1] = data.softIronmatrix[2][2] = 1.0f;
    }
    saveCalibrationToEEPROM(data);
    Serial.println("Calibration saved to EEPROM.");
}

void updateDynamicHardIron(float rawX, float rawY, float rawZ) {
    if (rawX < magRawMin[0]) magRawMin[0] = rawX;
    if (rawX > magRawMax[0]) magRawMax[0] = rawX;
    if (rawY < magRawMin[1]) magRawMin[1] = rawY;
    if (rawY > magRawMax[1]) magRawMax[1] = rawY;
    if (rawZ < magRawMin[2]) magRawMin[2] = rawZ;
    if (rawZ > magRawMax[2]) magRawMax[2] = rawZ;
}

void applyDynamicHardIron() {


    if (millis() - lastDynamicUpdate < DYNAMIC_UPDATE_PERIOD) return;

    lastDynamicUpdate = millis();

    // Require sufficient range (at least 0.3 Gauss) to trust estimate
    float rangeX = magRawMax[0] - magRawMin[0];
    float rangeY = magRawMax[1] - magRawMin[1];
    if (rangeX < 0.3 || rangeY < 0.3) {
        // Reset min/max to avoid stale data
        magRawMin[0] = magRawMin[1] = magRawMin[2] = 30000;
        magRawMax[0] = magRawMax[1] = magRawMax[2] = -30000;
        return;
    }
    Serial.println("Applying dynamic hardIron calibration");
    // Estimate new hard-iron offset (in Gauss, same units as raw)
    float newOffX = (magRawMax[0] + magRawMin[0]) / 2.0f;
    float newOffY = (magRawMax[1] + magRawMin[1]) / 2.0f;
    float newOffZ = (magRawMax[2] + magRawMin[2]) / 2.0f;

    // Low-pass filter with existing offset (get current centre from dri
    const QMC5883::CalibrationData cal = mag.getCalibrationData();
    if (!cal.calibrated) return;

    // Get current offset via const getter
    
    const QMC5883::Data& curOff = mag.getHardIronOffset();
    const float alpha = 0.05f;   // slow adaptation
    float smoothX = curOff.x + alpha * (newOffX - curOff.x);
    float smoothY = curOff.y + alpha * (newOffY - curOff.y);
    float smoothZ = curOff.z + alpha * (newOffZ - curOff.z);

    // Apply (keeps soft-iron & ranges intact)
    mag.updateHardIronOffset(smoothX, smoothY, smoothZ);
    mag.setCalibrationData(cal);

    // Reset min/max for next window
    magRawMin[0] = magRawMin[1] = magRawMin[2] = 30000;
    magRawMax[0] = magRawMax[1] = magRawMax[2] = -30000;
}


void printIMUPacket(ImuDataPacket_t data)
{
    Serial.print("Time stamp: "); Serial.println(data.timestamp_us);
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

    Serial.println();    
}


void sendVisualizationData(ImuDataPacket_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}


void calibrateMagnetometer2D(int duration) {
    Serial.println("2D Mag Cal: Spin robot 360° slowly in place...");
    mag.startCalibration();
    unsigned long start = millis();
    while(millis() - start < (duration / 2.0)) {
        mag.updateCalibration();
        analog_turn_l(35);
        delay(10);
    }
    halt();
    start = millis();
    while(millis() - start < (duration / 2.0)) {
        mag.updateCalibration();
        analog_turn_r(35);
        delay(10);
    }
    halt();
    mag.endCalibration();
    Serial.println("Calibration done."); 
}


void analog_turn_l(int dutyCycle)
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}

void analog_turn_r(int dutyCycle)
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  
  motorA.write(dutyCycle);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  motorB.write(dutyCycle);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(dutyCycle);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  motorD.write(dutyCycle);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}


void halt()
{
  digitalWrite(MOTOR_STBY_PIN, LOW);
  
  motorA.write(0);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], LOW);

  motorB.write(0);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], LOW);

  motorC.write(0);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], LOW);

  motorD.write(0);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], LOW);
}


bool initMotorDrivers()
{
  if(!motorA.attach())
  {
    return false;
  }
  if(!motorB.attach())
  {
    return false;
  }
  if(!motorC.attach())
  {
    return false;
  }
  if(!motorD.attach())
  {
    return false;
  }
  for(uint8_t i = 1; i < 3; i++)
  {
    pinMode(motorAcontrolpins[i], OUTPUT);
    pinMode(motorBcontrolpins[i], OUTPUT);
    pinMode(motorCcontrolpins[i], OUTPUT);
    pinMode(motorDcontrolpins[i], OUTPUT);
  }
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  return true;
}
