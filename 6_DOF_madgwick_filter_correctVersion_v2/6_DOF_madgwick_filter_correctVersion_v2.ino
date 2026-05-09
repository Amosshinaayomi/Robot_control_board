// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include <Wire.h>
#include "BMI160.h" // Your driver header
#include "QMC5883.h"
#include "Fusion.h" // The x-io Fusion library header


#define SDA_PIN PB7
#define SCL_PIN PB6
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


// Temperature variables
float referenceTemperature = 25.0f;
float currentTemperature = 25.0f;
float temperatureAlpha = 0.1f; //Low-pass filter coefficient for temperature
bool temperatureCompEnabled = true;
unsigned long lastTempUpdate = 0;
const unsigned long TEMP_UPDATE_INTERVAL = 500000; // 100ms in microseconds




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

// ==== 2. INITIALIZATION FUNCTION ====
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while(!Serial){}

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

    // Set Output Data Rates (ODR)
    imu.setAccelRate(0x0B); // 800Hz - High rate for responsive correction
    imu.setGyroRate(0x0B);  // 800Hz - Can be slightly lower than accelerometer

    // Check if gyro is in normal mode (required for temp sensor at 100Hz)

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
    

    if(!mag.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("Failed to connect to QMC5883P!");
        while(1);
    }
    Serial.println("QMC5883P Connected");
    // Configure magnetometer
    // QMC5883::Config magConfig = {
    //     .mode = QMC5883::MODE_CONTINUOUS,
    //     .odr = QMC5883::ODR_200Hz,      // Match gyro ODR (400Hz) as close as possible
    //     .range = QMC5883::RNG_8G,       // ±8 Gauss – good for earth field
    //     .osr = QMC5883::OSR_512,        // Highest oversampling for best accuracy
    //     .setReset = QMC5883::SET_RESET_ON
    // };
    // mag.configure(magConfig);

    // Set orientation to match your IMU's coordinate convention (NWU)
    // This depends on physical mounting; see Section 2.
    mag.setOrientation(QMC5883::ORIENTATION_NORMAL);

    // Optional: Set magnetic declination for your location (e.g., 10.5° East)
    mag.setDeclination(10.5f);
    
    // ---- C. CONFIGURE FUSION AHRS SETTINGS ----
    FusionAhrsSettings settings;
    settings.gain = 0.7f;               // Slightly higher than default 0.5 for better drift correction on a robot
    settings.accelerationRejection = 10.0f; // Degrees. Lower threshold for a robot that may accelerate smoothly.
    settings.gyroscopeRange = 500.0f;   // MUST match the dps setting above for correct over-range detection
    settings.recoveryTriggerPeriod = 50; // 5 seconds in samples (using 800Hz accel ODR)

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


    
    // Monitor filter health

    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    static int rejectCount = 0;
    if (states.accelerometerIgnored) rejectCount++;

    // ---- H. PERIODIC OUTPUT ----
    if(millis() - last_print_millis >= 50) {
        // Invert gyro axes for display (if needed)
        printIMUPacket(imuData);
        sendVisualizationData(imuData);
        Serial.print("Accel rejection count: "); Serial.println(rejectCount);
        last_print_millis = millis();        
    }
    // Control loop rate
    delay(5);
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