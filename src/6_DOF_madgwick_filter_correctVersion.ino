// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include <Wire.h>
#include "BMI160.h" // Your driver header
#include "Fusion.h" // The x-io Fusion library header

// Sensor and Fusion Objects
BMI160 imu; // Using default I2C address 0x69
FusionAhrs ahrsFilter;
FusionOffset offsetFilter; // For gyroscope run-time offset correction

// Timestamp for delta-time calculation
unsigned long previousMicros = 0;

float accelBias[3] = {0, 0, 0}; // Store biases for X, Y, Z
float gyroBias[3] = {0, 0, 0};


static FusionConvention convention = FusionConventionNwu;
FusionVector CustomAxesSwapPXNY(const FusionVector sensor)
{
    FusionVector result;
    result.axis.x = sensor.axis.x;
    result.axis.y = -sensor.axis.y;
    result.axis.z = sensor.axis.z;
    return result;
}
// ==== 2. INITIALIZATION FUNCTION ====
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while(!Serial)
    {
      
    }
    Wire.begin(PB9, PB8); // Your specified I2C pins

    // Initialize BMI160
    if (!imu.begin(PB9, PB8, 4000000UL)) {
        Serial.println("Failed to connect to BMI160!");
        while (true); // Halt
    }
    Serial.println("BMI160 Connected.");

    // ---- A. CONFIGURE SENSOR RANGES AND RATES ----
    // Set ranges (affects resolution and dynamic range)
    imu.setAccelRange(4); // ±4g - Good for ground robot dynamics
    imu.setGyroRange(500); // ±500 degrees/sec - Adequate for turning

    // Set Output Data Rates (ODR)
    imu.setAccelRate(0x0B); // 800Hz - High rate for responsive correction
    imu.setGyroRate(0x0A);  // 400Hz - Can be slightly lower than accelerometer

    // ---- B. PERFORM STARTUP CALIBRATION ----
    Serial.println("Calibrating Gyroscope (keep robot still)...");
    imu.calibrateGyro(500, &gyroBias[0], &gyroBias[1], &gyroBias[2]);

    Serial.println("Calibrating Accelerometer (keep robot level)...");
    imu.calibrateAccel(300, &accelBias[0], &accelBias[1], &accelBias[2]);

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

    previousMicros = micros();
    Serial.println("IMU and Fusion Library Initialized.");
}

long last_print_millis = millis();
// ==== 3. MAIN PROCESSING LOOP ====
void loop() {
    // ---- A. TIMING ----
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - previousMicros) / 1000000.0f; // Convert to seconds
    previousMicros = currentMicros;

    // Safety check: if deltaTime is too large (e.g., after a pause), skip.
    if (deltaTime > 0.1f) {
        return;
    }

    // ---- B. READ RAW SENSOR DATA ----
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (!imu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw)) {
        return; // Skip this loop if read failed
    }

    // ---- C. APPLY SENSOR CALIBRATION (Convert to Physical Units) ----
    // The Fusion library expects measurements in g's and degrees/sec.
    // BMI160 LSB values for ±4g and ±500dps ranges:
    // Use the public getters to get dynamic scaling
    const float accelScale = imu.getAccelScaleG();
    const float gyroScale  = imu.getGyroScaleDps();

    // 1. SUBTRACT BIASES IN RAW COUNTS (LSB) FIRST
    float ax_cal = (float)axRaw - accelBias[0];
    float ay_cal = (float)ayRaw - accelBias[1];
    float az_cal = (float)azRaw - accelBias[2]; // Bias already has 1g subtracted

    float gx_cal = (float)gxRaw - gyroBias[0];
    float gy_cal = (float)gyRaw - gyroBias[1];
    float gz_cal = (float)gzRaw - gyroBias[2];


    // 2. THEN CONVERT TO PHYSICAL UNITS (g and dps)
    float ax_g = ax_cal * accelScale;
    float ay_g = ay_cal * accelScale;
    float az_g = az_cal * accelScale; // Should be ~1.0g when level

    float gx_dps = gx_cal * gyroScale;
    float gy_dps = gy_cal * gyroScale;
    float gz_dps = gz_cal * gyroScale; // Should be ~0 dps when stationary
    // Serial.print("ay_g is "); Serial.println(ay_g);
    // Serial.print("gy_dps is "); Serial.println(gy_dps);
    // 3. CREATE FUSIONVECTOR STRUCTURES
    // FusionVector accelerometer = {ax_g, ay_g, az_g};
    // FusionVector gyroscope = {gx_dps, gy_dps, gz_dps};
    // FusionVector accelerometer = {ay_g, -ax_g, az_g};  // Y physical -> X software, invert X physical for left
    // FusionVector gyroscope = {gy_dps, -gx_dps, gz_dps}; // Must apply the SAME swap to gyro
    
    // 3. Create sensor frame vectors
    FusionVector accelerometer = {ax_g, ay_g, az_g};
    FusionVector gyroscope = {gx_dps, gy_dps, gz_dps};
    // ---- D. SWAP AXES TO CORRECT NWU CONVENTION ----
    // Current mapping: Sensor Y -> Forward, Sensor X -> Right, Sensor Z -> Up
    // We want NWU: X=Forward, Y=Left, Z=Up
    // Based on your tests: body X = sensor Y, body Y = -sensor X, body Z = sensor Z
    // This corresponds to FusionAxesAlignmentPYNXPZ
    // FusionVector accelerometerBody = FusionAxesSwap(accelerometer, FusionAxesAlignmentNXNYPZ);
    // FusionVector gyroscopeBody = FusionAxesSwap(gyroscope, FusionAxesAlignmentNXNYPZ); //None of the provided alignment worked 
    // so i implemented mine below which also didn't work well my x axis true value is inverted

    // FusionVector accelerometerBody = CustomAxesSwapPXNY(accelerometer);
    // FusionVector gyroscopeBody = CustomAxesSwapPXNY(gyroscope);
    // DEBUG: Check which axis is measuring gravity

    // ---- D. UPDATE GYROSCOPE OFFSET CORRECTION ----
    // This refines the gyroscope bias in real-time during stationary periods.
    gyroscope = FusionOffsetUpdate(&offsetFilter, gyroscope);

    // ---- E. UPDATE THE AHRS FILTER ----
    // This is the core fusion step. For 6DOF, we pass only gyro and accel.
    FusionAhrsUpdateNoMagnetometer(&ahrsFilter, gyroscope, accelerometer, deltaTime);

    // ---- F. GET AND USE THE OUTPUT ----
    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrsFilter);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    float roll = euler.angle.roll;
    float pitch = euler.angle.pitch;
    float yaw = euler.angle.yaw; // NOTE: Without a magnetometer, this will drift.


    // 3. Get other useful outputs
    FusionVector gravity = FusionAhrsGetGravity(&ahrsFilter);
    // FusionVector linearAcc = FusionAhrsGetLinearAcceleration(&ahrsFilter);
    
    // 4. Get and correct linear acceleration (in sensor frame)
    FusionVector linearAcc= FusionAhrsGetLinearAcceleration(&ahrsFilter);
    
    if(millis() - last_print_millis >= 200)
    {
        Serial.print("Accel Raw: ");
        Serial.print(ax_g, 3); Serial.print(", ");
        Serial.print(ay_g, 3); Serial.print(", ");
        Serial.println(az_g, 3); 
        // Serial.print("Body Accel x, y, z: ");
        // Serial.print(accelerometerBody.axis.x, 3); Serial.print(", ");
        // Serial.print(accelerometerBody.axis.y, 3); Serial.print(", ");
        // Serial.println(accelerometerBody.axis.z, 3);


        Serial.print("Gyro (dps) x, y, z: ");
        Serial.print(gx_dps, 3); Serial.print(", ");
        Serial.print(gy_dps, 3); Serial.print(", ");
        Serial.println(-gz_dps, 3);
        // ---- G. OUTPUT DATA (Example: Send via Serial) ----
        Serial.print("Roll:"); Serial.println(-roll, 1);
        Serial.print("Pitch:"); Serial.println(pitch, 1);
        Serial.print("Yaw:"); Serial.println(-yaw, 1);
        Serial.print("LinAcc X:"); Serial.println(linearAcc.axis.x, 3);
        Serial.print("LinAcc Y:"); Serial.println(linearAcc.axis.y, 3);
        Serial.print("LinAcc Z:"); Serial.println(-linearAcc.axis.z, 3);
        last_print_millis = millis();
    } 

    // Optional: Monitor filter health via internal states
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    if (states.accelerometerIgnored) {
        Serial.println("[Warning] High Acceleration - Accel data ignored.");
    }

    // Control loop rate. Aim for ~2x your gyro ODR (e.g., 400Hz -> 200Hz loop).
    delay(5);
}
