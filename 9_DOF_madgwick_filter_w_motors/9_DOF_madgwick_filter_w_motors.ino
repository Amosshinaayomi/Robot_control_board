// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include <Wire.h>
#include "STMPWMTimer.h"
#include "BMI160.h" // Your driver header
#include "QMC5883.h"  
#include "Fusion.h" // The x-io Fusion library header
#include <EEPROM.h>

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



bool initMotorDrivers();
void analog_turn_l(int dutyCycle);
void analog_turn_r(int dutyCycle);


void headingToCardinal(float heading) {
    String directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                          "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((heading + 11.25) / 22.5) % 16;
    Serial.printf("Heading is %s\n", directions[index]);
}



// ==== 2. INITIALIZATION FUNCTION ====
void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    // delay(100);
    while(!Serial) {delay(50);}
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
    delay(50); // Give time for power mode transition

    // Set Output Data Rates (ODR)
    imu.setAccelRate(0x0B); // 800Hz - High rate for responsive correction
    imu.setGyroRate(0x0B);  // 400Hz - Can be slightly lower than accelerometer

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

    // // Try to read initial temperature
    // float temp;
    // int attempts = 0;
    // while (attempts < 10) {
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

    calibrateMagnetometer2D(12000);

    // Optional: Set magnetic declination for your location (e.g., 10.5° East)
    // mag.setDeclination(10.5f);

    // if (attempts >= 10) {
    //     // Serial.println("Could not read temperature sensor. Using default 25°C.");
    //     referenceTemperature = 25.0f;
    //     currentTemperature = 25.0f;
    // } else {
    //     currentTemperature = referenceTemperature;
    // }
    
    // ---- C. CONFIGURE FUSION AHRS SETTINGS ----
    FusionAhrsSettings settings;
    settings.gain = 0.7f;  //0.75              // Slightly higher than default 0.5 for better drift correction on a robot
    settings.accelerationRejection = 13.0f; // Degrees. Lower threshold for a robot that may accelerate smoothly.
    settings.gyroscopeRange = 500.0f;   // MUST match the dps setting above for correct over-range detection
    settings.recoveryTriggerPeriod = 150; // 5 seconds in samples (using 00 Hz update rate)
    settings.magneticRejection = 20.0f;
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
    FusionOffsetInitialise(&offsetFilter, 800); // Sample rate must match your gyro ODR (400Hz).

    previousMicros = micros();
    Serial.println("IMU and Fusion Library Initialized.");
}

long last_print_millis = millis();
float currentVelX, currentVelY, currentVelZ, prevVelX, prevVelY, prevVelZ, posX, posY, posZ;

// ==== 3. MAIN PROCESSING LOOP ====
volatile unsigned int updateRate = 0;
void loop() {
    // ---- A. TIMING ----
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - previousMicros) / 1000000.0f; // Convert to seconds
    previousMicros = currentMicros;

    // Safety check
    // if (deltaTime > 0.1f) {
    //     return;
    // }
    
    // // === TEMPERATURE READING (every 500ms) ===
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
    // ---- READ MAGNETOMETER ----
    QMC5883::Data magData;
    bool magValid = mag.readCalibrated(magData, QMC5883::RNG_8G);
    FusionVector magnetometer;
    // Serial.print("Raw X:"); Serial.print(magData.x);
    // Serial.print(" Y:"); Serial.print(magData.y);
    // Serial.print(" Z:"); Serial.println(magData.z);
    if (magValid) {
        // Convert Gauss to µT (1 Gauss = 100 µT)
        magnetometer.axis.x = magData.x * 100.0f;
        magnetometer.axis.y = magData.y * 100.0f;
        magnetometer.axis.z = magData.z * 100.0f;
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
    if(millis() - last_print_millis >= 50) {
        // Invert gyro axes for display (if needed)
        printIMUPacket(imuData);
        sendVisualizationData(imuData);
        Serial.printf("updateRate count is %i\n", updateRate);
        headingToCardinal(imuData.yaw);
        last_print_millis = millis();
    }
    
    // Monitor filter health
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    if (states.accelerometerIgnored) {
        Serial.println("[Warning] High Acceleration - Accel data ignored.");
    }

    // Control loop rate
    // delay(2);
    updateRate++;
    updateRate = updateRate % 500;
    delayMicroseconds(2000);
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