/*!
 * @file 6DOF_IMU_Madgwick.ino
 * @brief BMI160 6DOF IMU with Madgwick Filter - ACCEL + GYRO ONLY
 */

#include <BMI160.h>
#include "Fusion.h"
#include <Wire.h>
#include <QMC5883.h>
// Sensor object
BMI160 bmi160;

// I2C pins
const uint8_t SDA_PIN = PB9;
const uint8_t SCL_PIN = PB8;

// =============================================
// FUSION LIBRARY SETUP
// =============================================
FusionAhrs ahrs;
FusionOffset offset;
#define SAMPLE_RATE_HZ (200.0f)  // Stable rate for 6DOF

// =============================================
// SENSOR ORIENTATION CONFIGURATION
// =============================================
enum SensorOrientation {
    ORIENTATION_NORMAL,
    ORIENTATION_ROTATE_90,
    ORIENTATION_ROTATE_180, 
    ORIENTATION_ROTATE_270
};

SensorOrientation bmi160_orientation = ORIENTATION_NORMAL;

// =============================================
// GLOBAL VARIABLES
// =============================================

// Orientation variables
float pitch = 0, roll = 0, yaw = 0;

// Sensor data
float ax = 0, ay = 0, az = 0; // Accelerometer (g)
float gx = 0, gy = 0, gz = 0; // Gyroscope (rad/s)

// Timing
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
unsigned long loopCount = 0;
float actualUpdateRate = 0;

// Performance monitoring
unsigned long totalReadTime = 0;
unsigned long totalProcessTime = 0;

// Calibration
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

// Update intervals
const int PRINT_INTERVAL = 200; // 5Hz printing

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("6DOF IMU - BMI160 + Madgwick Filter (Accel + Gyro Only)");
    Serial.println("Initializing BMI160...");
    
    // Initialize BMI160
    if (!bmi160.begin()) {
        Serial.println("BMI160 initialization failed!");
        while(1) {
            Serial.println("Check BMI160 wiring and try again!");
            delay(1000);
        }
    }
    Serial.println("BMI160 initialized successfully!");
    
    // Initialize Fusion library for 6DOF
    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, SAMPLE_RATE_HZ);
    
    // Optimized settings for 6DOF
    FusionAhrsSettings settings;
    settings.convention = FusionConventionNwu;
    settings.gain = 0.3f;                    // Lower gain for 6DOF stability
    settings.gyroscopeRange = 2000.0f;       // degrees/s
    settings.accelerationRejection = 30.0f;  // Higher rejection for accel-only
    settings.magneticRejection = 0.0f;       // Not used
    settings.recoveryTriggerPeriod = 0;      // Disabled for 6DOF
    FusionAhrsSetSettings(&ahrs, &settings);
    
    // Calibrate sensors
    calibrateSensors();
    
    lastTime = micros();
    lastPrintTime = millis();
    
    Serial.println("6DOF IMU Ready! (Magnetometer DISABLED)");
    Serial.println("NOTE: Yaw will drift over time without magnetometer");
    Serial.println("Pitch\tRoll\tYaw\tRate\tAccelZ");
}

void loop() {
    int16_t rawAccelGyro[6];
    
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f;
    
    // Ensure minimum delta time
    if (dt <= 0 || dt > 0.1f) {
        dt = 1.0f / SAMPLE_RATE_HZ;
    }
    lastTime = currentTime;
    
    // MEASURE READ TIME
    unsigned long readStart = micros();
    
    // Read BMI160
    bool imuSuccess = bmi160.getMotion6(&rawAccelGyro[0], &rawAccelGyro[1], &rawAccelGyro[2], 
                                       &rawAccelGyro[3], &rawAccelGyro[4], &rawAccelGyro[5]);
    
    unsigned long readTime = micros() - readStart;
    
    if (imuSuccess) {
        loopCount++;
        totalReadTime += readTime;
        
        // MEASURE PROCESS TIME
        unsigned long processStart = micros();
        
        // Apply calibration and convert units
        applyCalibration(rawAccelGyro);
        
        // Convert accelerometer to m/s² for Fusion
        FusionVector accelerometer = {ax * 9.80665f, ay * 9.80665f, az * 9.80665f};
        
        // Gyroscope data (already in rad/s from applyCalibration)
        FusionVector uncalibratedGyro = {gx, gy, gz};
        
        FusionVector calibratedGyro = FusionOffsetUpdate(&offset, uncalibratedGyro);
        
        // 6DOF update (accelerometer + gyroscope only)
        FusionAhrsUpdateNoMagnetometer(&ahrs, calibratedGyro, accelerometer, dt);
        
        // Get orientation
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        
        // Convert to degrees and store
        pitch = euler.angle.pitch;
        roll = euler.angle.roll; 
        yaw = euler.angle.yaw;
        
        // Normalize angles
        if (yaw > 180.0f) yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;
        
        unsigned long processTime = micros() - processStart;
        totalProcessTime += processTime;
        
        // Print data
        if (millis() - lastPrintTime > PRINT_INTERVAL) {
            float actualTimeElapsed = (millis() - lastPrintTime) / 1000.0f;
            actualUpdateRate = loopCount / actualTimeElapsed;
            
            unsigned long avgReadTime = totalReadTime / loopCount;
            unsigned long avgProcessTime = totalProcessTime / loopCount;
            
            Serial.print("Pitch:"); Serial.print(pitch, 1); 
            Serial.print("\tRoll:"); Serial.print(roll, 1);
            Serial.print("\tYaw:"); Serial.print(yaw, 1);
            Serial.print("\tRate:"); Serial.print(actualUpdateRate, 0);
            Serial.print("Hz\tAccelZ:"); Serial.println(az, 2);
            
            // Detailed sensor data every 2 seconds
            static unsigned long lastDebug = 0;
            if (millis() - lastDebug > 2000) {
                Serial.print("RAW - Accel:"); 
                Serial.print(rawAccelGyro[0]); Serial.print(",");
                Serial.print(rawAccelGyro[1]); Serial.print(",");
                Serial.print(rawAccelGyro[2]);
                
                Serial.print(" Gyro:"); 
                Serial.print(rawAccelGyro[3]); Serial.print(",");
                Serial.print(rawAccelGyro[4]); Serial.print(","); 
                Serial.println(rawAccelGyro[5]);
                
                sendExternalData();
                lastDebug = millis();
            }
            
            // Reset counters
            loopCount = 0;
            totalReadTime = 0;
            totalProcessTime = 0;
            lastPrintTime = millis();
        }
        
    } else {
        Serial.println("BMI160 read error");
    }
    
    checkSerialCommands();
}

void applyCalibration(int16_t* rawData) {
    // Convert accelerometer to g and apply calibration
    // BMI160 returns: [ax, ay, az, gx, gy, gz]
    ax = (rawData[0] / 16384.0f) - accelBiasX;
    ay = (rawData[1] / 16384.0f) - accelBiasY; 
    az = (rawData[2] / 16384.0f) - accelBiasZ;
    
    // Apply orientation (rotate 180)
    ax = -ax;
    ay = -ay;
    // Z remains same
    
    // Convert gyroscope to rad/s and apply calibration
    gx = ((rawData[3] / 131.0f) * (PI / 180.0f)) - gyroBiasX;
    gy = ((rawData[4] / 131.0f) * (PI / 180.0f)) - gyroBiasY;
    gz = ((rawData[5] / 131.0f) * (PI / 180.0f)) - gyroBiasZ;
    
    // Apply orientation (rotate 180)
    gx = -gx;
    gy = -gy;
    // Z remains same
}

void sendExternalData() {
    Serial.print("ORIENTATION:");
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll); Serial.print(","); 
    Serial.print(yaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx * (180.0/PI)); Serial.print(","); // Convert to deg/s for output
    Serial.print(gy * (180.0/PI)); Serial.print(",");
    Serial.println(gz * (180.0/PI));
}

void calibrateSensors() {
    Serial.println("=== 6DOF SENSOR CALIBRATION ===");
    
    calibrateAccelerometer();
    calibrateGyroscope();
    
    Serial.println("=== CALIBRATION COMPLETE ===");
}

void calibrateAccelerometer() {
    Serial.println("Calibrating accelerometer... Keep sensor FLAT and STILL");
    delay(3000);
    
    float sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 500;
    int16_t rawData[6];
    
    for (int i = 0; i < samples; i++) {
        if (bmi160.getMotion6(&rawData[0], &rawData[1], &rawData[2], &rawData[3], &rawData[4], &rawData[5])) {
            // For ORIENTATION_ROTATE_180, we invert X and Y in the final calculation
            sumX += rawData[0] / 16384.0f;
            sumY += rawData[1] / 16384.0f;
            sumZ += rawData[2] / 16384.0f;
        }
        delay(10);
    }
    
    // Account for 180° rotation in bias calculation
    accelBiasX = (-sumX / samples);  // Inverted for 180 rotation
    accelBiasY = (-sumY / samples);  // Inverted for 180 rotation  
    accelBiasZ = (sumZ / samples) - 1.0f; // Expecting 1g
    
    Serial.print("Accel Bias - X:"); Serial.print(accelBiasX, 4);
    Serial.print(" Y:"); Serial.print(accelBiasY, 4);
    Serial.print(" Z:"); Serial.println(accelBiasZ, 4);
    
    // Verify calibration
    Serial.print("Expected flat values - X:0.00 Y:0.00 Z:1.00");
    Serial.print(" Actual - X:"); Serial.print(-accelBiasX, 2);
    Serial.print(" Y:"); Serial.print(-accelBiasY, 2); 
    Serial.print(" Z:"); Serial.println(1.0 + accelBiasZ, 2);
}

void calibrateGyroscope() {
    Serial.println("Calibrating gyroscope... Keep sensor PERFECTLY STILL");
    delay(2000);
    
    float sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 500;
    int16_t rawData[6];
    
    for (int i = 0; i < samples; i++) {
        if (bmi160.getMotion6(&rawData[0], &rawData[1], &rawData[2], &rawData[3], &rawData[4], &rawData[5])) {
            sumX += (rawData[3] / 131.0f) * (PI / 180.0f);
            sumY += (rawData[4] / 131.0f) * (PI / 180.0f);
            sumZ += (rawData[5] / 131.0f) * (PI / 180.0f);
        }
        delay(10);
    }
    
    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples; 
    gyroBiasZ = sumZ / samples;
    
    Serial.print("Gyro Bias - X:"); Serial.print(gyroBiasX * (180.0f/PI), 4);
    Serial.print("°/s Y:"); Serial.print(gyroBiasY * (180.0f/PI), 4);
    Serial.print("°/s Z:"); Serial.print(gyroBiasZ * (180.0f/PI), 4); Serial.println("°/s");
    
    // Verify calibration quality
    float biasMagnitude = sqrt(gyroBiasX*gyroBiasX + gyroBiasY*gyroBiasY + gyroBiasZ*gyroBiasZ) * (180.0f/PI);
    Serial.print("Bias Magnitude: "); Serial.print(biasMagnitude, 4); Serial.println("°/s");
    if (biasMagnitude > 1.0) {
        Serial.println("WARNING: High gyro bias - ensure sensor was perfectly still during calibration!");
    }
}

void checkSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'r':
                // Reset orientation and clear any drift
                FusionAhrsReset(&ahrs);
                Serial.println("Orientation RESET - yaw set to 0");
                break;
            case 'c':
                // Recalibrate sensors
                calibrateSensors();
                break;
            case 's':
                // Print current sensor status
                Serial.println("=== SENSOR STATUS ===");
                Serial.print("Accel - X:"); Serial.print(ax, 3);
                Serial.print(" Y:"); Serial.print(ay, 3);
                Serial.print(" Z:"); Serial.println(az, 3);
                Serial.print("Gyro - X:"); Serial.print(gx * (180.0/PI), 2);
                Serial.print("°/s Y:"); Serial.print(gy * (180.0/PI), 2);
                Serial.print("°/s Z:"); Serial.print(gz * (180.0/PI), 2); Serial.println("°/s");
                break;
            case 'z':
                // Zero the yaw (useful for relative measurements)
                FusionAhrsReset(&ahrs);
                Serial.println("Yaw zeroed - current orientation set as reference");
                break;
        }
    }
}

// Test function to check sensor response
void testSensorResponse() {
    Serial.println("=== SENSOR RESPONSE TEST ===");
    Serial.println("Move sensor in different directions and observe values");
    Serial.println("Pitch should change when tilting forward/backward");
    Serial.println("Roll should change when tilting left/right"); 
    Serial.println("Yaw will drift over time (normal for 6DOF)");
    Serial.println("Press any key to stop test...");
    
    unsigned long startTime = millis();
    while (!Serial.available()) {
        int16_t rawData[6];
        if (bmi160.getMotion6(&rawData[0], &rawData[1], &rawData[2], &rawData[3], &rawData[4], &rawData[5])) {
            applyCalibration(rawData);
            
            FusionVector accelerometer = {ax * 9.80665f, ay * 9.80665f, az * 9.80665f};
            FusionVector gyroscope = {gx, gy, gz};
            
            float dt = 0.005f; // 200Hz
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
            
            FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            
            Serial.print("P:"); Serial.print(euler.angle.pitch, 1);
            Serial.print(" R:"); Serial.print(euler.angle.roll, 1); 
            Serial.print(" Y:"); Serial.print(euler.angle.yaw, 1);
            Serial.print(" AccZ:"); Serial.print(az, 2);
            Serial.println();
            
            delay(50); // 20Hz update for readability
        }
    }
    while (Serial.available()) Serial.read(); // Clear buffer
    Serial.println("Test stopped");
}