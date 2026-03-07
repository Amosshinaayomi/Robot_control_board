// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include <Wire.h>
#include "BMI160.h" // Your driver header
#include <QMC5883.h>
#include "Fusion.h" // The x-io Fusion library header
#include "STMPWMTimer.h"
#include <EncoderManager.h>
#include <pins.h>
#include <motor_control.h>

// Create encoder manager instance
EncoderManager encoderManager;

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


long last_print_millis = millis();
float currentVelX, currentVelY, currentVelZ, prevVelX, prevVelY, prevVelZ, posX, posY, posZ;


bool setupMag() {
  if(mag.begin(SDA, SCL)) {
    Serial.println("QMC5883 initialized successfully");
    Serial.print("Chip ID: 0x");
    Serial.println(mag.getChipID(), HEX);
    
    // Perform self-test
    if (mag.selfTest()) {
        Serial.println("Self-test PASSED");
    } else {
        Serial.println("Self-test FAILED");
        return false;
    }
    // Set your local magnetic declination
    mag.setDeclination(7.4); 
    // Set orientation (adjust based on your sensor mounting)
    mag.setOrientation(QMC5883::ORIENTATION_ROTATE_270);
    mag.wakeup();
    return true;  
  } else {
      Serial.println("Failed to initialize QMC5883");
      return false;
  }
}

String headingToCardinal(float heading) {
    String directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                          "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((heading + 11.25) / 22.5) % 16;
    return directions[index];
}

void processAHRS()
{

}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    // while(!Serial){}

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

    // Attach all motors (automatically manages timer sharing!)
    initMotorDrivers();

    // Setup encoders
    encoderManager.addEncoder(ENCODER_PIN_1);
    encoderManager.addEncoder(ENCODER_PIN_2);
    encoderManager.addEncoder(ENCODER_PIN_3);
    encoderManager.addEncoder(ENCODER_PIN_4);
    // Debug info    
    STMPWMTimer::debug();

    // Start polling at 100kHz (10µs intervals)
    if(encoderManager.begin(100000, TIM2)) {
      Serial.println("Encoder manager started successfully");
    } else {
      Serial.println("Failed to start encoder manager");
    }
    encoderManager.printDebugInfo();    
}




long currentMillis = millis();
int switchInterval = 1000;
uint8_t currentState = 0;
uint8_t motionStates = 3;

static unsigned long lastPrint;
uint8_t directionState;
uint8_t motor_speed = 20;
long motorChangeMillis = millis();
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
        // printIMUPacket(imuData);
        sendVisualizationData(imuData);
        last_print_millis = millis();
    }
    
    // Monitor filter health
    FusionAhrsInternalStates states = FusionAhrsGetInternalStates(&ahrsFilter);
    if (states.accelerometerIgnored) {
        Serial.println("[Warning] High Acceleration - Accel data ignored.");
    }
  

  QMC5883::Data data;
  float magnetic_heading, true_heading;
  
  // Read all data in one call
  // if (mag.getAllData(data, magnetic_heading, true_heading)) {
  //     // Print raw sensor data
  //     Serial.print("Magnetic Field - ");
  //     Serial.print("X: "); Serial.print(data.x, 4); Serial.print(" G, ");
  //     Serial.print("Y: "); Serial.print(data.y, 4); Serial.print(" G, ");
  //     Serial.print("Z: "); Serial.print(data.z, 4); Serial.println(" G");
      
  //     // Print heading information
  //     Serial.print("Heading - ");
  //     Serial.print("Magnetic: "); Serial.print(magnetic_heading, 1);
  //     Serial.print("°, True: "); Serial.print(true_heading, 1);
  //     Serial.print("°, Direction: "); Serial.println(headingToCardinal(true_heading));
      
  //     Serial.println("---");
  // } else {
  //     Serial.println("✗ Failed to read sensor data");
  // }


  // Print encoder ticks every second
  if(millis() - lastPrint >= 60) {
    if(directionState == 0)
    {
      analog_move_f(motor_speed);
      Serial.printf("motor speed is %i\n", motor_speed);
    } else if (directionState == 1)
    {
      analog_move_b(motor_speed);
      Serial.printf("motor speed is %i\n", motor_speed);
    } else if (directionState == 2)
    {
      analog_turn_l(motor_speed);
      Serial.printf("motor speed is %i\n", motor_speed);   
    } else if (directionState == 3)
    {
      analog_turn_r(motor_speed);
      Serial.printf("motor speed is %i\n", motor_speed);   
    } else if (directionState == 4)
    {
      analog_move_f(20);
      Serial.println("Halt");
    } 
    encoderManager.printAllTicks();
    motor_speed = motor_speed + 1;
    motor_speed = motor_speed % 71;    
    if(motor_speed == 0)
    {
      motor_speed = 20;
    }
    Serial.printf("directionState is %i\n", directionState);
    lastPrint = millis();
  }
  if(millis() - motorChangeMillis >= 3600)
  {
    directionState++;
    directionState = directionState % 5;
    motorChangeMillis = millis();
  }
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

void readModeFromSerial()
{
  if(Serial.available() > 0)
  {
    String serialInput = Serial.readStringUntil('\n');
    char CHAR = serialInput[0];
    CHAR = toupper(CHAR);
    if(CHAR == 'F' || CHAR == 'B' || CHAR == 'L' || CHAR == 'R' || CHAR == 'N')
    {
      // controlMode = CHAR;   
      uint8_t speed = (serialInput.substring(1)).toInt();
      Serial.printf("speed: %i\n", speed);
      return;
    }
    Serial.println("Please Enter the Character F or B or L or R to control the motors.");
  }
}


void sendVisualizationData(ImuDataPacket_t data)
{
    Serial.print("PITCH:"); Serial.print(data.pitch);
    Serial.print(",ROLL:"); Serial.print(data.roll);
    Serial.print(",YAW:"); Serial.println(data.yaw);       
}

