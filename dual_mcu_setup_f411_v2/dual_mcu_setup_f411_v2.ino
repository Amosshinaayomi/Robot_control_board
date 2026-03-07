// ==== 1. INCLUDES AND GLOBAL OBJECTS ====
#include "AHRS.h"

#include <EncoderManager.h>
#include <motor_control.h>

// Create encoder manager instance
EncoderManager encoderManager;
AHRS ahrs(SDA_PIN, SCL_PIN, 4000000UL);

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


long last_print_millis = millis();

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while(!Serial){}

    // Initialize AHRS (IMU + Magnetometer) 
    while(!ahrs.begin()) {
      Serial.println("AHRS initialization failed! Halt.");
      while (true);
    }
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
  
  ahrs.update();   // call as often as possible
  // Serial.print("Pitch is "); Serial.println(ahrs.getPitch());
  // Serial.print("Roll is "); Serial.println(ahrs.getRoll());
  // Serial.print("Yaw is "); Serial.println(ahrs.getYaw());
  Serial.printf("Pitch is %.2f\nRoll is %.2f\nYaw is %.2f\n",
   ahrs.getPitch(), ahrs.getRoll(), ahrs.getYaw()
  );
  if(millis() - lastPrint >= 30) {
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
    motor_speed = motor_speed % 110;    
    if(motor_speed == 0)
    {
      motor_speed = 20;
    }
    Serial.printf("directionState is %i\n", directionState);
    lastPrint = millis();
  }
  if(millis() - motorChangeMillis >= 2700)
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