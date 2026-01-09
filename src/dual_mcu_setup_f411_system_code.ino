#include <QMC5883.h>
#include <EncoderManager.h>

// Create encoder manager instance
EncoderManager encoderManager;
QMC5883 mag;


#define MPU_ADDRESS 0x68
// --- 1. Configuration ---
#define ENCODER_PIN_1 PB2 // Any GPIO on ESP32  PB2(STM32F411)
#define ENCODER_PIN_2 PC15 // PC15(F411)
#define ENCODER_PIN_3 PB5 // PB5
#define ENCODER_PIN_4 PA15 // PA15

#define MOTOR_STBY_PIN PB12

#define UART_TX PA2
#define UART_RX PA3

#define SCL PB8
#define SDA PB9

#define LED_PIN PC13

uint8_t motorAcontrolpins[3] = {PA10, PA6, PA7};
uint8_t motorBcontrolpins[3] = {PA1, PA5, PA4};
uint8_t motorCcontrolpins[3] = {PA8, PB1, PB10};
uint8_t motorDcontrolpins[3] = {PA9, PB0, PB13};

HardwareSerial Serial2(PA3, PA2);  // RX, TX (PA3=RX, PA2=TX)

void handleSerialCommands();
void initMotorDriver();
void initIMU();
bool setupMag();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200);
  while(!Serial)
  {

  }
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Encoder Manager Library Test");
  
  // Add encoders
  encoderManager.addEncoder(ENCODER_PIN_1);
  encoderManager.addEncoder(ENCODER_PIN_2);
  encoderManager.addEncoder(ENCODER_PIN_3);
  encoderManager.addEncoder(ENCODER_PIN_4);

  // Start polling at 100kHz (10µs intervals)
  if(encoderManager.begin(100000, TIM2)) {
    Serial.println("Encoder manager started successfully");
  } else {
    Serial.println("Failed to start encoder manager");
  }

  encoderManager.printDebugInfo();

  // initialize motor drivers
  initMotorDriver();

  // IMU initialization
  // if(mag.begin(SDA, SCL)) {
  //   Serial.println("QMC5883 initialized successfully");
  //   Serial.print("Chip ID: 0x");
  //   Serial.println(mag.getChipID(), HEX);
    
  //   // Perform self-test
  //   if (mag.selfTest()) {
  //       Serial.println("Self-test PASSED");
  //   } else {
  //       Serial.println("Self-test FAILED");
  //   }
  // } else {
  //     Serial.println("Failed to initialize QMC5883");
  // }

  // // Set your local magnetic declination
  // mag.setDeclination(7.4); 

  // // Set orientation (adjust based on your sensor mounting)
  // mag.setOrientation(QMC5883::ORIENTATION_ROTATE_270);  
  // bool magSetupStatus = setupMag();
  // if(magSetupStatus)
  // {
  //   Serial.println("Mag setup successful");
  // } else {
  //   Serial.println("Mag setup failed");
  // }
  // Serial.println("Hello New world");
}


long currentMillis = millis();
int switchInterval = 1000;
uint8_t currentState = 0;
uint8_t motionStates = 3;

static unsigned long lastPrint;
void loop() {
  // Check for sleep/wake commands
  if (Serial.available()) {
      char command = Serial.read();
      if (command == 's' || command == 'S') {
          if (mag.sleep()) {
              Serial.println("Sensor put to sleep");
          }
      } else if (command == 'w' || command == 'W') {
          if (mag.wakeup()) {
              Serial.println("Sensor woke up");
          }
      }
  }
  
  if (mag.isAsleep()) {
      Serial.println("Sensor sleeping... press 'w' to wake");
      delay(2000);
      return;
  }
  // Print encoder ticks every second
  if(millis() - lastPrint > 1000) {
    encoderManager.printAllTicks();
    lastPrint = millis();
  } 

    QMC5883::Data data;
    float magnetic_heading, true_heading;
    
    // Read all data in one call
    if (mag.getAllData(data, magnetic_heading, true_heading)) {
        // Print raw sensor data
        Serial.print("Magnetic Field - ");
        Serial.print("X: "); Serial.print(data.x, 4); Serial.print(" G, ");
        Serial.print("Y: "); Serial.print(data.y, 4); Serial.print(" G, ");
        Serial.print("Z: "); Serial.print(data.z, 4); Serial.println(" G");
        
        // Print heading information
        Serial.print("Heading - ");
        Serial.print("Magnetic: "); Serial.print(magnetic_heading, 1);
        Serial.print("°, True: "); Serial.print(true_heading, 1);
        Serial.print("°, Direction: "); Serial.println(headingToCardinal(true_heading));
        
        Serial.println("---");
    } else {
        Serial.println("✗ Failed to read sensor data");
    }
    
    delay(500);
}

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

void initMotorDriver()
{
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  for(uint8_t i = 0; i < 3; i++)
  {
    pinMode(motorAcontrolpins[i], OUTPUT);
    pinMode(motorBcontrolpins[i], OUTPUT);
    pinMode(motorCcontrolpins[i], OUTPUT);
    pinMode(motorDcontrolpins[i], OUTPUT);
  }
  // ledcAttach(motorAcontrolpins[0], PWM_FREQ, pwm_resolution_bit);
  // ledcAttach(motorBcontrolpins[0], PWM_FREQ, pwm_resolution_bit);
  // ledcAttach(motorDcontrolpins[0], PWM_FREQ, pwm_resolution_bit);
  // ledcAttach(motorCcontrolpins[0], PWM_FREQ, pwm_resolution_bit);
}

void move_f() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(motorAcontrolpins[0], HIGH);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  digitalWrite(motorBcontrolpins[0], HIGH);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);

  digitalWrite(motorCcontrolpins[0], HIGH);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  digitalWrite(motorDcontrolpins[0], HIGH);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}


void move_b() {
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(motorAcontrolpins[0], HIGH);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  digitalWrite(motorBcontrolpins[0], HIGH);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  digitalWrite(motorCcontrolpins[0], HIGH);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  digitalWrite(motorDcontrolpins[0], HIGH);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}

void turn_l()
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(motorAcontrolpins[0], HIGH);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], HIGH);

  digitalWrite(motorBcontrolpins[0], HIGH);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], HIGH);

  digitalWrite(motorCcontrolpins[0], HIGH);
  digitalWrite(motorCcontrolpins[1], HIGH);
  digitalWrite(motorCcontrolpins[2], LOW);

  digitalWrite(motorDcontrolpins[0], HIGH);
  digitalWrite(motorDcontrolpins[1], HIGH);
  digitalWrite(motorDcontrolpins[2], LOW);
}

void turn_r()
{
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  digitalWrite(motorAcontrolpins[0], HIGH);
  digitalWrite(motorAcontrolpins[1], HIGH);
  digitalWrite(motorAcontrolpins[2], LOW);

  digitalWrite(motorBcontrolpins[0], HIGH);
  digitalWrite(motorBcontrolpins[1], HIGH);
  digitalWrite(motorBcontrolpins[2], LOW);


  digitalWrite(motorCcontrolpins[0], HIGH);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], HIGH);

  digitalWrite(motorDcontrolpins[0], HIGH);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], HIGH);
}
void halt()
{
  digitalWrite(MOTOR_STBY_PIN, LOW);
  digitalWrite(motorAcontrolpins[0], 0);
  digitalWrite(motorAcontrolpins[1], LOW);
  digitalWrite(motorAcontrolpins[2], LOW); 

  digitalWrite(motorBcontrolpins[0], 0);
  digitalWrite(motorBcontrolpins[1], LOW);
  digitalWrite(motorBcontrolpins[2], LOW); 

  digitalWrite(motorCcontrolpins[0], 0);
  digitalWrite(motorCcontrolpins[1], LOW);
  digitalWrite(motorCcontrolpins[2], LOW); 

  digitalWrite(motorDcontrolpins[0], 0);
  digitalWrite(motorDcontrolpins[1], LOW);
  digitalWrite(motorDcontrolpins[2], LOW); 
}
