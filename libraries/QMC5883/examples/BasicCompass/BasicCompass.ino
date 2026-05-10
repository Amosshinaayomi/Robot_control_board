/**
 * Basic Compass Example for QMC5883 Magnetometer
 * 
 * This example shows basic usage of the QMC5883 library
 * to read magnetic field data and calculate heading.
 */

#include <QMC5883.h>

// Define your I2C pins here for your specific board
// Arduino Uno/Nano: SDA = A4, SCL = A5
// ESP32: Most GPIOs work, common: SDA = 21, SCL = 22  
// STM32: PB9, PB8 etc.
const uint8_t SDA_PIN = PB9;  // Change for your board
const uint8_t SCL_PIN = PB8;  // Change for your board

QMC5883 mag;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    Serial.println("QMC5883 Basic Compass Example");
    Serial.println("=============================");
    
    // Initialize sensor with I2C pins - REQUIRED
    if (mag.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("✓ QMC5883 initialized successfully");
        Serial.print("✓ Chip ID: 0x");
        Serial.println(mag.getChipID(), HEX);
    } else {
        Serial.println("✗ Failed to initialize QMC5883");
        while(1);
    }
    
    // Set magnetic declination for your location
    mag.setDeclination(-0.1f); // Example for locations near 0° declination
    
    // Optional: Set orientation if sensor is mounted rotated
    // mag.setOrientation(QMC5883::ORIENTATION_ROTATE_90);
    
    Serial.println("✓ Declination set");
    Serial.println("Ready! Press 's' to sleep, 'w' to wake up");
    Serial.println();
}

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
    
    delay(1000);
}

/**
 * Convert heading angle to cardinal direction
 */
String headingToCardinal(float heading) {
    String directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                          "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((heading + 11.25) / 22.5) % 16;
    return directions[index];
}