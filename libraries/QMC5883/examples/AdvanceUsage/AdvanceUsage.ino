/**
 * Advanced Usage Example for QMC5883 Magnetometer
 * 
 * This example demonstrates all advanced features of the QMC5883 library:
 * - Custom configuration
 * - Multiple reading modes
 * - Orientation correction
 * - Hard iron calibration
 * - Self-test
 * - Runtime configuration changes
 * - Status monitoring
 */

#include <QMC5883.h>

QMC5883 mag;

// Custom configuration for high precision
QMC5883::Config highPrecisionConfig = {
  .mode = QMC5883::MODE_NORMAL,     // Normal mode for balanced performance
  .odr = QMC5883::ODR_10Hz,         // 10Hz for lower noise
  .range = QMC5883::RNG_2G,         // ±2G for highest sensitivity
  .osr = QMC5883::OSR_512,          // 512 oversampling for lowest noise
  .setReset = QMC5883::SET_RESET_ON // Best accuracy
};

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("QMC5883 Advanced Usage Example");
  Serial.println("==============================");
  Serial.println();
  
  // Test multiple initialization methods
  testInitializationMethods();
  
  // Test configuration changes
  testConfigurationChanges();
  
  // Test orientation correction
  testOrientationCorrection();
  
  // Test calibration
  testCalibration();
  
  // Test self-test
  testSelfTest();
  
  // Test status monitoring
  testStatusMonitoring();
  
  Serial.println();
  Serial.println("Advanced Example Setup Complete!");
  Serial.println("Now starting main loop with all features...");
  Serial.println();
}

void loop() {
  static unsigned long lastDisplay = 0;
  static int displayMode = 0; // 0: All data, 1: Heading only, 2: Raw only
  
  // Cycle through display modes every 10 seconds
  if (millis() - lastDisplay > 10000) {
    displayMode = (displayMode + 1) % 3;
    lastDisplay = millis();
    
    Serial.println();
    Serial.print("Switching to display mode: ");
    switch(displayMode) {
      case 0: Serial.println("All Data"); break;
      case 1: Serial.println("Heading Only"); break;
      case 2: Serial.println("Raw Data Only"); break;
    }
    Serial.println();
  }
  
  // Read sensor based on current display mode
  switch(displayMode) {
    case 0: displayAllData(); break;
    case 1: displayHeadingOnly(); break;
    case 2: displayRawData(); break;
  }
  
  delay(500);
}

void testInitializationMethods() {
  Serial.println("1. Testing Initialization Methods");
  Serial.println("---------------------------------");
  
  // Method 1: Default initialization
  Serial.print("Default init... ");
  if (mag.begin()) {
    Serial.println("✓ SUCCESS");
    Serial.print("  Chip ID: 0x"); Serial.println(mag.getChipID(), HEX);
  } else {
    Serial.println("✗ FAILED");
    return;
  }
  
  // Method 2: Custom configuration
  Serial.print("Custom config init... ");
  if (mag.begin(SDA, SCL, highPrecisionConfig)) {
    Serial.println("✓ SUCCESS");
  } else {
    Serial.println("✗ FAILED");
  }
  
  Serial.println();
}

void testConfigurationChanges() {
  Serial.println("2. Testing Runtime Configuration");
  Serial.println("--------------------------------");
  
  // Test output data rate changes
  Serial.println("Testing ODR changes:");
  QMC5883::OutputDataRate odrValues[] = {
    QMC5883::ODR_10Hz, QMC5883::ODR_50Hz, 
    QMC5883::ODR_100Hz, QMC5883::ODR_200Hz
  };
  const char* odrNames[] = {"10Hz", "50Hz", "100Hz", "200Hz"};
  
  for(int i = 0; i < 4; i++) {
    Serial.print("  Setting ODR to "); Serial.print(odrNames[i]); Serial.print("... ");
    if (mag.setOutputDataRate(odrValues[i])) {
      Serial.println("✓ SUCCESS");
    } else {
      Serial.println("✗ FAILED");
    }
    delay(100);
  }
  
  // Test range changes
  Serial.println("Testing range changes:");
  QMC5883::Range rangeValues[] = {
    QMC5883::RNG_2G, QMC5883::RNG_8G, 
    QMC5883::RNG_12G, QMC5883::RNG_30G
  };
  const char* rangeNames[] = {"±2G", "±8G", "±12G", "±30G"};
  
  for(int i = 0; i < 4; i++) {
    Serial.print("  Setting range to "); Serial.print(rangeNames[i]); Serial.print("... ");
    if (mag.setRange(rangeValues[i])) {
      Serial.println("✓ SUCCESS");
    } else {
      Serial.println("✗ FAILED");
    }
    delay(100);
  }
  
  // Set back to optimal for compass
  mag.setOutputDataRate(QMC5883::ODR_10Hz);
  mag.setRange(QMC5883::RNG_8G);
  
  Serial.println();
}

void testOrientationCorrection() {
  Serial.println("3. Testing Orientation Correction");
  Serial.println("---------------------------------");
  
  // Test different orientations
  QMC5883::BoardOrientation orientations[] = {
    QMC5883::ORIENTATION_NORMAL,
    QMC5883::ORIENTATION_ROTATE_90,
    QMC5883::ORIENTATION_ROTATE_180,
    QMC5883::ORIENTATION_ROTATE_270
  };
  const char* orientationNames[] = {
    "Normal", "Rotate 90°", "Rotate 180°", "Rotate 270°"
  };
  
  for(int i = 0; i < 4; i++) {
    Serial.print("  Setting orientation to "); 
    Serial.print(orientationNames[i]); 
    Serial.print("... ");
    
    mag.setOrientation(orientations[i]);
    
    // Quick test read
    QMC5883::Data data;
    if (mag.read(data)) {
      Serial.print("✓ Read: X="); 
      Serial.print(data.x, 2); 
      Serial.print(" G");
    } else {
      Serial.print("✗ Read failed");
    }
    Serial.println();
    delay(100);
  }
  
  // Test custom orientation (swap X and Y, invert Z)
  Serial.print("  Setting custom orientation... ");
  mag.setCustomOrientation(1, 0, 2, false, false, true); // X=Y, Y=X, Z=-Z
  Serial.println("✓ DONE");
  
  // Set back to normal for rest of example
  mag.setOrientation(QMC5883::ORIENTATION_NORMAL);
  
  Serial.println();
}

void testCalibration() {
  Serial.println("4. Testing Calibration System");
  Serial.println("-----------------------------");
  
  Serial.println("  Starting calibration process...");
  mag.startCalibration();
  
  // Simulate calibration by updating multiple times
  for(int i = 0; i < 10; i++) {
    mag.updateCalibration();
    Serial.print("    Calibration update "); 
    Serial.print(i + 1); 
    Serial.println("/10");
    delay(200);
  }
  
  mag.endCalibration();
  
  if (mag.isCalibrated()) {
    Serial.println("  ✓ Calibration complete");
    
    // Show calibration results
    QMC5883::CalibrationData cal = mag.getCalibrationData();
    Serial.println("  Calibration Results:");
    Serial.print("    X: "); Serial.print(cal.x_min); Serial.print(" to "); Serial.println(cal.x_max);
    Serial.print("    Y: "); Serial.print(cal.y_min); Serial.print(" to "); Serial.println(cal.y_max);
    Serial.print("    Z: "); Serial.print(cal.z_min); Serial.print(" to "); Serial.println(cal.z_max);
  } else {
    Serial.println("  ✗ Calibration failed");
  }
  
  Serial.println();
}

void testSelfTest() {
  Serial.println("5. Testing Self-Test Function");
  Serial.println("-----------------------------");
  
  Serial.print("  Running self-test... ");
  if (mag.selfTest()) {
    Serial.println("✓ PASSED - Sensor is functioning correctly");
  } else {
    Serial.println("✗ FAILED - Sensor may have issues");
  }
  
  Serial.println();
}

void testStatusMonitoring() {
  Serial.println("6. Testing Status Monitoring");
  Serial.println("----------------------------");
  
  Serial.print("  Checking status... ");
  QMC5883::Status status = mag.getStatus();
  
  Serial.print("Data Ready: ");
  Serial.print(status.dataReady ? "Yes" : "No");
  Serial.print(", Overflow: ");
  Serial.println(status.overflow ? "Yes" : "No");
  
  Serial.print("  Sensor initialized: ");
  Serial.println(mag.isInitialized() ? "Yes" : "No");
  
  Serial.print("  Sensor calibrated: ");
  Serial.println(mag.isCalibrated() ? "Yes" : "No");
  
  // Set magnetic declination
  mag.setDeclination(-0.1f); // Example value
  Serial.print("  Magnetic declination set to: ");
  Serial.print(mag.getDeclination(), 2);
  Serial.println("°");
  
  Serial.println();
}

void displayAllData() {
  QMC5883::Data sensorData;
  float magnetic_heading, true_heading;
  
  if (mag.getAllData(sensorData, magnetic_heading, true_heading)) {
    // Sensor data
    Serial.print("SENSOR: X="); Serial.print(sensorData.x, 4); 
    Serial.print(" G, Y="); Serial.print(sensorData.y, 4);
    Serial.print(" G, Z="); Serial.print(sensorData.z, 4); Serial.println(" G");
    
    // Heading information
    Serial.print("HEADING: Magnetic="); Serial.print(magnetic_heading, 1);
    Serial.print("°, True="); Serial.print(true_heading, 1);
    Serial.print("°, Direction="); Serial.print(headingToCardinal(true_heading));
    
    // Additional info
    QMC5883::Status status = mag.getStatus();
    Serial.print(", DataReady="); Serial.print(status.dataReady ? "Y" : "N");
    Serial.print(", Overflow="); Serial.println(status.overflow ? "Y" : "N");
  } else {
    Serial.println("ERROR: Failed to read sensor data");
  }
}

void displayHeadingOnly() {
  float magnetic, true_heading;
  
  if (mag.getHeadings(magnetic, true_heading)) {
    Serial.print("COMPASS: ");
    Serial.print(headingToCardinal(true_heading));
    Serial.print(" (");
    Serial.print(true_heading, 1);
    Serial.print("°)");
    
    // Show difference between magnetic and true north
    float declination = mag.getDeclination();
    Serial.print(" [Declination: ");
    Serial.print(declination, 1);
    Serial.print("°]");
    
    Serial.println();
  } else {
    Serial.println("ERROR: Failed to read heading");
  }
}

void displayRawData() {
  QMC5883::RawData raw;
  
  if (mag.readRaw(raw)) {
    // Calculate approximate field strength
    float field_strength = sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z);
    
    Serial.print("RAW: X="); Serial.print(raw.x);
    Serial.print(", Y="); Serial.print(raw.y);
    Serial.print(", Z="); Serial.print(raw.z);
    Serial.print(" | Strength="); Serial.print(field_strength);
    Serial.print(" | Field="); Serial.print(field_strength / QMC5883::getSensitivity(QMC5883::RNG_8G), 2);
    Serial.println(" G");
  } else {
    Serial.println("ERROR: Failed to read raw data");
  }
}

String headingToCardinal(float heading) {
  String directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int index = (int)((heading + 11.25) / 22.5) % 16;
  return directions[index];
}