/**
 * Calibration Example for QMC5883 Magnetometer
 * 
 * This example demonstrates how to calibrate the magnetometer
 * to compensate for hard iron distortions.
 */

#include <QMC5883.h>

QMC5883 mag;

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    Serial.println("QMC5883 Calibration Example");
    Serial.println("===========================");
    
    if (mag.begin()) {
        Serial.println("✓ QMC5883 initialized successfully");
    } else {
        Serial.println("✗ Failed to initialize QMC5883");
        while(1);
    }
    
    // Start calibration process
    mag.startCalibration();
    Serial.println("Calibration Started!");
    Serial.println("Rotate the device slowly in all directions for 30 seconds");
    Serial.println("Make sure to cover all orientations (pitch, roll, yaw)");
    Serial.println();
}

void loop() {
    static unsigned long calibrationStart = millis();
    static bool calibrationComplete = false;
    
    // Calibration phase (first 30 seconds)
    if (!calibrationComplete && millis() - calibrationStart < 30000) {
        mag.updateCalibration();
        
        // Print progress every 5 seconds
        if ((millis() - calibrationStart) % 5000 == 0) {
            int secondsRemaining = (30000 - (millis() - calibrationStart)) / 1000;
            Serial.print("Calibrating... ");
            Serial.print(secondsRemaining);
            Serial.println(" seconds remaining");
        }
        
        delay(100);
        return;
    }
    
    // End calibration after 30 seconds
    if (!calibrationComplete) {
        mag.endCalibration();
        calibrationComplete = true;
        
        Serial.println();
        Serial.println("✓ Calibration Complete!");
        
        // Print calibration results
        QMC5883::CalibrationData calData = mag.getCalibrationData();
        Serial.println("Calibration Results:");
        Serial.print("X: min="); Serial.print(calData.x_min); 
        Serial.print(", max="); Serial.println(calData.x_max);
        Serial.print("Y: min="); Serial.print(calData.y_min);
        Serial.print(", max="); Serial.println(calData.y_max);
        Serial.print("Z: min="); Serial.print(calData.z_min);
        Serial.print(", max="); Serial.println(calData.z_max);
        Serial.println();
        
        Serial.println("Now reading calibrated data:");
        Serial.println();
    }
    
    // Read and display calibrated data
    QMC5883::Data data;
    float true_heading;
    
    if (mag.getTrueHeading(true_heading)) {
        Serial.print("True Heading: ");
        Serial.print(true_heading, 1);
        Serial.print("° (");
        Serial.print(headingToCardinal(true_heading));
        Serial.println(")");
    }
    
    delay(500);
}

String headingToCardinal(float heading) {
    String directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", 
                          "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    int index = (int)((heading + 11.25) / 22.5) % 16;
    return directions[index];
}