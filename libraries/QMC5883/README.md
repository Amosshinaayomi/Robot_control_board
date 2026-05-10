# QMC5883 Magnetometer Library

A comprehensive Arduino library for the QMC5883 3-axis magnetometer with advanced features including calibration, orientation correction, and magnetic declination compensation.

## Features

- **Easy Initialization**: Simple setup with default or custom configuration
- **Multiple Reading Modes**: Raw data, calibrated data, and heading calculations
- **Hard Iron Calibration**: Automatic calibration for magnetic distortions
- **Orientation Correction**: Fix sensor mounting offsets and axis remapping
- **Magnetic Declination**: True north heading calculation
- **Self-Test**: Built-in sensor self-test functionality
- **Multiple Output Rates**: 10Hz, 50Hz, 100Hz, 200Hz
- **Configurable Ranges**: ±2G, ±8G, ±12G, ±30G

## Installation

1. Download this library as a ZIP file
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Or manually copy to your Arduino libraries folder

## Quick Start

```cpp
#include <QMC5883.h>

QMC5883 mag;

void setup() {
    Serial.begin(115200);
    mag.begin(); // Initialize with default settings
    mag.setDeclination(-0.1f); // Set your magnetic declination
}

void loop() {
    float heading;
    if (mag.getTrueHeading(heading)) {
        Serial.print("Heading: ");
        Serial.println(heading);
    }
    delay(1000);
}