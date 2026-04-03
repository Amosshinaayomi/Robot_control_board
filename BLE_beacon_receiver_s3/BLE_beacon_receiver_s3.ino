/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

// #include <Arduino.h>
// #include <NimBLEDevice.h>
// #include <NimBLEAdvertisedDevice.h>
// #include "NimBLEEddystoneTLM.h"
// #include "NimBLEBeacon.h"

// #define ENDIAN_CHANGE_U16(x) ((((x) & 0xFF00) >> 8) + (((x) & 0xFF) << 8))

// int         scanTime = 5 * 1000; // In milliseconds
// NimBLEScan* pBLEScan;

// class ScanCallbacks : public NimBLEScanCallbacks {
//     void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
//         if (advertisedDevice->haveName()) {
//             Serial.print("Device name: ");
//             Serial.println(advertisedDevice->getName().c_str());
//             Serial.println("");
//         }

//         if (advertisedDevice->haveServiceUUID()) {
//             NimBLEUUID devUUID = advertisedDevice->getServiceUUID();
//             Serial.print("Found ServiceUUID: ");
//             Serial.println(devUUID.toString().c_str());
//             Serial.println("");
//         } else if (advertisedDevice->haveManufacturerData() == true) {
//             std::string strManufacturerData = advertisedDevice->getManufacturerData();
//             if (strManufacturerData.length() == 25 && strManufacturerData[0] == 0x4C && strManufacturerData[1] == 0x00) {
//                 Serial.println("Found an iBeacon!");
//                 NimBLEBeacon oBeacon = NimBLEBeacon();
//                 oBeacon.setData(reinterpret_cast<const uint8_t*>(strManufacturerData.data()), strManufacturerData.length());
//                 Serial.printf("iBeacon Frame\n");
//                 Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",
//                               oBeacon.getManufacturerId(),
//                               ENDIAN_CHANGE_U16(oBeacon.getMajor()),
//                               ENDIAN_CHANGE_U16(oBeacon.getMinor()),
//                               oBeacon.getProximityUUID().toString().c_str(),
//                               oBeacon.getSignalPower());
//             } else {
//                 Serial.println("Found another manufacturers beacon!");
//                 Serial.printf("strManufacturerData: %d ", strManufacturerData.length());
//                 for (int i = 0; i < strManufacturerData.length(); i++) {
//                     Serial.printf("[%X]", strManufacturerData[i]);
//                 }
//                 Serial.printf("\n");
//             }
//             return;
//         }

//         NimBLEUUID eddyUUID = (uint16_t)0xfeaa;

//         if (advertisedDevice->getServiceUUID().equals(eddyUUID)) {
//             std::string serviceData = advertisedDevice->getServiceData(eddyUUID);
//             if (serviceData[0] == 0x20) {
//                 Serial.println("Found an EddystoneTLM beacon!");
//                 NimBLEEddystoneTLM foundEddyTLM = NimBLEEddystoneTLM();
//                 foundEddyTLM.setData(reinterpret_cast<const uint8_t*>(serviceData.data()), serviceData.length());

//                 Serial.printf("Reported battery voltage: %dmV\n", foundEddyTLM.getVolt());
//                 Serial.printf("Reported temperature from TLM class: %.2fC\n", (double)foundEddyTLM.getTemp());
//                 int   temp     = (int)serviceData[5] + (int)(serviceData[4] << 8);
//                 float calcTemp = temp / 256.0f;
//                 Serial.printf("Reported temperature from data: %.2fC\n", calcTemp);
//                 Serial.printf("Reported advertise count: %d\n", foundEddyTLM.getCount());
//                 Serial.printf("Reported time since last reboot: %ds\n", foundEddyTLM.getTime());
//                 Serial.println("\n");
//                 Serial.print(foundEddyTLM.toString().c_str());
//                 Serial.println("\n");
//             }
//         }
//     }
// } scanCallbacks;

// void setup() {
//     Serial.begin(115200);
//     Serial.println("Scanning...");

//     NimBLEDevice::init("Beacon-scanner");
//     pBLEScan = BLEDevice::getScan();
//     pBLEScan->setScanCallbacks(&scanCallbacks);
//     pBLEScan->setActiveScan(true);
//     pBLEScan->setInterval(100);
//     pBLEScan->setWindow(100);
// }

// void loop() {
//     NimBLEScanResults foundDevices = pBLEScan->getResults(scanTime, false);
//     Serial.print("Devices found: ");
//     Serial.println(foundDevices.getCount());
//     Serial.println("Scan done!");
//     pBLEScan->clearResults(); // delete results scan buffer to release memory
//     delay(2000);
// }

#include <NimBLEDevice.h>
#include <NimBLEBeacon.h>

#define ENDIAN_CHANGE_U16(x) ((((x) & 0xFF00) >> 8) + (((x) & 0xFF) << 8))

const char* TARGET_UUID = "2D7A9F0C-E0E8-4CC9-A71B-A21DB2D034A1";
const uint16_t TARGET_MAJOR = 5;
const uint16_t TARGET_MINOR = 88;

float pathLossExponent = 2.5;
float emaRssi = 0;
const float alpha = 0.3;
float distance;


float calculateDistance(float rssi, int txPower, float n) {
    return pow(10, (txPower - rssi) / (10 * n));
}

class MyScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        if (!advertisedDevice->haveManufacturerData()) return;

        std::string manufData = advertisedDevice->getManufacturerData();
        uint8_t* data = (uint8_t*)manufData.data();
        size_t len = manufData.length();

        // Print address and RSSI for all manufacturer data packets
        // Serial.printf("Addr: %s, RSSI: %d, Len: %d, First two: %02X %02X\n",
        //               advertisedDevice->getAddress().toString().c_str(),
        //               advertisedDevice->getRSSI(),
        //               len, data[0], data[1]);

        // If it starts with 4C 00, it's likely an iBeacon (or Apple device)
        if (len >= 25 && data[0] == 0x4C && data[1] == 0x00) {
            NimBLEBeacon beacon;
            beacon.setData(data, len); // will use first 25 bytes

            NimBLEUUID uuid = beacon.getProximityUUID();
            uint16_t major = ENDIAN_CHANGE_U16(beacon.getMajor());
            uint16_t minor = ENDIAN_CHANGE_U16(beacon.getMinor());
            int8_t txPower = beacon.getSignalPower();

            // Serial.printf("  iBeacon: UUID=%s, Major=%d, Minor=%d, TX=%d\n",
            //               uuid.toString().c_str(), major, minor, txPower);

            if (uuid.equals(NimBLEUUID(TARGET_UUID)) && major == TARGET_MAJOR && minor == TARGET_MINOR) {
                int rawRssi = advertisedDevice->getRSSI();
                if (emaRssi == 0) emaRssi = rawRssi;
                else emaRssi = alpha * rawRssi + (1 - alpha) * emaRssi;

                float distance = calculateDistance(emaRssi, txPower, pathLossExponent);
                Serial.printf(">>> TARGET FOUND: distance=%.2f m\n", distance);
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    NimBLEDevice::init("RobotScanner");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new MyScanCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    pScan->setDuplicateFilter(false); // Disable duplicate filtering to see every packet
    pScan->setMaxResults(0);
    pScan->start(0, false, true);

    Serial.println("Scanning for beacon...");
}

void loop() {
    Serial.printf(">>> TARGET FOUND: distance=%.2f m\n", distance);
    delay(1000);
}