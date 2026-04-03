// #include <NimBLEDevice.h>
// #include <NimBLEBeacon.h>

// #include <NimBLEDescriptor.h>
// #define DEVICE_NAME         "ESP32"
// #define SERVICE_UUID        "7A0247E7-8E88-409B-A959-AB5092DDB03E"
// #define BEACON_UUID         "2D7A9F0C-E0E8-4CC9-A71B-A21DB2D034A1"
// #define CHARACTERISTIC_UUID "82258BAA-DF72-47E8-99BC-B73D7ECD08A5"

// NimBLEServer* pServer = nullptr;
// NimBLECharacteristic* pCharacteristic = nullptr;
// bool deviceConnected = false;
// uint8_t value = 0;

// // Server callbacks
// class MyServerCallbacks : public NimBLEServerCallbacks {
//     void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
//         deviceConnected = true;
//         Serial.println("deviceConnected = true");
//     }

//     void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
//         deviceConnected = false;
//         Serial.println("deviceConnected = false");
//         // Restart advertising so the device becomes visible again
//         pServer->getAdvertising()->start();
//         Serial.println("Advertising restarted");
//     }
// };

// // Characteristic callbacks
// class MyCharCallbacks : public NimBLECharacteristicCallbacks {
//     void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
//         std::string rxValue = pCharacteristic->getValue();
//         if (rxValue.length() > 0) {
//             Serial.print("Received value: ");
//             for (char c : rxValue) Serial.print(c);
//             Serial.println();
//         }
//     }
// };

// void setup() {
//     Serial.begin(115200);
//     NimBLEDevice::init(DEVICE_NAME);

//     // Create server
//     pServer = NimBLEDevice::createServer();
//     pServer->setCallbacks(new MyServerCallbacks());

//     // Create service
//     NimBLEService* pService = pServer->createService(SERVICE_UUID);

//     // Create characteristic
//     pCharacteristic = pService->createCharacteristic(
//         CHARACTERISTIC_UUID,
//         NIMBLE_PROPERTY::READ |
//         NIMBLE_PROPERTY::WRITE |
//         NIMBLE_PROPERTY::NOTIFY
//     );
//     pCharacteristic->setCallbacks(new MyCharCallbacks());


//     // Start service
//     pService->start();

//     // --- Set up iBeacon advertising data ---
//     NimBLEBeacon beacon;
//     beacon.setManufacturerId(0x4c00);
//     beacon.setMajor(5);
//     beacon.setMinor(88);
//     beacon.setSignalPower(0xc5);
//     beacon.setProximityUUID(NimBLEUUID(BEACON_UUID));

//     NimBLEAdvertisementData advData;
//     advData.setFlags(0x1A);
//     advData.setManufacturerData(beacon.getData());

//     // Get the advertising object from the server and apply the beacon data
//     NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
//     pAdvertising->setAdvertisementData(advData);
//     pAdvertising->addServiceUUID(pService->getUUID()); // optional: advertise the service too

//     // Start advertising
//     pAdvertising->start();
//     Serial.println("iBeacon + service advertising started!");
// }

// void loop() {
//     if (deviceConnected) {
//         Serial.printf("*** NOTIFY: %d ***\n", value);
//         pCharacteristic->setValue(&value, 1);
//         pCharacteristic->notify();
//         value++;
//     }
//     delay(2000);
// }

#include <NimBLEDevice.h>
#include <NimBLEBeacon.h>

// Replace these with your own values
#define DEVICE_NAME         "ESP32_Robot"
#define SERVICE_UUID        "7A0247E7-8E88-409B-A959-AB5092DDB03E"
#define BEACON_UUID         "2D7A9F0C-E0E8-4CC9-A71B-A21DB2D034A1" // Must match receiver
#define CHARACTERISTIC_UUID "82258BAA-DF72-47E8-99BC-B73D7ECD08A5"

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// Server callbacks – restart advertising on disconnect
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        Serial.println("Client connected");
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        Serial.println("Client disconnected – restarting advertising");
        // Small delay to let stack clean up
        delay(100);
        pServer->getAdvertising()->start();
        Serial.println("Advertising restarted");
    }
};

// Characteristic callbacks – handle incoming commands
class MyCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.print("Received command: ");
            for (char c : rxValue) Serial.print(c);
            Serial.println();
            // Parse your command byte(s) here and set motor speeds etc.
        }
    }
};

void setup() {
    Serial.begin(115200);
    NimBLEDevice::init(DEVICE_NAME);

    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create a service (even minimal – needed for connections)
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Create a characteristic (read/write/notify)
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::NOTIFY
    );
    pCharacteristic->setCallbacks(new MyCharCallbacks());

    // Start service
    pService->start();

    // --- iBeacon advertising data ---
    NimBLEBeacon beacon;
    beacon.setManufacturerId(0x4c00);          // Apple company ID
    beacon.setMajor(5);                        // Your major
    beacon.setMinor(88);                        // Your minor
    beacon.setSignalPower(0xc5);                // Calibrated TX power (replace after calibration)
    beacon.setProximityUUID(NimBLEUUID(BEACON_UUID));

    std::vector<uint8_t> beaconData = beacon.getData();
    Serial.print("Beacon data length: ");
    Serial.println(beaconData.length());
    Serial.print("Beacon data bytes: ");
    for (int i = 0; i < beaconData.length(); i++) {
        Serial.printf("%02X ", (uint8_t)beaconData[i]);
    }
    Serial.println();
    NimBLEAdvertisementData advData;
    advData.setFlags(0x1A);                     // BR/EDR not supported, LE general discoverable
    advData.setManufacturerData(beacon.getData());

    // Get advertising object from server and apply data
    NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->addServiceUUID(pService->getUUID()); // optional – helps clients identify

    // Start advertising
    pAdvertising->start();
    Serial.println("iBeacon + GATT server started");
}

void loop() {
    if (deviceConnected) {
        // Example: send a notification every 2 seconds
        static uint8_t counter = 0;
        pCharacteristic->setValue(&counter, 1);
        pCharacteristic->notify();
        counter++;
    }
    delay(2000);
}