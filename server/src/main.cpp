#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for BLE Service and Characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Global BLE objects
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;
BLEAdvertising *pAdvertising;

// Global counter
int counter = 0;

// Custom Callback to Handle Disconnections
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        Serial.println("Device Connected");
    }

    void onDisconnect(BLEServer* pServer) {
        Serial.println("Device Disconnected, Restarting advertising...");
        delay(500);
        BLEDevice::startAdvertising();
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work");

    // Initialize BLE
    BLEDevice::init("ESP32_BLE_2");

    // Create Server, Service, and Characteristic
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());  
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, 
        BLECharacteristic::PROPERTY_NOTIFY
    );

    // Set initial characteristic value
    pCharacteristic->setValue((uint8_t*)&counter, sizeof(counter));

    // Start the BLE Service
    pService->start();

    // Start Advertising
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("Characteristic defined. Now you can read it in your phone.");
}

void loop() {
    // Convert counter to raw byte data
    pCharacteristic->setValue((uint8_t*)&counter, sizeof(counter));
    pCharacteristic->notify();  // Send update over BLE

    // Print the value to Serial
    Serial.print("Sent value: ");
    Serial.println(counter);

    // Increment counter
    counter++;

    // Give some time for BLE stack processing
    delay(1000);
}
