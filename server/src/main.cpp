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
char counter_s[10];  // Buffer to store the counter as a string

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");

    // Initialize BLE
    BLEDevice::init("Long name works now");

    // Create Server, Service, and Characteristic
    pServer = BLEDevice::createServer();
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, 
        // BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
        BLECharacteristic::PROPERTY_READ |BLECharacteristic::PROPERTY_NOTIFY
    );

    // Set initial characteristic value
    snprintf(counter_s, sizeof(counter_s), "%d", counter);
    pCharacteristic->setValue(counter_s);
    
    // Start the BLE Service
    pService->start();

    // Start Advertising
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
    // Convert counter to a string and update BLE characteristic
    snprintf(counter_s, sizeof(counter_s), "%d", counter);
    pCharacteristic->setValue(counter_s);
    pCharacteristic->notify();  // Send update over BLE

    // Print the value to Serial
    Serial.print("Sent value: ");
    Serial.println(counter_s);

    // Increment counter
    counter++;

    // Wait before sending the next update
    delay(1000);
}
