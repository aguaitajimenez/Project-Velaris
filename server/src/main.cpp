#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for BLE Service and Characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = nullptr;
BLEService* pService = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLEAdvertising* pAdvertising = nullptr;
bool deviceConnected = false;
int counter = 0;

// Custom Callback to Handle Connections & Disconnections
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        Serial.println("Device Connected");
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
        Serial.println("Device Disconnected. Restarting advertising...");
        deviceConnected = false;
        delay(1000);  // Small delay before restarting advertising
        pServer->startAdvertising();
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work");

    BLEDevice::init("ESP32_BLE_2");

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, 
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pService->start();
    pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    Serial.println("BLE Server Ready. Waiting for connections...");
}

void loop() {
    if (deviceConnected) { // Only notify if a client is connected
        pCharacteristic->setValue((uint8_t*)&counter, sizeof(counter));
        pCharacteristic->notify();  // Send update over BLE

        Serial.print("Sent value: ");
        Serial.println(counter);

        counter++; // Increment counter
    } else {
        Serial.println("No client connected, skipping notification.");
    }

    delay(1000);
}
