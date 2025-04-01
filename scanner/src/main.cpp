#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

#define SCAN_TIME 5 // Duration of BLE scan in seconds

BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        Serial.print("Device found: ");
        Serial.println(advertisedDevice.toString().c_str());
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE Scanner");

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); // Active scanning to request scan response data
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
}

void loop() {
    Serial.println("Scanning for BLE devices...");
    BLEScanResults scanResults = pBLEScan->start(SCAN_TIME, false);
    Serial.print("Devices found: ");
    Serial.println(scanResults.getCount());
    pBLEScan->clearResults(); // Free memory from scan results
    delay(5000); // Wait before scanning again
}
