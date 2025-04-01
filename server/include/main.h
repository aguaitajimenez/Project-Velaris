#ifndef MAIN_H
#define MAIN_H

// -----------------------------------------------------------------------------
// File: YourHeaderFilename.h
// Description: Brief description of what this header file does
// Author: Your Name
// Date: YYYY-MM-DD
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include "sensors.h"

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define DEVICE_NAME "ESP32_Advertiser"

// UUIDs for the temperature service
#define TEMPERATURE_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57020"
#define CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc0d2" // Updated to standard Temperature Measurement UUID
// #define DESCRIPTOR_UUID "2901"     // Descriptor UUID

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

// typedef struct {
//     int exampleField;
// } ExampleStruct;


// BLE Server and Advertising objects
extern BLEServer* pServer;
extern BLEAdvertising* pAdvertising;

// BLE Service and Characteristic
extern BLEService* temperatureService;
extern BLECharacteristic* temperatureCharacteristic;

// -----------------------------------------------------------------------------
// Function Declarations
// -----------------------------------------------------------------------------

void initializeSensor();
float readSensorData();
void printSensorStatus();

// -----------------------------------------------------------------------------
// Optional Class Definition
// -----------------------------------------------------------------------------

class ExampleClass {
public:
    ExampleClass();
    void begin();
    float getValue();

private:
    int _internalState;
};

#endif // YOUR_HEADER_FILENAME_H
