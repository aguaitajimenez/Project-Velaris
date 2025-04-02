#ifndef MAIN_H
#define MAIN_H

// -----------------------------------------------------------------------------
// File: YourHeaderFilename.h
// Description: Brief description of what this header file does
// Author: Your Name
// Date: YYYY-MM-DD
// -----------------------------------------------------------------------------

#define CONFIG_DISABLE_HAL_LOCKS 0

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>  
#include <BLEAdvertising.h>
#include "sensors.h"
#include "Wire.h"

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define DEVICE_NAME "ESP32_Advertiser"

// UUIDs for the temperature service
#define TEMPERATURE_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57000"
#define HEARTRATE_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57001"
#define ACCELERATION_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57002"

#define TEMPERATURE_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc000"
#define HEARTRATE_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc001"
#define ACCX_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc002"
#define ACCY_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc003"
#define ACCZ_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc004"
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
extern BLEService* heartRateService;
extern BLEService* acceletationService;

extern BLECharacteristic* temperatureCharacteristic;
extern BLECharacteristic* heartRateCharacteristic;
extern BLECharacteristic* accXCharacteristic;
extern BLECharacteristic* accYCharacteristic;
extern BLECharacteristic* accZCharacteristic;

// -----------------------------------------------------------------------------
// Function Declarations
// -----------------------------------------------------------------------------

void initializeSensor();
float readSensorData();
void printSensorStatus();

// -----------------------------------------------------------------------------
// Optional Class Definition
// -----------------------------------------------------------------------------

#endif // MAIN_H
    