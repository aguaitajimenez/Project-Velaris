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
#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_GPS.h"

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define DEVICE_NAME "ESP32_Advertiser"

// UUIDs for the temperature service
#define APPLICATION_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57000"
#define HEARTRATE_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57001"
#define ACCELERATION_SERVICE_UUID "eab293ea-ab75-4fa3-a21f-66a937c57002"

#define TEMPERATURE_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc000"
#define HEARTRATE_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc001"
#define ACC_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc002"
#define BATT_CHARACTERISTIC_UUID "6bd16e28-6f99-40b9-abe5-dfc1ad6dc005"
// #define DESCRIPTOR_UUID "2901"     // Descriptor UUID




// RFM95 defines
#define RFM95_CS     8
#define RFM95_RST    9
#define RFM95_INT   14
#define RF95_FREQ 915.0

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
extern BLEService* applicationService;
// extern BLEService* heartRateService;
// extern BLEService* acceletationService;

extern BLECharacteristic* temperatureCharacteristic;
extern BLECharacteristic* heartRateCharacteristic;
extern BLECharacteristic* accCharacteristic;

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
    