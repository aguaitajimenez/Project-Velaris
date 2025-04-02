#ifndef SENSORS_H
#define SENSORS_H

// -----------------------------------------------------------------------------
// File: SENSORS_H.h
// Description: Brief description of what this header file does
// Author: Alejandro Guaita & Paula Lafarga
// Date: YYYY-MM-DD
// -----------------------------------------------------------------------------

#include <Arduino.h>  // Include core Arduino definitions
#include "main.h"
#include "Wire.h"
#include "temperature.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "BNO08x.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>



// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define ACTIVITY_PIN    13 // Built-in LED on most Arduino boards
#define PULSE_PIN       12
#define TMP117_ADDRESS 0x48 // Default I2C address for TMP117

// BLE Service and Characteristic
extern BLEService* temperatureService;
extern BLEService* heartRateService;
extern BLEService* acceletationService;

extern BLECharacteristic* temperatureCharacteristic;
extern BLECharacteristic* heartRateCharacteristic;
extern BLECharacteristic* accXCharacteristic;
extern BLECharacteristic* accYCharacteristic;
extern BLECharacteristic* accZCharacteristic;

// TFT display pins
// #define TFT_CS        7
// #define TFT_RST       -1
// #define TFT_DC        39

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

// typedef struct {
//     int exampleField;
// } ExampleStruct;

// -----------------------------------------------------------------------------
// Function Declarations
// -----------------------------------------------------------------------------



void sensors_begin(TwoWire &wirePort); 
void sensors_run();
void task_temperature(void * parameters);
void task_heartmonitor(void * parameters);        
void task_heartout(void *parameters);
void task_heartAndO2monitor(void *parameters);
void task_accelerometer(void *parameters);
void task_uart_output(void * parameters);
void task_tft(void *parameters);

// -----------------------------------------------------------------------------
// Optional Class Definition
// -----------------------------------------------------------------------------

// class ExampleClass {
// public:
//     ExampleClass();
//     void begin();
//     float getValue();

// private:
//     int _internalState;
// };

#endif // YOUR_HEADER_FILENAME_H
