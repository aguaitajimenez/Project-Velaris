#ifndef SENSORS_H
#define SENSORS_H

// -----------------------------------------------------------------------------
// File: SENSORS_H.h
// Description: Brief description of what this header file does
// Author: Alejandro Guaita & Paula Lafarga
// Date: YYYY-MM-DD
// -----------------------------------------------------------------------------

#include "Arduino.h"
#include <Wire.h>

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define LED_PIN 13 // Built-in LED on most Arduino boards
#define TMP117_ADDRESS 0x48 // Default I2C address for TMP117

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

// typedef struct {
//     int exampleField;
// } ExampleStruct;

// -----------------------------------------------------------------------------
// Function Declarations
// -----------------------------------------------------------------------------

float readTemperature();

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
