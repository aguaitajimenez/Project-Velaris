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
#include "temperature.h"
#include "MAX30105.h"
#include "heartRate.h"

// -----------------------------------------------------------------------------
// Constants and Macros
// -----------------------------------------------------------------------------

#define ACTIVITY_PIN    13 // Built-in LED on most Arduino boards
#define PULSE_PIN       12
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



void sensors_begin(TwoWire &wirePort); 
void sensors_run();
void task_sensors(void * parameters);
void task_heartmonitor(void * parameters);        
void task_heartout(void *parameters);



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
