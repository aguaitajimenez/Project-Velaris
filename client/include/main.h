#ifndef MAIN_H
#define MAIN_H

// Core Arduino
#include <Arduino.h>

// BLE libraries
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>

// I2C for battery gauge
#include <Wire.h>

// WiFi and UDP
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_timer.h"

#include <SPI.h>              // Required for SPI communication
#include <RH_RF95.h>          // Main RadioHead driver for RFM95

// ==== Constants ====

#define CONNECTED_LED_PIN        13
#define SERVER_NAME              "ESP32_Advertiser"

// MAX17048 Fuel Gauge (I2C)
#define MAX17048_I2CADDR_DEFAULT 0x36
#define VBATT_ADDRESS            0x02  // Voltage register
#define PBATT_ADDRESS            0x04  // Percentage (State of Charge) register

// LoRa pins
#define RFM95_CS 8     // Chip select
#define RFM95_INT 14     // DIO0 (interrupt)
#define RF95_FREQ  915.0 // or 915.0 depending on region

// ==== Function Declarations ====

void bluetooth_task(void *parameters);
void lora_task(void *parameters);

float readBattVoltage();     // Reads battery voltage from MAX17048
float readBattPercentage();  // Reads battery % (state of charge) from MAX17048

bool blRead();
bool sendJSON();

bool loraConfig();
bool loraReceiveAndForward();

#endif // MAIN_H
