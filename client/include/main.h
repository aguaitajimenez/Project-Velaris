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

// ==== Constants ====

#define CONNECTED_LED_PIN        13
#define SERVER_NAME              "ESP32_Advertiser"

// MAX17048 Fuel Gauge (I2C)
#define MAX17048_I2CADDR_DEFAULT 0x36
#define VBATT_ADDRESS            0x02  // Voltage register
#define PBATT_ADDRESS            0x04  // Percentage (State of Charge) register

// ==== Function Declarations ====

float readBattVoltage();     // Reads battery voltage from MAX17048
float readBattPercentage();  // Reads battery % (state of charge) from MAX17048

#endif // MAIN_H
