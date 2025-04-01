#include "temperature.h"

float readTemperature() {    
    Wire.requestFrom(TMP117_ADDRESS, 2);
    if (Wire.available() < 2) {
        Serial.println("Error: Could not read temperature");
        return NAN;
    }
    
    int16_t rawTemperature = (Wire.read() << 8) | Wire.read();
    return rawTemperature * 0.0078125; // Convert to Celsius
}