#include "sensors.h"

#define TEMP_DELAY_MS 500
#define TEMP_DELAY_TICKS (TEMP_DELAY_MS / portTICK_PERIOD_MS)

float readTemperature() {    
    Wire.requestFrom(TMP117_ADDRESS, 2);
    if (Wire.available() < 2) {
        Serial.println("Error: Could not read temperature");
        return NAN;
    }
    
    int16_t rawTemperature = (Wire.read() << 8) | Wire.read();
    return rawTemperature * 0.0078125; // Convert to Celsius
}

void task_sensors(void * parameters){
    pinMode(LED_PIN, OUTPUT);
    
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize time reference

    while(1){
        digitalWrite(LED_PIN, HIGH); // Turn the LED on

        float temperature = readTemperature();
        Serial.print("Temperature: ");Serial.print(temperature);Serial.println(" °C");    
        temperatureCharacteristic->setValue(String(temperature).c_str());

        digitalWrite(LED_PIN, LOW);  // Turn the LED off
        vTaskDelayUntil(&xLastWakeTime, TEMP_DELAY_TICKS);
    }
}
