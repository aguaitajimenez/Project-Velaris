#include "sensors.h"

#define TEMP_DELAY_MS 500
#define TEMP_DELAY_TICKS (TEMP_DELAY_MS / portTICK_PERIOD_MS)

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
