#include "sensors.h"

#define TEMP_DELAY_MS 1000
#define TEMP_DELAY_TICKS (TEMP_DELAY_MS / portTICK_PERIOD_MS)

MAX30105 particleSensor;
const byte RATE_SIZE = 8; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;

TwoWire *wirePort;

bool beat_f = 0;

void sensors_begin(TwoWire &wirePort)
{
    // Configure I2C
    wirePort = wirePort;
    wirePort.begin();
    wirePort.setClock(1e6);

    // Configure HR sensor
    particleSensor.begin(wirePort, I2C_SPEED_FAST);
    particleSensor.setup();                    // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

    pinMode(ACTIVITY_PIN, OUTPUT);
    pinMode(PULSE_PIN, OUTPUT);
}

void sensors_run()
{
    xTaskCreate(
        task_heartmonitor,
        "task_heartmonitor", // Task name
        2048,                // stack size
        NULL,                // Task parameters
        2,                   // Task priority
        NULL                 // Task handler
    );

    xTaskCreate(
        task_heartout,
        "task_heartout", // Task name
        2048,            // stack size
        NULL,            // Task parameters
        1,               // Task priority
        NULL             // Task handler
    );

    xTaskCreate(
        task_sensors,
        "task_sensors",     // Task name
        2048,               // stack size
        NULL,               // Task parameters
        3,                  // Task priority
        NULL                // Task handler
    );
}

void task_sensors(void *parameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize time reference

    while (1)
    {
        digitalWrite(ACTIVITY_PIN, HIGH); // Turn the LED on

        float temperature = readTemperature();
        // Serial.print("Temperature: ");
        // Serial.print(temperature);
        // Serial.println(" °C");
        temperatureCharacteristic->setValue(String(temperature).c_str());

        digitalWrite(LED_PIN, LOW); // Turn the LED off
        vTaskDelayUntil(&xLastWakeTime, TEMP_DELAY_TICKS);
    }
}

void task_heartmonitor(void *parameters)
{
    while (1)
    {
        irValue = particleSensor.getIR();
        if(irValue < 50000)
            continue;
        if (checkForBeat(irValue) == true)
        {
            // We sensed a beat!
            beat_f = 1;

            long delta = millis() - lastBeat;
            lastBeat = millis();

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
                rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                    // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }

        Serial.print("IR:");
        Serial.print(irValue);
        Serial.print(",BPM:");
        Serial.print(beatsPerMinute);
        Serial.print(",AvgBPM:");
        Serial.print(beatAvg);

        // if (irValue < 50000)
        //     Serial.print(",Nofinger:1");
        // else
        //     Serial.print(",Nofinger:0");

        Serial.println();
    }
}

void task_heartout(void *parameters)
{
    while (1)
    {
        if (beat_f == 1)
        {
            beat_f = 0;
            digitalWrite(PULSE_PIN, HIGH); // Turn the LED on
            vTaskDelay(50 / portTICK_PERIOD_MS);
            digitalWrite(PULSE_PIN, LOW);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}