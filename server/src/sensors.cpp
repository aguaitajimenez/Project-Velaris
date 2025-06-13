#include "sensors.h"

#define TEMP_DELAY_MS 500
#define TEMP_DELAY_TICKS (TEMP_DELAY_MS / portTICK_PERIOD_MS)

TwoWire *wirePort;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

float temperature_tmp117 = 0;
float temperature_max = 0;

MAX30105 particleSensor;
const byte RATE_SIZE = 8; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;
bool beat_f = 0;

#define MAX17048_I2CADDR_DEFAULT 0x36
#define VBATT_ADDRESS 0x02
#define PBATT_ADDRESS 0x04
float vbatt = 0;
float pbatt = 0;
// uint32_t irBuffer[100];     //infrared LED sensor data
// uint32_t redBuffer[100];    //red LED sensor data

// int32_t bufferLength; //data length
// int32_t spo2; //SPO2 value
// int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
// int32_t heartRate; //heart rate value
// int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
// sh2_SensorValue_t bnoValues;
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;

void sensors_begin(TwoWire &wire)
{

    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, LOW);
    delay(50);

    // Configure I2C
    wirePort = &wire;
    wirePort->begin();
    wirePort->setClock(400000);
    delay(10);
    // tft.init(135, 240); // Initialize ST7789 240x135
    // tft.setRotation(3);
    // tft.fillScreen(ST77XX_BLACK);
    // tft.setTextColor(ST77XX_WHITE);
    // tft.setTextSize(1);

    // Configure HR sensor
    particleSensor.begin(*wirePort, I2C_SPEED_FAST);
    particleSensor.setup(0x1F, 4, 3, 200, 411, 4096); // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A);        // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);         // Turn off Green LED
    delay(10);

    if (!rvc.begin(&Serial1))
    { // connect to the sensor over hardware serial
        Serial.println("Could not find BNO08x!");

        while (1)
        {
            Serial1.println("bug0");
            delay(10);
        }
    }


    delay(10);
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
        8,                   // Task priority
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
        task_temperature,
        "task_temperature", // Task name
        2048,               // stack size
        NULL,               // Task parameters
        8,                  // Task priority
        NULL                // Task handler
    );

    xTaskCreate(
        task_accelerometer,
        "task_accelerometer", // Task name
        2048,                 // stack size
        NULL,                 // Task parameters
        7,                    // Task priority
        NULL                  // Task handler
    );

    xTaskCreate(
        task_data_output,
        "task_data_output", // Task name
        20480,              // stack size
        NULL,               // Task parameters
        10,                 // Task priority
        NULL                // Task handler
    );

    // xTaskCreate(
    //     task_tft,
    //     "task_tft",
    //     2048,
    //     NULL,
    //     10,
    //     NULL
    // );
}

void task_temperature(void *parameters)
{

    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize time reference

    while (1)
    {
        // digitalWrite(ACTIVITY_PIN, HIGH); // Turn the LED on

        // temperature_tmp117 = readTemperature();
        // vTaskDelay(1 / portTICK_PERIOD_MS);
        temperature_max = particleSensor.readTemperature();
        delay(1);
        vbatt = readBattVoltage();
        delay(1);
        pbatt = readBattPercentage();

        // Serial.print("Temperature: ");
        // Serial.print(temperature);
        // Serial.println(" °C");
        // temperatureCharacteristic->setValue(String(temperature_tmp117).c_str());

        // digitalWrite(LED_PIN, LOW); // Turn the LED off
        vTaskDelayUntil(&xLastWakeTime, TEMP_DELAY_TICKS);
    }
}

void task_heartmonitor(void *parameters)
{
    while (1)
    {
        irValue = particleSensor.getIR();

        if (irValue < 50000)
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

        // Serial.print("IR:");
        // Serial.print(irValue);
        // Serial.print(",BPM:");
        // Serial.print(beatsPerMinute);
        // Serial.print(",AvgBPM:");
        // Serial.print(beatAvg);

        // if (irValue < 50000)
        //     Serial.print(",Nofinger:1");
        // else
        //     Serial.print(",Nofinger:0");

        // Serial.println();
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

void task_accelerometer(void *parameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 ms

    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    BNO08x_RVC_Data heading;

    while (1)
    {
        if (rvc.read(&heading))
        {
            acc_x = heading.x_accel;
            acc_y = heading.y_accel;
            acc_z = heading.z_accel;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}

void task_data_output(void *parameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Convert 200 ms to ticks

    // Initialize the xLastWakeTime variable with the current time
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        
        // Output sensor data

        char temp_s[20]; // Aumentado
        char bpm_s[20];
        char accX_s[20];
        char accY_s[20];
        char accZ_s[20];
        char battV_s[20];
        char battP_s[20];

        snprintf(temp_s, sizeof(temp_s), "%.2f", temperature_max);
        snprintf(bpm_s, sizeof(bpm_s), "%d", beatAvg);
        snprintf(accX_s, sizeof(accX_s), "%.2f", acc_x);
        snprintf(accY_s, sizeof(accY_s), "%.2f", acc_y);
        snprintf(accZ_s, sizeof(accZ_s), "%.2f", acc_z);
        snprintf(battV_s, sizeof(battV_s), "%.2f", vbatt);
        snprintf(battP_s, sizeof(battP_s), "%.2f", pbatt);

        if (deviceConnected)
        {   
            digitalWrite(ACTIVITY_PIN, HIGH);
            temperatureCharacteristic->setValue(temp_s);
            heartRateCharacteristic->setValue(bpm_s);
            accXCharacteristic->setValue(accX_s);
            accYCharacteristic->setValue(accY_s);
            accZCharacteristic->setValue(accZ_s);
            battVCharacteristic->setValue(battV_s);
            battPCharacteristic->setValue(battP_s);
            
            // temperatureCharacteristic->notify();
            
            Serial.print("tmpMAX:");
            Serial.print(temperature_max);
            Serial.print(",AvgBPM:");
            Serial.print(beatAvg);
            Serial.print(",accX:");
            Serial.print(acc_x);
            Serial.print(",accY:");
            Serial.print(acc_y);
            Serial.print(",accZ:");
            Serial.print(acc_z);
            Serial.print(",vbatt:");
            Serial.print(battV_s);
            Serial.print(",pbatt:");
            Serial.print(battP_s);
            Serial.print(",devConnected:");
            Serial.println(deviceConnected);
        }

        digitalWrite(ACTIVITY_PIN, LOW);
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void task_tft(void *parameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Update every 1000 ms

    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        tft.fillScreen(ST77XX_BLACK); // Clear screen

        tft.setCursor(0, 0);
        tft.print("tmp117: ");
        tft.println(temperature_tmp117);

        tft.print("tmpMAX: ");
        tft.println(temperature_max);

        tft.print("AvgBPM: ");
        tft.println(beatAvg);

        tft.print("accX: ");
        tft.println(acc_x);

        tft.print("accY: ");
        tft.println(acc_y);

        tft.print("accZ: ");
        tft.println(acc_z);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

float readBattVoltage()
{
    Wire.beginTransmission(MAX17048_I2CADDR_DEFAULT);
    Wire.write(VBATT_ADDRESS); // Voltage register
    Wire.endTransmission(false);
    Wire.requestFrom(MAX17048_I2CADDR_DEFAULT, 2);

    if (Wire.available() == 2)
    {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint16_t voltage_raw = ((uint16_t)msb << 8) | lsb;
        //   voltage_raw >>= 4; // lower 4 bits are not used
        return voltage_raw * 0.000078125; // each bit = 1.25 mV
    }
    return -1.0; // error
}

float readBattPercentage()
{
    Wire.beginTransmission(MAX17048_I2CADDR_DEFAULT);
    Wire.write(PBATT_ADDRESS); // Voltage register
    Wire.endTransmission(false);
    Wire.requestFrom(MAX17048_I2CADDR_DEFAULT, 2);

    if (Wire.available() == 2)
    {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint16_t percentage_raw = ((uint16_t)msb << 8) | lsb;
        //   voltage_raw >>= 4; // lower 4 bits are not used
        return percentage_raw * 0.00390635; // each bit = 1.25 mV
    }
    return -1.0; // error
}