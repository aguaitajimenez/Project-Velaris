#include "sensors.h"

#define DATAECHO 0
#define GPSECHO 1
#define GPSTASK 1

#define OUTPUT_PERIOD 2000

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


bool lora_config_f = 0;
bool enable_gps_lora_f = 0;

// Output variables
typedef enum{
    Bluetooth,
    LoRa
} server_state_t;




// Acc variables;
Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
float acc_x = 0;
float acc_y = 0;
float acc_z = 0;

// GPS variables
Adafruit_GPS gps(&Serial2);

bool gps_fix = false;
uint8_t gps_fixquality = 0;
float gps_lat = 0.0;
char gps_lat_dir = 'N';
float gps_lon = 0.0;
char gps_lon_dir = 'E';
float gps_altitude = 0.0;
uint8_t gps_satellites = 0;

// GPS data spinlock for thread safety
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

// LoRa variables
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void sensors_begin(TwoWire &wire){

    pinMode(GPS_LED_PIN, OUTPUT);
    digitalWrite(GPS_LED_PIN, LOW); // Start with LED off

    pinMode(TFT_I2C_POWER, OUTPUT);
    pinMode(ENABLE_GPS_LORA_PIN, OUTPUT);    
    
    digitalWrite(TFT_I2C_POWER, LOW);
    delay(50);

    // Configure I2C
    wirePort = &wire;
    wirePort->begin();
    wirePort->setClock(400000);
    delay(10);

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

void sensors_run(){
    xTaskCreate(
        task_heartmonitor,
        "task_heartmonitor", // Task name
        4096,                // stack size
        NULL,                // Task parameters
        10,                  // Task priority
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
        7,                  // Task priority
        NULL                // Task handler
    );

    xTaskCreate(
        task_accelerometer,
        "task_accelerometer", // Task name
        4096,                 // stack size
        NULL,                 // Task parameters
        8,                    // Task priority
        NULL                  // Task handler
    );

    xTaskCreate(
        task_output,
        "task_output", // Task name
        4096,              // stack size
        NULL,               // Task parameters
        9,                  // Task priority
        NULL                // Task handler
    );

#if GPSTASK
    xTaskCreate(
        task_gps,
        "task_gps", // Task name
        4096,       // stack size
        NULL,       // Task parameters
        2,          // Task priority
        NULL        // Task handler
    );
#endif

}

void task_temperature(void *parameters){
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize time reference
    while (1)
    {

        temperature_max = particleSensor.readTemperature();
        vTaskDelay(pdMS_TO_TICKS(1));
        vbatt = readBattVoltage();
        vTaskDelay(pdMS_TO_TICKS(1));
        pbatt = readBattPercentage();
        vTaskDelayUntil(&xLastWakeTime, TEMP_DELAY_TICKS);
    }
}

void task_heartmonitor(void *parameters){
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

void task_heartout(void *parameters){
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

void task_accelerometer(void *parameters){
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

void task_output(void *parameters){   
    server_state_t state = LoRa;
    server_state_t next_state = LoRa;

    lora_config_f = loraConfig();

    // Timers for periodic cycle of 
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(OUTPUT_PERIOD);
    xLastWakeTime = xTaskGetTickCount();

    while(1){
        char temp_s[20], bpm_s[20], acc_s[20], batt_s[20];
        // Write string messages in each variable
        snprintf(temp_s, sizeof(temp_s), "%.2f", temperature_max);
        snprintf(bpm_s, sizeof(bpm_s), "%d", beatAvg);
        snprintf(acc_s, sizeof(acc_s), "%.2f;%.2f;%.2f", acc_x, acc_y, acc_z);
        snprintf(batt_s, sizeof(batt_s), "%.2f;%.2f", vbatt, pbatt);

        switch (state) {
        case Bluetooth:
            Serial.println("\n[STATE] BLUETOOTH");
            break;
        case LoRa:
            Serial.println("\n[STATE] LORA");
            break;
        default:
            Serial.println("\n[STATE] UNKNOWN");
            break;
        }


        switch (state){
        case LoRa:
            if(!lora_config_f){
                Serial.println("Config LoRa 1");
                lora_config_f = loraConfig();
                next_state = LoRa;
                break;
            }

            if(bl_connected_f){
                loraDeactivate();
                Serial.println("Change to Bluetooth 1");
                next_state = Bluetooth;
            }

            if(!bl_connected_f && lora_config_f){
                digitalWrite(ACTIVITY_PIN, 1);
                lora_config_f = sendLoraPacket();
                digitalWrite(ACTIVITY_PIN, 0);
                xLastWakeTime = xTaskGetTickCount();
                next_state = LoRa;
                break;
            }
            break;

        case Bluetooth:
            if (!bl_connected_f){
                lora_config_f = loraConfig();
                next_state = LoRa;
                Serial.println("Change to LoRa 1");
                break;
            }

            if(bl_connected_f){
                digitalWrite(ACTIVITY_PIN, HIGH);

                temperatureCharacteristic->setValue(temp_s);
                heartRateCharacteristic->setValue(bpm_s);
                accCharacteristic->setValue(acc_s);
                battCharacteristic->setValue(batt_s);
                temperatureCharacteristic->notify();
                heartRateCharacteristic->notify();
                accCharacteristic->notify();
                battCharacteristic->notify();
                
                next_state = Bluetooth;
            }
            break;
        
        default:
            lora_config_f = loraConfig();
            next_state = LoRa;
            break;
        }        
        

#if DATAECHO
        portENTER_CRITICAL(&gpsMux);
        bool fix = gps_fix;
        uint8_t quality = gps_fixquality;
        float lat = gps_lat;
        char lat_dir = gps_lat_dir;
        float lon = gps_lon;
        char lon_dir = gps_lon_dir;
        float alt = gps_altitude;
        uint8_t sats = gps_satellites;
        portEXIT_CRITICAL(&gpsMux);

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
        Serial.println(bl_connected_f);

        // Then print safely outside the critical section
        Serial.print("GPS fix: ");
        Serial.print((int)fix);
        Serial.print(" quality: ");
        Serial.println((int)quality);

        if (fix)
        {
            Serial.print("Location: ");
            Serial.print(lat, 4);
            Serial.print(lat_dir);
            Serial.print(", ");
            Serial.print(lon, 4);
            Serial.println(lon_dir);

            Serial.print("Altitude: ");
            Serial.println(alt);
            Serial.print("Satellites: ");
            Serial.println((int)sats);
        }
#endif

        state = next_state;
        digitalWrite(ACTIVITY_PIN, LOW);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void task_gps(void *parameters){
    

    int64_t time_us = esp_timer_get_time();
    
    while (1){
        if (Serial2.available()){
            char c = gps.read();
#if GPSECHO
            if (GPSECHO && c)
                Serial.write(c);
#endif

            if (gps.newNMEAreceived()){
                if (!gps.parse(gps.lastNMEA())){
                    // Failed to parse, skip
                    vTaskDelay(pdMS_TO_TICKS(1));
                    continue;
                }
            }

            if (esp_timer_get_time() - time_us > 1e6){
                time_us = esp_timer_get_time();

                portENTER_CRITICAL(&gpsMux);
                gps_fix = gps.fix;
                if (gps_fix){
                    // If location is known, print it
                    gps_fixquality = gps.fixquality;
                    gps_lat = gps.latitude;
                    gps_lat_dir = gps.lat;
                    gps_lon = gps.longitude;
                    gps_lon_dir = gps.lon;
                    gps_altitude = gps.altitude;
                    gps_satellites = gps.satellites;
                }
                portEXIT_CRITICAL(&gpsMux);
                gps.fix ? digitalWrite(GPS_LED_PIN, HIGH) : digitalWrite(GPS_LED_PIN, LOW);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

float readBattVoltage(){
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

float readBattPercentage(){
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
        float ret = percentage_raw * 0.00390635;
        if(ret > 100.0)
            ret = 100.0; 
        return ret; // each bit = 1.25 mV
    }
    return -1.0; // error
}

void errorGPS(){
    digitalWrite(GPS_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(40));
    digitalWrite(GPS_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(40));
    digitalWrite(GPS_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(40));
    digitalWrite(GPS_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(40));
    digitalWrite(GPS_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(GPS_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(GPS_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(GPS_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
}

bool loraConfig() {

    digitalWrite(ENABLE_GPS_LORA_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1500));
    if (!rf95.init()) {
        Serial.println("LoRa init failed!");
        digitalWrite(ENABLE_GPS_LORA_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(ENABLE_GPS_LORA_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        return 0;
    }

    rf95.setFrequency(915.0);
    vTaskDelay(pdMS_TO_TICKS(50));
    rf95.setTxPower(20, false);         // High power mode
    vTaskDelay(pdMS_TO_TICKS(50));
    rf95.setSignalBandwidth(62500);  // Narrow bandwidth
    vTaskDelay(pdMS_TO_TICKS(50));
    rf95.setSpreadingFactor(12);       // Max spreading
    vTaskDelay(pdMS_TO_TICKS(50));
    rf95.setCodingRate4(8);            // Max redundancy
    vTaskDelay(pdMS_TO_TICKS(50));

    gpsConfig();

    enable_gps_lora_f = 1;
    Serial.println("LoRa config OK");
  return 1;
}

bool sendLoraPacket() {
    // Copy values from globals to local safe versions
    portENTER_CRITICAL(&gpsMux);
    bool fix = gps_fix;
    uint8_t quality = gps_fixquality;
    float lat = gps_lat;
    char lat_dir = gps_lat_dir;
    float lon = gps_lon;
    char lon_dir = gps_lon_dir;
    float alt = gps_altitude;
    uint8_t sats = gps_satellites;
    portEXIT_CRITICAL(&gpsMux);

    // Format sensor data from globals
    char temp_s[20], bpm_s[20];
    char accX_s[20], accY_s[20], accZ_s[20];
    char battV_s[20], battP_s[20];

    snprintf(temp_s, sizeof(temp_s), "%.2f", temperature_max);
    snprintf(bpm_s, sizeof(bpm_s), "%d", beatAvg);
    snprintf(accX_s, sizeof(accX_s), "%.2f", acc_x);
    snprintf(accY_s, sizeof(accY_s), "%.2f", acc_y);
    snprintf(accZ_s, sizeof(accZ_s), "%.2f", acc_z);
    snprintf(battV_s, sizeof(battV_s), "%.2f", vbatt);
    snprintf(battP_s, sizeof(battP_s), "%.2f", pbatt);

    // Construct JSON string
    String json = "{";
    json += "\"id\":\"0\",";                 // id
    json += "\"wbv\":" + String(battV_s) + ",";   // wrist_battV → bv
    json += "\"wbp\":" + String(battP_s) + ",";   // wrist_battP → bp
    json += "\"g\":{";                           // gps → g
    json += "\"f\":" + String(fix ? 1 : 0) + ","; // fix → f
    json += "\"la\":" + String(lat, 6) + ",";    // lat → la
    json += "\"ld\":\"" + String(lat_dir) + "\","; // lat_dir → ld
    json += "\"lo\":" + String(lon, 6) + ",";    // lon → lo
    json += "\"lod\":\"" + String(lon_dir) + "\","; // lon_dir → lod
    json += "\"a\":" + String(alt, 6) + ",";     // altitude → a
    json += "\"s\":" + String(sats);            // satellites → s
    json += "}";
    json += "}";


    bool sent = rf95.send((uint8_t *)json.c_str(), json.length());
    Serial.println(json);
    // Wait until package is sent
    while(rf95.mode() == 3)
        vTaskDelay(pdMS_TO_TICKS(10));

#if UART_TRACES
    Serial.println("LoRa packet sent:");
    Serial.println(json);
#endif

    return sent;
}

bool gpsConfig(){
    // Step 1: Start Serial2 at default GPS baud (usually 9600)
    Serial2.begin(9600, SERIAL_8N1, 10, 9);
    gps.begin(9600);

    // Step 2: Set GPS to use 115200 baud
    gps.sendCommand("$PMTK251,115200*1F");
    vTaskDelay(pdMS_TO_TICKS(50)); 

    // Step 3: Restart Serial2 and gps at 115200 baud
    Serial2.flush(); // Clear any old data

    Serial2.begin(115200, SERIAL_8N1, 10, 9);
    gps.begin(115200);
    
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    vTaskDelay(pdMS_TO_TICKS(50));
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
    return false;
}

bool loraDeactivate(){
    enable_gps_lora_f = 0;
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(ENABLE_GPS_LORA_PIN, LOW);
    digitalWrite(GPS_LED_PIN, LOW);
    return 0;
}