#include "main.h"

// BLE Server and Advertising objects
BLEServer* pServer;
BLEAdvertising* pAdvertising;

// BLE Service and Characteristic
BLEService* temperatureService;
BLEService* heartRateService;
BLEService* acceletationService;

BLECharacteristic* temperatureCharacteristic;
BLECharacteristic* heartRateCharacteristic;
BLECharacteristic* accXCharacteristic;
BLECharacteristic* accYCharacteristic;
BLECharacteristic* accZCharacteristic;

int counter = 0;

void setup() {
    Serial.begin(115200);
    // Wire.begin();
    sensors_begin(Wire);
    delay(1000); // give peripherals some time
    Serial.println("Starting BLE Server with Temperature Service");

    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pAdvertising = BLEDevice::getAdvertising();

    // Create and start the temperature service
    temperatureService = pServer->createService(TEMPERATURE_SERVICE_UUID);
    heartRateService = pServer->createService(HEARTRATE_SERVICE_UUID);
    acceletationService = pServer->createService(ACCELERATION_SERVICE_UUID);


    // Create and set up temperature characteristic with descriptor
    temperatureCharacteristic = temperatureService->createCharacteristic(
        TEMPERATURE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    heartRateCharacteristic = heartRateService->createCharacteristic(
        HEARTRATE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );



    accXCharacteristic = acceletationService->createCharacteristic(
        ACCX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    accYCharacteristic = acceletationService->createCharacteristic(
        ACCY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    accZCharacteristic = acceletationService->createCharacteristic(
        ACCX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );


    temperatureCharacteristic->setValue("Hello World - Temperature");

    // Start the service
    temperatureService->start();
    heartRateService->start();
    acceletationService->start();

    // Add service to advertising
    pAdvertising->addServiceUUID(TEMPERATURE_SERVICE_UUID);
    pAdvertising->addServiceUUID(HEARTRATE_SERVICE_UUID);
    pAdvertising->addServiceUUID(ACCELERATION_SERVICE_UUID);

    // Set up advertisement data
    BLEAdvertisementData advertisementData;
    advertisementData.setName(DEVICE_NAME);
    advertisementData.setManufacturerData("ESP32 BLE");
    pAdvertising->setAdvertisementData(advertisementData);

    // Configure advertisement parameters
    pAdvertising->setMinInterval(1600); // 1 second
    pAdvertising->setMaxInterval(3200); // 2 seconds
    pAdvertising->setScanResponse(false);
    pAdvertising->start();

    Serial.println("BLE Server with Temperature Service Started");
    // xTaskCreate(
    //     task_sensors,
    //     "task_sensors",     // Task name
    //     2048,               // stack size
    //     NULL,               // Task parameters
    //     1,                  // Task priority
    //     NULL                // Task handler
    // );
    sensors_run();

}

void loop() {
    // Serial.print("Program execution counter: ");
    // Serial.println(counter);
    // counter++;
    // delay(1000); // Delay to avoid unnecessary advertisement flooding
}
