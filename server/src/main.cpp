#include "main.h"



// BLE Server and Advertising objects
BLEServer* pServer;
BLEAdvertising* pAdvertising;

// BLE Service and Characteristic
BLEService* applicationService;


BLECharacteristic* temperatureCharacteristic;
BLECharacteristic* heartRateCharacteristic;
BLECharacteristic* accXCharacteristic;
BLECharacteristic* accYCharacteristic;
BLECharacteristic* accZCharacteristic;
BLECharacteristic* battVCharacteristic;
BLECharacteristic* battPCharacteristic;

bool deviceConnected = false;
int counter = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer){
        deviceConnected = true;
        delay(100);
        pAdvertising->start();
    }

    void onDisconnect(BLEServer* pServer){
        deviceConnected = false;
        // Optional: restart advertising so others can connect
        pAdvertising->start();
    }
};

// HardwareSerial bnoSerial(1);
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 18, 17);
    while (!Serial2)
    delay(10);
    
    // Wire.begin();
    sensors_begin(Wire);
    delay(1000); // give peripherals some time
    Serial.println("Starting BLE Server with Temperature Service");

    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); 
    pAdvertising = BLEDevice::getAdvertising();

    // Create and start the temperature service
    applicationService = pServer->createService(APPLICATION_SERVICE_UUID);


    // Create and set up temperature characteristic with descriptor
    temperatureCharacteristic = applicationService->createCharacteristic(
        TEMPERATURE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ  | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    heartRateCharacteristic = applicationService->createCharacteristic(
        HEARTRATE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    accXCharacteristic = applicationService->createCharacteristic(
        ACCX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    accYCharacteristic = applicationService->createCharacteristic(
        ACCY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    accZCharacteristic = applicationService->createCharacteristic(
        ACCZ_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    battVCharacteristic = applicationService->createCharacteristic(
        BATTV_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    battPCharacteristic = applicationService->createCharacteristic(
        BATTP_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );



    temperatureCharacteristic->setValue("INITIALSTRING");
    heartRateCharacteristic->setValue("INITIALSTRING");
    accXCharacteristic->setValue("INITIALSTRING");
    accYCharacteristic->setValue("INITIALSTRING");
    accZCharacteristic->setValue("INITIALSTRING");
    battVCharacteristic->setValue("INITIALSTRING");
    battPCharacteristic->setValue("INITIALSTRING");

    // Start the service
    applicationService->start();


    // Add service to advertising
    pAdvertising->addServiceUUID(APPLICATION_SERVICE_UUID);

    // Set up advertisement data
    BLEAdvertisementData advertisementData;
    advertisementData.setName(DEVICE_NAME);
    advertisementData.setManufacturerData("ESP32 BLE");
    pAdvertising->setAdvertisementData(advertisementData);
    pAdvertising->addServiceUUID(APPLICATION_SERVICE_UUID);

    // Configure advertisement parameters
    pAdvertising->setMinInterval(1600); // 1 second
    pAdvertising->setMaxInterval(3200); // 2 seconds
    pAdvertising->setScanResponse(false);
    pAdvertising->start();

    Serial.println("BLE Server with Temperature Service Started");
    sensors_run();

}

void loop() {
    // Serial.print("Program execution counter: ");
    // Serial.println(counter);
    // counter++;
    // delay(1000); // Delay to avoid unnecessary advertisement flooding
}
