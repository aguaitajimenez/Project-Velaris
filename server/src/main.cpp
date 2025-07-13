#include "main.h"

// BLE Server and Advertising objects
BLEServer* pServer;
BLEAdvertising* pAdvertising;

// BLE Service and Characteristic
BLEService* applicationService;

BLECharacteristic* temperatureCharacteristic;
BLECharacteristic* heartRateCharacteristic;
BLECharacteristic* accCharacteristic;
BLECharacteristic* battCharacteristic;

bool bl_connected_f = false;
int counter = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer){
        std::map<uint16_t, conn_status_t> peers = pServer->getPeerDevices(false);
        Serial.print("Number of Peer devices: ");
        Serial.println(peers.size());
        bl_connected_f = true;
        delay(10);
        pAdvertising->start();
    }

    void onDisconnect(BLEServer* pServer){
        // Get the current peer devices map
        std::map<uint16_t, conn_status_t> peers = pServer->getPeerDevices(false);
        Serial.print("Number of Peer devices: ");
        Serial.println(peers.size());
        // Only set bl_connected_f to false if NO devices are connected
        if(peers.size() <= 1){
            bl_connected_f = false;
        }
        
        // Optional: restart advertising so others can connect
        pAdvertising->start();
    }
};

// HardwareSerial bnoSerial(1);
void setup() {
    Serial.begin(5000000);
    Serial1.begin(115200, SERIAL_8N1, 18, 17);
    
    // while (!Serial)
    //     delay(10);
    while (!Serial1)
        delay(10);
    
    // Wire.begin();
    sensors_begin(Wire);
    delay(50); // give peripherals some time
    Serial.println("Starting BLE Server");


    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12); // for advertisements
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12); // per connection handle (if needed)

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); 
    pAdvertising = BLEDevice::getAdvertising();

    // Create and start the temperature service
    applicationService = pServer->createService(APPLICATION_SERVICE_UUID);


    // Create and set up temperature characteristic with descriptor
    temperatureCharacteristic = applicationService->createCharacteristic(
        TEMPERATURE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    heartRateCharacteristic = applicationService->createCharacteristic(
        HEARTRATE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    accCharacteristic = applicationService->createCharacteristic(
        ACC_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    battCharacteristic = applicationService->createCharacteristic(
        BATT_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );





    temperatureCharacteristic->setValue("INITIALSTRING");
    heartRateCharacteristic->setValue("INITIALSTRING");
    accCharacteristic->setValue("INITIALSTRING");
    battCharacteristic->setValue("INITIALSTRING");

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
