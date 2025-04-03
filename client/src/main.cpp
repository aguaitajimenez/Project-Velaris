#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>

#define CONNECTED_LED_PIN 13
#define SERVER_NAME "ESP32_Advertiser"

// UUID del servicio y características
static BLEUUID serviceUUID("eab293ea-ab75-4fa3-a21f-66a937c57000");
static BLEUUID tempUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc000");
static BLEUUID hrUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc001");
static BLEUUID accXUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc002");
static BLEUUID accYUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc003");
static BLEUUID accZUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc004");

// Flags y punteros
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = true;
static BLEAdvertisedDevice* myDevice;

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* tempChar;
BLERemoteCharacteristic* hrChar;
BLERemoteCharacteristic* accXChar;
BLERemoteCharacteristic* accYChar;
BLERemoteCharacteristic* accZChar;

// Callback de notificación
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    String value = String((char*)pData, length);
    Serial.print("Notify from ");
    Serial.print(pChar->getUUID().toString().c_str());
    Serial.print(": ");
    Serial.print(value);
    Serial.println();
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) override {
    digitalWrite(CONNECTED_LED_PIN, HIGH);
    Serial.println("Connected to BLE server.");
  }

  void onDisconnect(BLEClient* pClient) override {
    digitalWrite(CONNECTED_LED_PIN, LOW);
    Serial.println("Disconnected from BLE server.");
    connected = false;
    doScan = true;
  }
};

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  if (myDevice->haveName()) {
    Serial.print("Device name: ");
    Serial.println(myDevice->getName().c_str());
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);

  BLERemoteService* pService = pClient->getService(serviceUUID);
  if (!pService) {
    Serial.println("Failed to find service.");
    pClient->disconnect();
    return false;
  }

  tempChar = pService->getCharacteristic(tempUUID);
  hrChar   = pService->getCharacteristic(hrUUID);
  accXChar = pService->getCharacteristic(accXUUID);
  accYChar = pService->getCharacteristic(accYUUID);
  accZChar = pService->getCharacteristic(accZUUID);

  if (tempChar && tempChar->canNotify()) tempChar->registerForNotify(notifyCallback);
  if (hrChar   && hrChar->canNotify())   hrChar->registerForNotify(notifyCallback);
  if (accXChar && accXChar->canNotify()) accXChar->registerForNotify(notifyCallback);
  if (accYChar && accYChar->canNotify()) accYChar->registerForNotify(notifyCallback);
  if (accZChar && accZChar->canNotify()) accZChar->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

// Escaneo BLE
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    Serial.print("Device found: ");
    Serial.print(advertisedDevice.getAddress().toString().c_str());

    if (advertisedDevice.haveName()) {
      Serial.print(" Name: ");
      Serial.print(advertisedDevice.getName().c_str());
    }

    Serial.print(" RSSI: ");
    Serial.println(advertisedDevice.getRSSI());

    if (advertisedDevice.haveName() && advertisedDevice.getName() == SERVER_NAME) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(CONNECTED_LED_PIN, OUTPUT);
  digitalWrite(CONNECTED_LED_PIN, LOW);

  Serial.println("Starting BLE Client...");
  BLEDevice::init("");

  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected and subscribed to notifications.");
    } else {
      Serial.println("Connection failed.");
    }
    doConnect = false;
  }

  if (!connected && doScan) {
    BLEDevice::getScan()->start(0); // restart scan
    doScan = false;
  }

  if(connected){
    int rssi = pClient->getRssi();
        Serial.print(" RSSI: ");
        Serial.println(rssi);
  }

  delay(1000);
}
