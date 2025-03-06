#include <Arduino.h>
#include "BLEDevice.h"

// UUIDs for the BLE Service and Characteristic
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static BLEClient* pClient; // Global BLEClient pointer

// Notification Callback Function
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  Serial.print("Notification Received - Data Length: ");
  Serial.println(length);

  // Convert received bytes to an integer (Counter Value)
  int counter = atoi((char*)pData);

  Serial.print("Counter Value from Server: ");
  Serial.println(counter);

  // Retrieve and print the RSSI value
  if (pClient->isConnected()) {
    int rssi = pClient->getRssi();
    Serial.print("RSSI of Received Packet: ");
    Serial.print(rssi);
    Serial.println(" dBm");
  } else {
    Serial.println("Client not connected; unable to retrieve RSSI.");
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Connected to BLE Server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from server. Restarting scan...");
  }
};

// Connect to BLE Server
bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  pClient = BLEDevice::createClient();
  Serial.println("Client Created");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remote BLE Server
  if (!pClient->connect(myDevice)) {
    Serial.println("Connection failed");
    return false;
  }
  Serial.println("Connected to server");
  pClient->setMTU(517);  // Set max MTU

  // Get the Service
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Service UUID not found");
    pClient->disconnect();
    return false;
  }
  Serial.println("Service found");

  // Get the Characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Characteristic UUID not found");
    pClient->disconnect();
    return false;
  }
  Serial.println("Characteristic found");

  // Read initial characteristic value
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("Initial Characteristic Value: ");
    Serial.println(value.c_str());
  }

  // Enable Notifications
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Notifications Enabled");
  }

  connected = true;
  return true;
}

// BLE Scan Callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Found BLE Device: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Client");

  BLEDevice::init("");

  // Start Scanning
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  // Attempt connection if flagged
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to BLE Server");
    } else {
      Serial.println("Connection failed");
    }
    doConnect = false;
  }

  // Restart scanning if disconnected
  if (!connected && doScan) {
    BLEDevice::getScan()->start(0);
  }

  delay(1000);
}
