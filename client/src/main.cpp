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

int rssi[32];
int rssi_i = 0;
double rssi_avg = 0;

// Flag to indicate when to read RSSI
static boolean readRSSI = false;

// Notification Callback Function
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  
  // Serial.print("Notification Received - Data Length: ");
  // Serial.println(length);

  // Convert received bytes to an integer (Counter Value)
  int counter;
  memcpy(&counter, pData, sizeof(counter));

  // Serial.print("Counter Value from Server: ");
  // Serial.println(counter);

  // Set flag to read RSSI in the main loop
  // readRSSI = true;
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
  } else {
    Serial.println("Failed to enable notifications");
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
      Serial.print("Found BLE Device with service UUID: ");
      Serial.println(advertisedDevice.toString().c_str());
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
  pBLEScan->start(60, false); // Scan for 5 seconds
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

  // Read RSSI if flag is set
  if (connected) {
    rssi[rssi_i] = pClient->getRssi();
    rssi_i < 31 ? rssi_i++ : rssi_i = 0;

    // Calculate average RSSI
    rssi_avg = 0.0;
    for (int i = 0; i < 32; i++) {
      rssi_avg += (double)rssi[i];
    }
    rssi_avg /= 32.0;

    Serial.print("RSSI_avg of Received Packet: ");
    Serial.print(rssi_avg);
    Serial.print(" dBm");
    Serial.print(" -- ");
    // double alpha = 1.2;
    // double d = pow(10, (-60.0 - rssi_avg) / (10.0 * alpha));

    // ITU-R P.1238-7 model
    double ptx = -8.0;  // Power transmitted in dBm
    double f = 2.4e9;  // Frequency in Hz
    double N = 28.0;  // Coefficient for 2.4 GHz in residential area
    double d = pow(10, ((ptx - rssi_avg - 20 * log10(f) + 28) / N));  // Distance in meters

    // readRSSI = false; 
    Serial.print("Distance to Server: ");
    Serial.print(d);
    Serial.println(" meters");
  }

  // Restart scanning if disconnected
  if (!connected && doScan) {
    BLEDevice::getScan()->start(0);
  }

  delay(50);
}
