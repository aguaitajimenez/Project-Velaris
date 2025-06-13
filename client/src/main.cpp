#include "main.h"

// UUID del servicio y características
static BLEUUID serviceUUID("eab293ea-ab75-4fa3-a21f-66a937c57000");

static BLEUUID tempUUID ("6bd16e28-6f99-40b9-abe5-dfc1ad6dc000");
static BLEUUID hrUUID   ("6bd16e28-6f99-40b9-abe5-dfc1ad6dc001");
static BLEUUID accXUUID ("6bd16e28-6f99-40b9-abe5-dfc1ad6dc002");
static BLEUUID accYUUID ("6bd16e28-6f99-40b9-abe5-dfc1ad6dc003");
static BLEUUID accZUUID ("6bd16e28-6f99-40b9-abe5-dfc1ad6dc004");
static BLEUUID battVUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc005");
static BLEUUID battPUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc006");

// Flags y punteros
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = true;
static BLEAdvertisedDevice *myDevice;

BLEClient *pClient = nullptr;
BLERemoteCharacteristic *tempChar;
BLERemoteCharacteristic *hrChar;
BLERemoteCharacteristic *accXChar;
BLERemoteCharacteristic *accYChar;
BLERemoteCharacteristic *accZChar;
BLERemoteCharacteristic *battVChar;
BLERemoteCharacteristic *battPChar;

const int device_id = 1;
String bpm = "0";
String temp = "0";
String accX = "0";
String accY = "0";
String accZ = "0";
String wrist_battV = "0";
String wrist_battP = "0";
String my_battV = "0";
String my_battP = "0";
int rssi = 0;

uint64_t time_request = 10000;
const uint64_t request_period = 500e3;

class MyClientCallback : public BLEClientCallbacks{
  void onConnect(BLEClient *pClient) override{
    digitalWrite(CONNECTED_LED_PIN, HIGH);
    Serial.println("Connected to BLE server.");
  }

  void onDisconnect(BLEClient *pClient) override{
    digitalWrite(CONNECTED_LED_PIN, LOW);
    Serial.println("Disconnected from BLE server.");
    connected = false;
    doScan = true;
  }
};

bool connectToServer(){
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  if (myDevice->haveName()){
    Serial.print("Device name: ");
    Serial.println(myDevice->getName().c_str());
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);

  BLERemoteService *pService = pClient->getService(serviceUUID);
  if (!pService){
    Serial.println("Failed to find service.");
    pClient->disconnect();
    return false;
  }

  tempChar = pService->getCharacteristic(tempUUID);
  hrChar = pService->getCharacteristic(hrUUID);
  accXChar = pService->getCharacteristic(accXUUID);
  accYChar = pService->getCharacteristic(accYUUID);
  accZChar = pService->getCharacteristic(accZUUID);
  battVChar = pService->getCharacteristic(battVUUID);
  battPChar = pService->getCharacteristic(battPUUID);

  connected = true;
  return true;
}

// Escaneo BLE
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice) override{
    Serial.print("Device found: ");
    Serial.print(advertisedDevice.getAddress().toString().c_str());

    if (advertisedDevice.haveName()){
      Serial.print(" Name: ");
      Serial.print(advertisedDevice.getName().c_str());
    }

    Serial.print(" RSSI: ");
    Serial.println(advertisedDevice.getRSSI());

    if (advertisedDevice.haveName() && advertisedDevice.getName() == SERVER_NAME){
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

WiFiUDP udp;
char packetBuffer[255];
unsigned int localPort = 8000;
const char *ssid = "WhiteSky-Ion";
const char *password = "32889754";

void setup(){
  Serial.begin(115200);
  Wire.begin();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(200);
    Serial.print(F("."));
  }
  udp.begin(localPort);
  Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);

  pinMode(CONNECTED_LED_PIN, OUTPUT);
  digitalWrite(CONNECTED_LED_PIN, LOW);

  Serial.println("Starting BLE Client...");
  BLEDevice::init("");

  BLEScan *pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
}

void loop(){
  if (doConnect){
    if (connectToServer()){
      Serial.println("Connected and subscribed to notifications.");
    }
    else{
      Serial.println("Connection failed.");
    }
    doConnect = false;
  }

  if (!connected && doScan){
    BLEDevice::getScan()->start(0); // restart scan
    doScan = false;
  }

  if (connected && ((esp_timer_get_time()-time_request) > request_period)){
    
    temp = tempChar->readValue().c_str();
    bpm = hrChar->readValue().c_str();
    accX = accXChar->readValue().c_str();
    accY = accYChar->readValue().c_str();
    accZ = accZChar->readValue().c_str();
    wrist_battV = battVChar->readValue().c_str();
    wrist_battP = battPChar->readValue().c_str();
    my_battV = readBattVoltage();
    my_battP = readBattPercentage();
    
    rssi = pClient->getRssi();

    IPAddress iotIP;
    if (WiFi.hostByName("iot.local", iotIP)){

      String json = "{";
      json += "\"id\":\"" + String(device_id) + "\",";
      json += "\"rssi\":" + String(rssi) + ",";
      json += "\"temperature\":" + temp + ",";
      json += "\"acceleration\":{";
      json += "\"x\":" + accX + ",";
      json += "\"y\":" + accY + ",";
      json += "\"z\":" + accZ;
      json += "},";
      json += "\"bpm\":" + bpm + ",";
      json += "\"wrist_battV\":" + wrist_battV + ",";
      json += "\"wrist_battP\":" + wrist_battP + ",";
      json += "\"my_battV\":" + my_battV + ",";
      json += "\"my_battP\":" + my_battP;
      json += "}";

      // Serial.print("Payload: ");
      // Serial.println(json);

      udp.beginPacket(iotIP, localPort);
      udp.print(json);
      udp.endPacket();

      Serial.print("Packet sent to ");
      Serial.println(iotIP);
      Serial.print("Payload:");
      Serial.println(json);
    }
    else{
      Serial.println("Hostname resolution failed.");
    }

    delay(498);
  }
}

float readBattVoltage(){
  Wire.beginTransmission(MAX17048_I2CADDR_DEFAULT);
  Wire.write(VBATT_ADDRESS); // Voltage register
  Wire.endTransmission(false);
  Wire.requestFrom(MAX17048_I2CADDR_DEFAULT, 2);

  if (Wire.available() == 2){
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

  if (Wire.available() == 2){
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t percentage_raw = ((uint16_t)msb << 8) | lsb;
    //   voltage_raw >>= 4; // lower 4 bits are not used
    return percentage_raw * 0.00390635; // each bit = 1.25 mV
  }
  return -1.0; // error
}