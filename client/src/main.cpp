#include "main.h"

#define UART_TRACES 1
#define PERIOD_MS 500

// UUID del servicio y características
static BLEUUID serviceUUID("eab293ea-ab75-4fa3-a21f-66a937c57000");

static BLEUUID tempUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc000");
static BLEUUID hrUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc001");
static BLEUUID accXUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc002");
static BLEUUID accYUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc003");
static BLEUUID accZUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc004");
static BLEUUID battVUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc005");
static BLEUUID battPUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc006");

// Flags y punteros
static boolean bl_detected_f = false;
static boolean bl_connected_f = false;
bool lora_config_f = false;



static BLEAdvertisedDevice *myDevice;

BLEClient *pClient = nullptr;
BLERemoteCharacteristic *tempChar;
BLERemoteCharacteristic *hrChar;
BLERemoteCharacteristic *accXChar;
BLERemoteCharacteristic *accYChar;
BLERemoteCharacteristic *accZChar;
BLERemoteCharacteristic *battVChar;
BLERemoteCharacteristic *battPChar;

typedef enum
{
  bl_idle,
  bl_transmission,
  bl_search,
} bluetooth_state_t;

bluetooth_state_t bluetooth_state = bl_idle;


typedef enum{
  lora_idle,
  lora_transmission,
} lora_state_t;

lora_state_t lora_state = lora_idle;

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


// Lora
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // Already declared in your code
#define MAX_LORA_PACKET_LEN 255
char loraPacket[MAX_LORA_PACKET_LEN];



class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pClient) override
  {
    digitalWrite(CONNECTED_LED_PIN, HIGH);
    Serial.println("Connected to BLE server.");
  }

  void onDisconnect(BLEClient *pClient) override
  {
    digitalWrite(CONNECTED_LED_PIN, LOW);
    Serial.println("Disconnected from BLE server.");
    bl_connected_f = false;
  }
};

bool connectToServer()
{
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  if (myDevice->haveName())
  {
    Serial.print("Device name: ");
    Serial.println(myDevice->getName().c_str());
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);

  BLERemoteService *pService = pClient->getService(serviceUUID);
  if (!pService)
  {
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
      bl_detected_f = true;
    }
  }
};

WiFiUDP udp;
char packetBuffer[256];
unsigned int localPort = 8000;
const char *ssid = "WhiteSky-Ion";
const char *password = "32889754";

void setup()
{
  Serial.begin(5000000);
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

  xTaskCreate(
      bluetooth_task,
      "bluetooth_task", // Task name
      20480,       // stack size
      NULL,        // Task parameters
      3,           // Task priority
      NULL         // Task handler
  );

  xTaskCreate(
      lora_task,
      "bluetooth_task", // Task name
      20480,       // stack size
      NULL,        // Task parameters
      2,           // Task priority
      NULL         // Task handler
  );
}

void bluetooth_task(void *parameters){

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_MS); // Convert 200 ms to ticks
  xLastWakeTime = xTaskGetTickCount();

  bluetooth_state_t next_bluetooth_state = bl_idle;

  while (1){

    // **************
    // State function
    // **************
    #if UART_TRACES
    Serial.println("Bluetooth state:" + bluetooth_state);
    #endif

    switch (bluetooth_state){
    case bl_idle:
      if(bl_detected_f){
        bl_detected_f = false;
        bl_connected_f = connectToServer();
        next_bluetooth_state = bl_transmission;
      }

#if UART_TRACES
      bl_connected_f ? Serial.println("Connected to notifications.") :
      Serial.println("Connection failed.");
#endif

      break;

    case bl_transmission:
      if(bl_connected_f){
        blRead();
        sendJSON();
        next_bluetooth_state = bl_transmission;
      }
      if(!bl_connected_f){
        next_bluetooth_state = bl_search;
      }
      break;

    case bl_search:
        BLEDevice::getScan()->start(0); // restart scan
        next_bluetooth_state = bl_idle;
      break;

    default:
      next_bluetooth_state = bl_search;
      break;
    }

    bluetooth_state = next_bluetooth_state;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void lora_task(void *parameters){

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_MS); // Convert 200 ms to ticks
  xLastWakeTime = xTaskGetTickCount();

  lora_state_t next_lora_state = lora_idle;

  while (1){
  // ***************
  // State functions
  // ***************

  #if UART_TRACES
  Serial.println("LoRa state:" + lora_state);
  #endif

  switch (lora_state){
    case lora_idle:
      if(!bl_connected_f){
        lora_config_f = loraConfig();
        next_lora_state = lora_transmission;
      }
      break;

    case lora_transmission:
      if(!bl_connected_f){
        loraReceiveAndForward();
        next_lora_state = lora_transmission;
      }
      if(bl_connected_f){
        next_lora_state = lora_idle;
      }
      if(!lora_config_f){
        next_lora_state = lora_idle;
      }
      break;
    
    default:
      next_lora_state = lora_idle;
      break;
    }

    lora_state = next_lora_state;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

bool blRead(){
  try{
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
  }
  catch (const std::exception &e){
    return 1;
  }
  return 0;
}

bool sendJSON(){
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
    try{
      udp.beginPacket(iotIP, localPort);
      udp.print(json);
      udp.endPacket();
    }
    catch (const std::exception &e){
      return 1;
    }

#if UART_TRACES
    Serial.print("Packet sent to ");
    Serial.println(iotIP);
    Serial.print("Payload:");
    Serial.println(json);
#endif
  }
  else{
    return 1;
  }
  return 0;
}


bool loraConfig() {
  if (!rf95.init()) {
    Serial.println("LoRa init failed!");
    return false;
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("LoRa frequency setup failed!");
    return false;
  }

  rf95.setTxPower(14, false); // 14 dBm, useRFO = false
  Serial.println("LoRa config OK");
  return true;
}

bool loraReceiveAndForward() {
  if (rf95.available()) {
    uint8_t len = sizeof(loraPacket);
    if (rf95.recv((uint8_t *)loraPacket, &len)) {
      loraPacket[len] = '\0';  // Null-terminate

      Serial.println("LoRa packet received:");
      Serial.println(loraPacket);

      IPAddress iotIP;
      if (WiFi.hostByName("iot.local", iotIP)) {
        udp.beginPacket(iotIP, localPort);
        udp.print(loraPacket);
        udp.endPacket();

#if UART_TRACES
        Serial.print("LoRa UDP packet sent to ");
        Serial.println(iotIP);
        Serial.print("Payload: ");
        Serial.println(loraPacket);
#endif

        return true;
      } else {
        Serial.println("Failed to resolve iot.local");
        return false;
      }
    } else {
      Serial.println("LoRa packet receive failed.");
    }
  }

  return false;
}

void loop(){
  
}
