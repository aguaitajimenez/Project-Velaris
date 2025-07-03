#include "main.h"

#define UART_TRACES 1

#define PERIOD_MS 1000

#define BLUETOOTH_PIN 18
#define STATE_LED 13


// UUID del servicio y características
static BLEUUID serviceUUID("eab293ea-ab75-4fa3-a21f-66a937c57000");
static BLEUUID tempUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc000");
static BLEUUID hrUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc001");
static BLEUUID accUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc002");
static BLEUUID battUUID("6bd16e28-6f99-40b9-abe5-dfc1ad6dc005");


// Flags y punteros
static bool bl_detected_f = false;
static bool bl_connected_f = false;
static bool bl_notify_f = false;
static bool bl_scan_f = false;
bool lora_config_f = false;


static BLEAdvertisedDevice *myDevice;

BLEClient *pClient = nullptr;
BLERemoteCharacteristic *tempChar;
BLERemoteCharacteristic *hrChar;
BLERemoteCharacteristic *accChar;
BLERemoteCharacteristic *battChar;


typedef enum{
  Bluetooth,
  LoRa
} client_state_t;

client_state_t state = LoRa;


// typedef enum{
//   lora_idle,
//   lora_transmission,
// } lora_state_t;

// lora_state_t lora_state = lora_idle;


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
  void onConnect(BLEClient *pClient) override{
    Serial.println("Connected to BLE server.");
  }

  void onDisconnect(BLEClient *pClient) override{
    Serial.println("Disconnected from BLE server.");
    bl_connected_f = false;
  }
};

void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
    std::string uuid = pChar->getUUID().toString();
    String value = String((char*)pData); // assumes data is null-terminated string

    char buffer[64];  // adjust size based on expected max payload
    size_t len = min(length, sizeof(buffer) - 1);
    memcpy(buffer, pData, len);
    buffer[len] = '\0';  // ensure null-terminated string

    if (uuid == tempUUID.toString()) {
        temp = String(buffer);
    } else if (uuid == hrUUID.toString()) {
        bpm = String(buffer);
    } else if (uuid == accUUID.toString()) {
          String full = String(buffer);
          int sep1 = full.indexOf(';');
          int sep2 = full.indexOf(';', sep1 + 1);

          if (sep1 > 0 && sep2 > sep1) {
              accX = full.substring(0, sep1);
              accY = full.substring(sep1 + 1, sep2);
              accZ = full.substring(sep2 + 1);
          }

    } else if (uuid == battUUID.toString()) {
            String full = String(buffer);
            int sep = full.indexOf(';');
            if (sep > 0) {
                wrist_battV = full.substring(0, sep);
                wrist_battP = full.substring(sep + 1);
      }
    }

    bl_notify_f = true;

}



bool connectToServer(){
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  if (myDevice->haveName())
  {
    Serial.print("Device name: ");
    Serial.println(myDevice->getName().c_str());
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice))
  {
    Serial.println("Failed to connect.");
    return false;
  }

  BLERemoteService *pService = pClient->getService(serviceUUID);
  if (!pService)
  {
    Serial.println("Failed to find service.");
    pClient->disconnect();
    return false;
  }

  tempChar = pService->getCharacteristic(tempUUID);
  hrChar = pService->getCharacteristic(hrUUID);
  accChar = pService->getCharacteristic(accUUID);
  battChar = pService->getCharacteristic(battUUID);

  if (tempChar && tempChar->canNotify()) tempChar->registerForNotify(notifyCallback);
  if (hrChar && hrChar->canNotify()) hrChar->registerForNotify(notifyCallback);
  if (accChar && accChar->canNotify()) accChar->registerForNotify(notifyCallback);
  if (battChar && battChar->canNotify()) battChar->registerForNotify(notifyCallback);

  bl_connected_f = true;
  return true;
}


// Escaneo BLE
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks{
  void onResult(BLEAdvertisedDevice advertisedDevice) override{
    

    if (advertisedDevice.haveName() && advertisedDevice.getName() == SERVER_NAME){
      Serial.print("Device found: ");
      Serial.print(advertisedDevice.getAddress().toString().c_str());

      if (advertisedDevice.haveName()){
        Serial.print(" Name: ");
        Serial.print(advertisedDevice.getName().c_str());
      }

      Serial.print(" RSSI: ");
      Serial.println(advertisedDevice.getRSSI());
      
      bl_detected_f = true;
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
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
  // Configure GPIO18 as input with internal pull-up
  pinMode(BLUETOOTH_PIN, INPUT_PULLUP);


  Wire.begin();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(100);
    Serial.print(F("."));
  }
  udp.begin(localPort);
  Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);

  pinMode(BL_LED, OUTPUT);
  digitalWrite(BL_LED, LOW);

  pinMode(LORA_LED, OUTPUT);
  digitalWrite(LORA_LED, LOW); // Start OFF

  pinMode(STATE_LED, OUTPUT);


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
      20480,            // stack size
      NULL,             // Task parameters
      2,                // Task priority
      NULL              // Task handler
  );

  xTaskCreate(
      scan_task,
      "scan_task", // Task name
      20480,            // stack size
      NULL,             // Task parameters
      1,                // Task priority
      NULL              // Task handler
  );
}

void bluetooth_task(void *parameters){
  client_state_t next_state = LoRa;
  bool lora_ret = 0;

  #if UART_TRACES
      bl_connected_f ? Serial.println("Connected to notifications.") :
      Serial.println("Connection failed.");
#endif

  lora_config_f = loraConfig();

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(PERIOD_MS); // Convert PERIOD_MS to ticks
  xLastWakeTime = xTaskGetTickCount();

  while (1){

    // **************
    // State function
    // **************

    #if UART_TRACES
    state==LoRa ? Serial.println("Lora State") :  Serial.println("Bluetooth State");
    #endif

    state==LoRa ? digitalWrite(STATE_LED, 1) : digitalWrite(STATE_LED, 0);
    bool bluetooth_pin = digitalRead(BLUETOOTH_PIN);

    switch (state){
    case LoRa:
      if(bl_connected_f){
        next_state = Bluetooth;
        break;
      }

      if(bl_detected_f && bluetooth_pin){
        bl_detected_f = 0;
        bl_connected_f = connectToServer();
        next_state = LoRa;
        break;
      }

      if((!bl_connected_f)){
        bl_scan_f = 1;
        lora_ret = loraReceiveAndForward();
        if(lora_ret){
          digitalWrite(LORA_LED, 1);
          vTaskDelay(pdMS_TO_TICKS(5)); // To see the blink
          digitalWrite(LORA_LED, 0);
        }
        next_state = LoRa;
        break;
      }

      break;

    case Bluetooth:
      if(!bluetooth_pin){
        pClient->disconnect();
        lora_config_f = loraConfig();
        next_state = LoRa;
        break;
      }

      if(bl_connected_f && bl_notify_f){
          bl_notify_f = false;
          digitalWrite(BL_LED, 1);
          sendJSON();
          digitalWrite(BL_LED, 0);
          next_state = Bluetooth;
          break;
      }

      if(!bl_connected_f){
        lora_config_f = loraConfig();
        next_state = LoRa;
      }
      break;

    default:
      next_state = LoRa;
      break;
    }

    state = next_state;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void scan_task(void *parameters){
  vTaskDelay(pdMS_TO_TICKS(500));
  while(1){
    if(bl_scan_f){
      bl_scan_f = 0;
      BLEDevice::getScan()->start(1);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
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
    accX = accChar->readValue().c_str();
    accY = accChar->readValue().c_str();
    accZ = accChar->readValue().c_str();
    wrist_battV = battChar->readValue().c_str();
    wrist_battP = battChar->readValue().c_str();
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
  my_battV = readBattVoltage(); 
  my_battP = readBattPercentage();
  IPAddress iotIP;
  if (WiFi.hostByName("iot.local", iotIP)){

    String json = "{";
    json += "\"id\":\"" + String(device_id) + "\",";  // id
    json += "\"rs\":" + String(rssi) + ",";           // rssi → rs
    json += "\"tmp\":" + temp + ",";                  // temperature → tmp
    json += "\"acc\":{";                              // acceleration → acc
    json += "\"x\":" + accX + ",";                    // x
    json += "\"y\":" + accY + ",";                    // y
    json += "\"z\":" + accZ;                          // z
    json += "},";
    json += "\"bpm\":" + bpm + ",";                   // bpm
    json += "\"wbv\":" + wrist_battV + ",";           // wrist batt voltage
    json += "\"wbp\":" + wrist_battP + ",";           // wrist batt percent
    json += "\"mbv\":" + my_battV + ",";              // my batt voltage
    json += "\"mbp\":" + my_battP;                    // my batt percent
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
    bool ret = false;
    uint8_t len = sizeof(loraPacket);

    // Drain all old packets, keep only the latest
    while (rf95.available()) {
        len = sizeof(loraPacket);
        if (rf95.recv((uint8_t *)loraPacket, &len)) {
            ret = true;
        }
    }

    if (ret) {
        loraPacket[len] = '\0';
        Serial.println("LoRa forward (latest packet):");
        Serial.println(loraPacket);

        IPAddress iotIP;
        if (WiFi.hostByName("iot.local", iotIP)) {
            udp.beginPacket(iotIP, localPort);
            udp.print(loraPacket);
            udp.endPacket();
            return true;
        } else {
            Serial.println("Failed to resolve iot.local");
            return false;
        }
    }
    return false;
}


void loop(){
  
}
