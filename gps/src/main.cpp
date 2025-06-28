#include <Arduino.h>
#include <Adafruit_GPS.h>

// GPS Enable pin
#define GPS_ENABLE_PIN 5
#define GPS_LED_PIN 11  // LED pin to indicate GPS fix

// GPS Serial config (Serial2, RX=9, TX=10)
#define GPS_RX 10
#define GPS_TX 9
#define GPS_BAUD 115200  // Target GPS baud rate

// Create GPS object using Serial2
Adafruit_GPS GPS(&Serial2);

#define GPSECHO true

void setup() {
  // Start USB Serial for debug
  Serial.begin(115200);
  while (!Serial);  // Wait for USB connection
  Serial.println("ESP32-S3 GPS Test with Enable Pin");

  // Enable GPS module
  pinMode(GPS_ENABLE_PIN, OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, HIGH);
  delay(200); // Allow GPS to power up

  // Initialize LED pin
  pinMode(GPS_LED_PIN, OUTPUT);
  digitalWrite(GPS_LED_PIN, LOW); // Start with LED off

  // Step 1: Start at default baud (9600)
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  GPS.begin(9600);
  delay(100);

  // Step 2: Send command to change GPS to 115200 baud
  GPS.sendCommand("$PMTK251,115200*1F");  // Change baud rate
  delay(100);

  // Step 3: Switch Serial2 and GPS object to 115200
  Serial2.flush();
  Serial2.end();
  delay(100);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  GPS.begin(GPS_BAUD);

  // Step 4: Send GPS init commands
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Output RMC+GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);    // Update every 0.5 sec
  GPS.sendCommand(PGCMD_ANTENNA);               // Optional antenna info

  delay(1000);
  Serial.println("GPS initialized at 115200 baud.");
}

void loop() {
  // Read raw data from GPS
  char c = GPS.read();
  if (GPSECHO && c) Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;

    if (GPS.fix) {
      digitalWrite(GPS_LED_PIN, HIGH);  // Turn on LED if fix is valid

      Serial.println("------ GPS Info ------");
      Serial.print("Time: ");
      Serial.print(GPS.hour); Serial.print(':');
      Serial.print(GPS.minute); Serial.print(':');
      Serial.println(GPS.seconds);

      Serial.print("Date: ");
      Serial.print(GPS.day); Serial.print('/');
      Serial.print(GPS.month); Serial.print("/20");
      Serial.println(GPS.year);

      Serial.print("Fix Quality: "); Serial.println((int)GPS.fixquality);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

      Serial.print("Latitude: "); Serial.print(GPS.latitude, 4); Serial.println(GPS.lat);
      Serial.print("Longitude: "); Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
    } else {
      digitalWrite(GPS_LED_PIN, LOW);   // Turn off LED if no fix
      Serial.println("Waiting for GPS fix...");
    }
  }
}
