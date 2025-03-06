#include <Arduino.h>
#include <HardwareSerial.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  int result = myFunction(2, 3);
  pinMode(13, OUTPUT);
}

void loop() {
  Serial.println("Hello World");
  
  digitalWrite(13, 1);
  delay(1000);
  digitalWrite(13, 0);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}