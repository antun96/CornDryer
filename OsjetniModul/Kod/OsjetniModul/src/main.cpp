#include <Arduino.h>

int dsTempPin = 8; // DS18B20 Temperature Sensor

int cap1Pin = 2; // Capacitive Sensor 1
int cap2Pin = 3; // Capacitive Sensor 2
int cap3Pin = 4; // Capacitive Sensor 3
int cap4Pin = 5; // Capacitive Sensor 4
int cap5Pin = 6; // Capacitive Sensor 5
int cap6Pin = 7; // Capacitive Sensor 6

int softwareSerialRx = 9; // Software Serial RX -> MAX485 RO
int softwareSerialTx = 11; // Software Serial TX -> MAX485 DI

int fotoSensorPin = A0; // Foto Sensor

int KTypeMiso = A5; // K-Type Thermocouple MISO
int KTypeCs = A6; // K-Type Thermocouple CS
int KTypeSck = A7; // K-Type Thermocouple SCK


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
} 