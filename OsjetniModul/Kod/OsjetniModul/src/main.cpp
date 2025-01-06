#include <Arduino.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

int dsTempPin = 8; // DS18B20 Temperature Sensor

int cap1Pin = 2; // Capacitive Sensor 1
int cap2Pin = 3; // Capacitive Sensor 2
int cap3Pin = 4; // Capacitive Sensor 3
int cap4Pin = 5; // Capacitive Sensor 4
int cap5Pin = 6; // Capacitive Sensor 5
int cap6Pin = 7; // Capacitive Sensor 6

int RX_PIN = 9; // Software Serial RX -> MAX485 RO
int TX_PIN = 11; // Software Serial TX -> MAX485 DI

SoftwareSerial rs485Serial(RX_PIN, TX_PIN);

int fotoSensorPin = A0; // Foto Sensor

int KTypeMiso = A5; // K-Type Thermocouple MISO
int KTypeCs = A6; // K-Type Thermocouple CS
int KTypeSck = A7; // K-Type Thermocouple SCK

int ds1Temp = 0;
int ds2Temp = 0;
int ds3Temp = 0;
int ds4Temp = 0; // TODO: maybe not needed

int cap1State = 0;
int cap2State = 0;
int cap3State = 0;
int cap4State = 0;
int cap5State = 0;
int cap6State = 0;

int fotoSensorValue = 0;
int kTypeTemp = 0;

char[] dataToSend = new char[64];
void CreateReply(int ds1Temp, int ds2Temp, int ds3Temp, int ds4Temp, int cap1State, int cap2State, int cap3State, int cap4State, int cap5State, int cap6State, int fotoSensorValue, int kTypeTemp)
int ReadDsSensor(int sensorPin);
void setup() 
{
  rs485Serial.begin(9600);
}

void loop() 
{
  // put your main code here, to run repeatedly:
}



void CreateReply(int ds1Temp, int ds2Temp, int ds3Temp, int ds4Temp, int cap1State, int cap2State, int cap3State, int cap4State, int cap5State, int cap6State, int fotoSensorValue, int kTypeTemp)
{
  rs485Serial.Write(xD1);
  rs485Serial.Write(ds1Temp);
  rs485Serial.Write(0xD2);
  rs485Serial.Write(ds2Temp);
  rs485Serial.Write(0xD3);
  rs485Serial.Write(ds3Temp);
  rs485Serial.Write(0xD4);
  rs485Serial.Write(ds4Temp);

  rs485Serial.Write(0xC1);
  rs485Serial.Write(cap1State == HIGH ? 1 : 0);
  rs485Serial.Write(0xC2);
  rs485Serial.Write(cap2State == HIGH ? 1 : 0);
  rs485Serial.Write(0xC3);
  rs485Serial.Write(cap3State == HIGH ? 1 : 0);
  rs485Serial.Write(0xC4);
  rs485Serial.Write(cap4State == HIGH ? 1 : 0);
  rrs485Serial.Write(0xC5);
  rs485Serial.Write(cap5State == HIGH ? 1 : 0);
  rs485Serial.Write(0xC6);
  rs485Serial.Write(cap6State == HIGH ? 1 : 0);

  rs485Serial.Write(0xF1);
  rs485Serial.Write(fotoSensorValue);

  rs485Serial.Write(0xE1); // K-Type Thermocouple
  rs485Serial.Write(kTypeTemp);
} 