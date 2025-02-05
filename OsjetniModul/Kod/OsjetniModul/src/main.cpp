#include <Arduino.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "max6675.h"

int dsTempPin = 8; // DS18B20 Temperature Sensor
OneWire oneWire(dsTempPin);

DallasTemperature sensors(&oneWire);
int numberOfDevices;
DeviceAddress tempDeviceAddress;

int cap1Pin = 2; // Capacitive Sensor 1
int cap2Pin = 3; // Capacitive Sensor 2
int cap3Pin = 4; // Capacitive Sensor 3
int cap4Pin = 5; // Capacitive Sensor 4
int cap5Pin = 6; // Capacitive Sensor 5
int cap6Pin = 7; // Capacitive Sensor 6
int cap7Pin = 13; // Capacitive Sensor 7

int RX_PIN = 9;  // Software Serial RX -> MAX485 RO
int TX_PIN = 11; // Software Serial TX -> MAX485 DI
int TXEnable = 12; // DE,RE pin for MAX485

SoftwareSerial rs485Serial(RX_PIN, TX_PIN);

int fotoSensorPin = A0; // Foto Sensor

int KTypeMiso = A5; // K-Type Thermocouple MISO
int KTypeCs = A6;   // K-Type Thermocouple CS
int KTypeSck = A7;  // K-Type Thermocouple SCK

MAX6675 thermocouple(KTypeSck, KTypeCs, KTypeMiso);

uint8_t adresses[2][8] = {
    {0x28, 0x1E, 0x4D, 0x50, 0x00, 0x00, 0x00, 0x8B},
    {0x28, 0x73, 0xEA, 0x52, 0x00, 0x00, 0x00, 0x67}};

int ds1Temp = 0;
int ds2Temp = 0;
int ds3Temp = 0;
int ds4Temp = 0; // TODO: maybe not needed

unsigned long temperatureLastReadout;
unsigned long temperatureReadoutInterval = 10000; // 10 seconds

int cap1State = 0;
int cap2State = 0;
int cap3State = 0;
int cap4State = 0;
int cap5State = 0;
int cap6State = 0;
int cap7State = 0;

int fotoSensorValue = 0;
int kTypeTemp = 0;

void CreateReply();
int ReadDsSensor(int sensorPin);
void printAddress(DeviceAddress deviceAddress);
uint8_t findDevices(int pin);
void ReceiveDataFromMaster();

void setup()
{
  temperatureLastReadout = millis();

  rs485Serial.begin(9600);

  numberOfDevices = sensors.getDS18Count();
  Serial.begin(9600);
  // locate devices on the bus
  findDevices(dsTempPin);
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
}

void loop()
{
  cap1State = digitalRead(cap1Pin);
  cap2State = digitalRead(cap2Pin);
  cap3State = digitalRead(cap3Pin);
  cap4State = digitalRead(cap4Pin);
  cap5State = digitalRead(cap5Pin);
  cap6State = digitalRead(cap6Pin);
  cap7State = digitalRead(cap7Pin);
  fotoSensorValue = analogRead(fotoSensorPin);

  if (temperatureLastReadout + temperatureReadoutInterval < millis())
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    for (int i = 0; i < numberOfDevices; i++)
    {
      // Search the wire for address
      float tempC = sensors.getTempC(adresses[i]);
      if (i == 0)
        ds1Temp = (int)tempC;
      else if (i == 1)
        ds2Temp = (int)tempC;
      else if (i == 2)
        ds3Temp = (int)tempC;
      else if (i == 3)
        ds4Temp = (int)tempC;
    }

    kTypeTemp = thermocouple.readCelsius();
    temperatureLastReadout = millis();
  }

  ReceiveDataFromMaster();
}

void ReceiveDataFromMaster()
{
  if (rs485Serial.available() > 0)
  {
    int incomingByte = rs485Serial.read();
    if (incomingByte == 0xAA)
    {
      CreateReply();
    }
  }
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

uint8_t findDevices(int pin)
{
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;

  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do
    {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10)
          Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7)
          Serial.print(", ");
      }
      Serial.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }

  return count;
}

void CreateReply()
{
  digitalWrite(TXEnable, HIGH);
  rs485Serial.write(static_cast<uint8_t>(0x00));

  rs485Serial.write(static_cast<uint8_t>(0xD1));
  rs485Serial.write(static_cast<uint8_t>(ds1Temp));

  rs485Serial.write(static_cast<uint8_t>(0xD2));
  rs485Serial.write(static_cast<uint8_t>(ds2Temp));

  rs485Serial.write(static_cast<uint8_t>(0xD3));
  rs485Serial.write(static_cast<uint8_t>(ds3Temp));

  rs485Serial.write(static_cast<uint8_t>(0xD4));
  rs485Serial.write(static_cast<uint8_t>(ds4Temp));

  rs485Serial.write(static_cast<uint8_t>(0xC1));
  rs485Serial.write(static_cast<uint8_t>(cap1State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC2));
  rs485Serial.write(static_cast<uint8_t>(cap2State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC3));
  rs485Serial.write(static_cast<uint8_t>(cap3State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC4));
  rs485Serial.write(static_cast<uint8_t>(cap4State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC5));
  rs485Serial.write(static_cast<uint8_t>(cap5State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC6));
  rs485Serial.write(static_cast<uint8_t>(cap6State == HIGH ? 1 : 0));
  rs485Serial.write(static_cast<uint8_t>(0xC7));
  rs485Serial.write(static_cast<uint8_t>(cap7State == HIGH ? 1 : 0));

  rs485Serial.write(static_cast<uint8_t>(0xF1));
  rs485Serial.write(static_cast<uint8_t>(fotoSensorValue));

  rs485Serial.write(static_cast<uint8_t>(0xE1)); // K-Type Thermocouple
  rs485Serial.write(static_cast<uint8_t>(kTypeTemp));
  rs485Serial.write(static_cast<uint8_t>(0xFF));
  digitalWrite(TXEnable, LOW);
}