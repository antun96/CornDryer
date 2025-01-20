#include <Arduino.h>
#include <SoftwareSerial.h>

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

unsigned long lastCommandSent;
unsigned long commandInterval = 1000; // 1 second

int RX_PIN = 4;  // Software Serial RX -> MAX485 RO
int TX_PIN = 2; // Software Serial TX -> MAX485 DI
int TXEnable = 3; // DE,RE pin for MAX485

SoftwareSerial rs485Serial(RX_PIN, TX_PIN);

void ReceiveDataFromSlave();
void SendCommandToSlave();
void ParseMessage(uint8_t *message, int length);

void setup()
{
  lastCommandSent = millis();
}

void loop()
{
  if(millis() - commandInterval > lastCommandSent) {
    SendCommandToSlave();
  }
  
  ReceiveDataFromSlave();
}

void ReceiveDataFromSlave()
{
  if (rs485Serial.available() > 0)
  {
    const int bufferSize = 64; // Adjust buffer size as needed
    uint8_t buffer[bufferSize];
    int length = 0;
    int cache = 0;

    // Read data from SoftwareSerial into buffer
    while (rs485Serial.available() && length < bufferSize) {
      cache = rs485Serial.read();
      if(cache == 0xFF) {
        break;
      }
      buffer[length++] = cache;
    }
    ParseMessage(buffer, length);
  }
}

void SendCommandToSlave()
{
    digitalWrite(TXEnable, HIGH);
    rs485Serial.write(0xAA);
    digitalWrite(TXEnable, LOW);
    lastCommandSent = millis();
}

void ParseMessage(uint8_t *message, int length)
{
  int index = 0;
  while (index < length)
  {
    uint8_t id = message[index++];
    uint8_t value = message[index++];

    switch (id)
    {
    case 0xD1:
      ds1Temp = value;
      break;
    case 0xD2:
      ds2Temp = value;
      break;
    case 0xD3:
      ds3Temp = value;
      break;
    case 0xD4:
      ds4Temp = value;
      break;
    case 0xC1:
      cap1State = value;
      break;
    case 0xC2:
      cap2State = value;
      break;
    case 0xC3:
      cap3State = value;
      break;
    case 0xC4:
      cap4State = value;
      break;
    case 0xC5:
      cap5State = value;
      break;
    case 0xC6:
      cap6State = value;
      break;
    case 0xF1:
      fotoSensorValue = value;
      break;
    case 0xE1:
      kTypeTemp = value;
      break;
    case 0xFF:
      // End of message
      break;
    default:
      // Unknown ID
      break;
    }
  }
}