#include <Arduino.h>
#include <SoftwareSerial.h>
#include "EasyNextionLibrary.h"  // Include EasyNextionLibrary

/// @brief Pin for turning auger on/off
int augerPin = A0;
/// @brief Pin for turning horizontal auger on/off
int horizontalAugerPin = A1; 
/// @brief Pin for turning radial fan on/off
int radialFanPin = A2;
/// @brief Pin for turning heater on/off
int heaterPin = A3;
/// @brief Pin for turning cooling fan on/off
int coolingFanPin = A4;
/// @brief Pin for turning floor auger on/off
int floorAugerPin = A5;
/// @brief Pin for turning air blower on/off
int airBlowerPin = A6;
/// @brief Pin for turning buzzer on/off
int buzzerPin = A7;

unsigned long lastCommandSent;
unsigned long COMMAND_INTERVAL_TIME_MS = 1000; // 1 second
unsigned long const SOFT_START_DELAY_MS = 10000; // 10 seconds

int TX_PIN = 2; // Software Serial TX -> MAX485 DI
int TXEnable = 3; // DE,RE pin for MAX485
int RX_PIN = 4;  // Software Serial RX -> MAX485 RO

SoftwareSerial rs485Serial(RX_PIN, TX_PIN);

int NextionRX = 5; // Nextion RX
int NextionTX = 6; // Nextion TX
SoftwareSerial nextionSerial(NextionRX, NextionTX);
EasyNex nexDisplay(nextionSerial); 

/// @brief buffer lowest point temperature
int ds1Temp = 0;
/// @brief drying chamber lowest point temperature
int ds2Temp = 0;
/// @brief cooling chamber temperature
int ds3Temp = 0;
/// @brief Outside air temperature
int ds4Temp = 0;

uint8_t ds1Address[8];
uint8_t ds2Address[8];
uint8_t ds3Address[8];
uint8_t ds4Address[8];

/// @brief augerInput
int cap1State = 0;
/// @brief bufferFull
int cap2State = 0;
/// @brief bufferEmpty
int cap3State = 0;
/// @brief dryerEmpty
int cap4State = 0;
/// @brief blowerFull
int cap5State = 0;
/// @brief grainStorageFull
int cap6State = 0;
/// @brief heaterActive
int fotoSensorValue = 0;
/// @brief heaterTemperature
int kTypeTemp = 0;

bool heaterState = false;
bool radialFanState = false;
bool coolingFanState = false;
bool augersTurnOnActive = false;
bool horizontalAugerState = false;
bool floorAugerState = false;
bool airBlowerState = false;

TurnOnState fillingAugersState = TurnOnState::Stopped;
TurnOnState heatingState = TurnOnState::Stopped;
TurnOnState floorAugerState = TurnOnState::Stopped;

unsigned long augerTurnOnTime = 0;
unsigned long horizontalAugerTurnOnTime = 0;
unsigned long radialFanTurnOnTime = 0;

bool start = false;
DryingProcess currentProcessStage = DryingProcess::Nothing;

enum TurnOnState
{
  Cranking,
  Running,
  Stopping,
  Stopped,
};

enum DryingProcess
{
  Nothing,
  Loading,
  Starting,
  Drying,
  Cooling,
  Unloading
};

void ReceiveDataFromSlave();
void SendCommandToSlave();
void ParseMessage(uint8_t *message, int length);
void Start();
void StartDrying();
void TurnOnAuger();
void CheckIfDryerIsFull();
void TurnOnHeater();
void TurnOnCoolingFan();
void Drying();

void setup()
{
  pinMode(augerPin, OUTPUT);
  pinMode(horizontalAugerPin, OUTPUT);
  pinMode(radialFanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(coolingFanPin, OUTPUT);
  pinMode(floorAugerPin, OUTPUT);
  pinMode(airBlowerPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  nexDisplay.begin(9600);
  delay(500);
  nexDisplay.writeStr("page 0");
  lastCommandSent = millis();
}

void loop()
{
  if(start)
    Start();

  if(millis() - COMMAND_INTERVAL_TIME_MS > lastCommandSent) {
    SendCommandToSlave();
  }

  ReceiveDataFromSlave();
}

void Start()
{
  switch (currentProcessStage)
  {
  case DryingProcess::Nothing:
    break;
  case DryingProcess::Loading:
    break;
  case DryingProcess::Starting:
    StartDrying();
    break;
  case DryingProcess::Drying:
    Drying();
    break;
  case DryingProcess::Cooling:
    break;
  case DryingProcess::Unloading:
    break;
  default:
    break;
  }
}

void Drying()
{
  
}

void StartDrying()
{
  if(cap3State == 0)
  {
    TurnOnAuger();
  }

  if(fillingAugersState == TurnOnState::Cranking && (millis() - augerTurnOnTime > SOFT_START_DELAY_MS))
  {
    digitalWrite(horizontalAugerPin, HIGH);
    horizontalAugerState = true;
    horizontalAugerTurnOnTime = millis();
    fillingAugersState = TurnOnState::Running;
  }

  if(cap3State == 1 && (millis() - horizontalAugerTurnOnTime > SOFT_START_DELAY_MS))
  {
    TurnOnHeater();
  }

  if(cap4State == 1)
  {
    TurnOnCoolingFan();
  }

  CheckIfDryerIsFull();
}

void TurnOnCoolingFan()
{
  digitalWrite(coolingFanPin, HIGH);
  coolingFanState = true;
}

void TurnOnHeater()
{
  digitalWrite(radialFanPin, HIGH);
  radialFanState = true;
  digitalWrite(heaterPin, HIGH);
  heaterState = true;
  radialFanTurnOnTime = millis();
  heatingState = TurnOnState::Running;
}

void TurnOnAuger()
{
  digitalWrite(augerPin, HIGH);
  // augersTurnOnActive = true;
  fillingAugersState = TurnOnState::Cranking;
  augerTurnOnTime = millis();
}

void CheckIfDryerIsFull()
{
  if(cap2State == 1)
  {
    digitalWrite(augerPin, LOW);
    // augersTurnOnActive = false;
    digitalWrite(horizontalAugerPin, LOW);
    // horizontalAugerState = false;
    fillingAugersState = TurnOnState::Stopped;
    currentProcessStage = DryingProcess::Drying;
  }
}

void ReceiveDataFromSlave()
{
  if (rs485Serial.available() > 0)
  {
    const int bufferSize = 64; // Adjust buffer size as needed
    uint8_t buffer[bufferSize];
    int length = 0;
    int cache = 0;
    unsigned long timeout = millis() + 1000; // Timeout after 1 second
    // Read data from SoftwareSerial into buffer
    while (rs485Serial.available() && length < bufferSize && millis() < timeout){
      cache = rs485Serial.read();
      buffer[length++] = cache;
      if(cache == 0xFF) {
        break;
      }
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