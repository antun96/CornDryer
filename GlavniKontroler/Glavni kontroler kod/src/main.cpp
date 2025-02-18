#include <Arduino.h>
#include <SoftwareSerial.h>
#include "EasyNextionLibrary.h"
#include <EEPROM.h>

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

int maxGrainTempAddress = 0;
int maxHeaterTempAddress = 1;
int minTempDiffAddress = 2;
int maxTempDiffAddress = 3;
int emptyWholeDryerAddress = 4;
int heaterAutoModeAddress = 5;
int augerAutoModeAddress = 6;

unsigned long lastCommandSent;
unsigned long COMMAND_INTERVAL_TIME_MS = 1000;               // 1 second
unsigned long const SOFT_START_DELAY_MS = 10000;             // 10 seconds
unsigned long const FLOOR_TO_BLOWER_TURN__OFF_DELAY = 30000; // 30 seconds

// TODO: store it in 2 separe bytes in ROM
int const MINIMUM_TEMP_OFFSET = 5;
int const MAXIMUM_TEMP_OFFSET = 5;

int TX_PIN = 2;   // Software Serial TX -> MAX485 DI
int TXEnable = 3; // DE,RE pin for MAX485
int RX_PIN = 4;   // Software Serial RX -> MAX485 RO

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

/// @brief augerInput - if this is 0, auger should be turned off, there is no corn for fill the dryer
int cap1State = 0;
/// @brief buffer full, should stop filling it
int cap2State = 0;
/// @brief buffer lowest point - if this is 0, buffer is empty - should fill it
int cap3State = 0;
/// @brief lowestCapSensorOnDryer - if this is 0, dryer is empty
int cap4State = 0;
/// @brief blower full - if this is 0, blower is ok, it is functioning ok
int cap5State = 0;
/// @brief coolingTankFull - this is indicator that cooling tank is full and cooling fan can be running
int cap6State = 0;
/// @brief grainStorageFull - this sensor is located at the top of grain bin,
int cap7State = 0;
/// @brief heaterActive
int fotoSensorValue = 0;
/// @brief heaterTemperature
int kTypeTemp = 0;

bool heaterState = false;
bool radialFanState = false;
bool coolingFanState = false;
bool augersState = false;
bool horizontalAugerState = false;
bool floorAugerState = false;
bool airBlowerState = false;
bool buzzerState = false;

TurnOnState fillingAugersState = TurnOnState::Stopped;
TurnOnState heatingState = TurnOnState::Stopped;
TurnOnState floorAugerState = TurnOnState::Stopped;

unsigned long augerTurnOnTime = 0;
unsigned long horizontalAugerTurnOnTime = 0;
unsigned long radialFanTurnOnTime = 0;
unsigned long airBlowerTurnOnTime = 0;
unsigned long floorAugerTurnOffTime = 0;

DryingProcess currentProcessStage = DryingProcess::Nothing;
Pages currentPage = Pages::Start;
bool start = false;
bool emptyWholeDryer = false;
bool heaterAutoMode = true;
bool augerAutoMode = true;
bool blowerAutoMode = true;
// TODO: implement this, to whut down blower some time after this is triggered
unsigned long TimeEmptySensorTriggered = 0;
unsigned long BuzzerTurnTime = 0;

int maxGrainTemp = 0;
int maxHeaterTemp = 0;
int minTempDiff = 0;
int maxTempDiff = 0;

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
  Drying,
  Unloading,
  ShutDown,
  Error,
};

enum Pages
{
  Start,
  Drying,
  Settings,
};

void ReceiveDataFromSlave();
void SendCommandToSlave();
void ParseMessage(uint8_t *message, int length);
void Start();
void Dry();
void TurnOnAuger();
void TurnOnHeater();
void TurnOnCoolingFan();
void RemoveCorn();
void StopRemovingCorn();
bool CheckLastTurnOnTime();
void Unload();
void ShutDown();
void SignalError();
void UpdateScreen();

/// @brief Start button is pressed, dryer proccess started
void trigger0();
/// @brief End button is pressed, dryer proccess ended
void trigger1();
/// @brief Settings button is pressed
void trigger2();

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
  lastCommandSent = millis();
  maxGrainTemp = EEPROM.read(maxGrainTempAddress);
  maxHeaterTemp = EEPROM.read(maxHeaterTempAddress);
  minTempDiff = EEPROM.read(minTempDiffAddress);
  maxTempDiff = EEPROM.read(maxTempDiffAddress);

  emptyWholeDryer = EEPROM.read(emptyWholeDryerAddress) ? true : false;
  heaterAutoMode = EEPROM.read(heaterAutoModeAddress) ? true : false;
  augerAutoMode = EEPROM.read(augerAutoModeAddress) ? true : false;
}

void loop()
{
  if (start)
    Start();

  if (millis() - COMMAND_INTERVAL_TIME_MS > lastCommandSent)
  {
    SendCommandToSlave();
  }

  ReceiveDataFromSlave();

  UpdateScreen();
}

void Start()
{
  switch (currentProcessStage)
  {
  case DryingProcess::Nothing:
    break;
  case DryingProcess::Drying:
    Dry();
    break;
  case DryingProcess::Unloading:
    Unload();
    break;
  case DryingProcess::ShutDown:
    ShutDown();
    SignalEndOfProcess();
    break;
  case DryingProcess::Error:
    ShutDown();
    SignalError();
    break;
  default:
    break;
  }
}

void SignalEndOfProcess()
{
  if (!buzzerState && (millis() - BuzzerTurnTime > 30000))
  {
    digitalWrite(buzzerPin, HIGH);
    BuzzerTurnTime = millis();
    buzzerState = true;
  }
  else if (buzzerState && (millis() - BuzzerTurnTime > 5000))
  {
    digitalWrite(buzzerPin, LOW);
    buzzerState = false;
    BuzzerTurnTime = millis();
  }
}

void SignalError()
{
  if (!buzzerState && (millis() - BuzzerTurnTime > 5000))
  {
    digitalWrite(buzzerPin, HIGH);
    BuzzerTurnTime = millis();
    buzzerState = true;
  }
  else if (buzzerState && (millis() - BuzzerTurnTime > 5000))
  {
    digitalWrite(buzzerPin, LOW);
    buzzerState = false;
    BuzzerTurnTime = millis();
  }
}

void ShutDown()
{
  digitalWrite(augerPin, LOW);
  digitalWrite(horizontalAugerPin, LOW);
  digitalWrite(radialFanPin, LOW);
  digitalWrite(heaterPin, LOW);
  digitalWrite(coolingFanPin, LOW);
  digitalWrite(floorAugerPin, LOW);
  digitalWrite(airBlowerPin, LOW);
}

// TODO: add ending check to shutdown
void Unload()
{
  if (cap5State && (ds3Temp + MINIMUM_TEMP_OFFSET > ds4Temp) && CheckLastTurnOnTime())
    TurnOnCoolingFan();

  if (cap5State && (ds3Temp == ds4Temp) && CheckLastTurnOnTime())
  {
    digitalWrite(coolingFanPin, LOW);
    coolingFanState = false;
  }

  if ((ds3Temp + MINIMUM_TEMP_OFFSET) < ds4Temp && ds2Temp >= maxGrainTemp && CheckEmptyWholeDryer())
      RemoveCorn();
  else if ((ds3Temp + MAXIMUM_TEMP_OFFSET) >= ds4Temp || ds2Temp < maxGrainTemp || !CheckEmptyWholeDryer())
    StopRemovingCorn();
}

// add ending check when auger sensor is empty, and full buffer sensor is empty and temperature on upper part is reached max grain temp 
// - cool, empty warm chamber and shutdown
void Dry()
{
  if (!cap1State || cap2State || !augerAutoMode)
    StopLoadingGrain();
  else if (cap1State && !cap3State && augerAutoMode)
    LoadGrain();

  if (cap3State && CheckLastTurnOnTime() && kTypeTemp < maxHeaterTemp && heaterAutoMode)
    TurnOnHeater();

  if (!cap3State || kTypeTemp > maxHeaterTemp || !heaterAutoMode)
    TurnOffHeater();

  if (cap6State && (ds3Temp + MINIMUM_TEMP_OFFSET > ds4Temp) && CheckLastTurnOnTime())
    TurnOnCoolingFan();

  if (cap6State && (ds3Temp == ds4Temp) && CheckLastTurnOnTime())
    TurnOffCoolingFan();

  if (cap5State && !blowerFull)
  {
    blowerFull = true;
    blowerFullTime = millis();
  }

  if (blowerFull && (millis() - blowerFullTime > 30000))
  {
    currentProcessStage = DryingProcess::Error;
  }

  if ((ds3Temp + MINIMUM_TEMP_OFFSET) < ds4Temp && ds2Temp >= maxGrainTemp && CheckEmptyWholeDryer())
    RemoveCorn();
  else if ((ds3Temp + MAXIMUM_TEMP_OFFSET) >= ds4Temp || ds2Temp < maxGrainTemp || !CheckEmptyWholeDryer())
    StopRemovingCorn();
}

void TurnOffCoolingFan()
{
  digitalWrite(coolingFanPin, LOW);
  coolingFanState = false;
}

bool CheckEmptyWholeDryer()
{
  if(cap7State)
    return false;

  if (emptyWholeDryer)
    return true;

  if (!emptyWholeDryer && !cap6State)
  {
    currentProcessStage = DryingProcess::ShutDown;
    return false;
  }
    return false;
}

bool CheckLastTurnOnTime()
{
  if ((millis() - augerTurnOnTime) > SOFT_START_DELAY_MS && (millis() - horizontalAugerTurnOnTime) > SOFT_START_DELAY_MS && (millis() - radialFanTurnOnTime) > SOFT_START_DELAY_MS && (millis() - airBlowerTurnOnTime) > SOFT_START_DELAY_MS)
    return true;
  
  return false;
}

void StopRemovingCorn()
{
  if (floorAugerState)
  {
    digitalWrite(floorAugerPin, LOW);
    floorAugerState = false;
    floorAugerTurnOffTime = millis();
  }
  if (floorAugerState && (millis() - floorAugerTurnOffTime > FLOOR_TO_BLOWER_TURN__OFF_DELAY))
  {
    digitalWrite(airBlowerPin, LOW);
    airBlowerState = false;
  }
}

// TODO: fix this
void RemoveCorn()
  {
    digitalWrite(airBlowerPin, HIGH);
    airBlowerState = true;
    airBlowerTurnOnTime = millis();

  if (airBlowerState && CheckLastTurnOnTime())
  {
    digitalWrite(floorAugerPin, HIGH);
    floorAugerState = true;
  }
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

void TurnOffHeater()
{
  digitalWrite(heaterPin, LOW);
  heaterState = false;
  heatingState = TurnOnState::Cooling;
}

void TurnOnAuger()
{
  digitalWrite(augerPin, HIGH);
  augersState = true;
  fillingAugersState = TurnOnState::Cranking;
  augerTurnOnTime = millis();
}

void LoadGrain()
  {
    TurnOnAuger();
    if (fillingAugersState == TurnOnState::Cranking && (millis() - augerTurnOnTime > SOFT_START_DELAY_MS))
    {
      digitalWrite(horizontalAugerPin, HIGH);
      horizontalAugerState = true;
      horizontalAugerTurnOnTime = millis();
      fillingAugersState = TurnOnState::Running;
  }
}

void StopLoadingGrain()
  {
    digitalWrite(augerPin, LOW);
    augersState = false;
    digitalWrite(horizontalAugerPin, LOW);
    horizontalAugerState = false;
    fillingAugersState = TurnOnState::Stopped;
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
    while (rs485Serial.available() && length < bufferSize && millis() < timeout)
    {
      cache = rs485Serial.read();
      buffer[length++] = cache;
      if (cache == 0xFF)
      {
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
      if (cap4State && value == 0)
        TimeEmptySensorTriggered = millis();
      
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

void UpdateScreen()
{
  switch (currentPage)
  {
    case Pages::Drying:
    UpdateDryingPage();
    break;
    case Pages::Settings:
    UpdateSettingsPage();
    break;
    case Pages::Start:
    default:
      break;
  }
}

void UpdateDryingPage()
{
  nexDisplay.writeStr("inputAuger.picc", cap1State ? "2" : "1");
  nexDisplay.writeStr("augerMotor.picc", augersState ? "2" : "1");
  nexDisplay.writeStr("horizAuger.picc", horizontalAugerState ? "2" : "1");
  nexDisplay.writeStr("bufferFull.picc", cap2State ? "2" : "1");
  nexDisplay.writeStr("bufferEmpty.picc", cap3State ? "2" : "1");
  nexDisplay.writeStr("radialFan.picc", radialFanState ? "2" : "1");
  nexDisplay.writeStr("heater.picc", heaterState ? "2" : "1");
  nexDisplay.writeStr("coolingFan.picc", coolingFanState ? "2" : "1");
  nexDisplay.writeStr("coolChambFull.picc", cap6State ? "2" : "1");
  nexDisplay.writeStr("dryerEmpty.picc", cap4State ? "2" : "1");
  nexDisplay.writeStr("floorAuger.picc", floorAugerState ? "2" : "1");
  // reverse logic
  nexDisplay.writeStr("blowerFull.picc", cap5State ? "1" : "2");
  nexDisplay.writeStr("blowerMotor.picc", airBlowerState ? "2" : "1");
  // reverse logic
  nexDisplay.writeStr("binFull.picc", airBlowerState ? "2" : "1");

  nexDisplay.writeStr("heatUpTemp.txt", String(ds1Temp));
  nexDisplay.writeStr("heatDownTemp.txt", String(ds2Temp));
  nexDisplay.writeStr("coolGrainTemp.txt", String(ds3Temp));
  nexDisplay.writeStr("outsideTemp.txt", String(ds4Temp));  

  if (heaterAutoMode)
    nexDisplay.writeStr("heaterMode.txt", "Auto");
  else
    nexDisplay.writeStr("heaterMode.txt", "Cool");

  if (augerAutoMode)
    nexDisplay.writeStr("augerMode.txt", "Auto");
  else
    nexDisplay.writeStr("augerMode.txt", "Manual");
  
  if (emptyWholeDryer)
    nexDisplay.writeStr("blowerMode.txt", "True");
  else
    nexDisplay.writeStr("blowerMode.txt", "Manual");
}

void UpdateSettingsPage()
{
  nexDisplay.writeStr("maxGrainTemp.txt", String(maxGrainTemp));
  nexDisplay.writeStr("maxHeaterTemp.txt", String(maxHeaterTemp));
  nexDisplay.writeStr("minTempDiff.txt", String(minTempDiff));
  nexDisplay.writeStr("maxTempDiff.txt", String(maxTempDiff));
  if (emptyWholeDryer)
    nexDisplay.writeStr("emptyWholeDryer.txt", "True");
  else
    nexDisplay.writeStr("emptyWholeDryer.txt", "Manual");
}

/// @brief Start button is pressed, dryer proccess started
void trigger0()
{
  currentProcessStage = DryingProcess::Drying;
  start = true;
  currentPage = Pages::Drying;
}

/// @brief End button is pressed, dryer proccess ended
void trigger1()
{
  currentProcessStage = DryingProcess::Unloading;
  start = false;
  currentPage = Pages::Start;
}

/// @brief Settings button is pressed
void trigger2()
{
  currentPage = Pages::Settings;
}

/// @brief Return to drying screen from settings
void trigger3()
{
  currentPage = Pages::Drying;
}

/// @brief Lower max grain temperature
void trigger4()
{
  maxGrainTemp--;
  EEPROM.write(maxGrainTempAddress, maxGrainTemp);
  nexDisplay.writeStr("maxGrainTemp.txt", String(maxGrainTemp));
}

/// @brief Increase max grain temperature
void trigger5()
{
  maxGrainTemp++;
  EEPROM.write(maxGrainTempAddress, maxGrainTemp);
  nexDisplay.writeStr("maxGrainTemp.txt", String(maxGrainTemp));
}

/// @brief Lower max heater temperature
void trigger6()
{
  maxHeaterTemp--;
  EEPROM.write(maxHeaterTempAddress, maxHeaterTemp);
  nexDisplay.writeStr("maxHeaterTemp.txt", String(maxHeaterTemp));
}

/// @brief Increase max heater temperature
void trigger7()
{
  maxHeaterTemp++;
  EEPROM.write(maxHeaterTempAddress, maxHeaterTemp);
  nexDisplay.writeStr("maxHeaterTemp.txt", String(maxHeaterTemp));
}

/// @brief Lower min temperature difference
void trigger8()
{
  minTempDiff--;
  EEPROM.write(minTempDiffAddress, minTempDiff);
  nexDisplay.writeStr("minTempDiff.txt", String(minTempDiff));
}

/// @brief Increase min temperature difference
void trigger9()
{
  minTempDiff++;
  EEPROM.write(minTempDiffAddress, minTempDiff);
  nexDisplay.writeStr("minTempDiff.txt", String(minTempDiff));
}

/// @brief Lower max temperature difference
void trigger10()
{
  maxTempDiff--;
  EEPROM.write(maxTempDiffAddress, maxTempDiff);
  nexDisplay.writeStr("maxTempDiff.txt", String(maxTempDiff));
}

/// @brief Increase max temperature difference
void trigger11()
{
  maxTempDiff++;
  EEPROM.write(maxTempDiffAddress, maxTempDiff);
  nexDisplay.writeStr("maxTempDiff.txt", String(maxTempDiff));
}

/// @brief Change empty whole dryer mode
void trigger12()
{
  emptyWholeDryer = !emptyWholeDryer;
  EEPROM.write(emptyWholeDryerAddress, emptyWholeDryer);
  if (emptyWholeDryer)
    nexDisplay.writeStr("unloadMode.txt", "True");
  else
    nexDisplay.writeStr("unloadMode.txt", "Manual");
}

/// @brief Change auger mode
void trigger13()
{
  augerAutoMode = !augerAutoMode;
  EEPROM.write(augerAutoModeAddress, augerAutoMode);
  if (augerAutoMode)
    nexDisplay.writeStr("autoAugerFill.txt", "Auto");
  else
    nexDisplay.writeStr("autoAugerFill.txt", "Manual");
}

/// @brief Change heater mode
void trigger14()
{
  heaterAutoMode = !heaterAutoMode;
  EEPROM.write(heaterAutoModeAddress, heaterAutoMode);
  if (heaterAutoMode)
    nexDisplay.writeStr("heaterMode.txt", "Auto");
  else
    nexDisplay.writeStr("heaterMode.txt", "Cool");
}