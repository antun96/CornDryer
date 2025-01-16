// #include <OneWire.h>
// #include <OneWire.h>
// #include <DallasTemperature.h>

// int dsTempPin = 8; // DS18B20 Temperature Sensor
// OneWire oneWire(dsTempPin);
// DallasTemperature sensors(&oneWire);
// uint8_t adresses[2][8] = {
//   {0x28, 0x1E, 0x4D, 0x50, 0x00, 0x00, 0x00, 0x8B},
//   {0x28, 0x73, 0xEA, 0x52, 0x00, 0x00, 0x00, 0x67}
// };

// uint8_t findDevices(int pin);
// void setup1()
// {
//   Serial.begin(9600);
//   Serial.println("//\n// Start oneWireSearch.ino \n//");


//   findDevices(8);
//   Serial.println("\n//\n// End oneWireSearch.ino \n//");
// }

// void loop1()
// {

//   sensors.requestTemperatures(); // Send the command to get temperatures
//   // TODO: read capacitive sensors as fast as possible.
  

//   // Loop through each device, print out temperature data
//   for(int i=0;i<2; i++) {
//     // Search the wire for address
  

//     // Print the data
//     float tempC = sensors.getTempC(adresses[i]);
//     Serial.print("Temp C: ");
//     Serial.print(tempC);
//     Serial.print(" Temp F: ");
//     Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
//     } 	
// }

// uint8_t findDevices(int pin)
// {
//   OneWire ow(pin);

//   uint8_t address[8];
//   uint8_t count = 0;


//   if (ow.search(address))
//   {
//     Serial.print("\nuint8_t pin");
//     Serial.print(pin, DEC);
//     Serial.println("[][8] = {");
//     do {
//       count++;
//       Serial.println("  {");
//       for (uint8_t i = 0; i < 8; i++)
//       {
//         Serial.print("0x");
//         if (address[i] < 0x10) Serial.print("0");
//         Serial.print(address[i], HEX);
//         if (i < 7) Serial.print(", ");
//       }
//       Serial.println("  },");
//     } while (ow.search(address));

//     Serial.println("};");
//     Serial.print("// nr devices found: ");
//     Serial.println(count);
//   }

//   return count;
// }