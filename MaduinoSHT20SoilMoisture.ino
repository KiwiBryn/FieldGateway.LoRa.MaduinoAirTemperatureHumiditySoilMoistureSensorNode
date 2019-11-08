/*
    Copyright ® 2019 January devMobile Software, All Rights Reserved
 
    MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE

    Sample application for outdoor weather proof environmental monitoring device.

    DF Robot SHT20 Temperature & Humidity sensor 
      https://www.dfrobot.com/wiki/index.php/SHT20_I2C_Temperature_%26_Humidity_Sensor_(Waterproof_Probe)_SKU:_SEN0227
    Pino tech SoilWatch 10 soil moisture sensor 
      https://pino-tech.eu/product/soilwatch-10/
    Makerfabs Maduino 
      https://makerfabs.com/index.php?route=product/product&product_id=438 with customisation
    5V 0.5W Solar panel 
      https://www.elecrow.com/05w-solar-panel-with-wires-p-816.html
    500-1200mAh LiPo battery
    Polycarbonate enclosure approx 3.5" x 4.5"
    2 x Cable glands
    3M command adhesive strips to hold battery in place
    4 x stick-on PCB pillars to hold Maduino in place
*/
#define ATSHA204
#define BATTERY_VOLTAGE_MONITOR 

#include <DFRobot_SHT20.h>
#include <LoRa.h>
#include <LowPower.h>
#ifdef ATSHA204
  #include <sha204_library.h>
#endif

#define UNITS_HUMIDITY "%"
#define UNITS_TEMPERATURE "°c"
#define UNITS_SOIL_MOISTURE "%"

const unsigned long SensorUploadDelay = 300;

// LoRa field gateway configuration (these settings must match your field gateway)
const byte FieldGatewayAddress[] = {"LoRaIoT1"};
const byte FieldGatewayAddressLength = strlen( FieldGatewayAddress ) ;
const float FieldGatewayFrequency =  915000000.0;
const byte FieldGatewaySyncWord = 0x12 ;

// optional ATSHA204 secure authentication chip, validation with crypto and hashing (currently only using for unique serial number)
#ifdef ATSHA204
  const byte Atsha204Port = A0;
  atsha204Class sha204(Atsha204Port);
  const byte DeviceAddressLength = 9 ;
  byte DeviceAddress[DeviceAddressLength] = {""};
#else
  byte DeviceAddress[] = {"Maduino4"};
  const byte DeviceAddressLength = strlen( DeviceAddress );
#endif

// LoRa radio hardware configuration
//#define DEBUG_LORA_RADIO
const int LoRaChipSelectPin = 10;
const int LoRaResetPin = 9;

// LoRa radio payload configuration
//#define DEBUG_TELEMETRY
const byte PayloadSizeMaximum = 64;
const byte SensorIdValueSeperator = ' ';
const byte SensorReadingSeperator = ',';
byte payload[PayloadSizeMaximum];
byte payloadLength = 0 ;

// optional battery voltage monitoring 200K/100K voltage divider
#ifdef BATTERY_VOLTAGE_MONITOR
  //#define DEBUG_BATTERY_VOLTAGE_MONITOR
  const byte BatteryVoltageSensorPin = A6;
  const float BatteryVoltageDividerMultipler = 3.0;
  const float BatteryVoltageDividerReferenceVoltage = 3.3;
  const float BatteryVoltageADCValueMax = 1023.0 ;
#endif

// Soil moisture
//#define DEBUG_SOIL_MOISTURE
const int SoilMoistureSensorPin = A3; 
const int SoilMoistureSensorEnablePin = 4;
const int SoilMoistureSensorEnableDelay = 100;
const int SoilMoistureSensorMinimum = 0;     // replace with min ADC value when probe in air
const int SoilMoistureSensorMaximum = 880;   // replace with max ADC value when probe fully submerged in water
const int SoilMoistureValueMinimum = 0;
const int SoilMoistureValueMaximum = 100;

// Air temperature and humidity sensor
//#define DEBUG_TEMPERATURE_AND_HUMIDITY
DFRobot_SHT20 sht20;


void setup()
{
  Serial.begin(9600);
  while (!Serial);

#ifdef ATSHA204
  // Retrieve the serial number then display it nicely
  if(sha204.getSerialNumber(DeviceAddress))
  {
    Serial.println("sha204.getSerialNumber failed");
    while (true); // Drop into endless loop requiring restart
  }
#endif

  Serial.print("Device address:");
  DisplayHex( DeviceAddress, DeviceAddressLength);
  Serial.println();

  Serial.print("Field gateway:");
  DisplayHex( FieldGatewayAddress, FieldGatewayAddressLength);
  Serial.println();
  Serial.print("Frequency:");
  Serial.print((FieldGatewayFrequency/1000000.0),3 ) ;
  Serial.print("MHz SyncWord:0x");
  Serial.print( FieldGatewaySyncWord, HEX ) ;
  Serial.println();

  Serial.println("LoRa setup start");

  // override the default chip select and reset pins
  LoRa.setPins(LoRaChipSelectPin, LoRaResetPin);
  if (!LoRa.begin(FieldGatewayFrequency))
  {
    Serial.println("LoRa begin failed");
    while (true); // Drop into endless loop requiring restart
  }

  // Need to do this so field gateways pays attention to messsages from this device
  LoRa.enableCrc();
  LoRa.setSyncWord(FieldGatewaySyncWord);

#ifdef DEBUG_LORA_RADIO
  LoRa.dumpRegisters(Serial);
#endif
  Serial.println("LoRa Setup done.");

  PayloadHeader(FieldGatewayAddress,FieldGatewayAddressLength, DeviceAddress, DeviceAddressLength);

  // Setup the SHT20 temperature & humdity sensor
  sht20.initSHT20();
  delay(100);
  sht20.checkSHT20();    

  // Setup the Analog input for moisture measurement and digital output for sensor power control
  pinMode(SoilMoistureSensorEnablePin, OUTPUT);
  digitalWrite(SoilMoistureSensorEnablePin, LOW);

  Serial.println("Setup done");
}


void loop()
{
  PayloadReset();  

  float humidity = sht20.readHumidity();          
  PayloadAdd( "h", humidity, 0, false);

  float temperature = sht20.readTemperature();               
  PayloadAdd( "t", temperature, 1, false);
  
#ifdef DEBUG_TEMPERATURE_AND_HUMIDITY  
  Serial.print("H:");
  Serial.print( humidity, 0 ) ;
  Serial.print( UNITS_HUMIDITY ) ;
  Serial.print("T:");
  Serial.print( temperature, 1 ) ;
  Serial.print( UNITS_TEMPERATURE ) ;
#endif

  // Turn on soil mosture sensor, take reading then turn off to save power
  digitalWrite(SoilMoistureSensorEnablePin, HIGH);
  delay(SoilMoistureSensorEnableDelay);
  int soilMoistureADCValue = analogRead(SoilMoistureSensorPin);
  digitalWrite(SoilMoistureSensorEnablePin, LOW);
  int soilMoisture = map(soilMoistureADCValue,SoilMoistureSensorMinimum,SoilMoistureSensorMaximum, SoilMoistureValueMinimum, SoilMoistureValueMaximum); 
  PayloadAdd( "s", soilMoisture, false);
  
#ifdef DEBUG_SOIL_MOISTURE
  Serial.print("S ADC:" );
  Serial.print(soilMoistureADCValue);
  Serial.print("S:" );
  Serial.print(soilMoisture);
  Serial.print( UNITS_SOIL_MOISTURE ) ;
#endif

#ifdef BATTERY_VOLTAGE_MONITOR
  int batteryVoltageADCValue = analogRead(BatteryVoltageSensorPin);
  float batteryVoltage = ( batteryVoltageADCValue / BatteryVoltageADCValueMax) * BatteryVoltageDividerReferenceVoltage * BatteryVoltageDividerMultipler ;
  PayloadAdd( "v", batteryVoltage, 2, true);
  
  #ifdef DEBUG_BATTERY_VOLTAGE_MONITOR
    Serial.print(" V ADC:");
    Serial.print( batteryVoltageADCValue ) ;
    Serial.print(" V:");
    Serial.print( batteryVoltage, 2 ) ;
    Serial.print("v ");
  #endif
#endif
  
#ifdef DEBUG_TEMPERATURE_AND_HUMIDITY || DEBUG_SOIL_MOISTURE || DEBUG_BATTERY_VOLTAGE_MONITOR
  Serial.println();
#endif

#ifdef DEBUG_TELEMETRY
  Serial.print( "RFM9X/SX127X Payload length:");
  Serial.print( payloadLength );
  Serial.println( " bytes" );
#endif

  LoRa.idle();

  LoRa.beginPacket();
  LoRa.write( payload, payloadLength );
  LoRa.endPacket();

  LoRa.sleep();
  
  // Adjust the delay so period is close to desired sec as possible, first do 8sec chunks
  int delayCounter = SensorUploadDelay / 8 ;
  for( int i = 0 ; i < delayCounter ; i++ )
  {
     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  }
  
  // Then to 4 sec chunk
  delayCounter =  ( SensorUploadDelay % 8 ) / 4;
  for( int i = 0 ; i < delayCounter ; i++ )
  {
     LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);  
  }

  // Then to 2 sec chunk
  delayCounter =  ( SensorUploadDelay % 4 ) / 2 ;
  for( int i = 0 ; i < delayCounter ; i++ )
  {
     LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);  
  }

  // Then to 1 sec chunk
  delayCounter =  ( SensorUploadDelay % 2 ) ;
  for( int i = 0 ; i < delayCounter ; i++ )
  {
     LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
  }
}


void PayloadHeader( const byte *to, byte toAddressLength, const byte *from, byte fromAddressLength)
{
  byte addressesLength = toAddressLength + fromAddressLength ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadHeader- ");
  Serial.print( "To Address len:");
  Serial.print( toAddressLength );
  Serial.print( " From Address len:");
  Serial.print( fromAddressLength );
  Serial.print( " Addresses length:");
  Serial.print( addressesLength );
  Serial.println( );
#endif

  payloadLength = 0 ;

  // prepare the payload header with "To" Address length (top nibble) and "From" address length (bottom nibble)
  payload[payloadLength] = (toAddressLength << 4) | fromAddressLength ;
  payloadLength += 1;

  // Copy the "To" address into payload
  memcpy(&payload[payloadLength], to, toAddressLength);
  payloadLength += toAddressLength ;

  // Copy the "From" into payload
  memcpy(&payload[payloadLength], from, fromAddressLength);
  payloadLength += fromAddressLength ;
}


void PayloadAdd( const char *sensorId, float value, byte decimalPlaces, bool lastOne)
{
  byte sensorIdLength = strlen(sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-float ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value, decimalPlaces );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( dtostrf(value, -1, decimalPlaces, (char *)&payload[payloadLength]));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( const char *sensorId, int value, bool lastOne )
{
  byte sensorIdLength = strlen( sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-int ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif  

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( itoa( value,(char *)&payload[payloadLength],10));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }  
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadAdd( const char *sensorId, unsigned int value, bool lastOne )
{
  byte sensorIdLength = strlen( sensorId ) ;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadAdd-unsigned int ");
  Serial.print( "SensorId:");
  Serial.print( sensorId );
  Serial.print( " sensorIdLen:");
  Serial.print( sensorIdLength );
  Serial.print( " Value:");
  Serial.print( value );
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
#endif  

  memcpy( &payload[payloadLength], sensorId,  sensorIdLength) ;
  payloadLength += sensorIdLength ;
  payload[ payloadLength] = SensorIdValueSeperator;
  payloadLength += 1 ;
  payloadLength += strlen( utoa( value,(char *)&payload[payloadLength],10));
  if ( !lastOne)
  {
    payload[ payloadLength] = SensorReadingSeperator;
    payloadLength += 1 ;
  }
  
#ifdef DEBUG_TELEMETRY
  Serial.print( " payloadLength:");
  Serial.print( payloadLength);
  Serial.println( );
#endif
}


void PayloadReset()
{
  byte fromAddressLength = payload[0] & 0xf ;
  byte toAddressLength = payload[0] >> 4 ;
  byte addressesLength = toAddressLength + fromAddressLength ;

  payloadLength = addressesLength + 1;

#ifdef DEBUG_TELEMETRY
  Serial.println("PayloadReset");
  Serial.print( "To Address len:");
  Serial.print( toAddressLength );
  Serial.print( " From Address len:");
  Serial.print( fromAddressLength );
  Serial.print( " Addresses length:");
  Serial.print( addressesLength );
  Serial.println( );
#endif
}


void DisplayHex( const byte *byteArray, byte length) 
{
  for (int i = 0; i < length ; i++)
  {
    // Add a leading zero
    if ( byteArray[i] < 16)
    {
      Serial.print("0");
    }
    Serial.print(byteArray[i], HEX);
    if ( i < (length-1)) // Don't put a - after last digit
    {
      Serial.print("-");
    }
  }
}
