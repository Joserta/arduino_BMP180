/**********************************************************************************
 *
 * Copyright (C) 2018 
 *               Joeri Van hoyweghen
 *               Joserta Consulting & Engineering
 *
 *               All Rights Reserved
 *
 *
 * Contact:      Joeri@Joserta.be
 *
 * File:         bmp180.cpp
 * Description:  BMP180 Digital Pressure Sensor Driver
 *
 * This file is part of BMP180
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **********************************************************************************/

#include <Arduino.h>
#include "bmp180.h"

// Defining DEBUG will send debug info to the serial device. Info includes all intermediate results
// #define DEBUG

// Defining TEST_CODE will disable the hardware interface and insert the data from the BMP180 datasheet example to verify results
// #define TEST_CODE

#ifdef DEBUG
   // Adapt DebugSerial to the serial device you are using
   // The baudrate is fixed at 19200 (you can change it in the constructor)
   #define DebugSerial Serial1
   #define debug_print(a)   { DebugSerial.print(__func__); DebugSerial.print(": "); DebugSerial.print(a);   };
   #define debug_println(a) { DebugSerial.print(__func__); DebugSerial.print(": "); DebugSerial.println(a); };
#else
   #define debug_print(a) ;
   #define debug_println(a) ;
#endif


//********
// public
//********

bmp180::bmp180(uint8_t i2c_addr)
/*     ~~~~~~~~
 * Constructor.
 *
 * Parameters:
 *    i2c_addr [in]: i2c address of the sensor, defaults to 0x77.
 */
{
   i2c_address = i2c_addr;
   temperatureRead    = false;         // We first must read the temperature before reading the pressure
   currentPrecision   = precisionLow;  // Default precision
   pressureAtSeaLevel = standardPressureAtSeaLevel;
}
//----------------------------------------------------------

bool bmp180::begin(void)
/*          ~~~~~~~
 * Initialize the bmp180 driver.
 */
{
   uint8_t chipId;
   Wire.begin();

   // Read chip id and compare to known value
   if (!readBytesFromAddress(reg_chipId, 1, &chipId))   return false;
   if (chipId != bmp180_chipId)  return false;

   // Get the calibration data
   if (!getCalibrationData())   return false;
   return true;
}
//----------------------------------------------------------

void bmp180::setPrecision(precisionSetting precision)
/*          ~~~~~~~~~~~~~~
 * Set the precision for pressure readings.
 * Note that higher precisions yield (a lot) higher conversion times.
 *
 * Parameters:
 *    precision [in]: The desired precision (precisionLow...precisionUltraHigh)
 */
{
   currentPrecision = precision;
}
//----------------------------------------------------------

bmp180::precisionSetting bmp180::getPrecision()
/*                              ~~~~~~~~~~~~~~
 * Get the precision for pressure readings.
 *
 * Return: The current precision (precisionLow...precisionUltraHigh)
 */
{
   return currentPrecision;
}
//----------------------------------------------------------

void bmp180::resetSensor()
/*          ~~~~~~~~~~~~~
 * Reset the sensor
 */
{
   Wire.beginTransmission(i2c_address);
   Wire.write(reg_softReset);
   Wire.write(cmd_softReset);
   Wire.endTransmission();
   delay(20);   // Give the sensor some time to recover
}
//----------------------------------------------------------

bool bmp180::readTemperature(double* temperature)
/*          ~~~~~~~~~~~~~~~~~
 * Read temperature.
 * This is a blocking function, it will wait till the temperature conversion is complete.
 *
 * Parameters:
 *    temperature[out]: The temperature as read by the sensor in C
 *
 * Return: true if successful, false if something went wrong (in that case temperature will be 0.0)
 */
{
   uint8_t value[2]; // MSB LSB
   int32_t UT;        // MSB << 8 + LSB
   double  X1;        // (UT - AC6) * AC5 / 2^15
   double  X2;        // MC * 2^11 / (X1 + MD)

   // Read the temperature
   if (!readRawTemperature(&UT))   return false;

   // Calculate the temperature
   X1 = (double) (UT - deviceData.calibrationData.cAC6) * deviceData.calibrationData.cAC5 / (1<<15);
   X2 = (double) deviceData.calibrationData.cMC * (1<<11) / (X1 + deviceData.calibrationData.cMD);
   B5d = X1 + X2;  // This result is needed for pressure conversion
   *temperature = (B5d + 8.0) / (160.0);  // (B5 + 8) / 2^4 gives temperature in 0.1C

   debug_print("UT: "); debug_println(UT);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   debug_print("B5: "); debug_println(B5d);
   debug_print("T : "); debug_println(*temperature);

   temperatureReadDouble = true;
   return true;
}
//----------------------------------------------------------

bool bmp180::readTemperature(int32_t* temperature)
/*          ~~~~~~~~~~~~~~~~~
 * Read temperature.
 * This is a blocking function, it will wait till the temperature conversion is complete.
 *
 * Parameters:
 *    temperature[out]: The temperature as read by the sensor, in 0.1C (so 23.1C will be 231)
 *
 * Return: true if successful, false if something went wrong (in that case temperature will be 0)
 */
{
   int32_t UT;        // MSB << 8 + LSB
   int32_t X1;        // (UT - AC6) * AC5 / 2^15
   int32_t X2;        // MC * 2^11 / (X1 + MD)

   // Read the temperature
   if (!readRawTemperature(&UT))   return false;

   // Calculate the temperature
   X1 = ((UT - deviceData.calibrationData.cAC6) * deviceData.calibrationData.cAC5) >> 15;
   X2 = (deviceData.calibrationData.cMC << 11) / (X1 + deviceData.calibrationData.cMD);   // Note: in the BMP180 spec this is calculated as -2344, while using the same data it is -2343  (using the double function it yields -2343.87)
   B5 = X1 + X2;  // This result is needed for pressure conversion
   *temperature = (B5 + 8) >> 4;  // (B5 + 8) / 2^4 gives temperature in 0.1C

   debug_print("UT: "); debug_println(UT);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   debug_print("B5: "); debug_println(B5);
   debug_print("T : "); debug_println(*temperature);

   temperatureRead = true;
   return true;
}
//----------------------------------------------------------

bool bmp180::readPressure(double* pressure)
/*          ~~~~~~~~~~~~~~
 * Read pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 * The precision set by setPrecsision is used, or the default precision if not set
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    pressure [out]: The pressure as read by the sensor in Pa.
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
*/
{
   return readPressure(pressure, currentPrecision);
}
//-------------------------------------------------

bool bmp180::readPressure(double* pressure, precisionSetting precision)
/*          ~~~~~~~~~~~~~~
 * Read pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    pressure [out]: The pressure as read by the sensor in Pa.
 *    precision [in]: The required precision, one of bmp180::precisionSetting. Defaults to bmp180::precisionStandard (the set precision)
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
*/
{
   uint8_t  oss;       // Oversampling
   int32_t  UP;        // (MSB << 16 + LSB << 8 + XLSB) >> (8-oss)
   double   B3, B4, B6, B7;
   double   X1, X2, X3;

   *pressure = 0.0;

   // We must have a temperature reading before reading a pressure
   if (!temperatureReadDouble)
   {
      double temperature;  // No need to keep the temperature, we need the intermediate result B5
      if (!readTemperature(&temperature))   return false;
   }

   // Read the pressure
   if (!readRawPressure(&UP, &oss, precision))   return false;

   // Calculate the pressure
   B6 = B5d - 4000.0;   // B5 was stored during temperature reading
   X1 = (double) deviceData.calibrationData.cB2 * B6 * B6 / (1<<12);
   X2 = (double) deviceData.calibrationData.cAC2 * B6;
   X3 = (X1 + X2) / (1<<11);
   B3 = (((deviceData.calibrationData.cAC1 * 4.0) + X3) * (1<<oss) + 2) / 4.0;
   debug_print("UP: "); debug_println(UP);
   debug_print("B6: "); debug_println(B6);
   debug_print("X1: "); debug_println(X1);  // Warning: Do not compare to the BMP180 datasheet as the scaling is done in X3
   debug_print("X2: "); debug_println(X2);  // Warning: Do not compare to the BMP180 datasheet as the scaling is done in X3
   debug_print("X3: "); debug_println(X3);
   debug_print("B3: "); debug_println(B3);
   X1 = (double) deviceData.calibrationData.cAC3 * B6 / (1<<13);        // Note: in the BMP180 spec p15 this is incorrectly specified as (AC3 * B6) / 2^19
   X2 = (double) (deviceData.calibrationData.cB1 * B6 * B6) / (1<<28);
   X3 = (X1 + X2 + 2.0) / 4.0;
   B4 = (double) deviceData.calibrationData.cAC4 * (X3 + 32768.0) / (1<<15);
   B7 = (UP - B3) * (50000>>oss);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   debug_print("X3: "); debug_println(X3);
   debug_print("B4: "); debug_println(B4);
   debug_print("B7: "); debug_println(B7);
   *pressure = B7 * 2.0 / B4;
   X1 = *pressure / (1<<8);
   X1 = (X1 * X1 * 3038.0) / (1<<16);
   X2 = (-7357.0 * *pressure) / (1<<16);
   debug_print("p : "); debug_println(*pressure);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   *pressure += (X1 +X2 + 3791.0) / 16.0;
   debug_print("p : "); debug_println(*pressure);

   return true;
}
//----------------------------------------------------------

bool bmp180::readPressure(int32_t* pressure)
/*          ~~~~~~~~~~~~~~
 * Read pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 * The precision set by setPrecsision is used, or the default precision if not set
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    pressure [out]: The pressure as read by the sensor in Pa.
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
 */
{
   return readPressure(pressure, currentPrecision);
}
//----------------------------------------------------------

bool bmp180::readPressure(int32_t* pressure, precisionSetting precision)
/*          ~~~~~~~~~~~~~~
 * Read pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    pressure [out]: The pressure as read by the sensor in Pa.
 *    precision [in]: The required precision, one of bmp180::precisionSetting. Defaults to bmp180::precisionStandard (the set precision)
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
 */
{
   uint8_t  oss;       // Oversampling
   uint8_t  value[3];  // MSB LSB XLSB
   int32_t  UP;        // (MSB << 16 + LSB << 8 + XLSB) >> (8-oss)
   int32_t  B3, B6, X1, X2, X3;
   uint32_t B4, B7;

   *pressure = 0;

   // We must have a temperature reading before reading a pressure
   if (!temperatureRead)
   {
      int32_t temperature;  // No need to keep the temperature, we need the intermediate result B5
      if (!readTemperature(&temperature))   return false;
   }

   // Read the raw pressure
   if (!readRawPressure(&UP, &oss, precision))   return false;

   // Calculate the pressure
   B6 = B5 - 4000;   // B5 was stored during temperature reading
   X1 = (deviceData.calibrationData.cB2 * ((B6 * B6) >> 12)) >> 11;
   X2 = (deviceData.calibrationData.cAC2 * B6) >> 11;
   X3 = X1 + X2;
   B3 = ((((int32_t)deviceData.calibrationData.cAC1 * 4 + X3) << oss) + 2) / 4;
   debug_print("UP: "); debug_println(UP);
   debug_print("B6: "); debug_println(B6);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   debug_print("X3: "); debug_println(X3);
   debug_print("B3: "); debug_println(B3);
   X1 = (deviceData.calibrationData.cAC3 * B6) >> 13;  // Note: in the BMP180 spec p15 this is incorrectly specified as (AC3 * B6) / 2^19
   X2 = (deviceData.calibrationData.cB1 * ((B6 * B6) >> 12)) >> 16;
   X3 = ((X1 + X2) + 2) >> 2;
   B4 = (deviceData.calibrationData.cAC4 * (uint32_t)(X3 + 32768)) >> 15;
   B7 = ((uint32_t)UP - B3) * (50000 >> oss);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   debug_print("X3: "); debug_println(X3);
   debug_print("B4: "); debug_println(B4);
   debug_print("B7: "); debug_println(B7);
   *pressure = (B7 < 0x80000000) ? (B7 * 2) / B4 : (B7 / B4) * 2;
   X1 = (*pressure >> 8) * (*pressure >> 8);
   X1 = (X1 * 3038) >> 16;
   X2 = (-7357 * *pressure) >> 16;
   debug_print("p : "); debug_println(*pressure);
   debug_print("X1: "); debug_println(X1);
   debug_print("X2: "); debug_println(X2);
   *pressure += (X1 +X2 + 3791) >> 4;
   debug_print("p : "); debug_println(*pressure);
   debug_println("");

   return true;
}
//----------------------------------------------------------

void bmp180::setPressureAtSeaLevel(double p0)
/*          ~~~~~~~~~~~~~~~~~~~~~~~
 * Set the pressure at sea-level.
 * This function will set the current set pressure at sea level for all altitude calculations
 *
 * Note that this function does not use the sensor in any way.
 *
 * Parameters:
 *    p [in]: The pressure as sea-level.
 */
{
   pressureAtSeaLevel = p0;
}
//----------------------------------------------------------

double bmp180::getPressureAtSeaLevel()
/*            ~~~~~~~~~~~~~~~~~~~~~~~
 * Get the current pressure at sea-level.
 *
 * Note that this function does not use the sensor in any way.
 *
 */
{
   return pressureAtSeaLevel;
}
//----------------------------------------------------------

void bmp180::resetPressureAtSeaLevel()
/*          ~~~~~~~~~~~~~~~~~~~~~~~~~
 * Reset the current pressure at sea-level.
 *
 * Note that this function does not use the sensor in any way.
 *
 */
{
   pressureAtSeaLevel = standardPressureAtSeaLevel;
}
//----------------------------------------------------------

double bmp180::calculateAltitude(double p)
/*            ~~~~~~~~~~~~~~~~~~~
 * Calculate the altitude from the pressure.
 * This function will use the current set pressure at sea level (or the default).
 *
 * Note that this function does not use the sensor in any way.
 *
 * Parameters:
 *    p [in]: The pressure to convert to an altitude.
 *
 * Return: the altitude in m.
 */
{
   return calculateAltitude(p, pressureAtSeaLevel);
}
//----------------------------------------------------------

double bmp180::calculateAltitude(double p, double p0)
/*            ~~~~~~~~~~~~~~~~~~~
 * Calculate the altitude from the pressure.
 *
 * Note that this function does not use the sensor in any way.
 *
 * Parameters:
 *    p  [in]: The pressure to convert to an altitude.
 *    p0 [in]: The pressure at sea-level.
 *
 * Return: the altitude in m.
 */
{
   return 44330.0 *(1.0 - pow(p/p0, 1.0/5.255));
}
//----------------------------------------------------------

bool bmp180::readAltitude(double *altitude)
/*          ~~~~~~~~~~~~~~
 * Calculate the altitude from the sensor pressure.
 * This function will use the current set pressure at sea level (or the default).
 *
 * Note that this function will first read the pressure (and if needed temperature) from the sensor.
 *
 * Parameters:
 *    altitude [out]: Altitude in m.
 *
 * Return: true if successful, false if something went wrong (in that case altitude will be 0.0)
 */
{
   return readAltitude(altitude, pressureAtSeaLevel);
}

//----------------------------------------
bool bmp180::readAltitude(double *altitude, double p0)
/*          ~~~~~~~~~~~~~~
 * Calculate the altitude from the sensor pressure.
 *
 * Note that this function will first read the pressure (and if needed temperature) from the sensor.
 *
 * Parameters:
 *    altitude [out]: Altitude in m.
 *    p0        [in]: The pressure at sea-level.
 */
{
   double pressure;

   if (!readPressure(&pressure))   return false;
   *altitude = calculateAltitude(pressure, p0);
   return true;
}
//----------------------------------------------------------

void bmp180::setAltitude(double altitude, double pressure)
/*          ~~~~~~~~~~~~~
 * Set the current altitude.
 * This function will set the current pressure at sea level from a known altitude and a (measured) pressure.
 *
 * Note that this function does not use the sensor in any way.
 *
 * Parameters:
 *    altitude [in]: Altitude in m.
 *    pressure [in]: The pressure at altitude in Pa.
 */
{
   pressureAtSeaLevel = pressure / pow(1.0 - altitude/44330.0, 1.0/5.255);
}
//----------------------------------------------------------


//*********
// private
//*********

bool bmp180::getCalibrationData()
/*          ~~~~~~~~~~~~~~~~~~~~
 * Get the calibration data.
 *
 * Return: true if successful, false if something went wrong
 */
{
   uint8_t rawData[calibDataSize] = { 0 };
   uint8_t i = 0;

   if (!readBytesFromAddress(reg_calibration, calibDataSize, rawData))   return false;

   // Convert the raw data
   for (uint8_t i = 0; i < deviceDataSize; i++)
   {
      deviceData.rawData[i] = (uint16_t) (rawData[2*i] << 8 | rawData[2*i+1]);
      // Constants cannot be 0 or 0xFFFF
      if ((deviceData.rawData[i] == 0x0000) || (deviceData.rawData[i] == 0xFFFF))
      {
         debug_println("ERROR: Invalid calibration data");
         return false;
      }
   }

   // Debug
   debug_println("Calibration data:");
   debug_print("AC1 "); debug_println(deviceData.calibrationData.cAC1);
   debug_print("AC2 "); debug_println(deviceData.calibrationData.cAC2);
   debug_print("AC3 "); debug_println(deviceData.calibrationData.cAC3);
   debug_print("AC4 "); debug_println(deviceData.calibrationData.cAC4);
   debug_print("AC5 "); debug_println(deviceData.calibrationData.cAC5);
   debug_print("AC6 "); debug_println(deviceData.calibrationData.cAC6);
   debug_print("B1  "); debug_println(deviceData.calibrationData.cB1);
   debug_print("B2  "); debug_println(deviceData.calibrationData.cB2);
   debug_print("MB  "); debug_println(deviceData.calibrationData.cMB);
   debug_print("MC  "); debug_println(deviceData.calibrationData.cMC);
   debug_print("MD  "); debug_println(deviceData.calibrationData.cMD);
   debug_println("");

   return true;
}
//----------------------------------------------------------

bool bmp180::readRawTemperature(int32_t* rawTemperature)
/*          ~~~~~~~~~~~~~~~~~~~~
 * Read raw temperature.
 * This is a blocking function, it will wait till the temperature conversion is complete.
 *
 * Parameters:
 *    rawRemperature[out]: The temperature as read by the sensor
 *
 * Return: true if successful, false if something went wrong (in that case temperature will be 0)
 */
{
   uint8_t value[2]; // MSB LSB

   *rawTemperature = 0;

   // Start the temperature reading
   sendCommand(cmd_readTemp);

   // Wait for the conversion to finish
   if (!waitConversion(time_temp))   return false;  // Conversion failed

   // Read the temperature
   if (!readBytesFromAddress(reg_out_msb, 2, value))   return false;
   *rawTemperature = (int16_t) (value[0] << 8 | value[1]);

   return true;
}
//----------------------------------------------------------

bool bmp180::readRawPressure(int32_t* rawPressure, uint8_t* oss, precisionSetting precision)
/*          ~~~~~~~~~~~~~~~~~
 * Read raw pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    rawPressure [out]: The raw pressure as read by the sensor.
 *    oss         [out]: Oversampling (0..3).
 *    precision    [in]: The required precision, one of bmp180::precisionSetting. Defaults to bmp180::precisionStandard (the set precision)
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
*/
{
   uint8_t  cmd;       // Command to send
   uint32_t maxTime;   // Maximum conversion time
   uint8_t  value[3];  // MSB LSB XLSB

   *rawPressure = 0;

   // Get the right command and maximum conversion time
   switch(precision)    // An alternative would be to use a static const array in this file.
   {
      default:
      case precisionLow:
         *oss = 0;
         cmd  = cmd_readPress0;
         maxTime = time_press0;
         break;
      case precisionMedium:
         *oss = 1;
         cmd  = cmd_readPress1;
         maxTime = time_press1;
         break;
      case precisionHigh:
         *oss = 2;
         cmd  = cmd_readPress2;
         maxTime = time_press2;
         break;
      case precisionUltraHigh:
         *oss = 3;
         cmd  = cmd_readPress3;
         maxTime = time_press3;
         break;
   }

   // Start the pressure reading
   sendCommand(cmd);

   // Wait for the conversion to finish
   if (!waitConversion(maxTime))   return false;  // Conversion failed

   // Read the pressure
   if (!readBytesFromAddress(reg_out_msb, 3, value))   return false;
   *rawPressure = (uint32_t) (value[0] << 16 | value[1] << 8 | value[0]) >> (8 - *oss);

   return true;
}
//----------------------------------------------------------

bool bmp180::waitConversion(uint32_t maxTime)
/*          ~~~~~~~~~~~~~~~~
 * Wait for the conversion to finish.
 *
 * Parameters:
 *    maxTime [in]: Maximum conversion time in milliseconds.
 *
 * Return: true if conversion is finished, false if something went wrong
*/
{
   uint32_t waitTime    = (maxTime * waitFraction) / 100;
   uint32_t leadMillis  = waitTime / 1000;    // Time to wait before starting to poll  in milliseconds
   uint32_t leadMicros  = waitTime % 1000;    // ... and microseconds
   uint32_t pollingTime = maxTime - waitTime; // Time for polling
   uint32_t startTime;
   uint8_t  ctrl_meas;

   // Start by waiting, as this function is called directly after sending the command
   if (leadMillis > 0)   delay(leadMillis);              // Avoid waiting 0 as this gives unpredictable results
   if (leadMicros)       delayMicroseconds(leadMicros);

   startTime = micros();  // Now we start polling
   // Poll for the conversion to finish
   do
   {
      // Check if the conversion is finished. Takes around 400us
      if (!readBytesFromAddress(reg_ctrl_meas, 1, &ctrl_meas))   return false;
      if ((ctrl_meas & conversionBusy) == 0)   return true;      // Conversion finished OK

      // Wait a while longer
      delayMicroseconds(pollingInterval);
   }  while ((micros() - startTime) <= pollingTime);
 
   // If we get here there was a timeout. A logical recovery would be to reset the chip
   debug_println("ERROR: Timeout waiting for conversion.");
   return false;
}
//----------------------------------------------------------

#ifndef TEST_CODE
void bmp180::writeByte(uint8_t b)
/*          ~~~~~~~~~~~
 * Write a byte to the BMP180.
 *
 * Parameters:
 *    b [in]: Byte to write.
 */
{
   Wire.beginTransmission(i2c_address);
   Wire.write(b);
   Wire.endTransmission();  
}
//----------------------------------------------------------

bool bmp180::readBytesFromAddress(uint8_t addr, uint8_t bytes, uint8_t* data)
/*          ~~~~~~~~~~~~~~~~~~~~~~
 * Read a number of bytes from a register address.
 *
 * Parameters:
 *    addr  [in]: Register address to read from.
 *    bytes [in]: Number of bytes to read.
 *    data [out]: Read bytes.
 *
 * Return: true if the correct amount of bytes has been read, false if something went wrong
 */
{
   writeByte(addr);  // This is the address we start to read from
   Wire.requestFrom(i2c_address, bytes);
   for (uint8_t i = 0; i < bytes; i++)
   {
      if (!Wire.available())
      {
         debug_println("ERROR: Cannot read all bytes.");
         return false;  // Cannot read all bytes
      }
      *data++ = Wire.read();
   }
   return true;
}
//----------------------------------------------------------

void bmp180::sendCommand(uint8_t command)
{
   Wire.beginTransmission(i2c_address);
   Wire.write(reg_ctrl_meas);    // Write to ctrl_meas register ...
   Wire.write(command);          // ... this command
   Wire.endTransmission();   
}
//----------------------------------------------------------


//*******************************************************************
#else //  TEST_CODE   TEST_CODE   TEST_CODE   TEST_CODE   TEST_CODE
// Insert simulated data

void bmp180::writeByte(uint8_t b)
/*          ~~~~~~~~~~~
 * Write a byte to the BMP180.
 *
 * Do nothing
 */
{
}
//----------------------------------------------------------

bool bmp180::readBytesFromAddress(uint8_t addr, uint8_t bytes, uint8_t* data)
/*          ~~~~~~~~~~~~~~~~~~~~~~
 * Read a number of bytes from a register address.
 *
 * This is a test function that will insert simulated data
 *
 * Parameters:
 *    addr  [in]: Register address to read from.
 *    bytes [in]: Number of bytes to read.
 *    data [out]: Read bytes.
 *
 * Return: true if the parameters are acceptable, false if something went wrong (impossible parameters)
 */
{
   switch (addr)
   {
      case reg_calibration:   // Calibration data
         if (bytes < calibDataSize)   return false;
         data[0x00] = 0x01;   data[0x01] = 0x98;   //    408
         data[0x02] = 0xFF;   data[0x03] = 0xB8;   // -   72
         data[0x04] = 0xC7;   data[0x05] = 0xD1;   // -14383
         data[0x06] = 0x7F;   data[0x07] = 0xE5;   //  32741
         data[0x08] = 0x7F;   data[0x09] = 0xF5;   //  32757
         data[0x0A] = 0x5A;   data[0x0B] = 0x71;   //  23153
         data[0x0C] = 0x18;   data[0x0D] = 0x2E;   //   6190
         data[0x0E] = 0x00;   data[0x0F] = 0x04;   //      4
         data[0x10] = 0x80;   data[0x11] = 0x00;   // -32768
         data[0x12] = 0xDD;   data[0x13] = 0xF9;   // - 8711
         data[0x14] = 0x0B;   data[0x15] = 0x34;   //   2868
         break;

      case reg_chipId:        // Chip Id
         data[0] = bmp180_chipId;
         break;

      case reg_ctrl_meas:     // Conversion ready
         data[0] = 0x0A;
         break;

      case reg_out_msb:       // Temperature or pressure
         switch (bytes)
         {
            case 2:  // Temperature
               data[0x00] = 0x6C;
               data[0x01] = 0xFA;
               break;
            case 3:  // Pressure
               data[0x00] = 0x5D;
               data[0x01] = 0x23;
               data[0x02] = 0x00;  // Sadly the example in the datasheet does not use the high resolution bits (XLSB)
               break;
            default:
               return false;
         }
   }

   return true;
}
//----------------------------------------------------------

void bmp180::sendCommand(uint8_t command)
/*          ~~~~~~~~~~~
 * Send a command to the BMP180.
 *
 * Do nothing
 */
{
}
//----------------------------------------------------------
#endif // TEST_CODE
