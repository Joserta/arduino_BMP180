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
   temperatureRead = false;         // We first must read the temperature before reading the pressure
   currentPrecision = precisionLow; // Default precision
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

bool bmp180::readTemperature(double* temperature)
/*          ~~~~~~~~~~~~~~~~~
 * Read temperature.
 * This is a blocking function, it will wait till the temperature conversion is complete.
 *
 * Parameters:
 *    temperature[out]: The temperature as read by the sensor in 
 *
 * Return: true if successful, false if something went wrong (in that case temperature will be 0.0)
 */
{
   uint8_t value[2]; // MSB LSB
   double UT;        // MSB << 8 + LSB
   double X1;        // (UT - AC6) * AC5 / 2^15
   double X2;        // MC * 2^11 / (X1 + MD)

   *temperature = 0.0;

   // Start the temperature reading
   sendCommand(cmd_readTemp);

   // Wait for the conversion to finish
   if (!waitConversion(time_temp))   return false;  // Conversion failed

   // Read the temperature
   if (!readBytesFromAddress(reg_out_msb, 2, value))   return false;
   UT = (int16_t) (value[0] << 8 | value[1]);

   // Calculate the pressure
   X1 = (double) (UT - deviceData.calibrationData.cAC6) * deviceData.calibrationData.cAC5 / (1<<15);
   X2 = (double) deviceData.calibrationData.cMC * (1<<11) / (X1 + deviceData.calibrationData.cMD);
   B5 = X1 + X2;  // This result is needed for pressure conversion
   *temperature = (B5 + 8.0) / (160.0);  // (B5 + 8) / 2^4 gives temperature in 0.1C

   temperatureRead = true;
   return true;
}
//----------------------------------------------------------

bool bmp180::readPressure(double* pressure, precisionSetting precision)
/*          ~~~~~~~~~~~~~~
 * Read pressure.
 * This is a blocking function, it will wait till the pressure conversion is complete.
 *
 * Warning! for higher precisions this function can take a very long time (up to 26 ms).
 *
 * Parameters:
 *    pressure [out]: The temperature as read by the sensor.
 *    precision [in]: The required precision, one of bmp180::precisionSetting. Defaults to bmp180::precisionStandard (the set precision)
 *
 * Return: true if successful, false if something went wrong (in that case pressure will be 0.0)
*/
{
   uint8_t  cmd;       // Command to send
   uint8_t  oss;       // Oversampling
   uint32_t maxTime;   // Maximum conversion time
   uint8_t  value[3];  // MSB LSB XLSB
   double   UP;        // (MSB << 16 + LSB << 8 + XLSB) >> (8-oss)
   double   B3, B4, B6, B7;
   double   X1, X2, X3;

   *pressure = 0.0;

   // We must have a temperature reading before reading a pressure
   if (!temperatureRead)
   {
      double temperature;  // No need to keep the temperature, we need the intermediate result B5
      if (!readTemperature(&temperature))   return false;
   }

   // Get the right command and maximum conversion time
   if (precision == precisionStandard)    precision = currentPrecision;
   switch(precision)    // An alternative would be to use a static const array in this file.
   {
      default:
      case precisionLow:
         oss = 0;
         cmd = cmd_readPress0;
         maxTime = time_press0;
         break;
      case precisionMedium:
         oss = 1;
         cmd = cmd_readPress1;
         maxTime = time_press1;
         break;
      case precisionHigh:
         oss = 2;
         cmd = cmd_readPress2;
         maxTime = time_press2;
         break;
      case precisionUltraHigh:
         oss = 3;
         cmd = cmd_readPress3;
         maxTime = time_press3;
         break;
   }

   // Start the pressure reading
   sendCommand(cmd);

   // Wait for the conversion to finish
   if (!waitConversion(maxTime))   return false;  // Conversion failed

   // Read the pressure
   if (!readBytesFromAddress(reg_out_msb, 3, value))   return false;
   UP = (uint32_t) (value[0] << 16 | value[1] << 8 | value[0]) >> (8 - oss);

   // Calculate the pressure
   B6 = B5 - 4000.0;   // B5 was stored during temperature reading
   X1 = (double) deviceData.calibrationData.cB2 * (B6 * B6 / (1<<12));
   X2 = (double) deviceData.calibrationData.cAC2 * B6;
   X3 = (X1 + X2) / (1<<11);
   B3 = (((deviceData.calibrationData.cAC1 * 4.0) + X3) * (1<<oss) + 2) / 4.0;
   X1 = (double) deviceData.calibrationData.cAC3 * B6 / (1<<19);
   X2 = (double) (deviceData.calibrationData.cB1 * (B6 * B6 / (1<<11))) / (2<<16);
   X3 = (X1 + X2 + 2.0) / 4.0;
   B4 = (double) deviceData.calibrationData.cAC4 * (X3 + 23768.0) / (1<<15);
   B7 = (UP - B3) * (50000>>oss);
   *pressure = B7 * 2.0 / B4;
   X1 = *pressure / (1<<8);
   X1 = (X1 * X1 + 3038.0) / (1<<16);
   X2 = (-7357.0 * *pressure);
   *pressure += (X1 +X2 + 3791.0) / (1<<4);

   return true;
}
//----------------------------------------------------------


//*********
// private
//*********

bool bmp180::getCalibrationData()
{
   uint8_t rawData[calibDataSize] = { 0 };
   uint8_t i = 0;

   if (!readBytesFromAddress(reg_calibration, calibDataSize, rawData))   return false;

   // Convert the raw data
   for (uint8_t i = 0; i < deviceDataSize; i++)
   {
      deviceData.rawData[i] = (uint16_t) (rawData[2*i] << 8 | rawData[2*i+1]);
      // Constants cannot be 0 or 0xFFFF
      if ((deviceData.rawData[i] == 0x0000) || (deviceData.rawData[i] == 0xFFFF))  return false;
   }

   // Debug
   #if true
      Serial1.print("AC1 "); Serial1.println(deviceData.calibrationData.cAC1);
      Serial1.print("AC2 "); Serial1.println(deviceData.calibrationData.cAC2);
      Serial1.print("AC3 "); Serial1.println(deviceData.calibrationData.cAC3);
      Serial1.print("AC4 "); Serial1.println(deviceData.calibrationData.cAC4);
      Serial1.print("AC5 "); Serial1.println(deviceData.calibrationData.cAC5);
      Serial1.print("AC6 "); Serial1.println(deviceData.calibrationData.cAC6);
      Serial1.print("B1  "); Serial1.println(deviceData.calibrationData.cB1);
      Serial1.print("B2  "); Serial1.println(deviceData.calibrationData.cB2);
      Serial1.print("MB  "); Serial1.println(deviceData.calibrationData.cMB);
      Serial1.print("MC  "); Serial1.println(deviceData.calibrationData.cMC);
      Serial1.print("MD  "); Serial1.println(deviceData.calibrationData.cMD);
   #endif
   return true;
}
//----------------------------------------------------------

void bmp180::writeByte(uint8_t b)
{
   Wire.beginTransmission(i2c_address);
   Wire.write(b);
   Wire.endTransmission();  
}
//----------------------------------------------------------

bool bmp180::readBytesFromAddress(uint8_t addr, uint8_t bytes, uint8_t* data)
{
   writeByte(addr);  // This is the address we start to read from
   Wire.requestFrom(i2c_address, bytes);
   for (uint8_t i = 0; i < bytes; i++)
   {
      if (!Wire.available())   return false;  // Cannot read all bytes
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

bool bmp180::waitConversion(uint32_t maxTime)
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
   return false;  // If we get here there was a timeout. A logical recovery would be to reset the chip
}
//----------------------------------------------------------

