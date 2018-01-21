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
{
   i2c_address = i2c_addr;
   temperatureRead = false;  // We first must read the temperature before reading the pressure
}
//----------------------------------------------------------

bool bmp180::begin(void)
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
{
   uint8_t value[2];
   double UT;   // MSB << 8 + LSB
   double X1;   // (UT - AC6) * AC5 / 2^15
   double X2;   // MC * 2^11 / (X1 + MD)


   sendCommand(cmd_readTemp);
   if (!waitConversion(time_temp))   return false;  // Conversion failed

   // Read the temperature
   readBytesFromAddress(reg_out_msb, 2, value);
   UT = (int16_t) (value[0] << 8 | value[1]);

   X1 = (double) (UT - deviceData.calibrationData.cAC6) * deviceData.calibrationData.cAC5 / (1<<15);
   X2 = (double) deviceData.calibrationData.cMC * (1<<11) / (X1 + deviceData.calibrationData.cMD);
   B5 = X1 + X2;  // This result is needed for pressure conversion
   *temperature = (B5 + 8) / (160);  // (B5 + 8) / 2^4 gives temperature in 0.1C

   temperatureRead = true;
   return true;
}

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
   Serial1.print("Delay: "); Serial1.print(leadMillis); Serial1.print("ms and "); Serial1.print(leadMicros); Serial1.println("us"); 
   
   if (leadMillis > 0)   delay(leadMillis);              // Avoid waiting 0 as this give unpredictable results
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

