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
 * File:         bmp180.h
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


#ifndef BMP180_H
#define BMP180_H

#include <Wire.h>


class bmp180
{
   public:
      typedef enum
      {
         precisionLow,
         precisionMedium,
         precisionHigh,
         precisionUltraHigh,
         precisionStandard  // Use the set precision (stored in currentPrecision
      } precisionSetting;

      bmp180(uint8_t i2c_addr = defaultI2C_address);
      bool begin();
      bool readTemperature(double* Temp);
      bool readPressure(double* Pressure, precisionSetting precision = precisionStandard);

   protected:

   private:
      static const uint8_t  defaultI2C_address = 0x77;

      static const uint8_t  reg_out_xlsb    = 0xF8;
      static const uint8_t  reg_out_lsb     = 0xF7;
      static const uint8_t  reg_out_msb     = 0xF6;
      static const uint8_t  reg_ctrl_meas   = 0xF4;  // Bits 0..4: measurement control: See below
                                                     // Bit  5   : Sco, Start of conversion: 1 during conversion, 0 after conversion is complete
                                                     // Bits 6..7: Oss, Oversampling: 00: single, 01: 2 times, 10: 4 times, 11: 8 times
      static const uint8_t  reg_softReset   = 0xE0;  // Write 0xB6 to reset
      static const uint8_t  reg_chipId      = 0xD0;  // Should be 0x55
      static const uint8_t  reg_calibration = 0xAA;  // Beginning of calibration data

      static const uint8_t  cmd_softReset   = 0xB6;  // Write to reg_soft_reset to reset the chip
      static const uint8_t  cmd_readTemp    = 0x2E;  // Write to reg_ctrl_meas for temperature measurement. Takes max 4.5 ms
      static const uint8_t  cmd_readPress0  = 0x34;  // Write to reg_ctrl_meas for pressure measurement with single oversampling  (oss=0). Takes max  4.5 ms
      static const uint8_t  cmd_readPress1  = 0x74;  // Write to reg_ctrl_meas for pressure measurement with 2 times oversampling (oss=1). Takes max  7.5 ms
      static const uint8_t  cmd_readPress2  = 0xB4;  // Write to reg_ctrl_meas for pressure measurement with 4 times oversampling (oss=2). Takes max 13.5 ms
      static const uint8_t  cmd_readPress3  = 0xF4;  // Write to reg_ctrl_meas for pressure measurement with 8 times oversampling (oss=3). Takes max 25.5 ms

      static const uint32_t time_temp       =  4500; // us
      static const uint32_t time_press0     =  4500; // us
      static const uint32_t time_press1     =  7500; // us
      static const uint32_t time_press2     = 13500; // us
      static const uint32_t time_press3     = 25500; // us

      static const uint8_t  calibDataSize   = 0x16;  // 22 bytes calibration data
      static const uint8_t  deviceDataSize  = calibDataSize / 2;  // 11 words
      static const uint8_t  bmp180_chipId   = 0x55;   // This value must be read from reg_chipId
      static const uint32_t waitFraction    =   70;   // Percentage of maximum time to wait before starting to poll, e.g. (time_press3 * waitFraction) / 100 = (25500us * 70) / 100 = 17850us
      static const uint32_t pollingInterval =  120;   // Time in us to wait between polls for conversion done
      static const uint8_t  conversionBusy  = 1 << 5; // Bit 5 of reg_ctrl_meas is set while conversion is busy

      uint8_t  i2c_address;
      bool     temperatureRead;
      double   B5;              // Temporary result from temperature conversion needed for pressure conversion

      precisionSetting currentPrecision;  // Current precision. Can be changed by setPrecision.

      union
      {
         struct
         {
            int16_t  cAC1;
            int16_t  cAC2;
            int16_t  cAC3;
            uint16_t cAC4;
            uint16_t cAC5;
            uint16_t cAC6;
            int16_t  cB1;
            int16_t  cB2;
            int16_t  cMB;
            int16_t  cMC;
            int16_t  cMD;
         } calibrationData;
         uint16_t rawData[deviceDataSize];
      } deviceData;


      bool getCalibrationData();
      void writeByte(uint8_t b);
      bool readBytesFromAddress(uint8_t addr, uint8_t bytes, uint8_t* data);
      void sendCommand(uint8_t command);
      bool waitConversion(uint32_t maxTime);
};

#endif // BMP180_H
