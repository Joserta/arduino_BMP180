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
 * File:         BMP180.ino
 * Description:  Example using the BMP180 driver
 *
 * This file is part of BMP180
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

#include <SevenSegment.h>
#include "bmp180.h"

#define DEBUG
#define TEST_CODE

#ifdef DEBUG
   // Adapt DebugSerial to the serial device you are using
   #define DebugSerial Serial1
   #define debug_print(a) DebugSerial.print(a);
   #define debug_println(a) DebugSerial.println(a);
#else
   #define debug_print(a) ;
   #define debug_println(a) ;
#endif

static SevenSegment display = SevenSegment();
static bmp180 sensor = bmp180();

void setup()
{
   #ifdef DEBUG
      DebugSerial.begin(19200);
   #endif
   debug_println("");
   debug_println("------------------------");
   debug_println("BMP180 Temperature and Pressure Sensor");
   sensor.begin();
   display.begin();
   sensor.resetSensor();
   sensor.setPrecision(bmp180::precisionMedium);
}

void loop()
{
   double temperature, pressure, altitude;
   int32_t temp, press;

   if (sensor.readTemperature(&temperature))
   {
      debug_print("Temperature: ");
      debug_println(temperature);
   }
   else
      debug_println("Could not read temperature");

   // Display temperature, assume 0.00C <= temperature < 100.00C
   display.printNumber(100 * temperature);
   display.drawDot(1);
   display.writeDisplay();

   if (sensor.readPressure(&pressure))
   {
      debug_print("Pressure d : ");
      debug_println(pressure/100);  // hPa
   }
   else
      debug_println("Could not read pressure");
   delay (1000);

   delay (1000); // Wait a while (but after getting the pressure to decrease the temperature dependency)
   display.clearDisplay();
   display.printNumber(pressure/100);
   display.writeDisplay();

   debug_print("Altitude   : ");
   debug_println(sensor.calculateAltitude(pressure));             // First altitude
   debug_print("AltitudeASL: ");
   debug_println(sensor.calculateAltitude(pressure, pressure));   // Should be 0m
   sensor.setPressureAtSeaLevel(pressure);                        // Assume we are at sea-level
   debug_print("AltitudeSim: ");
   debug_println(sensor.calculateAltitude(pressure));             // Should be again 0m
   sensor.resetPressureAtSeaLevel();                              // Restore standard pressure at sea-level

   // Read the altitude (with default pressure at sea-level)
   if (sensor.readAltitude(&altitude))
   {
      debug_print("Altitude Rd: ");
      debug_println(altitude);                                    // Should be close to the first altitude
   }
   else
      debug_println("Could not read altitude");

   // Read the altitude with current pressure at sea-level set to the previosuly read pressure
   if (sensor.readAltitude(&altitude, pressure))
   {
      debug_print("AltitudeRSL: ");
      debug_println(altitude);                                    // Should be close to 0m
   }
   else
      debug_println("Could not read altitude");

   sensor.resetPressureAtSeaLevel();                              // Restore standard pressure at sea-level

   // Assume we are at 1000m, verification of formulae
   sensor.setAltitude(1000.0, 90605.0);
   debug_print("Pressure  1: ");
   debug_println(sensor.getPressureAtSeaLevel()/100.0);
   // This should be 1000m
   debug_print("Altitude  1: ");
   debug_println(sensor.calculateAltitude(90605.0));               // 1000m

   // Assume we are at 1m, verification of formulae
   sensor.setAltitude(1.0, 101302);
   debug_print("Pressure  2: ");
   debug_println(sensor.getPressureAtSeaLevel()/100.0);
   // This should be 100m
   debug_print("Altitude  2: ");
   debug_println(sensor.calculateAltitude(101302));               // 1m

   // Assume we are at 100m
   sensor.setAltitude(100.0, pressure);
   debug_print("PressureASL: ");
   debug_println(sensor.getPressureAtSeaLevel()/100.0);           // Should be close to 1013.14 hPa

   // This should be 100m
   debug_print("Altitude100: ");
   debug_println(sensor.calculateAltitude(pressure));             // 100m

   // Read the altitude (with the set pressure at sea-level)
   if (sensor.readAltitude(&altitude))
   {
      debug_print("Altitude1xx: ");
      debug_println(altitude);                                    // Should be close to 100m
   }
   else
      debug_println("Could not read altitude");
   

   // Integer functions
   if (sensor.readTemperature(&temp))
   {
      debug_print("Temperature: ");
      debug_println(temp);
   }
   else
      debug_println("Could not read temperature");

   if (sensor.readPressure(&press))
   {
      debug_print("Pressure i : ");
      debug_println(press);
   }
   else
      debug_println("Could not read pressure");

   for (uint8_t i = 0; i < 10; i++)
   {
      delay(500);
      display.toggleColon();
      display.writeColon();
   }
   debug_println("-----");
   debug_println("");
   display.clearDisplay();
}
