//======================================================================
//  Program: NMEA.ino
//
//  Description:  This program uses the fix-oriented methods available() and
//    read() to handle complete fix structures.
//
//    When the last character of the LAST_SENTENCE_IN_INTERVAL (see NMEAGPS_cfg.h)
//    is decoded, a completed fix structure becomes available and is returned
//    from read().  The new fix is saved the 'fix' structure, and can be used
//    anywhere, at any time.
//
//    If no messages are enabled in NMEAGPS_cfg.h, or
//    no 'gps_fix' members are enabled in GPSfix_cfg.h, no information will be
//    parsed, copied or printed.
//
//  Prerequisites:
//     1) Your GPS device has been correctly powered.
//          Be careful when connecting 3.3V devices.
//     2) Your GPS device is correctly connected to an Arduino serial port.
//          See GPSport.h for the default connections.
//     3) You know the default baud rate of your GPS device.
//          If 9600 does not work, use NMEAdiagnostic.ino to
//          scan for the correct baud rate.
//     4) LAST_SENTENCE_IN_INTERVAL is defined to be the sentence that is
//          sent *last* in each update interval (usually once per second).
//          The default is NMEAGPS::NMEA_RMC (see NMEAGPS_cfg.h).  Other
//          programs may need to use the sentence identified by NMEAorder.ino.
//     5) NMEAGPS_RECOGNIZE_ALL is defined in NMEAGPS_cfg.h
//
//  'Serial' is for debug output to the Serial Monitor window.
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

#include <Arduino.h>
#include <NMEAGPS.h>
#include <pmmConsts.h>
#include <pmmGps.h>

//-------------------------------------------------------------------------
//  The GPSport.h include file tries to choose a default serial port
//  for the GPS device.  If you know which serial port you want to use, edit the GPSport.h file.
#include <GPSport.h>

//------------------------------------------------------------
// For the NeoGPS example programs, "Streamers" is common set of printing and formatting routines for GPS data, in a
//   Comma-Separated Values text format (aka CSV).  The CSV data will be printed to the "debug output device".
// If you don't need these formatters, simply delete this section.
#include <Streamers.h>

/*
 Status,UTC Date/Time,Lat,Lon,Hdg,Spd,Alt,Sats,Rx ok,Rx err,Rx chars,
3,2018-04-25 16:32:02.840,-228587517,-432291933,4830,5200,,,5,0,356,
3,2018-04-25 16:32:03.840,-228588150,-432292417,4930,3900,630,3,12,0,791,
3,2018-04-25 16:32:04.840,-228588617,-432292883,4600,2800,630,3,19,0,1226,
3,2018-04-25 16:32:05.840,-228588867,-432293117,4330,1700,630,3,26,0,1661,
*/

GpsManager::GpsManager(){};

int GpsManager::init()
{
    Serial4.begin(9600);
    return 0;
}

int GpsManager::updateIfAvailable(gps_fix fix)
{
    int hadUpdate = 0;
    while (mGps.available(gpsPort))
    {
        mFix = mGps.read();
        doSomeWork();
        if (!hadUpdate)
            hadUpdate = 0;
    }
    return hadUpdate;
}



//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS quiet time.  It's the best place to do anything that might take
//  a while: print a bunch of things, write to SD, send an SMS, etc.
//  By doing the "hard" work during the quiet time, the CPU can get back to reading the GPS chars as they come in, so that no chars are lost.

void GpsManager::doSomeWork()
{
  // Print all the things!
  trace_all(DEBUG_PORT, mGps, mFix);
} // doSomeWork



//--------------------------
/*
void setup()
{

  DEBUG_PORT.print( F("NMEA.INO: started\n") );
  DEBUG_PORT.print( F("  fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("  gps object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifndef NMEAGPS_RECOGNIZE_ALL
    #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  #endif

  #ifdef NMEAGPS_INTERRUPT_PROCESSING
    #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  #endif

  #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    DEBUG_PORT.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

  #else
    if (gps.merging == NMEAGPS::NO_MERGING) {
      DEBUG_PORT.print  ( F("\nWARNING: displaying data from ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.print  ( F(" sentences ONLY, and only if ") );
      DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      DEBUG_PORT.println( F(" is enabled.\n"
                            "  Other sentences may be parsed, but their data will not be displayed.") );
    }
  #endif

  DEBUG_PORT.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  DEBUG_PORT.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  DEBUG_PORT.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );

  trace_header( DEBUG_PORT );
  DEBUG_PORT.flush();

  gpsPort.begin( 9600 );
}
*/
