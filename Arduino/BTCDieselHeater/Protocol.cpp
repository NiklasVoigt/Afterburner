/*
 * This file is part of the "bluetoothheater" distribution 
 * (https://gitlab.com/mrjones.id.au/bluetoothheater) 
 *
 * Copyright (C) 2018  Ray Jones <ray@mrjones.id.au>
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include <Arduino.h>
#include "Protocol.h"
#include "debugport.h"
#include "helpers.h"

#ifdef TELNET
#define DebugPort Debug
#endif

#ifndef TELNET
#define DebugPort Serial
#endif

unsigned short 
CProtocol::CalcCRC(int len) const
{
  // calculate a CRC-16/MODBUS checksum using the first 22 bytes of the data array
  unsigned short  wCRCWord = 0xFFFF;

  int wLength = len;
  const unsigned char* pData = Data;
   while (wLength--)
   {
      unsigned char nTemp = *pData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }

   return wCRCWord;
}

void 
CProtocol::setCRC()
{
  setCRC(CalcCRC(22));
}

void 
CProtocol::setCRC(unsigned short CRC)
{
  Data[22] = (CRC >> 8) & 0xff;   // MSB of CRC in Data[22]
  Data[23] = (CRC >> 0) & 0xff;   // LSB of CRC in Data[23]
}


unsigned short
CProtocol::getCRC() const
{
  unsigned short CRC;
  CRC = Data[22];   // MSB of CRC in Data[22]
  CRC <<= 8;
  CRC |= Data[23];  // LSB of CRC in Data[23]
  return CRC;
}

// return true for CRC match
bool
CProtocol::verifyCRC(bool bSilent) const
{
  unsigned short CRC = CalcCRC(22);  // calculate CRC based on first 22 bytes
  unsigned short FrameCRC = getCRC();
  bool bOK = (FrameCRC == CRC);
  if(!bOK && !bSilent) {
    DebugPort.print("verifyCRC FAILED: calc:");
    DebugPort.print(CRC, HEX);
    DebugPort.print(" data:");
    DebugPort.println(FrameCRC, HEX);
  }
  return bOK;        // does it match the stored values?
}

CProtocol& 
CProtocol::operator=(const CProtocol& rhs)
{
  memcpy(Data, rhs.Data, 24);
  return *this;
}


void 
CProtocol::setFan_Min(short Speed) 
{
  // Minimum speed set
  Controller.MinFanRPM_MSB = (Speed >> 8) & 0xff;
  Controller.MinFanRPM_LSB = (Speed >> 0) & 0xff;
}

void 
CProtocol::setFan_Max(short Speed) 
{
  // Minimum speed set
  Controller.MaxFanRPM_MSB = (Speed >> 8) & 0xff;
  Controller.MaxFanRPM_LSB = (Speed >> 0) & 0xff;
}

short 
CProtocol::getFan_Min() const
{
  short retval;
  // Minimum speed get
  retval = Controller.MinFanRPM_MSB;
  retval <<= 8;
  retval |= Controller.MinFanRPM_LSB;
  return retval;
}

short 
CProtocol::getFan_Max() const
{
  short retval;
  // Maximum speed get
  retval = Controller.MaxFanRPM_MSB;
  retval <<= 8;
  retval |= Controller.MaxFanRPM_LSB;
  return retval;
}

short
CProtocol::getFan_Actual() const
{  
  // Rx side, actual
  short retval;
  retval = Heater.FanRPM_MSB;
  retval <<= 8;
  retval |= Heater.FanRPM_LSB;
  return retval;
}

void 
CProtocol::setFan_Actual(short Speed)  // Heater side, actual
{
  // actual speed set
  Heater.FanRPM_MSB = (Speed >> 8) & 0xff;
  Heater.FanRPM_LSB = (Speed >> 0) & 0xff;
}

float 
CProtocol::getGlowPlug_Current() const   // glow plug current
{
  short val;
  val = Heater.GlowPlugCurrent_MSB;
  val <<= 8;
  val |= Heater.GlowPlugCurrent_LSB;
  return float(val) * 0.01f;             // 10mA / digit
}

void 
CProtocol::setGlowPlug_Current(short ampsx100)    // glow plug current
{
  Heater.GlowPlugCurrent_MSB = (ampsx100 >> 8) & 0xff;
  Heater.GlowPlugCurrent_LSB = (ampsx100 >> 0) & 0xff;
}

float 
CProtocol::getGlowPlug_Voltage() const   // glow plug voltage
{
  short val;
  val = Heater.GlowPlugVoltage_MSB;
  val <<= 8;
  val |= Heater.GlowPlugVoltage_LSB;
  return float(val) * 0.1f;   // 0.1V / digit
}


void 
CProtocol::setGlowPlug_Voltage(short voltsx10)    // glow plug voltage
{
  Heater.GlowPlugVoltage_MSB = (voltsx10 >> 8) & 0xff;
  Heater.GlowPlugVoltage_LSB = (voltsx10 >> 0) & 0xff;
}

short 
CProtocol::getTemperature_HeatExchg() const // temperature of heat exchanger
{
  short retval;
  retval = Heater.HeatExchgTemp_MSB;
  retval <<= 8;
  retval |= Heater.HeatExchgTemp_LSB;
  return retval;
}

void
CProtocol::setTemperature_HeatExchg(short degC) // temperature of heat exchanger
{
  Heater.HeatExchgTemp_MSB = (degC >> 8) & 0xff;
  Heater.HeatExchgTemp_LSB = (degC >> 0) & 0xff;
}

short 
CProtocol::getFan_Voltage() const    // temperature near inlet
{
  short retval;
  retval = Heater.FanVoltage_MSB;
  retval <<= 8;
  retval |= Heater.FanVoltage_LSB;
  return retval;
}

void
CProtocol::setFan_Voltage(short voltsx10)     // temperature near inlet
{
  Heater.FanVoltage_MSB = (voltsx10 >> 8) & 0xff;
  Heater.FanVoltage_LSB = (voltsx10 >> 0) & 0xff;
}

void
CProtocol::setVoltage_Supply(short voltsx10)
{
  Heater.SupplyV_MSB = (voltsx10 >> 8) & 0xff;
  Heater.SupplyV_LSB = (voltsx10 >> 0) & 0xff;
}

float
CProtocol::getVoltage_Supply() const
{
  short val = 0;
  val = Heater.SupplyV_MSB & 0xff;
  val <<= 8;
  val |= Heater.SupplyV_LSB & 0xff;
  float voltage = float(val) * 0.1f;
  return voltage;
}

void 
CProtocol::Init(int FrameMode)
{
  if(FrameMode == CtrlMode) { 
    Controller.Byte0 = 0x76;
    Controller.Len = 22;
    Controller.Command = 0;            // NOP
    setTemperature_Actual(18);  // 1degC / digit
    setTemperature_Desired(20); // 1degC / digit
    setPump_Min(1.4f);          // Hz
    setPump_Max(4.3f);          // Hz
    setFan_Min(1450);           // 1RPM / digit
    setFan_Max(4500);           // 1RPM / digit
    Controller.OperatingVoltage = 120;  // 0.1V/digit
    Controller.FanSensor = 1;           // SN-1 or SN-2
    Controller.OperatingMode = 0x32;    // 0x32:Thermostat, 0xCD:Fixed
    setTemperature_Min(8);      // Minimum settable temperature
    setTemperature_Max(35);     // Maximum settable temperature
    Controller.MinTempRise = 5;         // temp rise to sense fuel ignition - GLOW PLUG POWER?
    Controller.Prime = 0;               // 00: normal, 0x5A: fuel prime
    Controller.Unknown1_MSB = 0x01;     // always 0x01
    Controller.Unknown1_LSB = 0x2c;     // always 0x2c  16bit: "300 secs = max run without burn detected" ??
    Controller.Unknown2_MSB = 0x0d;     // always 0x0d
    Controller.Unknown2_LSB = 0xac;     // always 0xac  16bit: "3500" ??  Ignition fan max RPM????
    setCRC();
  }
  else if(FrameMode == HeatMode){
    Heater.Byte0 = 0x76;
    Heater.Len = 22;
    setRunState(0);
    setErrState(0);
    setVoltage_Supply(133);
    setFan_Actual(0);
    setFan_Voltage(0);
    setTemperature_HeatExchg(18);
    setGlowPlug_Voltage(0);
    setGlowPlug_Current(0);
    Heater.ActualPumpFreq = 0;     // fuel pump freq.: 0.1Hz / digit
    Heater.StoredErrorCode = 0;    // 
    Heater.Unknown1 = 0;           // always 0x00
    Heater.FixedPumpFreq = 23;     // fixed mode frequency set point: 0.1Hz / digit
    Heater.Unknown2 = 100;         // always 0x64  "100 ?"
    Heater.Unknown3 = 0;           // always 0x00  
    setCRC();    
  }
  else {
    memset(Data, 0, 24);
  }
}

void
CProtocol::DebugReport(const char* hdr, const char* ftr)
{
  DebugPort.print(hdr);                     // header
  for(int i=0; i<24; i++) {
    char str[16];
    sprintf(str, " %02X", Data[i]);  // build 2 dig hex values
    DebugPort.print(str);                   // and print     
  }
  DebugPort.print(ftr);                     // footer
}

void 
CProtocol::setThermostatMode(unsigned on) 
{ 
  Controller.OperatingMode = on ? 0x32 : 0xCD; 
  if(!on)
    setTemperature_Actual(0);   // if using fixed mode, actual must be reported as 0
};

const char* Runstates [] PROGMEM = {
  " Stopped/Ready ",
  "Starting...",
  "Igniting...",
  " Ignition retry ",
  "Ignited",
  "Running",
  "Stopping",
  "Shutting down",
  "Cooling",
  "Unknown run state"
};

 

const char* 
CProtocolPackage::getRunStateStr() const 
{ 
  uint8_t runstate = Heater.getRunState();
  UPPERLIMIT(runstate, 9);
  return Runstates[runstate]; 
}


const char* Errstates [] PROGMEM = {
  "",
  "",
  "Low voltage",     // E-01
  "High voltage",    // E-02
  "Glow plug fault", // E-03
  "Pump fault",      // E-04
  "Overheat",        // E-05
  "Motor fault",     // E-06
  "Comms fault",     // E-07
  "Flame out",       // E-08
  "Temp sense",      // E-09
  "Ignition fail",   // E-10
  "Unknown error?"   // mystery code!
};

const char* 
CProtocolPackage::getErrStateStr() const 
{ 
  uint8_t errstate = Heater.getErrState();
  UPPERLIMIT(errstate, 12);
  return Errstates[errstate]; 
}
