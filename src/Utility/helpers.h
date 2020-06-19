/*
 * This file is part of the "bluetoothheater" distribution 
 * (https://gitlab.com/mrjones.id.au/bluetoothheater) 
 *
 * Copyright (C) 2019  Ray Jones <ray@mrjones.id.au>
 * Copyright (C) 2018  James Clark
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

#ifndef __BTC_HELPERS_H__
#define __BTC_HELPERS_H__

#include "UtilClasses.h"
#include "DemandManager.h"

class CTempSense;
struct sGPIO;

extern void forceBootInit();

extern CDemandManager::eStartCode requestOn();
extern void  requestOff();
extern void  reqPumpPrime(bool on);

extern float getTemperatureSensor(int source = 0);
extern void  setDateTime(const char* newTime);
extern void  setDate(const char* newTime);
extern void  setTime(const char* newTime);
extern void  interpretJsonCommand(char* pLine);
extern void  resetWebModerator();
extern void  resetFuelGauge();
extern const char* getBlueWireStatStr();
extern bool  hasOEMcontroller();
extern bool  hasOEMLCDcontroller();
extern bool  hasHtrData();
extern int   getBlueWireStat();
extern int   getSmartError();
extern bool  isCyclicStopStartActive();
extern float getVersion(bool betarevision = false);
const char* getVersionStr(bool beta=false);
extern const char* getVersionDate();
extern int   getBoardRevision();
extern void  setupGPIO();
extern bool  setGPIOout(int channel, bool state);
extern bool  getGPIOout(int channel);
extern bool  toggleGPIOout(int channel);
extern void  feedWatchdog();
extern int   isUpdateAvailable(bool test=true);
extern void  checkFOTA();
extern void  setUploadSize(long val);
extern void getGPIOinfo(sGPIO& info);
extern void simulateGPIOin(uint8_t newKey);   
extern float getBatteryVoltage(bool fast);
extern float getGlowVolts();
extern float getGlowCurrent();
extern int getFanSpeed();
extern int sysUptime();
extern void doJSONwatchdog(int topup);
extern void reloadScreens();
extern CTempSense& getTempSensor() ;
extern void reqHeaterCalUpdate();


void setName(const char* name, int type);
void setPassword(const char* name, int type);

extern void ShowOTAScreen(int percent=0, eOTAmodes updateType=eOTAnormal);

extern void updateMQTT();
extern void refreshMQTT();
extern void requestMQTTrestart();

#endif