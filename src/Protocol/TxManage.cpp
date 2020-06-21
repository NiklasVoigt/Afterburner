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

#include "TxManage.h"
#include "../Utility/NVStorage.h"
#include "../Utility/helpers.h"
#include "../Utility/DemandManager.h"
#include <FreeRTOS.h>

//#define DEBUG_THERMOSTAT


// CTxManage is used to send a data frame to the blue wire
//
// As the blue wire is bidirectional, we need to only allow our transmit data
// to reach the blue wire when we actually want to send data.
// At all other times we are listening to the blue wire, receiving any async data
//
// This requires external circuitry to toggle the Tx/Rx modes.
// A "Tx Gating" signal is used.
//   when high, transmit data is sent to the blue wire
//   when low, transmit data is blocked (Hi-Z)
//
// Ideally the circuit also prevents feeding back our own Tx data into the Rx pin
// but the main software loop handles this situation by only accepting Rx data when expected.
//
//  Timing diagram
//                                 ____________________
//   Tx Gate  ____________________|                    |___________________________
//            _____________________________________________________________________
//   Tx Data                          |||||||||||||||

unsigned long CTxManage::m_nStartTime = 0;
int CTxManage::m_nTxGatePin = 0;
int altitudeTest = 0;

// Timer callback operates at ISRL
// this means we should avoid any function NOT in IRAM.
// Sadly digitalWrite falls into this category, so use a FreeRTOS queue
// to push the end event handling into a non ISRL task


CTxManage::CTxManage(int TxGatePin, HardwareSerial& serial) : 
  m_BlueWireSerial(serial),
  m_TxFrame(CProtocol::CtrlMode)
{
  m_sysUpdate = 0;
  m_bOnReq = false;
  m_bOffReq = false;
  m_bTxPending = false;
  m_nStartTime = 0;
  m_nTxGatePin = TxGatePin;
  _rawCommand = 0;
  m_HWTimer = NULL;
  _callback = NULL;
  _prime = false;
}

// static function used for the tx gate termination 
// must use IRAM_ATTR being a call back called at ISRL
void IRAM_ATTR 
CTxManage::callbackGateTerminate()
{
  digitalWrite(m_nTxGatePin, LOW);   // cancel Tx Gate
  m_nStartTime = 0;                  // cancel, we are DONE
}


void 
CTxManage::begin()
{
  pinMode(m_nTxGatePin, OUTPUT);
  digitalWrite(m_nTxGatePin, LOW);   // default to receive mode

  // use a hardware timer to terminate the Tx gate shortly after the completion of the 24 byte transmit packet
  m_HWTimer = timerBegin(2, 80, true);
  //set time in uS of Tx gate from when actual tx data bytes are loaded 
  // 240 bits @ 25000bps is 9.6ms, we'll use 9.7ms for a bit of tolerance
  timerAlarmWrite(m_HWTimer, 10000-300, false); 
  timerAttachInterrupt(m_HWTimer, &callbackGateTerminate, true);     
  timerAlarmDisable(m_HWTimer); //disable interrupt for now
  timerSetAutoReload(m_HWTimer, false);
}

void
CTxManage::queueOnRequest(bool set)
{
  m_bOnReq = set;  // allow cancellation via heater response frame decode
  m_bOffReq = false;
}

void 
CTxManage::queueOffRequest(bool set)
{
  m_bOffReq = set;  // allow cancellation via heater response frame decode 
  m_bOnReq = false;
}

void 
CTxManage::queueRawCommand(uint8_t val)
{
  _rawCommand = val;
}

void
CTxManage::queueSysUpdate() 
{
  m_sysUpdate = 10;
}

void 
CTxManage::PrepareFrame(const CProtocol& basisFrame, bool isBTCmaster)
{
  // copy supplied frame, typically this will be the values an OEM controller delivered
  // which means we parrot that data by default.
  // When parroting, we must especially avoid ping ponging "set temperature"!
  // Otherwise we are supplied with the default params for standalone mode, which we 
  // then instil the NV parameters
  m_TxFrame = basisFrame;  

  // ALWAYS install on/off commands if required
#ifndef PROTOCOL_INVESTIGATION    
  m_TxFrame.resetCommand();   // no command upon blue wire initially, unless a request is pending
  if(_rawCommand) {
    m_TxFrame.setRawCommand(_rawCommand);
    _rawCommand = 0;
  }
  else {
    if(m_bOnReq) {
//      m_bOnReq = false;   // requires cancel via queueOnRequest(false)
      m_TxFrame.onCommand();
    }
    if(m_bOffReq) {
//      m_bOffReq = false;   // requires cancel via queueOffRequest(false)
      m_TxFrame.offCommand();
    }
  }
#endif

  // 0x78 prevents the controller showing bum information when we parrot the OEM controller
  // heater is happy either way, the OEM controller has set the max/min stuff already
  if(isBTCmaster) {
    if(m_sysUpdate) {
      m_sysUpdate--;
      m_TxFrame.setActiveMode();   // this allows heater to save the tuning params to EEPROM
    }
    else {
      m_TxFrame.setPassiveMode();    // this prevents the tuning parameters being saved by heater
    }
    m_TxFrame.setFan_Min(NVstore.getHeaterTuning().Fmin);
    m_TxFrame.setFan_Max(NVstore.getHeaterTuning().Fmax);
    m_TxFrame.setPump_Min(NVstore.getHeaterTuning().getPmin());
    m_TxFrame.setPump_Max(NVstore.getHeaterTuning().getPmax());
    m_TxFrame.setTemperature_Min(NVstore.getHeaterTuning().Tmin);      // Minimum settable temperature
    m_TxFrame.setTemperature_Max(NVstore.getHeaterTuning().Tmax);      // Maximum settable temperature

    if(altitudeTest) {
      m_TxFrame.setAltitude(altitudeTest, true);  
    }
    else {
      float altitude;
      if(getTempSensor().getAltitude(altitude)) {  // if a BME280 is fitted
        // use calculated height
        // set 0xeb 0x47 in "unknown bytes" 
        // - 0xeb happens with all pressure quipped units
        // - 0x47 with all other than coffee pod which sends 0x00?
        m_TxFrame.setAltitude(altitude, true);  
      }
      else {
        // default height - yes it is weird, but that's what the simple controllers send!
        // set 0x01 0x2c in "unknown bytes" - all no pressure equipped OEM controlelrs do that
        m_TxFrame.setAltitude(3500, false);  
      }
    }

    m_TxFrame.setPump_Prime(_prime);

    float tActual = getTemperatureSensor();
    int8_t s8Temp = (int8_t)(tActual + 0.5);
    m_TxFrame.setTemperature_Actual(s8Temp);  // use current temp, for now
    m_TxFrame.setHeaterDemand(CDemandManager::getDegC());
    m_TxFrame.setThermostatModeProtocol(1);  // assume using thermostat control for now

    if(!CDemandManager::isThermostat()) {
      m_TxFrame.setThermostatModeProtocol(0);  // not using any form of thermostat control
      m_TxFrame.setHeaterDemand(CDemandManager::getPumpHz());  // set fixed Hz demand instead
      m_TxFrame.setTemperature_Actual(0);      // must force actual to 0 for Hz mode
    } 
    else if(NVstore.getUserSettings().ThermostatMethod) {
      uint8_t ThermoMode = NVstore.getUserSettings().ThermostatMethod;  // get the METHOD of thermostat control
      float Window = NVstore.getUserSettings().ThermostatWindow;
      float tCurrent = getTemperatureSensor();
      float tDesired = float(CDemandManager::getDegC());
      float tDelta = tCurrent - tDesired;
      float fTemp;
#ifdef DEBUG_THERMOSTAT
      if(_callback) {
        char msg[80];
        sprintf(msg, "Window=%.1f tCurrent=%.1f tDesired=%.1f tDelta=%.1f\r\n", Window, tCurrent, tDesired, tDelta); 
        _callback(msg);
      }
#endif
      Window /= 2;
      switch(ThermoMode) {

        case 3:  // GPIO controlled thermostat mode
          if(CDemandManager::isExtThermostatMode()) {
            if(CDemandManager::isExtThermostatOn()) { 
              s8Temp = m_TxFrame.getTemperature_Max();  // input active (contact closure) - max burn
            }
            else {
              s8Temp = m_TxFrame.getTemperature_Min();  // input not active (contact open) - min burn
            }
            m_TxFrame.setHeaterDemand(s8Temp);
            m_TxFrame.setThermostatModeProtocol(0);  // direct heater to use Hz Mode
            m_TxFrame.setTemperature_Actual(0);      // must force actual to 0 for Hz mode
            break;
          }
          // deliberately fall through if not enabled for GPIO control to standard thermostat
          // |
          // V
        case 0:  // conventional heater controlled thermostat mode
          m_TxFrame.setThermostatModeProtocol(1);  // using heater thermostat control
          s8Temp = (int8_t)(tActual + 0.5);
          m_TxFrame.setTemperature_Actual(s8Temp);
#ifdef DEBUG_THERMOSTAT
          if(_callback) {
            char msg[80];
            sprintf(msg, "Conventional thermostat mode: tActual = %d\r\n", u8Temp); 
            _callback(msg);
          }
#endif
          break;

        case 1:  // heater controlled thermostat mode - BUT actual temp is tweaked via a changed window
          m_TxFrame.setThermostatModeProtocol(1);  // using heater thermostat control
          s8Temp = (int8_t)(tActual + 0.5);  // use rounded actual unless within window
          if(fabs(tDelta) < Window) {
            // hold at desired if inside window
            s8Temp = CDemandManager::getDegC();   
          }
          else if(fabs(tDelta) <= 1.0) {
            // force outside if delta is <= 1 but greater than window
            s8Temp = CDemandManager::getDegC() + ((tDelta > 0) ? 1 : -1); 
          }
          m_TxFrame.setTemperature_Actual(s8Temp);  
#ifdef DEBUG_THERMOSTAT
          if(_callback) {
            char msg[80];
            sprintf(msg, "Heater controlled windowed thermostat mode: tActual=%d\r\n", u8Temp); 
            _callback(msg);
          }
#endif
          break;

        case 2:  // BTC controlled thermostat mode
          // map linear deviation within thermostat window to a Hz value,
          // Hz mode however uses the desired temperature field, somewhere between 8 - 35 for min/max
          // so create a desired "temp" according the the current hystersis
          tDelta /= Window;  // convert tDelta to fraction of window (CAUTION - may be > +-1 !)
#ifdef DEBUG_THERMOSTAT
          if(_callback) {
            char msg[80];
            DebugPort.printf("Linear window thermostat mode: Fraction=%f", tDelta);
            _callback(msg);
          }
#endif
          fTemp = (m_TxFrame.getTemperature_Max() + m_TxFrame.getTemperature_Min()) * 0.5;  // midpoint - tDelta = 0 hinges here
          tDelta *= (m_TxFrame.getTemperature_Max() - fTemp);  // linear offset from setpoint
          fTemp -= tDelta;  // lower Hz when over temp, higher Hz when under!
          // bounds limit - recall original tDelta was NOT managed prior!
          LOWERLIMIT(fTemp, m_TxFrame.getTemperature_Min());
          UPPERLIMIT(fTemp, m_TxFrame.getTemperature_Max());
          // apply modifed desired temperature (works in conjunction with thermostatmode = 0!)
          s8Temp = (int8_t)(fTemp + 0.5);
          m_TxFrame.setHeaterDemand(s8Temp);
          m_TxFrame.setThermostatModeProtocol(0);  // direct heater to use Hz Mode
          m_TxFrame.setTemperature_Actual(0);      // must force actual to 0 for Hz mode
#ifdef DEBUG_THERMOSTAT 
          if(_callback) {
            char msg[80];
            sprintf(msg, " tDesired (pseudo Hz demand) = %d\r\n", u8Temp); 
            _callback(msg);
          }
#endif
          break;

        case 4:
          m_TxFrame.setThermostatModeProtocol(0);  // direct heater to use Hz Mode
          m_TxFrame.setTemperature_Actual(0);      // must force actual to 0 for Hz mode
          m_TxFrame.setHeaterDemand(m_TxFrame.getTemperature_Max());  // maximum Hz
          break;
      }
    }

    m_TxFrame.Controller.OperatingVoltage = NVstore.getHeaterTuning().sysVoltage;
    m_TxFrame.Controller.FanSensor = NVstore.getHeaterTuning().fanSensor;
    m_TxFrame.Controller.GlowDrive = NVstore.getHeaterTuning().glowDrive;
  }
  else {
    m_TxFrame.setPassiveMode();    // this prevents the tuning parameters being saved by heater
  }

  // ensure CRC valid
  m_TxFrame.setCRC();
}

void
CTxManage::Start(unsigned long timenow)
{
  m_nStartTime = timenow + m_nStartDelay;   // create a dwell period if an OEM controller is present after it's data exchange
  m_nStartTime |=  1;                      // avoid a black hole if millis() has wrapped to zero
  m_bTxPending = true;
}

// generate a Tx Gate, then send the TxFrame to the Blue wire
// Note the serial data is ISR driven, we supply all 24 bytes to the Tx buffer which is then drained automatically
// the Tx Gate is closed shortly after the last byte is completed.
bool
CTxManage::CheckTx(unsigned long timenow)
{
  if(m_nStartTime && m_bTxPending) {

    long diff = timenow - m_nStartTime;

    // dwell since OEM exchange has expired ?
    if(diff >= 0) {
      // begin front porch of Tx gating pulse
      digitalWrite(m_nTxGatePin, HIGH);
    }
    if(diff >= m_nFrontPorch) {
      // front porch expired, start actual serial transmission
      // Tx gate remains held high
      // it is then brought low by the timer alarm callback, which also cancels m_nStartTime
      m_bTxPending = false;
      m_BlueWireSerial.write(m_TxFrame.Data, 24);  // write native binary values
      timerWrite(m_HWTimer, 0);       //reset tx gate timeout  
      timerAlarmEnable(m_HWTimer);    // timeout will cause cessation of the Tx gate
    }
  }
  return m_nStartTime == 0;   // returns true when done
}

void 
CTxManage::reqPrime(bool on)
{
  _prime = on;
}
