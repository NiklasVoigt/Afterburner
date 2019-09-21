/*
 * This file is part of the "bluetoothheater" distribution 
 * (https://gitlab.com/mrjones.id.au/bluetoothheater) 
 *
 * Copyright (C) 2019  Ray Jones <ray@mrjones.id.au>
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

#include "128x64OLED.h"
#include "DS18B20Screen.h"
#include "KeyPad.h"
#include "fonts/Icons.h"
#include "fonts/MiniFont.h"
#include "Utility/TempSense.h"

extern CTempSense TempSensor;


CDS18B20Screen::CDS18B20Screen(C128x64_OLED& display, CScreenManager& mgr) : CPasswordScreen(display, mgr) 
{
  _nNumSensors = 0;
  for(int i=0; i<3; i++) {
    _sensorRole[i] = -1;
    _Offset[i] = 0;
  }
  _initUI();
}

void 
CDS18B20Screen::onSelect()
{
  CScreenHeader::onSelect();
  _initUI();
  _nNumSensors = TempSensor.getNumSensors();
  _readNV();
}

void
CDS18B20Screen::_initUI()
{
  _rowSel = 0;
  _colSel = 0;
  _keyHold = -1;
  _scrollChar = 0;
}

bool 
CDS18B20Screen::show()
{
  char msg[32];

  _display.clearDisplay();

  if(!CPasswordScreen::show()) {  // for showing "saving settings"

    if(_rowSel == SaveConfirm) {
      _showConfirmMessage();
    }
    else {
      if(_colSel == 0)
        _showTitle("Temp Sensor Role");
      else
        _showTitle("Temp Sensor Offset");

      int baseLine = 40;
      switch(_nNumSensors) {
        case 1: baseLine = 18; break;
        case 2: baseLine = 30; break;
      }
      for(int i = 0; i<_nNumSensors; i++) {

        switch(_sensorRole[i]) {
          case 0: strcpy(msg, "Pri"); break;
          case 1: strcpy(msg, "Sec"); break;
          case 2: strcpy(msg, "Ter"); break;
          default: strcpy(msg, " ? "); break;
        }
        _printMenuText(border, baseLine-i*12, msg, _rowSel == (i+1) && _colSel == 0);

        OneWireBus_ROMCode romCode;
        if(!TempSensor.getRomCodeIdx(romCode, i)) {
          strcpy(msg, "missing?") ;
        }
        else {
          char* buffer = msg;
          for (int j = 5; j >= 0; j--) {
            sprintf(buffer, "%02X%s", romCode.fields.serial_number[j], j ? ":" : "");
            buffer += 3;
          }
        }
        {
          CTransientFont AF(_display, &miniFontInfo);
          _printMenuText(27, baseLine+2-i*12, msg);
        }

        if(_colSel == 0) {
          float temperature;
          TempSensor.getTemperatureIdx(temperature, i);
          sprintf(msg, "%.01fC", temperature + _Offset[i]);
        }
        else {
          sprintf(msg, "%+.01f", _Offset[i]);
        }
        _printMenuText(90, baseLine-i*12, msg, _rowSel == (i+1) && _colSel == 1);
      }
      
    }
  }
  return true;
}

bool 
CDS18B20Screen::animate()
{
  if(!CPasswordScreen::_busy() && !CPasswordScreen::isPasswordBusy()) {
    if(_rowSel != SaveConfirm) {
      const char* pMsg = NULL;
      switch(_rowSel) {
        case 0:
          _printMenuText(_display.xCentre(), 52, " \021  \030Edit  Exit   \020 ", true, eCentreJustify);
          break;
        case 1:
        case 2:
        case 3:
          if(_colSel == 0)
            pMsg = "                    Hold Right to adjust probe offset.                    "; 
          else
            pMsg = "                    Hold Left to select probe's role.                    "; 
          break;
      }
      if(pMsg != NULL) {
        _display.drawFastHLine(0, 52, 128, WHITE);
        _scrollMessage(56, pMsg, _scrollChar);
      }
      return true;
    }
  }
  return false;
}


bool 
CDS18B20Screen::keyHandler(uint8_t event)
{
  if(CPasswordScreen::keyHandler(event)) {
    if(_isPasswordOK()) {
      _rowSel = 1;
      _keyHold = -1;
    }
  }

  else {
    sUserSettings us;
    if(event & keyPressed) {
      _keyHold = 0;
      // UP press
      if(event & key_Up) {
        if(_rowSel == SaveConfirm) {
          _enableStoringMessage();
          _saveNV();
          NVstore.save();
          TempSensor.mapSensor(-1);   // reset existing mapping
          TempSensor.mapSensor(0, NVstore.getHeaterTuning().tempProbe[0].romCode);
          TempSensor.mapSensor(1, NVstore.getHeaterTuning().tempProbe[1].romCode);
          TempSensor.mapSensor(2, NVstore.getHeaterTuning().tempProbe[2].romCode);
          TempSensor.mapSensor(-2);   // report mapping
          _rowSel = 0;
        }
        else {
          if(_rowSel == 0) {
            _getPassword();
            if(_isPasswordOK()) {
              _rowSel = 1;
            }
          }
          else {
            _testCancel();
            _rowSel++;
            UPPERLIMIT(_rowSel, 3);
          }
        }
      }
      // DOWN press
      if(event & key_Down) {
        _testCancel();
        if(_rowSel == SaveConfirm)
          _rowSel = 0;
        _rowSel--;
        LOWERLIMIT(_rowSel, 0);
      }
    }


    if(event & keyRepeat) {
      if(_keyHold >= 0) {
        _keyHold++;
        if(_keyHold == 2) {
          if(event & key_Left) {
            _colSel = 0;
            _scrollChar = 0;
          }
          if(event & key_Right) {
            _colSel = 1;
            _scrollChar = 0;
          }
          if(event & key_Centre) {
            if(_colSel == 0)
              _sensorRole[_rowSel-1] = -1;
            else
              _Offset[_rowSel-1] = 0;
          }
          _keyHold = -1;
        }
      }
    }


    if(event & keyReleased) {
      if(_keyHold == 0) {
        // LEFT press
        if(event & key_Left) {
          if(_rowSel == 0)
            _ScreenManager.prevMenu();
          else 
            adjust(-1);
        }
        // RIGHT press
        if(event & key_Right) {
          if(_rowSel == 0)
            _ScreenManager.nextMenu();
          else 
            adjust(+1);
        }
        // CENTRE press
        if(event & key_Centre) {
          if(_rowSel == 0) {
            _ScreenManager.selectMenu(CScreenManager::RootMenuLoop);  // force return to main menu
          }
          else  {
            _rowSel = SaveConfirm;
          }
        }
      }
      _keyHold = -1;
    }

    _ScreenManager.reqUpdate();
  }

  return true;
}

void
CDS18B20Screen::adjust(int dir)
{
  switch(_rowSel) {
    case 1:
      if(_colSel == 0) {
        _sensorRole[0] += dir;
        WRAPLIMITS(_sensorRole[0], 0, _nNumSensors-1);
        if(_nNumSensors == 2)
          _sensorRole[1] = _sensorRole[0] == 0 ? 1 : 0;
      }
      else {
        _Offset[0] += dir * 0.1;
        BOUNDSLIMIT(_Offset[0], -10, +10);
      }
      break;
    case 2:
      if(_colSel == 0) {
        _sensorRole[1] += dir;
        WRAPLIMITS(_sensorRole[1], 0, _nNumSensors-1);
        if(_nNumSensors == 2)
          _sensorRole[0] = _sensorRole[1] == 0 ? 1 : 0;
      }
      else {
        _Offset[1] += dir * 0.1;
        BOUNDSLIMIT(_Offset[1], -10, +10);
      }
      break;
    case 3:
      if(_colSel == 0) {
        _sensorRole[2] += dir;
        WRAPLIMITS(_sensorRole[2], 0, _nNumSensors-1);
      }
      else {
        _Offset[2] += dir * 0.1;
        BOUNDSLIMIT(_Offset[2], -10, +10);
      }
      break;
  }
}

void
CDS18B20Screen::_testCancel()
{
  switch(_rowSel) {
    case 1:
      if(_sensorRole[1] == _sensorRole[0])  _sensorRole[1] = -1;
      if(_sensorRole[2] == _sensorRole[0])  _sensorRole[2] = -1;
      break;
    case 2:
      if(_sensorRole[0] == _sensorRole[1])  _sensorRole[0] = -1;
      if(_sensorRole[2] == _sensorRole[1])  _sensorRole[2] = -1;
      break;
    case 3:
      if(_sensorRole[0] == _sensorRole[2])  _sensorRole[0] = -1;
      if(_sensorRole[1] == _sensorRole[2])  _sensorRole[1] = -1;
      break;
  }
}


void
CDS18B20Screen::_readNV() 
{
  _sensorRole[0] = _sensorRole[1] = _sensorRole[2] = -1;

  const sHeaterTuning& tuning = NVstore.getHeaterTuning();
  
  for(int sensor = 0; sensor < TempSensor.getNumSensors(); sensor++) {
    OneWireBus_ROMCode romCode;
    TempSensor.getRomCodeIdx(romCode, sensor); // get rom code of each attached sensor
    // NV storage indices are the sensor role 
    // ie 0 is normal thermostat
    // scan the NV store and match the stored romCodes
    // if not matched, the sensor remains unmapped
    for(int i=0; i< 3; i++) {
      if(memcmp(tuning.tempProbe[i].romCode.bytes, romCode.bytes, 8) == 0) {
         _sensorRole[sensor] = i;  // assign role to sensor according to NV placement
         _Offset[sensor] = tuning.tempProbe[i].offset;
         continue; 
      }
    }
  }
  DebugPort.printf("Sensor roles: %d, %d, %d\r\n", _sensorRole[0], _sensorRole[1], _sensorRole[2]);
}

void
CDS18B20Screen::_saveNV() 
{
  sHeaterTuning tuning = NVstore.getHeaterTuning();
  for(int i=0; i<3; i++) {
    memset(tuning.tempProbe[i].romCode.bytes, 0, 8);
    tuning.tempProbe[i].offset = 0;
  }

  for(int sensor = 0; sensor < TempSensor.getNumSensors(); sensor++) { 
    int role = _sensorRole[sensor];  // role of probe determines placement in NV storage
    if(role != -1) {
      tuning.tempProbe[role].offset = _Offset[sensor];
      OneWireBus_ROMCode romCode;
      TempSensor.getRomCodeIdx(romCode, sensor); // get rom code of indexed sensor
      memcpy(tuning.tempProbe[role].romCode.bytes, romCode.bytes, 8);
    }
  }
  NVstore.setHeaterTuning(tuning);
}