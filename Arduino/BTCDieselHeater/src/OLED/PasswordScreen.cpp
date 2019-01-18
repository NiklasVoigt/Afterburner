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


///////////////////////////////////////////////////////////////////////////
//
// CPasswordScreen
//
// This class allows a password entry page to pop up
//
///////////////////////////////////////////////////////////////////////////

#include "PasswordScreen.h"
#include "KeyPad.h"
#include "../Protocol/helpers.h"
#include "../Wifi/BTCWifi.h"
#include "fonts/Arial.h"


CPasswordScreen::CPasswordScreen(C128x64_OLED& display, CScreenManager& mgr) : CScreenHeader(display, mgr) 
{
  _bGetPassword = false;
  _bPasswordOK = false;
  _PWcol = 0;
  for(int i= 0; i < 4; i++) 
    _PWdig[i] = -1;
  _SaveTime = 0;
}


void 
CPasswordScreen::show()
{
  CScreenHeader::show();

  char str[16];
  int xPos, yPos;
  const int col2 = 90;
  const int col3 = _display.width() - border;
  _printInverted(0, 16, _Title, true);

  if(_SaveTime) {
    long tDelta = millis() - _SaveTime;
    if(tDelta > 0) 
      _SaveTime = 0;
    _printInverted(_display.xCentre(), 28, "         ", true, eCentreJustify);
    _printInverted(_display.xCentre(), 39, "         ", true, eCentreJustify);
    _printInverted(_display.xCentre(), 34, " STORING ", true, eCentreJustify);
  }
  else {

    if(_bGetPassword) {
      _printMenuText(_display.xCentre(), 34, "Enter password...", false, eCentreJustify);
      _showPassword();
    }
  }
}


void 
CPasswordScreen::keyHandler(uint8_t event)
{
  if(_bGetPassword) {
    if(event & keyPressed) {

      // press CENTRE
      if(event & key_Centre) {
        // match "1688"
        if((_PWdig[0] == 1) && 
           (_PWdig[1] == 6) && 
           (_PWdig[2] == 8) && 
           (_PWdig[3] == 8)) {
          _bPasswordOK = true;
        }
        // reset PW digits
        for(int i= 0; i < 4; i++) 
          _PWdig[i] = -1;

      return;
      }

      // press LEFT 
      if(event & key_Left) {
        _PWcol--;
        LOWERLIMIT(_PWcol, 0);
      }

      // press RIGHT 
      if(event & key_Right) {
        _PWcol++;
        UPPERLIMIT(_PWcol, 5);
      }

      // press UP 
      if(event & key_Up) {
        _PWdig[_PWcol]++; 
        ROLLUPPERLIMIT(_PWdig[_PWcol], 9, 0);
      }

      // press DOWN
      if(event & key_Down) {
        _PWdig[_PWcol]--; 
        ROLLLOWERLIMIT(_PWdig[_PWcol], 0, 9);
      }
      _ScreenManager.reqUpdate();
    }
  }
}

bool
CPasswordScreen::_showPassword()
{
  if(_bGetPassword) {
    _printMenuText(_display.xCentre(), 34, "Enter password...", false, eCentreJustify);

    // determine metrics of character sizing
    CRect extents;
    _display.getTextExtents("X", extents);
    int charWidth = extents.width;
    _display.getTextExtents(" ", extents);
    int spaceWidth = extents.width;

    for(int idx =0 ; idx < 4; idx++) {

      extents.xPos = _display.xCentre() - (2 - idx) * (charWidth * 1.5); 
      extents.yPos = 50;

      char str[8];

      if(_PWdig[idx] < 0) {
        strcpy(str, "-");
      }
      else {
        sprintf(str, "%d", _PWdig[idx]);
      }
      _printMenuText(extents.xPos, extents.yPos, str, _PWcol == idx);
    }
  }
  return _bGetPassword;
}

void 
CPasswordScreen::_setGetPassword(bool state)
{
  _bGetPassword = state;
  if(state) {
    _bPasswordOK = false;
    _PWcol = 0;
    // reset PW digits
    for(int i= 0; i < 4; i++) 
      _PWdig[i] = -1;
  }
}

void 
CPasswordScreen::_setTitle(const char* title)
{
  strcpy(_Title, title);
}

bool 
CPasswordScreen::_isPasswordOK()
{
  return _bPasswordOK;
}