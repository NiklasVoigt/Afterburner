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

#include <stdint.h>
#include "ScreenHeader.h"

class C128x64_OLED;
class CScreenManager;
class CProtocol;

class CPasswordScreen : public CScreenHeader {
  int  _PWdig[4];
  bool _bGetPassword;
  bool _bPasswordOK;
  int  _PWcol;
  unsigned long _SaveTime;
  char _Title[20];
protected:
  bool _showPassword();
  void _setGetPassword(bool state);
  void _setTitle(const char* title);
  bool _isPasswordOK();
public:
  CPasswordScreen(C128x64_OLED& display, CScreenManager& mgr);
  void show();
  void keyHandler(uint8_t event);
  bool animate() { return CScreen::animate(); };
};