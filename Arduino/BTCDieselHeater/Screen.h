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

#ifndef __SCREEN_H__
#define __SCREEN_H__

#include <Arduino.h>
#include "FontTypes.h"
#include "UtilClasses.h"
#include "ScreenManager.h"
#include "128x64OLED.h"

class CProtocol;

enum eJUSTIFY { 
   eLeftJustify, eCentreJustify, eRightJustify 
};

const int border = 3;
const int radius = 4;

class CScreen {
protected:
  C128x64_OLED& _display;
  CScreenManager& _ScreenManager;
  void _printMenuText(int x, int y, const char* str, bool selected = false, eJUSTIFY justify = eLeftJustify, int border = 3, int radius = 4);
  void _printInverted(int x, int y, const char* str, bool selected, eJUSTIFY justify = eLeftJustify);
  void _adjustExtents(CRect& rect, eJUSTIFY justify, const char* str);
  void _drawMenuSelection(CRect extents, const char* str, int border = 3, int radius = 4);
public:
  CScreen(C128x64_OLED& disp, CScreenManager& mgr); 
  virtual ~CScreen(); 
  virtual void onSelect();
  virtual bool animate();
  virtual void show();
  virtual void keyHandler(uint8_t event) {};
};


class CTransientFont {
  C128x64_OLED& _display;
public:
  CTransientFont(C128x64_OLED& disp, const FONT_INFO* pFont);
  ~CTransientFont(); 
};

#endif // __SCREEN_H__