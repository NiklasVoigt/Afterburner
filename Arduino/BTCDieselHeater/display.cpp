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

#include <SPI.h>
#include <Wire.h>
#include "128x64OLED.h"
#include "MiniFont.h"
#include "tahoma16.h"
#include "protocol.h" 
#include "display.h"
#include "pins.h"
#include "BluetoothAbstract.h" 
#include "OLEDconsts.h"
#include "BTCWifi.h"
#include "BluetoothAbstract.h" 
#include "Screen1.h"
#include "Screen2.h"
#include "Screen3.h"
#include "Screen4.h"
#include "Screen5.h"
#include "KeyPad.h"
#include "helpers.h"
#include "clock.h"
#include "BTCConfig.h"

#define MAXIFONT tahoma_16ptFontInfo
#define MINIFONT miniFontInfo

#define X_BATT_ICON    95
#define Y_BATT_ICON     0
#define X_WIFI_ICON    22
#define Y_WIFI_ICON     0
#define X_BT_ICON      12
#define Y_BT_ICON       0

#define MINI_BATTLABEL

//
// **** NOTE: There are two very lame libaries conspiring to make life difficult ****
// A/ The ESP32 SPI.cpp library instatiates an instance of SPI, using the VSPI port (and pins)
// B/ The Adfruit_SH1106 library has hard coded "SPI" as the SPI port instance
//
// As an ESP32 has a pin multiplexer, this is very bad form.
// The design uses the defacto HSPI pins (and port), 
// You **MUST comment out the SPIClass SPI(VSPI);**  at the end of the ESP32 SPI library
// then we declare "SPI" here, which will use HSPI!!!!

// 128 x 64 OLED support
SPIClass SPI;    // default constructor opens HSPI on standard pins : MOSI=13,CLK=14,MISO=12(unused)

////////////////////////////////////////////////////////////////////////////////////////////////
// splash creen created using image2cpp http://javl.github.io/image2cpp/
//   Settings: 
//      Black background
//      Invert [X]
//      Arduino code, single bitmap
//      Identifier: DieselSplash
//      Draw Mode: Horizontal
//
const unsigned char DieselSplash [] PROGMEM = {
	// 'Splash2, 128x64px
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 
	0x01, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x3e, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x06, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 
	0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 
	0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 
	0x00, 0x08, 0x00, 0x00, 0x00, 0x0f, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x08, 0x00, 0x00, 
	0x00, 0x10, 0x00, 0x00, 0x21, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x88, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x20, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x84, 0x00, 0x00, 
	0x00, 0x14, 0x00, 0x00, 0x20, 0x40, 0x00, 0x20, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x84, 0x00, 0x00, 
	0x00, 0x52, 0x00, 0x00, 0x40, 0x40, 0x00, 0x10, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x84, 0x00, 0x00, 
	0x00, 0x34, 0x00, 0x00, 0x40, 0x20, 0x00, 0x08, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x82, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x00, 0x40, 0x10, 0x00, 0x04, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x34, 0x00, 0x00, 0x80, 0x10, 0x00, 0x02, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x52, 0x00, 0x00, 0x80, 0x08, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x14, 0x00, 0x00, 0x80, 0x08, 0x01, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x18, 0x00, 0x01, 0x00, 0x04, 0x3e, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x10, 0x00, 0x01, 0x00, 0x07, 0xc0, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x06, 0x80, 0x1c, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0xc4, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x18, 0x40, 0x64, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x3c, 0x78, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3c, 0x40, 0x84, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01, 0xc0, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x26, 0x23, 0x04, 0x00, 0x00, 0x00, 0x60, 0x00, 0x1e, 0x00, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x43, 0x1c, 0x02, 0x00, 0x00, 0x00, 0x1c, 0x01, 0xe0, 0x00, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x41, 0xf8, 0x02, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x41, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x60, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x30, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x10, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x19, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0x88, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x07, 0xf8, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x07, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xc2, 0x00, 0x00, 0xf8, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x1f, 0x20, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x03, 0xe8, 0x20, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x7c, 0x08, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x08, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x04, 0x00, 0x00, 0x00, 0x00, 0x21, 0x10, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x0a, 0x08, 0x00, 0x00, 0x00, 0x21, 0x10, 0x01, 0x00, 0x00, 0x84, 0x00, 0x40, 0x01, 0x40, 0x00, 
	0x11, 0x08, 0x00, 0x00, 0x00, 0x21, 0x10, 0x01, 0x00, 0x00, 0x80, 0x00, 0x40, 0x01, 0x40, 0x00, 
	0x11, 0x3a, 0x4e, 0x73, 0x18, 0xe1, 0x13, 0x1d, 0xcc, 0xb0, 0x80, 0xc7, 0x75, 0x99, 0x4c, 0xb0, 
	0x1f, 0x4a, 0x52, 0x94, 0xa5, 0x21, 0xf4, 0xa5, 0x12, 0xc0, 0x81, 0x29, 0x46, 0x25, 0x52, 0xc0, 
	0x11, 0x4a, 0x52, 0x94, 0x3d, 0x21, 0x17, 0xa5, 0x1e, 0x80, 0x81, 0x29, 0x44, 0x25, 0x5e, 0x80, 
	0x11, 0x49, 0x92, 0x94, 0xa1, 0x21, 0x14, 0x25, 0x10, 0x80, 0x85, 0x29, 0x44, 0x25, 0x50, 0x80, 
	0x11, 0x39, 0x8e, 0x93, 0x18, 0xe1, 0x13, 0x1c, 0xcc, 0x80, 0x78, 0xc9, 0x34, 0x19, 0x4c, 0x80
};

CScreenManager::CScreenManager() 
{
  _pDisplay = NULL;
  _pActiveScreen = NULL;
  for(int i = 0; i < _maxScreens; i++)
    _pScreen[i] = NULL;
  _currentScreen = 1;
  _bReqUpdate = false;
}

CScreenManager::~CScreenManager()
{
  _pActiveScreen = NULL;      
  for(int i=0; i<_maxScreens; i++) {
    if(_pScreen[i]) {
      delete _pScreen[i]; _pScreen[i] = NULL;
    }
  }
  if(_pDisplay) {
    delete _pDisplay; _pDisplay = NULL;
  }
}

void 
CScreenManager::init()
{

  // 128 x 64 OLED support (Hardware SPI)
  // SH1106_SWITCHCAPVCC = generate display voltage from 3.3V internally
#if OLED_HW_SPI == 1
  SPI.setFrequency(8000000);
  _pDisplay = new C128x64_OLED(OLED_DC_pin, -1, OLED_CS_pin);
  _pDisplay->begin(SH1106_SWITCHCAPVCC, 0, false);
#else
  _pDisplay = new C128x64_OLED(OLED_SDA_pin, OLED_SCL_pin);
  _pDisplay->begin(SH1106_SWITCHCAPVCC);
  Wire.begin(OLED_SDA_pin, OLED_SCL_pin, 800000);   // speed up I2C from the default crappy 100kHz set via the adafruit begin!
#endif

  // replace adafruit splash screen
  _pDisplay->clearDisplay();
  _pDisplay->drawBitmap(0, 0, DieselSplash, 128, 64, WHITE);

  // Show initial display buffer contents on the screen --
  _pDisplay->display();

  _pScreen[0] = new CScreen1(*_pDisplay, *this);
  _pScreen[1] = new CScreen2(*_pDisplay, *this);
  _pScreen[2] = new CScreen3(*_pDisplay, *this);
  _pScreen[3] = new CScreen4(*_pDisplay, *this);
  _pScreen[4] = new CScreen5(*_pDisplay, *this);

  _switchScreen();
}

void 
CScreenManager::checkUpdate(const CProtocol& CtlFrame, const CProtocol& HtrFrame)
{
  if(_bReqUpdate) {
    if(_pActiveScreen) {
      _pActiveScreen->show(CtlFrame, HtrFrame);
      _bReqUpdate = false;
    }
  }
}
  
void 
CScreenManager::reqUpdate()
{
  _bReqUpdate = true;
}

void 
CScreenManager::animate()
{
  if(_pActiveScreen) _pActiveScreen->animate();
}

void 
CScreenManager::_switchScreen()
{
  if(_currentScreen < _maxScreens)
    _pActiveScreen = _pScreen[_currentScreen]; 
  
  //        reqDisplayUpdate();
  reqUpdate();
}

void 
CScreenManager::nextScreen()
{
  _currentScreen++;
  if(_currentScreen >= _maxScreens) {
    _currentScreen = 0;
  }
  _switchScreen();
}

void 
CScreenManager::prevScreen()
{
  _currentScreen--;
  if(_currentScreen < 0) {
    _currentScreen = _maxScreens-1;
  }
  _switchScreen();
}

void 
CScreenManager::keyHandler(uint8_t event)
{
  if(_pActiveScreen) _pActiveScreen->keyHandler(event);
}



CScreen::CScreen(C128x64_OLED& disp, CScreenManager& mgr) : 
  _display(disp), 
  _Manager(mgr) 
{
}

CScreen::~CScreen()
{
}

void
CScreen::animate()
{
  _display.display();
}

void 
CScreen::show(const CProtocol& CtlFrame, const CProtocol& HtrFrame)
{
  _display.clearDisplay();

  // standard header items
  //Bluetooth
  if(getBluetoothClient().isConnected())
    showBTicon();
  // WiFi
  if(isWifiConnected()) {
    showWifiIcon();
  }
  // battery
  float voltage = HtrFrame.getVoltage_Supply() * 0.1f;
  showBatteryIcon(voltage);

  showTime(_display);
}

void 
CScreen::showBTicon()
{
  _display.drawBitmap(X_BT_ICON, Y_BT_ICON, BTicon, W_BT_ICON, H_BT_ICON, WHITE);
}

void 
CScreen::showWifiIcon()
{
  _display.drawBitmap(X_WIFI_ICON, Y_WIFI_ICON, wifiIcon, W_WIFI_ICON, H_WIFI_ICON, WHITE);
#ifdef DEMO_AP_MODE
  _display.fillRect(X_WIFI_ICON + 8, Y_WIFI_ICON + 5, 10, 7, BLACK);
  _display.setFontInfo(&MINIFONT);  // select Mini Font
  _display.setCursor(X_WIFI_ICON+9, Y_WIFI_ICON+6);
  _display.print("AP");
  _display.setFontInfo(NULL);  
#endif
}

void
CScreen::showBatteryIcon(float voltage)
{
  _display.drawBitmap(X_BATT_ICON, Y_BATT_ICON, BatteryIcon, W_BATT_ICON, H_BATT_ICON, WHITE);
#ifdef MINI_BATTLABEL
  char msg[16];
  sprintf(msg, "%.1fV", voltage);
  int xPos = X_BATT_ICON + 7 - strlen(msg) * 2;
  _display.setCursor(xPos, Y_BATT_ICON+H_BATT_ICON+2);
  _display.setFontInfo(&MINIFONT);  // select Mini Font
  _display.print(msg);
  _display.setFontInfo(NULL);  
#else
  _display.setCursor(85, 12);
  _display.setTextColor(WHITE);
  _display.print(voltage, 1);
  _display.print("V");
#endif

  // nominal 10.5 -> 13.5V bargraph
  int Capacity = (voltage - 10.7) * 4;
  if(Capacity < 0)   Capacity = 0;
  if(Capacity > 11)  Capacity = 11;
  _display.fillRect(X_BATT_ICON+2 + Capacity, Y_BATT_ICON+2, W_BATT_ICON-4-Capacity, 6, BLACK);
}

void
CScreen::_drawSelectionBox(int x, int y, const char* str, int border, int radius)
{
  CRect extents;
  _display.getTextExtents(str, extents);
  extents.xPos = x;
  extents.yPos = y;
  extents.Expand(border);
  _display.drawRoundRect(extents.xPos, extents.yPos, extents.width, extents.height, radius, WHITE);
}

void
CScreen::_drawSelectionBoxCentreJustified(int x, int y, const char* str, int border, int radius)
{
  CRect extents;
  _display.getTextExtents(str, extents);
  x -= extents.width / 2;
  _drawSelectionBox(x, y, str, border, radius);
}

void
CScreen::_drawSelectionBoxRightJustified(int x, int y, const char* str, int border, int radius)
{
  CRect extents;
  _display.getTextExtents(str, extents);
  x -= extents.width;
  _drawSelectionBox(x, y, str, border, radius);
}

