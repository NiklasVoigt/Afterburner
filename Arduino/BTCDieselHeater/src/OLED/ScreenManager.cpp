#include "ScreenManager.h"
#include "DetailedScreen.h"
#include "BasicScreen.h"
#include "PrimingScreen.h"
#include "WiFiScreen.h"
#include "FuelMixtureScreen.h"
#include "SetClockScreen.h"
#include "SetTimerScreen.h"
#include "ClockScreen.h"
#include "RebootScreen.h"
#include "HeaterSettingsScreen.h"
#include "SettingsScreen.h"
#include <Wire.h>
#include "../cfg/pins.h"
#include "../cfg/BTCConfig.h"
#include "../protocol/helpers.h"


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
  _currentScreen = -1;
  _timerScreen = -1;
  _settingScreen = -1;
  _bReqUpdate = false;
  _bSetTime = false;
  _DimTime = millis() + 60000;
  _pRebootScreen = NULL;
}

CScreenManager::~CScreenManager()
{
	for(int i=0; i < _Screens.size(); i++) {
		if(_Screens[i]) {
			delete _Screens[i];
			_Screens[i] = NULL;
		}
	}
	for(int i=0; i < _TimerScreens.size(); i++) {
		if(_TimerScreens[i]) {
			delete _TimerScreens[i];
			_TimerScreens[i] = NULL;
		}
	}
  if(_SetTimeScreen) {
    delete _SetTimeScreen;
    _SetTimeScreen = NULL;
  }
  if(_pDisplay) {
    delete _pDisplay; _pDisplay = NULL;
  }
}

void 
CScreenManager::begin(bool bNoClock)
{

  // 128 x 64 OLED support (I2C)
  // xxxx_SWITCHCAPVCC = generate display voltage from 3.3V internally
  _pDisplay = new C128x64_OLED(OLED_SDA_pin, OLED_SCL_pin);
#if USE_ADAFRUIT_SH1106 == 1
  _pDisplay->begin(SH1106_SWITCHCAPVCC);
   Wire.begin(OLED_SDA_pin, OLED_SCL_pin, 800000);   // speed up I2C from the default crappy 100kHz set via the adafruit begin!
#elif USE_ADAFRUIT_SSD1306 == 1
  _pDisplay->begin(SSD1306_SWITCHCAPVCC, 0x3c);
  _pDisplay->ssd1306_command(SSD1306_SETPRECHARGE); // 0xd9
  _pDisplay->ssd1306_command(0x1F);  // correct lame reversal of OLED current phases
#endif

  // replace adafruit splash screen
  _pDisplay->clearDisplay();
  _pDisplay->drawBitmap(0, 0, DieselSplash, 128, 64, WHITE);

  // Show initial display buffer contents on the screen --
  _pDisplay->display();

  DebugPort.println("Creating Screens");
  _Screens.push_back(new CDetailedScreen(*_pDisplay, *this));         //  detail control
  _Screens.push_back(new CBasicScreen(*_pDisplay, *this));            //  basic control
  if(!bNoClock)
    _Screens.push_back(new CClockScreen(*_pDisplay, *this));          //  clock
  _Screens.push_back(new CPrimingScreen(*_pDisplay, *this));          //  mode / priming
  _Screens.push_back(new CWiFiScreen(*_pDisplay, *this));             //  comms info
  _Screens.push_back(new CSettingsScreen(*_pDisplay, *this));         // tuning info
  _SetTimeScreen = new CSetClockScreen(*_pDisplay, *this);            // clock set
  _TimerScreens.push_back(new CSetTimerScreen(*_pDisplay, *this, 0)); // set timer 1
  _TimerScreens.push_back(new CSetTimerScreen(*_pDisplay, *this, 1)); // set timer 2
  _SettingsScreens.push_back(new CFuelMixtureScreen(*_pDisplay, *this));      //  tuning
  _SettingsScreens.push_back(new CHeaterSettingsScreen(*_pDisplay, *this));   // tuning

#if RTC_USE_DS3231==0 && RTC_USE_DS1307==0 && RTC_USE_PCF8523==0
  _currentScreen = 6;   // bring up clock set screen first if using millis based RTC!
#else
	_currentScreen = 1;   // basic control screen
#endif

  _switchScreen();
}

bool 
CScreenManager::checkUpdate()
{
  if(_DimTime) {
    long tDelta = millis() - _DimTime;
    if(tDelta > 0) {
      // time to dim the display
//      if(NVstore.getDimTime())
        _pDisplay->dim(true);
      _DimTime = 0;

      // fall back to main menu 
      selectTimerScreen(false);
      selectSetTimeScreen(false);
      selectSettingsScreen(false);
      // sticky screens are Detailed Control, Basic Control, or Clock.
      // otherwise return to Basic Control screen
      if(_currentScreen > 2) {
        _currentScreen = 1;
      }
    }
  }

  if(_bReqUpdate) {
    if(_pRebootScreen) {
      _pRebootScreen->show();
      _bReqUpdate = false;
      return true;
    }
    else {
      if(_bSetTime) {
        _SetTimeScreen->show();
        _bReqUpdate = false;
        return true;
      }
      else if(_settingScreen >= 0) {
        _SettingsScreens[_settingScreen]->show();
        _bReqUpdate = false;
        return true;
      }
      else if(_timerScreen >= 0) {
        _TimerScreens[_timerScreen]->show();
        _bReqUpdate = false;
        return true;
      }
      else if(_currentScreen >= 0) {
        _Screens[_currentScreen]->show();
        _bReqUpdate = false;
        return true;
      }
    }
  }
  return false;
}
  
void 
CScreenManager::reqUpdate()
{
  _bReqUpdate = true;
}

bool 
CScreenManager::animate()
{
	if(_settingScreen >= 0) return _SettingsScreens[_settingScreen]->animate();
	if(_timerScreen >= 0)   return _TimerScreens[_timerScreen]->animate();
	if(_currentScreen >= 0) return _Screens[_currentScreen]->animate();
	return false;
}

void
CScreenManager::refresh()
{
	if(_pDisplay)
		_pDisplay->display();
}

void 
CScreenManager::_switchScreen()
{
  if(_timerScreen >= 0)        _TimerScreens[_timerScreen]->onSelect();
  else if(_settingScreen >= 0) _SettingsScreens[_settingScreen]->onSelect();
  else if(_currentScreen >= 0) _Screens[_currentScreen]->onSelect();
		
  reqUpdate();
}

void 
CScreenManager::nextScreen()
{
  if(_bSetTime) {

  }
  else if(_timerScreen >= 0) {
    _timerScreen++;
    ROLLUPPERLIMIT(_timerScreen, _TimerScreens.size()-1, 0);
  }
  else if(_settingScreen >= 0) {
    _settingScreen++;
    ROLLUPPERLIMIT(_settingScreen, _SettingsScreens.size()-1, 0);
  }
  else {
    _currentScreen++;
    ROLLUPPERLIMIT(_currentScreen, _Screens.size()-1, 0);
  }
  _switchScreen();
}

void 
CScreenManager::prevScreen()
{
  if(_bSetTime) {
  }
  else if(_timerScreen >=0) {
    _timerScreen--;
    ROLLLOWERLIMIT(_timerScreen, 0, _TimerScreens.size()-1);
  }
  else if(_settingScreen >= 0) {
    _settingScreen--;
    ROLLLOWERLIMIT(_settingScreen, 0, _SettingsScreens.size()-1);
  }
  else {
    _currentScreen--;
    ROLLLOWERLIMIT(_currentScreen, 0, _Screens.size()-1);
  }
  _switchScreen();
}

void 
CScreenManager::keyHandler(uint8_t event)
{
  if(_DimTime == 0) {
    _pDisplay->dim(false);
    _DimTime = (millis() + NVstore.getDimTime()) | 1;
    return;   // initial press when dimmed is thrown away
  }

  _DimTime = (millis() + NVstore.getDimTime()) | 1;

  // call handler for active screen
  if(_bSetTime)                _SetTimeScreen->keyHandler(event);
  else if(_settingScreen >= 0) _SettingsScreens[_settingScreen]->keyHandler(event);
  else if(_timerScreen >= 0)   _TimerScreens[_timerScreen]->keyHandler(event);
	else if(_currentScreen >= 0) _Screens[_currentScreen]->keyHandler(event);

}

void 
CScreenManager::selectTimerScreen(bool show)
{
  _timerScreen = show ? 0 : -1;
  _settingScreen = -1;
  _bSetTime = false;
  _switchScreen();
}

void 
CScreenManager::selectSettingsScreen(bool show)
{
  _settingScreen = show ? 0 : -1;
  _timerScreen = -1;
  _bSetTime = false;
  _switchScreen();
}

void 
CScreenManager::selectSetTimeScreen(bool show)
{
  _bSetTime = show;
  _settingScreen = -1;
  _timerScreen = -1;
  _switchScreen();
}

void 
CScreenManager::showRebootMsg(const char* content[2], long delayTime)
{
  if(_pRebootScreen == NULL)
    _pRebootScreen = new CRebootScreen(*_pDisplay, *this);

  _pRebootScreen->setMessage(content, delayTime);
  _bReqUpdate = true;
  _pDisplay->dim(false);
}

