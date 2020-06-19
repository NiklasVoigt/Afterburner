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

#include "128x64OLED.h"
#include "VersionInfoScreen.h"
#include "KeyPad.h"
#include "../Utility/helpers.h"
#include "../Utility/UtilClasses.h"
#include "../Utility/NVStorage.h"
#include "../WiFi/BTCWifi.h"
#include "../Utility/BoardDetect.h"
#include "fonts/Icons.h"

// nominally show the current version of firmware & hardware
// from here we can also update the firmware using web server update (requires internet STA connection)
// or factory default the stored non volatile memory contents
//
// progression is basically via the UP key:

//  _rowSel=0 - standard view, may animate upload arrow if update is available, help prompt shows 'Exit': 
//       CENTRE > exit menu
//
//  UP > _rowSel=1 - if update is available, help prompt shows 'Get Update', otherwise a silent step:
//       CENTRE > _rowSel=20 - present firmware update confirmation (UP to perform)
//           UP > update initated, reboot upon conclusion, % progress shown on display
//
//  UP > _rowSel=2 - Factory default cancel selection, help prompt shows 'Exit':
//       CENTRE > exit menu
//
//  UP > _rowSel=3 - Factory default perform selection, help prompt shows 'Apply':
//       CENTRE > _rowSel=10 - request factory default confirm 
//           UP > _rowSel=11 - defaults installed, present DONE screen, REBOOT after 5 seconds 


CVersionInfoScreen::CVersionInfoScreen(C128x64_OLED& display, CScreenManager& mgr) : CUIEditScreen(display, mgr) 
{
}

bool 
CVersionInfoScreen::onSelect()
{
  CUIEditScreen::onSelect();
  checkFOTA();
  return true;
}

bool 
CVersionInfoScreen::show()
{
  _display.clearDisplay();

  if(!CUIEditScreen::show()) {  // for showing "saving settings"

    if(_rowSel < 2) {
      // standard version information screens,
      // animation of update available via animate() if firmware update is available on web server
      _showTitle("Version Information");
      
      _drawBitmap(8, 12, FirmwareIconInfo);
      _printMenuText(41, 15, getVersionStr());
      _printMenuText(41, 26, getVersionDate());
       int newVer = isUpdateAvailable();
      _drawBitmap(18, 34, HardwareIconInfo);
      int PCB = getBoardRevision();
//      _printMenuText(41, 38, getBoardRevisionString(PCB));
      if(PCB == BRD_V2_GPIO_NOALG || PCB == BRD_V3_GPIO_NOALG) {
        _display.fillRect(41, 36, 57, 11, WHITE);
        _printInverted(70, 38, "No Analog", true, eCentreJustify);
      }
      else if(PCB == BRD_V2_NOGPIO) {
        _display.fillRect(41, 36, 45, 11, WHITE);
        _printInverted(76, 38, "No GPIO", true, eCentreJustify);
      }
      else {
        _display.fillRect(41, 36, 57, 11, WHITE);
        _printInverted(70, 38, "Full GPIO", true, eCentreJustify);
      }

      if(_rowSel == 1 && newVer) {
        // prompt 'Get Update' for new firmware available and first UP press from home
        char msg[32];
        int major = (int)(newVer * 0.01);
        int minor = newVer - major*100;
        float prtMajor = major * 0.1;
        sprintf(msg, "Get V%.1f.%d update", prtMajor, minor);
        _printMenuText(_display.xCentre(), 53, "                    ", true, eCentreJustify);
        _printMenuText(_display.xCentre(), 53, msg, false, eCentreJustify);
      }
      else {
        _printMenuText(_display.xCentre(), 53, " \021                \020 ", true, eCentreJustify);
        _printMenuText(_display.xCentre(), 53, "Exit", false, eCentreJustify);
      }
    }
    else {
      if(_rowSel == 11) {  // after the saving popup has expired
        // factory default completed screen, progress to REBOOT
        const char* content[2];
        content[0] = "Factory reset";
        content[1] = "completed";
        _ScreenManager.showRebootMsg(content, 5000);
      }
      else if(_rowSel == 20) {
        // firmware update confirmation screen
        _showTitle("Firmware update");
        _printMenuText(_display.xCentre(), 35, "Press UP to", false, eCentreJustify);
        _printMenuText(_display.xCentre(), 43, "confirm download", false, eCentreJustify);
      }
      else {
        _showTitle("Factory Default");
        if(_rowSel == SaveConfirm) {
          // factory default confirmation screen
          _printMenuText(_display.xCentre(), 35, "Press UP to", false, eCentreJustify);
          _printMenuText(_display.xCentre(), 43, "confirm save", false, eCentreJustify);
        }
        else {
          // factory default apply/abort screens
          _drawBitmap(10, 15, CautionIconInfo);

          _printMenuText(50, 30, "Abort", _rowSel == 2);
          _printMenuText(50, 16, "Apply", _rowSel == 3);
          if(_rowSel == 3) 
            _printMenuText(_display.xCentre(), 53, " \021    Apply     \020 ", true, eCentreJustify);
          else
            _printMenuText(_display.xCentre(), 53, " \021     Exit     \020 ", true, eCentreJustify);
        }
      }
    }
  }

  return true;
}

bool
CVersionInfoScreen::animate()
{
  CUIEditScreen::animate();

  if(_rowSel <= 1 && isUpdateAvailable()) {
    // show ascending up arrow if firmware update is available on web server
    _animateCount++;
    WRAPUPPERLIMIT(_animateCount, 3, 0);

    int newVer = isUpdateAvailable();
    if((_animateCount & 0x02) && newVer) {
      _display.setTextColor(WHITE);
      _drawBitmap(118, 24, UpdateIconInfo);
    }
    else {
      _display.setTextColor(BLACK);
      _display.fillRect(118, 24, 9, 10, BLACK);
    }
    char msg[32];
    int major = (int)(newVer * 0.01);
    int minor = newVer - major*100;
    float prtMajor = major * 0.1;
    sprintf(msg, "V%.1f.%d", prtMajor, minor);
    _printMenuText(128, 15, msg, false, eRightJustify);
  }
  return true;
}

bool 
CVersionInfoScreen::keyHandler(uint8_t event)
{
  if(CUIEditScreen::keyHandler(event)) {  // handle confirm save
    return true;
  }

  if(event & keyPressed) {

    if(_rowSel == 20) {      // firmware update confirm
      if(event & key_Up) {
        isUpdateAvailable(false);   // make firmware update happen
      }
      _rowSel = 0;
      return true;
    }

    // UP press
    if(event & key_Up) {
      if(NVstore.getUserSettings().menuMode != 2) {
        _rowSel++;
        UPPERLIMIT(_rowSel, 3);
      } 
    }
    // DOWN press
    if(event & key_Down) {
      _rowSel--;
      LOWERLIMIT(_rowSel, 0);
    }
    // LEFT press
    if(event & key_Left) {
      _ScreenManager.prevMenu();
    }
    // RIGHT press
    if(event & key_Right) {
      _ScreenManager.nextMenu();
    }
    // CENTRE press
    if(event & key_Centre) {
      if(_rowSel == 3) {  // factory enable selection
        _confirmSave();
      }
      else if(_rowSel == 1) {  // firmware update selection
        _rowSel = 20;
      }
      else {
        _ScreenManager.selectMenu(CScreenManager::RootMenuLoop);  // force return to main menu
      }
    }
  }

  _ScreenManager.reqUpdate();

  return true;
}

void 
CVersionInfoScreen::_saveNV()
{
  wifiFactoryDefault();
  BoardRevisionReset();
  NVstore.init();
  NVstore.save();
  _rowSel = 11;

  const char* content[2];
  content[0] = "Factory reset";
  content[1] = "completed";
  _ScreenManager.showRebootMsg(content, 5000);
}
