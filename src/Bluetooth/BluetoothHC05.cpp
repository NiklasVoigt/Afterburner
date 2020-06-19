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

#include "BluetoothHC05.h"
#include "../cfg/pins.h"
#include "../cfg/BTCConfig.h"
#include "../Protocol/Protocol.h"
#include "../Utility/helpers.h"
#include "../Utility/DebugPort.h"

// Bluetooth access via HC-05 Module, using a UART

static const int BTRates[] = {
  9600, 38400, 115200, 19200, 57600, 2400, 4800, 1200
};


#if USE_HC05_BLUETOOTH == 1
CBluetoothHC05::CBluetoothHC05(int keyPin, int sensePin)
{
  // extra control pins required to fully drive a HC05 module 
  _keyPin = keyPin;      // used to enable AT command mode (ONLY ON SUPPORTED MODULES!!!!)
  _sensePin = sensePin;  // feedback signal used to sense if a client is connected
  
  pinMode(_keyPin, OUTPUT);              
  digitalWrite(_keyPin, LOW);              // request HC-05 module to enter data mode
  // attach to the SENSE line from the HC-05 module
  // this line goes high when a BT client is connected :-)
  pinMode(_sensePin, INPUT);              
  _bTest = false;
  _bGotMAC = false;
  _BTbaudIdx = 0;
  strcpy(_MAC, "unknown");
}


void 
CBluetoothHC05::begin()
{
  _rxLine.clear();

  _setCommandMode(true);
  // digitalWrite(_keyPin, HIGH);              // request HC-05 module to enter command mode

  // delay(50);

  // _openSerial(9600); // virtual function, may call derived class method here

  DebugPort.println("\r\n\r\nAttempting to detect HC-05 Bluetooth module...");

//  int BTidx = 0;
  int maxTries =  sizeof(BTRates)/sizeof(int);
  for(_BTbaudIdx = 0; _BTbaudIdx < maxTries; _BTbaudIdx++) {
    DebugPort.printf("  @ %d baud... ", BTRates[_BTbaudIdx]);
    _openSerial(BTRates[_BTbaudIdx]);      // open serial port at a std. baud rate
    delay(10);
    _flush();
    HC05_SerialPort.print("AT\r\n");   // clear the throat!
    delay(100);
    HC05_SerialPort.setTimeout(100);

    if(ATCommand("AT\r\n")) {        // probe with a simple "AT"
      DebugPort.println(" OK.");     // got a response - woo hoo found the module!
      break;
    }
    if(ATCommand("AT\r\n")) {        // sometimes a second try is good...
      DebugPort.println(" OK.");
      break;
    }

    // failed, try another baud rate
    DebugPort.println("");
    HC05_SerialPort.flush();
    HC05_SerialPort.end();
    delay(100);
  }

  DebugPort.println("");
  if(_BTbaudIdx == maxTries) {
    // we could not get anywhere with the AT commands, but maybe this is the other module
    // plough on and assume 9600 baud, but at the mercy of whatever the module name is...
    DebugPort.println("FAILED to detect a HC-05 Bluetooth module :-(");
    // leave the EN pin high - if other style module keeps it powered!
    // assume it is 9600, and just (try to) use it like that...
    // we will sense the STATE line to prove a client is hanging off the link...
    DebugPort.println("ASSUMING a HC-05 module @ 9600baud (Unknown name)");
//    _openSerial(9600); 
    _BTbaudIdx = 0;
    _setCommandMode(false);
  }
  else {
    // found a HC-05 module at one of its supported baud rates.
    // now program it's name and force a 9600 baud data interface.
    // this is the defacto standard as shipped!

    DebugPort.println("HC-05 found");

    Reset(true);  // reset, staying in command mode

    _openSerial(38400);      // open serial port at a std. baud rate

    delay(100);

    DebugPort.print("  Setting Name to \"Afterburner\"... ");
    if(!ATCommand("AT+NAME=\"Afterburner\"\r\n")) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
    }

    DebugPort.print("  Setting baud rate to 9600N81...");
    //if(!ATCommand("AT+UART=9600,1,0\r\n")) {
    if(!ATCommand("AT+UART=38400,1,0\r\n")) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
    }
/*    DebugPort.print("  Lowering power consumption...");
    if(!ATCommand("AT+IPSCAN=1024,1,1024,1\r\n")) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
    }*/
    DebugPort.print("  Getting MAC address...");
    int len = 32;
    char response[32];
    if(!ATResponse("AT+ADDR?\r\n", "+ADDR:", response, len)) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
      _decodeMACresponse(response, len);
      DebugPort.print("    "); DebugPort.println(_MAC);
    }
/*
    DebugPort.print("  Lowering power consumption...");
    if(!ATCommand("AT+SNIFF=40,20,1,8\r\n")) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
    }

    DebugPort.print("  Lowering power consumption...");
    if(!ATCommand("AT+ENSNIFF=0002,72,0A3C7F\r\n")) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
    }*/
    _flush();
    delay(100);  
    // _openSerial(9600); 

    // leave HC-05 command mode, return to data mode
    Reset(false);  // reset, shift into data mode6
    // digitalWrite(_keyPin, LOW);  

  }

  delay(50);
  _flush();    // ensure any AT command reponse dribbles are cleaned up!

  DebugPort.println("");
}


void 
CBluetoothHC05::check()
{  
  // check for data coming back over Bluetooth
  if(HC05_SerialPort.available()) {           // serial rx data is available
    char rxVal = HC05_SerialPort.read();
    if(_bTest) {   
      DebugPort.print(rxVal);
    }
    else {
      collectRxData(rxVal);
    }
  }
}

bool 
CBluetoothHC05::isConnected()
{
  return digitalRead(_sensePin);
}

bool
CBluetoothHC05::send(const char* Str)
{
  if(isConnected() && !_bTest) {
    HC05_SerialPort.print(Str);
    return true;
  }
  else {
//    DebugPort.print("No Bluetooth client");
    return false;
  }
}

void 
CBluetoothHC05::_openSerial(int baudrate)
{
  // standard serial port for Due, Mega (ESP32 uses virtual, derived from this class)
  HC05_SerialPort.begin(baudrate);
}

// protected function, to perform Hayes commands with HC-05
bool 
CBluetoothHC05::ATCommand(const char* cmd)
{
  if(!_bTest) {
    _flush();   // ensure response is for *this* command!
    HC05_SerialPort.print(cmd);
    char RxBuffer[16];
    memset(RxBuffer, 0, 16);
    int read = HC05_SerialPort.readBytesUntil('\n', RxBuffer, 32);  // \n is not included in returned string!
    if((read == 3) && (0 == strcmp(RxBuffer, "OK\r")) ) {
      return true;
    }
  }
  return false;
}

bool
CBluetoothHC05::Reset(bool keystate)
{
  HC05_SerialPort.print("AT+RESET\r\n");
  digitalWrite(_keyPin, keystate ? HIGH : LOW);  
  delay(1000);
  _flush();   
  return true;
}

// protected function, to perform Hayes commands with HC-05
bool 
CBluetoothHC05::ATResponse(const char* cmd, const char* respHdr, char* response, int& len)
{
  if(!_bTest) {
    _flush();   // ensure response is for *this* command!
    HC05_SerialPort.print(cmd);
    memset(response, 0, len);
    int read = HC05_SerialPort.readBytesUntil('\n', response, len);  // \n is not included in returned string!
//    DebugPort.print(response); DebugPort.print(" ? "); DebugPort.println(respHdr);
    if(0 == strncmp(response, respHdr, strlen(respHdr))) {
      len = read;
      return true;
    }
    len = 0;
  }
  return false;
}

void 
CBluetoothHC05::_foldbackDesiredTemp()
{
  StaticJsonBuffer<32> jsonBuffer;               // create a JSON buffer on the stack
  JsonObject& root = jsonBuffer.createObject();   // create object to add JSON commands to

	if(foldbackModerator.addJson("TempDesired", CDemandManager::getDemand(), root)) { 
    char opStr[32];
		root.printTo(opStr);
    send(opStr);
  }
}

void 
CBluetoothHC05::_flush()
{
  while(HC05_SerialPort.available())  
    HC05_SerialPort.read();
}

bool
CBluetoothHC05::test(char val) 
{
  DebugPort.enable(true);

  if(!val) {
    _bTest = false;
  }
  else {
    _bTest = true;
    if(val == 0xff) {  // special entry command
      DebugPort.println("ENTERING Test Bluetooth mode");
    }
    else if(val == ('b' & 0x1f)) {   // CTRL-B - leave bluetooth test mode
      DebugPort.println("LEAVING Test Bluetooth mode");
      digitalWrite(_keyPin, LOW);              // request HC-05 module to enter command mode
      _openSerial(9600); 
      _bTest = false;
    }
    else if(val == ('c' & 0x1f)) {   // CTRL-C - data mode
      DebugPort.println("Test Bluetooth COMMAND mode");
      digitalWrite(_keyPin, HIGH);              // request HC-05 module to enter command mode
      _openSerial(9600); 
    }
    else if(val == ('d' & 0x1f)) {   // CTRL-D - data mode
      DebugPort.println("Test Bluetooth DATA mode");
      digitalWrite(_keyPin, LOW);              // request HC-05 module to enter command mode
      _openSerial(9600); 
    }
    else {
      HC05_SerialPort.write(val);
    }
  }

  if(_bTest)
    DebugPort.enable(false);
    
  return _bTest;
}

void 
CBluetoothHC05::_decodeMACresponse(char* pResponse, int len)
{
  // decode ADDR response from a HC-05 
  // NOTE:
  //   the full complement of digits may not be sent!
  //   leading zeroes are suppressed, digits are grouped by colons.
  //
  // eg, HC-05 response: +ADDR:18:e5:449a7
  //
  // 00:18:e5:04:49:a7 is how we'd normally expect to present it!

	char stage[16];
	char MACdecode[16];
	memset(MACdecode, 0, 16);
	char* pDecode = MACdecode;   // extract and build digits into MACdecode using this ptr 
	char* pStage = stage;
	int hexCount = 0;

	for (int i = 6; i <= len; i++) {   // skip initial response header
		if (pResponse[i] == ':' || i == len) {
			if (hexCount & 0x01) {         // leading zeros are suppressed in response, replace them!
				*pDecode++ = '0';
			}
			pStage = stage;
			while (hexCount) {
				*pDecode++ = *pStage++;
				hexCount--;
			}
			pStage = stage;
		}
		if (isxdigit(pResponse[i])) {
			*pStage++ = pResponse[i];;
			hexCount++;
		}
	}

	// ideally 12 characters in MAC digit sequence..
	int deficit = 12 - strlen(MACdecode);
	if (deficit > 0) {
    // not enough, shuffle to rear
		char* pSrc = &MACdecode[strlen(MACdecode) - 1];
		char* pDest = &MACdecode[11];
		int loop = strlen(MACdecode);
		// move from back forward
		while (loop--) {
			*pDest-- = *pSrc--;
		}
    // now insert 0's at start
		pDest = MACdecode;
		while (deficit--) {
			*pDest++ = '0';
		}
    deficit = 0;
	}
	if (deficit < 0) {  // more than 12 digits! - WHOA!
		strcpy(_MAC, "unknown");
    // _bGotMAC = false;
    _bGotMAC = true;
	}
	else {
    // build final colon separated MAC address
		char* pDest = _MAC;        
		char* pSrc = MACdecode;
		for (int i = 0; i < 6; i++) {
			*pDest++ = toupper(*pSrc++);
			*pDest++ = toupper(*pSrc++);
			*pDest++ = ':';
		}
		*--pDest = 0;  // step back and replace last colon with the null terminator!
    _bGotMAC = true;
	}
}

const char*
CBluetoothHC05::getMAC()
{
  if(!_bGotMAC) {
    DebugPort.print("  Getting MAC address...");
    _setCommandMode(true);
    int len = 32;
    char response[32];
    if(!ATResponse("AT+ADDR?\r\n", "+ADDR:", response, len)) {
      DebugPort.println("FAILED");
    }
    else {
      DebugPort.println("OK");
      _decodeMACresponse(response, len);
      DebugPort.print("    "); DebugPort.println(_MAC);
    }
    _setCommandMode(false);
  }
  return _MAC;
}

void
CBluetoothHC05::_setCommandMode(bool commandMode) 
{
  if(commandMode) {
    digitalWrite(_keyPin, HIGH);              // request HC-05 module to enter command mode
    delay(50);
    _openSerial(BTRates[_BTbaudIdx]);      // open serial port at a std. baud rate
  }
  else {
    digitalWrite(_keyPin, LOW);              // request HC-05 module to enter data mode
    delay(50);
    _openSerial(9600); // virtual function, may call derived class method here
  }
}


#endif 
