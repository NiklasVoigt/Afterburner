/*
   esp32 firmware OTA
   Date: December 2018
   Author: Chris Joyce <https://chrisjoyce911/esp32FOTA>
   Purpose: Perform an OTA update from a bin located on a webserver (HTTP Only)

   Modifications Dec 2019:
   RLJ Added usage of asyncHTTPrequest to avoid hang issues with flaky internet connections during update poll
   However using  AsyncTCP for the actual binary update causes other issues in the callback realm, 
   so persisting with the original synchronous update method which blocks all user mode code.
*/

#ifndef esp32fota_h
#define esp32fota_h

//#include <Arduino.h>
#include <functional>
#include "../../asyncHTTPrequest/src/asyncHTTPrequest.h"
#include "freertos/queue.h"

struct sFOTAqueue{
  uint8_t len;
  uint8_t data[255];
};

class esp32FOTA
{
public:
  esp32FOTA(String firwmareType, int firwmareVersion, bool isBeta);
  void execOTA();
  bool execHTTPcheck();
  bool useDeviceID;
  void onProgress( std::function<void(size_t, size_t)> func );
  void onComplete( std::function<bool(int)> func );
  void onSuccess( std::function<void()> func );
  void onFail( std::function<void()> func );
  int  getNewVersion() { return _newVersion; };

  void setCheckURL(const char* host);
  void setupAsync(const char* host);
  void execAsyncHTTPcheck();
  bool decodeResponse(String payload);
  bool decodeResponse(char* resp);
  void process();
  void queueDLdata(asyncHTTPrequest* request);

private:
  String getHeaderValue(String header, String headerName);
  String getDeviceID();
  String _firwmareType;
  int _firwmareVersion;
  int _newVersion;
  bool _bIsBeta;
  String _checkURL;
  String _host;
  String _bin;
  int _port;
  std::function<bool(int)> _onComplete;
  std::function<void()> _onSuccess;
  std::function<void()> _onFail;
  asyncHTTPrequest _versionTest;
  QueueHandle_t _queue;
  String _pollResponse;

};

#endif