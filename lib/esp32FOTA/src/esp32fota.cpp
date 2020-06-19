/*
   esp32 firmware OTA
   Date: December 2018
   Author: Chris Joyce <https://chrisjoyce911/esp32FOTA>
   Purpose: Perform an OTA update from a bin located on a webserver (HTTP Only)

   Modifications Dec 2019:
   RLJ Added usage of AsyncHTTPrequest to avoid hang issues with flaky internet connections during update poll
   However using  AsyncTCP for the actual binary update causes other issues in the callback realm, 
   so persisting with the original synchronous update method which blocks all user mode code.
   Modifications Mar 2020:
   RLJ Added FreeRTOS queue to separate callbacks from potential system calls - random reboots in some afterburners...
*/

#include <Arduino.h>
#include "esp32fota.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include "../../ArduinoJson/ArduinoJson.h"
#include "../../asyncHTTPrequest/src/asyncHTTPrequest.h"

extern void forceBootInit();

#define USE_QUEUE


esp32FOTA::esp32FOTA(String firwmareType, int firwmareVersion, bool isBeta)
{
  _firwmareType = firwmareType;
  _firwmareVersion = firwmareVersion;
  _bIsBeta = isBeta;
  useDeviceID = false;
//    _endCallback = NULL;
  _queue = xQueueCreate(1, 256);
}

// Utility to extract header value from headers
String 
esp32FOTA::getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}

// OTA Logic
void 
esp32FOTA::execOTA()
{

  WiFiClient client;
  int contentLength = 0;
  bool isValidContentType = false;

  Serial.println("Connecting to: " + String(_host));
  // Connect to Webserver
  if (client.connect(_host.c_str(), _port))
  {
    // Connection Succeed.
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(_bin));

    // Get the contents of the bin file
    client.print(String("GET ") + _bin + " HTTP/1.1\r\n" +
                  "Host: " + _host + "\r\n" +
                  "Cache-Control: no-cache\r\n" +
                  "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0)
    {
      if (millis() - timeout > 5000) 
      {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }

    while (client.available())
    {
      // read line till /n
      String line = client.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      if (!line.length())
      {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1"))
      {
        if (line.indexOf("200") < 0)
        {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: "))
      {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: "))
      {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream")
        {
          isValidContentType = true;
        }
      }
    }
  }
  else
  {
    // Connect to webserver failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(_host) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType)
  {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin)
    {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength)
      {
        Serial.println("Written : " + String(written) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
        // retry??
        // execOTA();
      }

      if ( _onComplete != NULL) {
        if(!_onComplete(contentLength)) {
          Serial.println("ESP32FOTA: OnComplete handler returned false");
          Update.abort();
        }
      }

      if (Update.end())
      {
        Serial.println("OTA done!");
        if (Update.isFinished())
        {
          Serial.println("Update successfully completed. Rebooting.");
          if(_onSuccess != NULL) {
            _onSuccess();
          }
          ESP.restart();
        }
        else
        {
          if(_onFail != NULL) {
            _onFail();
          }
          Serial.println("Update not finished? Something went wrong!");
        }
      }
      else
      {
        if(_onFail != NULL) {
          _onFail();
        }
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      client.flush();
    }
  }
  else
  {
    Serial.println("There was no content in the response");
    client.flush();
  }
}

// Synchronous mode update check - may hang on flakey Internet connections
bool 
esp32FOTA::execHTTPcheck()
{

  String useURL;

  if (useDeviceID)
  {
      // String deviceID = getDeviceID() ;
      useURL = _checkURL + "?id=" + getDeviceID();
  }
  else
  {
      useURL = _checkURL;
  }

  WiFiClient client;
  _port = 80;

  Serial.println("Getting HTTP");
  Serial.println(useURL);
  Serial.println("------");
  if ((WiFi.status() == WL_CONNECTED))
  { //Check the current connection status

    HTTPClient http;

    http.begin(useURL);        //Specify the URL
    int httpCode = http.GET(); //Make the request

    if (httpCode == 200)
    { //Check is a file was returned
      String payload = http.getString();

      int str_len = payload.length() + 1;
      char* JSONMessage = new char[str_len];
      payload.toCharArray(JSONMessage, str_len);

      StaticJsonBuffer<300> JSONBuffer;                         //Memory pool
      JsonObject &parsed = JSONBuffer.parseObject(JSONMessage); //Parse message

      if (!parsed.success())
      { //Check for errors in parsing
        delete[] JSONMessage;
        Serial.println("Parsing failed");
        delay(5000);
        http.end(); //Free the resources
        return false;
      }

      const char *pltype = parsed["type"];
      int plversion = parsed["version"];
      const char *plhost = parsed["host"];
      _port = parsed["port"];
      const char *plbin = parsed["bin"];

      String jshost(plhost);
      String jsbin(plbin);

      _host = jshost;
      _bin = jsbin;

      String fwtype(pltype);

      delete[] JSONMessage;

      if(fwtype == _firwmareType) {
        if(_bIsBeta) {   // if beta version being used, check for equal version number
          if (plversion >= _firwmareVersion) 
          {
            _newVersion = plversion;
            http.end(); //Free the resources
            return true;
          }
        }
        else {
          if (plversion > _firwmareVersion)
          {
            _newVersion = plversion;
            http.end(); //Free the resources
            return true;
          }
        }
      }
      _newVersion = 0;

    }

    else
    {
      Serial.println("Error on HTTP request");
      // return false;
    }

    http.end(); //Free the resources
  }
  return false;
}

String 
esp32FOTA::getDeviceID()
{
  char deviceid[21];
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  sprintf(deviceid, "%" PRIu64, chipid);
  String thisID(deviceid);
  return thisID;
}

/**
 * onProgress, set a callback called during upload, passed directly thru to OTA Updater class
 * @access public 
 * @param {[type]} void (*func)(size_t, size_t)
 */
void 
esp32FOTA::onProgress( std::function<void(size_t, size_t)> func ) {
  Update.onProgress(func);
}


/**
 * onComplete, set a callback after upload is completed, but not yet verified
 * @access public 
 * @param {[type]} void (*func)(void)
 */
void 
esp32FOTA::onComplete( std::function<bool(int)> func ) {
  _onComplete = func;
}

/**
 * onSuccess, set a callback after upload is fully completed and verified
 * @access public 
 * @param {[type]} void (*func)(void)
 */
void 
esp32FOTA::onSuccess( std::function<void()> func ) {
  _onSuccess = func;
}

/**
 * onComplete, set a callback after upload is completed, but not yet verified
 * @access public 
 * @param {[type]} void (*func)(void)
 */
void 
esp32FOTA::onFail( std::function<void()> func ) {
  _onFail = func;
}




// Callback for when AsyncTCP ready state changes
// queue data to be processed later in user loop
void FOTA_PollCallback(void* optParm, asyncHTTPrequest* pRequest, int readyState)
{
  if(readyState == 4) {  // response


#ifdef USE_QUEUE
    esp32FOTA* pFOTA = (esp32FOTA*)optParm;
    pFOTA->queueDLdata(pRequest);
#else
    String JSONinfo(pRequest->responseText());
    Serial.println(JSONinfo);
    Serial.println();
    esp32FOTA* pFOTA = (esp32FOTA*) optParm;
    if(pFOTA) {
      if(pFOTA->decodeResponse(JSONinfo)) {
      }
    }
#endif

  }
  if(readyState == 1) {  // connection established
    pRequest->send();
  }
}

void 
esp32FOTA::setCheckURL(const char* host)
{
  _checkURL = host;
}

void 
esp32FOTA::setupAsync(const char* host) 
{
#ifdef DEBUG_ASYNC_FOTA
  _versionTest.setDebug(true);
#endif
}

// Asynchronous update check - performs more reliably with flakey Internet connections
void 
esp32FOTA::execAsyncHTTPcheck() 
{
  _newVersion = 0;
  if ((WiFi.status() == WL_CONNECTED)) { 
    if(_versionTest.readyState() == 0 || _versionTest.readyState() == 4) {
      Serial.println("Querying firmware update server");
      _versionTest.setTimeout(10);
      _versionTest.onReadyStateChange(FOTA_PollCallback, this);
      _versionTest.onBuildHeaders(NULL);
      _versionTest.onData(NULL);
      _versionTest.open("GET", _checkURL.c_str());
    }
  }
  else {
    Serial.println("Firmware update check skipped = no STA");
  }
}


// wrapper function to allow use of String
bool
esp32FOTA::decodeResponse(String payload) 
{
  int str_len = payload.length() + 1;
  char* JSONMessage = new char[str_len];
  payload.toCharArray(JSONMessage, str_len);
  
  bool retval = decodeResponse(JSONMessage);

  delete[] JSONMessage;

  return retval;
}


bool 
esp32FOTA::decodeResponse(char* resp)
{
  StaticJsonBuffer<300> JSONBuffer;                         //Memory pool
  JsonObject &parsed = JSONBuffer.parseObject(resp); //Parse message

  if (!parsed.success())
  { //Check for errors in parsing
    Serial.println("FOTA Parsing failed\r\n");
    return false;
  }

  // extract from expected JSON fields
  const char *pltype = parsed["type"];   // update type
  int plversion = parsed["version"];     // version number
  const char *plhost = parsed["host"];
  const char *plbin = parsed["bin"];    // filename

  String fwtype(pltype);
  _host = plhost;                       // host that holds new firmware
  _port = parsed["port"];               // port to use
  _bin = plbin;


  if(fwtype == _firwmareType) {
    if(_bIsBeta) {   // if beta version being used, check for equal version number
      if (plversion >= _firwmareVersion) 
      {
        _newVersion = plversion;
        return true;
      }
    }
    else {
      if (plversion > _firwmareVersion)
      {
        _newVersion = plversion;
        return true;
      }
    }
  }
  _newVersion = 0;
  return false;

}

void 
esp32FOTA::queueDLdata(asyncHTTPrequest* pRequest) 
{
  sFOTAqueue entry;

  int len = pRequest->available();
  if(len <= sizeof(sFOTAqueue::data)) {
    entry.len = len;
    pRequest->responseRead(entry.data, len);
    BaseType_t awoken;
    xQueueSendFromISR(_queue, &entry, &awoken);
  }
}

// routine called regularly by the "loop" task - ie not IRQL
// it is not safe to do system things in the AsyncTCP callbacks!
void 
esp32FOTA::process() 
{
  sFOTAqueue entry;
  if(xQueueReceive(_queue, &entry, 0)) {
    int16_t len = entry.len;

    char working[256];
    memcpy(working, entry.data, 255);
    working[len] = 0;
    String JSONinfo(working);
    Serial.println(JSONinfo);
    Serial.println();
    decodeResponse(JSONinfo);
  }
}

