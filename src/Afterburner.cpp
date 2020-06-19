/*
 * This file is part of the "bluetoothheater" distribution 
 * (https://gitlab.com/mrjones.id.au/bluetoothheater) 
 *
 * Copyright (C) 2019  Ray Jones <ray@mrjones.id.au>
 * Copyright (C) 2018  James Clark
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

  /*
  Chinese Heater Half Duplex Serial Data Sending Tool

  Connects to the blue wire of a Chinese heater, which is the half duplex serial link.
  Sends and receives data from hardware serial port 1. 

  Terminology: Tx is to the heater unit, Rx is from the heater unit.
  
  Typical data frame timing on the blue wire is:
  __Tx_Rx____________________________Tx_Rx____________________________Tx_Rx___________
  
  This software can connect to the blue wire in a normal OEM system, detecting the 
  OEM controller and allowing extraction of the data or injecting on/off commands.

  If Pin 21 is grounded on the Due, this simple stream will be reported over Serial and
  no control from the Arduino will be allowed.
  This allows passive sniffing of the blue wire in a normal system.
  
  The binary data is received from the line.
  If it has been > 100ms since the last blue wire activity this indicates a new frame 
  sequence is starting from the OEM controller.
  Synchronise as such then count off the next 24 bytes storing them in the Controller's 
  data array. These bytes are then reported over Serial to the PC in ASCII.

  It is then expected the heater will respond with it's 24 bytes.
  Capture those bytes and store them in the Heater1 data array.
  Once again these bytes are then reported over Serial to the PC in ASCII.

  If no activity is sensed in a second, it is assumed no OEM controller is attached and we
  have full control over the heater.

  Either way we can now inject a message onto the blue wire allowing our custom 
  on/off control.
  We must remain synchronous with an OEM controller if it exists otherwise E-07 
  faults will be caused.

  Typical data frame timing on the blue wire is then:
  __OEMTx_HtrRx__OurTx_HtrRx____________OEMTx_HtrRx__OurTx_HtrRx____________OEMTx_HtrRx__OurTx_HtrRx_________
    
  The second HtrRx to the next OEMTx delay is always > 100ms and is paced by the OEM controller.
  The delay before seeing Heater Rx data after any Tx is usually much less than 10ms.
  But this does rise if new max/min or voltage settings are sent.
  **The heater only ever sends Rx data in response to a data frame from a controller**

  For Bluetooth connectivity, a HC-05 Bluetooth module is attached to Serial2:
  TXD -> Rx2 (pin 17)
  RXD -> Tx2 (pin 16)
  EN(key) -> pin 15
  STATE -> pin 4
  
 
  This code only works with boards that have more than one hardware serial port like Arduino 
  Mega, Due, Zero, ESP32 etc.


  The circuit:
  - a Tx Rx multiplexer is required to combine the Arduino's Tx1 And Rx1 pins onto the blue wire.
  - a Tx Enable signal from pin 22 controls the multiplexer, high for Tx, low for Rx
  - Serial logging software on Serial0 via USB link

  created 23 Sep 2018 by Ray Jones

  This example code is in the public domain.
*/

#include "WiFi/ABMQTT.h"
#include "cfg/BTCConfig.h"
#include "cfg/pins.h"
#include "RTC/Timers.h"
#include "RTC/Clock.h"
#include "RTC/RTCStore.h"
#include "WiFi/BTCWifi.h"
#include "WiFi/BTCWebServer.h"
#include "WiFi/BTCota.h"
#include "Protocol/Protocol.h"
#include "Protocol/TxManage.h"
#include "Protocol/SmartError.h"
#include "Utility/helpers.h" 
#include "Utility/NVStorage.h"
#include "Utility/DebugPort.h"
#include "Utility/macros.h"
#include "Utility/UtilClasses.h"
#include "Utility/BTC_JSON.h"
#include "Utility/BTC_GPIO.h"
#include "Utility/BoardDetect.h"
#include "Utility/FuelGauge.h"
#include "OLED/ScreenManager.h"
#include "OLED/KeyPad.h"
#include "Utility/TempSense.h"
#include "Utility/DataFilter.h"
#include "Utility/HourMeter.h"
#include <rom/rtc.h>
#include <esp_spiffs.h>
#include <SPIFFS.h>
#include <nvs.h>
#include "Utility/MQTTsetup.h"
#include <FreeRTOS.h>
#include "RTC/TimerManager.h"
#include "Utility/GetLine.h"
#include "Utility/DemandManager.h"
#include "Protocol/HeaterManager.h"
#include "Protocol/433MHz.h"
#if USE_TWDT == 1
#include "esp_task_wdt.h"
#endif

// SSID & password now stored in NV storage - these are still the default values.
//#define AP_SSID "Afterburner"
//#define AP_PASSWORD "thereisnospoon"

// #define RX_DATA_TIMOUT 50

const int FirmwareRevision = 40;
const int FirmwareSubRevision = 0;
const int FirmwareMinorRevision = 1;
const char* FirmwareDate = "19 Jun 2020";

/*
 * Macro to check the outputs of TWDT functions and trigger an abort if an
 * incorrect code is returned.
 */
#define TWDT_TIMEOUT_S 15
#define CHECK_ERROR_CODE(returned, expected) ({                        \
            if(returned != expected){                                  \
                printf("TWDT ERROR\n");                                \
                abort();                                               \
            }                                                          \
})


#ifdef ESP32
#include "Bluetooth/BluetoothESP32.h"
#else
#include "Bluetooth/BluetoothHC05.h"
#endif

bool validateFrame(const CProtocol& frame, const char* name);
void checkDisplayUpdate();
void checkDebugCommands();
void manageStopStartMode();
void manageCyclicMode();
void manageFrostMode();
void manageHumidity();
void doStreaming();
void heaterOn();
void heaterOff();
bool HandleMQTTsetup(char rxVal);
void showMainmenu();
bool checkTemperatureSensors();
void checkBlueWireEvents();
void checkUHF();

// DS18B20 temperature sensor support
// Uses the RMT timeslot driver to operate as a one-wire bus
//CBME280Sensor BMESensor;
CTempSense TempSensor;
long lastTemperatureTime;            // used to moderate DS18B20 access
int DS18B20holdoff = 2;

int BoardRevision = 0;
bool bTestBTModule = false;
bool bSetupMQTT = false;
bool bReportStack = false;

unsigned long lastAnimationTime;     // used to sequence updates to LCD for animation

sFilteredData FilteredSamples;
CSmartError SmartError;
CKeyPad KeyPad;
CScreenManager ScreenManager;
ABTelnetSpy DebugPort;

#if USE_JTAG == 0
//CANNOT USE GPIO WITH JTAG DEBUG
CGPIOin GPIOin;
CGPIOout GPIOout;
CGPIOalg GPIOalg;
#endif

CMQTTsetup MQTTmenu;
CSecuritySetup SecurityMenu;

TaskHandle_t handleWatchdogTask = NULL;
// TaskHandle_t handleBlueWireTask = NULL;
// TaskHandle_t handleAltCtrlTask = NULL;
extern TaskHandle_t handleWebServerTask;

// these variables will persist over a soft reboot.
__NOINIT_ATTR float persistentRunTime;
__NOINIT_ATTR float persistentGlowTime;

CFuelGauge FuelGauge; 
CRTC_Store RTC_Store;
CHourMeter* pHourMeter = NULL;

bool bReportBlueWireData = REPORT_RAW_DATA;
bool bReportJSONData = REPORT_JSON_TRANSMIT;
bool bReportRecyleEvents = REPORT_BLUEWIRE_RECYCLES;
bool bReportOEMresync = REPORT_OEM_RESYNC;
bool pair433MHz = false;


bool bUpdateDisplay = false;
bool bHaveWebClient = false;
bool bBTconnected = false;
long BootTime;

////////////////////////////////////////////////////////////////////////////////////////////////////////
//               Bluetooth instantiation
//
#ifdef ESP32

// Bluetooth options for ESP32
#if USE_HC05_BLUETOOTH == 1
CBluetoothESP32HC05 Bluetooth(HC05_KeyPin, HC05_SensePin, Rx2Pin, Tx2Pin); // Instantiate ESP32 using a HC-05
#elif USE_BLE_BLUETOOTH == 1
CBluetoothESP32BLE Bluetooth;              // Instantiate ESP32 BLE server
#elif USE_CLASSIC_BLUETOOTH == 1
CBluetoothESP32Classic Bluetooth;          // Instantiate ESP32 Classic Bluetooth server
#else   // none selected
CBluetoothAbstract Bluetooth;           // default no bluetooth support - empty shell
#endif  

#else   //  !ESP32

// Bluetooth for boards other than ESP32
#if USE_HC05_BLUETOOTH == 1
CBluetoothHC05 Bluetooth(HC05_KeyPin, HC05_SensePin);  // Instantiate a HC-05
#else   // none selected  
CBluetoothAbstract Bluetooth;           // default no bluetooth support - empty shell
#endif  // closing USE_HC05_BLUETOOTH

#endif  // closing ESP32
//
//                 END Bluetooth instantiation
////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup Non Volatile storage
// this is very much hardware dependent, we can use the ESP32's FLASH
//
#ifdef ESP32
CESP32HeaterStorage actualNVstore;
#else
CHeaterStorage actualNVstore;   // dummy, for now
#endif

  // create reference to CHeaterStorage
  // via the magic of polymorphism we can use this to access whatever 
  // storage is required for a specific platform in a uniform way
CHeaterStorage& NVstore = actualNVstore;

//
////////////////////////////////////////////////////////////////////////////////////////////////////////

CBluetoothAbstract& getBluetoothClient() 
{
  return Bluetooth;
}

// char taskMsg[BLUEWIRE_MSGQUEUESIZE];

void checkBlueWireEvents()
{
// collect and report any debug messages from the blue wire task
  // if(BlueWireMsgQueue && xQueueReceive(BlueWireMsgQueue, taskMsg, 0))
  //   DebugPort.print(taskMsg);

  // if(AltCommsTask.getMsgQueue(taskMsg))
  //   DebugPort.print(taskMsg);
  // QueueHandle_t MsgQueue = HeaterManager.getMsgQueue();
  // if(MsgQueue && xQueueReceive(MsgQueue, taskMsg, 0))
  //   DebugPort.print(taskMsg);
  
  // if(AltControllerMsgQueue && xQueueReceive(AltControllerMsgQueue, taskMsg, 0))
  //   DebugPort.print(taskMsg);

  // check for completed data exchange from the blue wire task
  SemaphoreHandle_t handleSemaphore;
  handleSemaphore = HeaterManager.getSemaphore();

  if(handleSemaphore && xSemaphoreTake(handleSemaphore, 0)) {
    updateJSONclients(bReportJSONData);
    updateMQTT();
    NVstore.doSave();   // now is a good time to store to the NV storage, well away from any blue wire activity
  }

  // // collect transmitted heater data from blue wire task
  // if(BlueWireTxQueue && xQueueReceive(BlueWireTxQueue, BlueWireTxData.Data, 0)) {
  // }

  // // collect and process received heater data from blue wire task
  // if(BlueWireRxQueue && xQueueReceive(BlueWireRxQueue, BlueWireRxData.Data, 0)) {
  //   BlueWireData.set(BlueWireRxData, BlueWireTxData);
  //   SmartError.monitor(BlueWireRxData);

  //   updateFilteredData(BlueWireRxData);

  //   FuelGauge.Integrate(BlueWireRxData.getPump_Actual());

  //   if(INBOUNDS(BlueWireRxData.getRunState(), 1, 5)) {  // check for Low Voltage Cutout
  //     SmartError.checkVolts(FilteredSamples.FastipVolts.getValue(), FilteredSamples.FastGlowAmps.getValue());
  //     SmartError.checkfuelUsage();
  //   }

  //   // trap being in state 0 with a heater error - cancel user on memory to avoid unexpected cyclic restarts
  //   if(RTC_Store.getUserStart() && (BlueWireRxData.getRunState() == 0) && (BlueWireRxData.getErrState() > 1)) {
  //     DebugPort.println("Forcing cyclic cancel due to error induced shutdown");
  //     // DebugPort.println("Forcing cyclic cancel due to error induced shutdown");
  //     RTC_Store.setUserStart(false);
  //   }

  //   pHourMeter->monitor(BlueWireRxData);
  // }

  // int AltHeater;
  // if(AltRxQueue && xQueueReceive(AltRxQueue, &AltHeater, 0)) {
  //   decodeAltHeater(AltHeater);
  // }
  // isAltActive();

  HeaterManager.checkMsgEvents();
  HeaterManager.checkTxEvents();
  HeaterManager.checkRxEvents();
}


// callback function for Keypad events.
// must be an absolute function, cannot be a class member due the "this" element!
void parentKeyHandler(uint8_t event) 
{
  ScreenManager.keyHandler(event);   // call into the Screen Manager
}

void interruptReboot()
{     
  ets_printf("%ld Software watchdog reboot......\r\n", millis());
  abort();
  // esp_restart();
}

unsigned long WatchdogTick = -1;
unsigned long JSONWatchdogTick = -1;

void WatchdogTask(void * param)
{
  for(;;) {
    if(WatchdogTick >= 0) {
      if(WatchdogTick == 0) {
        interruptReboot();
      }
      else {
        WatchdogTick--;
      }
    }
    if(JSONWatchdogTick >= 0) {
      if(JSONWatchdogTick == 0) {
        interruptReboot();
      }
      else {
        JSONWatchdogTick--;
      }
    }
    vTaskDelay(10);
  }
}



//**************************************************************************************************
//**                                                                                              **
//**         WORKAROUND for crap ESP32 millis() standard function                                 **
//**                                                                                              **
//**************************************************************************************************
//
// Substitute shitfull ESP32 millis() with a true and proper ms counter
// The standard millis() on ESP32 is actually micros()/1000.
// This wraps every 71.5 minutes in a **very non linear fashion**.
//
// The FreeRTOS Tick Counter however does increment each ms, and rolls naturally past 0 every 49days.
// With this proper linear behaviour you can use valid timeout calcualtions even through wrap around.
// This elegance breaks using the standard library function, leading to many weird and obtuse issues.
//
// *** IMPORTANT ***
//
// You **MUST** use --wrap millis in the linker command, or -Wl,--wrap,millis in the GCC command.
// platformio.ini file for this project defines the latter as a build_flags entry.
//
// The linker will now link to __wrap_millis() instead of millis() for *any* usage of millis().
// Best of all this includes any library usages of millis() :-D
// If you really must call the shitty ESP32 Arduino millis(), you must call __real_millis() 
// from your dubious code ;-) - basically DON'T do this.

extern "C" unsigned long __wrap_millis() {
  return xTaskGetTickCount();
}


void setup() {

  vTaskPrioritySet(NULL, TASK_PRIORITY_ARDUINO);   // elevate normal Arduino loop etc higher than the usual '1'

  // ensure cyclic mode is disabled after power on
  bool bESP32PowerUpInit = false;
  if(rtc_get_reset_reason(0) == 1/* || bForceInit*/) {
    bESP32PowerUpInit = true;
//    bForceInit = false;
  }
  
  // initially, ensure the GPIO outputs are not activated during startup
  // (GPIO2 tends to be one with default chip startup)
#if USE_JTAG == 0
  //CANNOT USE GPIO WITH JTAG DEBUG
  pinMode(GPIOout1_pin, OUTPUT);  
  pinMode(GPIOout2_pin, OUTPUT);  
  digitalWrite(GPIOout1_pin, LOW);
  digitalWrite(GPIOout2_pin, LOW);
#endif

  // initialise TelnetSpy (port 23) as well as Serial to 115200 
  // Serial is the usual USB connection to a PC
  // DO THIS BEFORE WE TRY AND SEND DEBUG INFO!
  
  DebugPort.setWelcomeMsg((char*)(
                          "*************************************************\r\n"
                          "* Connected to BTC heater controller debug port *\r\n"
                          "*************************************************\r\n"
                          ));
  DebugPort.setBufferSize(8192);
  DebugPort.begin(115200);
  DebugPort.println("_______________________________________________________________");

  DebugPort.printf("Getting NVS stats\r\n");

  nvs_stats_t nvs_stats;
  while( nvs_get_stats(NULL, &nvs_stats) == ESP_ERR_NVS_NOT_INITIALIZED);

  DebugPort.printf("Reset reason: core0:%d, core1:%d\r\n", rtc_get_reset_reason(0), rtc_get_reset_reason(0));
//  DebugPort.printf("Previous user ON = %d\r\n", bUserON);   // state flag required for cyclic mode to persist properly after a WD reboot :-)

  // initialise DS18B20 sensor interface
  TempSensor.begin(DS18B20_Pin, 0x76);
  TempSensor.startConvert();  // kick off initial temperature sample


  lastTemperatureTime = millis();
  lastAnimationTime = millis();
  
  BoardRevision = BoardDetect();
  DebugPort.printf("Board revision: V%.1f\r\n", float(BoardRevision) * 0.1);

  DebugPort.printf("ESP32 IDF Version: %s\r\n", esp_get_idf_version());
  DebugPort.printf("NVS:  entries- free=%d used=%d total=%d namespace count=%d\r\n", nvs_stats.free_entries, nvs_stats.used_entries, nvs_stats.total_entries, nvs_stats.namespace_count);

 // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    DebugPort.println("An Error has occurred while mounting SPIFFS");
  }
  else {
    DebugPort.println("Mounted SPIFFS OK");
    DebugPort.printf("SPIFFS usage: %d/%d\r\n", SPIFFS.usedBytes(), SPIFFS.totalBytes());
    DebugPort.println("Listing SPIFFS contents:");
    String report;
    listSPIFFS("/", 2, report);
  }

  NVstore.init();
  NVstore.load();
  
  initJSONMQTTmoderator();   // prevents JSON for MQTT unless requested
  initJSONIPmoderator();   // prevents JSON for IP unless requested
  initJSONTimermoderator();  // prevents JSON for timers unless requested
  initJSONSysModerator();

  
  KeyPad.begin(keyLeft_pin, keyRight_pin, keyCentre_pin, keyUp_pin, keyDown_pin);
  KeyPad.setCallback(parentKeyHandler);

  // Initialize the rtc object
  Clock.begin();

  BootTime = Clock.get().secondstime();
  
  ScreenManager.begin();
  if(Clock.lostPower()) {
    ScreenManager.selectMenu(CScreenManager::BranchMenu, CScreenManager::SetClockUI);
  }

#if USE_WIFI == 1


  if(NVstore.getUserSettings().wifiMode) {
    initWifi();   
#if USE_OTA == 1
    if(NVstore.getUserSettings().enableOTA) {
      initOTA();
    }
#endif // USE_OTA
#if USE_WEBSERVER == 1
    initWebServer();
#endif // USE_WEBSERVER
    initFOTA();
#if USE_MQTT == 1
    mqttInit();
#endif // USE_MQTT
  }

#endif // USE_WIFI

  pinMode(LED_Pin, OUTPUT);               // On board LED indicator
  digitalWrite(LED_Pin, LOW);

  bBTconnected = false;
  Bluetooth.begin();

  setupGPIO(); 

#if USE_TWDT == 1
  DebugPort.println("Initialize TWDT");
  //Initialize or reinitialize TWDT
  CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S, true), ESP_OK);  // invoke panic if WDT kicks
  //Subscribe this task to TWDT, then check if it is subscribed
  CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
  CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);

#else

#if USE_SW_WATCHDOG == 1 && USE_JTAG == 0
  // create a high priority FreeRTOS task as a watchdog monitor
  xTaskCreate(WatchdogTask,
             "watchdogTask",
             1024,
             NULL,
             configMAX_PRIORITIES-1,
             &handleWatchdogTask);
#endif
#endif

  JSONWatchdogTick = -1;
  WatchdogTick = -1;

  FilteredSamples.ipVolts.setRounding(0.1);
  FilteredSamples.GlowAmps.setRounding(0.01);
  FilteredSamples.GlowVolts.setRounding(0.1);
  FilteredSamples.Fan.setRounding(10);
  FilteredSamples.Fan.setAlpha(0.7);
  FilteredSamples.AmbientTemp.reset(-100.0);
  FilteredSamples.FastipVolts.setRounding(0.1);
  FilteredSamples.FastipVolts.setAlpha(0.7);
  FilteredSamples.FastGlowAmps.setRounding(0.01);
  FilteredSamples.FastGlowAmps.setAlpha(0.7);
  
  RTC_Store.begin();
  FuelGauge.init(RTC_Store.getFuelGauge());
  DebugPort.printf("Previous user start = %d\r\n", RTC_Store.getUserStart());   // state flag required for cyclic mode to persist properly after a WD reboot :-)

  pHourMeter = new CHourMeter(persistentRunTime, persistentGlowTime); // persistent vars passed by reference so they can be valid after SW reboots
  pHourMeter->init(bESP32PowerUpInit || RTC_Store.getBootInit());     // ensure persistent memory variable are reset after powerup, or OTA update
  RTC_Store.setBootInit(false);

  // apply saved set points!
  CDemandManager::reload();
  
  // Check for solo DS18B20
  // store it's serial number as the primary sensor
  // This allows seamless standard operation, and marks the iniital sensor 
  // as the primary if another is added later
  OneWireBus_ROMCode romCode;
  TempSensor.getDS18B20().getRomCodeIdx(0, romCode);
  if(TempSensor.getDS18B20().getNumSensors() == 1 && 
     memcmp(NVstore.getHeaterTuning().DS18B20probe[0].romCode.bytes, romCode.bytes, 8) != 0) 
  {   
    sHeaterTuning tuning = NVstore.getHeaterTuning();
    tuning.DS18B20probe[0].romCode = romCode;
    tuning.DS18B20probe[1].romCode = {0};
    tuning.DS18B20probe[2].romCode = {0};
    tuning.DS18B20probe[0].offset = 0;
    NVstore.setHeaterTuning(tuning);
    NVstore.save();

    DebugPort.printf("Saved solo DS18B20 %02X:%02X:%02X:%02X:%02X:%02X to NVstore\r\n",
                      romCode.fields.serial_number[5], 
                      romCode.fields.serial_number[4], 
                      romCode.fields.serial_number[3], 
                      romCode.fields.serial_number[2], 
                      romCode.fields.serial_number[1], 
                      romCode.fields.serial_number[0] 
                    );
  }
  TempSensor.getDS18B20().mapSensor(0, NVstore.getHeaterTuning().DS18B20probe[0].romCode);
  TempSensor.getDS18B20().mapSensor(1, NVstore.getHeaterTuning().DS18B20probe[1].romCode);
  TempSensor.getDS18B20().mapSensor(2, NVstore.getHeaterTuning().DS18B20probe[2].romCode);

  // create task to run heater comms
  // HeaterManager.setHeaterStyle(1);  // alternate blue digit LCD
  // HeaterManager.setHeaterStyle(0);  // conventional blue wire
  if(HeaterManager.detect())
    DebugPort.printf("Detected heater type %d\r\r", HeaterManager.getHeaterStyle());
  else 
    DebugPort.println("UNABLE TO DETECT HEATER???");

  UHFremote.begin(Rx433MHz_pin, RMT_CHANNEL_4);


  delay(1000); // just to hold the splash screeen for while

  ScreenManager.clearDisplay();
}



// main functional loop is based about a state machine approach, waiting for data 
// to appear upon the blue wire, and marshalling into an appropriate receive buffers
// according to the state.

void loop() 
{
  // DebugPort.handle();    // keep telnet spy alive

  feedWatchdog(); // feed watchdog
      
  doStreaming();   // do wifi, BT tx etc 

  Clock.update();

  if(checkTemperatureSensors())
    ScreenManager.reqUpdate();

  checkDisplayUpdate();    

  checkBlueWireEvents();

  checkUHF();

  vTaskDelay(1);
}  // loop


bool checkTemperatureSensors()
{
  long tDelta = millis() - lastTemperatureTime;
  if(tDelta > MIN_TEMPERATURE_INTERVAL) {  // maintain a minimum holdoff period
    lastTemperatureTime = millis();    // reset time to observe temeprature        

    if(bReportStack) {
      DebugPort.println("Stack high water marks");
      DebugPort.printf("  Arduino: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
      const TaskHandle_t  handleCommsTask = HeaterManager.getTaskHandle();
      if(handleCommsTask) DebugPort.printf("  Heater Comms: %d\r\n", uxTaskGetStackHighWaterMark(handleCommsTask));
      if(handleWatchdogTask) DebugPort.printf("  Watchdog: %d\r\n", uxTaskGetStackHighWaterMark(handleWatchdogTask));
      if(handleWebServerTask) DebugPort.printf("  SSL loop: %d\r\n", uxTaskGetStackHighWaterMark(handleWebServerTask));
    }

    TempSensor.readSensors();

    float fTemperature;
    if(TempSensor.getTemperature(0, fTemperature)) {  // get Primary sensor temperature
      if(DS18B20holdoff) {
        DS18B20holdoff--; 
        DebugPort.printf("Skipped initial DS18B20 reading: %f\r\n", fTemperature);
      }                           // first value upon sensor connect is bad
      else {
        // exponential mean to stabilse readings
        FilteredSamples.AmbientTemp.update(fTemperature);

        manageCyclicMode();
        manageFrostMode();
        manageHumidity();
        manageStopStartMode();
      }
    }
    else {
      DS18B20holdoff = 3;
      FilteredSamples.AmbientTemp.reset(-100.0);
    }

    TempSensor.startConvert();  // request a new conversion, will be ready by the time we loop back around

    return true;
  }
  return false;
}

void manageStopStartMode()
{
  if(NVstore.getUserSettings().ThermostatMethod == 4 && RTC_Store.getUserStart() ) {
    float deltaT = getTemperatureSensor() - CDemandManager::getDegC(); 
    float thresh = NVstore.getUserSettings().ThermostatWindow/2;
    // int heaterState = getHeaterInfo().getRunState();   // native heater state
    int heaterState = HeaterManager.getRunState();   // native heater state
    if(deltaT > thresh) {
      if(heaterState > 0 && heaterState <= 5) {  
        DebugPort.printf("STOP START MODE: Stopping heater, deltaT > +%.1f\r\n", thresh);
        heaterOff();    // over temp - request heater stop
      }
    }
    if(deltaT < -thresh) {
      if(heaterState == 0) {
        DebugPort.printf("STOP START MODE: Restarting heater, deltaT <%.1f\r\n", thresh);
        heaterOn();  // under temp, start heater again
      }
    }
  }
}

void manageCyclicMode()
{
  const sCyclicThermostat& cyclic = NVstore.getUserSettings().cyclic;
  if(cyclic.Stop && RTC_Store.getUserStart()) {   // cyclic mode enabled, and user has started heater
    int stopDeltaT = cyclic.Stop + 1;  // bump up by 1 degree - no point invoking at 1 deg over!
    float deltaT = getTemperatureSensor() - CDemandManager::getDegC(); 
//    DebugPort.printf("Cyclic=%d bUserOn=%d deltaT=%d\r\n", cyclic, bUserON, deltaT);

    // ensure we cancel user ON mode if heater throws an error
    // int errState = getHeaterInfo().getErrState();
    int errState = HeaterManager.getErrState();
    if((errState > 1) && (errState < 12) && (errState != 8)) {
      // excludes errors 0,1(OK), 12(E1-11,Retry) & 8(E-07,Comms Error)
      DebugPort.println("CYCLIC MODE: cancelling user ON status"); 
      requestOff();   // forcibly cancel cyclic operation - pretend user pressed OFF
    }
    // int heaterState = getHeaterInfo().getRunState();
    int heaterState = HeaterManager.getRunState();
    // check if over temp, turn off heater
    if(deltaT > stopDeltaT) {
      if(heaterState > 0 && heaterState <= 5) {  
        DebugPort.printf("CYCLIC MODE: Stopping heater, deltaT > +%d\r\n", stopDeltaT);
        heaterOff();    // over temp - request heater stop
      }
    }
    // check if under temp, turn on heater
    if(deltaT < cyclic.Start) {
      // typ. 1 degree below set point - restart heater
      if(heaterState == 0) {
        DebugPort.printf("CYCLIC MODE: Restarting heater, deltaT <%d\r\n", cyclic.Start);
        heaterOn();
      }
    }
  }
}


void manageFrostMode()
{
  uint8_t engage = NVstore.getUserSettings().FrostOn;
  if(engage) {
    float deltaT = getTemperatureSensor() - engage;
    // int heaterState = getHeaterInfo().getRunState();
    int heaterState = HeaterManager.getRunState();
    if(deltaT < 0) {
      if(heaterState == 0) {
        RTC_Store.setFrostOn(true);        
        DebugPort.printf("FROST MODE: Starting heater, < %d`C\r\n", engage);
        if(NVstore.getUserSettings().FrostRise == 0)
          RTC_Store.setUserStart(true);    // enable cyclic mode if user stop
        heaterOn();
      }
    }
    uint8_t rise = NVstore.getUserSettings().FrostRise;
    if(rise && (deltaT > rise)) {  // if rise is set to 0, user must shut off heater
      if(RTC_Store.getFrostOn()) {
        DebugPort.printf("FROST MODE: Stopping heater, > %d`C\r\n", engage+rise);
        heaterOff();
        RTC_Store.setFrostOn(false);  // cancel active frost mode
        RTC_Store.setUserStart(false);   // for cyclic mode
      }
    }
  }
}

void manageHumidity()
{
  uint8_t humidity = NVstore.getUserSettings().humidityStart;
  if(humidity) {
    float reading;
    if(getTempSensor().getHumidity(reading)) {
      uint8_t testval = (uint8_t)reading;
      if(testval > humidity) {
        DebugPort.printf("HUMIDITY MODE: Starting heater, > %d%%\r\n", humidity);
        requestOn();
      }
    }
  }
}


CDemandManager::eStartCode 
requestOn()
{
  bool altHeater = HeaterManager.getHeaterStyle() == 1;

  if(HeaterManager.getRunState())  // already running?
    return CDemandManager::eStartOK;

  DebugPort.println("Start Request!");

  bool fuelOK = 2 != SmartError.checkfuelUsage() || altHeater;
  if(!fuelOK) {
    DebugPort.println("Start denied - Low fuel");
    return CDemandManager::eStartLowFuel;
  }
  bool LVCOK = 2 != SmartError.checkVolts(FilteredSamples.FastipVolts.getValue(), FilteredSamples.FastGlowAmps.getValue());
  if((hasHtrData() || altHeater) && LVCOK) {
    RTC_Store.setUserStart(true);    // for cyclic mode
    RTC_Store.setFrostOn(false);         // cancel frost mode
    // only start if below appropriate temperature threshold, raised for cyclic mode
    // int denied = checkStartTemp();
    CDemandManager::eStartCode startCode = CDemandManager::checkStart();
    if(startCode == CDemandManager::eStartOK) {
      heaterOn();
    }
    else {
      if(startCode == CDemandManager::eStartSuspend) {
        SmartError.inhibit(true);  // ensure our suspend does not get immediately cancelled by prior error sitting in system!
        DebugPort.printf("CYCLIC MODE: Skipping directly to suspend, deltaT > +%d\r\n", NVstore.getUserSettings().cyclic.Stop+1);
        heaterOff();    // over temp - request heater stop
      }
    }
    return startCode;
  }
  else {
    DebugPort.println("Start denied - LVC");
    return CDemandManager::eStartLVC;   // LVC
  }
}

void requestOff()
{
  if(HeaterManager.getRunState()) {  // heater running?
    DebugPort.println("Stop Request!");
    heaterOff();
    RTC_Store.setUserStart(false);   // for cyclic mode
    RTC_Store.setFrostOn(false);  // cancel active frost mode
    CTimerManager::cancelActiveTimer();
  }
}

void heaterOn() 
{
  HeaterManager.reqOnOff(true);
}

void heaterOff()
{
  HeaterManager.reqOnOff(false);
}


void checkDisplayUpdate()
{
  // only update OLED when not processing blue wire
  if(ScreenManager.checkUpdate()) {
    lastAnimationTime = millis() + 100; 
    ScreenManager.animate();
    ScreenManager.refresh();   // always refresh post major update
  }
 

  long tDelta = millis() - lastAnimationTime;
  if(tDelta >= 100) {
    lastAnimationTime = millis() + 100; 
    if(ScreenManager.animate())
      ScreenManager.refresh();
  }
}

void forceBootInit()
{
  RTC_Store.setBootInit();
}


float getTemperatureSensor(int source)
{
  float retval;
  TempSensor.getTemperature(source, retval);
  return retval;

}


bool isWebClientConnected()
{
  return bHaveWebClient;
}

void checkDebugCommands()
{
  static CGetLine line;

  // check for test commands received over Debug serial port or telnet
  char rxVal;
  if(DebugPort.getch(rxVal)) {

#ifdef PROTOCOL_INVESTIGATION    
    static int mode = 0;6
    static int val = 0;
#endif

    if(bTestBTModule) {
      bTestBTModule = Bluetooth.test(rxVal);
      return;
    }
    if(MQTTmenu.Handle(rxVal)) {
      if(rxVal == 0) {
        showMainmenu();
      }
      return;
    }
    if(SecurityMenu.Handle(rxVal)) {
      if(rxVal == 0) {
        showMainmenu();
      }
      return;
    }

    rxVal = toLowerCase(rxVal);

#ifdef PROTOCOL_INVESTIGATION    
    bool bSendVal = false;
#endif
    if(rxVal == '\n') {    // "End of Line"
#ifdef PROTOCOL_INVESTIGATION    
      String convert(line.getString());
      val = convert.toInt();
      bSendVal = true;
      line.reset();
#endif
    }
    else {
      if(rxVal == ' ') {   // SPACE to bring up menu
        showMainmenu();
      }
#ifdef PROTOCOL_INVESTIGATION    
      else if(isDigit(rxVal)) {
        line.handle(rxVal);
      }
      else if(rxVal == 'p') {
        DebugPort.println("Test Priming Byte... ");
        mode = 1;
      }
      else if(rxVal == 'g') {
        DebugPort.println("Test glow power byte... ");
        mode = 2;
      }
      else if(rxVal == 'i') {
        DebugPort.println("Test unknown bytes MSB");
        mode = 3;
      }
      else if(rxVal == 'a') {
        DebugPort.println("Test unknown bytes LSB");
        mode = 5;
      }
      else if(rxVal == 'c') {
        DebugPort.println("Test Command Byte... ");
        mode = 4;
      }
      else if(rxVal == 'x') {
        DebugPort.println("Special mode cancelled");
        val = 0;
        mode = 0;
        DefaultBTCParams.Controller.Command = 0;
      }
      else if(rxVal == ']') { 
        val++;
        bSendVal = true;
      }
      else if(rxVal == '[') {
        val--;
        bSendVal = true;
      }
#endif
      else if(rxVal == 'b') { 
        bReportBlueWireData = !bReportBlueWireData;
        DebugPort.printf("Toggled raw blue wire data reporting %s\r\n", bReportBlueWireData ? "ON" : "OFF");
      }
      else if(rxVal == 'j')  {
        bReportJSONData = !bReportJSONData;
        DebugPort.printf("Toggled JSON data reporting %s\r\n", bReportJSONData ? "ON" : "OFF");
      }
      else if(rxVal == ('w' & 0x1f))  {
        bReportRecyleEvents = !bReportRecyleEvents;
        if(NVstore.getUserSettings().menuMode == 2)
          bReportRecyleEvents = false;
        DebugPort.printf("Toggled blue wire recycling event reporting %s\r\n", bReportRecyleEvents ? "ON" : "OFF");
      }
      else if(rxVal == 'm') {
        MQTTmenu.setActive();
      }
      else if(rxVal == 's') {
        SecurityMenu.setActive();
      }
      else if(rxVal == ('o' & 0x1f))  {
        bReportOEMresync = !bReportOEMresync;
        DebugPort.printf("Toggled OEM resync event reporting %s\r\n", bReportOEMresync ? "ON" : "OFF");
      }
      else if(rxVal == ('c' & 0x1f)) {
        CommState.toggleReporting();
      }
      else if(rxVal == '+') {
        TxManage.queueOnRequest();
      }
      else if(rxVal == '-') {
        TxManage.queueOffRequest();
      }
      else if(rxVal == 'h') {
        getWebContent(true);
      }
      else if(rxVal == '!') {
        DebugPort.println("Invoking deliberate halt loop");
        for(;;);    // force watchdog reboot
      }
      else if(rxVal == ('b' & 0x1f)) {   // CTRL-B Tst Mode: bluetooth module route
        bTestBTModule = !bTestBTModule;
        Bluetooth.test(bTestBTModule ? 0xff : 0x00);  // special enter or leave BT test commands
      }
      else if(rxVal == ('h' & 0x1f)) {   // CTRL-H hourmeter reset
        pHourMeter->resetHard();
      }
      else if(rxVal == ('p' & 0x1f)) {   // CTRL-P fuel usage reset 
        FuelGauge.reset();
      }
      else if(rxVal == ('r' & 0x1f)) {   // CTRL-R reboot
        ESP.restart();            // reset the esp
      }
      else if(rxVal == ('s' & 0x1f)) {   // CTRL-S Test Mode: bluetooth module route
        bReportStack = !bReportStack;
      }
    }
#ifdef PROTOCOL_INVESTIGATION    
    if(bSendVal) {
      switch(mode) {
        case 1:
          DefaultBTCParams.Controller.Prime = val & 0xff;     // always  0x32:Thermostat, 0xCD:Fixed
          break;
        case 2:
          DefaultBTCParams.Controller.GlowDrive = val & 0xff;     // always 0x05
          break;
        case 3:
          DefaultBTCParams.Controller.Unknown1_MSB = val & 0xff;     
          break;
        case 4:
          DebugPort.printf("Forced controller command = %d\r\n", val&0xff);
          DefaultBTCParams.Controller.Command = val & 0xff;
          break;
        case 5:
          DefaultBTCParams.Controller.Unknown1_LSB = val & 0xff;     
          break;
      }
    }
#endif
  }
}


int getSmartError()
{
  return SmartError.getError();
}

bool isCyclicStopStartActive()
{
  return RTC_Store.getUserStart() && (NVstore.getUserSettings().cyclic.isEnabled() || NVstore.getUserSettings().ThermostatMethod == 4);
}

void setupGPIO()
{
#if USE_JTAG == 1
  //CANNOT USE GPIO WITH JTAG DEBUG
  return;
#else
  if(BoardRevision == 10 || BoardRevision == 20 || BoardRevision == 21 || BoardRevision == 30) {
    // some special considerations for GPIO inputs, depending upon PCB hardware
    // V1.0 PCBs only expose bare inputs, which are pulled high. Active state into ESP32 is LOW. 
    // V2.0+ PCBs use an input transistor buffer. Active state into ESP32 is HIGH (inverted).
    int activePinState = (BoardRevision == 10) ? LOW : HIGH;  
    int Input1 = BoardRevision == 20 ? GPIOin1_pinV20 : GPIOin1_pinV21V10;
    GPIOin.begin(Input1, 
                 GPIOin2_pin, 
                 NVstore.getUserSettings().GPIO.in1Mode, 
                 NVstore.getUserSettings().GPIO.in2Mode, 
                 activePinState);

    // GPIO out is always active high from ESP32
    // V1.0 PCBs only expose the bare pins
    // V2.0+ PCBs provide an open collector output that conducts when active
    GPIOout.begin(GPIOout1_pin, 
                  GPIOout2_pin, 
                  NVstore.getUserSettings().GPIO.out1Mode, 
                  NVstore.getUserSettings().GPIO.out2Mode);
    GPIOout.setThresh(NVstore.getUserSettings().GPIO.thresh[0], 
                      NVstore.getUserSettings().GPIO.thresh[1]);

    // ### MAJOR ISSUE WITH ADC INPUTS ###
    //
    // V2.0 PCBs that have not been modified connect the analogue input to GPIO26.
    // This is ADC2 channel (#9). 
    // Unfortunately it was subsequently discovered that any ADC2 input cannot be 
    // used if Wifi is enabled. 
    // THIS ISSUE IS NOT RESOLVABLE IN SOFTWARE.
    // *** It is not possible to use ANY of the 10 ADC2 channels if Wifi is enabled :-( ***
    //
    // Fix is to cut traces to GPIO33 & GPIO26 and swap the connections.
    // This directs GPIO input1 into GPIO26 and the analogue input into GPIO33 (ADC1_CHANNEL_5)
    // This will be properly fixed in V2.1 PCBs
    //
    // As V1.0 PCBS expose the bare pins, the correct GPIO33 input can be readily chosen.
    CGPIOalg::Modes algMode = NVstore.getUserSettings().GPIO.algMode;
    if(BoardRevision == 20)  
      algMode = CGPIOalg::Disabled;      // force off analogue support in V2.0 PCBs
    GPIOalg.begin(GPIOalg_pin, algMode);
  }
  else {
    // unknown board or forced no GPIO by grounding pin26 - deny all GPIO operation 
    // set all pins as inputs with pull ups
    pinMode(GPIOin2_pin, INPUT_PULLUP);
    pinMode(GPIOin1_pinV21V10, INPUT_PULLUP);
    pinMode(GPIOin1_pinV20, INPUT_PULLUP);
    pinMode(GPIOout1_pin, INPUT_PULLUP);
    pinMode(GPIOout2_pin, INPUT_PULLUP);
    GPIOin.begin(0, 0, CGPIOin1::Disabled, CGPIOin2::Disabled, LOW);            // ensure modes disabled (should already be by constructors)
    GPIOout.begin(0, 0, CGPIOout1::Disabled, CGPIOout2::Disabled);
    GPIOalg.begin(ADC1_CHANNEL_5, CGPIOalg::Disabled);
  }
#endif
}

bool toggleGPIOout(int channel) 
{
#if USE_JTAG == 0
  //CANNOT USE GPIO WITH JTAG DEBUG
  if(channel == 0) {
    if(NVstore.getUserSettings().GPIO.out1Mode == CGPIOout1::User) {
      setGPIOout(channel, !getGPIOout(channel));  // toggle selected GPIO output 
      return true;
    }
  }
  else if(channel == 1) {
    if(NVstore.getUserSettings().GPIO.out2Mode == CGPIOout2::User) {
      setGPIOout(channel, !getGPIOout(channel));  // toggle selected GPIO output 
      return true;
    }
  }
#endif
  return false;
}

bool setGPIOout(int channel, bool state)
{
#if USE_JTAG == 0
  //CANNOT USE GPIO WITH JTAG DEBUG
  if(channel == 0) {
    if(GPIOout.getMode1() != CGPIOout1::Disabled) {
      DebugPort.printf("setGPIO: Output #%d = %d\r\n", channel+1, state);
      GPIOout.setState(channel, state);
      return true;
    }
  }
  else if(channel == 1) {
    if(GPIOout.getMode2() != CGPIOout2::Disabled) {
      DebugPort.printf("setGPIO: Output #%d = %d\r\n", channel+1, state);
      GPIOout.setState(channel, state);
      return true;
    }
  }
#endif
  return false;
}

bool getGPIOout(int channel)
{
#if USE_JTAG == 0
  bool retval = GPIOout.getState(channel);
  DebugPort.printf("getGPIO: Output #%d = %d\r\n", channel+1, retval);
  return retval;
#else
  //CANNOT USE GPIO WITH JTAG DEBUG
  return false;
#endif
}

float getVersion(bool betarevision)
{
  if(betarevision)
    return float(FirmwareMinorRevision);
  else
    return float(FirmwareRevision) * 0.1f + float(FirmwareSubRevision) * .001f;
}

const char* getVersionStr(bool beta) {
  static char vStr[32];
  if(beta) {
    if(FirmwareMinorRevision)
      return "BETA";
    else
      return "";
  }
  else {
    if(FirmwareMinorRevision)
      sprintf(vStr, "V%.1f.%d.%d", float(FirmwareRevision) * 0.1f, FirmwareSubRevision, FirmwareMinorRevision);
    else
      sprintf(vStr, "V%.1f.%d", float(FirmwareRevision) * 0.1f, FirmwareSubRevision);
  }
  return vStr;
}

const char* getVersionDate()
{
  return FirmwareDate;
}

int getBoardRevision()
{
  return BoardRevision;
}

void ShowOTAScreen(int percent, eOTAmodes updateType)
{
  ScreenManager.showOTAMessage(percent, updateType);
}

void feedWatchdog()
{
#if USE_TWDT == 1  
  CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);  //Comment this line to trigger a TWDT timeout
#else

#if USE_SW_WATCHDOG == 1 && USE_JTAG == 0
    // BEST NOT USE WATCHDOG WITH JTAG DEBUG :-)
  // DebugPort.printf("\r %ld Watchdog fed", millis());
  // DebugPort.print("~");
  WatchdogTick = 1500;
#else
  WatchdogTick = -1;
#endif
#endif
}

void doJSONwatchdog(int topup)
{
 if(topup) {
   JSONWatchdogTick = topup * 100;
 }
 else {
   JSONWatchdogTick = -1;
 }
}


void doStreaming() 
{
#if USE_WIFI == 1

  if(NVstore.getUserSettings().wifiMode) {
    doWiFiManager();
#if USE_OTA == 1
    doOTA();
#endif // USE_OTA 
#if USE_WEBSERVER == 1
    bHaveWebClient = doWebServer();
#endif //USE_WEBSERVER
#if USE_MQTT == 1
    // most MQTT is managed via callbacks, but need some sundry housekeeping
    doMQTT();
#endif
  }

#endif // USE_WIFI

  checkDebugCommands();

  KeyPad.update();      // scan keypad - key presses handler via callback functions!

#if USE_JTAG == 0
#if DBG_FREERTOS == 0
  //CANNOT USE GPIO WITH JTAG DEBUG
  GPIOin.manage();
  GPIOout.manage(); 
  GPIOalg.manage();
#endif
#endif

  Bluetooth.check();    // check for Bluetooth activity

  // manage changes in Bluetooth connection status
  if(Bluetooth.isConnected()) {
    if(!bBTconnected) {
      resetAllJSONmoderators();  // force full send upon BT client connect
    }
    bBTconnected = true;
  }
  else {
    bBTconnected = false;
  }
  // manage changes in number of wifi clients
  if(isWebSocketClientChange()) {
    resetAllJSONmoderators();  // force full send upon increase of Wifi clients
  }

  DebugPort.handle();    // keep telnet spy alive

}

void getGPIOinfo(sGPIO& info)
{
#if USE_JTAG == 0
  info.inState[0] = GPIOin.getState(0);
  info.inState[1] = GPIOin.getState(1);
  info.outState[0] = GPIOout.getState(0);
  info.outState[1] = GPIOout.getState(1);
  info.algVal = GPIOalg.getValue();
  info.in1Mode = GPIOin.getMode1();
  info.in2Mode = GPIOin.getMode2();
  info.out1Mode = GPIOout.getMode1();
  info.out2Mode = GPIOout.getMode2();
  info.algMode = GPIOalg.getMode();
#endif
}

// hook for JSON input, simulating a GPIO key press
void simulateGPIOin(uint8_t newKey)   
{
#if USE_JTAG == 0
  GPIOin.simulateKey(newKey);
#endif
}

float getBatteryVoltage(bool fast)
{
#ifdef RAW_SAMPLES
  return getHeaterInfo().getBattVoltage();
#else
  if(fast)
    return FilteredSamples.FastipVolts.getValue();
  else
    return FilteredSamples.ipVolts.getValue();
#endif
}

float getGlowVolts()
{
#ifdef RAW_SAMPLES
	return  getHeaterInfo().getGlow_Voltage();
#else
  return FilteredSamples.GlowVolts.getValue();
#endif
}

float getGlowCurrent()
{
#ifdef RAW_SAMPLES
	return getHeaterInfo().getGlow_Current();
#else
  return FilteredSamples.GlowAmps.getValue();
#endif
}

int getFanSpeed()
{
  return HeaterManager.getFanRPM();
/*#ifdef RAW_SAMPLES
	return getHeaterInfo().getFan_Actual();
#else
  return (int)FilteredSamples.Fan.getValue();
#endif*/
}

int sysUptime()
{
  return Clock.get().secondstime() - BootTime;
}

void resetFuelGauge()
{
  FuelGauge.reset();
}

void setName(const char* name, int type)
{
  sCredentials creds = NVstore.getCredentials();
  char* pDest = NULL;
  switch (type) {
    case 0: pDest = creds.APSSID; break;
    case 1: pDest = creds.webUsername; break;
    case 2: pDest = creds.webUpdateUsername; break;
  }
  if(pDest) {
    strncpy(pDest, name, 31);
    pDest[31] = 0;
  }
  NVstore.setCredentials(creds);
  NVstore.save();
  NVstore.doSave();   // ensure NV storage
  if(type == 0) {
    DebugPort.println("Restarting ESP to invoke new network credentials");
    DebugPort.handle();
    // initiate reboot
    const char* content[2];
    content[0] = "AP reconfig reset";
    content[1] = "initiated";
    ScreenManager.showRebootMsg(content, 1000);
  // delay(1000);
  //   ESP.restart();
  }
}

void setPassword(const char* name, int type)
{
  sCredentials creds = NVstore.getCredentials();
  char* pDest = NULL;
  switch (type) {
    case 0: pDest = creds.APpassword; break;
    case 1: pDest = creds.webPassword; break;
    case 2: pDest = creds.webUpdatePassword; break;
  }
  if(pDest) {
    strncpy(pDest, name, 31);
    pDest[31] = 0;
  }
  NVstore.setCredentials(creds);
  NVstore.save();
  NVstore.doSave();   // ensure NV storage
  if(type == 0) {
    DebugPort.println("Restarting ESP to invoke new network credentials");
    DebugPort.handle();
    // initate reboot
    const char* content[2];
    content[0] = "AP password";
    content[1] = "changed";
    ScreenManager.showRebootMsg(content, 1000);
    // delay(1000);
    // ESP.restart();
  }
}


void showMainmenu()
{
  DebugPort.print("\014");
  DebugPort.println("MENU options");
  DebugPort.println("");
  DebugPort.printf("  <B> - toggle raw blue wire data reporting, currently %s\r\n", bReportBlueWireData ? "ON" : "OFF");
  DebugPort.printf("  <J> - toggle output JSON reporting, currently %s\r\n", bReportJSONData ? "ON" : "OFF");
  DebugPort.println("  <M> - configure MQTT");
  DebugPort.println("  <S> - configure Security");
  DebugPort.println("  <+> - request heater turns ON");
  DebugPort.println("  <-> - request heater turns OFF");
  DebugPort.println("  <CTRL-R> - restart the ESP");
  DebugPort.printf("  <CTRL-C> - toggle reporting of state machine transits %s\r\n", CommState.isReporting() ? "ON" : "OFF");        
  DebugPort.printf("  <CTRL-O> - toggle reporting of OEM resync event, currently %s\r\n", bReportOEMresync ? "ON" : "OFF");        
  DebugPort.printf("  <CTRL-W> - toggle reporting of blue wire timeout/recycling event, currently %s\r\n", bReportRecyleEvents ? "ON" : "OFF");
  DebugPort.println("");
  DebugPort.println("");
  DebugPort.println("");
  DebugPort.println("");
  DebugPort.println("");
  DebugPort.println("");
  DebugPort.println("");
}

void reloadScreens()
{
  ScreenManager.reqReload();
}

CTempSense& getTempSensor()
{
  return TempSensor;
}

void reqHeaterCalUpdate()
{
  TxManage.queueSysUpdate();
}

// const CProtocolPackage& getHeaterInfo()
// {
//   return BlueWireData;
// }

// int UHFsubcode(int val)
// {
//   val &= 0x03;
//   val = 0x0001 << val;
//   return val;
// }

void checkUHF()
{
  if(!pair433MHz) {
    UHFremote.manage();
    // unsigned long test = 0xF5F0AC10;
    // if(UHFremote.available()) {
    //   unsigned long code;
    //   UHFremote.read(code);
    //   DebugPort.printf("UHF remote code = %08lX\r\n", code);

    //   unsigned long ID = (test >> 8) & 0xfffff0;
    //   if(((code ^ ID) & 0xfffff0) == 0) {
    //     int subCode = code & 0xf;
    //     if(test & 0x800) {
    //       if((UHFsubcode(test >> 6) ^ subCode) == 0xf) {
    //         DebugPort.println("UHF start request!");
    //         HeaterManager.reqOnOff(true);
    //       }
    //     }
    //     if(test & 0x400) {
    //       if((UHFsubcode(test >> 4) ^ subCode) == 0xf) {
    //         DebugPort.println("UHF stop request!");
    //         HeaterManager.reqOnOff(false);
    //       }
    //     }
    //     if(test & 0x200) {
    //       if((UHFsubcode(test >> 2) ^ subCode) == 0xf) {
    //         DebugPort.println("UHF inc temp request!");
    //         CDemandManager::deltaDemand(+1);
    //       }
    //     }
    //     if(test & 0x100) {
    //       if((UHFsubcode(test >> 0) ^ subCode) == 0xf) {
    //         DebugPort.println("UHF dec temp request!");
    //         CDemandManager::deltaDemand(+1);
    //       }
    //     }
    //   }
    // }
  }
}