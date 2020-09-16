# BluetoothHeater

Bluetooth & WiFi interface for Chinese Diesel Heaters

Requirements
--------------------------
* Requires "blue wire" compatible heater unit
* Blue wire interface circuitry - please refer to the hardware design schematics in the wiki for guidance.  
  It is important to take note that the blue wire is 5V logic, an ESP32 is 3.3V.  
  Level translation is important!
* ESP32 dev module
* HC-05 Bluetooth module - preferably exposing the key pin, not power control)
* DS18B20 one wire temperature sensor
* 1.3" I2C OLED using SH1106 controller
* DS3231 Real Time Clock
* Technical ability to solder SMD components
* Technical ability to program/flash in the Arduino environement

Working so far:
--------------------------
* Power On/Off
* Temperature + & -
* Fuel mixture tuning
* One wire remote temperature sensing (DS18B20)
* Heat exchanger body temperature, as reported by heater unit
* I2C Interface to 1.3" SH1106 based OLED for a full featured controller
* 5 button keypad interface
* Smart error detection, observes unusual heater state progression (ignition fail)
* ESP32 with HC-05 Bluetooth
* JSON based communications between Bluetooth and Wifi apps
* Bluetooth Connectivity
* Bluetooth Control App for Android (App Inventor based)
* WiFi Connection to existing network or Standalone Access Point Mode (Passwd: thereisnospoon)
* Wifi control
* DebugPort data sent via Telnet if/when available on the network.
* 14 timers - including selectable day and repeat functionality    
  Simplisticly this allows each day to have 2 individual start stop regimes, but 
  a single timer can be set to repeat every day if desired, or on certain days.
  Timers can also be set to be one-shot.
  This is an extremely flexible system!   
* Battery backed Real Time Clock - DS3231
* Mk1 "Green PCB" using naked ESP32 and HC-05 modules 
* Temperature readout in Celcius or Farenheit
* Two new experimental thermostat modes (No practical testing - presently too hot in .au!):  
    Tighten or loosen the thermostat temperature range by specifying a hysteresis value. eg tell the heater it is 23 degrees when it really is only 22.25 degrees (only 0.25 above set point).  
    Request a linear change in Hz according to the deviation from the setpoint
* 2 external digital inputs, 2 digital outputs, analogue input .This would allow external timer units for example, or analogue temperature demand (which is still only 1 degree resolution with standard heater thermostat). 
* New Mk2 "Red PCB" that properly fits into an off the shelf case (requires machining)
* Software updates over WiFi: 
    Native OTA via espota.exe or espota.py (AP or STA mode)
    Web browser upload new binary to controller (AP or STA mode)
    Direct discovery and download of updates from internet server (STA mode)
* Factory default menu option
* "Fuel gauge" - Integrates pump frequency, assumes a repeatable dose of fuel per pump stroke.
* Low voltage cut out, definable threshold - auto adjusts for cable voltage drop during start
* Temperature probe offset to correct adverse readings.
* Hour meter - run time, glow time
* 433MHz Rx stream, 433MHz Tx stream connections. 
* MQTT pub/sub 
 

To be implemented 
--------------------------
* Regular Hot Burn cycle (DPF mode!)
* list under construction.....

Suggestions
--------------------------
If you have a burning desire for a feature that you think will useful, please feel free to drop me a line at gitlab@mrjones.id.au.  
Your request will be thoughtfully considered.  
If deemed viable and worthwhile to the greater good it will be added to the "To be implemented list" above and worked on in good time.  
Likewise, if it's really easy to implement, it may make it to "Working So Far" at speed :-)

Please be aware that this is not my day job, so "good things take time".

Case for Green PCB
--------------------------
https://www.thingiverse.com/thing:3398068

