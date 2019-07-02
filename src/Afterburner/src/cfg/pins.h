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
#include <driver/adc.h>

const uint8_t UART_Tx = 1;
const uint8_t LED_Pin = 2;
const uint8_t UART_Rx = 3;
const uint8_t HC05_KeyPin = 4;
const uint8_t TxEnbPin = 5;
const uint8_t Tx433MHz_pin = 12;      // HSPI std pins
const uint8_t Rx433MHz_pin = 13;      //  "
const uint8_t GPIOout2_pin = 14;      //  "
const uint8_t DS18B20_Pin = 15; 
const uint8_t Rx1Pin = 16;
const uint8_t Tx1Pin = 17;
const uint8_t Tx2Pin = 18;
const uint8_t Rx2Pin = 19;
const uint8_t OLED_SDA_pin = 21;     // I2C std pins
const uint8_t OLED_SCL_pin = 22;     //  "
const uint8_t HC05_SensePin = 23;
const uint8_t GPIOin2_pin = 25;
const uint8_t GPIOin1_pinV21V10 = 26;
const adc2_channel_t GPIOalg_pinINVALID = ADC2_CHANNEL_9; // GPIO 26 - Cannot use ADC2 with WiFi enabled!!!
const uint8_t GPIOout1_pin = 27;

const uint8_t keyUp_pin = 32;
const uint8_t GPIOin1_pinV20 = 33;
const adc1_channel_t GPIOalg_pin = ADC1_CHANNEL_5; // GPIO 33 - OK with Wifi, ADC1 channel
const uint8_t keyDown_pin = 34;      // input only, no chip pullup
const uint8_t keyCentre_pin = 35;    // input only, no chip pullup
const uint8_t keyRight_pin = 36;     // input only, no chip pullup
const uint8_t keyLeft_pin = 39;      // input only, no chip pullup

//const uint8_t ListenOnlyPin = 33;    
const uint8_t WiFi_TriggerPin = 0;    // BOOT switch!
