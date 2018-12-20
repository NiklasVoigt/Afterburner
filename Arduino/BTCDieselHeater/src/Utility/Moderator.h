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

#ifndef __BTC_MODERATOR_H__
#define __BTC_MODERATOR_H__

#include <map>
#include <ArduinoJSON.h>
#include "DebugPort.h"

template <class T>
class TModerator {
  std::map<const char*, T> Memory;
public:
  bool shouldSend(const char* name, T value);
  bool addJson(const char* name, T value, JsonObject& root);
	void reset();
	void reset(const char* name);
};

template<class T>
bool TModerator<T>::shouldSend(const char* name, T value) 
{
  bool retval = true;
  auto it = Memory.find(name);
  if(it != Memory.end()) {
    retval = it->second != value;
    it->second = value;
  }
  else {
    Memory[name] = value;
  }
  return retval;
}

template<class T>
bool TModerator<T>::addJson(const char* name, T value, JsonObject& root) 
{
  bool retval;
  if( retval = shouldSend(name, value ) ) 
	  root.set(name, value);
  return retval;
}


template<class T>
void TModerator<T>::reset() 
{
 	for(auto it = Memory.begin(); it != Memory.end(); ++it) {
    if(std::is_pointer<T>::value) {
      it->second = NULL;
    }
    else {
      it->second = it->second+100;
    }
  } 
}

template<class T>
void TModerator<T>::reset(const char* name)
{
  auto it = Memory.find(name);
  if(it != Memory.end()) {
    DebugPort.print("Resetting moderator: \""); DebugPort.print(name); DebugPort.println("\"");
    if(std::is_pointer<T>::value) {
      it->second = NULL;
    }
    else {
      it->second = it->second+100;
    }
  }
}

class CModerator {
  TModerator<int> iModerator;
  TModerator<float> fModerator;
  TModerator<unsigned char> ucModerator;
  TModerator<const char*> szModerator;
public:
  // integer values
  bool shouldSend(const char* name, int value) {
    return iModerator.shouldSend(name, value);
  };
  bool addJson(const char* name, int value, JsonObject& root) { 
    return iModerator.addJson(name, value, root); 
  };
  // float values
  bool shouldSend(const char* name, float value) {
    return fModerator.shouldSend(name, value);
  };
  bool addJson(const char* name, float value, JsonObject& root) { 
    return fModerator.addJson(name, value, root); 
  };
  // unsigned char values
  bool shouldSend(const char* name, unsigned char value) {
    return ucModerator.shouldSend(name, value);
  };
  bool addJson(const char* name, unsigned char value, JsonObject& root) { 
    return ucModerator.addJson(name, value, root); 
  };
  // const char* values
  bool shouldSend(const char* name, const char* value) {
    return szModerator.shouldSend(name, value);
  };
  bool addJson(const char* name, const char* value, JsonObject& root) { 
    return szModerator.addJson(name, value, root); 
  };
  // force changes on all held values
  void reset() {
    iModerator.reset();
    fModerator.reset();
    ucModerator.reset();
    szModerator.reset();
  };
  void reset(const char* name) {
    iModerator.reset(name);
    fModerator.reset(name);
    ucModerator.reset(name);
    szModerator.reset(name);
  };
};

#endif // __BTC_MODERATOR_H__