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

template <class T>
class TModerator {
  std::map<const char*, T> Memory;
public:
  bool shouldSend(const char* name, T value);
  bool send(const char* name, T value, JsonObject& root);
	void reset();
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
bool TModerator<T>::send(const char* name, T value, JsonObject& root) 
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
    it->second = it->second+100;
  } 
}


class CModerator {
  TModerator<int> iModerator;
  TModerator<float> fModerator;
  TModerator<unsigned char> ucModerator;
public:
  bool send(const char* name, int value, JsonObject& root) { 
    return iModerator.send(name, value, root); 
  };
  bool send(const char* name, float value, JsonObject& root) { 
    return fModerator.send(name, value, root); 
  };
  bool send(const char* name, unsigned char value, JsonObject& root) { 
    return ucModerator.send(name, value, root); 
  };
  void reset() {
    iModerator.reset();
    fModerator.reset();
    ucModerator.reset();
  };
};

#endif // __BTC_MODERATOR_H__