/*This file is part of the Maslow Control Software.

The Maslow Control Software is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Maslow Control Software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.

Copyright 2014-2017 Bar Smith*/

// This file contains functions related to the probe

#include "Maslow.h"

// the variable probePin is assigned in configAuxLow() in System.cpp

bool checkForProbeTouch(const int& probePin) {
  /*
      Check to see if ProbePin has gone LOW
  */
  if (digitalRead(probePin) == LOW) {
    readyCommandString = "";
    return 1;
  }
  return 0;
}
