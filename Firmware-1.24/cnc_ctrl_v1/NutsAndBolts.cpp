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

// This file contains helper functions that are used throughout

#include "Maslow.h"

float readFloat(const String& str, byte& index, float& retVal){
    /*
    Takes a string and a starting character index and returns a float if it can
    be parsed from the string, it will skip leading spaces.  Does not support 
    scientific notation as this is officially not supported by GCode.  
    Code was adopted from arduino Stream::parseFloat and some from Grbl's 
    read_float.  It is a custom function because all arduino and c++ functions 
    appear to handle scientific notation or hexadecimal notation, or some other 
    type of numerical representation that we don't want supported.
    */
    bool isNegative = false;
    bool isFraction = false;
    long value = 0;
    float fraction = 1.0;
    byte ndigit = 0;
    while (str[index] == ' ' && index < str.length()){
      index++;
    }
    do{
      if (index < str.length()){
        if(str[index] == '-')
          isNegative = true;
        else if (str[index] == '.')
          isFraction = true;
        else if(str[index] >= '0' && str[index] <= '9')  {// is a digit?
          ndigit++;
          value = value * 10 + str[index] - '0';
          if(isFraction)
             fraction *= 0.1;
        }
        index++;
      }
    }
    while(((str[index] >= '0' && str[index] <= '9')  || (str[index] == '.' && !isFraction)) && index < str.length());

    if (!ndigit) { return false; };

    if(isNegative)
      value = -value;
    if(isFraction)
      retVal = value * fraction;
    else
      retVal = value;

    return true;
}