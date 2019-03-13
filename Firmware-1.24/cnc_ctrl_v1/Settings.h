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

// This file contains the machine settings that are saved to eeprom

#ifndef settings_h
#define settings_h

#define SETTINGSVERSION 5      // The current version of settings, if this doesn't
                               // match what is in EEPROM then settings on
                               // machine are reset to defaults
#define EEPROMVALIDDATA 56     // This is just a random byte value that is used 
                               // to determine if the data in the EEPROM was 
                               // saved by maslow, or something else.

// Reset Types
#define SETTINGS_RESTORE_SETTINGS bit(0)
#define SETTINGS_RESTORE_MASLOW bit(1)
#define SETTINGS_RESTORE_ALL bit(2)

enum SpindleAutomationType {
  NONE,
  SERVO,
  RELAY_ACTIVE_HIGH,
  RELAY_ACTIVE_LOW };

typedef struct {  // I think this is about ~128 bytes in size if I counted correctly
  float machineWidth;
  float machineHeight;
  float distBetweenMotors;
  float motorOffsetY;
  float sledWidth;
  float sledHeight;
  float sledCG;
  byte kinematicsType;
  float rotationDiskRadius;
  unsigned int axisDetachTime;
  unsigned int chainLength;
  unsigned int originalChainLength;
  float encoderSteps;
  float distPerRot;
  unsigned int maxFeed;
  bool zAxisAttached;
  SpindleAutomationType spindleAutomateType;
  float maxZRPM;
  float zDistPerRot;
  float zEncoderSteps;
  float KpPos;
  float KiPos;
  float KdPos;
  float propWeightPos;
  float KpV;
  float KiV;
  float KdV;
  float propWeightV;
  float zKpPos;
  float zKiPos;
  float zKdPos;
  float zPropWeightPos;
  float zKpV;
  float zKiV;
  float zKdV;
  float zPropWeightV;
  float chainSagCorrection;
  byte chainOverSprocket;
  byte fPWM;
  float leftChainTolerance;
  float rightChainTolerance;
  float positionErrorLimit;
  byte eepromValidData;  // This should always be last, that way if an error
                         // happens in writing, it will not be written and we
} settings_t;            // will know to reset the settings
extern settings_t sysSettings;

typedef struct {
  byte settingsVersion;
  byte eepromValidData;
} settingsVersion_t;

typedef struct {
  long lSteps;
  long rSteps;
  long zSteps;
  byte eepromValidData;
} settingsStepsV1_t;

void settingsLoadFromEEprom();
void settingsReset();
void settingsWipe(byte);
void settingsSaveToEEprom();
void settingsSaveStepstoEEprom();
void settingsLoadStepsFromEEprom();
byte settingsStoreGlobalSetting(const byte&,const float&);

#endif
