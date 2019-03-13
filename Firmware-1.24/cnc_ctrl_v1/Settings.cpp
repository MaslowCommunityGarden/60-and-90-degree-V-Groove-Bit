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

// EEPROM addresses 300 and up can be used by Maslow.  Under 300 was used
// previously by pre v1.00 Firmware.

#include "Maslow.h"
#include <EEPROM.h>

void settingsLoadFromEEprom(){
    /*
    Loads data from EEPROM if EEPROM data is valid, only called on startup

    Settings are stored starting at address 340 all the way up.
    */
    settingsVersion_t settingsVersionStruct;
    settings_t tempSettings;

    settingsReset(); // Load default values first
    EEPROM.get(300, settingsVersionStruct);
    EEPROM.get(340, tempSettings);
    if (settingsVersionStruct.settingsVersion == SETTINGSVERSION &&
        settingsVersionStruct.eepromValidData == EEPROMVALIDDATA &&
        tempSettings.eepromValidData == EEPROMVALIDDATA){
          sysSettings = tempSettings;
    }
    else {
      reportStatusMessage(STATUS_SETTING_READ_FAIL);
    }

    // Apply settings
    setPWMPrescalers(int(sysSettings.fPWM));
    kinematics.recomputeGeometry();
    leftAxis.changeEncoderResolution(&sysSettings.encoderSteps);
    rightAxis.changeEncoderResolution(&sysSettings.encoderSteps);
    leftAxis.changePitch(&sysSettings.distPerRot);
    rightAxis.changePitch(&sysSettings.distPerRot);
    zAxis.changePitch(&sysSettings.zDistPerRot);
    zAxis.changeEncoderResolution(&sysSettings.zEncoderSteps);
}

void settingsReset() {
    /*
    Loads default data into settings, many of these values are approximations
    from the an ideal stock frame.  Other values are just the recommended
    value.  Ideally we want these defaults to match the defaults in GroundControl
    so that if a value is not changed by a user or is not used, it doesn't
    need to be updated here.
    */
    sysSettings.machineWidth = 2438.4; // float machineWidth;
    sysSettings.machineHeight = 1219.2; // float machineHeight;
    sysSettings.distBetweenMotors = 2978.4; // float distBetweenMotors;
    sysSettings.motorOffsetY = 463.0;  // float motorOffsetY;
    sysSettings.sledWidth = 310.0;  // float sledWidth;
    sysSettings.sledHeight = 139.0;  // float sledHeight;
    sysSettings.sledCG = 79.0;   // float sledCG;
    sysSettings.kinematicsType = 1;      // byte kinematicsType;
    sysSettings.rotationDiskRadius = 250.0;  // float rotationDiskRadius;
    sysSettings.axisDetachTime = 2000;   // int axisDetachTime;
    sysSettings.chainLength = 3360;   // int maximum length of chain;
    sysSettings.originalChainLength = 1650;   // int originalChainLength;
    sysSettings.encoderSteps = 8113.73; // float encoderSteps;
    sysSettings.distPerRot = 63.5;   // float distPerRot;
    sysSettings.maxFeed = 700;   // int maxFeed;
    sysSettings.zAxisAttached = true;   // zAxisAttached;
    sysSettings.spindleAutomateType = NONE;  // bool spindleAutomate;
    sysSettings.maxZRPM = 12.60;  // float maxZRPM;
    sysSettings.zDistPerRot = 3.17;   // float zDistPerRot;
    sysSettings.zEncoderSteps = 7560.0; // float zEncoderSteps;
    sysSettings.KpPos = 1300.0; // float KpPos;
    sysSettings.KiPos = 0.0;    // float KiPos;
    sysSettings.KdPos = 34.0;   // float KdPos;
    sysSettings.propWeightPos = 1.0;    // float propWeightPos;
    sysSettings.KpV = 5.0;    // float KpV;
    sysSettings.KiV = 0.0;    // float KiV;
    sysSettings.KdV = 0.28;   // float KdV;
    sysSettings.propWeightV = 1.0;    // float propWeightV;
    sysSettings.zKdPos = 1300.0; // float zKpPos;
    sysSettings.zKiPos = 0.0;    // float zKiPos;
    sysSettings.zKdPos = 34.0;   // float zKdPos;
    sysSettings.zPropWeightPos = 1.0;    // float zPropWeightPos;
    sysSettings.zKpV = 5.0;    // float zKpV;
    sysSettings.zKiV = 0.0;    // float zKiV;
    sysSettings.zKdV = 0.28;   // float zKdV;
    sysSettings.zPropWeightV = 1.0;    // float zPropWeightV;
    sysSettings.chainSagCorrection = 0.0;  // float chainSagCorrection;
    sysSettings.chainOverSprocket = 1;   // byte chainOverSprocket;
    sysSettings.fPWM = 3;   // byte fPWM;
    sysSettings.leftChainTolerance = 0.0;    // float leftChainTolerance;
    sysSettings.rightChainTolerance = 0.0;    // float rightChainTolerance;
    sysSettings.positionErrorLimit = 2.0;  // float positionErrorLimit;
    sysSettings.eepromValidData = EEPROMVALIDDATA; // byte eepromValidData;
}

void settingsWipe(byte resetType){
  /*
  Wipes certain bytes in the EEPROM, you probably want to reset after calling
  this
  */
  if (bit_istrue(resetType, SETTINGS_RESTORE_SETTINGS)){
    for (size_t i = 340 ; i < sizeof(sysSettings) + 340 ; i++) {
      EEPROM.write(i, 0);
    }
  }
  else if (bit_istrue(resetType, SETTINGS_RESTORE_MASLOW)){
    for (size_t i = 300 ; i < sizeof(sysSettings) + 340; i++) {
      EEPROM.write(i, 0);
    }
  }
  else if (bit_istrue(resetType, SETTINGS_RESTORE_ALL)){
    for (size_t i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
  }
}

void settingsSaveToEEprom(){
    /*
    Saves settings to EEPROM, only called when settings change

    Settings are stored starting at address 340 all the way up.
    */
    settingsVersion_t settingsVersionStruct = {SETTINGSVERSION, EEPROMVALIDDATA};
    EEPROM.put(300, settingsVersionStruct);
    EEPROM.put(340, sysSettings);
}

void settingsSaveStepstoEEprom(){
    /*
    Saves position to EEPROM, is called frequently by execSystemRealtime

    Steps are saved in address 310 -> 339.  Room for expansion for additional
    axes in the future.
    */
    // don't run if old position data has not been incorporated yet
    if (!sys.oldSettingsFlag){
      settingsStepsV1_t sysSteps = {
        leftAxis.steps(),
        rightAxis.steps(),
        zAxis.steps(),
        EEPROMVALIDDATA
      };
      EEPROM.put(310, sysSteps);
    }
}

void settingsLoadStepsFromEEprom(){
    /*
    Loads position to EEPROM, is called on startup.

    Steps are saved in address 310 -> 339.  Room for expansion for additional
    axes in the future.
    */
    settingsStepsV1_t tempStepsV1;

    EEPROM.get(310, tempStepsV1);
    if (tempStepsV1.eepromValidData == EEPROMVALIDDATA){
            leftAxis.setSteps(tempStepsV1.lSteps);
            rightAxis.setSteps(tempStepsV1.rSteps);
            zAxis.setSteps(tempStepsV1.zSteps);
    }
    else if (EEPROM.read(5) == EEPROMVALIDDATA &&
        EEPROM.read(105) == EEPROMVALIDDATA &&
        EEPROM.read(205) == EEPROMVALIDDATA){
        bit_true(sys.oldSettingsFlag, NEED_ENCODER_STEPS);
        bit_true(sys.oldSettingsFlag, NEED_DIST_PER_ROT);
        bit_true(sys.oldSettingsFlag, NEED_Z_ENCODER_STEPS);
        bit_true(sys.oldSettingsFlag, NEED_Z_DIST_PER_ROT);
        sys.state = STATE_OLD_SETTINGS;
        Serial.println(F("Old position data detected."));
        Serial.println(F("Please set $12, $13, $19, and $20 to load position."));
    }
    else {
        systemRtExecAlarm |= ALARM_POSITION_LOST;  // if this same global is touched by ISR then need to make atomic somehow
                                                   // also need to consider if need difference between flag with bits and
                                                   // error message as a byte.
    }
}

void settingsLoadOldSteps(){
    /*
    Loads the old version of step settings, only called once encoder steps
    and distance per rotation have been loaded.  Wipes the old data once
    incorporated to prevent oddities in the future
    */
    if (sys.state == STATE_OLD_SETTINGS){
      float l, r , z;
      EEPROM.get(9, l);
      EEPROM.get(109, r);
      EEPROM.get(209, z);
      leftAxis.set(l);
      rightAxis.set(r);
      zAxis.set(z);
      for (int i = 0; i <= 200; i = i +100){
        for (int j = 5; j <= 13; j++){
          EEPROM.write(i + j, 0);
        }
      }
      sys.state = STATE_IDLE;
    }
  }

byte settingsStoreGlobalSetting(const byte& parameter,const float& value){
    /*
    Alters individual settings which are then stored to EEPROM.  Returns a
    status message byte value
    */

    // We can add whatever sanity checks we want here and error out if we like
    switch(parameter) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
            switch(parameter) {
                case 0:
                      sysSettings.machineWidth = value;
                      break;
                case 1:
                      sysSettings.machineHeight = value;
                      break;
                case 2:
                      sysSettings.distBetweenMotors = value;
                      break;
                case 3:
                      sysSettings.motorOffsetY = value;
                      break;
                case 4:
                      sysSettings.sledWidth = value;
                      break;
                case 5:
                      sysSettings.sledHeight = value;
                      break;
                case 6:
                      sysSettings.sledCG = value;
                      break;
                case 7:
                      sysSettings.kinematicsType = value;
                      break;
                case 8:
                      sysSettings.rotationDiskRadius = value;
                      break;
            }
            kinematics.init();
            break;
        case 9:
              sysSettings.axisDetachTime = value;
              break;
        case 10:
              sysSettings.chainLength = value;
              break;
        case 11:
              sysSettings.originalChainLength = value;
              break;
        case 12:
              sysSettings.encoderSteps = value;
              leftAxis.changeEncoderResolution(&sysSettings.encoderSteps);
              rightAxis.changeEncoderResolution(&sysSettings.encoderSteps);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_ENCODER_STEPS);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              kinematics.init();
              break;
        case 13:
              sysSettings.distPerRot = value;
              kinematics.R = (sysSettings.distPerRot)/(2.0 * 3.14159);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_DIST_PER_ROT);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              kinematics.init();
              break;
        case 15:
              sysSettings.maxFeed = value;
              break;
        case 16:
              sysSettings.zAxisAttached = value;
              break;
        case 17: 
              sysSettings.spindleAutomateType = static_cast<SpindleAutomationType>(value);
              break;
        case 18:
              sysSettings.maxZRPM = value;
              break;
        case 19:
              sysSettings.zDistPerRot = value;
              zAxis.changePitch(&sysSettings.zDistPerRot);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_Z_DIST_PER_ROT);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              break;
        case 20:
              sysSettings.zEncoderSteps = value;
              zAxis.changeEncoderResolution(&sysSettings.zEncoderSteps);
              if (sys.oldSettingsFlag){
                bit_false(sys.oldSettingsFlag, NEED_Z_ENCODER_STEPS);
                if (!sys.oldSettingsFlag){
                  settingsLoadOldSteps();
                }
              }
              break;
        case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28:
            switch(parameter) {
                case 21:
                      sysSettings.KpPos = value;
                      break;
                case 22:
                      sysSettings.KiPos = value;
                      break;
                case 23:
                      sysSettings.KdPos = value;
                      break;
                case 24:
                      sysSettings.propWeightPos = value;
                      break;
                case 25:
                      sysSettings.KpV = value;
                      break;
                case 26:
                      sysSettings.KiV = value;
                      break;
                case 27:
                      sysSettings.KdV = value;
                      break;
                case 28:
                      sysSettings.propWeightV = value;
                      break;
                }
                leftAxis.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
                rightAxis.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
                break;
        case 29: case 30: case 31: case 32: case 33: case 34: case 35: case 36:
            switch(parameter) {
                case 29:
                      sysSettings.zKpPos = value;
                      break;
                case 30:
                      sysSettings.zKiPos = value;
                      break;
                case 31:
                      sysSettings.zKdPos = value;
                      break;
                case 32:
                      sysSettings.zPropWeightPos = value;
                      break;
                case 33:
                      sysSettings.zKpV = value;
                      break;
                case 34:
                      sysSettings.zKiV = value;
                      break;
                case 35:
                      sysSettings.zKdV = value;
                      break;
                case 36:
                      sysSettings.zPropWeightV = value;
                      break;
            }
            zAxis.setPIDValues(&sysSettings.zKpPos, &sysSettings.zKiPos, &sysSettings.zKdPos, &sysSettings.zPropWeightPos, &sysSettings.zKpV, &sysSettings.zKiV, &sysSettings.zKdV, &sysSettings.zPropWeightV);
            break;
        case 37:
              sysSettings.chainSagCorrection = value;
              break;
        case 38:
              settingsSaveStepstoEEprom();
              sysSettings.chainOverSprocket = value;
              setupAxes();
              settingsLoadStepsFromEEprom();
              // Set initial desired position of the machine to its current position
              leftAxis.write(leftAxis.read());
              rightAxis.write(rightAxis.read());
              zAxis.write(zAxis.read());
              kinematics.init();
              break;
        case 39:
              sysSettings.fPWM = value;
              setPWMPrescalers(value);
              break;
        case 40:
              sysSettings.leftChainTolerance = value;
              break;
        case 41:
              sysSettings.rightChainTolerance = value;
              break;
        case 42:
              sysSettings.positionErrorLimit = value;
              break;
        default:
              return(STATUS_INVALID_STATEMENT);
    }
    settingsSaveToEEprom();
    return(STATUS_OK);
}
