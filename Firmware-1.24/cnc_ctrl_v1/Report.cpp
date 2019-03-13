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

// This file contains the functions for outgoing Serial responses

#include "Maslow.h"

void  reportStatusMessage(byte status_code){
    /*
    
    Sends confirmation protocol response for commands. For every incoming line,
    this method responds with an 'ok' for a successful command or an 'error:'
    to indicate some error event with the line or some critical system error during
    operation.
    
    Taken from Grbl http://github.com/grbl/grbl
    */
    if (status_code == 0) { // STATUS_OK
      Serial.println(F("ok"));
    } else {
      Serial.print(F("error: "));
      #ifdef REPORT_GUI_MODE
        Serial.println(status_code);
      #else
        switch(status_code) {
          // case STATUS_EXPECTED_COMMAND_LETTER=")); Serial.println(  // Serial.println(F("Expected command letter")); break;
          case STATUS_BAD_NUMBER_FORMAT:
          Serial.println(F("Bad number format")); break;
          case STATUS_INVALID_STATEMENT:
            Serial.println(F("Invalid statement")); break;
          case STATUS_OLD_SETTINGS:
            Serial.println(F("Please set $12, $13, $19, and $20 to load old position data.")); break;
          // case STATUS_NEGATIVE_VALUE:
          // Serial.println(F("Value < 0")); break;
          // case STATUS_SETTING_DISABLED:
          // Serial.println(F("Setting disabled")); break;
          // case STATUS_SETTING_STEP_PULSE_MIN:
          // Serial.println(F("Value < 3 usec")); break;
          case STATUS_SETTING_READ_FAIL:
            Serial.println(F("EEPROM read fail. Using default settings.")); break;
          // case STATUS_IDLE_ERROR:
          // Serial.println(F("Not idle")); break;
          // case STATUS_ALARM_LOCK:
          // Serial.println(F("Alarm lock")); break;
          // case STATUS_SOFT_LIMIT_ERROR:
          // Serial.println(F("Homing not enabled")); break;
          // case STATUS_OVERFLOW:
          // Serial.println(F("Line overflow")); break;
          // #ifdef MAX_STEP_RATE_HZ
          //   case STATUS_MAX_STEP_RATE_EXCEEDED:
          //   Serial.println(F("Step rate > 30kHz")); break;
          // #endif
          // Common g-code parser errors.
          // case STATUS_GCODE_MODAL_GROUP_VIOLATION:
          // Serial.println(F("Modal group violation")); break;
          // case STATUS_GCODE_UNSUPPORTED_COMMAND:
          // Serial.println(F("Unsupported command")); break;
          // case STATUS_GCODE_UNDEFINED_FEED_RATE:
          // Serial.println(F("Undefined feed rate")); break;
          default:
            // Remaining g-code parser errors with error codes
            Serial.print(F("Invalid gcode ID:"));
            Serial.println(status_code); // Print error code for user reference
        }
      #endif
    }
}

void reportFeedbackMessage(byte message_code){
  Serial.print(F("Message: "));
  switch(message_code) {
    // case MESSAGE_CRITICAL_EVENT:
    //   Serial.print(F("Reset to continue")); break;
    // case MESSAGE_ALARM_LOCK:
    //   Serial.print(F("'$H'|'$X' to unlock")); break;
    // case MESSAGE_ALARM_UNLOCK:
    //   Serial.print(F("Caution: Unlocked")); break;
    // case MESSAGE_ENABLED:
    //   Serial.print(F("Enabled")); break;
    // case MESSAGE_DISABLED:
    //   Serial.print(F("Disabled")); break;
    // case MESSAGE_SAFETY_DOOR_AJAR:
    //   Serial.print(F("Check Door")); break;
    // case MESSAGE_CHECK_LIMITS:
    //   Serial.print(F("Check Limits")); break;
    // case MESSAGE_PROGRAM_END:
    //   Serial.print(F("Pgm End")); break;
    case MESSAGE_RESTORE_DEFAULTS:
      Serial.print(F("Restoring defaults")); break;
    // case MESSAGE_SPINDLE_RESTORE:
    //   Serial.print(F("Restoring spindle")); break;
    // case MESSAGE_SLEEP_MODE:
    //   Serial.print(F("Sleeping")); break;
  }
  Serial.println(F(" "));
}

// Prints alarm messages.
void  reportAlarmMessage(byte alarm_code) {
  Serial.print(F("ALARM: "));
  #ifdef REPORT_GUI_MODE
    Serial.println(alarm_code);
  #else
    switch (alarm_code) {
      case ALARM_POSITION_LOST: {
        Serial.println(F("Position Lost")); break;
        }
      case ALARM_GCODE_PARAM_ERROR: {
        Serial.println(F("There is a parameter error in this line of Gcode - make a note of the line number. Cutting will be put on hold until you choose whether to 'Resume Cut' (skipping this line) or 'Stop'.   "));
        pause(); //This pause() waits for user acknowledgement of message
        pause(); //Now wait for user decision about 'Stop' or 'Resume'
        break;
        }
      case ALARM_POSITION_LIMIT_ERROR: {
        Serial.println(F("The sled is not keeping up with its expected position and has halted. Click the 'Stop' button to clear the alarm. More information at: https://github.com/MaslowCNC/Firmware/wiki/Keeping-Up  "));
        sys.stop = true;
        break;
        }
    }
  #endif
}

// Maslow global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void reportMaslowSettings() {
  // Print Maslow settings.
  // Taken from Grbl. http://github.com/grbl/grbl
  #ifdef REPORT_GUI_MODE
    Serial.print(F("$0=")); Serial.println(sysSettings.machineWidth, 8);
    Serial.print(F("$1=")); Serial.println(sysSettings.machineHeight, 8);
    Serial.print(F("$2=")); Serial.println(sysSettings.distBetweenMotors, 8);
    Serial.print(F("$3=")); Serial.println(sysSettings.motorOffsetY, 8);
    Serial.print(F("$4=")); Serial.println(sysSettings.sledWidth, 8);
    Serial.print(F("$5=")); Serial.println(sysSettings.sledHeight, 8);
    Serial.print(F("$6=")); Serial.println(sysSettings.sledCG, 8);
    Serial.print(F("$7=")); Serial.println(sysSettings.kinematicsType);
    Serial.print(F("$8=")); Serial.println(sysSettings.rotationDiskRadius, 8);
    Serial.print(F("$9=")); Serial.println(sysSettings.axisDetachTime);
    Serial.print(F("$10=")); Serial.println(sysSettings.chainLength);
    Serial.print(F("$11=")); Serial.println(sysSettings.originalChainLength);
    Serial.print(F("$12=")); Serial.println(sysSettings.encoderSteps, 8);
    Serial.print(F("$13=")); Serial.println(sysSettings.distPerRot, 8);
    Serial.print(F("$15=")); Serial.println(sysSettings.maxFeed);
    Serial.print(F("$16=")); Serial.println(sysSettings.zAxisAttached);
    Serial.print(F("$17=")); Serial.println(sysSettings.spindleAutomate);
    Serial.print(F("$18=")); Serial.println(sysSettings.maxZRPM, 8);
    Serial.print(F("$19=")); Serial.println(sysSettings.zDistPerRot, 8);
    Serial.print(F("$20=")); Serial.println(sysSettings.zEncoderSteps, 8);
    Serial.print(F("$21=")); Serial.println(sysSettings.KpPos, 8);
    Serial.print(F("$22=")); Serial.println(sysSettings.KiPos, 8);
    Serial.print(F("$23=")); Serial.println(sysSettings.KdPos, 8);
    Serial.print(F("$24=")); Serial.println(sysSettings.propWeightPos, 8);
    Serial.print(F("$25=")); Serial.println(sysSettings.KpV, 8);
    Serial.print(F("$26=")); Serial.println(sysSettings.KiV, 8);
    Serial.print(F("$27=")); Serial.println(sysSettings.KdV, 8);
    Serial.print(F("$28=")); Serial.println(sysSettings.propWeightV, 8);
    Serial.print(F("$29=")); Serial.println(sysSettings.zKpPos, 8);
    Serial.print(F("$30=")); Serial.println(sysSettings.zKiPos, 8);
    Serial.print(F("$31=")); Serial.println(sysSettings.zKdPos, 8);
    Serial.print(F("$32=")); Serial.println(sysSettings.zPropWeightPos, 8);
    Serial.print(F("$33=")); Serial.println(sysSettings.zKpV, 8);
    Serial.print(F("$34=")); Serial.println(sysSettings.zKiV, 8);
    Serial.print(F("$35=")); Serial.println(sysSettings.zKdV, 8);
    Serial.print(F("$36=")); Serial.println(sysSettings.zPropWeightV, 8);
    Serial.print(F("$37=")); Serial.println(sysSettings.chainSagCorrection, 8);
    Serial.print(F("$38=")); Serial.println(sysSettings.chainOverSprocket);
    Serial.print(F("$39=")); Serial.println(sysSettings.fPWM);
    Serial.print(F("$40=")); Serial.println(sysSettings.leftChainTolerance, 8);
    Serial.print(F("$41=")); Serial.println(sysSettings.rightChainTolerance, 8);
    Serial.print(F("$42=")); Serial.println(sysSettings.positionErrorLimit, 8);
    
  #else
    Serial.print(F("$0=")); Serial.print(sysSettings.machineWidth);
    Serial.print(F(" (machine width, mm)\r\n$1=")); Serial.print(sysSettings.machineHeight, 8);
    Serial.print(F(" (machine height, mm)\r\n$2=")); Serial.print(sysSettings.distBetweenMotors, 8);
    Serial.print(F(" (motor distance, mm)\r\n$3=")); Serial.print(sysSettings.motorOffsetY, 8);
    Serial.print(F(" (motor height, mm)\r\n$4=")); Serial.print(sysSettings.sledWidth, 8);
    Serial.print(F(" (sled width, mm)\r\n$5=")); Serial.print(sysSettings.sledHeight, 8);
    Serial.print(F(" (sled height, mm)\r\n$6=")); Serial.print(sysSettings.sledCG, 8);
    Serial.print(F(" (sled cg, mm)\r\n$7=")); Serial.print(sysSettings.kinematicsType);
    Serial.print(F(" (Kinematics Type 1=Quadrilateral, 2=Triangular)\r\n$8=")); Serial.print(sysSettings.rotationDiskRadius, 8);
    Serial.print(F(" (rotation radius, mm)\r\n$9=")); Serial.print(sysSettings.axisDetachTime);
    Serial.print(F(" (axis idle before detach, ms)\r\n$10=")); Serial.print(sysSettings.chainLength);
    Serial.print(F(" (full length of chain, mm)\r\n$11=")); Serial.print(sysSettings.originalChainLength);
    Serial.print(F(" (calibration chain length, mm)\r\n$12=")); Serial.print(sysSettings.encoderSteps, 8);
    Serial.print(F(" (main steps per revolution)\r\n$13=")); Serial.print(sysSettings.distPerRot, 8);
    Serial.print(F(" (distance / rotation, mm)\r\n$15=")); Serial.print(sysSettings.maxFeed);
    Serial.print(F(" (max feed, mm/min)\r\n$16=")); Serial.print(sysSettings.zAxisAttached);
    Serial.print(F(" (Auto Z Axis, 1 = Yes)\r\n$17=")); Serial.print(sysSettings.spindleAutomateType);
    Serial.print(F(" (auto spindle enable 1=servo, 2=relay_h, 3=relay_l)\r\n$18=")); Serial.print(sysSettings.maxZRPM, 8);
    Serial.print(F(" (max z axis RPM)\r\n$19=")); Serial.print(sysSettings.zDistPerRot, 8);
    Serial.print(F(" (z axis distance / rotation)\r\n$20=")); Serial.print(sysSettings.zEncoderSteps, 8);
    Serial.print(F(" (z axis steps per revolution)\r\n$21=")); Serial.print(sysSettings.KpPos, 8);
    Serial.print(F(" (main Kp Pos)\r\n$22=")); Serial.print(sysSettings.KiPos, 8);
    Serial.print(F(" (main Ki Pos)\r\n$23=")); Serial.print(sysSettings.KdPos, 8);
    Serial.print(F(" (main Kd Pos)\r\n$24=")); Serial.print(sysSettings.propWeightPos, 8);
    Serial.print(F(" (main Pos proportional weight)\r\n$25=")); Serial.print(sysSettings.KpV, 8);
    Serial.print(F(" (main Kp Velocity)\r\n$26=")); Serial.print(sysSettings.KiV, 8);
    Serial.print(F(" (main Ki Velocity)\r\n$27=")); Serial.print(sysSettings.KdV, 8);
    Serial.print(F(" (main Kd Velocity)\r\n$28=")); Serial.print(sysSettings.propWeightV, 8);
    Serial.print(F(" (main Velocity proportional weight)\r\n$29=")); Serial.print(sysSettings.zKpPos, 8);
    Serial.print(F(" (z axis Kp Pos)\r\n$30=")); Serial.print(sysSettings.zKiPos, 8);
    Serial.print(F(" (z axis Ki Pos)\r\n$31=")); Serial.print(sysSettings.zKdPos, 8);
    Serial.print(F(" (z axis Kd Pos)\r\n$32=")); Serial.print(sysSettings.zPropWeightPos, 8);
    Serial.print(F(" (z axis Pos proportional weight)\r\n$33=")); Serial.print(sysSettings.zKpV, 8);
    Serial.print(F(" (z axis Kp Velocity)\r\n$34=")); Serial.print(sysSettings.zKiV, 8);
    Serial.print(F(" (z axis Ki Velocity)\r\n$35=")); Serial.print(sysSettings.zKdV, 8);
    Serial.print(F(" (z axis Kd Velocity)\r\n$36=")); Serial.print(sysSettings.zPropWeightV, 8);
    Serial.print(F(" (z axis Velocity proportional weight)\r\n$37=")); Serial.print(sysSettings.chainSagCorrection, 8);
    Serial.print(F(" (chain sag correction value)\r\n$38=")); Serial.print(sysSettings.chainOverSprocket);
    Serial.print(F(" (chain over sprocket)\r\n$39=")); Serial.print(sysSettings.fPWM);
    Serial.print(F(" (PWM frequency value 1=39,000Hz, 2=4,100Hz, 3=490Hz)\r\n$40=")); Serial.print(sysSettings.leftChainTolerance, 8);
    Serial.print(F(" (chain tolerance, left chain, mm)\r\n$41=")); Serial.print(sysSettings.rightChainTolerance, 8);
    Serial.print(F(" (chain tolerance, right chain, mm)\r\n$42=")); Serial.print(sysSettings.positionErrorLimit, 8);
    Serial.print(F(" (position error alarm limit, mm)"));
    Serial.println();
  #endif
}

void  returnError(){
    /*
    Prints the machine's positional error and the amount of space available in the 
    gcode buffer
    */
        Serial.print(F("[PE:"));
        Serial.print(leftAxis.error());
        Serial.print(',');
        Serial.print(rightAxis.error());
        Serial.print(',');
        Serial.print(incSerialBuffer.spaceAvailable());
        Serial.println(F("]"));
        if (!sys.stop) {
          if (!(sys.state & STATE_POS_ERR_IGNORE)) {
            if ((abs(leftAxis.error()) >= sysSettings.positionErrorLimit) || (abs(rightAxis.error()) >= sysSettings.positionErrorLimit)) {
                reportAlarmMessage(ALARM_POSITION_LIMIT_ERROR);
            }
          }
        }
}

void  returnPoz(){
    /*
    Causes the machine's position (x,y) to be sent over the serial connection updated on the UI
    in Ground Control. Also causes the error report to be sent. Only executes 
    if hasn't been called in at least POSITIONTIMEOUT ms.
    */
    
    static unsigned long lastRan = millis();
    
    if (millis() - lastRan > POSITIONTIMEOUT){
        
        
        Serial.print(F("<"));
        if (sys.stop){
            Serial.print(F("Stop,MPos:"));
        }
        else if (sys.pause){
            Serial.print(F("Pause,MPos:"));
        }
        else{
            Serial.print(F("Idle,MPos:"));
        }
        Serial.print(sys.xPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(sys.yPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(zAxis.read()/sys.inchesToMMConversion);
        Serial.println(F(",WPos:0.000,0.000,0.000>"));
        
        
        returnError();
        
        lastRan = millis();
    }
    
}

void  reportMaslowHelp(){
    /*
    This function outputs a brief summary of the $ system commands available.
    The list is somewhat aspirational based on what Grbl offers. Maslow
    does not currently support all of these features.

    This is taken heavily from grbl.  https://github.com/grbl/grbl
    */
    #ifndef REPORT_GUI_MODE
        Serial.println(F("$$ (view Maslow settings)"));
        // Serial.println(F("$# (view # parameters)"));
        // Serial.println(F("$G (view parser state)"));
        // Serial.println(F("$I (view build info)"));
        // Serial.println(F("$N (view startup blocks)"));
        Serial.println(F("$x=value (save Maslow setting)"));
        // Serial.println(F("$Nx=line (save startup block)"));
        // Serial.println(F("$C (check gcode mode)"));
        // Serial.println(F("$X (kill alarm lock)"));
        // Serial.println(F("$H (run homing cycle)"));
        Serial.println(F("~ (cycle start)"));  // Maslow treats this as resume or un-pause currently
        Serial.println(F("! (feed hold)"));    // Maslow treats this as a cycle stop.
        // Serial.println(F("? (current status)"));
        // Serial.println(F("ctrl-x (reset Maslow)"));
    #endif
}
