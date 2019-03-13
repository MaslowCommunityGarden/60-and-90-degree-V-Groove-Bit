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

#ifndef system_h
#define system_h

// Convenience Defines - Maybe move into a nuts and bolts file?
#define FORWARD           1
#define BACKWARD         -1
#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1
#define MILLIMETERS 1
#define INCHES      25.4

// Define various pause bits
#define PAUSE_FLAG_USER_PAUSE bit(0)  // a pause triggered within the code that must be cleared by user using the ~ command

// Define system state bit map. The state variable primarily tracks the individual functions
// of Maslow to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE          0      // Must be zero. No flags.
#define STATE_ALARM         bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE    bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_OLD_SETTINGS  bit(2) // Locks out all g-code processes, allows settings access until old settings are loaded
#define STATE_CYCLE         bit(3) // Cycle is running or motions are being executed.
#define STATE_HOLD          bit(4) // Active feed hold
#define STATE_SAFETY_DOOR   bit(5) // Safety door is ajar. Feed holds and de-energizes system.
#define STATE_MOTION_CANCEL bit(6) // Motion cancel by feed hold and return to idle.
#define STATE_POS_ERR_IGNORE bit(7) // Motion not checked for position error

// Define old settings flag details
#define NEED_ENCODER_STEPS bit(0)
#define NEED_DIST_PER_ROT bit(1)
#define NEED_Z_ENCODER_STEPS bit(2)
#define NEED_Z_DIST_PER_ROT bit(3)

// Storage for global system states
// Some of this could be more appropriately moved to the gcode parser
typedef struct {
  bool stop;                  // Stop flag.
  byte state;                 // State tracking flag
  byte pause;                 // Pause flag.
  float xPosition;            // Cartessian position of XY axes
  float yPosition;            // Cached because calculating position is intensive
  float steps[3];             // Encoder position of axes
  bool  useRelativeUnits;     //
  unsigned long lastSerialRcvd; // The millis of the last rcvd serial command, used by watchdo
  int   lastGCommand;         //Stores the value of the last command run eg: G01 -> 1
  int   lastTool;             //Stores the value of the last tool number eg: T4 -> 4
  int   nextTool;             //Stores the value of the next tool number eg: T4 -> 4
  float inchesToMMConversion; //Used to track whether to convert from inches, can probably be done in a way that doesn't require RAM
  float feedrate;             //The feedrate of the machine in mm/min
  // THE FOLLOWING IS USED FOR IMPORTING SETTINGS FROM FIRMWARE v1.00 AND EARLIER 
  // It can be deleted at some point
  byte oldSettingsFlag;
} system_t;
extern system_t sys;
extern Axis leftAxis;
extern Axis rightAxis;
extern Axis zAxis;
extern RingBuffer incSerialBuffer;
extern Kinematics kinematics;
extern byte systemRtExecAlarm;
extern int SpindlePowerControlPin;
extern int LaserPowerPin;
extern int ProbePin;

void  calibrateChainLengths(String);
void  setupAxes();
int   getPCBVersion();
void pause();
void maslowDelay(unsigned long);
void execSystemRealtime();
void systemSaveAxesPosition();
void systemReset();
byte systemExecuteCmdstring(String&);
void setPWMPrescalers(int prescalerChoice);
void configAuxLow(int A1, int A2, int A3, int A4, int A5, int A6);
void configAuxHigh(int A7, int A8, int A9);
#endif
