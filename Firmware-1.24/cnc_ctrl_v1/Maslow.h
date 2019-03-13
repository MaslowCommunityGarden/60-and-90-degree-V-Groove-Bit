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

// This is the main maslow include file

#ifndef maslow_h
#define maslow_h

// Maslow Firmware Version tracking
#define VERSIONNUMBER 1.24

// Define standard libraries used by maslow.
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <Servo.h>

// Define the maslow system include files. This ensures that dependencies are
// loaded in the proper order.  Be careful moving these around.
#include "Config.h"
#include "TimerOne.h"
#include "Motor.h"
#include "PID_v1.h"
#include "utility/direct_pin_read.h"
#include "Encoder.h"
#include "MotorGearboxEncoder.h"
#include "Axis.h"
#include "Kinematics.h"
#include "RingBuffer.h"
#include "GCode.h"
#include "Testing.h"
#include "Motion.h"
#include "Report.h"
#include "Spindle.h"
#include "Probe.h"
#include "Settings.h"
#include "NutsAndBolts.h"
#include "System.h"
#include "SimavrSerial.h"

#endif
