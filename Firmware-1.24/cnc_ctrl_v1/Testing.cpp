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

// This file contains various testing provisions

#include "Maslow.h"

void PIDTestVelocity(Axis* axis, const float start, const float stop, const float steps, const float version){
    // Moves the defined Axis at series of speed steps for PID tuning
    // Start Log
    Serial.println(F("--PID Velocity Test Start--"));
    Serial.println(axis->motorGearboxEncoder.getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output"));
    }

    double startTime;
    double print = micros();
    double current = micros();
    float error;
    float reportedSpeed;
    float span = stop - start;
    float speed;
    
    // Start the steps
    axis->disablePositionPID();
    axis->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        speed = start;
        if (i > 0){
            speed = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        axis->motorGearboxEncoder.write(speed);
        while (startTime + 2000000 > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              Serial.println(axis->motorGearboxEncoder.pidState());
            }
            else {
              reportedSpeed= axis->motorGearboxEncoder.cachedSpeed();
              error =  reportedSpeed - speed;
              print = current;
              Serial.println(error);
            }
          }
          current = micros();
        }
    }
    axis->motorGearboxEncoder.write(0);
    
    // Print end of log, and update axis for use again
    Serial.println(F("--PID Velocity Test Stop--\n"));
    axis->write(axis->read());
    axis->detach();
    axis->enablePositionPID();
    kinematics.forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition, 0.0, 0.0);
}

void positionPIDOutput (Axis* axis, float setpoint, float startingPoint){
  Serial.print((setpoint - startingPoint), 4);
  Serial.print(F(","));
  Serial.print((axis->pidInput() - startingPoint),4);
  Serial.print(F(","));  
  Serial.print(axis->pidOutput(),4);
  Serial.print(F(","));
  Serial.print(axis->motorGearboxEncoder.cachedSpeed(), 4);
  Serial.print(F(","));
  Serial.println(axis->motorGearboxEncoder.motor.lastSpeed());
}

void PIDTestPosition(Axis* axis, float start, float stop, const float steps, const float stepTime, const float version){
    // Moves the defined Axis at series of chain distance steps for PID tuning
    // Start Log
    Serial.println(F("--PID Position Test Start--"));
    Serial.println(axis->getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output,rpminput,voltage"));
    }

    unsigned long startTime;
    unsigned long print = micros();
    unsigned long current = micros();
    float error;
    float startingPoint = axis->read();
    start = startingPoint + start;
    stop  = startingPoint + stop;
    float span = stop - start;
    float location;
    
    // Start the steps
    axis->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        location = start;
        if (i > 0){
            location = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        current = micros();
        axis->write(location);
        while (startTime + (stepTime * 1000) > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              positionPIDOutput(axis, location, startingPoint);
            }
            else {
              error   =  axis->read() - location;
              Serial.println(error);
            }
            print = current;
          }
          current = micros();
        }
    }
    startTime = micros();
    current = micros();
    //Allow 1 seccond to settle out
    while (startTime + 1000000 > current){
      if (current - print > LOOPINTERVAL){
        if (version == 2) {
          positionPIDOutput(axis, location, startingPoint);
        }            
        else {
          error   =  axis->read() - location;
          Serial.println(error);
        }
        print = current;
      }
      current = micros();
    }
    // Print end of log, and update axis for use again
    Serial.println(F("--PID Position Test Stop--\n"));
    axis->write(axis->read());
    axis->detach();
    kinematics.forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition, 0.0, 0.0);
}

void voltageTest(Axis* axis, int start, int stop){
    // Moves the defined Axis at a series of voltages and reports the resulting
    // RPM
    Serial.println(F("--Voltage Test Start--"));
    int direction = 1;
    if (stop < start){ direction = -1;}
    int steps = abs(start - stop);
    unsigned long startTime = millis() + 200;
    unsigned long currentTime = millis();
    unsigned long printTime = 0;
    
    for (int i = 0; i <= steps; i++){
        axis->motorGearboxEncoder.motor.directWrite((start + (i*direction)));
        while (startTime > currentTime - (i * 200)){
            currentTime = millis();
            if ((printTime + 50) <= currentTime){
                Serial.print((start + (i*direction)));
                Serial.print(F(","));
                Serial.print(axis->motorGearboxEncoder.computeSpeed(),4);
                Serial.print(F("\n"));
                printTime = millis();
            }
        }
    }
    
    // Print end of log, and update axis for use again
    axis->motorGearboxEncoder.motor.directWrite(0);
    Serial.println(F("--Voltage Test Stop--\n"));
    axis->write(axis->read());
    kinematics.forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition, 0.0, 0.0);
}