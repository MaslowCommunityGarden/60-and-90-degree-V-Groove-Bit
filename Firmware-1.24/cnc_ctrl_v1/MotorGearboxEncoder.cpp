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

/*
The Motor module imitates the behavior of the Arduino servo module. It allows a gear motor (or any electric motor)
to be a drop in replacement for a continuous rotation servo.

*/

#include "Maslow.h"

void MotorGearboxEncoder::setup(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const unsigned long& loopInterval)
{
    //initialize encoder
    encoder.setup(encoderPin1,encoderPin2);
    // I don't like this, but I don't know how else to initialize a pointer to a value
    float zero = 0.0;
    float one = 1.0;
    _Kp = _Ki = _Kd = &zero;
    
    //initialize motor
    motor.setupMotor(pwmPin, directionPin1, directionPin2);
    motor.write(0);
    
    //initialize the PID
    _PIDController.setup(&_currentSpeed, &_pidOutput, &_targetSpeed, _Kp, _Ki, _Kd, &one, DIRECT);
    initializePID(loopInterval);
    
    
}

void  MotorGearboxEncoder::write(const float& speed){
    /*
    Command the motor to turn at the given speed. Should be RPM is PWM right now.
    */
    
    _targetSpeed = speed;
    
}

void   MotorGearboxEncoder::initializePID(const unsigned long& loopInterval){
    //setup positive PID controller
    _PIDController.SetMode(AUTOMATIC);
    _PIDController.SetOutputLimits(-255, 255);
    _PIDController.SetSampleTime(loopInterval / 1000);
}

void  MotorGearboxEncoder::computePID(){
    /*
    Recompute the speed control PID loop and command the motor to move.
    */
    _currentSpeed = computeSpeed();

    _PIDController.Compute();

    motor.additiveWrite(_pidOutput);
}

void  MotorGearboxEncoder::setPIDValues(float* KpV, float* KiV, float* KdV, float* propWeightV){
    /*
    
    Set PID tuning values
    
    */
    
    _Kp = KpV;
    _Ki = KiV;
    _Kd = KdV;
    
    _PIDController.SetTunings(_Kp, _Ki, _Kd, propWeightV);
}

String  MotorGearboxEncoder::getPIDString(){
    /*
    
    Get PID tuning values
    
    */
    String PIDString = "Kp=";
    return PIDString + *_Kp + ",Ki=" + *_Ki + ",Kd=" + *_Kd;
}

String  MotorGearboxEncoder::pidState(){
    /*
    
    Get current value of PID variables, setpoint, input, output
    
    */
    return _PIDController.pidState();
}

void MotorGearboxEncoder::setPIDAggressiveness(float aggressiveness){
    /*
    
    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.
    
    */
    
    float adjustedKp = aggressiveness * *_Kp;
    float one = 1.0;
    
    _PIDController.SetTunings(&adjustedKp, _Ki, _Kd, &one);
    
}

void MotorGearboxEncoder::setEncoderResolution(float resolution){
    /*
    
    Change the encoder resolution
    
    */
    
    _encoderStepsToRPMScaleFactor = 60000000.0/resolution; //6*10^7 us per minute divided by 8113.73 steps per revolution
    
}

float MotorGearboxEncoder::computeSpeed(){
    /*
    
    Returns the motors speed in RPM since the last time this function was called
    should only be called by the PID process otherwise we are calculating the
    distance moved over a varying amount of time.
    
    */
    
    float currentPosition = encoder.read();
    float currentMicros = micros();
    
    float distMoved   =  currentPosition - _lastPosition;
    if (distMoved > 3 || distMoved < -3){
      
        // This dampens some of the effects of quantization without having 
        // a big effect on other changes
        float saveDistMoved = distMoved;
        if (distMoved - _lastDistMoved <= -1){ distMoved += .5;}
        else if (distMoved - _lastDistMoved >= 1){distMoved -= .5;}
        _lastDistMoved = saveDistMoved;
        
        unsigned long timeElapsed =  currentMicros - _lastTimeStamp;
        //Compute the speed in RPM
        _RPM = (_encoderStepsToRPMScaleFactor*distMoved)/float(timeElapsed);
    
    }
    else {
        float elapsedTime = encoder.elapsedTime();
        float lastTime = micros() - encoder.lastStepTime();  // no direction associated with this
        if (lastTime > abs(elapsedTime)) {
            // This allows the RPM to approach 0
            if (elapsedTime < 0){
                elapsedTime = -lastTime;
            }
            else {
                elapsedTime = lastTime;
            }
        };

        _RPM = 0 ;
        if (elapsedTime != 0){
          _RPM = _encoderStepsToRPMScaleFactor / elapsedTime;
        }
    }
    _RPM = _RPM * -1.0;
    
    //Store values for next time
    _lastTimeStamp = currentMicros;
    _lastPosition  = currentPosition;
    
    return _RPM;
}

float MotorGearboxEncoder::cachedSpeed(){
    /*
    Returns the last result of computeSpeed
    */
    return _RPM;
}

void MotorGearboxEncoder::setName(char *newName){
    /*
    Set the name for the object
    */
    _motorName = newName;
}

char MotorGearboxEncoder::name(){
    /*
    Get the name for the object
    */
    return *_motorName;
}
