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
    
    #ifndef Axis_h
    #define Axis_h

    class Axis{
        public:
            void   setup(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const char& axisName, const unsigned long& loopInterval);
            void   write(const float& targetPosition);
            float  read();
            void   set(const float& newAxisPosition);
            void   setSteps(const long& steps);
            int    updatePositionFromEncoder();
            void   initializePID(const unsigned long& loopInterval);
            int    detach();
            int    attach();
            void   detachIfIdle();
            void   endMove(const float& finalTarget);
            void   stop();
            float  target();
            float  error();
            float  setpoint();
            void   computePID();
            void   disablePositionPID();
            void   enablePositionPID();
            void   setPIDAggressiveness(float aggressiveness);
            void   test();
            void   changePitch(float* newPitch);
            float  getPitch();
            void   changeEncoderResolution(float* newResolution);
            bool   attached();
            MotorGearboxEncoder    motorGearboxEncoder;
            void   setPIDValues(float* Kp, float* Ki, float* Kd, float* propWeight, float* KpV, float* KiV, float* KdV, float* propWeightV);
            String     getPIDString();
            double     pidInput();
            double     pidOutput();
            long  steps();
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(const unsigned int& addr, const float& x);
            float      _readFloat(const unsigned int& addr);
            unsigned long   _timeLastMoved;
            volatile double _pidSetpoint;
            volatile double _pidInput; 
            volatile double _pidOutput;
            float      *_Kp, *_Ki, *_Kd;
            PID        _pidController;
            float      *_mmPerRotation;
            float      *_encoderSteps;
            bool       _disableAxisForTesting = false;
            char       _axisName;
    };

    #endif
