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
    
    #ifndef Kinematics_h
    #define Kinematics_h


    //Calculation tolerances
    #define DELTAPHI 0.001
    #define DELTAY 0.01
    #define KINEMATICSMAXERROR 0.001
    #define KINEMATICSMAXINVERSE 10
    #define KINEMATICSMAXGUESS 200

    class Kinematics{
        public:
            Kinematics();
            void init  ();
            void  inverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  quadrilateralInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  triangularInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  recomputeGeometry();
            void  forward(const float& chainALength, const float& chainBLength, float* xPos, float* yPos, float xGuess, float yGuess);
            //geometry
            float h; //distance between sled attach point and bit
            float R             = 10.1;                                //sprocket radius

            float halfWidth;                      //Half the machine width
            float halfHeight;                    //Half the machine height
        private:
            float _moment(const float& Y1Plus, const float& Y2Plus, const float& MSinPhi, const float& MSinPsi1, const float& MCosPsi1, const float& MSinPsi2, const float& MCosPsi2);
            float _YOffsetEqn(const float& YPlus, const float& Denominator, const float& Psi);
            void  _MatSolv();
            void  _MyTrig();
            void _verifyValidTarget(float* xTarget,float* yTarget);
            //target router bit coordinates.
            float x = 0;
            float y = 0;
            float _xCordOfMotor;
            float _yCordOfMotor;

            //utility variables
            boolean Mirror;

            //Criterion Computation Variables
            float Phi = -0.2;
            float TanGamma; 
            float TanLambda;
            float Y1Plus ;
            float Y2Plus;
            float Theta;
            float Psi1 = Theta - Phi;
            float Psi2 = Theta + Phi;
            float Jac[9];
            float Solution[3];
            float Crit[3];
            float Offsetx1;
            float Offsetx2;
            float Offsety1;
            float Offsety2;
            float SinPsi1;
            float CosPsi1;
            float SinPsi2;
            float CosPsi2;
            float SinPsi1D;
            float CosPsi1D;
            float SinPsi2D;
            float CosPsi2D;
            float MySinPhi;
            float MySinPhiDelta;

            //intermediate output
            float Lambda;
            float Gamma;

            // Motor axes length to the bit for triangular kinematics
            float Motor1Distance; //left motor axis distance to sled
            float Motor2Distance; //right motor axis distance to sled

            // output = chain lengths measured from 12 o'clock
            float Chain1; //left chain length 
            float Chain2; //right chain length
    };

    #endif
