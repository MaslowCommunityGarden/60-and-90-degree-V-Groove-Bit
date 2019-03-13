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
The Kinematics module relates the lengths of the chains to the position of the cutting head
in X-Y space.
*/

#include "Maslow.h"


Kinematics::Kinematics(){
     recomputeGeometry();
}

void Kinematics::init(){
    recomputeGeometry();
    if (sys.state != STATE_OLD_SETTINGS){
      forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition, sys.xPosition, sys.yPosition);
    }
}

void Kinematics::_verifyValidTarget(float* xTarget,float* yTarget){
    //If the target point is beyond one of the edges of the board, the machine stops at the edge

    *xTarget = (*xTarget < -halfWidth) ? -halfWidth : (*xTarget > halfWidth) ? halfWidth : *xTarget;
    *yTarget = (*yTarget < -halfHeight) ? -halfHeight : (*yTarget > halfHeight) ? halfHeight : *yTarget;

}

void Kinematics::recomputeGeometry(){
    /*
    Some variables are computed on class creation for the geometry of the machine to reduce overhead,
    calling this function regenerates those values.  These are all floats so they take up
    ~32bytes of RAM to keep them in memory.
    */
    Phi = -0.2;
    h = sqrt((sysSettings.sledWidth/2)*(sysSettings.sledWidth/2) + sysSettings.sledHeight * sysSettings.sledHeight);
    Theta = atan(2*sysSettings.sledHeight/sysSettings.sledWidth);
    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;
  
    halfWidth = sysSettings.machineWidth / 2.0;
    halfHeight = sysSettings.machineHeight / 2.0;
    _xCordOfMotor = sysSettings.distBetweenMotors/2;
    _yCordOfMotor = halfHeight + sysSettings.motorOffsetY;

}

void  Kinematics::inverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    /*
    
    This function works as a switch to call either the quadrilateralInverse kinematic function 
    or the triangularInverse kinematic function
    
    */
    
    if(sysSettings.kinematicsType == 1){
        quadrilateralInverse(xTarget, yTarget, aChainLength, bChainLength);
    }
    else{
        triangularInverse(xTarget, yTarget, aChainLength, bChainLength);
    }
    
}

void  Kinematics::quadrilateralInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){

    //Confirm that the coordinates are on the wood
    _verifyValidTarget(&xTarget, &yTarget);

    //coordinate shift to put (0,0) in the center of the plywood from the left sprocket
    y = (halfHeight) + sysSettings.motorOffsetY  - yTarget;
    x = (sysSettings.distBetweenMotors/2.0) + xTarget;

    //Coordinates definition:
    //         x -->, y |
    //                  v
    // (0,0) at center of left sprocket
    // upper left corner of plywood (270, 270)

    byte Tries = 0;                                  //initialize
    if(x > sysSettings.distBetweenMotors/2.0){                              //the right half of the board mirrors the left half so all computations are done  using left half coordinates.
      x = sysSettings.distBetweenMotors-x;                                  //Chain lengths are swapped at exit if the x,y is on the right half
      Mirror = true;
    }
    else{
        Mirror = false;
    }

    TanGamma = y/x;
    TanLambda = y/(sysSettings.distBetweenMotors-x);
    Y1Plus = R * sqrt(1 + TanGamma * TanGamma);
    Y2Plus = R * sqrt(1 + TanLambda * TanLambda);


    while (Tries <= KINEMATICSMAXINVERSE) {

        _MyTrig();
                                             //These criteria will be zero when the correct values are reached
                                             //They are negated here as a numerical efficiency expedient

        Crit[0]=  - _moment(Y1Plus, Y2Plus, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2);
        Crit[1] = - _YOffsetEqn(Y1Plus, x - h * CosPsi1, SinPsi1);
        Crit[2] = - _YOffsetEqn(Y2Plus, sysSettings.distBetweenMotors - (x + h * CosPsi2), SinPsi2);

        if (abs(Crit[0]) < KINEMATICSMAXERROR) {
            if (abs(Crit[1]) < KINEMATICSMAXERROR) {
                if (abs(Crit[2]) < KINEMATICSMAXERROR){
                    break;
                }
            }
        }

                   //estimate the tilt angle that results in zero net _moment about the pen
                   //and refine the estimate until the error is acceptable or time runs out

                          //Estimate the Jacobian components

        Jac[0] = (_moment( Y1Plus, Y2Plus, MySinPhiDelta, SinPsi1D, CosPsi1D, SinPsi2D, CosPsi2D) + Crit[0])/DELTAPHI;
        Jac[1] = (_moment( Y1Plus + DELTAY, Y2Plus, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2) + Crit[0])/DELTAY;
        Jac[2] = (_moment(Y1Plus, Y2Plus + DELTAY, MySinPhi, SinPsi1, CosPsi1, SinPsi2, CosPsi2) + Crit[0])/DELTAY;
        Jac[3] = (_YOffsetEqn(Y1Plus, x - h * CosPsi1D, SinPsi1D) + Crit[1])/DELTAPHI;
        Jac[4] = (_YOffsetEqn(Y1Plus + DELTAY, x - h * CosPsi1,SinPsi1) + Crit[1])/DELTAY;
        Jac[5] = 0.0;
        Jac[6] = (_YOffsetEqn(Y2Plus, sysSettings.distBetweenMotors - (x + h * CosPsi2D), SinPsi2D) + Crit[2])/DELTAPHI;
        Jac[7] = 0.0;
        Jac[8] = (_YOffsetEqn(Y2Plus + DELTAY, sysSettings.distBetweenMotors - (x + h * CosPsi2D), SinPsi2) + Crit[2])/DELTAY;


        //solve for the next guess
        _MatSolv();     // solves the matrix equation Jx=-Criterion

        // update the variables with the new estimate

        Phi = Phi + Solution[0];
        Y1Plus = Y1Plus + Solution[1];                         //don't allow the anchor points to be inside a sprocket
        Y1Plus = (Y1Plus < R) ? R : Y1Plus;

        Y2Plus = Y2Plus + Solution[2];                         //don't allow the anchor points to be inside a sprocke
        Y2Plus = (Y2Plus < R) ? R : Y2Plus;

        Psi1 = Theta - Phi;
        Psi2 = Theta + Phi;

    Tries++;                                       // increment itteration count

    }

    //Variables are within accuracy limits
    //  perform output computation

    Offsetx1 = h * CosPsi1;
    Offsetx2 = h * CosPsi2;
    Offsety1 = h *  SinPsi1;
    Offsety2 = h * SinPsi2;
    TanGamma = (y - Offsety1 + Y1Plus)/(x - Offsetx1);
    TanLambda = (y - Offsety2 + Y2Plus)/(sysSettings.distBetweenMotors -(x + Offsetx2));
    Gamma = atan(TanGamma);
    Lambda =atan(TanLambda);

    //compute the chain lengths

    if(Mirror){
        Chain2 = sqrt((x - Offsetx1)*(x - Offsetx1) + (y + Y1Plus - Offsety1)*(y + Y1Plus - Offsety1)) - R * TanGamma + R * Gamma;   //right chain length
        Chain1 = sqrt((sysSettings.distBetweenMotors - (x + Offsetx2))*(sysSettings.distBetweenMotors - (x + Offsetx2))+(y + Y2Plus - Offsety2)*(y + Y2Plus - Offsety2)) - R * TanLambda + R * Lambda;   //left chain length
    }
    else{
        Chain1 = sqrt((x - Offsetx1)*(x - Offsetx1) + (y + Y1Plus - Offsety1)*(y + Y1Plus - Offsety1)) - R * TanGamma + R * Gamma;   //left chain length
        Chain2 = sqrt((sysSettings.distBetweenMotors - (x + Offsetx2))*(sysSettings.distBetweenMotors - (x + Offsetx2))+(y + Y2Plus - Offsety2)*(y + Y2Plus - Offsety2)) - R * TanLambda + R * Lambda;   //right chain length
    }

    *aChainLength = Chain1;
    *bChainLength = Chain2;

}

void  Kinematics::triangularInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    /*
    
    The inverse kinematics (relating an xy coordinate pair to the required chain lengths to hit that point)
    function for a triangular set up where the chains meet at a point, or are arranged so that they simulate 
    meeting at a point.
    
    */
    
    //Confirm that the coordinates are on the wood
    _verifyValidTarget(&xTarget, &yTarget);

    //Set up variables
    float Chain1Angle = 0;
    float Chain2Angle = 0;
    float Chain1AroundSprocket = 0;
    float Chain2AroundSprocket = 0;

    //Calculate motor axes length to the bit
    float Motor1Distance = sqrt(pow((-1*_xCordOfMotor - xTarget),2)+pow((_yCordOfMotor - yTarget),2));
    float Motor2Distance = sqrt(pow((_xCordOfMotor - xTarget),2)+pow((_yCordOfMotor - yTarget),2));

    //Calculate the chain angles from horizontal, based on if the chain connects to the sled from the top or bottom of the sprocket
    if(sysSettings.chainOverSprocket == 1){
        Chain1Angle = asin((_yCordOfMotor - yTarget)/Motor1Distance) + asin(R/Motor1Distance);
        Chain2Angle = asin((_yCordOfMotor - yTarget)/Motor2Distance) + asin(R/Motor2Distance);

        Chain1AroundSprocket = R * Chain1Angle;
        Chain2AroundSprocket = R * Chain2Angle;
    }
    else{
        Chain1Angle = asin((_yCordOfMotor - yTarget)/Motor1Distance) - asin(R/Motor1Distance);
        Chain2Angle = asin((_yCordOfMotor - yTarget)/Motor2Distance) - asin(R/Motor2Distance);

        Chain1AroundSprocket = R * (3.14159 - Chain1Angle);
        Chain2AroundSprocket = R * (3.14159 - Chain2Angle);
    }

    //Calculate the straight chain length from the sprocket to the bit
    float Chain1Straight = sqrt(pow(Motor1Distance,2)-pow(R,2));
    float Chain2Straight = sqrt(pow(Motor2Distance,2)-pow(R,2));

    //Correct the straight chain lengths to account for chain sag
    Chain1Straight *= (1 + ((sysSettings.chainSagCorrection / 1000000000000) * pow(cos(Chain1Angle),2) * pow(Chain1Straight,2) * pow((tan(Chain2Angle) * cos(Chain1Angle)) + sin(Chain1Angle),2)));
    Chain2Straight *= (1 + ((sysSettings.chainSagCorrection / 1000000000000) * pow(cos(Chain2Angle),2) * pow(Chain2Straight,2) * pow((tan(Chain1Angle) * cos(Chain2Angle)) + sin(Chain2Angle),2)));

    //Calculate total chain lengths accounting for sprocket geometry and chain sag
    float Chain1 = Chain1AroundSprocket + Chain1Straight * (1.0f + sysSettings.leftChainTolerance / 100.0f);
    float Chain2 = Chain2AroundSprocket + Chain2Straight * (1.0f + sysSettings.rightChainTolerance / 100.0f);

    //Subtract of the virtual length which is added to the chain by the rotation mechanism
    Chain1 = Chain1 - sysSettings.rotationDiskRadius;
    Chain2 = Chain2 - sysSettings.rotationDiskRadius;
    
    *aChainLength = Chain1;
    *bChainLength = Chain2;
}

void  Kinematics::forward(const float& chainALength, const float& chainBLength, float* xPos, float* yPos, float xGuess, float yGuess){
  
    Serial.println(F("[Forward Calculating Position]"));
    

    float guessLengthA;
    float guessLengthB;

    int guessCount = 0;

    while(1){


        //check our guess
        inverse(xGuess, yGuess, &guessLengthA, &guessLengthB);

        float aChainError = chainALength - guessLengthA;
        float bChainError = chainBLength - guessLengthB;


        //adjust the guess based on the result
        xGuess = xGuess + .1*aChainError - .1*bChainError;
        yGuess = yGuess - .1*aChainError - .1*bChainError;
        
        guessCount++;

        #if defined (KINEMATICSDBG) && KINEMATICSDBG > 0 
          Serial.print(F("[PEk:"));
          Serial.print(aChainError);
          Serial.print(',');
          Serial.print(bChainError);
          Serial.print(',');
          Serial.print('0');
          Serial.println(F("]"));
        #endif

        execSystemRealtime();
        // No need for sys.stop check here

        //if we've converged on the point...or it's time to give up, exit the loop
        if((abs(aChainError) < .1 && abs(bChainError) < .1) or guessCount > KINEMATICSMAXGUESS or guessLengthA > sysSettings.chainLength  or guessLengthB > sysSettings.chainLength){
            if((guessCount > KINEMATICSMAXGUESS) or guessLengthA > sysSettings.chainLength or guessLengthB > sysSettings.chainLength){
                Serial.print(F("Message: Unable to find valid machine position for chain lengths "));
                Serial.print(chainALength);
                Serial.print(", ");
                Serial.print(chainBLength);
                Serial.println(F(" . Please set the chains to a known length (Actions -> Set Chain Lengths)"));
                *xPos = 0;
                *yPos = 0;
            }
            else{
                Serial.println("position loaded at:");
                Serial.println(xGuess);
                Serial.println(yGuess);
                *xPos = xGuess;
                *yPos = yGuess;
            }
            break;
        }
    }
}

void  Kinematics::_MatSolv(){
    float Sum;
    int NN;
    int i;
    int ii;
    int J;
    int JJ;
    int K;
    int KK;
    int L;
    int M;
    int N;

    float fact;

    // gaus elimination, no pivot

    N = 3;
    NN = N-1;
    for (i=1;i<=NN;i++){
        J = (N+1-i);
        JJ = (J-1) * N-1;
        L = J-1;
        KK = -1;
        for (K=0;K<L;K++){
            fact = Jac[KK+J]/Jac[JJ+J];
            for (M=1;M<=J;M++){
                Jac[KK + M]= Jac[KK + M] -fact * Jac[JJ+M];
            }
        KK = KK + N;
        Crit[K] = Crit[K] - fact * Crit[J-1];
        }
    }

//Lower triangular matrix solver

    Solution[0] =  Crit[0]/Jac[0];
    ii = N-1;
    for (i=2;i<=N;i++){
        M = i -1;
        Sum = Crit[i-1];
        for (J=1;J<=M;J++){
            Sum = Sum-Jac[ii+J]*Solution[J-1];
        }
    Solution[i-1] = Sum/Jac[ii+i];
    ii = ii + N;
    }
}

float Kinematics::_moment(const float& Y1Plus, const float& Y2Plus, const float& MSinPhi, const float& MSinPsi1, const float& MCosPsi1, const float& MSinPsi2, const float& MCosPsi2){   //computes net moment about center of mass
    float Offsetx1;
    float Offsetx2;
    float Offsety1;
    float Offsety2;
    float TanGamma;
    float TanLambda;

    Offsetx1 = h * MCosPsi1;
    Offsetx2 = h * MCosPsi2;
    Offsety1 = h * MSinPsi1;
    Offsety2 = h * MSinPsi2;
    TanGamma = (y - Offsety1 + Y1Plus)/(x - Offsetx1);
    TanLambda = (y - Offsety2 + Y2Plus)/(sysSettings.distBetweenMotors -(x + Offsetx2));

    return sysSettings.sledCG*MSinPhi + (h/(TanLambda+TanGamma))*(MSinPsi2 - MSinPsi1 + (TanGamma*MCosPsi1 - TanLambda * MCosPsi2));
}

void Kinematics::_MyTrig(){
    float Phisq = Phi * Phi;
    float Phicu = Phi * Phisq;
    float Phidel = Phi + DELTAPHI;
    float Phidelsq = Phidel * Phidel;
    float Phidelcu = Phidel * Phidelsq;
    float Psi1sq = Psi1 * Psi1;
    float Psi1cu = Psi1sq * Psi1;
    float Psi2sq = Psi2 * Psi2;
    float Psi2cu = Psi2 * Psi2sq;
    float Psi1del = Psi1 - DELTAPHI;
    float Psi1delsq = Psi1del * Psi1del;
    float Psi1delcu = Psi1del * Psi1delsq;
    float Psi2del = Psi2 + DELTAPHI;
    float Psi2delsq = Psi2del * Psi2del;
    float Psi2delcu = Psi2del * Psi2delsq;

    // Phirange is 0 to -27 degrees
    // sin -0.1616   -0.0021    1.0002   -0.0000 (error < 6e-6)
    // cos(phi): 0.0388   -0.5117    0.0012    1.0000 (error < 3e-5)
    // Psi1 range is 42 to  69 degrees,
    // sin(Psi1):  -0.0942   -0.1368    1.0965   -0.0241 (error < 2.5 e-5)
    // cos(Psi1):  0.1369   -0.6799    0.1077    0.9756  (error < 1.75e-5)
    // Psi2 range is 15 to 42 degrees
    // sin(Psi2): -0.1460   -0.0197    1.0068   -0.0008 (error < 1.5e-5)
    // cos(Psi2):  0.0792   -0.5559    0.0171    0.9981 (error < 2.5e-5)

    MySinPhi = -0.1616*Phicu - 0.0021*Phisq + 1.0002*Phi;
    MySinPhiDelta = -0.1616*Phidelcu - 0.0021*Phidelsq + 1.0002*Phidel;

    SinPsi1 = -0.0942*Psi1cu - 0.1368*Psi1sq + 1.0965*Psi1 - 0.0241;//sinPsi1
    CosPsi1 = 0.1369*Psi1cu - 0.6799*Psi1sq + 0.1077*Psi1 + 0.9756;//cosPsi1
    SinPsi2 = -0.1460*Psi2cu - 0.0197*Psi2sq + 1.0068*Psi2 - 0.0008;//sinPsi2
    CosPsi2 = 0.0792*Psi2cu - 0.5559*Psi2sq + 0.0171*Psi2 + 0.9981;//cosPsi2

    SinPsi1D = -0.0942*Psi1delcu - 0.1368*Psi1delsq + 1.0965*Psi1del - 0.0241;//sinPsi1
    CosPsi1D = 0.1369*Psi1delcu - 0.6799*Psi1delsq + 0.1077*Psi1del + 0.9756;//cosPsi1
    SinPsi2D = -0.1460*Psi2delcu - 0.0197*Psi2delsq + 1.0068*Psi2del - 0.0008;//sinPsi2
    CosPsi2D = 0.0792*Psi2delcu - 0.5559*Psi2delsq + 0.0171*Psi2del +0.9981;//cosPsi2

}

float Kinematics::_YOffsetEqn(const float& YPlus, const float& Denominator, const float& Psi){
    float Temp;
    Temp = ((sqrt(YPlus * YPlus - R * R)/R) - (y + YPlus - h * sin(Psi))/Denominator);
    return Temp;
}
