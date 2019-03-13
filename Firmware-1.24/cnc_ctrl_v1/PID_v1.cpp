/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Maslow.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID()
{
	inAuto = false;
}

void PID::setup(volatile double* Input, volatile double* Output, volatile double* Setpoint,
        float* Kp, float* Ki, float* Kd, float* POn, const int& ControllerDirection)
{

    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
	inAuto = false;

	PID::SetOutputLimits(0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()-SampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   //unsigned long now = millis();
   //unsigned long timeChange = (now - lastTime);
   //if(timeChange>=SampleTime)
   //{  <--- This if statement has been removed to reduce timing jitter on the interrupt.
    //because we are calling the function from within an interrupt timer it will be called
    //with a consistent sample period
  
    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    double dInput = (input - lastInput);
    outputSum+= (ki * error);

    /*Add Proportional on Measurement, if P_ON_M is specified*/
    if(pOnM) outputSum-= pOnMKp * dInput;

    if(outputSum > outMax) outputSum= outMax;
    else if(outputSum < outMin) outputSum= outMin;

    /*Add Proportional on Error, if P_ON_E is specified*/
    double output;
    if(pOnE) output = pOnEKp * error;
    else output = 0;

    /*Compute Rest of PID Output*/
    output += outputSum - kd * dInput;

    if(output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
    *myOutput = output;

    /*Remember some variables for next time*/
    lastInput = input;
    //lastTime = now;
    return true;
   //}
   //else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(float* Kp, float* Ki, float* Kd, float* pOn)
{
   if (*Kp<0 || *Ki<0 || *Kd<0 || *pOn<0 || *pOn>1) return;

   pOnE = *pOn>0; //some p on error is desired;
   pOnM = *pOn<1; //some p on measurement is desired;  

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = *Kp;
   ki = *Ki * SampleTimeInSec;
   kd = *Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   pOnEKp = *pOn * kp; 
   pOnMKp = (1 - *pOn) * kp;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(const int& NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(const double& Min, const double& Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {

   //clamp myOutput and ITerm
   *myOutput =
      (*myOutput > outMax) ? outMax : (*myOutput < outMin) ? outMin : *myOutput;
      
    if(outputSum > outMax) outputSum= outMax;
    else if(outputSum < outMin) outputSum= outMin;

   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(const int& Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(const int& Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  *dispKp;}
double PID::GetKi(){ return  *dispKi;}
double PID::GetKd(){ return  *dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}
double PID::GetIterm(){ return outputSum; }
String PID::pidState() {
    /*
    Returns a comma seperated string of the PID setpoint, input, & output
    useful for debugging
    */
    double input = *myInput;
    double setpoint = *mySetpoint;
    double output = *myOutput;
    String ret = "";
    ret.concat((double)setpoint);
    ret.concat(",");
    ret.concat((double)input);
    ret.concat(",");
    ret.concat((double)output);
    return ret;
}

