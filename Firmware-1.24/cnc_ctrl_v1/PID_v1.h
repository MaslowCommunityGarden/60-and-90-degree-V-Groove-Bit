#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0.0
  #define P_ON_E 1.0
  bool pOnE = true, pOnM = false;
  double pOnEKp, pOnMKp;

  //commonly used functions **************************************************************************
    
    PID();
    
    void setup(volatile double*, volatile double*, volatile double*,        // * constructor.  links the PID to the Input, Output, and 
        float*, float*, float*, float*, const int&);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(const int& Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(const double&, const double&); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(float*, float*,       // * While most users will set the tunings once in the 
                    float*);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(float*, float*,       // * overload for specifying proportional mode
                    float*, float*);         	  

	void SetControllerDirection(const int&);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(const int&);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100


  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //
    double GetIterm();
  String pidState();

  private:
	void Initialize();
	
	float *dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	float *dispKi;				//   format for display purposes
	float *dispKd;				//

// While shared by both main motors, these are not pointers because they are 
// calculated using sample time in this file.
	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    volatile double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    volatile double *myOutput;             //   This creates a hard link between the variables and the 
    volatile double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double outputSum, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto;
};
#endif

