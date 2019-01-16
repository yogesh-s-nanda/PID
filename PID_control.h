/***********************************************************************
 *  ====================================================================
 *    --------------------------------
 *      PID Control
 *    --------------------------------
 *    
 *    Author    : Jogesh S Nanda
 *    Date      : 02.01.2019
 *    
 *    Revision  : 07.01.2019
 *    
 *    Description   :   " Contains the Header Files for PID_control.c "
 *    
 *  @2019 Copyright reserved to Toboids Automata Pvt. (Ltd)
 *  ====================================================================
 */
 
#ifndef control_h
#define control_h
#define LIBRARY_VERSION  1.2.1

class PID
{

  public:

    //*** [ Constants ] *****************************************************************************
    #define AUTOMATIC 1
    #define MANUAL  0
    #define DIRECT  0
    #define REVERSE  1
    #define P_ON_M 0
    #define P_ON_E 1

    //*** [ PID Main Functions ] *********************************************************************
    PID(double*, double*, double*, double, double, double, int, int);
    PID(double*, double*, double*, double, double, double, int);

    //*** [ PID Functions to used in Setup and Main() ] **********************************************
    void SetMode(int Mode);                  // sets PID to either Manual (0) or Auto (non-0)
    bool Compute();
    void SetOutputLimits(double, double);   //  Limits the Ouptut to Specific Limit

    void ResetPID();
    
    //*** [ Tuning Releated  Fuctions ] **************************************************************
    void SetTunings(double, double, double);
    void SetTunings(double, double, double, int);             
    void SetControllerDirection(int);
    void SetSampleTime(int);                //  Sets the frequency, in Milliseconds                    
                      
    //*** [ Display functions ] **********************************************************************
    double GetKp();             
    double GetKi();            
    double GetKd();   
    
    int GetMode();             
    int GetDirection();    

  private:
  
    void Initialize();
  
    double dispKp;        // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;        //   format for display purposes
    double dispKd;        //
    
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;
    int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             
    double *mySetpoint;           
        
    unsigned long lastTime;
    double outputSum, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;
    
};
#endif
