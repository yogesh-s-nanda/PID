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
 *    Description   :   " Contains the Functions for PID Control "
 *    
 *  @2019 Copyright reserved to Toboids Automata Pvt. (Ltd)
 *  ====================================================================
 */
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_control.h"

/************************************************************************
 *  Header Files
 */
PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255);

    SampleTime = 100;             //default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis() - SampleTime;
}

PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}

/************************************************************************
 *  PID Reset
 *  
 *    Call the Function to Reset the Integral Term.
 */
 void PID::ResetPID(){
  //*myOutput = 0;
  outputSum = 0;
 }
 
/************************************************************************
 *  PID Compute.
 *  
 *    Calls this compute function every time in the loop.
 */
bool PID::Compute(){
  
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      double dInput = (input - lastInput);
      outputSum+= (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
      double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;

      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
   }
   else return false;
}

/************************************************************************
 *  PID Compute.
 *  
 *    Calls this compute function when required.
 */
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/************************************************************************
 *  PID Set Tuning parameters.
 *    Set Kp, Ki, Kd
 */
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/************************************************************************
 *  Set Sample time.
 */
void PID::SetSampleTime(int NewSampleTime)
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

/************************************************************************
 *  Set the Output limit of PID output
 */
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
     if(*myOutput > outMax) *myOutput = outMax;
     else if(*myOutput < outMin) *myOutput = outMin;

     if(outputSum > outMax) outputSum= outMax;
     else if(outputSum < outMin) outputSum= outMin;
   }
}

/************************************************************************
 *  Set Mode 
 *      Automatic or Manual
 */
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/************************************************************************
 *  Initilize
 *      Does all required for a Bumbless transfer
 */
void PID::Initialize()
{
   outputSum = *myOutput;
   lastInput = *myInput;
   if(outputSum > outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

/************************************************************************
 *  Set the control Direction of PID
 */
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/************************************************************************
 *  Status Function to Display.
 */
  double PID::GetKp(){ return  dispKp; }
  double PID::GetKi(){ return  dispKi;}
  double PID::GetKd(){ return  dispKd;}
  
  int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
  int PID::GetDirection(){ return controllerDirection;}
