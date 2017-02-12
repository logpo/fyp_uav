//Control Loop:
//Class for controlling a control loop output

//Created by Theo Drissner-Devine 11/07/16

#ifndef ControlLoop_ControlLoop_h
#define ControlLoop_ControlLoop_h
#include <deque>

using namespace std;

class ControlLoop{

public:
  void computeDerivative(float error);
  float computeOutput(float setpoint, float measurement);
  void setKp(float k);
  void setKi(float k);
  void setKd(float k);
  void setFilterL(float n);
  void printGains(void);


  ControlLoop();//Default Constructor
  ControlLoop(float KP, float KI, float KD);//Overloaded Constructor for gains
  ControlLoop(float KP, float KI, float KD, int filterL);//Overloaded Constructor for gains and filter
protected:
  deque<float> d_hist;
  deque<float>::iterator it;


private:
  unsigned d_filter_length;
  float kp;
  float ki;
  float kd;
  float integral;
  float derivative;
  float prev_error;
  float T;// Time step in s assume to be 50Hz





};


#endif
