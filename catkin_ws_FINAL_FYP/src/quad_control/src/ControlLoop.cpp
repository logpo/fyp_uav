//Control Loop:
//Class for controlling a control loop output

//Created by Theo Drissner-Devine 11/07/16

#include "ControlLoop.h"
#include <iostream>


ControlLoop::ControlLoop(void) //Default Constructor
{
  kp = 1;
  ki = 0;
  kd = 0;
  integral = 0;
  derivative = 0;
  d_filter_length = 0;
  prev_error = 0;
  T = 0.02;
}

ControlLoop::ControlLoop(float KP, float KI, float KD)//Overloaded Constructor for gains
{
  kp = KP;
  ki = KI;
  kd = KD;
  integral = 0;
  derivative = 0;
  d_filter_length = 5;
  prev_error = 0;
  T = 0.02;
}

ControlLoop::ControlLoop(float KP, float KI, float KD, int filterL)//Overloaded Constructor for gains and filter
{
  kp = KP;
  ki = KI;
  kd = KD;
  integral = 0;
  derivative = 0;
  d_filter_length = filterL;
  prev_error = 0;
  T = 0.02;
}

void::ControlLoop::printGains(void)
{
 std::cout << "Kp gain is: "<< kp << std::endl;
 std::cout << "Ki gain is: "<< ki << std::endl; 
 std::cout << "Kd gain is: "<< kd << std::endl; 
 return;
  
}

//Setter functions for K values

void ControlLoop::setKp(float k)
{
  kp = k;
}

void ControlLoop::setKi(float k)
{
  ki = k;
}

void ControlLoop::setKd(float k)
{
  kd = k;
}

void ControlLoop::setFilterL(float n)
{
  d_filter_length = n;
}

float ControlLoop::computeOutput(float setpoint, float measurement)
{
  float output; //control output for the loop
  float difference = setpoint - measurement; //signal difference between sp and measurement
  output = kp * difference;

  //When Integral term active compute integral
  if (ki != 0) {
    integral += (difference + prev_error) * T /2;
    output += (ki * integral);
  }

  //When Defivative term active compute derivative
  if (kd != 0) {
    this->computeDerivative(difference);
    output += (kd * derivative);
  }

  prev_error = difference;
  return output;
}

void ControlLoop::computeDerivative(float error){
//Function to find derivative of error, using previous errors
  float total = 0;
  float d = (error - prev_error) / T;

  d_hist.push_front(d); //Add newest element

  if (d_hist.size()>=d_filter_length) {
    /*If filter is required size, remove an element, otherwise add until right size*/
    d_hist.pop_back();
  }

  //Iterate and add to the total of the past derivative.
   it = d_hist.begin();
   while (it != d_hist.end()) {
     total += *it;
     ++it;
   }

   derivative = total / d_hist.size();

}
