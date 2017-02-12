//Class for taking the derivative of recent measurements

//Created by Theo Drissner-Devine 11/07/16

#ifndef Derivative_Derivative_h
#define Derivative_Derivative_h

#include <deque>

using namespace std;

class Derivative{

public:
  float computeDerivative(float current);
  void setT(float timestep);
  void setSize(unsigned size);


  Derivative(unsigned size);

protected:
  deque<float> history;
  deque<float>::iterator i;

private:
  unsigned buffer_size;
  float past_value;
  float T; //Timestep
};



#endif
